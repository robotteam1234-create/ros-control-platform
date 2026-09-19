from __future__ import annotations

import asyncio
from collections import deque
from dataclasses import dataclass
from uuid import UUID, uuid4

from pinky_control_center.models import CommandRequest, UserInfo
from pinky_control_center.storage import Storage


class QueueFull(Exception):
    pass


@dataclass(frozen=True)
class QueuedCommand:
    command_id: str
    operation: str
    target: str
    parameters: dict[str, object] | None = None


class CommandQueue:
    """A bounded normal lane and an unbounded safety lane."""
    def __init__(self) -> None:
        self.normal: deque[QueuedCommand] = deque()
        self.priority: deque[QueuedCommand] = deque()

    def submit(self, value: QueuedCommand | str, *, priority: bool = False) -> None:
        if priority:
            self.priority.append(value)  # type: ignore[arg-type]
            return
        if len(self.normal) >= 100:
            raise QueueFull()
        self.normal.append(value)  # type: ignore[arg-type]

    def pop(self):
        return self.priority.popleft() if self.priority else self.normal.popleft()

    def __bool__(self) -> bool:
        return bool(self.priority or self.normal)


class CommandDispatcher:
    """The sole production path from an accepted command to adapter.execute."""
    def __init__(self, storage: Storage, adapter) -> None:
        self.storage = storage
        self.adapter = adapter
        self.queue = CommandQueue()
        self._wake = asyncio.Event()
        self._task: asyncio.Task[None] | None = None
        self._queued_ids: set[str] = set()
        self._active_stops: dict[str, list[str]] = {}
        self.handlers: dict[str, object] = {}

    def submit(self, user: UserInfo, request_id: UUID, target: str, operation: str, *, priority: bool = False, parameters: dict[str, object] | None = None) -> dict[str, object]:
        command = self.storage.create_command(user, request_id, target, {"operation": operation, "target": target, "parameters": parameters or {}})
        command_id = str(command["command_id"])
        if command_id not in self._queued_ids and command.get("state") == "ACCEPTED":
            try:
                self.queue.submit(QueuedCommand(command_id, operation, target, parameters), priority=priority)
            except QueueFull:
                self.storage.delete_command(command_id)
                raise
            self._queued_ids.add(command_id)
            self._wake.set()
        return command

    async def protective_stop(self, robot_id: str) -> None:
        """Safety path bypasses normal work and records the adapter result in memory/state."""
        command_id = uuid4()
        for target in ("robot_1", "robot_2"):
            try:
                result = await self.adapter.execute(CommandRequest(command_id=command_id, robot_id=target, operation="stop", parameters={"reason": "WATCHDOG", "source_robot": robot_id}))
                self.storage.record_history_safe(event_type="SAFETY_STOP", robot_id=target, payload={"source_robot": robot_id, "accepted": result.accepted, "outcome": "ACKNOWLEDGED" if result.accepted else "REJECTED"}, dedupe_key=f"safety-stop:{command_id}:{target}")
            except Exception:
                # A watchdog must complete its remaining safety work after one adapter fails.
                # Never persist an exception message: adapters can contain credentials or transport details.
                self.storage.record_history_safe(event_type="SAFETY_STOP", robot_id=target, payload={"source_robot": robot_id, "accepted": False, "outcome": "EXCEPTION", "failure_code": "ADAPTER_EXCEPTION"}, dedupe_key=f"safety-stop:{command_id}:{target}")
                continue

    def refresh_stops(self, observations: dict[str, str]) -> None:
        """Stop completion comes from safety observation, never adapter acceptance alone."""
        for command_id, targets in list(self._active_stops.items()):
            states = [{"robot_id": robot_id, "state": observations.get(robot_id, "REQUESTED")} for robot_id in targets]
            self.storage.set_command_result(command_id, {"targets": states})
            values = {item["state"] for item in states}
            if values == {"CONFIRMED"}:
                self.storage.set_command_state(command_id, "SUCCEEDED")
                # A watchdog tick and a test/client shutdown can observe the
                # same completed stop concurrently.  Completion is idempotent.
                self._active_stops.pop(command_id, None)
            elif values.issubset({"CONFIRMED", "UNCONFIRMED"}) and "UNCONFIRMED" in values:
                self.storage.set_command_state(command_id, "TIMED_OUT")
                self._active_stops.pop(command_id, None)

    async def start(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self._worker())

    async def close(self) -> None:
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None

    async def _worker(self) -> None:
        while True:
            await self._wake.wait()
            self._wake.clear()
            while self.queue:
                await self.process_next()

    async def process_next(self) -> None:
        item = self.queue.pop()
        if not isinstance(item, QueuedCommand):
            return
        self._queued_ids.discard(item.command_id)
        self.storage.set_command_state(item.command_id, "RUNNING")
        handler = self.handlers.get(item.operation)
        if handler is not None:
            try:
                accepted, result = await handler(item.operation, item.parameters or {})
                self.storage.set_command_result(item.command_id, result)
                self.storage.set_command_state(item.command_id, "SUCCEEDED" if accepted else "FAILED")
            except Exception as error:
                print(f"command {item.command_id} handler error: {error!r}", flush=True)
                self.storage.set_command_result(item.command_id, {"reason_code": "EXECUTION_FAILED"})
                self.storage.set_command_state(item.command_id, "FAILED")
            return
        targets = ["robot_1", "robot_2"] if item.target == "all" else [item.target]
        result_targets: list[dict[str, str]] = []
        rejected = False
        for robot_id in targets:
            try:
                accepted = await self.adapter.execute(CommandRequest(command_id=UUID(item.command_id), robot_id=robot_id, operation=item.operation, parameters=item.parameters or {}))
                if accepted.accepted:
                    result_targets.append({"robot_id": robot_id, "state": "ACKNOWLEDGED"})
                else:
                    rejected = True
                    result_targets.append({"robot_id": robot_id, "state": "UNCONFIRMED"})
            except Exception:
                rejected = True
                result_targets.append({"robot_id": robot_id, "state": "UNCONFIRMED"})
        self.storage.set_command_result(item.command_id, {"targets": result_targets})
        if item.operation == "stop" and not rejected:
            self._active_stops[item.command_id] = targets
            return
        self.storage.set_command_state(item.command_id, "FAILED" if rejected else "SUCCEEDED")


CommandService = CommandDispatcher
