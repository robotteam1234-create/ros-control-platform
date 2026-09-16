from __future__ import annotations

import time
from collections.abc import Callable


class SafetyService:
    """Mock stop-observation state machine; it never claims physical braking."""
    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self.clock = clock; self.requested: dict[str, float] = {}; self.stable_since: dict[str, float] = {}; self.odom_disagree: set[str] = set()

    def request(self, robot_ids: list[str]) -> None:
        now = self.clock()
        for robot_id in robot_ids: self.requested[robot_id] = now

    def observe(self, robot_id: str, *, stop_latched: bool, linear_mps: float | None, angular_rps: float | None, fresh: bool, odom_linear_mps: float | None = None, odom_angular_rps: float | None = None, odom_fresh: bool = False) -> None:
        if robot_id not in self.requested: return
        status_stopped = fresh and stop_latched and linear_mps is not None and angular_rps is not None and abs(linear_mps) < .01 and abs(angular_rps) < .02
        if odom_fresh and status_stopped and ((odom_linear_mps is not None and abs(odom_linear_mps) >= .01) or (odom_angular_rps is not None and abs(odom_angular_rps) >= .02)):
            self.odom_disagree.add(robot_id)
            self.stable_since.pop(robot_id, None)
            return
        self.odom_disagree.discard(robot_id)
        stopped = status_stopped
        if stopped: self.stable_since.setdefault(robot_id, self.clock())
        else: self.stable_since.pop(robot_id, None)

    def disagreed(self, robot_id: str) -> bool:
        return robot_id in self.odom_disagree

    def states(self) -> dict[str, str]:
        now = self.clock(); result = {}
        for robot_id, requested_at in self.requested.items():
            stable_at = self.stable_since.get(robot_id)
            result[robot_id] = "CONFIRMED" if stable_at is not None and now - stable_at >= .5 else "UNCONFIRMED" if now - requested_at >= 2 else "ACKNOWLEDGED" if stable_at is not None else "REQUESTED"
        return result
