from __future__ import annotations
from uuid import UUID, uuid4
from fastapi import HTTPException

STAGES = ["wall_follow", "frontier_explore", "wall_fill", "return_home"]
VALID = {"validate": {"DRAFT"}, "start": {"READY", "PAUSED"}, "pause": {"RUNNING"}, "resume": {"PAUSED"}, "cancel": {"DRAFT", "READY", "RUNNING", "PAUSED"}}

class MappingService:
    def __init__(self, storage, settings_provider, runner=None) -> None:
        self.storage = storage
        self.settings_provider = settings_provider
        self.runner = runner
        self.on_stop = None
        self.active_id: str | None = None

    def create(self, user, payload: dict) -> dict:
        stages = payload.get("stages", STAGES)
        if not isinstance(stages, list) or not stages or any(s not in STAGES for s in stages):
            raise HTTPException(422, detail="INVALID_VALUE")
        name = str(payload.get("name", "mapping"))
        if not 1 <= len(name) <= 128:
            raise HTTPException(422, detail="INVALID_VALUE")
        return {"mapping_id": str(uuid4()), "stages": stages, "stage_index": 0, "state": "DRAFT", "name": name}

    def action(self, user, mapping_id: str, request_id: UUID, action: str, dispatcher, robot_id: str = "robot_1", lease_id: str = ""):
        if action not in VALID:
            raise HTTPException(422, detail="INVALID_VALUE")
        result = dispatcher.submit(user, request_id, mapping_id, "mapping_" + action, parameters={"mapping_id": mapping_id, "action": action, "robot_id": robot_id, "lease_id": lease_id})
        return result

    async def execute_mapping(self, operation: str, parameters: dict, user=None) -> tuple[bool, dict]:
        action = operation.removeprefix("mapping_")
        mapping_id = str(parameters.get("mapping_id", ""))
        if action not in VALID:
            return False, {"reason_code": "INVALID_VALUE"}
        if self.runner is None:
            return True, {"mapping_id": mapping_id, "mapping_state": action.upper()}
        robot_id = str(parameters.get("robot_id", "robot_1"))
        lease_id = str(parameters.get("lease_id") or "")
        try:
            if action in ("start", "resume"):
                await self.runner.launch(robot_id, lease_id)
            elif action in ("pause", "cancel"):
                await self.runner.cancel()
                if self.on_stop is not None:
                    await self.on_stop(robot_id)
        except ValueError as error:
            return False, {"reason_code": str(error)}
        mapping_state = {"start": "RUNNING", "resume": "RUNNING", "pause": "PAUSED", "cancel": "CANCELLED"}.get(action, "VALIDATED")
        if action == "start":
            self.active_id = mapping_id
        return True, {"mapping_id": mapping_id, "mapping_state": mapping_state}
