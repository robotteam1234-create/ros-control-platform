# backend/pinky_control_center/line_follow_service.py
from __future__ import annotations
import time
from collections.abc import Callable
from pinky_control_center.line_detector import detect_line

LINEAR_MPS = 0.10
KP = 0.6
KD = 0.15
LOSS_LIMIT = 3
MAX_ANGULAR = 0.50

class LineFollowService:
    def __init__(self, adapter, clock: Callable[[], float] = time.monotonic) -> None:
        self.adapter = adapter
        self._clock = clock
        self._active: dict[str, dict] = {}

    async def start(self, robot_id: str, mode: str = "auto") -> dict:
        if robot_id != "robot_1":
            raise ValueError("ROBOT_NOT_SUPPORTED")
        if mode not in {"auto", "white", "black"}:
            raise ValueError("INVALID_VALUE")
        if self.adapter.frame(robot_id) is None:
            raise RuntimeError("CAMERA_STALLED")
        self._active[robot_id] = {"mode": mode, "misses": 0, "prev": 0.0, "state": "TRACKING", "offset": 0.0, "polarity": "unknown"}
        return self.status(robot_id)

    async def stop(self, robot_id: str) -> dict:
        self._active.pop(robot_id, None)
        try:
            await self.adapter.publish_manual_velocity(robot_id, 0.0, 0.0)
        except Exception:
            pass
        return {"robot_id": robot_id, "state": "IDLE"}

    def status(self, robot_id: str) -> dict:
        return {"robot_id": robot_id, **self._active.get(robot_id, {"state": "IDLE"})}

    async def tick_once(self, robot_id: str) -> dict:
        st = self._active.get(robot_id)
        if st is None:
            return {"robot_id": robot_id, "state": "IDLE"}
        frame = self.adapter.frame(robot_id)
        if frame is None:
            st["misses"] += 1
        else:
            res = detect_line(frame.jpeg, st["mode"])
            st["polarity"] = res.polarity
            if not res.found:
                st["misses"] += 1
            else:
                st["misses"] = 0
                st["offset"] = res.offset
                ang = -(KP * res.offset + KD * (res.offset - st["prev"]))
                st["prev"] = res.offset
                ang = max(-MAX_ANGULAR, min(MAX_ANGULAR, ang))
                await self.adapter.publish_manual_velocity(robot_id, LINEAR_MPS, ang)
                st["state"] = "TRACKING"
                return self.status(robot_id)
        if st["misses"] >= LOSS_LIMIT:
            st["state"] = "LOST"
            await self.adapter.publish_manual_velocity(robot_id, 0.0, 0.0)
        return self.status(robot_id)
