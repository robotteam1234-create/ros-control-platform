# backend/pinky_control_center/line_follow_service.py
from __future__ import annotations
import time
from collections.abc import Callable
from datetime import UTC, datetime
from pinky_control_center.line_detector import detect_line

LINEAR_MPS = 0.10
KP = 0.6
KD = 0.15
LOSS_LIMIT = 3
MAX_ANGULAR = 0.50
STALE_AFTER_S = 2.0
TICK_PERIOD_S = 0.1
SUPPORTED_ROBOTS = ("robot_1",)


def _frame_age_s(frame) -> float | None:
    ts = getattr(frame, "received_at", None) or getattr(frame, "captured_at", None)
    if ts is None:
        return None
    if isinstance(ts, datetime):
        stamp = ts.timestamp()
        if ts.tzinfo is None:
            stamp = ts.replace(tzinfo=UTC).timestamp()
        return datetime.now(UTC).timestamp() - stamp
    try:
        return time.time() - float(ts)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None


class LineFollowService:
    def __init__(self, adapter, clock: Callable[[], float] = time.monotonic) -> None:
        self.adapter = adapter
        self._clock = clock
        self._active: dict[str, dict] = {}
        self._last_tick: dict[str, float] = {}

    def active_robot_ids(self) -> list[str]:
        return [rid for rid in self._active if rid in SUPPORTED_ROBOTS]

    def tick_due(self, robot_id: str, now: float | None = None) -> bool:
        """Throttle helper: watchdog runs at 20Hz, line-follow at ~10Hz."""
        current = now if now is not None else self._clock()
        last = self._last_tick.get(robot_id)
        if last is None:
            self._last_tick[robot_id] = current
            return True
        if current - last >= TICK_PERIOD_S:
            self._last_tick[robot_id] = current
            return True
        return False

    def take_safety_stop(self, robot_id: str) -> bool:
        """Consume a pending LOST protective-stop exactly once (no stop storm)."""
        st = self._active.get(robot_id)
        if st is None or not st.get("safety_pending"):
            return False
        st["safety_pending"] = False
        return True

    async def start(self, robot_id: str, mode: str = "auto") -> dict:
        if robot_id != "robot_1":
            raise ValueError("ROBOT_NOT_SUPPORTED")
        if mode not in {"auto", "white", "black"}:
            raise ValueError("INVALID_VALUE")
        if self.adapter.frame(robot_id) is None:
            raise RuntimeError("CAMERA_STALLED")
        self._active[robot_id] = {"mode": mode, "misses": 0, "prev": 0.0, "state": "TRACKING", "offset": 0.0, "polarity": "unknown", "safety_pending": False}
        self._last_tick.pop(robot_id, None)
        return self.status(robot_id)

    async def stop(self, robot_id: str) -> dict:
        self._active.pop(robot_id, None)
        self._last_tick.pop(robot_id, None)
        try:
            await self.adapter.publish_manual_velocity(robot_id, 0.0, 0.0)
        except Exception:
            pass
        return {"robot_id": robot_id, "state": "IDLE"}

    def status(self, robot_id: str) -> dict:
        return {"robot_id": robot_id, **self._active.get(robot_id, {"state": "IDLE"})}

    async def tick_once(self, robot_id: str) -> dict:
        st = self._active.get(robot_id)
        if st is None or robot_id not in SUPPORTED_ROBOTS:
            return {"robot_id": robot_id, "state": "IDLE"}
        # Every direct tick counts as a run for the 10 Hz throttle clock.
        try:
            self._last_tick[robot_id] = self._clock()
        except Exception:
            pass
        frame = self.adapter.frame(robot_id)
        stale = frame is not None and (_frame_age_s(frame) or 0.0) > STALE_AFTER_S
        if frame is None or stale:
            st["misses"] += 1
            if st.get("state") != "LOST":
                st["state"] = "STALLED"
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
            if st.get("state") != "LOST":
                st["state"] = "LOST"
                st["safety_pending"] = True
            await self.adapter.publish_manual_velocity(robot_id, 0.0, 0.0)
        return self.status(robot_id)
