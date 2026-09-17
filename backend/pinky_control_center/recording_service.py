"""Dataset recording for later learning (YOLO/OpenCV).

Taps the live camera frame stream per robot and stores JPEG frames plus a
JSONL manifest (frame_id, timestamps, size) under root/<label>/<robot_id>/.
"""

from __future__ import annotations

import json
from pathlib import Path

from pinky_control_center.camera_service import CameraFrame

_VALID_ROBOTS = frozenset(("robot_1", "robot_2"))


class RecordingService:
    def __init__(self, root: str | Path) -> None:
        self._root = Path(root)
        self._active: dict[str, dict] = {}

    def start(self, robot_id: str, label: str) -> dict:
        if robot_id not in _VALID_ROBOTS:
            raise ValueError("unknown robot")
        safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in label)[:64] or "run"
        run_dir = self._root / safe / robot_id
        run_dir.mkdir(parents=True, exist_ok=True)
        manifest = run_dir / "meta.jsonl"
        if not manifest.exists():
            manifest.touch()
        self._active[robot_id] = {"label": safe, "dir": run_dir, "frames": 0, "last_frame_id": None}
        return {"robot_id": robot_id, "label": safe, "dir": str(run_dir)}

    def record(self, robot_id: str, frame: CameraFrame) -> bool:
        run = self._active.get(robot_id)
        if run is None:
            return False
        if frame.frame_id == run["last_frame_id"]:
            return False
        run["last_frame_id"] = frame.frame_id
        idx = run["frames"]
        (run["dir"] / f"{idx:06d}.jpg").write_bytes(frame.jpeg)
        with open(run["dir"] / "meta.jsonl", "a", encoding="utf-8") as fh:
            fh.write(
                json.dumps(
                    {
                        "frame_id": frame.frame_id,
                        "captured_at": frame.captured_at.isoformat() if frame.captured_at else None,
                        "received_at": frame.received_at.isoformat(),
                        "width": frame.width,
                        "height": frame.height,
                    }
                )
                + "\n"
            )
        run["frames"] += 1
        return True

    def stop(self, robot_id: str) -> dict:
        run = self._active.pop(robot_id)
        return {"robot_id": robot_id, "label": run["label"], "frames": run["frames"], "dir": str(run["dir"])}

    def active(self) -> dict[str, dict]:
        return {rid: {"label": r["label"], "frames": r["frames"]} for rid, r in self._active.items()}
