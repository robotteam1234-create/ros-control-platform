from __future__ import annotations

import asyncio
import json

import cv2
import numpy as np


def encode_map_frame(meta: dict, png: bytes) -> bytes:
    """4-byte big-endian metadata length + JSON metadata + PNG payload."""
    encoded = json.dumps(meta, separators=(",", ":")).encode("utf-8")
    return len(encoded).to_bytes(4, "big") + encoded + png


def grid_to_png(msg: dict) -> tuple[bytes, dict]:
    """Render a rosbridge OccupancyGrid message to a top-down PNG."""
    info = msg["info"]
    width, height = int(info["width"]), int(info["height"])
    data = np.array(msg["data"], dtype=np.int16).reshape(height, width)
    image = np.full((height, width), 128, np.uint8)
    image[data == 0] = 255
    image[data > 0] = 0
    image = np.flipud(image)
    ok, buffer = cv2.imencode(".png", image)
    if not ok:
        raise ValueError("PNG_ENCODE_FAILED")
    origin = info.get("origin", {}).get("position", {})
    meta = {
        "width": width, "height": height,
        "resolution": float(info["resolution"]),
        "origin": {"x": float(origin.get("x", 0.0)), "y": float(origin.get("y", 0.0))},
    }
    return buffer.tobytes(), meta


class MapStreamService:
    """Holds the latest live SLAM map per robot and fans frames to viewers."""

    def __init__(self) -> None:
        self._latest: dict[str, bytes] = {}
        self._seq = 0
        self._viewers: dict[str, set[asyncio.Queue]] = {}

    def update_grid(self, robot_id: str, msg: dict) -> dict:
        png, meta = grid_to_png(msg)
        self._seq += 1
        meta["seq"] = self._seq
        frame = encode_map_frame(meta, png)
        self._latest[robot_id] = frame
        for queue in self._viewers.get(robot_id, ()):
            if queue.full():
                queue.get_nowait()
            queue.put_nowait(frame)
        return meta

    def latest(self, robot_id: str) -> bytes | None:
        return self._latest.get(robot_id)

    def subscribe(self, robot_id: str) -> asyncio.Queue:
        queue: asyncio.Queue = asyncio.Queue(maxsize=1)
        self._viewers.setdefault(robot_id, set()).add(queue)
        return queue

    def unsubscribe(self, robot_id: str, queue: asyncio.Queue) -> None:
        self._viewers.get(robot_id, set()).discard(queue)
