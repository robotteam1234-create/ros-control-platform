import asyncio
import io
import json

import numpy as np
from PIL import Image

from pinky_control_center.map_stream_service import MapStreamService, encode_map_frame


def _grid():
    return {
        "info": {
            "width": 2, "height": 2, "resolution": 0.05,
            "origin": {"position": {"x": -1.0, "y": -2.0, "z": 0.0}},
        },
        "data": [0, 100, -1, -1],
    }


def _pixels(png: bytes) -> np.ndarray:
    return np.array(Image.open(io.BytesIO(png)).convert("L"))


def test_encode_map_frame_framing():
    frame = encode_map_frame({"seq": 1}, b"PNGDATA")
    meta_len = int.from_bytes(frame[:4], "big")
    assert json.loads(frame[4:4 + meta_len]) == {"seq": 1}
    assert frame[4 + meta_len:] == b"PNGDATA"


def test_update_grid_renders_tri_state():
    svc = MapStreamService()
    meta = svc.update_grid("robot_1", _grid())
    assert meta["seq"] == 1 and meta["width"] == 2 and meta["height"] == 2
    assert meta["resolution"] == 0.05 and meta["origin"]["x"] == -1.0
    frame = svc.latest("robot_1")
    assert frame is not None
    meta_len = int.from_bytes(frame[:4], "big")
    pixels = _pixels(frame[4 + meta_len:])
    # data row 0 is bottom-left; PNG row 0 is top
    assert pixels.tolist() == [[128, 128], [255, 0]]


def test_latest_none_before_first_grid():
    assert MapStreamService().latest("robot_1") is None


def test_subscribers_receive_latest():
    svc = MapStreamService()
    q1 = svc.subscribe("robot_1")
    q2 = svc.subscribe("robot_1")
    svc.update_grid("robot_1", _grid())
    assert q1.get_nowait() == svc.latest("robot_1")
    assert q2.get_nowait() == svc.latest("robot_1")
    svc.unsubscribe("robot_1", q1)
    assert q1 not in svc._viewers["robot_1"]
    assert q2 in svc._viewers["robot_1"]


def test_seq_increments():
    svc = MapStreamService()
    assert svc.update_grid("robot_1", _grid())["seq"] == 1
    assert svc.update_grid("robot_1", _grid())["seq"] == 2
