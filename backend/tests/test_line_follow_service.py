# backend/tests/test_line_follow_service.py
import asyncio
from pinky_control_center.line_follow_service import LineFollowService
from pinky_control_center.models import CommandAcceptance
from pinky_control_center.line_detector import detect_line
from PIL import Image
import io

def _tape_jpeg(x0=140, line="white"):
    bg = (40, 40, 40) if line == "white" else (220, 220, 220)
    fg = (255, 255, 255) if line == "white" else (0, 0, 0)
    img = Image.new("RGB", (320, 240), bg)
    px = img.load()
    for y in range(120, 240):
        for x in range(x0, x0 + 30):
            px[x, y] = fg
    b = io.BytesIO()
    img.save(b, format="JPEG", quality=90)
    return b.getvalue()

class FakeFrame:
    def __init__(self, jpeg):
        self.jpeg = jpeg

class FakeAdapter:
    def __init__(self, jpeg):
        self._jpeg = jpeg
        self.published: list[tuple] = []
    def frame(self, robot_id):
        return FakeFrame(self._jpeg)
    async def publish_manual_velocity(self, robot_id, lin, ang):
        self.published.append((robot_id, lin, ang))
        return CommandAcceptance(accepted=True)

def test_tick_publishes_bounded_velocity():
    svc = LineFollowService(FakeAdapter(_tape_jpeg(20, "white")))
    asyncio.run(svc.start("robot_1", "auto"))
    out = asyncio.run(svc.tick_once("robot_1"))
    assert out["state"] == "TRACKING"
    _, lin, ang = svc.adapter.published[-1]
    assert abs(lin) <= 0.15 and abs(ang) <= 0.50

def test_three_misses_stop_and_lost():
    img = Image.new("RGB", (320, 240), (40, 40, 40))
    b = io.BytesIO()
    img.save(b, format="JPEG", quality=90)
    svc = LineFollowService(FakeAdapter(b.getvalue()))
    asyncio.run(svc.start("robot_1", "auto"))
    for _ in range(3):
        out = asyncio.run(svc.tick_once("robot_1"))
    assert out["state"] == "LOST"
    assert svc.adapter.published[-1][1:] == (0.0, 0.0)
