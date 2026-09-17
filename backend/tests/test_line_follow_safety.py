"""Stale-camera and LOST safety for LineFollowService + runtime_tick."""
import asyncio
import io
from datetime import UTC, datetime, timedelta
from pathlib import Path

from PIL import Image

from pinky_control_center.main import create_app
from pinky_control_center.line_follow_service import LineFollowService
from pinky_control_center.models import CommandAcceptance


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
    def __init__(self, jpeg, received_at=None):
        self.jpeg = jpeg
        self.received_at = received_at
        self.captured_at = received_at


class FakeAdapter:
    def __init__(self, frame):
        self._frame = frame
        self.published: list[tuple] = []

    def frame(self, robot_id):
        return self._frame

    async def publish_manual_velocity(self, robot_id, lin, ang):
        self.published.append((robot_id, lin, ang))
        return CommandAcceptance(accepted=True)


def test_stale_frame_is_miss_not_trusted():
    stale_at = datetime.now(UTC) - timedelta(seconds=5)
    adapter = FakeAdapter(FakeFrame(_tape_jpeg(20, "white"), received_at=stale_at))
    svc = LineFollowService(adapter)
    asyncio.run(svc.start("robot_1", "auto"))
    out = asyncio.run(svc.tick_once("robot_1"))
    # Tape pixels present but timestamp stale -> must not publish drive velocity.
    assert out["state"] == "STALLED"
    assert not adapter.published


def test_lost_transition_flags_safety_stop_once():
    img = Image.new("RGB", (320, 240), (40, 40, 40))
    b = io.BytesIO()
    img.save(b, format="JPEG", quality=90)
    adapter = FakeAdapter(FakeFrame(b.getvalue(), received_at=datetime.now(UTC)))
    svc = LineFollowService(adapter)
    asyncio.run(svc.start("robot_1", "auto"))
    for _ in range(3):
        out = asyncio.run(svc.tick_once("robot_1"))
    assert out["state"] == "LOST"
    assert adapter.published[-1][1:] == (0.0, 0.0)
    assert svc.take_safety_stop("robot_1") is True
    assert svc.take_safety_stop("robot_1") is False  # no stop storm
    # No auto-resume: further ticks stay LOST without drive velocity.
    out2 = asyncio.run(svc.tick_once("robot_1"))
    assert out2["state"] == "LOST"
    assert adapter.published[-1][1:] == (0.0, 0.0)


def test_runtime_tick_routes_lost_through_protective_stop(tmp_path: Path):
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    svc = app.state.line_follow_service
    asyncio.run(svc.start("robot_1", "auto"))
    # Force empty frames so ticks accumulate misses -> LOST.
    img = Image.new("RGB", (320, 240), (40, 40, 40))
    b = io.BytesIO()
    img.save(b, format="JPEG", quality=90)

    class EmptyFrame:
        jpeg = b.getvalue()
        received_at = datetime.now(UTC)
        captured_at = received_at

    app.state.adapter.frame = lambda robot_id: EmptyFrame()  # type: ignore[assignment]
    svc.adapter = app.state.adapter
    # runtime_tick closure captured the original protective_stop; patch via
    # direct service-level assertion plus history instead. Drive ticks manually
    # until LOST, then verify safety flag + zero publish.
    for _ in range(6):
        svc._last_tick.clear()
        asyncio.run(app.state.runtime_tick())
    assert svc.status("robot_1")["state"] == "LOST"
    rows = app.state.storage.audit_connection.execute(
        "SELECT event_type FROM history_events WHERE event_type='LINE_FOLLOW_LOST'"
    ).fetchall()
    assert rows, "expected LINE_FOLLOW_LOST history record"
    safety = app.state.storage.audit_connection.execute(
        "SELECT event_type FROM history_events WHERE event_type='SAFETY_STOP'"
    ).fetchall()
    assert safety, "expected SAFETY_STOP history via protective-stop path"
