"""Runtime wiring: runtime_tick must drive LineFollowService.tick_once at ~10Hz."""
import asyncio
from pathlib import Path

from pinky_control_center.main import create_app


def test_runtime_tick_drives_line_follow_publish(tmp_path: Path):
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    svc = app.state.line_follow_service
    asyncio.run(svc.start("robot_1", "auto"))
    published: list[tuple] = []
    orig = app.state.adapter.publish_manual_velocity

    async def spy(robot_id, lin, ang):
        published.append((robot_id, lin, ang))
        return await orig(robot_id, lin, ang)

    app.state.adapter.publish_manual_velocity = spy  # type: ignore[method-assign]
    svc.adapter = app.state.adapter
    asyncio.run(app.state.runtime_tick())
    assert published, "runtime_tick did not drive line-follow publish"
    rid, lin, ang = published[-1]
    assert rid == "robot_1"
    assert abs(lin) <= 0.15 and abs(ang) <= 0.50


def test_runtime_tick_throttles_to_10hz(tmp_path: Path):
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    svc = app.state.line_follow_service
    asyncio.run(svc.start("robot_1", "auto"))
    count = 0
    orig = app.state.adapter.publish_manual_velocity

    async def spy(robot_id, lin, ang):
        nonlocal count
        count += 1
        return await orig(robot_id, lin, ang)

    app.state.adapter.publish_manual_velocity = spy  # type: ignore[method-assign]
    svc.adapter = app.state.adapter
    asyncio.run(app.state.runtime_tick())
    asyncio.run(app.state.runtime_tick())
    # watchdog runs at 20Hz; line-follow must run at ~10Hz (every other pass)
    assert count <= 1, f"expected throttled single tick, got {count}"
