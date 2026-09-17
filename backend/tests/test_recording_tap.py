"""RED: runtime_tick taps live camera frames into active recording sessions."""

import asyncio
from pathlib import Path


def _app(tmp_path):
    from fastapi.testclient import TestClient

    from pinky_control_center.main import create_app
    from pinky_control_center.models import UserRole

    app = create_app(mode="mock", database_path=tmp_path / "control.db", start_command_worker=False)
    client = TestClient(app)
    with client:
        app.state.storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
        yield app, client


def test_runtime_tick_records_only_new_frames(tmp_path):
    from pinky_control_center.models import MockScenario

    for app, _client in _app(tmp_path):
        svc = app.state.recording_service
        run = svc.start("robot_1", label="tap")
        run_dir = Path(run["dir"])
        asyncio.run(app.state.runtime_tick())
        asyncio.run(app.state.runtime_tick())
        assert len(list(run_dir.glob("*.jpg"))) == 1, "same frame must not duplicate"
        app.state.adapter.set_scenario(MockScenario.COMMAND_REJECTED)
        asyncio.run(app.state.runtime_tick())
        assert len(list(run_dir.glob("*.jpg"))) == 2, "new frame_id must be recorded"
        assert svc.stop("robot_1")["frames"] == 2
