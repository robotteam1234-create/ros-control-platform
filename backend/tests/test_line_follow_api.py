import asyncio
from pathlib import Path
from uuid import uuid4

from fastapi.testclient import TestClient

from pinky_control_center.main import create_app
from pinky_control_center.models import MockScenario, UserRole

ORIGIN = "http://localhost:5173"


def headers(client):
    client.app.state.storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
    login = client.post("/api/v1/session", json={"username": "operator", "password": "operator-password"}, headers={"origin": ORIGIN})
    csrf = login.json()["csrf_token"]
    lease = client.post("/api/v1/control-lease", json={"request_id": str(uuid4())}, headers={"origin": ORIGIN, "x-csrf-token": csrf}).json()["lease_id"]
    return {"origin": ORIGIN, "x-csrf-token": csrf, "x-control-lease-id": lease}


def _action(h, action="start", mode="auto"):
    return {"request_id": str(uuid4()), "action": action, "mode": mode}


def test_line_follow_status_idle(tmp_path: Path):
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    with TestClient(app) as client:
        h = headers(client)
        r = client.get("/api/v1/robots/robot_1/line-follow", headers=h)
        assert r.status_code == 200
        assert r.json() == {"robot_id": "robot_1", "state": "IDLE"}


def test_line_follow_status_rejects_unknown_robot(tmp_path: Path):
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    with TestClient(app) as client:
        h = headers(client)
        r = client.get("/api/v1/robots/robot_2/line-follow", headers=h)
        assert r.status_code == 404
        assert r.json()["error"]["code"] == "ROBOT_NOT_FOUND"


def test_line_follow_start_rejects_unknown_robot(tmp_path: Path):
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    with TestClient(app) as client:
        h = headers(client)
        r = client.post("/api/v1/robots/robot_2/line-follow/actions", json=_action(h), headers=h)
        assert r.status_code == 404
        assert r.json()["error"]["code"] == "ROBOT_NOT_FOUND"


def test_line_follow_start_rejects_invalid_action(tmp_path: Path):
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    with TestClient(app) as client:
        h = headers(client)
        r = client.post("/api/v1/robots/robot_1/line-follow/actions", json=_action(h, action="fly"), headers=h)
        assert r.status_code == 422
        assert r.json()["error"]["code"] == "INVALID_VALUE"


def test_line_follow_start_rejects_invalid_mode(tmp_path: Path):
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    with TestClient(app) as client:
        h = headers(client)
        r = client.post("/api/v1/robots/robot_1/line-follow/actions", json=_action(h, mode="red"), headers=h)
        assert r.status_code == 422
        assert r.json()["error"]["code"] == "INVALID_VALUE"


def test_line_follow_start_rejects_missing_action(tmp_path: Path):
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    with TestClient(app) as client:
        h = headers(client)
        r = client.post("/api/v1/robots/robot_1/line-follow/actions", json={"request_id": str(uuid4()), "mode": "auto"}, headers=h)
        assert r.status_code == 422
        assert r.json()["error"]["code"] == "INVALID_VALUE"


def test_line_follow_actions_require_lease(tmp_path: Path):
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    with TestClient(app) as client:
        h = headers(client)
        no_lease = {k: v for k, v in h.items() if k != "x-control-lease-id"}
        r = client.post("/api/v1/robots/robot_1/line-follow/actions", json=_action(h), headers=no_lease)
        assert r.status_code == 409


def test_line_follow_start_stop_roundtrip(tmp_path: Path):
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    with TestClient(app) as client:
        h = headers(client)
        started = client.post("/api/v1/robots/robot_1/line-follow/actions", json=_action(h), headers=h)
        assert started.status_code == 202
        asyncio.run(app.state.command_dispatcher.process_next())
        command = client.get(f"/api/v1/commands/{started.json()['command_id']}", headers=h).json()
        assert command["state"] == "SUCCEEDED"
        assert command["line_follow"]["state"] == "TRACKING"
        assert client.get("/api/v1/robots/robot_1/line-follow", headers=h).json()["state"] == "TRACKING"
        stopped = client.post("/api/v1/robots/robot_1/line-follow/actions", json=_action(h, action="stop"), headers=h)
        assert stopped.status_code == 202
        asyncio.run(app.state.command_dispatcher.process_next())
        assert client.get("/api/v1/robots/robot_1/line-follow", headers=h).json()["state"] == "IDLE"


def test_line_follow_start_maps_camera_stalled_to_reason_code(tmp_path: Path):
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    with TestClient(app) as client:
        h = headers(client)
        app.state.adapter.frame = lambda robot_id: None
        started = client.post("/api/v1/robots/robot_1/line-follow/actions", json=_action(h), headers=h)
        assert started.status_code == 202
        asyncio.run(app.state.command_dispatcher.process_next())
        command = client.get(f"/api/v1/commands/{started.json()['command_id']}", headers=h).json()
        assert command["state"] == "FAILED"
        assert command["reason_code"] == "CAMERA_STALLED"


def test_line_follow_start_rejects_stalled_robot_2_scenario(tmp_path: Path):
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    with TestClient(app) as client:
        h = headers(client)
        app.state.adapter.set_scenario(MockScenario.CAMERA_STALL)
        assert client.get("/api/v1/robots/robot_2/line-follow", headers=h).status_code == 404
