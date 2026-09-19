# backend/tests/test_mapping_api.py
import asyncio
import json
from io import BytesIO
from uuid import uuid4

from PIL import Image
from fastapi.testclient import TestClient

from pinky_control_center.main import create_app
from pinky_control_center.models import UserRole

ORIGIN = "http://localhost:5173"


class FakeRunner:
    def __init__(self) -> None:
        self._session: dict = {"state": "IDLE", "stage_index": -1, "stage": "", "message": "", "reason": ""}

    def status(self) -> dict:
        return dict(self._session)

    async def launch(self, robot_id: str, lease_id: str | None = None) -> dict:
        if robot_id != "robot_1":
            raise ValueError("ROBOT_NOT_SUPPORTED")
        self._session["state"] = "RUNNING"
        return self.status()

    async def cancel(self) -> dict:
        self._session["state"] = "PAUSED"
        return self.status()

    def log_tail(self, limit: int = 100) -> list[str]:
        return ["line-1", "line-2"]


def _client(tmp_path):
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    fake = FakeRunner()
    app.state.mapping_runner = fake
    app.state.mapping_service.runner = fake
    client = TestClient(app)
    with client:
        app.state.storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
        login = client.post("/api/v1/session", json={"username": "operator", "password": "operator-password"}, headers={"origin": ORIGIN})
        assert login.status_code == 200
        csrf = login.json()["csrf_token"]
        lease_id = client.post("/api/v1/control-lease", json={"request_id": str(uuid4())}, headers={"origin": ORIGIN, "x-csrf-token": csrf}).json()["lease_id"]
        yield client, app, {"origin": ORIGIN, "x-csrf-token": csrf, "x-control-lease-id": lease_id}


def _create_mapping(client, headers, robot_id="robot_1"):
    response = client.post("/api/v1/mappings", json={"request_id": str(uuid4()), "name": "auto", "robot_id": robot_id, "map_id": "mock_lab"}, headers=headers)
    assert response.status_code == 201
    return response.json()["mapping_id"]


def _dispatch(app):
    asyncio.run(app.state.command_dispatcher.process_next())


def test_start_pause_status_flow(tmp_path):
    for client, app, headers in _client(tmp_path):
        mapping_id = _create_mapping(client, headers)
        r = client.post(f"/api/v1/mappings/{mapping_id}/actions", json={"request_id": str(uuid4()), "action": "start", "robot_id": "robot_1", "lease_id": headers["x-control-lease-id"]}, headers=headers)
        assert r.status_code == 202
        command_id = r.json()["command_id"]
        _dispatch(app)
        detail = client.get(f"/api/v1/commands/{command_id}").json()
        assert detail["state"] in {"SUCCEEDED", "ACCEPTED"}
        status = client.get(f"/api/v1/mappings/{mapping_id}", headers=headers).json()
        assert status["state"] == "RUNNING"
        r = client.post(f"/api/v1/mappings/{mapping_id}/actions", json={"request_id": str(uuid4()), "action": "pause", "lease_id": headers["x-control-lease-id"]}, headers=headers)
        assert r.status_code == 202
        _dispatch(app)
        assert client.get(f"/api/v1/mappings/{mapping_id}", headers=headers).json()["state"] == "PAUSED"


def test_start_rejects_unsupported_robot(tmp_path):
    for client, app, headers in _client(tmp_path):
        mapping_id = _create_mapping(client, headers)
        r = client.post(f"/api/v1/mappings/{mapping_id}/actions", json={"request_id": str(uuid4()), "action": "start", "robot_id": "robot_2", "lease_id": headers["x-control-lease-id"]}, headers=headers)
        assert r.status_code == 202
        _dispatch(app)
        command_id = r.json()["command_id"]
        detail = client.get(f"/api/v1/commands/{command_id}").json()
        assert detail["reason_code"] == "ROBOT_NOT_SUPPORTED"


def test_log_endpoint(tmp_path):
    for client, app, headers in _client(tmp_path):
        mapping_id = _create_mapping(client, headers)
        r = client.get(f"/api/v1/mappings/{mapping_id}/log?tail=10", headers=headers)
        assert r.status_code == 200
        assert r.json()["lines"] == ["line-1", "line-2"]


def test_import_registers_dynamic_map(tmp_path, monkeypatch):
    lap = tmp_path / "lap585"
    lap.mkdir()
    Image.new("L", (4, 4), 254).save(lap / "map_real.pgm", format="PPM")
    (lap / "map_real.yaml").write_text(
        "image: map_real.pgm\nmap_id: map_real\nname: real\nframe_id: map\nresolution: 0.05\nwidth: 4\nheight: 4\norigin:\n  x: 0.0\n  y: 0.0\n  yaw: 0.0\nversion: \"1\"\n",
        encoding="utf-8")
    monkeypatch.setenv("PINKY_MAPPING_LAP585_DIR", str(lap))
    for client, app, headers in _client(tmp_path):
        mapping_id = _create_mapping(client, headers)
        r = client.post(f"/api/v1/mappings/{mapping_id}/import", json={"request_id": str(uuid4())}, headers=headers)
        assert r.status_code == 201
        imported = r.json()["imported_map_id"]
        assert imported.startswith("map_auto_")
        maps = client.get("/api/v1/maps", headers=headers).json()["items"]
        assert imported in [item["map_id"] for item in maps]


def test_mapping_ws_streams_latest_frame(tmp_path):
    for client, app, headers in _client(tmp_path):
        grid = {"info": {"width": 1, "height": 1, "resolution": 0.05, "origin": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}}}, "data": [0]}
        app.state.map_stream_service.update_grid("robot_1", grid)
        with client.websocket_connect("/ws/mapping/robot_1", headers={"origin": ORIGIN}) as ws:
            frame = ws.receive_bytes()
        meta_len = int.from_bytes(frame[:4], "big")
        meta = json.loads(frame[4:4 + meta_len])
        assert meta["width"] == 1 and frame[4 + meta_len:].startswith(b"\x89PNG")
