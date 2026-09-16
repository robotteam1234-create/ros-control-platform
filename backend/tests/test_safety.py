from __future__ import annotations

from pathlib import Path
from uuid import uuid4

from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from pinky_control_center.main import create_app
from pinky_control_center.models import UserRole

ORIGIN = "http://localhost:5173"


def test_operator_can_request_all_stop_without_lease_and_observe_each_target(tmp_path: Path) -> None:
    with TestClient(create_app(database_path=tmp_path / "control.db")) as client:
        client.app.state.storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
        login = client.post("/api/v1/session", json={"username": "operator", "password": "operator-password"}, headers={"origin": ORIGIN})
        response = client.post("/api/v1/stop", json={"request_id": str(uuid4()), "target": "all"}, headers={"origin": ORIGIN, "x-csrf-token": login.json()["csrf_token"]})
        assert response.status_code == 202
        body = response.json()
        assert {item["robot_id"] for item in body["targets"]} == {"robot_1", "robot_2"}
        assert {item["state"] for item in body["targets"]} == {"REQUESTED"}
        status = client.get(f"/api/v1/commands/{body['command_id']}")
        assert status.status_code == 200
        assert status.json()["state"] in {"ACCEPTED", "RUNNING", "SUCCEEDED"}


def test_stop_reset_requires_the_active_control_lease(tmp_path: Path) -> None:
    with TestClient(create_app(database_path=tmp_path / "control.db", start_command_worker=False)) as client:
        client.app.state.storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
        login = client.post("/api/v1/session", json={"username": "operator", "password": "operator-password"}, headers={"origin": ORIGIN})
        headers = {"origin": ORIGIN, "x-csrf-token": login.json()["csrf_token"]}
        denied = client.post("/api/v1/stop/reset", json={"request_id": str(uuid4()), "target": "robot_1"}, headers=headers)
        assert denied.status_code == 409
        lease = client.post("/api/v1/control-lease", json={"request_id": str(uuid4())}, headers=headers).json()
        accepted = client.post("/api/v1/stop/reset", json={"request_id": str(uuid4()), "target": "robot_1", "lease_id": lease["lease_id"]}, headers=headers)
        assert accepted.status_code == 202
        assert accepted.json()["state"] == "ACCEPTED"


def test_teleop_socket_rejects_missing_lease_before_manual_input(tmp_path: Path) -> None:
    with TestClient(create_app(database_path=tmp_path / "control.db")) as client:
        client.app.state.storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
        client.post("/api/v1/session", json={"username": "operator", "password": "operator-password"}, headers={"origin": ORIGIN})
        with client.websocket_connect("/ws/teleop", headers={"origin": ORIGIN}) as socket:
            socket.send_json({"lease_id": str(uuid4()), "robot_id": "robot_1", "seq": 1, "linear_mps": 0.1, "angular_rps": 0.0})
            assert socket.receive_json()["reason_code"] == "CONTROL_CONFLICT"


def test_stop_request_id_is_persisted_and_rejects_changed_payload(tmp_path: Path) -> None:
    with TestClient(create_app(database_path=tmp_path / "control.db")) as client:
        client.app.state.storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
        login = client.post("/api/v1/session", json={"username": "operator", "password": "operator-password"}, headers={"origin": ORIGIN})
        headers = {"origin": ORIGIN, "x-csrf-token": login.json()["csrf_token"]}; request_id = str(uuid4())
        first = client.post("/api/v1/stop", json={"request_id": request_id, "target": "robot_1"}, headers=headers)
        again = client.post("/api/v1/stop", json={"request_id": request_id, "target": "robot_1"}, headers=headers)
        changed = client.post("/api/v1/stop", json={"request_id": request_id, "target": "robot_2"}, headers=headers)
        assert first.json()["command_id"] == again.json()["command_id"]
        assert changed.status_code == 409


def test_stop_confirmation_and_timeout_use_fake_monotonic_clock() -> None:
    from pinky_control_center.safety_service import SafetyService
    now = 0.0; safety = SafetyService(clock=lambda: now)
    safety.request(["robot_1", "robot_2"]); safety.observe("robot_1", stop_latched=True, linear_mps=0, angular_rps=0, fresh=True)
    now = .49; assert safety.states() == {"robot_1": "ACKNOWLEDGED", "robot_2": "REQUESTED"}
    now = .5; assert safety.states()["robot_1"] == "CONFIRMED"
    now = 2.0; assert safety.states()["robot_2"] == "UNCONFIRMED"


def test_odom_disagreement_blocks_stop_confirmation() -> None:
    from pinky_control_center.safety_service import SafetyService
    now = 0.0; safety = SafetyService(clock=lambda: now)
    safety.request(["robot_1"])
    safety.observe("robot_1", stop_latched=True, linear_mps=0, angular_rps=0, fresh=True,
                   odom_linear_mps=0.05, odom_angular_rps=0.0, odom_fresh=True)
    now = 5.0
    assert safety.states() == {"robot_1": "UNCONFIRMED"}
    assert safety.disagreed("robot_1") is True


def test_odom_agreement_confirms_stop() -> None:
    from pinky_control_center.safety_service import SafetyService
    now = 0.0; safety = SafetyService(clock=lambda: now)
    safety.request(["robot_1"])
    safety.observe("robot_1", stop_latched=True, linear_mps=0, angular_rps=0, fresh=True,
                   odom_linear_mps=0.0, odom_angular_rps=0.0, odom_fresh=True)
    now = 0.5
    assert safety.states() == {"robot_1": "CONFIRMED"}
    assert safety.disagreed("robot_1") is False


def test_stale_odom_falls_back_to_status_only() -> None:
    from pinky_control_center.safety_service import SafetyService
    now = 0.0; safety = SafetyService(clock=lambda: now)
    safety.request(["robot_1"])
    safety.observe("robot_1", stop_latched=True, linear_mps=0, angular_rps=0, fresh=True,
                   odom_linear_mps=0.09, odom_angular_rps=0.0, odom_fresh=False)
    now = 0.5
    assert safety.states() == {"robot_1": "CONFIRMED"}


def test_stop_route_executes_each_mock_target(tmp_path: Path) -> None:
    with TestClient(create_app(database_path=tmp_path / "control.db", start_command_worker=False)) as client:
        client.app.state.storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
        login = client.post("/api/v1/session", json={"username":"operator","password":"operator-password"}, headers={"origin":ORIGIN})
        calls=[]; original=client.app.state.adapter.execute
        async def execute(command): calls.append(command.robot_id); return await original(command)
        client.app.state.adapter.execute=execute
        response=client.post("/api/v1/stop",json={"request_id":str(uuid4()),"target":"all"},headers={"origin":ORIGIN,"x-csrf-token":login.json()["csrf_token"]})
        assert response.status_code==202
        import asyncio
        asyncio.run(client.app.state.command_dispatcher.process_next())
        assert calls==["robot_1","robot_2"]


def test_stop_refresh_persists_mixed_confirmed_and_unconfirmed(tmp_path: Path) -> None:
    now=0.; app=create_app(database_path=tmp_path/"control.db",monotonic_clock=lambda:now, start_command_worker=False)
    with TestClient(app) as client:
        client.app.state.storage.create_or_reset_user("operator","operator-password",UserRole.OPERATOR); login=client.post("/api/v1/session",json={"username":"operator","password":"operator-password"},headers={"origin":ORIGIN}); h={"origin":ORIGIN,"x-csrf-token":login.json()["csrf_token"]}
        command=client.post("/api/v1/stop",json={"request_id":str(uuid4()),"target":"all"},headers=h).json(); source=app.state.state_store.snapshot(); master=source.robots[0].model_copy(update={"stop_latched":True,"linear_mps":0.,"angular_rps":0.})
        app.state.state_store.snapshot=lambda: source.model_copy(update={"robots":[master,source.robots[1]]})
        import asyncio
        asyncio.run(app.state.command_dispatcher.process_next()); asyncio.run(app.state.runtime_tick()); now=.5; asyncio.run(app.state.runtime_tick()); now=2.; asyncio.run(app.state.runtime_tick())
        states={x["robot_id"]:x["state"] for x in client.get(f"/api/v1/commands/{command['command_id']}").json()["targets"]}; assert states=={"robot_1":"CONFIRMED","robot_2":"UNCONFIRMED"}
