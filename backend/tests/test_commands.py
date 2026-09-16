from __future__ import annotations

import pytest
import asyncio
from datetime import UTC, datetime, timedelta
from threading import Barrier, Thread
from pathlib import Path
from uuid import uuid4

from fastapi.testclient import TestClient

from pinky_control_center.main import create_app
from pinky_control_center.models import CommandAcceptance, MockScenario, UserRole

ORIGIN = "http://localhost:5173"


def test_general_queue_caps_at_100_but_stop_bypasses_it() -> None:
    from pinky_control_center.command_service import CommandQueue, QueueFull
    queue = CommandQueue()
    for index in range(100): queue.submit(f"normal-{index}")
    with pytest.raises(QueueFull): queue.submit("overflow")
    queue.submit("stop", priority=True)
    assert queue.pop() == "stop"


def test_teleop_watchdogs_and_individual_stop_protect_both_robots() -> None:
    from pinky_control_center.teleop_service import TeleopService
    now = 0.0; service = TeleopService(clock=lambda: now)
    service.enter("robot_1", lease_valid=True); assert service.ingest("robot_1", 1, .2, 0) == "ACCEPTED"
    now = .31; assert service.tick()["robot_1"] == "ZERO"
    service.enter("robot_1", lease_valid=True); now = 1.32; assert service.tick()["robot_1"] == "ZERO"
    assert service.protective_stop("robot_1") == {"robot_1": "STOPPED", "robot_2": "STOPPED"}


def test_set_mode_rejected_fast_when_stop_latched(tmp_path: Path) -> None:
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    with TestClient(app) as client:
        app.state.storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
        login = client.post("/api/v1/session", json={"username": "operator", "password": "operator-password"}, headers={"origin": ORIGIN})
        headers = {"origin": ORIGIN, "x-csrf-token": login.json()["csrf_token"]}
        source = app.state.state_store.snapshot()
        latched = source.robots[0].model_copy(update={"stop_latched": True})
        app.state.state_store.snapshot = lambda: source.model_copy(update={"robots": [latched, source.robots[1]]})
        denied = client.post("/api/v1/robots/robot_1/mode", json={"request_id": str(uuid4()), "mode": "MANUAL"}, headers=headers)
        assert denied.status_code == 409
        assert denied.json()["error"]["code"] == "STOP_LATCHED"
        allowed = client.post("/api/v1/robots/robot_1/mode", json={"request_id": str(uuid4()), "mode": "STOPPED"}, headers=headers)
        assert allowed.status_code == 202


def test_routes_cap_normal_work_and_execute_priority_stop_first(tmp_path: Path) -> None:
    """The dispatcher is paused so route acceptance, capacity, and priority are observable."""
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    with TestClient(app) as client:
        app.state.storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
        login = client.post("/api/v1/session", json={"username": "operator", "password": "operator-password"}, headers={"origin": ORIGIN})
        headers = {"origin": ORIGIN, "x-csrf-token": login.json()["csrf_token"]}
        for _ in range(100):
            accepted = client.post("/api/v1/robots/robot_1/mode", json={"request_id": str(uuid4()), "mode": "MANUAL"}, headers=headers)
            assert accepted.status_code == 202
        full = client.post("/api/v1/robots/robot_1/mode", json={"request_id": str(uuid4()), "mode": "MANUAL"}, headers=headers)
        assert full.status_code == 503
        assert full.json()["error"]["code"] == "QUEUE_FULL"
        stopped = client.post("/api/v1/stop", json={"request_id": str(uuid4()), "target": "robot_2"}, headers=headers)
        assert stopped.status_code == 202
        asyncio.run(app.state.command_dispatcher.process_next())
        status = client.get(f"/api/v1/commands/{stopped.json()['command_id']}")
        assert status.json()["state"] == "RUNNING"
        assert status.json()["targets"] == [{"robot_id": "robot_2", "state": "ACKNOWLEDGED"}]


def test_runtime_watchdog_zeros_expired_teleop_without_latching_stop(tmp_path: Path) -> None:
    now = 0.0
    app = create_app(database_path=tmp_path / "control.db", monotonic_clock=lambda: now, start_command_worker=False)
    with TestClient(app):
        calls: list[tuple[str, float, float]] = []

        async def publish_manual_velocity(robot_id: str, linear_mps: float, angular_rps: float) -> CommandAcceptance:
            calls.append((robot_id, linear_mps, angular_rps))
            return CommandAcceptance(accepted=True)

        app.state.adapter.publish_manual_velocity = publish_manual_velocity
        app.state.teleop_service.enter("robot_2", lease_valid=True)
        assert app.state.teleop_service.ingest("robot_2", 1, .1, 0) == "ACCEPTED"
        now = .31
        asyncio.run(app.state.runtime_tick())
        assert calls == [("robot_2", 0.0, 0.0)]
        assert app.state.state_store.snapshot().robots[1].connection.value == "ONLINE"


def test_stop_stays_running_until_observation_confirms_then_succeeds(tmp_path: Path) -> None:
    now = 0.0
    app = create_app(database_path=tmp_path / "control.db", monotonic_clock=lambda: now, start_command_worker=False)
    with TestClient(app) as client:
        app.state.storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
        login = client.post("/api/v1/session", json={"username": "operator", "password": "operator-password"}, headers={"origin": ORIGIN})
        headers = {"origin": ORIGIN, "x-csrf-token": login.json()["csrf_token"]}
        created = client.post("/api/v1/stop", json={"request_id": str(uuid4()), "target": "all"}, headers=headers).json()
        asyncio.run(app.state.command_dispatcher.process_next())
        assert client.get(f"/api/v1/commands/{created['command_id']}").json()["state"] == "RUNNING"
        source = app.state.state_store.snapshot()
        stopped = [robot.model_copy(update={"stop_latched": True, "linear_mps": 0.0, "angular_rps": 0.0}) for robot in source.robots]
        app.state.state_store.snapshot = lambda: source.model_copy(update={"robots": stopped})
        asyncio.run(app.state.runtime_tick())
        now = .5
        asyncio.run(app.state.runtime_tick())
        status = client.get(f"/api/v1/commands/{created['command_id']}").json()
        assert {target["state"] for target in status["targets"]} == {"CONFIRMED"}
        assert status["state"] == "SUCCEEDED"


def test_adapter_exception_marks_failure_and_worker_processes_the_next_command(tmp_path: Path) -> None:
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    with TestClient(app) as client:
        app.state.storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
        login = client.post("/api/v1/session", json={"username": "operator", "password": "operator-password"}, headers={"origin": ORIGIN})
        headers = {"origin": ORIGIN, "x-csrf-token": login.json()["csrf_token"]}
        first = client.post("/api/v1/robots/robot_1/mode", json={"request_id": str(uuid4()), "mode": "MANUAL"}, headers=headers).json()
        second = client.post("/api/v1/robots/robot_2/mode", json={"request_id": str(uuid4()), "mode": "MANUAL"}, headers=headers).json()
        original = app.state.adapter.execute
        calls = 0

        async def execute(command):
            nonlocal calls
            calls += 1
            if calls == 1:
                raise RuntimeError("adapter unavailable")
            return await original(command)

        app.state.adapter.execute = execute
        asyncio.run(app.state.command_dispatcher.process_next())
        asyncio.run(app.state.command_dispatcher.process_next())
        assert client.get(f"/api/v1/commands/{first['command_id']}").json()["state"] == "FAILED"
        assert client.get(f"/api/v1/commands/{second['command_id']}").json()["state"] == "SUCCEEDED"


def test_rejected_stop_is_failed_with_an_unconfirmed_target(tmp_path: Path) -> None:
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    with TestClient(app) as client:
        app.state.storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
        login = client.post("/api/v1/session", json={"username": "operator", "password": "operator-password"}, headers={"origin": ORIGIN})
        headers = {"origin": ORIGIN, "x-csrf-token": login.json()["csrf_token"]}
        app.state.adapter.set_scenario(MockScenario.COMMAND_REJECTED)
        created = client.post("/api/v1/stop", json={"request_id": str(uuid4()), "target": "robot_1"}, headers=headers).json()
        asyncio.run(app.state.command_dispatcher.process_next())
        status = client.get(f"/api/v1/commands/{created['command_id']}").json()
        assert status["state"] == "FAILED"
        assert status["targets"] == [{"robot_id": "robot_1", "state": "UNCONFIRMED"}]


def test_request_id_is_idempotent_for_24_hours_but_keeps_expired_audit_history(tmp_path: Path) -> None:
    now = datetime(2026, 1, 1, tzinfo=UTC)
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False, storage_clock=lambda: now)
    with TestClient(app) as client:
        app.state.storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
        login = client.post("/api/v1/session", json={"username": "operator", "password": "operator-password"}, headers={"origin": ORIGIN})
        headers = {"origin": ORIGIN, "x-csrf-token": login.json()["csrf_token"]}
        request_id = str(uuid4())
        first = client.post("/api/v1/robots/robot_1/mode", json={"request_id": request_id, "mode": "MANUAL"}, headers=headers)
        same = client.post("/api/v1/robots/robot_1/mode", json={"request_id": request_id, "mode": "MANUAL"}, headers=headers)
        changed = client.post("/api/v1/robots/robot_1/mode", json={"request_id": request_id, "mode": "IDLE"}, headers=headers)
        assert same.json()["command_id"] == first.json()["command_id"]
        assert changed.status_code == 409
        now += timedelta(hours=24)
        login = client.post("/api/v1/session", json={"username": "operator", "password": "operator-password"}, headers={"origin": ORIGIN})
        headers = {"origin": ORIGIN, "x-csrf-token": login.json()["csrf_token"]}
        renewed = client.post("/api/v1/robots/robot_1/mode", json={"request_id": request_id, "mode": "IDLE"}, headers=headers)
        assert renewed.status_code == 202
        assert renewed.json()["command_id"] != first.json()["command_id"]
        assert app.state.storage.connection.execute("SELECT count(*) FROM commands").fetchone()[0] == 2


def test_concurrent_request_claims_are_atomic_for_same_and_changed_payload(tmp_path: Path) -> None:
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    storage = app.state.storage
    user = storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)

    def claim(request_id, mode, barrier, results):
        barrier.wait()
        try:
            results.append(("ok", storage.create_command(user, request_id, "robot_1", {"operation": "set_mode", "target": "robot_1", "parameters": {"mode": mode}})["command_id"]))
        except ValueError:
            results.append(("conflict", None))

    same_id, barrier, results = uuid4(), Barrier(2), []
    threads = [Thread(target=claim, args=(same_id, "MANUAL", barrier, results)) for _ in range(2)]
    for thread in threads: thread.start()
    for thread in threads: thread.join()
    assert [kind for kind, _ in results] == ["ok", "ok"]
    assert len({command_id for _, command_id in results}) == 1
    assert storage.connection.execute("SELECT count(*) FROM commands WHERE request_id=?", (str(same_id),)).fetchone()[0] == 1

    changed_id, barrier, results = uuid4(), Barrier(2), []
    threads = [Thread(target=claim, args=(changed_id, mode, barrier, results)) for mode in ("MANUAL", "IDLE")]
    for thread in threads: thread.start()
    for thread in threads: thread.join()
    assert sorted(kind for kind, _ in results) == ["conflict", "ok"]
    assert storage.connection.execute("SELECT count(*) FROM commands WHERE request_id=?", (str(changed_id),)).fetchone()[0] == 1


def test_concurrent_expired_request_creates_one_new_generation_and_retains_audit(tmp_path: Path) -> None:
    now = datetime(2026, 1, 1, tzinfo=UTC)
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False, storage_clock=lambda: now)
    storage = app.state.storage
    user = storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
    request_id = uuid4()
    original = storage.create_command(user, request_id, "robot_1", {"operation": "set_mode", "target": "robot_1", "parameters": {"mode": "MANUAL"}})
    now += timedelta(hours=24)
    barrier, results = Barrier(2), []

    def claim() -> None:
        barrier.wait()
        results.append(storage.create_command(user, request_id, "robot_1", {"operation": "set_mode", "target": "robot_1", "parameters": {"mode": "IDLE"}})["command_id"])

    threads = [Thread(target=claim) for _ in range(2)]
    for thread in threads: thread.start()
    for thread in threads: thread.join()
    assert len(set(results)) == 1
    assert results[0] != original["command_id"]
    assert storage.connection.execute("SELECT count(*) FROM commands WHERE request_id=?", (str(request_id),)).fetchone()[0] == 2
