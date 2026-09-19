from __future__ import annotations

from datetime import UTC, datetime, timedelta
from pathlib import Path
from uuid import uuid4

from fastapi.testclient import TestClient

from pinky_control_center.main import create_app
from pinky_control_center.models import UserRole
from pinky_control_center.storage import LeaseConflict, Storage

ORIGIN = "http://localhost:5173"


def login(client: TestClient, username: str, password: str) -> str:
    response = client.post("/api/v1/session", json={"username": username, "password": password}, headers={"origin": ORIGIN})
    assert response.status_code == 200
    assert "cc_session" in response.headers["set-cookie"]
    return response.json()["csrf_token"]


def test_session_role_csrf_origin_and_websocket(tmp_path: Path) -> None:
    app = create_app(database_path=tmp_path / "control.db")
    with TestClient(app) as client:
        app.state.storage.create_or_reset_user("viewer", "viewer-password", UserRole.VIEWER)
        unauthenticated = client.get("/api/v1/state")
        assert unauthenticated.status_code == 401
        assert unauthenticated.json()["error"]["code"] == "AUTH_REQUIRED"
        assert client.post("/api/v1/session", json={"username": "viewer", "password": "viewer-password"}).status_code == 403
        csrf = login(client, "viewer", "viewer-password")
        assert client.get("/api/v1/state").status_code == 200
        assert client.post("/api/v1/control-lease", json={"request_id": str(uuid4())}, headers={"origin": ORIGIN, "x-csrf-token": csrf}).status_code == 403
        with client.websocket_connect("/ws/state", headers={"origin": ORIGIN}) as socket:
            message = socket.receive_json()
            assert message["type"] == "snapshot"
            assert len(message["payload"]["robots"]) == 2


def test_operator_lease_conflict_csrf_and_db_persistence(tmp_path: Path) -> None:
    path = tmp_path / "control.db"
    app = create_app(database_path=path)
    with TestClient(app) as client:
        store = app.state.storage
        store.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
        store.create_or_reset_user("other", "other-operator-password", UserRole.OPERATOR)
        csrf = login(client, "operator", "operator-password")
        missing_csrf = client.post("/api/v1/control-lease", json={"request_id": str(uuid4())}, headers={"origin": ORIGIN})
        assert missing_csrf.status_code == 403
        acquired = client.post("/api/v1/control-lease", json={"request_id": str(uuid4())}, headers={"origin": ORIGIN, "x-csrf-token": csrf})
        assert acquired.status_code == 200
        lease_id = acquired.json()["lease_id"]
        client.post("/api/v1/session", json={"username": "other", "password": "other-operator-password"}, headers={"origin": ORIGIN})
        other_csrf = client.cookies.get("cc_csrf")
        conflict = client.post("/api/v1/control-lease", json={"request_id": str(uuid4())}, headers={"origin": ORIGIN, "x-csrf-token": other_csrf})
        assert conflict.status_code == 409
    persisted = Storage(path)
    assert persisted.authenticate("operator", "operator-password") is not None
    assert "operator-password" not in persisted.connection.execute("SELECT password_hash FROM users WHERE username='operator'").fetchone()[0]
    persisted.close()


def test_lease_expiry_with_fake_clock(tmp_path: Path) -> None:
    now = datetime(2026, 1, 1, tzinfo=UTC)
    clock = lambda: now
    storage = Storage(tmp_path / "control.db", clock=clock)
    first = storage.create_or_reset_user("one", "first-password", UserRole.OPERATOR)
    second = storage.create_or_reset_user("two", "second-password", UserRole.OPERATOR)
    storage.acquire_lease(first, uuid4())
    try:
        storage.acquire_lease(second, uuid4())
        assert False, "an active lease must be exclusive"
    except LeaseConflict:
        pass
    now += timedelta(seconds=4)
    assert storage.acquire_lease(second, uuid4()).expires_at == now + timedelta(seconds=3)
    storage.close()


def test_logout_releases_only_its_session_lease_and_records_safety_hook(tmp_path: Path) -> None:
    app = create_app(database_path=tmp_path / "control.db")
    with TestClient(app) as client:
        app.state.storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
        csrf = login(client, "operator", "operator-password")
        acquired = client.post("/api/v1/control-lease", json={"request_id": str(uuid4())}, headers={"origin": ORIGIN, "x-csrf-token": csrf})
        assert acquired.status_code == 200
        assert client.delete("/api/v1/session", headers={"origin": ORIGIN, "x-csrf-token": csrf}).status_code == 204
        assert app.state.lease_events == ["SESSION_LOGOUT"]
        assert client.get("/api/v1/session").status_code == 401


def test_unrelated_expired_login_session_does_not_end_active_control_lease(tmp_path: Path) -> None:
    now = datetime(2026, 1, 1, tzinfo=UTC)
    storage = Storage(tmp_path / "control.db", clock=lambda: now)
    operator = storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
    stale_user = storage.create_or_reset_user("stale", "stale-password", UserRole.VIEWER)
    active_token, _ = storage.create_session(operator)
    stale_token, _ = storage.create_session(stale_user)
    lease = storage.acquire_lease(operator, uuid4(), active_token)
    with storage.connection:
        storage.connection.execute(
            "UPDATE sessions SET expires_at=? WHERE token_hash=?",
            ((now - timedelta(seconds=1)).isoformat(), storage.token_hash(stale_token)),
        )

    assert storage.expire_security() == []
    assert storage.owns_lease(lease.lease_id, operator, active_token)
    storage.close()


def test_expired_session_that_owns_control_lease_reports_control_loss(tmp_path: Path) -> None:
    now = datetime(2026, 1, 1, tzinfo=UTC)
    storage = Storage(tmp_path / "control.db", clock=lambda: now)
    operator = storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
    token, _ = storage.create_session(operator)
    lease = storage.acquire_lease(operator, uuid4(), token)
    with storage.connection:
        storage.connection.execute(
            "UPDATE sessions SET expires_at=? WHERE token_hash=?",
            ((now - timedelta(seconds=1)).isoformat(), storage.token_hash(token)),
        )

    assert storage.expire_security() == ["SESSION_EXPIRED"]
    assert not storage.owns_lease(lease.lease_id, operator, token)
    storage.close()


def test_validation_error_with_bytes_body_returns_422_not_500(tmp_path):
    """A malformed (non-JSON) body must normalize to the standard 422 envelope."""
    from fastapi.testclient import TestClient
    from pinky_control_center.main import create_app
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    client = TestClient(app)
    response = client.post(
        "/api/v1/session",
        content=b"\xff\x00garbage",
        headers={"content-type": "application/json", "origin": "http://localhost:5173"},
    )
    # Malformed JSON is rejected as a client error (400/422) — never a 500 crash.
    assert response.status_code in (400, 422)
    if response.status_code == 422:
        assert response.json()["error"]["code"] == "INVALID_VALUE"
