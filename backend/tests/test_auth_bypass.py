from __future__ import annotations

from pathlib import Path

from fastapi.testclient import TestClient

from pinky_control_center.main import create_app
from pinky_control_center.models import UserRole

ORIGIN = "http://localhost:5173"


def test_auth_bypass_defaults_off_wrong_password_rejected(tmp_path: Path) -> None:
    app = create_app(database_path=tmp_path / "control.db")
    with TestClient(app) as client:
        app.state.storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
        response = client.post(
            "/api/v1/session",
            json={"username": "operator", "password": "wrong-password"},
            headers={"origin": ORIGIN},
        )
        assert response.status_code == 401
        assert response.json()["error"]["code"] == "AUTH_REQUIRED"


def test_auth_bypass_issues_operator_session_without_password(tmp_path: Path) -> None:
    app = create_app(database_path=tmp_path / "control.db", auth_bypass=True)
    with TestClient(app) as client:
        app.state.storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
        response = client.post(
            "/api/v1/session",
            json={"username": "operator", "password": "wrong-password"},
            headers={"origin": ORIGIN},
        )
        assert response.status_code == 200
        assert response.json()["user"]["username"] == "operator"
        assert "cc_session" in response.headers["set-cookie"]
        assert client.get("/api/v1/session").status_code == 200


def test_auth_bypass_unknown_user_still_rejected(tmp_path: Path) -> None:
    app = create_app(database_path=tmp_path / "control.db", auth_bypass=True)
    with TestClient(app) as client:
        response = client.post(
            "/api/v1/session",
            json={"username": "ghostuser", "password": "wrong-password"},
            headers={"origin": ORIGIN},
        )
        assert response.status_code == 401
