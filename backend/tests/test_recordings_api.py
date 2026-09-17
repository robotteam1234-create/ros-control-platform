"""RED: recordings HTTP API (start/stop/status) + dataset helpers."""

from uuid import uuid4


def _client(tmp_path):
    from fastapi.testclient import TestClient

    from pinky_control_center.main import create_app
    from pinky_control_center.models import UserRole

    origin = "http://localhost:5173"
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    with TestClient(app) as client:
        app.state.storage.create_or_reset_user("operator", "operator-password", UserRole.OPERATOR)
        login = client.post(
            "/api/v1/session",
            json={"username": "operator", "password": "operator-password"},
            headers={"origin": origin},
        )
        assert login.status_code == 200
        csrf = login.json()["csrf_token"]
        lease_id = client.post(
            "/api/v1/control-lease",
            json={"request_id": str(uuid4())},
            headers={"origin": origin, "x-csrf-token": csrf},
        ).json()["lease_id"]
        headers = {"origin": origin, "x-csrf-token": csrf}
        yield client, headers, lease_id


def test_recordings_lifecycle(tmp_path):
    for client, headers, lease_id in _client(tmp_path):
        r = client.post(
            "/api/v1/recordings/start",
            json={"request_id": str(uuid4()), "lease_id": lease_id, "robot_id": "robot_2", "label": "t1"},
            headers=headers,
        )
        assert r.status_code == 201, r.text
        r = client.get("/api/v1/recordings", headers=headers)
        assert r.status_code == 200
        assert r.json()["active"].get("robot_2", {}).get("label") == "t1"
        r = client.post(
            "/api/v1/recordings/stop",
            json={"request_id": str(uuid4()), "lease_id": lease_id, "robot_id": "robot_2"},
            headers=headers,
        )
        assert r.status_code == 200, r.text
        assert r.json()["frames"] == 0


def test_recordings_status_needs_only_cookies(tmp_path):
    from fastapi.testclient import TestClient

    from pinky_control_center.main import create_app
    from pinky_control_center.models import UserRole

    origin = "http://localhost:5173"
    app = create_app(database_path=tmp_path / "control.db", start_command_worker=False)
    with TestClient(app) as client:
        app.state.storage.create_or_reset_user("viewer", "viewer-password", UserRole.OPERATOR)
        login = client.post(
            "/api/v1/session",
            json={"username": "viewer", "password": "viewer-password"},
            headers={"origin": origin},
        )
        assert login.status_code == 200
        r = client.get("/api/v1/recordings")
        assert r.status_code == 200, r.text
        assert "active" in r.json()


def test_dataset_helpers(tmp_path):
    import json

    from pinky_control_center.dataset import read_manifest, split_rows, write_yolo_label

    run = tmp_path / "run" / "robot_2"
    run.mkdir(parents=True)
    rows = [
        {"frame_id": f"f{i}", "captured_at": None, "received_at": "2026-01-01T00:00:00+00:00", "width": 640, "height": 480}
        for i in range(4)
    ]
    (run / "meta.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    for i in range(4):
        (run / f"{i:06d}.jpg").write_bytes(b"\xff\xd8x")
    got = read_manifest(run)
    assert len(got) == 4
    train, val = split_rows(got, val_ratio=0.25)
    assert (len(train), len(val)) == (3, 1)
    lp = write_yolo_label(run, "000000", cls=0, cx=0.5, cy=0.5, w=0.2, h=0.2)
    assert lp.exists()
