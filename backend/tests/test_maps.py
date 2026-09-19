from __future__ import annotations

from pathlib import Path
from io import BytesIO

from fastapi.testclient import TestClient
from PIL import Image

from pinky_control_center.map_service import MapService
from pinky_control_center.adapters.mock import MockRobotAdapter
from pinky_control_center.main import create_app
from pinky_control_center.models import UserRole
from pinky_control_center.state_store import StateStore

ORIGIN = "http://localhost:5173"


def authenticated_client(tmp_path: Path):
    app = create_app(database_path=tmp_path / "control.db")
    client = TestClient(app)
    app.state.storage.create_or_reset_user("map-viewer", "map-viewer-password", UserRole.VIEWER)
    login = client.post("/api/v1/session", json={"username": "map-viewer", "password": "map-viewer-password"}, headers={"origin": ORIGIN})
    assert login.status_code == 200
    return client


def test_map_metadata_png_etag_and_authentication(tmp_path: Path) -> None:
    with authenticated_client(tmp_path) as client:
        listing = client.get("/api/v1/maps")
        assert listing.status_code == 200
        assert listing.json()["items"] == [
            {"map_id": "map_260905", "name": "260905 실습 트랙", "version": "1"},
            {"map_id": "mock_lab", "name": "Mock Lab", "version": "1"},
            {"map_id": "mock_lab_b", "name": "Mock Lab B (alternate occupancy)", "version": "1"},
        ]
        primary = client.get("/api/v1/maps/mock_lab/data")
        alternate = client.get("/api/v1/maps/mock_lab_b/data")
        assert primary.status_code == alternate.status_code == 200
        assert primary.content != alternate.content
        metadata = client.get("/api/v1/maps/mock_lab")
        assert metadata.status_code == 200
        assert metadata.json()["frame_id"] == "map"
        assert metadata.json()["origin"] == {"x": 0.0, "y": 0.0, "yaw": 0.0}
        png = client.get("/api/v1/maps/mock_lab/data", params={"version": "1"})
        assert png.status_code == 200
        assert png.headers["content-type"] == "image/png"
        assert png.content.startswith(b"\x89PNG")
        cells = Image.open(BytesIO(png.content)).convert("L")
        assert cells.size == (20, 20)
        assert cells.getpixel((5, 5)) == 254
        assert cells.getpixel((8, 5)) == 0
        assert cells.getpixel((15, 13)) == 205
        etag = png.headers["etag"]
        cached = client.get("/api/v1/maps/mock_lab/data", headers={"if-none-match": etag})
        assert cached.status_code == 304
        assert client.get("/api/v1/maps/missing").status_code == 404


def test_world_map_has_world_dimensions_and_occupancy_walls(tmp_path: Path) -> None:
    with authenticated_client(tmp_path) as client:
        metadata = client.get("/api/v1/maps/map_260905").json()
        assert metadata["frame_id"] == "map"
        assert metadata["resolution"] == 0.005
        assert metadata["width"] == 542
        assert metadata["height"] == 252
        assert metadata["origin"] == {"x": -1.355, "y": -0.63, "yaw": 0.0}
        image = Image.open(BytesIO(client.get("/api/v1/maps/map_260905/data").content)).convert("L")
        assert image.size == (542, 252)
        assert image.getpixel((0, 126)) == 0
        assert image.getpixel((271, 0)) == 0
        assert image.getpixel((271, 126)) == 254


def test_map_api_rejects_unauthenticated_requests(tmp_path: Path) -> None:
    app = create_app(database_path=tmp_path / "control.db")
    with TestClient(app) as client:
        assert client.get("/api/v1/maps").status_code == 401


def test_mock_map_pose_geometry_is_bounded_and_tf_invalid_clears_formation_measurements() -> None:
    source = MockRobotAdapter().snapshot()
    master = source.robots[0]
    assert source.map_id == "mock_lab"
    assert master.goal is not None and master.goal.frame_id == "map"
    assert 1 <= len(master.trail) <= 200
    assert 1 <= len(master.path) <= 200
    invalid = master.model_copy(update={"tf_valid": False, "tf_reason_code": "TF_UNAVAILABLE"})
    snapshot = source.model_copy(update={"robots": [invalid, source.robots[1]]})
    state = StateStore(lambda: snapshot).snapshot()
    assert state.formation.distance_m is None
    assert state.formation.gap_error_m is None
    assert state.formation.bearing_rad is None


def _write_occupancy(directory, map_id: str, value_at: dict[tuple[int, int], int]) -> None:
    image = Image.new("L", (4, 4), 254)
    pixels = image.load()
    for (x, y), value in value_at.items():
        pixels[x, y] = value
    image.save(directory / f"{map_id}.pgm", format="PPM")
    (directory / f"{map_id}.yaml").write_text(
        f"image: {map_id}.pgm\nmap_id: {map_id}\nname: {map_id} test\nframe_id: map\nresolution: 0.05\n"
        f"width: 4\nheight: 4\norigin:\n  x: 0.0\n  y: 0.0\n  yaw: 0.0\nversion: \"1\"\n",
        encoding="utf-8")


def test_dynamic_map_loaded_from_extra_dir(tmp_path):
    _write_occupancy(tmp_path, "map_auto_one", {(2, 2): 0})
    service = MapService(extra_dir=tmp_path)
    ids = [item.map_id for item in service.summaries()]
    assert "map_auto_one" in ids
    payload, etag = service.png("map_auto_one")
    assert payload.startswith(b"\x89PNG") and "map_auto_one:1" in etag
    # occupied pixel (2,2) with origin 0,0 res 0.05 -> world (0.1, 0.1) is NOT free;
    # pixel (0,0) row=height-1-row_from_bottom conventions exercised via is_free
    assert service.is_free("map_auto_one", 0.05, 0.05) is True
    assert service.is_free("map_auto_one", 0.11, 0.075) is False


def test_register_runtime_map(tmp_path):
    _write_occupancy(tmp_path, "map_auto_first", {})
    service = MapService(extra_dir=tmp_path)
    _write_occupancy(tmp_path, "map_auto_second", {})
    map_id = service.register(tmp_path / "map_auto_second.yaml")
    assert map_id == "map_auto_second"
    assert "map_auto_second" in [item.map_id for item in service.summaries()]
