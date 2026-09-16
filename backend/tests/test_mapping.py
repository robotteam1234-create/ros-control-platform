import asyncio
from uuid import uuid4

import pytest
from fastapi import HTTPException

from pinky_control_center.mapping_service import STAGES, MappingService


def test_mapping_stages_match_lap585_pipeline():
    assert STAGES == ["wall_follow", "frontier_explore", "wall_fill", "return_home"]


def test_mapping_action_rejects_invalid_transition():
    svc = MappingService(storage=None, settings_provider=None)
    with pytest.raises(HTTPException) as exc_info:
        svc.action(
            user=None,
            mapping_id="mapping-1",
            request_id=uuid4(),
            action="bogus_action",
            dispatcher=None,
        )
    assert exc_info.value.status_code == 422
    ok, result = asyncio.run(svc.execute_mapping("mapping_bogus", {"mapping_id": "mapping-1"}))
    assert ok is False
    assert result == {"reason_code": "INVALID_VALUE"}


def test_create_mapping_requires_lease():
    from fastapi.testclient import TestClient
    from pinky_control_center.main import create_app
    client = TestClient(create_app(mode="mock"))
    r = client.post("/api/v1/mappings", json={"request_id": "00000000-0000-0000-0000-000000000000", "name": "m", "robot_id": "robot_1", "map_id": "map_260905"})
    assert r.status_code in (401, 403, 409)
