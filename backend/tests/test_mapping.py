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
