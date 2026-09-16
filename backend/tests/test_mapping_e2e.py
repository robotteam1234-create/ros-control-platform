import pytest


@pytest.mark.parametrize("op", ["validate", "start", "pause", "resume", "cancel"])
def test_mapping_ops_via_dispatcher(op):
    from uuid import uuid4
    from pinky_control_center.mapping_service import MappingService
    seen = []
    class Dispatcher:
        def submit(self, user, request_id, target, operation, parameters=None, priority=False):
            seen.append(operation)
            return {"state": "ACCEPTED", "command_id": str(uuid4())}
    svc = MappingService(storage=None, settings_provider=lambda: None)
    out = svc.action(None, "mid", uuid4(), op, Dispatcher())
    assert seen == [f"mapping_{op}"]
    assert out["state"] == "ACCEPTED"
