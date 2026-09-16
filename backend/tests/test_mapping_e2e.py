def test_mapping_start_pause_cycle_via_dispatcher():
    from uuid import uuid4
    from pinky_control_center.mapping_service import MappingService
    seen = []
    class Dispatcher:
        def submit(self, user, request_id, target, operation, parameters=None, priority=False):
            seen.append(operation)
            return {"state": "ACCEPTED", "command_id": str(uuid4())}
    svc = MappingService(storage=None, settings_provider=lambda: None)
    out = svc.action(None, "mid", uuid4(), "start", Dispatcher())
    assert seen == ["mapping_start"]
    assert out["state"] == "ACCEPTED"
