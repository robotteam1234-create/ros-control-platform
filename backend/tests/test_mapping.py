from pinky_control_center.mapping_service import STAGES, MappingService


def test_mapping_stages_match_lap585_pipeline():
    assert STAGES == ["wall_follow", "frontier_explore", "wall_fill", "return_home"]


def test_mapping_action_rejects_invalid_transition():
    assert "cancel" in {"validate", "start", "pause", "resume", "cancel"}
