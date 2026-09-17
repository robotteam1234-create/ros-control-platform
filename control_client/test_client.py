"""Tests for control-client helpers (pure, no hardware)."""

from control_client.client import new_request_id, teleop_frame


def test_request_ids_unique():
    assert new_request_id() != new_request_id()


def test_teleop_frame_schema():
    import json

    msg = json.loads(teleop_frame("L", "robot_1", 3, 0.1, 0.2))
    assert msg == {"lease_id": "L", "robot_id": "robot_1", "seq": 3, "linear_mps": 0.1, "angular_rps": 0.2}


def test_drive_pattern_distances():
    from control_client.patterns import square_legs

    legs = square_legs(side_secs=3.0, turn_secs=3.2, speed=0.1, turn_rate=0.4)
    assert len(legs) == 8
    assert legs[0] == (0.1, 0.0, 3.0)
    assert legs[1] == (0.0, 0.4, 3.2)
