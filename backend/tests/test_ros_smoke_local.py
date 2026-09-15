"""Task 5 TDD: local ROS YAML must point at the PC rosbridge gateway.

Binding ruling (overrides brief): Task 4 proved no ROS camera driver exists
on either robot (NOT_RUN both legs), so camera.enabled stays FALSE for both
robots and NOT_RUN camera status is expected in the smoke test.
"""
from pathlib import Path

from pinky_control_center.config import load_ros_config


def test_local_ros_yaml_is_pc_gateway():
    cfg = load_ros_config(Path("deployment/robots.ros.local.yaml"))
    by_id = {r.robot_id: r for r in cfg.robots}
    assert by_id["robot_1"].domain_id == 12
    assert by_id["robot_2"].domain_id == 13
    assert by_id["robot_1"].bridge_url == "ws://127.0.0.1:9090"
    assert by_id["robot_2"].bridge_url == "ws://127.0.0.1:9091"
    # Binding ruling: no camera driver on either robot -> stays disabled.
    assert by_id["robot_1"].camera.enabled is False
    assert by_id["robot_2"].camera.enabled is False
    assert by_id["robot_1"].services.control_available is True
    assert by_id["robot_2"].services.control_available is True
    # FollowCommand.srv does not exist yet -> stays unavailable.
    assert by_id["robot_2"].services.follow_available is False
