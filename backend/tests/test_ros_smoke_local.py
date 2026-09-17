"""Task 5 TDD: local ROS YAML must point at the PC rosbridge gateway.

Supersedes the Task 4 no-camera ruling: the zero-build rpicam-vid MJPEG
publisher (ros/pinky_rpicam_pub.py -> sensor_msgs/CompressedImage) runs
on both robots. robot_2 (1e3e) was verified live end-to-end; robot_1 is
enabled for line following (line_follow_service is robot_1-only).
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
    # Both robots run the rpicam-vid publisher -> cameras enabled
    # (robot_1 for line following, robot_2 verified live on 1e3e).
    assert by_id["robot_1"].camera.enabled is True
    assert by_id["robot_2"].camera.enabled is True
    assert by_id["robot_1"].services.control_available is True
    assert by_id["robot_2"].services.control_available is True
    # FollowCommand.srv does not exist yet -> stays unavailable.
    assert by_id["robot_2"].services.follow_available is False
