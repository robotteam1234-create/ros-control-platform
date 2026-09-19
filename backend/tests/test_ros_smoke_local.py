"""Local ROS YAML smoke: both robots must use their ONBOARD rosbridge.

Both Pinkys run pinky-bringup@/pinky-session@ user units (ros/
pinky_control_bringup deploy) that serve rosbridge on the robot itself
(robot_1 d12 :9090, robot_2 d13 :9091). The backend connects to those
endpoints directly; the PC-side gateway pair is no longer the path.
"""
from pathlib import Path

from pinky_control_center.config import load_ros_config


def test_local_ros_yaml_is_onboard_bridges():
    cfg = load_ros_config(Path("deployment/robots.ros.local.yaml"))
    by_id = {r.robot_id: r for r in cfg.robots}
    assert by_id["robot_1"].domain_id == 12
    assert by_id["robot_2"].domain_id == 13
    assert by_id["robot_1"].bridge_url == "ws://192.168.1.201:9090"
    assert by_id["robot_2"].bridge_url == "ws://192.168.1.202:9091"
    # Onboard watchdogs ship the control contract on both robots.
    assert by_id["robot_1"].services.control_available is True
    assert by_id["robot_2"].services.control_available is True
    # FollowCommand.srv does not exist yet -> stays unavailable.
    assert by_id["robot_2"].services.follow_available is False
