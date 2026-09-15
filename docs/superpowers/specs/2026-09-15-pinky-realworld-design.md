# Pinky Real-World Design (PC rosbridge gateway)

Date: 2026-09-15 · Scope: full missions + Nav2 · Status: approved design, not yet implemented

## Goal
Run control-platform against real Pinkys with parity to mock UI:
view telemetry + teleop + full stop + AMCL reset + map goals +
patrol/formation via Nav2, cameras on both robots.

Locked inputs: 764e = robot_1 MASTER, 1e3e = robot_2 SLAVE;
lab-slow + latch (0.15 m/s clamp); full SSH + install allowed;
cameras required both robots.

## Context
- Backend `RosbridgeAdapter` expects 2x rosbridge websockets,
  domains 12/13, topics `/odom /scan /battery/* /tf`,
  custom `/control/status`, `/control/command`
  (`pinky_control_interfaces/srv/ControlCommand`),
  compressed camera. Browser never touches ROS.
- Reality found 2026-09-15: robots on 192.168.1.201 (.764e, domain 0)
  and .202 (.1e3e, domain 12), ~3-5ms ping, topics
  `/odom ~30Hz /scan ~10Hz /battery/* /tf /cmd_vel`, no `/control/*`,
  no camera topics, no map->odom TF. No rosbridge installed,
  nothing on 9090/9091. domain_bridge forwards cmd_vel PC->robot
  but odom/scan return path absent on domain 5.

## Decision (Approach B)
PC rosbridge gateway, robots realigned to domains 12/13.
Rejected: A onboard rosbridge (heavier Pi load, same robot installs
anyway), C native DDS adapter (new rclpy adapter, longest to parity).

Architecture:
robots (domain 12/13 DDS) -> PC 2x rosbridge_websocket
(12 -> ws://127.0.0.1:9090, 13 -> ws://127.0.0.1:9091)
-> existing RosbridgeAdapter -> FastAPI /api + /ws.
Retire domain_bridge for this path to avoid dual cmd_vel writers.

## Changes
Robot-side (SSH, both robots):
- `ROS_DOMAIN_ID`: 764e 0 -> 12, 1e3e 12 -> 13; fix stack_start.sh / systemd.
- Install `pinky_control_interfaces`, `pinky_control_watchdog`
  (manual/nav mux -> /cmd_vel, clamp 0.15 m/s / 0.5 rad/s, boot latch),
  `pinky_control_navigation` (map_server + AMCL + planner/controller +
  lifecycle gate on map->base_footprint).
- Camera republisher -> `/camera/image_raw/compressed`
  (throttle/fragment per RosbridgeCameraOptions).
PC-side:
- `deployment/launch/control_center.launch.py`: launch 2x rosbridge_server.
- `deployment/robots.ros.local.yaml`: bridge_url ws://127.0.0.1:9090/9091,
  domain_id 12/13, control_available=true (after no-motion check),
  camera.enabled=true, topics/services per existing schema.
- Backend code: no contract change (validator stays 12/13).
  Only revisit `config.py` if domain realignment proves impossible.

## Data flow
Sub: odom / battery / scan / tf+tf_static / path / compressed camera.
Pub: TwistStamped -> /control/manual_velocity;
PoseWithCovarianceStamped -> /initialpose.
Service: ControlCommand{command_id, operation, parameters_json}
-> /control/command; follow ops only if follow_available.

## Safety / errors
Boot latched STOP, no auto-resume. Deadman 0.35s manual / 0.5s nav
-> zero velocity + protective stop. MAP_TF_UNVERIFIED blocks goals.
Rosbridge loss -> OFFLINE/STALE, backoff 1/2/4/8s, commands
ROSBRIDGE_OFFLINE, camera 503 CAMERA_STALLED, no mock fallback.
Lease expiry stops owner lease only (prior over-stop bug fixed).

## Rollout / acceptance
P0 gateway + no-motion gate (status/camera/stop/reset service).
P1 watchdog/teleop supervised slow. P2 AMCL reset + single goal
(tolerance 0.08 m / 0.17 rad). P3 patrol/formation.
Wheels-up/tether first, then supervised driving.
Evidence appended to acceptance-report.md ROS/Gazebo gate table;
unrun stays NOT_RUN, never PASS by default.
