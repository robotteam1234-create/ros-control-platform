<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-17 | Updated: 2026-09-17 -->

# ros

## Purpose
Robot-side ROS 2 (Jazzy) code: the safety watchdog that is the **sole `/cmd_vel` publisher**, the shared msg/srv definitions, the static-map Nav2 navigation stack, and two zero-build camera publisher scripts meant to be `scp`-ed onto the robots.

## Key Files
| File | Description |
|------|-------------|
| `pinky_camera_pub.py` | Standalone camera publisher (picamera2 preferred, V4L2/OpenCV fallback) → JPEG `CompressedImage` ~10 fps on `/camera/image_raw/compressed`; deploy by scp, run manually with `ROS_DOMAIN_ID` set — no colcon build |
| `pinky_rpicam_pub.py` | Alternative publisher spawning `rpicam-vid --codec mjpeg` and parsing SOI/EOI frames from stdout (pip picamera2 crashes on the Pinkys; `camera_ros` finds no cameras). Strips `LD_LIBRARY_PATH` for the child and wraps it in `stdbuf -o0` — **preserve both if editing** |

## Subdirectories
| Directory | Purpose |
|-----------|---------|
| `pinky_control_interfaces/` | ament_cmake package: `ControlStatus.msg` + `ControlCommand.srv` (see its `AGENTS.md`) |
| `pinky_control_watchdog/` | ament_python: `ManualVelocityWatchdog` — single safety mediator, only `/cmd_vel` publisher (see its `AGENTS.md`) |
| `pinky_control_navigation/` | ament_python: static-map Nav2 launch, map assets, params, AMCL lifecycle gate (see its `AGENTS.md`) |

## For AI Agents

### Working In This Directory
- **Never publish `/cmd_vel` from anything new.** Nav2 is remapped to `/control/nav_velocity`; any other motion source must go through the watchdog service or a TwistStamped shim on `/control/manual_velocity`.
- Start watchdog/bringup **before** the backend — the backend subscribes to `/control/status` without a type, and rosbridge only resolves it if a publisher exists at connect time.
- Build with colcon into the robot workspace; `pinky_control_interfaces` must build before the watchdog and navigation packages (they import generated msg/srv).
- Watchdog node defaults `robot_id:=robot_2`; robot_1 deployments must override the parameter.
- Field configs keep cameras disabled; expect 503 `CAMERA_STALLED` from the backend.

### Testing Requirements
- No automated tests here. Backend contract tests (`../backend/tests/test_rosbridge_adapter.py`, `test_ros_smoke_local.py`) cover the wire contracts; real-graph verification is a field gate per `../docs/tdd/T12-rosbridge.md`.

## Dependencies

### External
- `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `action_msgs`, `nav2_msgs`, `tf2_ros`, full Nav2 suite, `rosidl_default_generators` (interfaces pkg)
- OpenCV/picamera2 (only `pinky_camera_pub.py`); external binary `rpicam-vid` (`pinky_rpicam_pub.py`)

### Internal
- Backend `RosbridgeAdapter` (`../backend/pinky_control_center/adapters/ros.py`) calls `/control/command` (`ControlCommand` srv) and consumes `/control/status` (`ControlStatus` msg) — keep msg/srv fields in sync with `models.py` expectations.

<!-- MANUAL: -->
