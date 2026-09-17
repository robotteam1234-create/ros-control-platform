<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-17 | Updated: 2026-09-17 -->

# pinky_control_interfaces

## Purpose
ament_cmake package defining the two shared message/service types used by the backend (via rosbridge), the watchdog, and the Nav2 integration. Must be built before any package that imports the generated code.

## Key Files
| File | Description |
|------|-------------|
| `msg/ControlStatus.msg` | Robot status heartbeat: `stamp, robot_id, mode, stop_latched, active_command_id, command_state, reason_code, capabilities[], linear_mps, angular_rps` |
| `srv/ControlCommand.srv` | Idempotent command contract: request `command_id, operation, parameters_json` → response `accepted, reason_code`. Ops: `stop`, `reset_stop`, `set_mode`, `navigate`, `cancel_navigation`, `apply_settings` |
| `CMakeLists.txt` | ament_cmake + `rosidl_generate_interfaces`, depends on `builtin_interfaces` |
| `package.xml` | rosidl interface package manifest |

## For AI Agents

### Working In This Directory
- Changing msg/srv fields breaks three consumers at once — keep fields in sync with backend `models.py` (`ControlStatus` parsing in `../pinky_control_watchdog/`, `../../backend/pinky_control_center/adapters/ros.py`, and `../../backend/pinky_control_center/models.py`).
- Build first with colcon: `colcon build --packages-select pinky_control_interfaces` in the robot/PC workspace before watchdog/navigation packages.

### Testing Requirements
- No tests; interface changes are validated indirectly by `../../backend/tests/test_rosbridge_adapter.py` contract tests on the wire format.

<!-- MANUAL: -->
