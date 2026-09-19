# pinky_control_bringup

## Purpose
ament_python package: robot-side boot auto-start. Two generalized runtime scripts (hardware bringup, control session), template systemd **user** units instantiated per robot (`pinky-bringup@<instance>`, `pinky-session@<instance>`), per-robot env files, and a no-sudo installer run on the robot over SSH. Both robots start their full stack at boot: bringup → rosbridge → watchdog → Nav2. Watchdog boots STOP-latched (no auto-resume), so auto-start is safe; AMCL initial pose stays a manual web-UI step after every reboot.

## Key Files
| File | Description |
|------|-------------|
| `scripts/robot_bringup.sh` | Unit payload for `pinky-bringup@`: sources Jazzy + both overlays (`set -u` only after), runs vendor `pinky_bringup bringup_robot.launch.xml` in the foreground, gates `/odom`+`/scan` (45 s each), stops legacy `rosy-session-*` units as a migration guard |
| `scripts/robot_session.sh` | Unit payload for `pinky-session@`: rosbridge + watchdog (env-driven `robot_id`) + optional Nav2 (`START_NAV2`); duplicate `pgrep` guards (bracket patterns), `setsid` groups, health gates `/control/status` (15 s) + `/navigate_to_pose` (60 s), `wait -n` supervisor |
| `scripts/lib/wait_for.sh` | Shared gates: `wait_for_publisher`, `wait_for_action_server`, `start_lidar_motor` (returns, does not exit) |
| `scripts/install.sh <robot_1\|robot_2>` | On-robot installer: verifies workspaces built, copies payload → `~/pinky/startup/bin/`, env → `~/.config/pinky-control/<i>.env`, units → `~/.config/systemd/user/`, disables legacy `rosy-session-*`, verifies linger, `enable --now` both units |
| `systemd/pinky-bringup@.service` | Template unit: `EnvironmentFile=%h/.config/pinky-control/%i.env`, `Restart=on-failure` 5 s, `StartLimitBurst=5`/120 s, `WantedBy=default.target` |
| `systemd/pinky-session@.service` | Same + `Requires`/`After`/`PartOf` `pinky-bringup@%i.service` — a session crash never re-opens serial devices |
| `config/robot_1.env` | `ROBOT_ID=robot_1`, `ROS_DOMAIN_ID=12`, `ROSBRIDGE_PORT=9090`, `START_NAV2=1`, workspace paths |
| `config/robot_2.env` | `ROBOT_ID=robot_2`, `ROS_DOMAIN_ID=13`, `ROSBRIDGE_PORT=9091`, `START_NAV2=1`, workspace paths |

## For AI Agents

### Working In This Directory
- Scripts take NO positional args and NO robot literals — every input comes from the unit `EnvironmentFile`. `robot_session.sh` hard-fails unless the domain/port pair is the fixed mapping 12↔9090 or 13↔9091.
- Keep the proven idioms when editing: `set -u` only after sourcing ROS overlays, `unset ROS_LOCALHOST_ONLY`, `PATH=/usr/bin:/bin` first, bracket-pattern `pgrep -f '[x]...'`, per-process `setsid` + INT group-kill cleanup.
- The watchdog stays the SOLE `/cmd_vel` publisher; never add a motion publisher here. Nav2 keeps its `/control/nav_velocity` remap (owned by `../pinky_control_navigation/`).
- Units run WITHOUT an SSH session (linger). Payload path `~/pinky/startup/bin/` is a fixed convention shared by `install.sh` and the unit files — change all of them together or none.
- Legacy `~/pinky/stack_start.sh` and `deployment/scripts/start-pinky-robot2-*.sh` remain as manual fallbacks; `install.sh` refuses to run while those wrappers are alive.

### Testing Requirements
- No automated tests (repo convention for `ros/`). Gate per change: `bash -n` on every script, `systemd-analyze --user verify` on unit templates, `colcon build --packages-select pinky_control_bringup` smoke.
- Field gate per robot: reboot → `journalctl --user -u 'pinky-*'` shows bringup gates then session READY → `/control/status` publishing, watchdog STOP-latched → backend ONLINE → set AMCL pose in UI → low-speed drive.

## Dependencies

### External
- systemd (user units + linger), ROS Jazzy, `rosbridge_server`, Nav2 (via `pinky_control_navigation`)

### Internal
- Vendor bringup `pinky_bringup` (workspace `~/pinky_pro`, NOT this repo); `../pinky_control_watchdog/`, `../pinky_control_navigation/`, `../pinky_control_interfaces/` built in `~/dev_ws/wj`.
- Consumers: field operators (install/runbook) and the backend (starts AFTER these units — typeless `/control/status` subscription needs the watchdog up first; backend restart after robot reboot remains a manual step).
