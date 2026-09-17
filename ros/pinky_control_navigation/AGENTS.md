<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-17 | Updated: 2026-09-17 -->

# pinky_control_navigation

## Purpose
ament_python package: static-map Nav2 navigation for the robots — launch file, occupancy-grid map assets, Nav2 params, and the lifecycle gate node that activates navigation only after AMCL has a pose. Starts **no SLAM**.

## Key Files
| File | Description |
|------|-------------|
| `launch/robot_nav2.launch.py` | map_server + AMCL (localization lifecycle autostarts), then via 2 s TimerAction the controller/planner/behavior/bt_navigator/waypoint_follower + a navigation lifecycle manager with `autostart:=False` + the gate node; **velocity remap `cmd_vel → /control/nav_velocity`** on controller_server and behavior_server; TF remapped to relative names |
| `pinky_control_navigation/nav2_lifecycle_gate.py` | Console-script node polling TF `map → base_footprint` every 0.5 s; calls `ManageLifecycleNodes STARTUP` only after AMCL has a pose (fixed-timer activation would permanently leave costmaps inactive); retries failures |
| `params/nav2_params.yaml` | 338-line Nav2 config (Korean comments): Regulated Pure Pursuit (`desired_linear_vel: 0.2`, plain Twist via `enable_stamped_cmd_vel: false`), AMCL differential, NavFn planner, rectangle footprint, 8 cm xy goal tolerance |
| `map/map_260905.pgm/.yaml` | Occupancy grid (0.005 m/cell, origin `[-1.355, -0.63, 0]`) generated from the collision boxes in `../../backend/pinky_control_center/resources/worlds/map_260905.world` — dashboard and Nav2 share the same frame/origin |
| `setup.py` | Installs launch/map/params as data_files; console script `nav2_lifecycle_gate` |
| `README.md` | Ordering constraint: launch only after hardware bringup AND watchdog; AMCL repositioning via `/initialpose` after manually moving the robot |

## For AI Agents

### Working In This Directory
- **Never let Nav2 publish `/cmd_vel` directly** — the watchdog is the sole `/cmd_vel` publisher; Nav2 feeds `/control/nav_velocity` via the launch remap. Preserve the remap when editing the launch file.
- Launch ordering: hardware bringup → watchdog → this package → backend. The gate node exists because lifecycle activation timing matters — don't replace it with fixed timers.
- Map regeneration: derive from the `.world` collision boxes so dashboard and Nav2 stay in the same frame/origin.

### Testing Requirements
- No automated tests. Real-graph verification (AMCL/TF/low-speed drive) is a field gate per `../../acceptance-report.md` (T15 NOT_RUN items).

## Dependencies

### External
- Full Nav2 suite (amcl, controller, planner, behaviors, bt_navigator, waypoint_follower, lifecycle_manager, map_server), `rclpy`, `tf2_ros`

### Internal
- Watchdog consumes `/control/nav_velocity` (`../pinky_control_watchdog/`); map frame matches backend `map_service` raster (`../../backend/pinky_control_center/map_service.py`).

<!-- MANUAL: -->
