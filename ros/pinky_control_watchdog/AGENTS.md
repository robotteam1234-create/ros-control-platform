<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-17 | Updated: 2026-09-17 -->

# pinky_control_watchdog

## Purpose
ament_python package containing `ManualVelocityWatchdog` — the single safety mediator and the **only `/cmd_vel` publisher** in the entire system. Arbitrates manual vs navigation velocity, enforces deadmen/clamps/latching, serves the `ControlCommand` service, and publishes the `ControlStatus` heartbeat the backend depends on.

## Key Files
| File | Description |
|------|-------------|
| `pinky_control_watchdog/manual_velocity_watchdog.py` | Subs `TwistStamped /control/manual_velocity` + `Twist /control/nav_velocity`; serves `ControlCommand` on `/control/command`; publishes `Twist /cmd_vel` (reliable QoS, 20 Hz), `ControlStatus /control/status` (10 Hz), `UInt64 /control/heartbeat` |
| `setup.py` / `package.xml` | ament_python packaging; exec_deps on `nav2_msgs` (NavigateToPose action client) and `pinky_control_interfaces` |

## For AI Agents

### Working In This Directory
- **Boot latch:** starts `stop_latched=True` (`STARTUP_STOP_LATCH`) publishing zeros until `reset_stop`. **No auto-resume** — `reset_stop` always cancels navigation; pre-stop goals are never re-driven.
- **Deadmen:** manual 0.35 s, nav 0.50 s — stale input ⇒ zero velocity (`VELOCITY_WATCHDOG_TIMEOUT`).
- **Clamps:** `max_linear_mps 0.15`, `max_angular_rps 0.50` (runtime-adjustable via `apply_settings`).
- **Mode gate:** MANUAL uses the manual topic; AUTO/FOLLOW use the nav topic; anything else ⇒ zeros.
- `navigate` op drives a `NavigateToPose` action client; duplicate `command_id`s (128-deep deque) are acknowledged-but-ignored (idempotency).
- Status `capabilities` = `["manual"]` + `"navigate"` only while the action server is up.
- Node defaults `robot_id:=robot_2` — robot_1 deployments must override the parameter.
- Start this BEFORE the backend: the backend subscribes `/control/status` typelessly and rosbridge resolves it only if a publisher exists at connect time.

### Testing Requirements
- No unit tests here. Contract behavior (deadmen, latching, clamps) is mirrored by backend tests (`../../backend/tests/test_commands.py`, `test_safety.py`); physical stop verification is a field gate (T12/T15).

## Dependencies

### External
- `rclpy`, `geometry_msgs`, `nav2_msgs` (NavigateToPose action), `action_msgs`, `std_msgs`

### Internal
- `../pinky_control_interfaces/` (ControlCommand srv, ControlStatus msg) — must be built first.
- Consumers: backend `RosbridgeAdapter` (`/control/command` + `/control/status`) and lap585 scripts (`/control/manual_velocity` TwistStamped shim).

<!-- MANUAL: -->
