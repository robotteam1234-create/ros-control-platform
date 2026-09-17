<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-17 | Updated: 2026-09-17 -->

# adapters

## Purpose
Robot data sources behind a common structural interface. Two implementations — a deterministic in-process mock and a rosbridge websocket client — selected at startup (`--mode`).

## Key Files
| File | Description |
|------|-------------|
| `base.py` | `RobotAdapter` Protocol: `connect/close/execute/events/frames`. Concrete adapters additionally expose the sync surface the app calls: `snapshot()`, `frame()`, `drain_events()`, `publish_manual_velocity()`, `apply_settings()`, `sensor_layers()` |
| `mock.py` | `MockRobotAdapter` — deterministic two-robot source driven by `MockScenario` (`NORMAL`, `SLAVE_OFFLINE`, `CAMERA_STALL`, `FOLLOW_LOST`, `COMMAND_REJECTED`); PIL-rendered frames; fixed lab poses; atomic settings with range validation |
| `ros.py` | `RosbridgeAdapter` — two independent websocket clients (one per robot; routing authority is the fixed config URL+domain, never a request); 1/2/4/8 s reconnect backoff; typeless `/control/status` subscribe; bounded `fragment` reassembly; commands via `call_service` (2 s timeout) to `/control/command` — **never writes `/cmd_vel`** (manual velocity → `TwistStamped` `/control/manual_velocity`); TF-graph BFS for map-frame pose; LaserScan → map projection; TLS/bearer from env-var names at connect time |

## For AI Agents

### Working In This Directory
- Mock vs ros key differences: mock is synchronous/deterministic/scenario-driven; ros is async with reconnect/freshness semantics and graceful degradation (`STALE`/`UNKNOWN`, reason codes `ROSBRIDGE_OFFLINE`, `ROSBRIDGE_TIMEOUT`, `MAP_TF_UNVERIFIED`). Mock fabricates formation measurements; ros always reports `UNPAIRED` (follow state lives on the robot). Ros `apply_settings` always returns False → settings run in observation mode.
- Unsupported data is a first-class state (`UNSUPPORTED`) — never fabricate it in either adapter.
- The typeless `/control/status` subscribe only resolves if a publisher exists when the backend connects — start watchdogs/bringup before the backend (or restart backend after).
- Op names starting with `follow_` divert to the slave follow service; everything else goes to `/control/command` (`pinky_control_interfaces/srv/ControlCommand`).
- Keep `connect_factory`, `clock`, and `reconnect_delays` injectable in `RosbridgeAdapter` — tests depend on them.

### Testing Requirements
- `../../tests/test_rosbridge_adapter.py` (domain lock, routing, camera-disabled subscribe, compressed decode, TF composition, scan expiry) and `../../tests/test_ros_smoke_local.py` (local gateway YAML). Run via `backend/.venv/bin/python -m pytest backend/tests/ -q` from repo root.

<!-- MANUAL: -->
