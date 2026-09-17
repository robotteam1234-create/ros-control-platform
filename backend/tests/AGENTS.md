<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-17 | Updated: 2026-09-17 -->

# tests

## Purpose
26 pytest files / ~161 tests, all app-level (no ROS runtime needed). 19 files drive the app via FastAPI `TestClient`/httpx. This is the backend TDD home: RED → GREEN per root AGENTS.md.

## Key Files
| Group | Files | Covers |
|-------|-------|--------|
| Auth/session/lease | `test_auth.py`, `test_auth_bypass.py` | Login roles, CSRF, Origin, WS auth (4401/4403), lease conflicts/expiry, bypass defaults off |
| Commands/dispatcher | `test_commands.py` | Queue cap 100 + priority stop bypass, teleop watchdog zeros, stop latching, CONFIRMED completion |
| Contracts | `test_contracts.py` | NaN/bounds rejection, config validators (exactly-2-robots), mock scenario isolation, per-robot JPEG frames |
| Alerts | `test_alerts.py` | Battery hysteresis, dedupe per `(code, robot_id)`, ack semantics, FOLLOW_LOSS/COMM_LOSS single protective stop |
| Line following | `test_line_detector.py`, `test_line_follow_service.py`, `test_line_follow_api.py`, `test_line_follow_runtime.py`, `test_line_follow_safety.py` | Offset sign/polarity, bounded velocity, 3-miss LOST, ~10 Hz runtime ticks, stale-camera protective stop |
| Mapping | `test_mapping.py`, `test_mapping_e2e.py` | Stage names match lap585 pipeline, invalid transitions, mapping ops through dispatcher |
| Maps | `test_maps.py` | Metadata/PNG/ETag, world-vs-occupancy consistency, TF-invalid clears formation measurements |
| Recording | `test_recording.py`, `test_recordings_api.py`, `test_recording_tap.py` | Service files+manifest, HTTP start/stop, runtime frame taps |
| ROS adapter | `test_rosbridge_adapter.py`, `test_ros_smoke_local.py` | Domain-id lock, per-robot routing, compressed decode, TF composition, LaserScan projection (injected `connect_factory`); local gateway YAML smoke |
| Safety/state | `test_safety.py`, `test_state.py` | Lease-less all-stop, CONFIRMED semantics, per-field freshness thresholds |
| T-numbered suites | `test_t06_mission.py`, `test_t07_patrol.py`, `test_t09_settings.py`, `test_t10_history.py`, `test_t14_acceptance.py`, `test_t15_map_navigation.py` | Feature acceptance per functional spec (T06/T10 timing-sensitive under load — rerun passes) |

## For AI Agents

### Working In This Directory
- **No `conftest.py`** — apps are built per-test via `create_app(...)` with `tmp_path` databases and injected fake clocks. Keep that pattern; don't introduce shared fixtures casually.
- Clocks and the ROS adapter's `connect_factory` are injectable — use them for deterministic timing tests instead of sleeps.
- Naming: feature tests follow functional-spec tickets (`test_tNN_*.py`); new tasks get a matching file, and evidence goes to `../../docs/tdd/`.

### Testing Requirements
- Full suite: `../../.venv`-style path from repo root — `backend/.venv/bin/python -m pytest backend/tests/ -q`.
- Single test: `backend/.venv/bin/python -m pytest backend/tests/test_X.py::test_y -v`.
- ROS plugin clash (`launch_testing`/`lark`)? Retry with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` (also disabled in `pyproject.toml` addopts).
- Known flakes: `test_t06`/`test_t10` under load — rerun passes.

## Dependencies

### External
- `pytest==8.3.4`, `httpx==0.28.1` (TestClient/AsyncClient)

### Internal
- Exercised through `../pinky_control_center/main.create_app()`; acceptance suite is invoked by `../../deployment/scripts/acceptance.sh`.

<!-- MANUAL: -->
