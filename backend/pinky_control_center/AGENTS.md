<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-17 | Updated: 2026-09-17 -->

# pinky_control_center

## Purpose
The FastAPI application package. `main.create_app()` is the composition root: builds one adapter (mock or ros), all services, SQLite storage, the command-dispatcher handler registry, and mounts all API routers.

## Key Files
| File | Description |
|------|-------------|
| `main.py` | `create_app(mode="mock"\|"ros")` composition root; registers op-name handlers; 20 Hz `watchdog()` → `runtime_tick()`; global error normalization; CLI (`--mode --host --port --config --database --reset-password ...`); enforces `CONTROL_PLATFORM_WORKERS=1` |
| `command_service.py` | `CommandQueue` (bounded-100 normal + unbounded priority lane) and `CommandDispatcher` — sole production path from accepted command to `adapter.execute`; `handlers` dict is the op-name registry |
| `auth.py` | Session cookie `cc_session` / CSRF cookie `cc_csrf`; `current_user`, `require_role`, `verify_mutation` (Origin + CSRF, constant-time), `websocket_user` |
| `config.py` | Pydantic YAML config models; validators enforce exactly 2 robots, domains 12/13, distinct bridge URLs, ws:// or wss://+TLS envs |
| `state_store.py` | `StateStore` — snapshot normalization with per-field staleness (OFFLINE >3 s / STALE >1 s, pose >1 s, battery >15 s); sticky `disconnect()` with 3 s auto-rejoin (`_link_alive`) |
| `storage.py` | `Storage` — all SQLite persistence: users (scrypt), sessions+CSRF, leases, commands (24 h idempotency), missions, alerts, single-row settings, `history_events` (30-day retention, dedupe keys) |
| `safety_service.py` | Stop-observation state machine: CONFIRMED = latch + odom agreement + 0.5 s stable; flags odom-vs-status disagreement; never claims physical braking |
| `teleop_service.py` | Manual teleop gating: lease required, monotonic `seq`, speed limits, 0.3 s deadman → tick returns ZERO |
| `mission_service.py` | T06/T07 orchestration: formation pair/ready, follow start, master navigation, 10 s slave settle, patrol advancement; restart lands missions in `PAUSED`/`SERVER_RESTART` (never auto-resumes motion) |
| `navigation_service.py` | Guarded single-robot map navigation + localization reset; validates map/frame/cells/pose/TF/capability; raises `NavigationConflict` |
| `line_follow_service.py` | Camera-driven line following, **robot_1 only**: PD control (KP 0.6, KD 0.15), ~10 Hz, LOST after 3 misses → zero velocity + one-shot `safety_pending` |
| `line_detector.py` | `detect_line(jpeg, mode)` → found/offset/area/polarity; OpenCV grayscale lower-half ROI, largest contour, offset ±1 |
| `mapping_service.py` | Mapping-session lifecycle over stages `wall_follow, frontier_explore, wall_fill, return_home`; validates transitions, submits `mapping_*` ops |
| `map_service.py` | Loads `resources/maps/*.yaml`, renders deterministic occupancy PNG (ETag), `is_free()` point check against the same raster |
| `camera_service.py` | Fans latest frames to viewer queues (quality 5/10/15 fps); `encode_camera_frame()` = 4-byte BE metadata length + JSON + JPEG |
| `recording_service.py` | Dataset recording: taps frames into `root/<label>/<robot_id>/NNNNNN.jpg` + `meta.jsonl` manifest |
| `alert_service.py` | Snapshot observations → one durable alert per `(code, robot_id)` with hysteresis; `evaluate()` returns only newly activated alerts (prevents stop storms); Korean user messages |
| `settings_service.py` | Versioned (optimistic-concurrency) admin settings; refuses changes while robots move or mid-mission; `initial_pose_allowed()` gate |
| `models.py` | All pydantic contracts/StrEnums (`RobotState`, `StateSnapshot`, `CommandRequest`, mission/alert/settings/history models); `FiniteFloat` rejects NaN/inf |
| `motion.py` | `STILL_LINEAR_MPS=0.01` / `STILL_ANGULAR_RPS=0.03` odom zero-noise tolerances used by settings/navigation gates |
| `dataset.py` | Pure helpers for recorded datasets: `read_manifest`, `split_rows`, `write_yolo_label` |

## Subdirectories
| Directory | Purpose |
|-----------|---------|
| `adapters/` | `RobotAdapter` implementations: mock + rosbridge (see `adapters/AGENTS.md`) |
| `api/` | Thin FastAPI route modules under `/api/v1` (see `api/AGENTS.md`) |
| `migrations/` | Numbered SQLite migrations applied by `Storage` at startup (001 users/leases → 007 history) |
| `resources/` | Ship-with package data: `config/` (defaults, robots.mock/ros YAML), `maps/` (mock_lab, mock_lab_b, map_260905), `worlds/` (map_260905.world SDF) |

## For AI Agents

### Working In This Directory
- Everything is constructed explicitly in `create_app()` and exposed via `app.state` — no DI framework.
- **Op-name registry:** `command_service.handlers[op_name]` maps ops to `async (operation, parameters, user=None) -> (accepted, result)` handlers. New ops go in the `formation_*`/`mission_*`/`mapping_*` blocks in `main.py`. **Op names must NOT start with `follow_`** — the ROS adapter diverts those to the robot follow service.
- **Mutation gates:** every mutation needs `request_id` (24 h idempotency, dupes → 409), a control lease (`lease_id` in payload or `X-Control-Lease-Id`; `POST /stop` is deliberately lease-free), `X-CSRF-Token`, and matching `Origin`.
- **Error contract:** raise `HTTPException(status, detail=CODE)`; validation failures → 422 `INVALID_VALUE`.
- **Safety invariants:** stop completion comes from `SafetyService` observation, never adapter acceptance; `linear_mps` is last-writer-wins (odom vs status) — use `odom_*` fields for independent checks; protective stops latch with no auto-resume; adapter exception text is never persisted.
- **Timing constants:** runtime watchdog 20 Hz; line-follow ~10 Hz; state WS 0.2 s; teleop deadman 0.3 s; stop CONFIRMED 0.5 s stability — keep clocks injectable.
- Pydantic models use `extra="forbid"`; domain IDs 12/13 enforced by config validators; ROS topic names are global (isolation is URL+domain) — never add namespace prefixes.

### Testing Requirements
- Suite lives in `../tests/` — run `backend/.venv/bin/python -m pytest backend/tests/ -q` from repo root (see `../tests/AGENTS.md`).
- TDD mandatory: RED → GREEN → full suite before commit.

<!-- MANUAL: -->
