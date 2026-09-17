<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-17 | Updated: 2026-09-17 -->

# api

## Purpose
Thin FastAPI route modules over `app.state.*` services. All paths under `/api/v1`; most modules use module-level `router = APIRouter(prefix="/api/v1")`, state/maps/settings/cameras use `create_router(...)` factories (they need service instances).

## Key Files
| File | Description |
|------|-------------|
| `session.py` | `POST /session` (login: Origin check, sets `cc_session` + `cc_csrf` cookies), `GET /session`, `DELETE /session` (CSRF-verified logout) |
| `control.py` | `operator` dependency (OPERATOR/ADMIN + `verify_mutation`); control-lease CRUD (409 `CONTROL_CONFLICT`); `POST /stop` (202, **no lease**, priority lane); `POST /stop/reset`; `POST /robots/{id}/mode` (409 `STOP_LATCHED` gate); `GET /commands/{id}`; `WS /ws/teleop` (seq monotonicity, speed limits, lease ownership; disconnect → protective stop) |
| `missions.py` | `request_id()` (422) + `leased_operator()` (409) helpers reused by other modules; `POST /formation/actions`; missions CRUD (paging, `state`/`from`/`to` filters, PATCH with request-id replay → 409); `POST /missions/{id}/actions` (202) |
| `line_follow.py` | `POST /robots/{id}/line-follow/actions` (202; robot_1 only) + `GET` status; `build_line_start_handler`/`build_line_stop_handler` factories registered in `main.py` |
| `mappings.py` | `POST /mappings` (201, DRAFT session); `POST /mappings/{id}/actions` (202, validate/start/pause/resume/cancel) |
| `recordings.py` | Dataset recording: `POST /recordings/start` (201), `POST /recordings/stop`, `GET /recordings` |
| `navigation.py` | `POST /robots/{id}/navigate` (202), `POST /robots/{id}/localization-reset` (202); lease from payload or `X-Control-Lease-Id` header |
| `maps.py` | `create_router(map_service)`: `GET /maps`, `GET /maps/{id}` (metadata), `GET /maps/{id}/data` (PNG, ETag + 304 + private cache) |
| `settings.py` | `create_router()` with `admin` dependency: `GET/PUT /settings` (optimistic `version`), `POST /robots/{id}/initial-pose` (202; 409 `ROBOT_NOT_STOPPED`) |
| `state.py` | `create_router(state_store)`: `GET /state`; `WS /ws/state` (snapshot every 0.2 s) |
| `cameras.py` | `create_router(camera_service)`: `WS /ws/cameras/{robot_id}?quality=` (binary framed JPEG; close 4404/4400) |
| `alerts.py` | `GET /alerts` (state/severity/robot filters, cursor paging); `POST /alerts/{id}/ack` (CSRF + request_id) |
| `history.py` | `GET /history` (filters, default 24 h, max 30 d; naive timestamps → 422 `UTC_TIMESTAMP_REQUIRED`); `GET /history/export` (>10k → 413, never truncated) |
| `sensors.py` | `GET /robots/{id}/sensor-layers` — scan/costmap overlay payload from the adapter |

## For AI Agents

### Working In This Directory
- Route modules fetch collaborators from `request.app.state` — no DI framework.
- Mutations need `request_id` (24 h idempotency, dupes → 409) + control lease (payload or `X-Control-Lease-Id`; `POST /stop` is deliberately lease-free) + `X-CSRF-Token` + `Origin` → else 403/409/422.
- New op handlers belong in `main.py`'s `formation_*`/`mission_*`/`mapping_*` registration blocks, not here — this directory routes, it doesn't implement ops.
- `POST /stop` must never gain a lease requirement — it is the priority-lane safety path.

### Testing Requirements
- Route behavior is covered by `../../tests/test_*.py` files (auth, contracts, t-numbered suites). Run from repo root: `backend/.venv/bin/python -m pytest backend/tests/ -q`.

<!-- MANUAL: -->
