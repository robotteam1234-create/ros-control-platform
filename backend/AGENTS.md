<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-17 | Updated: 2026-09-17 -->

# backend

## Purpose
Root of the `pinky-control-center` Python package — the mock-first FastAPI control API for the 2-robot platform. Provides HTTP + WebSocket APIs, SQLite persistence, an adapter abstraction over mock/ROS robots, and a ~160-test pytest suite that runs without any ROS runtime.

## Key Files
| File | Description |
|------|-------------|
| `pyproject.toml` | Package metadata (Python `>=3.12,<3.13`), pinned deps, console script `pinky-control-center`, pytest config (`addopts = "-p no:launch_testing -p no:launch_ros"`) |
| `requirements.lock` | Fully pinned reproducible dep set; ROS `rclpy` stays a Jazzy system dependency, never pip-installed |

## Subdirectories
| Directory | Purpose |
|-----------|---------|
| `pinky_control_center/` | The application package: `main.create_app()` composition root, services, adapters, API routers (see `pinky_control_center/AGENTS.md`) |
| `tests/` | 26 pytest files / ~161 tests, all app-level (see `tests/AGENTS.md`) |
| `.venv/` | Python 3.12 virtualenv — do not edit; recreate per root recipe if needed |

## For AI Agents

### Working In This Directory
- All state lives on `app.state` — no DI framework. Route modules fetch collaborators from `request.app.state`.
- Runtime watchdog ticks at 20 Hz in `main.py`; injectable clocks everywhere (`monotonic_clock`, `storage_clock`, adapter `connect_factory`) — preserve injectability when editing.
- Adapter exception messages are never persisted (credential hygiene); passwords are scrypt-only, never logged.
- `CONTROL_PLATFORM_WORKERS` must be `1` (startup-enforced and asserted in tests).

### Testing Requirements
- Full suite: `backend/.venv/bin/python -m pytest backend/tests/ -q` from repo root.
- Single test: `... pytest backend/tests/test_X.py::test_y -v`.
- If ROS plugin clash breaks collection: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.
- TDD: failing test first (RED), minimal implement (GREEN), full suite before commit.

### Common Patterns
- Handlers raise `HTTPException(status, detail=CODE)`; `main.py` normalizes errors to `{"error": {"code","message","details"}}`.
- Mutations need `request_id` (24 h idempotency → 409 dupes) + control lease (except `POST /stop`) + `X-CSRF-Token` + matching `Origin`.

## Dependencies

### External
- `fastapi==0.115.6`, `uvicorn[standard]==0.34.0`, `pydantic==2.10.4`, `websockets==17.1`, `numpy==2.5.3`, `opencv-python-headless==5.0.0.93`, `pillow==11.0.0`
- Dev: `pytest==8.3.4`, `httpx==0.28.1`

### Internal
- The app never imports `rclpy` — it talks to robots purely over rosbridge websockets (`ws://127.0.0.1:9090` d12, `:9091` d13).
- `ros/pinky_control_interfaces` defines the msg/srv the ROS adapter invokes robot-side via `call_service`.

<!-- MANUAL: -->
