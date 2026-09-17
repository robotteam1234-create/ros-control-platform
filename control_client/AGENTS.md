<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-17 | Updated: 2026-09-17 -->

# control_client

## Purpose
Small async Python package that drives the robots **through the control-center web API** (never ROS directly): login → CSRF → control lease → stop-reset → MANUAL mode → teleop WebSocket streaming. Doubles as an integration/field-test driver and a reference implementation of the full HTTP+WS contract.

## Key Files
| File | Description |
|------|-------------|
| `client.py` | Core helpers: `new_request_id()`, `teleop_frame()`, `login()`, `acquire_lease()`, `reset_and_manual()` (re-leases twice — mode changes invalidate the lease), `stream_velocity()` (WS teleop, 10 Hz frames, 2 s ack timeout). Hardcoded `BASE=http://127.0.0.1:8081`, `ORIGIN=http://127.0.0.1:4173` |
| `patterns.py` | Pure drive-pattern generators returning `(linear, angular, seconds)` tuples: `square_legs()`, `straight_legs()` |
| `web_drive.py` | CLI: `web_drive.py <robot_1|robot_2|both> <pattern>`; logs in as `operator`, drives robots concurrently via `asyncio.gather` |
| `test_client.py` | Pure pytest unit tests — no hardware, no server needed |

## For AI Agents

### Working In This Directory
- Dev/field tool, not production code: `BASE`, `ORIGIN`, and credentials are hardcoded. `ORIGIN` must match the backend's allowed origin or mutations get 403.
- Every mutating call must carry `request_id` + `lease_id` + `X-CSRF-Token` + `Origin`, mirroring the backend's 422/409/403 contract.
- `stream_velocity` breaks on a 2 s ack timeout — that is protective-stop semantics upstream, not a bug.

### Testing Requirements
- Tests live outside `backend/tests/`, so run explicitly: `backend/.venv/bin/python -m pytest control_client/test_client.py` from repo root (standard backend suite does not pick them up).

## Dependencies

### External
- `httpx`, `websockets` (needs the `additional_headers` API ⇒ websockets ≥ 11-style), stdlib `asyncio`/`json`/`uuid`

### Internal
- Requires a running backend (mock or ros mode) on `:8081` for anything beyond `test_client.py`.
- Exercises the same contract documented in `backend/pinky_control_center/api/` and `docs/02-functional-spec.md`.

<!-- MANUAL: -->
