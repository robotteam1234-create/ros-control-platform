# AGENTS.md — control-platform

Pinky Pro 2-robot control platform: FastAPI backend (`backend/`), React+TS frontend (`frontend/`), rosbridge adapters, SQLite. `robot_1` MASTER domain 12, `robot_2` SLAVE domain 13 — deployment invariants, never change.

## Commands

- Backend venv: `backend/.venv` (Python 3.12 only). If recreating without `python3.12-venv` package: `python3.12 -m venv --without-pip backend/.venv` + `get-pip.py`, then `backend/.venv/bin/pip install -e 'backend[dev]'`.
- Backend tests: `backend/.venv/bin/python -m pytest backend/tests/ -q` from repo root. Single test: `... pytest backend/tests/test_X.py::test_y -v`. If ROS plugin clash breaks collection (`launch_testing`/`lark`), retry with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.
- Frontend: `cd frontend && npm install`, tests `npx vitest run [path]`, typecheck `npx tsc --noEmit`, build `npm run build` (`tsc -b && vite build`).
- Dev servers: backend `CONTROL_PLATFORM_WORKERS=1 backend/.venv/bin/python -m pinky_control_center.main --mode mock --host 127.0.0.1 --port 8081 --database ~/.local/state/control-platform/control.db`; frontend `npm run dev` (5173, proxies `/api`+`/ws` to 8081). Published preview needs `preview.proxy` in `vite.config.ts` — plain `vite preview` serves a dead static page otherwise.
- Admin account: `backend/.venv/bin/python -m pinky_control_center.main --database <db> --reset-password <user> --password '<pw>' --role ADMIN`. Never log passwords. Default login for local mock: `operator`.
- Never run two backends on 8081; `pkill -f` self-matches its own command line — use bracket patterns (`[v]ite preview`).

## Architecture (non-obvious)

- Browser never touches ROS. Backend holds one rosbridge websocket per robot (`ws://127.0.0.1:9090` d12, `:9091` d13); isolation boundary is URL+domain, topics are global (`/odom`, `/tf`).
- `main.py` handler registry: new ops go in the `formation_*`/`mission_*`/`mapping_*` blocks; op names must NOT start with `follow_` (diverts to follow service). All mutations need `request_id` + lease + `X-CSRF-Token` + `Origin`, else 422/409/403.
- `RosbridgeAdapter` subscribes `/control/status` WITHOUT type — rosbridge can only resolve it if a publisher exists when the backend connects. Always start robot watchdogs/bringups BEFORE the backend, or restart backend after.
- `StateStore.disconnect()` is sticky by design; fresh adapter data rejoins automatically (see `state_store.py:_link_alive`, 3 s). Teleop WS close triggers protective stop + disconnect.
- Stop semantics: `SafetyService` CONFIRMED = latch + odom-agreement + 0.5 s stable. `linear_mps` is last-writer-wins (odom vs status); `odom_*` fields are odom-only — use those for independent checks. Stop UI copy must say software-only, never "emergency".
- Watchdog (`ros/pinky_control_watchdog`) is the SOLE `/cmd_vel` publisher (clamp 0.15/0.50, deadmen 0.35 s manual / 0.50 s nav, boot latch, no auto-resume). Nothing else may publish `/cmd_vel` — lap585 scripts use the TwistStamped shim.

## Conventions

- TDD: failing test first (RED), minimal implement (GREEN), full suite before commit. Backend `addopts` disables launch plugins; frontend tests use `within(view.container)` queries (render binds to body).
- Commits: `git -c user.name="opencode" -c user.email="opencode@localhost" commit` (never touch git config). Don't update `git config`, skip hooks, or force-push.
- Docs that matter: `docs/02-functional-spec.md` (contracts), `runbook.md` (field ops), `acceptance-report.md` (PASS needs log evidence; unrun stays NOT_RUN, never mark PASS by default).
- Robot SSH: `~/.config/pinky-control/robots.env` (600). Robots reboot often (suspect power); after reboot, bringups need manual `stack_start.sh <domain>` (no auto-start on `.202`), then restart backend so typeless subscriptions resolve.

## Environment gotchas

- No sudo on this PC. No system `rosbridge_server` — PC gateway runs from an overlay that MUST live outside `/tmp` (it gets wiped); rebuild via `apt-get download` + `dpkg-deb -x` per `deployment/launch/control_center.launch.py` env layout, with `~/pc_control_ws` sourced for `pinky_control_interfaces`.
- `CONTROL_PLATFORM_WORKERS` must be `1` (enforced at startup).
- Cameras: `camera.enabled=false` in field YAMLs (no ROS camera driver on robots); expect 503 `CAMERA_STALLED`.
- Known flakes: `test_t06`/`test_t10` timing-sensitive under load (rerun passes); PC ROS discovery goes stale (`ros2 daemon stop` before trusting listings); `.201` shells force `ROS_LOCALHOST_ONLY` — always export `ROS_DOMAIN_ID` + unset it explicitly over SSH.
