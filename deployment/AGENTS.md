<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-17 | Updated: 2026-09-17 -->

# deployment

## Purpose
Everything needed to run the platform operationally: the PC-side rosbridge pair launch, nginx TLS reverse proxy, environment/robot config templates, a hardened systemd unit, and shell/Python ops scripts for validation, acceptance, and robot-side bringup.

## Key Files
| File | Description |
|------|-------------|
| `launch/control_center.launch.py` | Starts two isolated `rosbridge_websocket` instances (:9090 domain 12, :9091 domain 13); forces `/usr/bin`-first PATH (dev venvs lack Debian's `bson`); runnable directly with `python3` |
| `nginx/control-platform.conf` | TLS reverse proxy template: serves `frontend/dist`, proxies `/api/`+`/ws/` to `127.0.0.1:8081`, SPA fallback; certs at `/etc/pinky-control-center/tls/` |
| `env.example` | Template for `deployment/.env` (gitignored-by-convention): mode, port 8081, `WORKERS=1`, fixed `ROS_DOMAIN_ID_ROBOT_1=12` / `ROBOT_2=13` |
| `robots.ros.example.yaml` | Production robot mapping template: d12/:9090 + d13/:9091, global topics, `control_available:false`, camera off, secrets via env only |
| `robots.ros.local.yaml` | Local-testing profile: `control_available:true`, `camera.enabled:true` on both robots |
| `systemd/pinky-control-center.service` | Runs `start-backend.sh` as `control-center` user, `Restart=on-failure`, systemd hardening (`ProtectSystem=strict`) |
| `scripts/start-backend.sh` | Validates `MODE`/`SECURE_COOKIES`, creates DB dir, `exec`s `python -m pinky_control_center.main` (`--config` only in ros mode) |
| `scripts/validate-config.py` | Enforces invariants in the env file: required keys, `WORKERS=1`, domains exactly 12/13, distinct ws/wss URLs |
| `scripts/validate-deployment.sh` | Wrapper running `validate-config.py` against `deployment/.env` |
| `scripts/acceptance.sh` | Runs `backend/tests/test_t14_acceptance.py` via backend venv — the gate behind `acceptance-report.md` |
| `scripts/start-pinky-robot2-session.sh` | Robot-side session for robot_2: rosbridge :9091 + watchdog + optional Nav2 (`START_NAV2=1`); blocks until `/control/status` has a publisher |
| `scripts/start-pinky-robot2-all.sh` | One-stop robot_2 supervisor: bringup → wait `/odom`+`/scan` → session; one Ctrl+C tears down everything; hard-fails unless `ROS_DOMAIN_ID=13` |
| `scripts/drive-both-pinky.sh` | Manual field test driving BOTH robots via the domain bridge — **bypasses the watchdog**, valid only against namespaced bridged topics |

## Subdirectories
| Directory | Purpose |
|-----------|---------|
| `launch/` | rosbridge pair launch file |
| `nginx/` | TLS reverse proxy config |
| `scripts/` | Ops/bringup/acceptance scripts |
| `systemd/` | Service unit |

## For AI Agents

### Working In This Directory
- `deployment/` is deliberately **not** a colcon package — invoke `control_center.launch.py` directly with `python3`, not `ros2 launch`.
- Domain IDs, ports, and `CONTROL_PLATFORM_WORKERS=1` are invariants enforced by `validate-config.py`; do not parameterize them further.
- Scripts use bracket-pattern `pgrep -f '[/x]...'` to avoid self-matching — keep that pattern when editing.
- `start-pinky-robot2-*` scripts enable `set -u` only **after** sourcing ROS underlays (ROS setup scripts read unset vars) and unset `ROS_LOCALHOST_ONLY` explicitly.
- `env.example` → `.env` is never committed; real secrets live in env vars, never in `robots.ros.*.yaml`.

### Testing Requirements
- `deployment/scripts/validate-deployment.sh` validates the env file.
- `deployment/scripts/acceptance.sh` runs the mock acceptance smoke (from repo root); results go to `acceptance-report.md` — PASS needs logged evidence, otherwise stay NOT_RUN.

## Dependencies

### External
- ROS Jazzy + `rosbridge_server`, nginx, systemd, Python 3.12 backend venv
- Robot-side workspaces (`/home/pinky/pinky_pro`, `/home/pinky/dev_ws/wj`); `domain_bridge` (for `drive-both-pinky.sh` only)

### Internal
- Backend (`../backend/`) and frontend build (`../frontend/dist`) are what this directory deploys.
- Robot-side session scripts start the watchdog from `../ros/pinky_control_watchdog/`.

<!-- MANUAL: -->
