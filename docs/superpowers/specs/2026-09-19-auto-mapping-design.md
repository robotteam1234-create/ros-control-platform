# Auto-Mapping Mode Design (robot_1)

Date: 2026-09-19 · Status: draft for review
Supersedes the unimplemented parts of `2026-09-16-lap585-mapping-design.md`
(supervisory absorption, Approach A) and adds live map streaming + dynamic
map import.

## Goal

When no static map covers the space, an operator starts autonomous mapping
from the web, watches the map being drawn live, and the finished result
becomes a selectable map for navigation — without shell access.

Non-goal: fully hands-off start. A supervisor must be on-site with an
e-stop; the UI button is lease-gated, not scheduled.

## Decisions

- Approach A (backend-managed runner), per approved discussion 2026-09-19.
  Rejected: side-car daemon (extra deployment unit), bare shell endpoint
  (rejected in 2026-09-16 doc as unsafe).
- robot_1 only (`SUPPORTED_ROBOTS = ("robot_1",)`), domain 12.
- The pipeline stays the vendored lap585 stack (`run_lap_real.sh` +
  `pinky_hybrid.py`) running on the PC. No port, no rewrite.
- Live map transport is rosbridge `/map` → backend → browser PNG over WS.
  The backend never imports rclpy; the browser never touches ROS.

## Architecture

```
Browser ──HTTP──► Backend API (mappings ops; lease + request_id + CSRF + Origin)
   │                    │ MappingRunner: setsid spawn / SIGINT→SIGKILL group
   │                    ▼
   │             run_lap_real.sh → pinky_hybrid.py   (PC; env from config)
   │                    │ /map  (slam_toolbox, nav_msgs/OccupancyGrid)
   └──WS /ws/mapping ◄── RosbridgeAdapter subscribe ── MapStreamService PNG ~1 Hz
                └── progress events (parsed from runner stdout)
```

## Components

### MappingRunner (`backend/pinky_control_center/mapping_runner.py`, new)

- `start(robot_id, lease_id)`: preflight — robot_1, no active session,
  rosbridge connected, `/scan` publishing, watchdog not latched. Spawns
  `bash run_lap_real.sh` via `setsid`, process-group leader, detached.
- Environment from config: `mapping.lap585_dir` (default repo `lap585/`),
  `mapping.overlay_dir` (default `~/nav_overlay` → AMENT_PREFIX_PATH,
  LD_LIBRARY_PATH incl. `usr/lib/x86_64-linux-gnu` + `usr/lib`,
  PYTHONPATH, PATH), `mapping.domain` (12), `ROS_LOCALHOST_ONLY` unset,
  `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, `PINKY_TRACK_W=0`,
  `PINKY_TRACK_H=0`. stdout+stderr → `<state_dir>/mapping/last_run.log`.
- Progress: async readline parses stage banners (`[N단계`, `N단계 결과`,
  `회차`, `최종:`, `맵 저장 완료`, `좌표계 손상`) → progress dict
  `{stage_index 0-3, stage, message, updated_at}` surfaced in status.
- `cancel()`: SIGINT group, 5 s grace, SIGKILL; then the existing stop path
  latches the watchdog (stop kills runner + latch, no auto-resume).
- Completion: exit code 0 AND `맵 저장 완료` in log → auto-import. Any
  other exit → session `FAILED` with last log lines in status.
- Backend restart mid-run: PID file under `<state_dir>/mapping/`; on boot,
  a live unknown PID marks the session `ORPHANED`; operator can kill it
  from the UI. Mirrors the SERVER_RESTART missions invariant.

### Live map stream (modify `adapters/ros.py`, new `map_stream_service.py`)

- While a session is RUNNING, RosbridgeAdapter subscribes `/map`
  (`nav_msgs/msg/OccupancyGrid`, `queue_length: 1`); unsubscribes on end.
- `MapStreamService` keeps the latest grid per robot, encodes to PNG
  (255 free / 0 occupied / 128 unknown) with metadata `{width, height,
  resolution, origin, seq}`; pushes on change at most 1 Hz.
- WS `/ws/mapping/robot_1` (auth: `websocket_user`): binary frames using
  the camera framing convention (4-byte BE metadata length + JSON + PNG)
  interleaved with JSON progress events.

### Dynamic map import (modify `map_service.py`)

- New scan path `<state_dir>/maps/*.yaml` merged at startup alongside
  packaged resources; identical YAML schema (`map_id, name, frame_id,
  resolution, width, height, origin, version`).
- On runner success: `lap585/map_real.{pgm,yaml}` copied in as
  `map_auto_<UTC timestamp>`, version "1", registered immediately (no
  restart). Navigation's `is_free` gate works on it unchanged.
- Re-import same space → new `map_auto_<ts>`; old ones stay (operator
  deletes files manually; deletion API out of scope).

### API (modify `api/mappings.py`, `main.py` registry)

- Existing ops get real behavior: `mapping_start` (lease required),
  `mapping_pause` (kill + latch, `PAUSED`), `mapping_resume` (fresh runner
  run — never auto-resumes mid-stage), `mapping_cancel`.
- New: `GET /api/v1/mappings/{id}` (state + progress), `GET
  /api/v1/mappings/{id}/log?tail=100`, `POST /api/v1/mappings/{id}/import`.
- Mapping creation payload validates `robot_id == "robot_1"` (422
  `ROBOT_NOT_SUPPORTED` otherwise).

### Frontend (new `MappingPanel.tsx`; modify `MapPanel.tsx`, `api.ts`, `App.tsx`)

- MappingPanel: start card ("이 공간 자동 매핑") with lease gate + stage
  progress list (`mappingStages.ts`), live map canvas (WS, PNG blit +
  resolution/origin scaling), cancel button (software-stop copy only),
  log tail expander, import button surfaced on completion.
- MapPanel: when no map covers the space, hint links to MappingPanel.
- `api.ts`: mapping status/log/import helpers; `mappingStream.ts` WS client.

## Safety

- All mutations: request_id + lease + X-CSRF-Token + Origin (422/409/403
  otherwise); start also requires control lease of robot_1.
- Watchdog stays the only `/cmd_vel` publisher; runner adds no publishers.
- Stop/pause kills the runner process group then latches; no auto-resume.
- UI copy says software-only stop, never "emergency".
- Preflight refuses start when `/scan` is dead or rosbridge is down.

## Error handling

- Runner spawn failure, missing overlay dir, or missing script → session
  `FAILED`, 409 `MAPPING_RUNNER_ERROR` with reason (adapter exception text
  never persisted).
- WS client disconnect/reconnect: latest-frame semantics; no backlog.
- Duplicate `mapping_start` while RUNNING → 409 (existing idempotency).

## Testing

- Backend pytest (TDD, fake runner script fixture that emits real banner
  lines): lifecycle transitions, progress parsing, cancel-kill group,
  preflight rejections, completion import (`map_auto_*` visible in
  `/api/v1/maps`), WS PNG framing, orphan-PID marking, op registry wiring.
- Frontend vitest: MappingPanel state machine, canvas draw from fake
  frames, api helpers (mirrors existing panel tests).
- Field: supervised P1 run on robot_1 → evidence to acceptance-report.md;
  unrun items stay NOT_RUN.

## Rollout

1. P0: runner + ops + import behind fake runner, full backend/frontend
   tests green.
2. P1: supervised real run on robot_1; live map + import verified in
   browser; acceptance evidence recorded.
