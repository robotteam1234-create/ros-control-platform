<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-17 | Updated: 2026-09-17 -->

# src

## Purpose
All frontend source: the root dashboard (`App.tsx`), typed REST/WS client modules, map/camera/teleop/mission panels, and colocated vitest tests. Korean UI; the browser only ever talks to the backend, never to ROS.

## Key Files
| File | Description |
|------|-------------|
| `main.tsx` | Entry: `createRoot(#root).render(<StrictMode><App/></StrictMode>)`, imports `styles.css` |
| `App.tsx` | Root dashboard: login gate, `stateSocket()` WS + 10 s REST poll backstop, lease lifecycle (1 s renew), blocker gating for buttons; renders all panels; exports pure helpers `dashboardErrorMessage()` / `isRobotStill()` |
| `api.ts` | Typed REST client for `/api/v1/*` (session, maps, settings, leases, missions, mapping, navigation, line-follow, recordings, alerts, history) + `stateSocket()`: auto-reconnect WS with seq-gap detection → REST resync, backoff 1 s→8 s, close 1008/4401/4403 → session-expiry logout |
| `control.ts` | Focused helpers: `stop(target)` (lease-less), `command(id)` poll, `resetStop`, `setMode` |
| `camera.ts` | `decodeFrame(ArrayBuffer)`: parses binary WS frames (4-byte BE metadata length + JSON + JPEG, SOI/EOI validated) |
| `transforms.ts` | Map↔canvas math (origin yaw, resolution, y-flip), `isFreeOccupancyCell` (254 = free), `staleAgeLabel` (Korean) |
| `mappingStages.ts` | `STAGE_INFO` for the 4 mapping stages + `SAFE_CAPS` (0.15 m/s / 0.5 rad/s, display-only) — **ids must stay in sync with backend `MappingService` STAGES and lap585** |
| `MapPanel.tsx` | SVG occupancy viewer: PNG + trails/paths/goals, scan/costmap overlays (200 ms poll), click-drag start/goal with yaw, free-cell validation, zoom/pan, navigate + AMCL reset with blocker reasons |
| `CameraGrid.tsx` / `CameraTile.tsx` | Camera grid: per-robot WS video tiles, blob object-URLs, FPS + stall watchdog (≥2 s banner, ≥5 s "영상 없음"), quality switch, backoff reconnect |
| `Teleop.tsx` | Manual drive: lease + MANUAL mode required; 10 Hz frames on `/ws/teleop`; sends zero-velocity stop on pointer up/blur/unmount/socket close; caps 0.15 m/s, 0.30 rad/s |
| `StopBar.tsx` | 전체/robot stop buttons (lease-less), 250 ms poll for CONFIRMED/UNCONFIRMED; copy states software-only |
| `FormationPanel.tsx` | Formation state + pair/pause/unpair/rejoin, gated by state machine + lease; delegates via `onAction` |
| `MissionPanel.tsx` | Mission builder: waypoints from map selections (cap 100), validation, create/validate/start/pause/resume/cancel, optimistic `version` on save, progress readout |
| `MappingPanel.tsx` | Mapping run control: create+start, validate/pause/resume/cancel; **no backend GET** — state refreshed optimistically via local `stateAfterAction` |
| `LineFollowPanel.tsx` | Line-follow start/stop (robot_1 only) with auto/white/black mode; surfaces FAILED/CAMERA_STALLED |
| `RecordPanel.tsx` | Dataset recording: robot + label, start/stop with lease, reads `GET /api/v1/recordings` for REC frame count |
| `HistoryPanel.tsx` | Event history: filters, cursor pagination ("더 보기"), JSON download (paged, 10k cap) |
| `SettingsPage.tsx` | Admin settings form (active map, follow distance, speed limits, camera quality) with optimistic `version`; initial-pose entry; read-only for non-ADMIN |
| `AlertList.tsx` | Active alerts + 확인 (ack) button |
| `MapHealth.tsx` | Presentational `<dl>` mirroring `pinky_mapcheck.health()` shape; `null` → placeholder |
| `PinkyCard.tsx` | Standalone robot status card (tested standalone; App inlines its own card markup) |
| `ControlDock.tsx` / `DashboardTabs.tsx` | Alternative dock / tab nav — tested standalone, currently not rendered by App |
| `styles.css` | Single global stylesheet (dark theme, CSS grid, 650/900 px breakpoints) |

## For AI Agents

### Working In This Directory
- **Mutation envelope:** every mutating call sends `credentials: 'include'`, `X-CSRF-Token` from the `cc_csrf` cookie, `request_id: crypto.randomUUID()`, and (control ops) `lease_id` — components raise "제어권이 필요합니다." without it.
- **Command-ack pattern:** mutations return `{command_id}`; await `waitForCommand(id)` polling until terminal state.
- Reconnect discipline: exponential backoff 1 s→8 s; teleop close paths must send a zero-velocity stop first (safety).
- Copy invariants: UI text is Korean; stop copy must say software-only ("소프트웨어 정지…물리 안전 장치가 아닙니다"), never "긴급 정지" (enforced by T05 tests).
- Optimistic concurrency: settings PUT / mission PATCH carry `version`.
- Keep `mappingStages.ts` stage ids in sync with backend `STAGES` (`../../backend/pinky_control_center/mapping_service.py`).

### Testing Requirements
- Run: `npx vitest run [path]` from `frontend/`; typecheck `npx tsc --noEmit`.
- Colocated `*.test.ts(x)`; import `@testing-library/jest-dom/vitest` per test (no setup file); globals off.
- Prefer `within(view.container)` queries (render binds to body); fake `fetch` via `vi.stubGlobal`, fake sockets via stubbed `WebSocket` classes, timers via `vi.useFakeTimers()`.
- `T05`–`T10.test.tsx` are acceptance-ticket suites mapping to functional-spec tickets; App itself is covered only through exported pure helpers (`AppError.test.ts`).

<!-- MANUAL: -->
