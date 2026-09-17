# Pinky dashboard tabs + control dock design

Date: 2026-09-17 · Status: approved design (architectural path) · Scope: frontend only, no backend/API change

## 1. Goal

Current dashboard (`frontend/src/App.tsx:106-123`) renders one long vertical stack:
Map → Settings → Alerts → History → Formation → Mission → Robot cards → Stop → Camera → Teleop.
Stop and Teleop are buried, robot select is far from controls, formation+mission flow is split.

Success: operator can drive formation+mission tasks without scrolling to find Stop/Teleop/robot state.
Safety copy stays software-only, `robot_1` MASTER / `robot_2` SLAVE invariants unchanged.

## 2. Architecture

No backend change. `App.tsx` keeps all state (user, lease, state, selectedRobot, goal, initialPose, mapResetVersion, cameraQuality, navigation/localization busy).
New composition only:

```
App
 ├─ ControlDock (sticky top, always mounted)
 │   ├─ robot selector pills (robot_1/robot_2, role badge, connection dot)
 │   ├─ lease acquire/release
 │   └─ compact stop: 전체 정지 / 선택 로봇 정지 / 해제 (reuse control.ts stop/resetStop)
 ├─ DashboardTabs (hash-routed: #drive #mission #camera #alerts #settings)
 │   ├─ Drive: MapPanel + PinkyCard list + Teleop + single-robot navigate/AMCL (existing MapPanel props)
 │   ├─ Mission: FormationPanel + formation readout + MissionPanel + MappingPanel
 │   ├─ Camera: CameraGrid
 │   ├─ Alerts: AlertList + HistoryPanel
 │   └─ Settings: SettingsPage
 └─ error / socket status (existing)
```

Hash routing: `#drive` default. `window.location.hash` sync, refresh preserves tab. No react-router dependency.

## 3. Components

- `frontend/src/ControlDock.tsx` (new):
  Props: robots, selectedRobot, onSelect, lease, onAcquire, onRelease, role, socketStatus.
  Compact stop buttons call existing `stop('all'|'robot_1'|'robot_2')` + `resetStop(target, lease)` with CONFIRMED/UNCONFIRMED polling reused from `StopBar.tsx`. Full `StopBar` remains in Drive tab for detailed targets list; dock shows condensed result.
  Sticky CSS: `position: sticky; top: 0; z-index: 20`.

- `frontend/src/DashboardTabs.tsx` (new):
  Props: active, onChange. Buttons with `role=tab`, `aria-selected`. Horizontal scroll on mobile.

- `frontend/src/PinkyCard.tsx` (new, extracted from App.tsx:119 inline cards):
  Props: robot, selected, onSelect. Shows 연결/모드/배터리·전압/속도/위치신선도/TF/capabilities badges (`navigate`, `camera`), stop latch badge, mode switch button (`setMode(robot_id,'MANUAL'|'IDLE')` via `control.ts`, lease-gated).
  Uses `odom_*` fields note: display `linear_mps` but tooltip clarifies last-writer-wins per AGENTS.md.

- `Teleop.tsx` (extend, not rewrite):
  Keep 10 Hz WS logic. Add 후진 (`-0.10`), 우회전 (`-0.30`), 정지 (send 0) buttons. Props unchanged. Keyboard: optional ArrowUp/Left/Right hold (only when MANUAL + lease, blur stops).

- `App.tsx` (refactor):
  Remove long stack, render `<ControlDock/>` + `<DashboardTabs/>` + conditional tab panels. All existing handlers (`runNavigation`, `runLocalizationReset`, formation `onAction`, `onSaved`) unchanged. `cameraRobots` filter unchanged.

- `styles.css` (extend):
  `.dock`, `.tabs`, `.tab.active`, `.pinky-cards` grid. Reuse `.card`, `.pill`, `.stop-bar` tokens. Mobile: dock wraps, tabs scroll-x, Drive stacks Map→cards→teleop.

## 4. Data flow

1. Login → session + `getState` poll (10 s) + `stateSocket` unchanged.
2. Operator picks tab (hash change only, no fetch).
3. Drive: select robot in dock or card or map marker (all call `setSelectedRobot`, synced). Map drag sets `initialPose`/`goal`, navigate/AMCL buttons use existing lease-gated `navigateRobot`/`resetLocalization` + `waitForCommand`.
4. Mission: `FormationPanel pair → READY` then `MissionPanel create → validate → start`. Goal from Drive tab shared via `goal` state (cross-tab banner: "Drive 탭에서 지정된 waypoint N개").
5. Stop: dock or Drive `StopBar` → `stop()` → poll `command()` for CONFIRMED. `resetStop` needs lease, no auto-resume.
6. Teleop: MANUAL mode + lease + hold button → `/ws/teleop` 10 Hz; release/blur/visibilitychange → 0.

## 5. Error handling

- Existing `dashboardErrorMessage`, `SESSION_EXPIRED`, lease 409, CSRF/Origin errors unchanged.
- Blockers text (`localizationBlockers`, `navigationBlockers`) unchanged, shown in Drive tab near buttons.
- Dock stop errors show inline `<em>` same as StopBar, never hide UNCONFIRMED.
- Hash unknown → fallback `#drive`.

## 6. Testing

- New `frontend/src/DashboardTabs.test.tsx`: tab switching changes visible panel, hash preserved, dock always visible.
- New `frontend/src/PinkyCard.test.tsx`: select callback, MANUAL switch disabled without lease / when stopLatched, capability badges.
- Extend `T06.test.tsx`/`T07.test.tsx` if selectors move (use `within(view.container)` per AGENTS.md).
- `npx vitest run`, `npx tsc --noEmit`. No backend tests (no backend change).
- TDD: RED test for tabs/dock first, GREEN minimal, full suite before commit.

## 7. Non-goals

- No backend, API, ROS, rosbridge, watchdog, DB, auth change.
- No domain ID change (12/13 fixed), no robot register/role change.
- No video record (T11 excluded), no SLAM rebuild, map file untouched.
- No react-router, no state library, no design system migration.
