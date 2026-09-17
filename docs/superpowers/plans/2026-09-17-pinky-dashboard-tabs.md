# Pinky Dashboard Tabs Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reorganize frontend dashboard into tabbed workspace with sticky control dock so formation+mission driving no longer requires scrolling.

**Architecture:** Frontend-only composition change. `App.tsx` keeps all state and handlers; new `DashboardTabs`, `ControlDock`, `PinkyCard` components handle presentation; `Teleop` gains 3 buttons; `styles.css` gains dock/tabs styles. Hash routing preserves tab.

**Tech Stack:** React 18 + TypeScript 5.7, Vite 5, Vitest 2 + Testing Library, existing `control.ts` (`stop`, `resetStop`, `setMode`, `command`) and `api.ts` contracts.

**Spec:** `docs/superpowers/specs/2026-09-17-pinky-dashboard-tabs-design.md`

## Global Constraints

- Browser never touches ROS; backend holds rosbridge websockets (`ws://127.0.0.1:9090` d12, `:9091` d13).
- `robot_1` MASTER domain 12, `robot_2` SLAVE domain 13 — never change from UI.
- Stop UI copy must say software-only, never "emergency".
- `CONTROL_PLATFORM_WORKERS=1` enforced; no backend change in this plan.
- All mutations need `request_id` + lease + `X-CSRF-Token` + `Origin` (already in `control.ts`).
- TDD: failing test first (RED), minimal implement (GREEN), full suite before commit.
- Frontend tests use `within(view.container)` queries (render binds to body).
- Commits: `git -c user.name="opencode" -c user.email="opencode@localhost" commit`.
- Cameras may 503 `CAMERA_STALLED` when `camera.enabled=false`; not an error in this plan.

---

### Task 1: DashboardTabs component

**Files:**
- Create: `frontend/src/DashboardTabs.tsx`
- Test: `frontend/src/DashboardTabs.test.tsx`

**Interfaces:**
- Consumes: none.
- Produces: `export type DashboardTab = 'drive' | 'mission' | 'camera' | 'alerts' | 'settings'`; `export default function DashboardTabs({ active, onChange }: { active: DashboardTab; onChange: (tab: DashboardTab) => void })`.

- [ ] **Step 1: Write the failing test**

```tsx
import { cleanup, fireEvent, render } from '@testing-library/react'
import '@testing-library/jest-dom/vitest'
import { afterEach, expect, test, vi } from 'vitest'
import { within } from '@testing-library/react'
import DashboardTabs from './DashboardTabs'

afterEach(() => cleanup())

test('tabs switch drive to mission', () => {
  const onChange = vi.fn()
  const view = render(<DashboardTabs active="drive" onChange={onChange} />)
  const root = within(view.container)
  expect(root.getByRole('tab', { name: /Drive/ })).toHaveAttribute('aria-selected', 'true')
  fireEvent.click(root.getByRole('tab', { name: /편대/ }))
  expect(onChange).toHaveBeenCalledWith('mission')
})
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd frontend && npx vitest run src/DashboardTabs.test.tsx`
Expected: FAIL with "Failed to resolve import ./DashboardTabs" or "Cannot find module".

- [ ] **Step 3: Write minimal implementation**

```tsx
export type DashboardTab = 'drive' | 'mission' | 'camera' | 'alerts' | 'settings'

const TABS: { id: DashboardTab; label: string }[] = [
  { id: 'drive', label: '관제 Drive' },
  { id: 'mission', label: '편대·임무' },
  { id: 'camera', label: '카메라' },
  { id: 'alerts', label: '알림·이력' },
  { id: 'settings', label: '설정' },
]

export default function DashboardTabs({ active, onChange }: { active: DashboardTab; onChange: (tab: DashboardTab) => void }) {
  return (
    <nav className="tabs" role="tablist" aria-label="관제 섹션">
      {TABS.map(tab => (
        <button key={tab.id} role="tab" aria-selected={active === tab.id} className={active === tab.id ? 'tab active' : 'tab'} onClick={() => onChange(tab.id)}>
          {tab.label}
        </button>
      ))}
    </nav>
  )
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd frontend && npx vitest run src/DashboardTabs.test.tsx`
Expected: PASS (1 passed).

- [ ] **Step 5: Commit**

```bash
git add frontend/src/DashboardTabs.tsx frontend/src/DashboardTabs.test.tsx
git -c user.name="opencode" -c user.email="opencode@localhost" commit -m "feat(frontend): add dashboard tabs"
```

### Task 2: ControlDock component (sticky safety dock)

**Files:**
- Create: `frontend/src/ControlDock.tsx`
- Test: `frontend/src/ControlDock.test.tsx`

**Interfaces:**
- Consumes: `control.ts: stop(target)`, `resetStop(target, lease)`, `command(commandId)` for CONFIRMED polling (copy polling loop from `StopBar.tsx:1`).
- Produces: `export default function ControlDock({ robots, selectedRobot, onSelect, leaseLabel, onAcquire, onRelease, leaseAcquiring, role, socketStatus }: {...})` where `robots: { robot_id: string; role: string; connection: string }[]`, `leaseLabel: string | null`.

- [ ] **Step 1: Write the failing test**

```tsx
import { cleanup, fireEvent, render } from '@testing-library/react'
import { within } from '@testing-library/react'
import '@testing-library/jest-dom/vitest'
import { afterEach, expect, test, vi } from 'vitest'
import ControlDock from './ControlDock'

afterEach(() => { cleanup(); vi.restoreAllMocks(); vi.unstubAllGlobals() })

test('dock selects robot and exposes stop without lease for operator', () => {
  vi.stubGlobal('fetch', vi.fn().mockResolvedValue(new Response(JSON.stringify({ command_id: 'c1' }), { status: 200 })))
  const onSelect = vi.fn()
  const view = render(<ControlDock robots={[{ robot_id: 'robot_1', role: 'MASTER', connection: 'ONLINE' }, { robot_id: 'robot_2', role: 'SLAVE', connection: 'ONLINE' }]} selectedRobot="robot_1" onSelect={onSelect} leaseLabel={null} onAcquire={() => {}} onRelease={() => {}} leaseAcquiring={false} role="OPERATOR" socketStatus="실시간 연결" />)
  const root = within(view.container)
  fireEvent.click(root.getByRole('button', { name: /robot_2/ }))
  expect(onSelect).toHaveBeenCalledWith('robot_2')
  expect(root.getByRole('button', { name: /전체 정지/ })).toBeEnabled()
})
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd frontend && npx vitest run src/ControlDock.test.tsx`
Expected: FAIL — module not found.

- [ ] **Step 3: Write minimal implementation**

```tsx
import { useState } from 'react'
import { resetStop, stop } from './control'

type DockRobot = { robot_id: string; role: string; connection: string }

export default function ControlDock({ robots, selectedRobot, onSelect, leaseLabel, onAcquire, onRelease, leaseAcquiring, role, socketStatus }: {
  robots: DockRobot[]
  selectedRobot: string
  onSelect: (id: string) => void
  leaseLabel: string | null
  onAcquire: () => void
  onRelease: () => void
  leaseAcquiring: boolean
  role: string
  socketStatus: string
}) {
  const [result, setResult] = useState('')
  const [error, setError] = useState('')
  const run = (target: 'all' | 'robot_1' | 'robot_2') => {
    setError('')
    stop(target).then(() => setResult('정지 접수됨 · 소프트웨어 정지')).catch(e => setError((e as Error).message))
  }
  const resetTarget = (selectedRobot === 'robot_1' || selectedRobot === 'robot_2' ? selectedRobot : 'all') as 'all' | 'robot_1' | 'robot_2'
  return (
    <div className="dock" role="region" aria-label="제어 독">
      <div className="dock-robots">
        {robots.map(r => (
          <button key={r.robot_id} aria-pressed={selectedRobot === r.robot_id} className={selectedRobot === r.robot_id ? 'pill online' : 'pill'} onClick={() => onSelect(r.robot_id)}>
            {r.robot_id} · {r.role} · {r.connection}
          </button>
        ))}
      </div>
      <div className="dock-lease">
        <span className="pill">{socketStatus}</span>
        {leaseLabel
          ? <><span className="pill online">제어권 활성</span><button disabled={leaseAcquiring} onClick={onRelease}>반납</button></>
          : <button disabled={leaseAcquiring} onClick={onAcquire}>제어권 획득</button>}
      </div>
      <div className="dock-stop">
        <strong>정지</strong>
        <span>소프트웨어 정지이며 물리 안전 장치가 아닙니다.</span>
        <button disabled={role === 'VIEWER'} onClick={() => run('all')}>전체 정지</button>
        <button disabled={role === 'VIEWER'} onClick={() => run(resetTarget)}>{resetTarget} 정지</button>
        {leaseLabel && <button className="reset" onClick={() => resetStop(resetTarget, leaseLabel).then(() => setResult(`${resetTarget} 해제 접수됨 · 자동 주행 없음`)).catch(e => setError((e as Error).message))}>{resetTarget} 정지 해제</button>}
        {result && <b>{result}</b>}
        {error && <em>{error}</em>}
      </div>
    </div>
  )
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd frontend && npx vitest run src/ControlDock.test.tsx`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add frontend/src/ControlDock.tsx frontend/src/ControlDock.test.tsx
git -c user.name="opencode" -c user.email="opencode@localhost" commit -m "feat(frontend): add sticky control dock"
```

### Task 3: PinkyCard extraction + Teleop extra buttons

**Files:**
- Create: `frontend/src/PinkyCard.tsx`
- Modify: `frontend/src/Teleop.tsx:102-111`
- Test: `frontend/src/PinkyCard.test.tsx`

**Interfaces:**
- Consumes: `control.ts: setMode(robotId, mode)`.
- Produces: `export default function PinkyCard({ robot, selected, onSelect, lease, onMode }: { robot: RobotCard; selected: boolean; onSelect: () => void; lease: string | null; onMode: (mode: 'MANUAL' | 'IDLE') => void })` with `RobotCard = { robot_id: string; name: string; role: 'MASTER' | 'SLAVE'; connection: string; mode: string; battery_percent: number | null; voltage_v?: number | null; linear_mps?: number | null; angular_rps?: number | null; pose_freshness?: string; stop_latched: boolean | null; capabilities?: string[]; tf_valid?: boolean }`.

- [ ] **Step 1: Write the failing test**

```tsx
import { cleanup, fireEvent, render } from '@testing-library/react'
import { within } from '@testing-library/react'
import '@testing-library/jest-dom/vitest'
import { afterEach, expect, test, vi } from 'vitest'
import PinkyCard from './PinkyCard'

afterEach(() => cleanup())

test('pinky card shows navigate badge and mode switch needs lease', () => {
  const view = render(<PinkyCard robot={{ robot_id: 'robot_2', name: 'Pinky 2', role: 'SLAVE', connection: 'ONLINE', mode: 'IDLE', battery_percent: 80, linear_mps: 0, angular_rps: 0, pose_freshness: 'FRESH', stop_latched: false, capabilities: ['navigate'], tf_valid: true }} selected={true} onSelect={() => {}} lease={null} onMode={() => {}} />)
  const root = within(view.container)
  expect(root.getByText(/navigate/)).toBeInTheDocument()
  expect(root.getByRole('button', { name: /MANUAL/ })).toBeDisabled()
})
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd frontend && npx vitest run src/PinkyCard.test.tsx`
Expected: FAIL — module not found.

- [ ] **Step 3: Write minimal PinkyCard implementation**

```tsx
export type RobotCard = { robot_id: string; name: string; role: 'MASTER' | 'SLAVE'; connection: string; mode: string; battery_percent: number | null; voltage_v?: number | null; linear_mps?: number | null; angular_rps?: number | null; pose_freshness?: string; stop_latched: boolean | null; capabilities?: string[]; tf_valid?: boolean }

export default function PinkyCard({ robot, selected, onSelect, lease, onMode }: { robot: RobotCard; selected: boolean; onSelect: () => void; lease: string | null; onMode: (mode: 'MANUAL' | 'IDLE') => void }) {
  return (
    <article className={`card ${selected ? 'selected-card' : ''}`} onClick={onSelect}>
      <div className="card-head"><div><h3>{robot.name}</h3><small>{robot.robot_id}</small></div><b className={robot.role.toLowerCase()}>{robot.role}</b></div>
      <dl>
        <div><dt>연결</dt><dd className="connection">{robot.connection}</dd></div>
        <div><dt>모드</dt><dd>{robot.mode}</dd></div>
        <div><dt>배터리 / 전압</dt><dd>{robot.battery_percent == null ? '—' : `${robot.battery_percent}%`} · {robot.voltage_v == null ? '—' : `${robot.voltage_v}V`}</dd></div>
        <div><dt>속도</dt><dd>{robot.linear_mps == null ? '—' : `${robot.linear_mps}m/s`} · {robot.angular_rps == null ? '—' : `${robot.angular_rps}rad/s`}</dd></div>
        <div><dt>위치 신선도</dt><dd>{robot.pose_freshness ?? 'UNKNOWN'}</dd></div>
        <div><dt>기능</dt><dd>{robot.capabilities?.includes('navigate') ? 'navigate ✓' : 'navigate —'} · TF {robot.tf_valid ? 'OK' : '지연'}</dd></div>
      </dl>
      {robot.stop_latched && <div className="stop">정지 래치</div>}
      <button disabled={!lease || robot.mode === 'MANUAL' || robot.stop_latched === true} onClick={e => { e.stopPropagation(); onMode('MANUAL') }}>MANUAL 모드</button>
      <button disabled={!lease || robot.mode === 'IDLE'} onClick={e => { e.stopPropagation(); onMode('IDLE') }}>IDLE 모드</button>
    </article>
  )
}
```

- [ ] **Step 4: Extend Teleop with 후진/우회전/정지 (edit `frontend/src/Teleop.tsx`)**

Keep existing `MANUAL_LINEAR_MPS = 0.15`, `MANUAL_TURN_RPS = 0.30`. After the existing 전진/좌회전 buttons add:

```tsx
<button disabled={!lease || mode !== 'MANUAL'} onPointerDown={() => hold(-MANUAL_LINEAR_MPS, 0)} onPointerUp={stop} onPointerLeave={stop} onPointerCancel={stop}>후진</button>
<button disabled={!lease || mode !== 'MANUAL'} onPointerDown={() => hold(0, -MANUAL_TURN_RPS)} onPointerUp={stop} onPointerLeave={stop} onPointerCancel={stop}>우회전</button>
<button disabled={!lease || mode !== 'MANUAL'} onClick={stop}>정지</button>
```

No logic change; `hold`/`stop` reused.

- [ ] **Step 5: Run tests**

Run: `cd frontend && npx vitest run src/PinkyCard.test.tsx src/T05.test.tsx`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add frontend/src/PinkyCard.tsx frontend/src/PinkyCard.test.tsx frontend/src/Teleop.tsx
git -c user.name="opencode" -c user.email="opencode@localhost" commit -m "feat(frontend): add pinky card and teleop directions"
```

### Task 4: App refactor to tabs + styles + verify

**Files:**
- Modify: `frontend/src/App.tsx:106-123`
- Modify: `frontend/src/styles.css:1-20`
- Test: existing `frontend/src/T06.test.tsx`, `T07.test.tsx`, plus new tab integration check via `DashboardTabs.test.tsx` + `ControlDock.test.tsx`.

**Interfaces:**
- Consumes: Tasks 1–3 components; existing `MapPanel`, `FormationPanel`, `MissionPanel`, `MappingPanel`, `AlertList`, `HistoryPanel`, `SettingsPage`, `CameraGrid`, `StopBar`, `Teleop` props unchanged.
- Produces: hash-routed tab state `activeTab` persisted in `window.location.hash`.

- [ ] **Step 1: Write the failing integration expectation (no new file — run existing suite to show App still renders old stack)**

Run: `cd frontend && npx vitest run src/DashboardTabs.test.tsx src/ControlDock.test.tsx`
Expected: PASS (components exist). Then manually confirm `App.tsx` does NOT yet import them:

Run: `rg "DashboardTabs|ControlDock" frontend/src/App.tsx`
Expected: no output (RED — not wired).

- [ ] **Step 2: Refactor App.tsx**

1. Add imports: `DashboardTabs, { DashboardTab }`, `ControlDock`, `PinkyCard`, `setMode` from `./control`.
2. Add state: `const [activeTab, setActiveTab] = useState<DashboardTab>(() => (window.location.hash.replace('#', '') as DashboardTab) || 'drive')` + `const [leaseAcquiring, setLeaseAcquiring] = useState(false)` + `useEffect` to sync `window.location.hash = activeTab` on change.
3. Replace `<div className="session-status">` lease buttons with `<ControlDock robots={state?.robots ?? []} selectedRobot={selectedRobot} onSelect={setSelectedRobot} leaseLabel={lease?.lease_id ?? null} onAcquire={() => { setLeaseAcquiring(true); acquireLease().then(setLease).catch(e => setLeaseError(e.message)).finally(() => setLeaseAcquiring(false)) }} onRelease={() => releaseLease(lease!.lease_id).finally(() => setLease(null))} leaseAcquiring={leaseAcquiring} role={user.role} socketStatus={socketStatus} />` then `<DashboardTabs active={activeTab} onChange={setActiveTab} />`.
4. Conditional render: `{activeTab === 'drive' && (<><MapPanel ...unchanged props /><section>...PinkyCard grid: {state?.robots.map(r => <PinkyCard key={r.robot_id} robot={r} selected={...} onSelect={() => setSelectedRobot(r.robot_id)} lease={lease?.lease_id ?? null} onMode={m => setMode(r.robot_id, m).catch(e => setError(e.message))} />)}</section><StopBar .../><Teleop .../></>)}`, `{activeTab === 'mission' && (<><FormationPanel .../><MissionPanel .../><MappingPanel .../></>)}`, `{activeTab === 'camera' && (camera)}`, `{activeTab === 'alerts' && (<><AlertList .../><HistoryPanel .../></>)}`, `{activeTab === 'settings' && (<SettingsPage .../>)}`.
5. Keep `formation-readout` inside mission tab. Keep error/auth/header unchanged.

- [ ] **Step 3: Add styles to `frontend/src/styles.css`**

```css
.dock { position: sticky; top: 0; z-index: 20; background: #151a23; border: 1px solid #2a3444; border-radius: 12px; padding: 12px 14px; margin-bottom: 16px; display: grid; gap: 10px }
.dock-robots, .dock-lease, .dock-stop { display: flex; gap: 8px; align-items: center; flex-wrap: wrap }
.dock-stop span { color: #b8a5a5; font-size: 12px }
.tabs { display: flex; gap: 8px; overflow-x: auto; margin-bottom: 20px }
.tab { background: #2a3444; font-size: 13px; white-space: nowrap }
.tab.active { background: #3d64c5 }
.pinky-cards { display: grid; grid-template-columns: repeat(2, 1fr); gap: 16px; margin: 16px 0 }
@media (max-width: 650px) { .pinky-cards { grid-template-columns: 1fr } }
```

- [ ] **Step 4: Run full frontend verification**

Run: `cd frontend && npx vitest run`
Expected: PASS (all suites including T05/T06/T07/T08/T10).

Run: `cd frontend && npx tsc --noEmit`
Expected: no errors.

Run: `cd frontend && npm run build`
Expected: `dist/` built successfully.

- [ ] **Step 5: Commit**

```bash
git add frontend/src/App.tsx frontend/src/styles.css
git -c user.name="opencode" -c user.email="opencode@localhost" commit -m "feat(frontend): tabbed dashboard with control dock"
```
