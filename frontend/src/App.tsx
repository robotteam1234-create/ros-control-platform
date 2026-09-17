import { FormEvent, useEffect, useState } from 'react'
import { acquireLease, formationAction, login, logout, navigateRobot, releaseLease, renewLease, resetLocalization, session, stateSocket, waitForCommand } from './api'
import type { Goal, UserSession } from './api'
import MapPanel from './MapPanel'
import CameraGrid from './CameraGrid'
import StopBar from './StopBar'
import Teleop from './Teleop'
import FormationPanel from './FormationPanel'
import MissionPanel from './MissionPanel'
import LineFollowPanel from './LineFollowPanel'
import RecordPanel from './RecordPanel'
import MappingPanel from './MappingPanel'
import AlertList from './AlertList'
import SettingsPage from './SettingsPage'
import HistoryPanel from './HistoryPanel'
import DashboardTabs, { DashboardTab } from './DashboardTabs'
import ControlDock from './ControlDock'
import PinkyCard from './PinkyCard'
import { setMode } from './control'

type Point = { x: number; y: number }
type Robot = { robot_id: string; name: string; role: 'MASTER' | 'SLAVE'; connection: string; mode: string; battery_percent: number | null; voltage_v?: number | null; linear_mps?: number | null; angular_rps?: number | null; pose?: { x: number; y: number; yaw: number } | null; pose_freshness?: string; stop_latched: boolean | null; capabilities?: string[]; trail?: Point[]; path?: Point[]; goal?: { x: number; y: number; yaw: number; frame_id: string } | null; tf_valid?: boolean; tf_reason_code?: string | null; received_at?: string | null }
type State = { robots: Robot[]; mode: string; seq: number; server_time: string; map_id?: string | null; active_mission?: null; active_alerts?: { alert_id: string; code: string; severity: string; state: string; message: string; occurrences: number; robot_id?: string | null; acknowledged_by?: string | null; acknowledged_at?: string | null }[]; formation?: { state: string; master_id?: string | null; slave_id?: string | null; distance_m: number | null; bearing_rad: number | null; reason_code?: string | null } }

async function getState(): Promise<State> {
  const response = await fetch('/api/v1/state')
  if (response.status === 401) throw new Error('SESSION_EXPIRED')
  if (!response.ok) throw new Error(`상태 요청 실패 (${response.status})`)
  return response.json() as Promise<State>
}

export function dashboardErrorMessage(message: string) {
  const connectionFailure = /failed to fetch|networkerror|network request failed|load failed/i.test(message)
  return connectionFailure
    ? '관제 백엔드에 연결할 수 없습니다. 127.0.0.1:8081의 backend와 Vite proxy 상태를 확인해 주세요.'
    : message
}

export function isRobotStill(linearMps: number | null | undefined, angularRps: number | null | undefined) {
  return linearMps != null && Math.abs(linearMps) <= 0.01 && angularRps != null && Math.abs(angularRps) <= 0.03
}

export default function App() {
  const [user, setUser] = useState<UserSession | null>(null)
  const [checking, setChecking] = useState(true)
  const [credentials, setCredentials] = useState({ username: '', password: '' })
  const [authError, setAuthError] = useState('')
  const [state, setState] = useState<State | null>(null)
  const [error, setError] = useState('')
  const [socketStatus, setSocketStatus] = useState('연결 대기')
  const [lease, setLease] = useState<{ lease_id: string; expires_at: string } | null>(null)
  const [leaseError, setLeaseError] = useState('')
  const [selectedRobot, setSelectedRobot] = useState('robot_1')
  const [goal, setGoal] = useState<Goal | null>(null)
  const [initialPose, setInitialPose] = useState<Goal | null>(null)
  const [mapResetVersion, setMapResetVersion] = useState(0)
  const [cameraQuality, setCameraQuality] = useState<'low' | 'default' | 'high'>('default')
  const [navigationBusy, setNavigationBusy] = useState(false)
  const [localizationBusy, setLocalizationBusy] = useState(false)
  const [navigationStatus, setNavigationStatus] = useState('')
  const [activeTab, setActiveTab] = useState<DashboardTab>(() => {
    const hash = window.location.hash.replace('#', '')
    return hash === 'drive' || hash === 'mission' || hash === 'camera' || hash === 'alerts' || hash === 'settings' ? hash as DashboardTab : 'drive'
  })
  const [leaseAcquiring, setLeaseAcquiring] = useState(false)
  useEffect(() => { window.location.hash = activeTab }, [activeTab])
  useEffect(() => { session().then(setUser).catch(e => setAuthError(e.message)).finally(() => setChecking(false)) }, [])
  useEffect(() => { if (!user) return; let active = true; const refresh = () => getState().then(value => { if (active) { setState(value); setError('') } }).catch(e => { if (!active) return; if (e.message === 'SESSION_EXPIRED') setUser(null); else setError(e.message) }); refresh(); const stopSocket = stateSocket(value => active && setState(value), status => { setSocketStatus(status); if (status === '세션 만료') setUser(null) }); const timer = setInterval(refresh, 10000); return () => { active = false; clearInterval(timer); stopSocket(); } }, [user])
  useEffect(() => { if (!lease) return; const timer = setInterval(() => renewLease(lease.lease_id).then(setLease).catch(() => setLease(null)), 1000); return () => clearInterval(timer) }, [lease?.lease_id])
  if (checking) return <main><p>세션 확인 중입니다…</p></main>
  if (!user) return <main><section className="login"><p className="eyebrow">PINKY PRO · CONTROL CENTER</p><h1>관제 로그인</h1><form onSubmit={(event: FormEvent) => { event.preventDefault(); setAuthError(''); login(credentials.username, credentials.password).then(setUser).catch(e => setAuthError(e.message)) }}><label>아이디<input autoComplete="username" value={credentials.username} onChange={e => setCredentials({ ...credentials, username: e.target.value })} required /></label><label>비밀번호<input type="password" autoComplete="current-password" value={credentials.password} onChange={e => setCredentials({ ...credentials, password: e.target.value })} required /></label><button type="submit">로그인</button></form>{authError && <div className="error">{authError}</div>}</section></main>
  const selected = state?.robots.find(robot => robot.robot_id === selectedRobot)
  const formationSafe = state?.formation?.state === 'UNPAIRED' || state?.formation?.state === 'STOPPED'
  const still = isRobotStill(selected?.linear_mps, selected?.angular_rps)
  const localizationBlockers = [
    !lease && '제어권이 없습니다.',
    !state?.map_id && '활성 지도가 없습니다.',
    !initialPose && '시작점을 지정하지 않았습니다.',
    selected?.connection !== 'ONLINE' && '선택 로봇이 ONLINE이 아닙니다.',
    selected?.mode !== 'IDLE' && selected?.mode !== 'STOPPED' && '선택 로봇이 정지 상태가 아닙니다.',
    !still && '선택 로봇의 속도가 0이 아닙니다.',
    !formationSafe && '편대를 먼저 정지하거나 해제해야 합니다.',
    state?.active_mission != null && '활성 임무를 먼저 종료해야 합니다.',
  ].filter((reason): reason is string => Boolean(reason))
  const localizationReady = localizationBlockers.length === 0
  const navigationBlockers = [
    ...localizationBlockers,
    selected?.pose_freshness !== 'FRESH' && '로봇 위치 데이터가 아직 신선하지 않습니다.',
    !selected?.tf_valid && 'AMCL map TF가 아직 유효하지 않습니다.',
    selected?.mode !== 'IDLE' && '선택 로봇이 IDLE 상태가 아닙니다.',
    selected?.stop_latched !== false && '선택 로봇의 정지 래치를 해제해야 합니다.',
    !selected?.capabilities?.includes('navigate') && '로봇에서 navigate 기능이 준비되지 않았습니다.',
  ].filter((reason): reason is string => Boolean(reason))
  const navigationReady = navigationBlockers.length === 0
  const cameraRobots = state?.robots.filter(robot => robot.capabilities?.includes('camera')) ?? []
  const runNavigation = async () => {
    if (!lease || !state?.map_id || !initialPose || !goal) return
    setError(''); setNavigationStatus('이동 요청 중…'); setNavigationBusy(true)
    try {
      const accepted = await navigateRobot(selectedRobot, state.map_id, initialPose, goal, lease.lease_id)
      await waitForCommand(accepted.command_id, 60, 250)
      setNavigationStatus('주행 요청이 로봇에 전달되었습니다. AUTO 상태와 계획 경로를 확인하세요.')
    } catch (reason) {
      setNavigationStatus(''); setError((reason as Error).message)
    } finally { setNavigationBusy(false) }
  }
  const runLocalizationReset = async () => {
    if (!lease || !state?.map_id || !initialPose) return
    setError(''); setNavigationStatus('초기 위치를 로봇에 전달하는 중…'); setLocalizationBusy(true)
    try {
      const accepted = await resetLocalization(selectedRobot, state.map_id, initialPose, lease.lease_id)
      await waitForCommand(accepted.command_id, 30, 200)
      setNavigationStatus('위치 재설정 완료 · 지도는 유지되며 AMCL 위치만 갱신되었습니다.')
    } catch (reason) {
      setNavigationStatus(''); setError((reason as Error).message)
    } finally { setLocalizationBusy(false) }
  }
  return <main>
    <header><div><p className="eyebrow">PINKY PRO · CONTROL CENTER</p><h1>2대 로봇 관제</h1></div><div className="userbar"><span>{user.username} · {user.role}</span><button onClick={() => logout().finally(() => { setUser(null); setState(null); setLease(null); setLeaseError('') })}>로그아웃</button></div></header>
    <ControlDock robots={state?.robots ?? []} selectedRobot={selectedRobot} onSelect={setSelectedRobot} leaseLabel={lease?.lease_id ?? null} onAcquire={() => { setLeaseAcquiring(true); acquireLease().then(setLease).catch(e => setLeaseError(e.message)).finally(() => setLeaseAcquiring(false)) }} onRelease={() => releaseLease(lease!.lease_id).finally(() => setLease(null))} leaseAcquiring={leaseAcquiring} role={user.role} socketStatus={socketStatus} />
    <DashboardTabs active={activeTab} onChange={setActiveTab} />
    {leaseError && <div className="error">{leaseError}</div>}
    {error && <div className="error">{dashboardErrorMessage(error)}</div>}
    {activeTab === 'drive' && (<><MapPanel robots={state?.robots ?? []} selected={selectedRobot} mapId={state?.map_id} onSelect={setSelectedRobot} onGoalChange={setGoal} initialPose={initialPose} onInitialPoseChange={setInitialPose} onResetSelections={() => { setMapResetVersion(value => value + 1); setNavigationStatus('') }} onNavigate={runNavigation} navigationReady={navigationReady} navigationBusy={navigationBusy} navigationBlockers={navigationBlockers} onResetLocalization={runLocalizationReset} localizationReady={localizationReady} localizationBusy={localizationBusy} localizationBlockers={localizationBlockers} navigationStatus={navigationStatus} /><section><div className="section-title"><h2>로봇 상태</h2><span>{state?.server_time ? new Date(state.server_time).toLocaleTimeString('ko-KR') : '—'}</span></div><div className="pinky-cards">{state?.robots.map(r => <PinkyCard key={r.robot_id} robot={r} selected={selectedRobot === r.robot_id} onSelect={() => setSelectedRobot(r.robot_id)} lease={lease?.lease_id ?? null} onMode={m => setMode(r.robot_id, m).catch(e => setError(e.message))} />) ?? <div className="empty">상태를 기다리는 중입니다.</div>}</div></section><StopBar role={user.role} lease={lease?.lease_id ?? null} selectedRobot={selectedRobot} /><Teleop robotId={selectedRobot} lease={lease?.lease_id ?? null} mode={state?.robots.find(r => r.robot_id === selectedRobot)?.mode ?? 'UNKNOWN'} stopLatched={state?.robots.find(r => r.robot_id === selectedRobot)?.stop_latched ?? null} /></>)}
    {activeTab === 'mission' && (<>{state?.formation && <p className="formation-readout">편대 거리: {state.formation.distance_m == null ? '— (TF 확인 필요)' : `${state.formation.distance_m.toFixed(2)}m`} · 방위각: {state.formation.bearing_rad == null ? '—' : `${state.formation.bearing_rad.toFixed(2)}rad`}</p>}<FormationPanel formation={state?.formation} lease={lease?.lease_id ?? null} onAction={async action => { setError(''); try { await formationAction(action, state?.formation?.master_id ?? 'robot_1', state?.formation?.slave_id ?? 'robot_2', lease?.lease_id ?? '') } catch (reason) { setError((reason as Error).message) } }} /><MissionPanel lease={lease?.lease_id ?? null} mapId={state?.map_id} formation={state?.formation} goal={goal} resetVersion={mapResetVersion} onError={setError} /><LineFollowPanel lease={lease?.lease_id ?? null} onError={setError} /><RecordPanel lease={lease?.lease_id ?? null} onError={setError} /><MappingPanel lease={lease?.lease_id ?? null} mapId={state?.map_id} onError={setError} /></>)}
    {activeTab === 'camera' && (cameraRobots.length > 0 ? <CameraGrid robots={cameraRobots} initialQuality={cameraQuality} /> : <div className="empty">카메라 로봇이 없습니다.</div>)}
    {activeTab === 'alerts' && (<><AlertList alerts={state?.active_alerts ?? []} onError={setError} /><HistoryPanel robots={state?.robots ?? []} onError={setError} /></>)}
    {activeTab === 'settings' && (<SettingsPage role={user.role} robots={state?.robots ?? []} selectedRobot={selectedRobot} initialPose={initialPose} poseResetVersion={mapResetVersion} onError={setError} onLoaded={value => setCameraQuality(value.camera_quality)} onSaved={value => { setGoal(null); setCameraQuality(value.camera_quality); setState(current => current ? { ...current, map_id: value.active_map_id } : current) }} />)}
  </main>
}
