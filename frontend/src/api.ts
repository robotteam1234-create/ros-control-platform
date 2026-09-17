export type UserSession = { user_id?: string; username: string; role: string }
export type MapMetadata = { map_id: string; name: string; frame_id: string; resolution: number; width: number; height: number; origin: { x: number; y: number; yaw: number }; version: string; data_url: string }
export type Goal = { x: number; y: number; yaw: number; frame_id: string }
export type Formation = { state: string; master_id?: string | null; slave_id?: string | null; distance_m?: number | null; bearing_rad?: number | null; reason_code?: string | null }
export type Mission = { mission_id: string; state: string; failure_code?: string | null; progress_distance_m?: number | null; waypoint_index?: number; lap_index?: number; total_distance_m?: number | null; waypoints?: Goal[]; name?: string; repeat_count?: number; version?: number }
export type MapSummary = { map_id: string; name: string; version: string }
export type ActiveSettings = { version: number; active_map_id: string; follow_distance_m: number; follow_tolerance_m: number; max_linear_mps: number; max_angular_rps: number; camera_quality: 'low' | 'default' | 'high' }
export type HistoryEvent = { event_id: number; event_type: string; robot_id?: string | null; mission_id?: string | null; occurred_at: string; payload: Record<string, unknown> }
export type HistoryFilters = { event_type?: string; robot_id?: string; mission_id?: string; from?: string; to?: string }

async function errorMessage(response: Response, fallback: string) {
  try { const body = await response.json(); return body?.error?.message ?? body?.detail ?? fallback } catch { return fallback }
}

function csrf() {
  return document.cookie.split('; ').find(value => value.startsWith('cc_csrf='))?.split('=')[1] ?? ''
}

export async function session(): Promise<UserSession | null> {
  const response = await fetch('/api/v1/session', { credentials: 'include' })
  if (response.status === 401) return null
  if (!response.ok) throw new Error(await errorMessage(response, `세션 조회 실패 (${response.status})`))
  const value = await response.json()
  return value.user ?? value
}

export async function login(username: string, password: string): Promise<UserSession> {
  const response = await fetch('/api/v1/session', { method: 'POST', credentials: 'include', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ username, password }) })
  if (response.status === 401) throw new Error(await errorMessage(response, '아이디 또는 비밀번호가 올바르지 않습니다.'))
  if (!response.ok) throw new Error(await errorMessage(response, `로그인 실패 (${response.status})`))
  const value = await response.json()
  return value.user ?? value
}

export async function mapMetadata(mapId: string): Promise<MapMetadata> {
  const response = await fetch(`/api/v1/maps/${mapId}`, { credentials: 'include' })
  if (!response.ok) throw new Error(await errorMessage(response, `지도 조회 실패 (${response.status})`))
  return response.json() as Promise<MapMetadata>
}

export async function listMaps(): Promise<MapSummary[]> {
  const response = await fetch('/api/v1/maps', { credentials: 'include' })
  if (!response.ok) throw new Error(await errorMessage(response, `지도 목록 조회 실패 (${response.status})`))
  return ((await response.json()) as { items?: MapSummary[] }).items ?? []
}

export async function getSettings(): Promise<ActiveSettings> {
  const response = await fetch('/api/v1/settings', { credentials: 'include' })
  if (!response.ok) throw new Error(await errorMessage(response, `설정 조회 실패 (${response.status})`))
  return response.json() as Promise<ActiveSettings>
}

export async function updateSettings(values: ActiveSettings): Promise<ActiveSettings> {
  const response = await fetch('/api/v1/settings', { method: 'PUT', credentials: 'include', headers: { 'Content-Type': 'application/json', 'X-CSRF-Token': csrf() }, body: JSON.stringify({ request_id: crypto.randomUUID(), ...values }) })
  if (!response.ok) throw new Error(await errorMessage(response, `설정 저장 실패 (${response.status})`))
  return response.json() as Promise<ActiveSettings>
}

export async function setInitialPose(robotId: string, pose: Goal): Promise<{ command_id: string }> {
  const response = await fetch(`/api/v1/robots/${robotId}/initial-pose`, { method: 'POST', credentials: 'include', headers: { 'Content-Type': 'application/json', 'X-CSRF-Token': csrf() }, body: JSON.stringify({ request_id: crypto.randomUUID(), pose }) })
  if (!response.ok) throw new Error(await errorMessage(response, `초기 위치 적용 실패 (${response.status})`))
  return response.json() as Promise<{ command_id: string }>
}

export async function logout() {
  await fetch('/api/v1/session', { method: 'DELETE', credentials: 'include', headers: { 'X-CSRF-Token': csrf() } })
}

export async function acquireLease() {
  const response = await fetch('/api/v1/control-lease', { method: 'POST', credentials: 'include', headers: { 'Content-Type': 'application/json', 'X-CSRF-Token': csrf() }, body: JSON.stringify({ request_id: crypto.randomUUID() }) })
  if (response.status === 409) throw new Error('다른 사용자가 제어권을 사용 중입니다.')
  if (!response.ok) throw new Error(await errorMessage(response, `제어권 획득 실패 (${response.status})`))
  return response.json() as Promise<{ lease_id: string; expires_at: string }>
}

export async function releaseLease(leaseId: string) {
  await fetch(`/api/v1/control-lease/${leaseId}`, { method: 'DELETE', credentials: 'include', headers: { 'X-CSRF-Token': csrf(), 'Content-Type': 'application/json' }, body: JSON.stringify({ request_id: crypto.randomUUID() }) })
}
export async function renewLease(leaseId: string) {
  const response = await fetch(`/api/v1/control-lease/${leaseId}`, { method: 'PATCH', credentials: 'include', headers: { 'X-CSRF-Token': csrf(), 'Content-Type': 'application/json' }, body: JSON.stringify({ request_id: crypto.randomUUID() }) })
  if (!response.ok) throw new Error(await errorMessage(response, '제어권 갱신 실패'))
  return response.json() as Promise<{ lease_id: string; expires_at: string }>
}

async function mutation(path: string, body: Record<string, unknown>, fallback: string) {
  const response = await fetch(path, { method: 'POST', credentials: 'include', headers: { 'Content-Type': 'application/json', 'X-CSRF-Token': csrf() }, body: JSON.stringify({ request_id: crypto.randomUUID(), ...body }) })
  if (!response.ok) throw new Error(await errorMessage(response, `${fallback} (${response.status})`))
  return response.json()
}

export async function navigateRobot(robotId: string, mapId: string, startPose: Goal, goal: Goal, leaseId: string): Promise<{ command_id: string; navigation_id: string }> {
  if (!leaseId) throw new Error('제어권이 필요합니다.')
  return mutation(`/api/v1/robots/${robotId}/navigate`, { map_id: mapId, start_pose: startPose, goal, lease_id: leaseId }, '이동 요청 실패') as Promise<{ command_id: string; navigation_id: string }>
}

export async function resetLocalization(robotId: string, mapId: string, pose: Goal, leaseId: string): Promise<{ command_id: string }> {
  if (!leaseId) throw new Error('제어권이 필요합니다.')
  return mutation(`/api/v1/robots/${robotId}/localization-reset`, { map_id: mapId, pose, lease_id: leaseId }, '위치 재설정 요청 실패') as Promise<{ command_id: string }>
}

export async function formationAction(action: 'pair' | 'pause' | 'unpair' | 'rejoin', master_id: string, slave_id: string, lease_id: string) {
  if (!lease_id) throw new Error('제어권이 필요합니다.')
  return mutation('/api/v1/formation/actions', { action, master_id, slave_id, lease_id }, '편대 요청 실패')
}

export async function createMission(goal: Goal | Goal[], map_id: string, lease_id: string, name = '단일 목표 임무', repeat_count = 1): Promise<Mission> {
  if (!lease_id) throw new Error('제어권이 필요합니다.')
  return mutation('/api/v1/missions', { name, map_id, waypoints: Array.isArray(goal) ? goal : [goal], repeat_count, lease_id }, '임무 생성 실패') as Promise<Mission>
}

export async function missionAction(missionId: string, action: 'validate' | 'start' | 'pause' | 'resume' | 'cancel', lease_id: string) {
  if (!lease_id) throw new Error('제어권이 필요합니다.')
  return mutation(`/api/v1/missions/${missionId}/actions`, { action, lease_id }, '임무 요청 실패')
}

export type Mapping = { mapping_id: string; state: string; name?: string; stages?: string[]; stage_index?: number }

export async function createMapping(robotId: string, mapId: string, lease: string, name: string, stages?: string[]) {
  if (!lease) throw new Error('제어권이 필요합니다.')
  return mutation('/api/v1/mappings', { robot_id: robotId, map_id: mapId, lease_id: lease, name, ...(stages ? { stages } : {}) }, '매핑 생성 실패') as Promise<Mapping>
}
export async function mappingAction(id: string, action: string, lease: string) {
  if (!lease) throw new Error('제어권이 필요합니다.')
  return mutation(`/api/v1/mappings/${id}/actions`, { action, lease_id: lease }, '매핑 요청 실패')
}
export async function getMission(missionId: string): Promise<Mission> {
  const response = await fetch(`/api/v1/missions/${missionId}`, { credentials: 'include' })
  if (!response.ok) throw new Error(await errorMessage(response, `임무 조회 실패 (${response.status})`))
  return response.json() as Promise<Mission>
}
export async function listMissions(): Promise<Mission[]> {
  const response = await fetch('/api/v1/missions', { credentials: 'include' })
  if (!response.ok) throw new Error(await errorMessage(response, '임무 목록 조회 실패'))
  const value = await response.json() as { items?: Mission[] }
  return value.items ?? []
}
export async function updateMission(id: string, waypoints: Goal[], name: string, repeat_count: number, version: number, lease_id: string) {
  const response = await fetch(`/api/v1/missions/${id}`, { method: 'PATCH', credentials: 'include', headers: { 'Content-Type': 'application/json', 'X-CSRF-Token': csrf() }, body: JSON.stringify({ request_id: crypto.randomUUID(), waypoints, name, repeat_count, version, lease_id }) })
  if (!response.ok) throw new Error(await errorMessage(response, '임무 저장 실패'))
  return response.json() as Promise<Mission>
}

export async function acknowledgeAlert(alertId: string) {
  return mutation(`/api/v1/alerts/${alertId}/ack`, {}, '경고 확인 실패') as Promise<{ alert_id: string; code: string; severity: string; state: string; message: string; occurrences: number; robot_id?: string | null; acknowledged_by?: string | null; acknowledged_at?: string | null }>
}

function historyQuery(filters: HistoryFilters, cursor?: number) {
  const query = new URLSearchParams()
  for (const [key, value] of Object.entries(filters)) if (value) query.set(key, value)
  if (cursor) query.set('cursor', String(cursor))
  return query.toString()
}

export async function getHistory(filters: HistoryFilters = {}, cursor?: number, limit = 50): Promise<{ items: HistoryEvent[]; next_cursor: number | null }> {
  const query = new URLSearchParams(historyQuery(filters, cursor)); query.set('limit', String(limit))
  const response = await fetch(`/api/v1/history?${query.toString()}`, { credentials: 'include' })
  if (!response.ok) throw new Error(await errorMessage(response, '이력 조회 실패'))
  return response.json() as Promise<{ items: HistoryEvent[]; next_cursor: number | null }>
}

export async function downloadHistory(filters: HistoryFilters = {}) {
  const maxEvents = 10_000
  const items: HistoryEvent[] = []
  let cursor: number | undefined
  do {
    const page = await getHistory(filters, cursor, 100)
    items.push(...page.items)
    if (items.length > maxEvents) throw new Error(`이력 내보내기는 최대 ${maxEvents.toLocaleString()}건입니다.`)
    cursor = page.next_cursor ?? undefined
  } while (cursor)
  const content = JSON.stringify({ items, next_cursor: null }, null, 2)
  const blob = new Blob([content], { type: 'application/json' })
  const link = document.createElement('a')
  link.href = URL.createObjectURL ? URL.createObjectURL(blob) : `data:application/json;charset=utf-8,${encodeURIComponent(content)}`
  link.download = 'history.json'; link.click()
  if (URL.revokeObjectURL && link.href.startsWith('blob:')) URL.revokeObjectURL(link.href)
}

type CommandResult = { command_id: string; state: string; reason_code?: string | null; error_code?: string | null }
const terminalCommandStates = new Set(['SUCCEEDED', 'FAILED', 'REJECTED', 'TIMED_OUT', 'CANCELED'])
const delay = (milliseconds: number) => new Promise<void>(resolve => setTimeout(resolve, milliseconds))

export async function waitForCommand(commandId: string, maxAttempts = 30, intervalMs = 100): Promise<CommandResult> {
  for (let attempt = 0; attempt < maxAttempts; attempt += 1) {
    const response = await fetch(`/api/v1/commands/${commandId}`, { credentials: 'include' })
    if (!response.ok) throw new Error(await errorMessage(response, `명령 상태 조회 실패 (${response.status})`))
    const command = await response.json() as CommandResult
    if (terminalCommandStates.has(command.state)) {
      if (command.state !== 'SUCCEEDED') throw new Error(command.reason_code ?? command.error_code ?? `명령 실패 (${command.state})`)
      return command
    }
    if (attempt + 1 < maxAttempts) await delay(intervalMs)
  }
  throw new Error('명령 완료 시간 초과')
}

export async function lineFollowAction(robotId: string, action: 'start' | 'stop', lease_id: string, mode: 'auto' | 'white' | 'black' = 'auto') {
  if (!lease_id) throw new Error('제어권이 필요합니다.')
  return mutation(`/api/v1/robots/${robotId}/line-follow/actions`, { action, mode, lease_id }, '라인 추종 요청 실패')
}
export async function lineFollowStatus(robotId: string) {
  // GET needs session cookies only; no CSRF/lease header (mutations carry those).
  const response = await fetch(`/api/v1/robots/${robotId}/line-follow`, { credentials: 'include' })
  if (!response.ok) throw new Error(await errorMessage(response, `라인 추종 조회 실패 (${response.status})`))
  return response.json()
}

export function stateSocket(onState: (state: any) => void, onStatus: (status: string) => void) {
  let closed = false
  let socket: WebSocket | undefined
  let retry = 1000
  const connect = () => {
    if (closed) return
    onStatus('연결 중')
    socket = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws/state`)
    socket.onopen = () => { retry = 1000; onStatus('실시간 연결') }
    let lastSeq = 0
    socket.onmessage = event => { try { const message = JSON.parse(event.data); const nextSeq = Number(message.seq); if (lastSeq && nextSeq > lastSeq + 1) fetch('/api/v1/state', { credentials: 'include' }).then(response => response.ok ? response.json() : null).then(value => { if (value && Number(value.seq) >= lastSeq) onState(value) }); lastSeq = Math.max(lastSeq, nextSeq || 0); if (message.type === 'snapshot' || message.payload?.robots) onState(message.payload ?? message) } catch { /* discard malformed event */ } }
    socket.onclose = event => { if (closed) return; onStatus([1008, 4401, 4403].includes(event.code) ? '세션 만료' : '재연결 대기'); setTimeout(connect, retry); retry = Math.min(retry * 2, 8000) }
    socket.onerror = () => socket?.close()
  }
  connect()
  return () => { closed = true; socket?.close() }
}
