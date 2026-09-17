import { afterEach, expect, test, vi } from 'vitest'
import { acquireLease, login, releaseLease, renewLease, session, stateSocket, tryBypassLogin } from './api'
import { setMode } from './control'

afterEach(() => vi.restoreAllMocks())

test('login reports invalid credentials', async () => {
  vi.stubGlobal('fetch', vi.fn().mockResolvedValue(new Response('', { status: 401 })))
  await expect(login('operator', 'bad')).rejects.toThrow('아이디 또는 비밀번호')
})

test('expired session becomes anonymous', async () => {
  vi.stubGlobal('fetch', vi.fn().mockResolvedValue(new Response('', { status: 401 })))
  await expect(session()).resolves.toBeNull()
})

test('bypass auto-login returns operator session when backend allows it', async () => {
  vi.stubGlobal('fetch', vi.fn().mockResolvedValue(new Response(JSON.stringify({ user: { username: 'operator', role: 'OPERATOR' } }), { status: 200 })))
  await expect(tryBypassLogin()).resolves.toMatchObject({ username: 'operator' })
})

test('bypass auto-login yields null when backend refuses', async () => {
  vi.stubGlobal('fetch', vi.fn().mockResolvedValue(new Response('', { status: 401 })))
  await expect(tryBypassLogin()).resolves.toBeNull()
})

test('state socket resyncs after a sequence gap and reconnects', async () => {
  const events: unknown[] = []; const statuses: string[] = []
  vi.stubGlobal('fetch', vi.fn().mockResolvedValue(new Response(JSON.stringify({ robots: [] }), { status: 200 })))
  class FakeSocket { static instances: FakeSocket[] = []; onopen = () => {}; onmessage = (_: MessageEvent) => {}; onclose = (_: CloseEvent) => {}; onerror = () => {}; constructor() { FakeSocket.instances.push(this) } close() {} emit(value: unknown) { this.onmessage(new MessageEvent('message', { data: JSON.stringify(value) })) } reconnect() { this.onclose(new CloseEvent('close', { code: 1006 })) } }
  vi.stubGlobal('WebSocket', FakeSocket)
  const stop = stateSocket(value => events.push(value), value => statuses.push(value)); FakeSocket.instances[0].onopen(); FakeSocket.instances[0].emit({ type: 'snapshot', seq: 1, payload: { robots: [{ robot_id: 'robot_1' }] } }); FakeSocket.instances[0].emit({ type: 'robot_state', seq: 3, payload: { robots: [{ robot_id: 'robot_2' }] } })
  await new Promise(resolve => setTimeout(resolve, 0)); expect(events.length).toBe(2); expect(fetch).toHaveBeenCalledWith('/api/v1/state', { credentials: 'include' }); expect(statuses).toContain('실시간 연결'); stop()
})

test('lease follows acquire, renew and release contract', async () => {
  const fetchMock = vi.fn()
    .mockResolvedValueOnce(new Response(JSON.stringify({ lease_id: 'l1', expires_at: 'later' }), { status: 200 }))
    .mockResolvedValueOnce(new Response(JSON.stringify({ lease_id: 'l1', expires_at: 'latest' }), { status: 200 }))
    .mockResolvedValueOnce(new Response('', { status: 200 }))
  vi.stubGlobal('fetch', fetchMock)
  await expect(acquireLease()).resolves.toMatchObject({ lease_id: 'l1' }); await expect(renewLease('l1')).resolves.toMatchObject({ expires_at: 'latest' }); await releaseLease('l1')
  expect(fetchMock.mock.calls.map(call => call[0])).toEqual(['/api/v1/control-lease', '/api/v1/control-lease/l1', '/api/v1/control-lease/l1'])
  expect(fetchMock.mock.calls[1][1]).toMatchObject({ method: 'PATCH' }); expect(fetchMock.mock.calls[2][1]).toMatchObject({ method: 'DELETE' })
})

test('late REST resync cannot overwrite a newer websocket snapshot', async () => {
  let resolve!: (value: Response) => void; const pending = new Promise<Response>(r => { resolve = r }); const events: any[] = []
  vi.stubGlobal('fetch', vi.fn().mockReturnValue(pending)); class Socket { static instances: Socket[] = []; onopen = () => {}; onmessage = (_: MessageEvent) => {}; onclose = () => {}; onerror = () => {}; constructor() { Socket.instances.push(this) } close() {} }
  vi.stubGlobal('WebSocket', Socket); const stop = stateSocket(value => events.push(value), () => {}); const instance = (Socket as any).instances?.[0]; instance.onmessage({ data: JSON.stringify({ type: 'snapshot', seq: 2, payload: { seq: 2 } }) }); instance.onmessage({ data: JSON.stringify({ type: 'robot_state', seq: 4, payload: { seq: 4, robots: [] } }) }); resolve(new Response(JSON.stringify({ seq: 3 }), { status: 200 })); await new Promise(r => setTimeout(r, 0)); expect(events.map(value => value.seq)).toEqual([2, 4]); stop()
})

test('manual mode request targets the selected robot', async () => {
  const fetchMock = vi.fn().mockResolvedValue(new Response(JSON.stringify({ command_id: 'cmd-1' }), { status: 202 }))
  vi.stubGlobal('fetch', fetchMock)
  await expect(setMode('robot_2', 'MANUAL')).resolves.toMatchObject({ command_id: 'cmd-1' })
  expect(fetchMock.mock.calls[0][0]).toBe('/api/v1/robots/robot_2/mode')
  expect(JSON.parse(fetchMock.mock.calls[0][1].body)).toMatchObject({ mode: 'MANUAL' })
})
