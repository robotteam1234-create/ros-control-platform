import { cleanup, fireEvent, render, waitFor } from '@testing-library/react'
import { within } from '@testing-library/react'
import '@testing-library/jest-dom/vitest'
import { afterEach, expect, test, vi } from 'vitest'
import App from './App'
import { parseHash } from './DashboardTabs'

afterEach(() => { cleanup(); vi.restoreAllMocks(); vi.unstubAllGlobals(); window.location.hash = '' })

test('parseHash falls back to drive on unknown hash', () => {
  expect(parseHash('#mission')).toBe('mission')
  expect(parseHash('#unknown')).toBe('drive')
  expect(parseHash('')).toBe('drive')
})

function mockAppBackend() {
  vi.stubGlobal('crypto', { randomUUID: () => 'request-id' })
  const state = {
    robots: [
      { robot_id: 'robot_1', name: 'r1', role: 'MASTER', connection: 'ONLINE', mode: 'IDLE' },
      { robot_id: 'robot_2', name: 'r2', role: 'SLAVE', connection: 'ONLINE', mode: 'IDLE' },
    ],
    mode: 'IDLE',
    seq: 1,
    server_time: new Date().toISOString(),
    map_id: null,
    formation: { state: 'STOPPED', master_id: 'robot_1', slave_id: 'robot_2', distance_m: null, bearing_rad: null },
    active_alerts: [],
  }
  vi.stubGlobal('fetch', vi.fn((url: string) => {
    if (url === '/api/v1/session') return Promise.resolve(new Response(JSON.stringify({ user: { username: 'op', role: 'OPERATOR' } }), { status: 200 }))
    if (url === '/api/v1/state') return Promise.resolve(new Response(JSON.stringify(state), { status: 200 }))
    if (url.includes('/sensor-layers')) return Promise.resolve(new Response(JSON.stringify({ robot_id: 'robot_1', scan: { state: 'UNSUPPORTED' }, costmaps: [] }), { status: 200 }))
    return Promise.resolve(new Response(JSON.stringify({}), { status: 200 }))
  }))
  class FakeSocket {
    onopen = () => {}
    onmessage = (_: MessageEvent) => {}
    onclose = (_: CloseEvent) => {}
    onerror = () => {}
    close() {}
  }
  vi.stubGlobal('WebSocket', FakeSocket)
}

test('dock visible on every tab, mission shows FormationPanel and hides MapPanel, hash updates', async () => {
  window.location.hash = '#drive'
  mockAppBackend()
  const view = render(<App />)
  const root = within(view.container)
  await waitFor(() => expect(root.getByRole('region', { name: /제어 독/ })).toBeInTheDocument())
  // drive tab shows map
  expect(root.getByText(/지도 · 로봇 위치/)).toBeInTheDocument()
  fireEvent.click(root.getByRole('tab', { name: /편대/ }))
  await waitFor(() => expect(root.getByText(/편대 상태/)).toBeInTheDocument())
  expect(root.queryByText(/지도 · 로봇 위치/)).toBeNull()
  expect(root.getByRole('region', { name: /제어 독/ })).toBeInTheDocument()
  expect(window.location.hash).toBe('#mission')
})

test('unknown hash falls back to drive', async () => {
  window.location.hash = '#nope'
  mockAppBackend()
  const view = render(<App />)
  const root = within(view.container)
  await waitFor(() => expect(root.getByRole('region', { name: /제어 독/ })).toBeInTheDocument())
  expect(root.getByRole('tab', { name: /Drive/ })).toHaveAttribute('aria-selected', 'true')
  expect(root.getByText(/지도 · 로봇 위치/)).toBeInTheDocument()
})
