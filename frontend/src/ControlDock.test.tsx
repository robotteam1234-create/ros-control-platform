import { cleanup, fireEvent, render } from '@testing-library/react'
import { within } from '@testing-library/react'
import '@testing-library/jest-dom/vitest'
import { afterEach, expect, test, vi } from 'vitest'
import ControlDock from './ControlDock'

afterEach(() => { cleanup(); vi.restoreAllMocks(); vi.unstubAllGlobals() })

test('dock selects robot and exposes stop without lease for operator', () => {
  vi.stubGlobal('crypto', { randomUUID: () => 'request-id' })
  vi.stubGlobal('fetch', vi.fn().mockResolvedValue(new Response(JSON.stringify({ command_id: 'c1' }), { status: 200 })))
  const onSelect = vi.fn()
  const view = render(<ControlDock robots={[{ robot_id: 'robot_1', role: 'MASTER', connection: 'ONLINE' }, { robot_id: 'robot_2', role: 'SLAVE', connection: 'ONLINE' }]} selectedRobot="robot_1" onSelect={onSelect} leaseLabel={null} onAcquire={() => {}} onRelease={() => {}} leaseAcquiring={false} role="OPERATOR" socketStatus="실시간 연결" />)
  const root = within(view.container)
  fireEvent.click(root.getByRole('button', { name: /robot_2/ }))
  expect(onSelect).toHaveBeenCalledWith('robot_2')
  expect(root.getByRole('button', { name: /전체 정지/ })).toBeEnabled()
})

test('polls stop command and shows per-target CONFIRMED', async () => {
  vi.stubGlobal('crypto', { randomUUID: () => 'request-id' })
  const fetchMock = vi.fn((url: string) => {
    if (url === '/api/v1/stop') return Promise.resolve(new Response(JSON.stringify({ command_id: 'c1' }), { status: 200 }))
    return Promise.resolve(new Response(JSON.stringify({ state: 'SUCCEEDED', targets: [{ robot_id: 'robot_1', state: 'CONFIRMED' }] }), { status: 200 }))
  })
  vi.stubGlobal('fetch', fetchMock)
  const view = render(<ControlDock robots={[{ robot_id: 'robot_1', role: 'MASTER', connection: 'ONLINE' }]} selectedRobot="robot_1" onSelect={() => {}} leaseLabel={null} onAcquire={() => {}} onRelease={() => {}} leaseAcquiring={false} role="OPERATOR" socketStatus="실시간 연결" />)
  const root = within(view.container)
  fireEvent.click(root.getByRole('button', { name: /전체 정지/ }))
  expect(await root.findByText(/robot_1: CONFIRMED/, {}, { timeout: 3000 })).toBeInTheDocument()
})

test('FAILED stop shows UNCONFIRMED and clears optimistic result', async () => {
  vi.stubGlobal('crypto', { randomUUID: () => 'request-id' })
  vi.stubGlobal('fetch', vi.fn((url: string) => {
    if (url === '/api/v1/stop') return Promise.resolve(new Response(JSON.stringify({ command_id: 'c1' }), { status: 200 }))
    return Promise.resolve(new Response(JSON.stringify({ state: 'FAILED', targets: [{ robot_id: 'robot_1', state: 'UNCONFIRMED' }] }), { status: 200 }))
  }))
  const view = render(<ControlDock robots={[{ robot_id: 'robot_1', role: 'MASTER', connection: 'ONLINE' }]} selectedRobot="robot_1" onSelect={() => {}} leaseLabel={null} onAcquire={() => {}} onRelease={() => {}} leaseAcquiring={false} role="OPERATOR" socketStatus="실시간 연결" />)
  const root = within(view.container)
  fireEvent.click(root.getByRole('button', { name: /전체 정지/ }))
  expect(await root.findByText(/UNCONFIRMED · 정지 완료/, {}, { timeout: 3000 })).toBeInTheDocument()
  expect(root.queryByText(/접수됨/)).toBeNull()
})

test('polling failure shows error and clears optimistic result; no stale success on retry', async () => {
  vi.stubGlobal('crypto', { randomUUID: () => 'request-id' })
  let calls = 0
  vi.stubGlobal('fetch', vi.fn((url: string) => {
    if (url === '/api/v1/stop') {
      calls += 1
      if (calls === 1) return Promise.resolve(new Response(JSON.stringify({ command_id: 'c1' }), { status: 200 }))
      return Promise.resolve(new Response('fail', { status: 500 }))
    }
    if (url === '/api/v1/commands/c1') return Promise.resolve(new Response(JSON.stringify({ state: 'SUCCEEDED', targets: [{ robot_id: 'robot_1', state: 'CONFIRMED' }] }), { status: 200 }))
    return Promise.resolve(new Response('{}', { status: 200 }))
  }))
  const view = render(<ControlDock robots={[{ robot_id: 'robot_1', role: 'MASTER', connection: 'ONLINE' }]} selectedRobot="robot_1" onSelect={() => {}} leaseLabel={null} onAcquire={() => {}} onRelease={() => {}} leaseAcquiring={false} role="OPERATOR" socketStatus="실시간 연결" />)
  const root = within(view.container)
  fireEvent.click(root.getByRole('button', { name: /전체 정지/ }))
  expect(await root.findByText(/robot_1: CONFIRMED/, {}, { timeout: 3000 })).toBeInTheDocument()
  fireEvent.click(root.getByRole('button', { name: /전체 정지/ }))
  expect(root.queryByText(/robot_1: CONFIRMED/)).toBeNull()
  expect(await root.findByText(/정지 요청 실패/, {}, { timeout: 3000 })).toBeInTheDocument()
  expect(root.queryByText(/robot_1: CONFIRMED/)).toBeNull()
})
