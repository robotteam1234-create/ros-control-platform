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
