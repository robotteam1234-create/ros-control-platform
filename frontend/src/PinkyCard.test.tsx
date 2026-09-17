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
