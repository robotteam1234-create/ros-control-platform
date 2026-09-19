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

import { parseHash } from './DashboardTabs'

test('control tab is first and legacy camera hash maps to control', () => {
  const onChange = vi.fn()
  const view = render(<DashboardTabs active="control" onChange={onChange} />)
  const root = within(view.container)
  expect(root.getAllByRole('tab')[0].textContent).toBe('제어 Control')
  expect(root.getByRole('tab', { name: /제어 Control/ })).toHaveAttribute('aria-selected', 'true')
  expect(parseHash('#camera')).toBe('control')
  expect(parseHash('#control')).toBe('control')
  expect(parseHash('')).toBe('drive')
})
