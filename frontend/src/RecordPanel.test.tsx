import '@testing-library/jest-dom/vitest'
import { render, within } from '@testing-library/react'
import { describe, expect, it } from 'vitest'
import RecordPanel from './RecordPanel'

describe('RecordPanel', () => {
  it('shows idle state and gates start without lease', () => {
    const view = render(<RecordPanel lease={null} onError={() => {}} />)
    const scope = within(view.container)
    expect(scope.getByTestId('record-state').textContent).toBe('IDLE')
    expect(scope.getByText('녹화 시작')).toBeDisabled()
  })
})
