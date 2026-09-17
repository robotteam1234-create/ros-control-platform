import { render, within } from '@testing-library/react'
import '@testing-library/jest-dom/vitest'
import { describe, expect, it } from 'vitest'
import LineFollowPanel from './LineFollowPanel'

describe('LineFollowPanel', () => {
  it('shows software-only stop copy', () => {
    const view = render(<LineFollowPanel lease="L" onError={() => {}} />)
    expect(within(view.container).getByTestId('line-follow-state')).toBeTruthy()
    expect(within(view.container).getByText(/software-only/i)).toBeTruthy()
  })

  it('offers auto/white/black mode selector', () => {
    const view = render(<LineFollowPanel lease="L" onError={() => {}} />)
    const select = within(view.container).getByLabelText('선 색상') as HTMLSelectElement
    expect(Array.from(select.options).map(option => option.value)).toEqual(['auto', 'white', 'black'])
  })

  it('disables actions without lease', () => {
    const view = render(<LineFollowPanel lease={null} onError={() => {}} />)
    expect(within(view.container).getByText('추종 시작')).toBeDisabled()
  })
})
