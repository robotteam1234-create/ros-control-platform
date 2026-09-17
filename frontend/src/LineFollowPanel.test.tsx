import { fireEvent, render, within, waitFor } from '@testing-library/react'
import '@testing-library/jest-dom/vitest'
import { beforeEach, describe, expect, it, vi } from 'vitest'
import LineFollowPanel from './LineFollowPanel'
import { lineFollowAction, lineFollowStatus, waitForCommand } from './api'

vi.mock('./api', async importOriginal => {
  const actual = await importOriginal<typeof import('./api')>()
  return { ...actual, lineFollowAction: vi.fn(), lineFollowStatus: vi.fn(), waitForCommand: vi.fn() }
})

const mockedAction = vi.mocked(lineFollowAction)
const mockedStatus = vi.mocked(lineFollowStatus)
const mockedWait = vi.mocked(waitForCommand)

beforeEach(() => {
  vi.clearAllMocks()
  mockedAction.mockResolvedValue({ command_id: 'cmd-1' } as never)
  mockedWait.mockResolvedValue({ command_id: 'cmd-1', state: 'SUCCEEDED' } as never)
  mockedStatus.mockResolvedValue({ robot_id: 'robot_1', state: 'TRACKING' } as never)
})

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

  it('waits for command and status before showing TRACKING', async () => {
    const view = render(<LineFollowPanel lease="L" onError={() => {}} />)
    fireEvent.click(within(view.container).getByText('추종 시작'))
    await waitFor(() => expect(mockedWait).toHaveBeenCalledWith('cmd-1'))
    await waitFor(() => expect(mockedStatus).toHaveBeenCalledWith('robot_1'))
    await waitFor(() => expect(within(view.container).getByTestId('line-follow-state')).toHaveTextContent('TRACKING'))
  })

  it('surfaces command failure via onError without claiming TRACKING', async () => {
    mockedWait.mockRejectedValueOnce(new Error('CAMERA_STALLED'))
    const onError = vi.fn()
    const view = render(<LineFollowPanel lease="L" onError={onError} />)
    fireEvent.click(within(view.container).getByText('추종 시작'))
    await waitFor(() => expect(onError).toHaveBeenCalledWith(expect.stringContaining('CAMERA_STALLED')))
    expect(within(view.container).getByTestId('line-follow-state')).toHaveTextContent('IDLE')
  })
})
