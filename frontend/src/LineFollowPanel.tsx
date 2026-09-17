import { useState } from 'react'
import { lineFollowAction, lineFollowStatus, waitForCommand } from './api'

export default function LineFollowPanel({ lease, onError }: { lease: string | null; onError: (m: string) => void }) {
  const [state, setState] = useState('IDLE')
  const [mode, setMode] = useState<'auto' | 'white' | 'black'>('auto')
  const [busy, setBusy] = useState(false)
  const act = async (action: 'start' | 'stop') => {
    if (!lease || busy) return
    setBusy(true)
    try {
      const issued = await lineFollowAction('robot_1', action, lease, mode) as { command_id?: string }
      if (issued?.command_id) await waitForCommand(issued.command_id)
      const status = await lineFollowStatus('robot_1') as { state?: string }
      setState(status.state ?? (action === 'start' ? 'TRACKING' : 'IDLE'))
      if (status.state === 'FAILED' || status.state === 'CAMERA_STALLED') onError(status.state)
    } catch (e) { onError((e as Error).message) } finally { setBusy(false) }
  }
  const refresh = async () => {
    if (busy) return
    setBusy(true)
    try {
      const status = await lineFollowStatus('robot_1') as { state?: string }
      if (status.state) setState(status.state)
    } catch (e) { onError((e as Error).message) } finally { setBusy(false) }
  }
  return <section className="line-follow-panel">
    <h2>라인 추종 · {state}</h2>
    <p data-testid="line-follow-state">{state}</p>
    {/* Offset bar + camera-grid overlay (center line + cx dot) are follow-up after MVP status API lands; panel polls lineFollowStatus here. */}
    <label>선 색상<select aria-label="선 색상" value={mode} onChange={e => setMode(e.target.value as typeof mode)}><option value="auto">자동(흰/검정)</option><option value="white">흰색</option><option value="black">검정색</option></select></label>
    <button disabled={!lease || busy} onClick={() => act('start')}>추종 시작</button>
    <button disabled={!lease || busy} onClick={() => act('stop')}>정지 (software-only)</button>
    <button disabled={busy} onClick={refresh}>상태 새로고침</button>
  </section>
}
