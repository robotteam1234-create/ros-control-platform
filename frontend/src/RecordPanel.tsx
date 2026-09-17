import { useState } from 'react'
import { recordAction, recordStatus } from './api'

export default function RecordPanel({ lease, onError }: { lease: string | null; onError: (m: string) => void }) {
  const [robot, setRobot] = useState('robot_1')
  const [label, setLabel] = useState('run')
  const [frames, setFrames] = useState<number | null>(null)
  const [active, setActive] = useState(false)
  const [busy, setBusy] = useState(false)
  const act = async (action: 'start' | 'stop') => {
    if (!lease || busy) return
    setBusy(true)
    try {
      const result = await recordAction(action, robot, lease, label) as { frames?: number; label?: string }
      if (action === 'start') { setActive(true); setFrames(0) }
      else { setActive(false); setFrames(result.frames ?? null) }
    } catch (e) { onError((e as Error).message) } finally { setBusy(false) }
  }
  const refresh = async () => {
    if (busy) return
    setBusy(true)
    try {
      const status = await recordStatus() as { active?: Record<string, { label?: string; frames?: number }> }
      const entry = status.active?.[robot]
      setActive(Boolean(entry))
      setFrames(entry?.frames ?? null)
    } catch (e) { onError((e as Error).message) } finally { setBusy(false) }
  }
  return <section className="record-panel">
    <h2>데이터셋 녹화 · {active ? `REC ${frames ?? 0}장` : 'IDLE'}</h2>
    <p data-testid="record-state">{active ? 'REC' : 'IDLE'}</p>
    <label>로봇<select aria-label="녹화 로봇" value={robot} onChange={e => setRobot(e.target.value)}><option value="robot_1">robot_1</option><option value="robot_2">robot_2</option></select></label>
    <label>라벨<input aria-label="녹화 라벨" maxLength={64} value={label} onChange={e => setLabel(e.target.value)} /></label>
    <button disabled={!lease || busy} onClick={() => act('start')}>녹화 시작</button>
    <button disabled={!lease || busy} onClick={() => act('stop')}>녹화 정지</button>
    <button disabled={busy} onClick={refresh}>상태 새로고침</button>
  </section>
}
