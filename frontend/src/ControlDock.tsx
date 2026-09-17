import { useState } from 'react'
import { resetStop, stop } from './control'

type DockRobot = { robot_id: string; role: string; connection: string }

export default function ControlDock({ robots, selectedRobot, onSelect, leaseLabel, onAcquire, onRelease, leaseAcquiring, role, socketStatus }: {
  robots: DockRobot[]
  selectedRobot: string
  onSelect: (id: string) => void
  leaseLabel: string | null
  onAcquire: () => void
  onRelease: () => void
  leaseAcquiring: boolean
  role: string
  socketStatus: string
}) {
  const [result, setResult] = useState('')
  const [error, setError] = useState('')
  const run = (target: 'all' | 'robot_1' | 'robot_2') => {
    setError('')
    stop(target).then(() => setResult('정지 접수됨 · 소프트웨어 정지')).catch(e => setError((e as Error).message))
  }
  const resetTarget = (selectedRobot === 'robot_1' || selectedRobot === 'robot_2' ? selectedRobot : 'all') as 'all' | 'robot_1' | 'robot_2'
  return (
    <div className="dock" role="region" aria-label="제어 독">
      <div className="dock-robots">
        {robots.map(r => (
          <button key={r.robot_id} aria-pressed={selectedRobot === r.robot_id} className={selectedRobot === r.robot_id ? 'pill online' : 'pill'} onClick={() => onSelect(r.robot_id)}>
            {r.robot_id} · {r.role} · {r.connection}
          </button>
        ))}
      </div>
      <div className="dock-lease">
        <span className="pill">{socketStatus}</span>
        {leaseLabel
          ? <><span className="pill online">제어권 활성</span><button disabled={leaseAcquiring} onClick={onRelease}>반납</button></>
          : <button disabled={leaseAcquiring} onClick={onAcquire}>제어권 획득</button>}
      </div>
      <div className="dock-stop">
        <strong>정지</strong>
        <span>소프트웨어 정지이며 물리 안전 장치가 아닙니다.</span>
        <button disabled={role === 'VIEWER'} onClick={() => run('all')}>전체 정지</button>
        <button disabled={role === 'VIEWER'} onClick={() => run(resetTarget)}>{resetTarget} 정지</button>
        {leaseLabel && <button className="reset" onClick={() => resetStop(resetTarget, leaseLabel).then(() => setResult(`${resetTarget} 해제 접수됨 · 자동 주행 없음`)).catch(e => setError((e as Error).message))}>{resetTarget} 정지 해제</button>}
        {result && <b>{result}</b>}
        {error && <em>{error}</em>}
      </div>
    </div>
  )
}
