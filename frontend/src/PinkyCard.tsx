export type RobotCard = { robot_id: string; name: string; role: 'MASTER' | 'SLAVE'; connection: string; mode: string; battery_percent: number | null; voltage_v?: number | null; linear_mps?: number | null; angular_rps?: number | null; pose_freshness?: string; stop_latched: boolean | null; capabilities?: string[]; tf_valid?: boolean }

export default function PinkyCard({ robot, selected, onSelect, lease, onMode }: { robot: RobotCard; selected: boolean; onSelect: () => void; lease: string | null; onMode: (mode: 'MANUAL' | 'IDLE') => void }) {
  return (
    <article className={`card ${selected ? 'selected-card' : ''}`} onClick={onSelect}>
      <div className="card-head"><div><h3>{robot.name}</h3><small>{robot.robot_id}</small></div><b className={robot.role.toLowerCase()}>{robot.role}</b></div>
      <dl>
        <div><dt>연결</dt><dd className="connection">{robot.connection}</dd></div>
        <div><dt>모드</dt><dd>{robot.mode}</dd></div>
        <div><dt>배터리 / 전압</dt><dd>{robot.battery_percent == null ? '—' : `${robot.battery_percent}%`} · {robot.voltage_v == null ? '—' : `${robot.voltage_v}V`}</dd></div>
        <div><dt>속도</dt><dd>{robot.linear_mps == null ? '—' : `${robot.linear_mps}m/s`} · {robot.angular_rps == null ? '—' : `${robot.angular_rps}rad/s`}</dd></div>
        <div><dt>위치 신선도</dt><dd>{robot.pose_freshness ?? 'UNKNOWN'}</dd></div>
        <div><dt>기능</dt><dd>{robot.capabilities?.includes('navigate') ? 'navigate ✓' : 'navigate —'} · TF {robot.tf_valid ? 'OK' : '지연'}</dd></div>
      </dl>
      {robot.stop_latched && <div className="stop">정지 래치</div>}
      <button disabled={!lease || robot.mode === 'MANUAL' || robot.stop_latched === true} onClick={e => { e.stopPropagation(); onMode('MANUAL') }}>MANUAL 모드</button>
      <button disabled={!lease || robot.mode === 'IDLE'} onClick={e => { e.stopPropagation(); onMode('IDLE') }}>IDLE 모드</button>
    </article>
  )
}
