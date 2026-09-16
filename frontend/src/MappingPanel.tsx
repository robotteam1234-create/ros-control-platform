import { useState } from 'react'
import { createMapping, mappingAction, waitForCommand } from './api'
import type { Mapping } from './api'
import { MapHealth } from './MapHealth'
import { SAFE_CAPS, STAGE_INFO } from './mappingStages'

type Props = { lease: string | null; mapId?: string | null; robotId?: string; onError?: (message: string) => void }

const STAGES = STAGE_INFO.map(stage => stage.id)
const stateAfterAction: Record<string, string> = { validate: 'READY', start: 'RUNNING', pause: 'PAUSED', resume: 'RUNNING', cancel: 'CANCELED' }

export function MappingPanel({ lease, mapId, robotId = 'robot_1', onError }: Props) {
  const [mapping, setMapping] = useState<Mapping | null>(null)
  const [busy, setBusy] = useState(false)
  const state = mapping?.state ?? 'IDLE'
  const canStart = Boolean(lease && mapId) && !busy

  const report = (message: string) => onError?.(message)

  const runAction = async (action: 'validate' | 'start' | 'pause' | 'resume' | 'cancel') => {
    if (!mapping || !lease || busy) return
    setBusy(true)
    try {
      const issued = (await mappingAction(mapping.mapping_id, action, lease)) as { command_id?: string }
      if (issued?.command_id) await waitForCommand(String(issued.command_id))
      // No backend GET route (Plan 2 scope): refresh from local state explicitly.
      setMapping(current => (current ? { ...current, state: stateAfterAction[action] ?? current.state } : current))
    } catch (error) {
      report((error as Error).message)
      /* preserve command error: no refetch, local state unchanged */
    } finally {
      setBusy(false)
    }
  }

  const start = async () => {
    if (!lease || !mapId || busy) return
    setBusy(true)
    try {
      const created = await createMapping(robotId, mapId, lease, 'mapping', STAGES)
      setMapping(created)
      const issued = (await mappingAction(created.mapping_id, 'start', lease)) as { command_id?: string }
      if (issued?.command_id) await waitForCommand(String(issued.command_id))
      // No backend GET route (Plan 2 scope): refresh from local state explicitly.
      setMapping({ ...created, state: 'RUNNING' })
    } catch (error) {
      report((error as Error).message)
    } finally {
      setBusy(false)
    }
  }

  const stop = () => runAction('cancel')

  return (
    <section className="mapping-panel">
      <h2>매핑 · {state}</h2>
      <ul>
        {STAGE_INFO.map(stage => (
          <li key={stage.id}>
            {stage.id} · {stage.label} · {stage.description}{' '}
            {stage.requirement === 'lidar-only' ? '(라이다 전용)' : '(SLAM 필요)'}
          </li>
        ))}
      </ul>
      <p>
        안전 상한: {SAFE_CAPS.maxLinearMps} m/s (로봇 측 watchdog 강제)
      </p>
      <MapHealth health={null} />
      <button disabled={!canStart} onClick={start}>
        매핑 시작
      </button>
      <button disabled={state !== 'RUNNING'} onClick={stop}>
        중지
      </button>
      {!lease && <p className="map-warning">매핑을 시작하려면 제어권이 필요합니다.</p>}
      {!mapId && <p className="map-warning">매핑을 시작하려면 활성 지도가 필요합니다.</p>}
    </section>
  )
}

export default MappingPanel
