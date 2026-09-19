import { useEffect, useState } from 'react'
import { createMapping, mappingAction, mappingImport, mappingStatus, waitForCommand } from './api'
import type { Mapping } from './api'
import { SAFE_CAPS, STAGE_INFO } from './mappingStages'
import { openMappingStream } from './mappingStream'

type Props = { lease: string | null; mapId?: string | null; robotId?: string; onError?: (message: string) => void }

const STAGES = STAGE_INFO.map(stage => stage.id)
const stateAfterAction: Record<string, string> = { validate: 'READY', start: 'RUNNING', pause: 'PAUSED', resume: 'RUNNING', cancel: 'CANCELED' }

export function MappingPanel({ lease, mapId, robotId = 'robot_1', onError }: Props) {
  const [mapping, setMapping] = useState<Mapping | null>(null)
  const [busy, setBusy] = useState(false)
  const [run, setRun] = useState<{ state: string; stage_index?: number; message?: string; reason?: string } | null>(null)
  const [mapMeta, setMapMeta] = useState<{ seq: number; width: number; height: number; resolution: number } | null>(null)
  const [mapUrl, setMapUrl] = useState<string | null>(null)
  const [imported, setImported] = useState<string | null>(null)
  const state = mapping?.state ?? 'IDLE'
  const liveState = run?.state ?? state
  const canStart = Boolean(lease && mapId) && !busy
  const running = liveState === 'RUNNING'
  const stageIndex = run?.stage_index ?? -1

  const report = (message: string) => onError?.(message)

  // While the backend reports RUNNING, poll authoritative status and draw the live SLAM map.
  useEffect(() => {
    if (liveState !== 'RUNNING' || !mapping) return
    const poll = setInterval(() => {
      mappingStatus(mapping.mapping_id)
        .then(setRun)
        .catch(() => {})
    }, 1000)
    const close = openMappingStream(robotId, (meta, url) => {
      setMapMeta(meta)
      setMapUrl(url)
    })
    return () => {
      clearInterval(poll)
      close()
    }
  }, [liveState, mapping?.mapping_id, robotId])

  const doImport = async () => {
    if (!mapping || busy) return
    setBusy(true)
    try {
      const result = await mappingImport(mapping.mapping_id)
      setImported(result.imported_map_id)
    } catch (error) {
      report((error as Error).message)
    } finally {
      setBusy(false)
    }
  }

  const applyAction = async (action: 'validate' | 'start' | 'pause' | 'resume' | 'cancel') => {
    if (!mapping || !lease || busy) return
    setBusy(true)
    try {
      const issued = (await mappingAction(mapping.mapping_id, action, lease)) as { command_id?: string }
      if (issued?.command_id) await waitForCommand(String(issued.command_id))
      setMapping(current => (current ? { ...current, state: stateAfterAction[action] ?? current.state } : current))
      const status = await mappingStatus(mapping.mapping_id)
      setRun(status)
    } catch (error) {
      report((error as Error).message)
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
      setImported(null)
      const issued = (await mappingAction(created.mapping_id, 'start', lease)) as { command_id?: string }
      if (issued?.command_id) await waitForCommand(String(issued.command_id))
      setMapping({ ...created, state: 'RUNNING' })
      setRun({ state: 'RUNNING', stage_index: -1 })
    } catch (error) {
      report((error as Error).message)
    } finally {
      setBusy(false)
    }
  }

  const act = (action: 'pause' | 'resume' | 'cancel') => {
    if (!mapping || !lease || busy) return
    setBusy(true)
    void (async () => {
      try {
        await applyAction(action)
      } finally {
        setBusy(false)
      }
    })()
  }

  return (
    <section className="mapping-panel">
      <div className="mapping-head">
        <h2>자동 매핑</h2>
        <span className={`mapping-chip mapping-${liveState.toLowerCase()}`} data-testid="mapping-state-chip">
          {liveState}
        </span>
      </div>

      <ol className="mapping-stages">
        {STAGE_INFO.map((stage, index) => {
          const cls =
            liveState === 'COMPLETED' || (stageIndex >= 0 && index < stageIndex)
              ? 'done'
              : running && index === stageIndex
                ? 'active'
                : ''
          return (
            <li key={stage.id} className={`mapping-stage ${cls}`}>
              <span className="mapping-stage-name">
                {index + 1}. {stage.label}
              </span>
              <span className="mapping-stage-desc">
                {stage.description} · {stage.requirement === 'lidar-only' ? '라이다 전용' : 'SLAM 필요'}
              </span>
            </li>
          )
        })}
      </ol>

      {running && (
        <div className="mapping-live">
          {mapUrl ? (
            <>
              <img data-testid="live-map" src={mapUrl} alt="실시간 SLAM 지도" />
              {mapMeta && (
                <p className="mapping-live-meta">
                  실시간 지도 · {mapMeta.width}×{mapMeta.height}칸 · {mapMeta.resolution.toFixed(3)} m/칸 · 갱신 #{mapMeta.seq}
                </p>
              )}
            </>
          ) : (
            <p className="empty">실시간 지도를 기다리는 중… (SLAM이 첫 지도를 만들면 여기에 그려집니다)</p>
          )}
          {run?.message && <p data-testid="mapping-stage-readout">{run.message}</p>}
        </div>
      )}

      {liveState === 'FAILED' && (
        <p className="map-warning" data-testid="mapping-failed">
          매핑 실패{run?.reason ? ` (${run.reason})` : ''} — 로봇 주변을 확인하고 다시 시작하세요.
        </p>
      )}

      <p className="mapping-safety">
        안전 상한: {SAFE_CAPS.maxLinearMps} m/s / {SAFE_CAPS.maxAngularRps} rad/s (로봇 측 watchdog 강제) · 정지는 소프트웨어 정지이며 물리 안전 장치가 아닙니다
      </p>

      <div className="mapping-actions">
        {liveState !== 'RUNNING' && liveState !== 'PAUSED' && (
          <button disabled={!canStart || busy} onClick={start}>
            {liveState === 'PAUSED' ? '매핑 재개' : '자동 매핑 시작'}
          </button>
        )}
        {running && (
          <button data-testid="mapping-pause" disabled={busy} onClick={() => act('pause')}>
            일시정지
          </button>
        )}
        {liveState === 'PAUSED' && (
          <button disabled={busy} onClick={() => act('cancel')}>
            종료
          </button>
        )}
        {liveState === 'COMPLETED' && !imported && (
          <button data-testid="mapping-import" disabled={busy} onClick={doImport}>
            결과를 지도로 가져오기
          </button>
        )}
      </div>

      {imported && (
        <p data-testid="mapping-imported">
          가져오기 완료: <strong>{imported}</strong> — 지도 탭 또는 설정에서 활성 지도로 선택하세요.
        </p>
      )}
      {!lease && <p className="map-warning">매핑을 시작하려면 제어권이 필요합니다.</p>}
      {!mapId && <p className="map-warning">매핑을 시작하려면 활성 지도가 필요합니다.</p>}
    </section>
  )
}

export default MappingPanel
