import { useEffect, useRef, useState } from 'react'
import { setMode } from './control'

type VelocityCommand = { linear_mps: number; angular_rps: number }
const MANUAL_LINEAR_MPS = 0.15
const MANUAL_TURN_RPS = 0.30

export default function Teleop({ robotId, lease, mode, stopLatched }: { robotId: string; lease: string | null; mode: string; stopLatched?: boolean | null }) {
  const [active, setActive] = useState(false)
  const [modeBusy, setModeBusy] = useState(false)
  const [modeError, setModeError] = useState('')
  const seq = useRef(0)
  const ws = useRef<WebSocket | null>(null)
  const timer = useRef<number | undefined>()
  const desired = useRef<VelocityCommand | null>(null)

  const send = (command: VelocityCommand) => {
    if (ws.current?.readyState !== 1) return
    ws.current.send(JSON.stringify({
      lease_id: lease,
      robot_id: robotId,
      seq: ++seq.current,
      ...command,
    }))
  }

  const sendDesired = () => {
    if (desired.current) send(desired.current)
  }

  const stop = () => {
    desired.current = null
    setActive(false)
    if (timer.current !== undefined) {
      clearInterval(timer.current)
      timer.current = undefined
    }
    send({ linear_mps: 0, angular_rps: 0 })
  }

  useEffect(() => {
    const blur = () => stop()
    window.addEventListener('blur', blur)
    document.addEventListener('visibilitychange', blur)
    return () => {
      blur()
      window.removeEventListener('blur', blur)
      document.removeEventListener('visibilitychange', blur)
      ws.current?.close()
      ws.current = null
    }
  }, [robotId, lease, mode])

  const hold = (linear: number, angular: number) => {
    if (!lease || mode !== 'MANUAL' || timer.current !== undefined) return
    desired.current = { linear_mps: linear, angular_rps: angular }

    let socket = ws.current
    if (!socket || socket.readyState === 3) {
      socket = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws/teleop`)
      ws.current = socket
      socket.onopen = sendDesired
      socket.onmessage = event => {
        try {
          const message = JSON.parse(event.data) as { type?: string; reason_code?: string }
          if (message.type === 'rejected') setModeError(`수동 주행 거부: ${message.reason_code ?? 'UNKNOWN'}`)
          else if (message.type === 'accepted') setModeError('')
        } catch {
          setModeError('수동 제어 응답을 해석하지 못했습니다.')
        }
      }
      socket.onerror = () => setModeError('수동 제어 웹소켓 연결 오류')
      socket.onclose = () => {
        if (desired.current) send({ linear_mps: 0, angular_rps: 0 })
        if (ws.current === socket) ws.current = null
        if (desired.current) {
          desired.current = null
          setActive(false)
          if (timer.current !== undefined) {
            clearInterval(timer.current)
            timer.current = undefined
          }
          setModeError('수동 제어 연결이 끊겼습니다.')
        }
      }
    }

    setActive(true)
    sendDesired()
    timer.current = window.setInterval(sendDesired, 100)
  }

  const manualMode = () => {
    if (!lease || modeBusy) return
    setModeBusy(true)
    setModeError('')
    setMode(robotId, 'MANUAL')
      .catch(error => setModeError((error as Error).message))
      .finally(() => setModeBusy(false))
  }

  return <section className="teleop">
    <h2>수동 조작 · {robotId}</h2>
    <p>MANUAL 모드와 제어권이 필요하며 버튼을 누르는 동안만 10Hz 전송합니다.</p>
    <button disabled={!lease || mode === 'MANUAL' || modeBusy || stopLatched === true} onClick={manualMode}>MANUAL 모드 전환</button>
    {stopLatched === true && <p>정지 해제 후 모드 전환 가능</p>}
    <button disabled={!lease || mode !== 'MANUAL'} onPointerDown={() => hold(MANUAL_LINEAR_MPS, 0)} onPointerUp={stop} onPointerLeave={stop} onPointerCancel={stop}>전진</button>
    <button disabled={!lease || mode !== 'MANUAL'} onPointerDown={() => hold(0, MANUAL_TURN_RPS)} onPointerUp={stop} onPointerLeave={stop} onPointerCancel={stop}>좌회전</button>
    <button disabled={!lease || mode !== 'MANUAL'} onPointerDown={() => hold(-MANUAL_LINEAR_MPS, 0)} onPointerUp={stop} onPointerLeave={stop} onPointerCancel={stop}>후진</button>
    <button disabled={!lease || mode !== 'MANUAL'} onPointerDown={() => hold(0, -MANUAL_TURN_RPS)} onPointerUp={stop} onPointerLeave={stop} onPointerCancel={stop}>우회전</button>
    <button disabled={!lease || mode !== 'MANUAL'} onClick={stop}>정지</button>
    <span>{active ? '전송 중' : '정지'} · 현재 모드 {mode}</span>
    {modeError && <em>{modeError}</em>}
  </section>
}
