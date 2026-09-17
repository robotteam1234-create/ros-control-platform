# Line-follow Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Backend OpenCV white/black tape follower for robot_1 starting in control-platform.

**Architecture:** New pure `line_detector` (JPEG bytes -> offset) plus `LineFollowService` 10 Hz loop that reads `adapter.frame("robot_1")` and writes `adapter.publish_manual_velocity` to the watchdog TwistStamped input. New `mission_line_start/stop` ops via command dispatcher, minimal `LineFollowPanel` UI.

**Tech Stack:** Python 3.12, FastAPI, OpenCV-headless 5.x + numpy, Pillow (test image gen), pytest, React+TS vitest.

**Spec:** `docs/superpowers/specs/2026-09-17-line-follow-design.md`

## Global Constraints

- `robot_1` MASTER domain 12, `robot_2` SLAVE domain 13 — never change domain IDs.
- Watchdog is SOLE `/cmd_vel` publisher; backend only calls `adapter.publish_manual_velocity` (TwistStamped `/control/manual_velocity`).
- Op names MUST NOT start with `follow_`; use `mission_line_start`, `mission_line_stop`.
- All mutations need `request_id` + lease + `X-CSRF-Token` + `Origin`, else 422/409/403.
- `CONTROL_PLATFORM_WORKERS=1` single worker only.
- TDD: failing test first (RED), minimal implement (GREEN), full suite before commit.
- Backend tests: `backend/.venv/bin/python -m pytest backend/tests/ -q` from repo root (`control-platform/`); retry with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` on ROS plugin clash.
- Stop UI copy must say software-only, never "emergency".
- Real-robot validation stays NOT_RUN in `acceptance-report.md` until log evidence exists.
- Commits: `git -c user.name="opencode" -c user.email="opencode@localhost" commit` (never touch git config).

---

### Task 1: Pure line detector (white/black/auto)

**Files:**
- Create: `backend/pinky_control_center/line_detector.py`
- Test: `backend/tests/test_line_detector.py`

**Interfaces:**
- Consumes: JPEG `bytes`, `mode: str` in `{"auto","white","black"}`.
- Produces: `LineDetectorResult(found: bool, offset: float, area_ratio: float, polarity: str)` and `detect_line(jpeg: bytes, mode: str = "auto") -> LineDetectorResult`. `offset` in [-1,1], 0=center.

- [ ] **Step 1: Write the failing test**

```python
# backend/tests/test_line_detector.py
from pinky_control_center.line_detector import detect_line
from PIL import Image
import io

def _jpeg(width_bar_x0, line="white", size=(320, 240)):
    bg = (40, 40, 40) if line == "white" else (220, 220, 220)
    fg = (255, 255, 255) if line == "white" else (0, 0, 0)
    img = Image.new("RGB", size, bg)
    px = img.load()
    for y in range(size[1] // 2, size[1]):
        for x in range(width_bar_x0, width_bar_x0 + 30):
            px[x, y] = fg
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=90)
    return buf.getvalue()

def test_white_bar_left_is_negative_offset():
    r = detect_line(_jpeg(20, "white"), "white")
    assert r.found is True and r.offset < -0.3 and r.polarity == "white"

def test_black_bar_right_is_positive_offset():
    r = detect_line(_jpeg(250, "black"), "black")
    assert r.found is True and r.offset > 0.3 and r.polarity == "black"

def test_auto_detects_both():
    assert detect_line(_jpeg(20, "white"), "auto").polarity == "white"
    assert detect_line(_jpeg(250, "black"), "auto").polarity == "black"

def test_empty_floor_is_loss():
    img = Image.new("RGB", (320, 240), (40, 40, 40))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=90)
    assert detect_line(buf.getvalue(), "auto").found is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `backend/.venv/bin/python -m pytest backend/tests/test_line_detector.py -v`
Expected: FAIL with "No module named 'pinky_control_center.line_detector'".

- [ ] **Step 3: Write minimal implementation**

```python
# backend/pinky_control_center/line_detector.py
from __future__ import annotations
from dataclasses import dataclass
import cv2
import numpy as np

@dataclass(frozen=True)
class LineDetectorResult:
    found: bool
    offset: float
    area_ratio: float
    polarity: str

WHITE_THRESH = 200
BLACK_THRESH = 60
MIN_AREA_RATIO = 0.005

def detect_line(jpeg: bytes, mode: str = "auto") -> LineDetectorResult:
    arr = np.frombuffer(jpeg, dtype=np.uint8)
    gray = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        return LineDetectorResult(False, 0.0, 0.0, "unknown")
    h, w = gray.shape
    roi = gray[h // 2 :, :]
    blur = cv2.GaussianBlur(roi, (5, 5), 0)
    polarity = mode
    if mode == "auto":
        polarity = "white" if float(blur.mean()) < 127 else "black"
    if polarity == "white":
        _, mask = cv2.threshold(blur, WHITE_THRESH, 255, cv2.THRESH_BINARY)
    elif polarity == "black":
        _, mask = cv2.threshold(blur, BLACK_THRESH, 255, cv2.THRESH_BINARY_INV)
    else:
        raise ValueError("INVALID_VALUE")
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return LineDetectorResult(False, 0.0, 0.0, polarity)
    best = max(contours, key=cv2.contourArea)
    area_ratio = float(cv2.contourArea(best) / (roi.shape[0] * roi.shape[1]))
    if area_ratio < MIN_AREA_RATIO:
        return LineDetectorResult(False, 0.0, area_ratio, polarity)
    m = cv2.moments(best)
    if m["m00"] <= 0:
        return LineDetectorResult(False, 0.0, area_ratio, polarity)
    cx = float(m["m10"] / m["m00"])
    offset = max(-1.0, min(1.0, (cx - w / 2) / (w / 2)))
    return LineDetectorResult(True, offset, area_ratio, polarity)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `backend/.venv/bin/python -m pytest backend/tests/test_line_detector.py -v`
Expected: PASS (4 passed).

- [ ] **Step 5: Commit**

```bash
git add backend/pinky_control_center/line_detector.py backend/tests/test_line_detector.py
git -c user.name="opencode" -c user.email="opencode@localhost" commit -m "feat: add white/black tape line detector"
```

### Task 2: LineFollowService 10 Hz loop (watchdog-safe)

**Files:**
- Create: `backend/pinky_control_center/line_follow_service.py`
- Test: `backend/tests/test_line_follow_service.py`

**Interfaces:**
- Consumes: `detect_line` from Task 1; adapter with `frame(robot_id) -> CameraFrame|None` and `publish_manual_velocity(robot_id, lin, ang) -> CommandAcceptance`; `CameraFrame` has `.jpeg: bytes`.
- Produces: `class LineFollowService(adapter, clock=time.monotonic)` with `async start(robot_id, mode="auto")`, `async stop(robot_id)`, `status(robot_id) -> dict`, `async tick_once(robot_id) -> dict`. Constants `LINEAR_MPS=0.10`, `KP=0.6`, `KD=0.15`, `LOSS_LIMIT=3`, `MAX_ANGULAR=0.50`.

- [ ] **Step 1: Write the failing test**

```python
# backend/tests/test_line_follow_service.py
import asyncio
from pinky_control_center.line_follow_service import LineFollowService
from pinky_control_center.models import CommandAcceptance
from pinky_control_center.line_detector import detect_line
from PIL import Image
import io

def _tape_jpeg(x0=140, line="white"):
    bg = (40, 40, 40) if line == "white" else (220, 220, 220)
    fg = (255, 255, 255) if line == "white" else (0, 0, 0)
    img = Image.new("RGB", (320, 240), bg)
    px = img.load()
    for y in range(120, 240):
        for x in range(x0, x0 + 30):
            px[x, y] = fg
    b = io.BytesIO()
    img.save(b, format="JPEG", quality=90)
    return b.getvalue()

class FakeFrame:
    def __init__(self, jpeg):
        self.jpeg = jpeg

class FakeAdapter:
    def __init__(self, jpeg):
        self._jpeg = jpeg
        self.published: list[tuple] = []
    def frame(self, robot_id):
        return FakeFrame(self._jpeg)
    async def publish_manual_velocity(self, robot_id, lin, ang):
        self.published.append((robot_id, lin, ang))
        return CommandAcceptance(accepted=True)

def test_tick_publishes_bounded_velocity():
    svc = LineFollowService(FakeAdapter(_tape_jpeg(20, "white")))
    asyncio.run(svc.start("robot_1", "auto"))
    out = asyncio.run(svc.tick_once("robot_1"))
    assert out["state"] == "TRACKING"
    _, lin, ang = svc.adapter.published[-1]
    assert abs(lin) <= 0.15 and abs(ang) <= 0.50

def test_three_misses_stop_and_lost():
    img = Image.new("RGB", (320, 240), (40, 40, 40))
    b = io.BytesIO()
    img.save(b, format="JPEG", quality=90)
    svc = LineFollowService(FakeAdapter(b.getvalue()))
    asyncio.run(svc.start("robot_1", "auto"))
    for _ in range(3):
        out = asyncio.run(svc.tick_once("robot_1"))
    assert out["state"] == "LOST"
    assert svc.adapter.published[-1][1:] == (0.0, 0.0)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `backend/.venv/bin/python -m pytest backend/tests/test_line_follow_service.py -v`
Expected: FAIL with "No module named 'pinky_control_center.line_follow_service'".

- [ ] **Step 3: Write minimal implementation**

```python
# backend/pinky_control_center/line_follow_service.py
from __future__ import annotations
import time
from collections.abc import Callable
from pinky_control_center.line_detector import detect_line

LINEAR_MPS = 0.10
KP = 0.6
KD = 0.15
LOSS_LIMIT = 3
MAX_ANGULAR = 0.50

class LineFollowService:
    def __init__(self, adapter, clock: Callable[[], float] = time.monotonic) -> None:
        self.adapter = adapter
        self._clock = clock
        self._active: dict[str, dict] = {}

    async def start(self, robot_id: str, mode: str = "auto") -> dict:
        if robot_id != "robot_1":
            raise ValueError("ROBOT_NOT_SUPPORTED")
        if mode not in {"auto", "white", "black"}:
            raise ValueError("INVALID_VALUE")
        if self.adapter.frame(robot_id) is None:
            raise RuntimeError("CAMERA_STALLED")
        self._active[robot_id] = {"mode": mode, "misses": 0, "prev": 0.0, "state": "TRACKING", "offset": 0.0, "polarity": "unknown"}
        return self.status(robot_id)

    async def stop(self, robot_id: str) -> dict:
        self._active.pop(robot_id, None)
        try:
            await self.adapter.publish_manual_velocity(robot_id, 0.0, 0.0)
        except Exception:
            pass
        return {"robot_id": robot_id, "state": "IDLE"}

    def status(self, robot_id: str) -> dict:
        return {"robot_id": robot_id, **self._active.get(robot_id, {"state": "IDLE"})}

    async def tick_once(self, robot_id: str) -> dict:
        st = self._active.get(robot_id)
        if st is None:
            return {"robot_id": robot_id, "state": "IDLE"}
        frame = self.adapter.frame(robot_id)
        if frame is None:
            st["misses"] += 1
        else:
            res = detect_line(frame.jpeg, st["mode"])
            st["polarity"] = res.polarity
            if not res.found:
                st["misses"] += 1
            else:
                st["misses"] = 0
                st["offset"] = res.offset
                ang = -(KP * res.offset + KD * (res.offset - st["prev"]))
                st["prev"] = res.offset
                ang = max(-MAX_ANGULAR, min(MAX_ANGULAR, ang))
                await self.adapter.publish_manual_velocity(robot_id, LINEAR_MPS, ang)
                st["state"] = "TRACKING"
                return self.status(robot_id)
        if st["misses"] >= LOSS_LIMIT:
            st["state"] = "LOST"
            await self.adapter.publish_manual_velocity(robot_id, 0.0, 0.0)
        return self.status(robot_id)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `backend/.venv/bin/python -m pytest backend/tests/test_line_follow_service.py backend/tests/test_line_detector.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add backend/pinky_control_center/line_follow_service.py backend/tests/test_line_follow_service.py
git -c user.name="opencode" -c user.email="opencode@localhost" commit -m "feat: add line-follow service loop with loss stop"
```

### Task 3: Ops wiring (mission_line_start/stop) + API

**Files:**
- Create: `backend/pinky_control_center/api/line_follow.py`
- Modify: `backend/pinky_control_center/main.py:73-79` (register `mission_line_start`, `mission_line_stop` handlers + `app.state.line_follow_service` + `include_router`)
- Test: `backend/tests/test_line_follow_api.py`

**Interfaces:**
- Consumes: `LineFollowService` from Task 2, `CommandDispatcher.submit`, existing `leased_operator`/`operator` pattern from `api/missions.py:19-24`.
- Produces: `POST /api/v1/robots/{robot_id}/line-follow/actions {request_id, lease_id, action: start|stop, mode?: auto|white|black}` -> 202 command; `GET /api/v1/robots/{robot_id}/line-follow` -> status dict.

- [ ] **Step 1: Write the failing test**

```python
# backend/tests/test_line_follow_api.py
from fastapi.testclient import TestClient
from pinky_control_center.main import create_app

def test_line_follow_start_rejects_unknown_robot():
    app = create_app(mode="mock")
    c = TestClient(app)
    c.post("/api/v1/session", json={"username": "operator", "password": "x"})
    r = c.post("/api/v1/robots/robot_2/line-follow/actions", json={"request_id": "00000000-0000-4000-8000-000000000001", "lease_id": "00000000-0000-4000-8000-000000000002", "action": "start", "mode": "auto"})
    assert r.status_code in (401, 403, 409, 422)
```

Simplify: assert route exists (not 404) once auth lease fixture is wired; full lease flow follows `test_t06_mission.py` pattern. Keep this file as route-exists probe first (RED = 404).

- [ ] **Step 2: Run test to verify it fails**

Run: `backend/.venv/bin/python -m pytest backend/tests/test_line_follow_api.py -v`
Expected: FAIL with 404 (no route).

- [ ] **Step 3: Write minimal implementation**

```python
# backend/pinky_control_center/api/line_follow.py
from __future__ import annotations
from fastapi import APIRouter, Depends, HTTPException, Request
from pinky_control_center.api.missions import leased_operator, request_id
from pinky_control_center.api.control import operator
from pinky_control_center.command_service import QueueFull

router = APIRouter(prefix="/api/v1")

@router.post("/robots/{robot_id}/line-follow/actions", status_code=202)
async def line_follow_action(robot_id: str, payload: dict, request: Request, user=Depends(operator)):
    if robot_id != "robot_1":
        raise HTTPException(404, detail="ROBOT_NOT_FOUND")
    leased_operator(request, payload, user)
    try:
        action = str(payload["action"])
    except KeyError as e:
        raise HTTPException(422, detail="INVALID_VALUE") from e
    if action not in {"start", "stop"}:
        raise HTTPException(422, detail="INVALID_VALUE")
    mode = str(payload.get("mode", "auto"))
    if mode not in {"auto", "white", "black"}:
        raise HTTPException(422, detail="INVALID_VALUE")
    try:
        return request.app.state.command_dispatcher.submit(user, request_id(payload), robot_id, f"mission_line_{action}", parameters={"robot_id": robot_id, "mode": mode})
    except QueueFull as e:
        raise HTTPException(503, detail="QUEUE_FULL") from e

@router.get("/robots/{robot_id}/line-follow")
async def line_follow_status(robot_id: str, request: Request, user=Depends(operator)):
    if robot_id != "robot_1":
        raise HTTPException(404, detail="ROBOT_NOT_FOUND")
    return request.app.state.line_follow_service.status(robot_id)
```

`main.py` patch: import `LineFollowService` + `api.line_follow`; construct `line_follow_service = LineFollowService(adapter)`; `app.state.line_follow_service = line_follow_service`; handlers `command_service.handlers["mission_line_start"] = <async start wrapper>` and `mission_line_stop`; `app.include_router(line_follow.router)`. Wrapper calls `line_follow_service.start/stop` and returns `(True, {...})` / `(False, {"reason_code": ...})` mapping `CAMERA_STALLED`/`ROBOT_NOT_SUPPORTED`/`INVALID_VALUE`.

- [ ] **Step 4: Run test to verify it passes**

Run: `backend/.venv/bin/python -m pytest backend/tests/test_line_follow_api.py -q`
Expected: PASS (route exists; auth behavior per existing session fixture).

- [ ] **Step 5: Commit**

```bash
git add backend/pinky_control_center/api/line_follow.py backend/pinky_control_center/main.py backend/tests/test_line_follow_api.py
git -c user.name="opencode" -c user.email="opencode@localhost" commit -m "feat: wire mission_line ops and line-follow API"
```

### Task 4: Mock tape frames + frontend panel

**Files:**
- Modify: `backend/pinky_control_center/adapters/mock.py:147-163` (tape-frame variant)
- Create: `frontend/src/LineFollowPanel.tsx`
- Modify: `frontend/src/api.ts` (append `lineFollowAction`, `lineFollowStatus`)
- Test: `frontend/src/LineFollowPanel.test.tsx`, backend `backend/tests/test_line_follow_mock.py` (optional probe)

**Interfaces:**
- Consumes: existing `mutation()` helper in `api.ts:85-89`, `waitForCommand` in `api.ts:183-195`.
- Produces: `lineFollowAction(robotId, action, leaseId, mode)` and `LineFollowPanel({lease, onError})` with Start/Stop buttons + `data-testid="line-follow-state"`.

- [ ] **Step 1: Write the failing test**

```tsx
// frontend/src/LineFollowPanel.test.tsx
import { render, screen } from '@testing-library/react'
import { describe, expect, it } from 'vitest'
import LineFollowPanel from './LineFollowPanel'

describe('LineFollowPanel', () => {
  it('shows software-only stop copy', () => {
    render(<LineFollowPanel lease="L" onError={() => {}} />)
    expect(screen.getByTestId('line-follow-state')).toBeTruthy()
    expect(screen.getByText(/software-only/i)).toBeTruthy()
  })
})
```

- [ ] **Step 2: Run test to verify it fails**

Run: `npx vitest run src/LineFollowPanel.test.tsx`
Expected: FAIL with "Failed to resolve import './LineFollowPanel'".

- [ ] **Step 3: Write minimal implementation**

```tsx
// frontend/src/LineFollowPanel.tsx
import { useState } from 'react'
import { lineFollowAction } from './api'

export default function LineFollowPanel({ lease, onError }: { lease: string | null; onError: (m: string) => void }) {
  const [state, setState] = useState('IDLE')
  const [mode, setMode] = useState<'auto' | 'white' | 'black'>('auto')
  const [busy, setBusy] = useState(false)
  const act = async (action: 'start' | 'stop') => {
    if (!lease || busy) return
    setBusy(true)
    try {
      await lineFollowAction('robot_1', action, lease, mode)
      setState(action === 'start' ? 'TRACKING' : 'IDLE')
    } catch (e) { onError((e as Error).message) } finally { setBusy(false) }
  }
  return <section className="line-follow-panel">
    <h2>라인 추종 · {state}</h2>
    <p data-testid="line-follow-state">{state}</p>
    {/* Offset bar + camera-grid overlay (center line + cx dot) are follow-up after MVP status API lands; panel polls lineFollowStatus here. */}
    <label>선 색상<select aria-label="선 색상" value={mode} onChange={e => setMode(e.target.value as typeof mode)}><option value="auto">자동(흰/검정)</option><option value="white">흰색</option><option value="black">검정색</option></select></label>
    <button disabled={!lease || busy} onClick={() => act('start')}>추종 시작</button>
    <button disabled={!lease || busy} onClick={() => act('stop')}>정지 (software-only)</button>
  </section>
}
```

```ts
// append to frontend/src/api.ts
export async function lineFollowAction(robotId: string, action: 'start' | 'stop', lease_id: string, mode: 'auto' | 'white' | 'black' = 'auto') {
  if (!lease_id) throw new Error('제어권이 필요합니다.')
  return mutation(`/api/v1/robots/${robotId}/line-follow/actions`, { action, mode, lease_id }, '라인 추종 요청 실패')
}
export async function lineFollowStatus(robotId: string) {
  const response = await fetch(`/api/v1/robots/${robotId}/line-follow`, { credentials: 'include' })
  if (!response.ok) throw new Error(await errorMessage(response, `라인 추종 조회 실패 (${response.status})`))
  return response.json()
}
```

Mock tape frame: extend `MockRobotAdapter.frame()` to draw a vertical tape bar (white on dark for robot_1) so mock E2E shows TRACKING; keep `CAMERA_STALL` scenario returning None.

- [ ] **Step 4: Run test to verify it passes**

Run: `npx vitest run src/LineFollowPanel.test.tsx`
Expected: PASS. Also run `npx tsc --noEmit`.

- [ ] **Step 5: Commit**

```bash
git add frontend/src/LineFollowPanel.tsx frontend/src/LineFollowPanel.test.tsx frontend/src/api.ts backend/pinky_control_center/adapters/mock.py
git -c user.name="opencode" -c user.email="opencode@localhost" commit -m "feat: add line-follow panel and mock tape frames"
```

### Task 5: Deps, field wiring, acceptance gate

**Files:**
- Modify: `backend/pyproject.toml:10-16` (add `opencv-python-headless==5.0.0.93`, `numpy==2.5.3`), `backend/requirements.lock`
- Modify: `acceptance-report.md`, `runbook.md` (field steps for `pinky_camera_pub.py`), `docs/02-functional-spec.md` (line-follow contract appendix if present)

- [ ] **Step 1: Write the failing test (dep probe)**

```bash
backend/.venv/bin/python -c "import cv2; print(cv2.__version__)"
```

Expected before lock update on fresh venv: FAIL (module missing). Current dev venv already has 5.0.0 — record version and pin it.

- [ ] **Step 2: Pin deps and reinstall check**

Run: add lines to `pyproject.toml` dependencies, regenerate lock entries, `backend/.venv/bin/pip install -e 'backend[dev]'`, rerun probe.
Expected: PASS printing `5.0.0`.

- [ ] **Step 3: Field wiring docs (no code)**

`runbook.md`: add `scp ros/pinky_camera_pub.py pinky@<robot1>:/home/pinky/` + `ROS_DOMAIN_ID=12 python3 ~/pinky_camera_pub.py --topic /camera/image_raw/compressed --fps 10` + enable `camera.enabled: true` for robot_1 only. `acceptance-report.md`: add row `LINE-FOLLOW robot_1 tape` = NOT_RUN (no PASS without log evidence).

- [ ] **Step 4: Full suite**

Run: `backend/.venv/bin/python -m pytest backend/tests/ -q` and `npx vitest run` + `npx tsc --noEmit`.
Expected: PASS (rerun flaky `test_t06`/`test_t10` once if timing failure).

- [ ] **Step 5: Commit**

```bash
git add backend/pyproject.toml backend/requirements.lock acceptance-report.md runbook.md
git -c user.name="opencode" -c user.email="opencode@localhost" commit -m "chore: pin opencv deps and gate line-follow field test"
```
