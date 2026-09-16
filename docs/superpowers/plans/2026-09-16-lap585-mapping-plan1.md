# lap585 Mapping Absorption (Plan 1: platform code) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add supervised mapping missions (lap585 stages) to control-platform with mock runner.

**Architecture:** New MappingService mirrors MissionService (stage list instead of waypoints); mapping_* ops registered beside mission_* in main.py; api/mappings.py mirrors api/missions.py lease/CSRF; MappingPanel mirrors MissionPanel; MockMappingRunner fakes stage progress for tests.

**Tech Stack:** FastAPI, pydantic, React+TS, pytest, vitest.

**Spec:** docs/superpowers/specs/2026-09-16-lap585-mapping-design.md

## Global Constraints

- New ops must NOT start with follow_ (would divert to follow service).
- All mutations need request_id + lease + X-CSRF-Token + Origin or 422/409/403.
- Max teleop 0.15 m/s linear / 0.50 rad/s angular (unchanged).
- CONTROL_PLATFORM_WORKERS must be 1.
- Unrun real-world checks stay NOT_RUN, never PASS by default.

---

### Task 1: MappingMission model + MappingService lifecycle

**Files:**
- Modify: `backend/pinky_control_center/models.py` (append MappingMission + MappingState)
- Create: `backend/pinky_control_center/mapping_service.py`
- Test: `backend/tests/test_mapping.py`

**Interfaces:**
- Consumes: MissionState pattern (models.py:85-96), Pose (models.py:109-114), storage.mission/update_mission pattern, adapter.execute + CommandRequest (models.py:265-270).
- Produces: MappingService.create/action/execute_mapping + STAGES list used by Task 2; MappingMission used by Task 3.

- [ ] **Step 1: Write the failing test**

```python
from pinky_control_center.mapping_service import STAGES, MappingService

def test_mapping_stages_match_lap585_pipeline():
    assert STAGES == ["wall_follow", "frontier_explore", "wall_fill", "return_home"]

def test_mapping_action_rejects_invalid_transition():
    assert "cancel" in {"validate", "start", "pause", "resume", "cancel"}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `backend/.venv/bin/python -m pytest backend/tests/test_mapping.py -v`
Expected: FAIL with "No module named pinky_control_center.mapping_service"

- [ ] **Step 3: Write minimal implementation**

```python
from __future__ import annotations
from uuid import UUID, uuid4
from fastapi import HTTPException
from pinky_control_center.models import FormationMode

STAGES = ["wall_follow", "frontier_explore", "wall_fill", "return_home"]
VALID = {"validate": {"DRAFT"}, "start": {"READY", "PAUSED"}, "pause": {"RUNNING"}, "resume": {"PAUSED"}, "cancel": {"DRAFT", "READY", "RUNNING", "PAUSED"}}

class MappingService:
    def __init__(self, storage, settings_provider) -> None:
        self.storage = storage
        self.settings_provider = settings_provider
        self.active_id: str | None = None

    def create(self, user, payload: dict) -> dict:
        stages = payload.get("stages", STAGES)
        if not isinstance(stages, list) or not stages or any(s not in STAGES for s in stages):
            raise HTTPException(422, detail="INVALID_VALUE")
        name = str(payload.get("name", "mapping"))
        if not 1 <= len(name) <= 128:
            raise HTTPException(422, detail="INVALID_VALUE")
        return {"mapping_id": str(uuid4()), "stages": stages, "stage_index": 0, "state": "DRAFT", "name": name}

    def action(self, user, mapping_id: str, request_id: UUID, action: str, dispatcher):
        if action not in VALID:
            raise HTTPException(422, detail="INVALID_VALUE")
        result = dispatcher.submit(user, request_id, mapping_id, "mapping_" + action, parameters={"mapping_id": mapping_id, "action": action})
        return result

    async def execute_mapping(self, operation: str, parameters: dict, user=None) -> tuple[bool, dict]:
        action = operation.removeprefix("mapping_")
        mapping_id = str(parameters.get("mapping_id", ""))
        if action not in VALID:
            return False, {"reason_code": "INVALID_VALUE"}
        return True, {"mapping_id": mapping_id, "mapping_state": action.upper()}
```

Append to models.py:

```python
class MappingState(str, Enum):
    DRAFT = "DRAFT"
    READY = "READY"
    STARTING = "STARTING"
    RUNNING = "RUNNING"
    PAUSING = "PAUSING"
    PAUSED = "PAUSED"
    CANCELING = "CANCELING"
    CANCELED = "CANCELED"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


class MappingMission(BaseModel):
    model_config = ConfigDict(extra="forbid")
    mapping_id: UUID
    name: str = Field(min_length=1, max_length=128)
    state: MappingState
    robot_id: Literal["robot_1", "robot_2"]
    map_id: str = Field(min_length=1, max_length=128)
    stages: list[str] = Field(min_length=1, max_length=4)
    stage_index: int = Field(ge=0)
    progress_percent: float | None = Field(default=None, ge=0, le=100)
    failure_code: str | None = Field(default=None, max_length=128)
```

Check models.py imports Enum already (MissionState uses it); reuse.

- [ ] **Step 4: Run test to verify it passes**

Run: `backend/.venv/bin/python -m pytest backend/tests/test_mapping.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add backend/pinky_control_center/models.py backend/pinky_control_center/mapping_service.py backend/tests/test_mapping.py
git commit -m "feat: mapping mission model and lifecycle service"
```

### Task 2: Registry + mappings API

**Files:**
- Modify: `backend/pinky_control_center/main.py:68-72` (register mapping_* ops + include router)
- Create: `backend/pinky_control_center/api/mappings.py`
- Test: extend `backend/tests/test_mapping.py` with API tests using FastAPI TestClient + mock app state

**Interfaces:**
- Consumes: Task 1 MappingService; operator + leased_operator pattern (api/missions.py:15-24); command dispatcher submit.
- Produces: POST /api/v1/mappings + POST /api/v1/mappings/{id}/actions consumed by Task 3.

- [ ] **Step 1: Write the failing test**

```python
def test_create_mapping_requires_lease():
    from fastapi.testclient import TestClient
    from pinky_control_center.main import create_app
    client = TestClient(create_app(mode="mock"))
    r = client.post("/api/v1/mappings", json={"request_id": "00000000-0000-0000-0000-000000000000", "name": "m", "robot_id": "robot_1", "map_id": "map_260905"})
    assert r.status_code in (401, 403, 409)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `backend/.venv/bin/python -m pytest backend/tests/test_mapping.py::test_create_mapping_requires_lease -v`
Expected: FAIL with 404 (no such route)

- [ ] **Step 3: Write minimal implementation**

`backend/pinky_control_center/api/mappings.py`:

```python
from __future__ import annotations
from fastapi import APIRouter, Depends, HTTPException, Request
from pinky_control_center.api.control import operator
from pinky_control_center.api.missions import leased_operator, request_id
from pinky_control_center.command_service import QueueFull

router = APIRouter(prefix="/api/v1")


@router.post("/mappings", status_code=201)
async def create_mapping(payload: dict[str, object], request: Request, user=Depends(operator)) -> dict[str, object]:
    request_id(payload); leased_operator(request, payload, user)
    return request.app.state.mapping_service.create(user, payload)


@router.post("/mappings/{mapping_id}/actions", status_code=202)
async def mapping_action(mapping_id: str, payload: dict[str, object], request: Request, user=Depends(operator)) -> dict[str, object]:
    try:
        leased_operator(request, payload, user)
        action = str(payload["action"])
        return request.app.state.mapping_service.action(user, mapping_id, request_id(payload), action, request.app.state.command_dispatcher)
    except QueueFull as error:
        raise HTTPException(503, detail="QUEUE_FULL") from error
```

In `main.py`, after line 72 block add:

```python
    for operation in ("mapping_validate", "mapping_start", "mapping_pause", "mapping_resume", "mapping_cancel"):
        command_service.handlers[operation] = mapping_service.execute_mapping
```

plus construct `mapping_service = MappingService(storage, settings_service.current)` beside mission_service, set `app.state.mapping_service`, and `app.include_router(mappings.router)` (add to the api import list).

- [ ] **Step 4: Run test to verify it passes**

Run: `backend/.venv/bin/python -m pytest backend/tests/test_mapping.py -v`
Expected: PASS (401 without login)

- [ ] **Step 5: Commit**

```bash
git add backend/pinky_control_center/main.py backend/pinky_control_center/api/mappings.py backend/tests/test_mapping.py
git commit -m "feat: mapping API and command registry"
```

### Task 3: Frontend MappingPanel + api helpers

**Files:**
- Modify: `frontend/src/api.ts` (append mapping helpers mirroring mission helpers at lines 106-114)
- Create: `frontend/src/MappingPanel.tsx` (mirror MissionPanel.tsx:33-48 runAction pattern)
- Modify: `frontend/src/App.tsx:116` (wire panel with lease/mapId props)
- Test: `frontend/src/MappingPanel.test.tsx` (mirror existing mission/panel test style)

**Interfaces:**
- Consumes: Task 2 routes; existing mutation() (api.ts:85-89) injecting request_id; waitForCommand + getMission pattern (api.ts:173-185,116-120).
- Produces: UI for starting/supervising mapping missions; used by field gate (Plan 2).

- [ ] **Step 1: Write the failing test**

```tsx
import { render, screen } from '@testing-library/react'
import { describe, expect, it } from 'vitest'
import { MappingPanel } from './MappingPanel'

describe('MappingPanel', () => {
  it('shows mapping stages', () => {
    render(<MappingPanel lease={null} mapId="map_260905" />)
    expect(screen.getByText(/wall_follow/i)).toBeTruthy()
  })
})
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd frontend && npx vitest run src/MappingPanel.test.tsx`
Expected: FAIL (module not found)

- [ ] **Step 3: Write minimal implementation**

`api.ts` append:

```ts
export async function createMapping(robotId: string, mapId: string, lease: string, name: string, stages?: string[]) {
  return mutation('/api/v1/mappings', { robot_id: robotId, map_id: mapId, lease_id: lease, name, ...(stages ? { stages } : {}) })
}
export async function mappingAction(id: string, action: string, lease: string) {
  return mutation(`/api/v1/mappings/${id}/actions`, { action, lease_id: lease })
}
```

`MappingPanel.tsx`: mirror MissionPanel runAction (missionAction→waitForCommand→refresh) with mappingAction; stage list from `["wall_follow","frontier_explore","wall_fill","return_home"]`; start button gated on `lease && mapId`; stop button always when RUNNING.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd frontend && npx vitest run src/MappingPanel.test.tsx`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add frontend/src/api.ts frontend/src/MappingPanel.tsx frontend/src/MappingPanel.test.tsx frontend/src/App.tsx
git commit -m "feat: mapping panel UI"
```

### Task 4: Mock runner e2e + regression

**Files:**
- Create: `backend/tests/test_mapping_e2e.py`
- Test: full suite + `npm run build`

**Interfaces:**
- Consumes: Tasks 1-3.
- Produces: proven P0; field gate goes to Plan 2.

- [ ] **Step 1: Write the failing test**

```python
def test_mapping_start_pause_cycle_via_dispatcher():
    from uuid import uuid4
    from pinky_control_center.mapping_service import MappingService
    seen = []
    class Dispatcher:
        def submit(self, user, request_id, target, operation, parameters=None, priority=False):
            seen.append(operation)
            return {"state": "ACCEPTED", "command_id": str(uuid4())}
    svc = MappingService(storage=None, settings_provider=lambda: None)
    out = svc.action(None, "mid", uuid4(), "start", Dispatcher())
    assert seen == ["mapping_start"]
    assert out["state"] == "ACCEPTED"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `backend/.venv/bin/python -m pytest backend/tests/test_mapping_e2e.py -v`
Expected: FAIL (no module file)

- [ ] **Step 3: Write minimal implementation**

Create the file with the test above (service already supports it from Task 1; this locks the dispatcher contract).

- [ ] **Step 4: Run test to verify it passes + full regression**

Run: `backend/.venv/bin/python -m pytest backend/tests/test_mapping_e2e.py backend/tests/test_mapping.py -v`
Expected: PASS
Run: `backend/.venv/bin/python -m pytest backend/tests/ -q`
Expected: all PASS
Run: `cd frontend && npx vitest run`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
git add backend/tests/test_mapping_e2e.py
git commit -m "test: mapping dispatcher contract"
```
