# backend/pinky_control_center/api/mappings.py
from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, Request

from pinky_control_center.api.control import operator
from pinky_control_center.auth import current_user
from pinky_control_center.models import UserInfo
from pinky_control_center.api.missions import leased_operator, request_id

router = APIRouter(prefix="/api/v1")


@router.post("/mappings", status_code=201)
async def create_mapping(payload: dict[str, object], request: Request, user=Depends(operator)) -> dict[str, object]:
    mapping_service = request.app.state.mapping_service
    return mapping_service.create(user, payload)


@router.post("/mappings/{mapping_id}/actions", status_code=202)
async def mapping_action(mapping_id: str, payload: dict[str, object], request: Request, user: UserInfo = Depends(operator)) -> dict[str, object]:
    leased_operator(request, payload, user)
    action = str(payload.get("action", ""))
    mapping_service = request.app.state.mapping_service
    return mapping_service.action(user, mapping_id, request_id(payload), action, request.app.state.command_dispatcher, robot_id=str(payload.get("robot_id", "robot_1")), lease_id=str(payload.get("lease_id") or ""))


@router.get("/mappings/{mapping_id}")
async def mapping_status(mapping_id: str, request: Request, user=Depends(operator)) -> dict[str, object]:
    runner = getattr(request.app.state, "mapping_runner", None)
    status = runner.status() if runner is not None else {"state": "IDLE"}
    return {"mapping_id": mapping_id, **status}


@router.get("/mappings/{mapping_id}/log")
async def mapping_log(mapping_id: str, request: Request, tail: int = 100, user=Depends(operator)) -> dict[str, object]:
    runner = getattr(request.app.state, "mapping_runner", None)
    if runner is None:
        raise HTTPException(status_code=409, detail="MAPPING_RUNNER_ERROR")
    return {"mapping_id": mapping_id, "lines": runner.log_tail(tail)}


@router.post("/mappings/{mapping_id}/import", status_code=201)
async def mapping_import(mapping_id: str, payload: dict[str, object], request: Request, user: UserInfo = Depends(operator)) -> dict[str, object]:
    request_id(payload)
    leased_operator(request, payload, user)
    importer = getattr(request.app.state, "mapping_import", None)
    result = importer() if importer else None
    if result is None:
        raise HTTPException(status_code=409, detail="MAPPING_NOT_SAVED")
    return {"mapping_id": mapping_id, **result}
