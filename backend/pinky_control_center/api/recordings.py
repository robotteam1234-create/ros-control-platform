"""Dataset recording HTTP API (thin routes over RecordingService)."""

from __future__ import annotations

from uuid import UUID

from fastapi import APIRouter, Depends, HTTPException, Request

from pinky_control_center.api.control import operator
from pinky_control_center.api.missions import leased_operator, request_id
from pinky_control_center.auth import verify_mutation
from pinky_control_center.models import UserInfo

router = APIRouter(prefix="/api/v1")


def _service(request: Request):
    return request.app.state.recording_service


@router.post("/recordings/start", status_code=201)
async def start_recording(payload: dict[str, object], request: Request, user: UserInfo = Depends(operator)) -> dict[str, object]:
    verify_mutation(request, user)
    leased_operator(request, payload, user)
    request_id(payload)
    robot_id = str(payload.get("robot_id", ""))
    label = str(payload.get("label", "run"))
    if robot_id not in {"robot_1", "robot_2"}:
        raise HTTPException(status_code=404, detail="ROBOT_NOT_FOUND")
    try:
        return _service(request).start(robot_id, label)
    except ValueError as error:
        raise HTTPException(status_code=422, detail="INVALID_VALUE") from error


@router.post("/recordings/stop")
async def stop_recording(payload: dict[str, object], request: Request, user: UserInfo = Depends(operator)) -> dict[str, object]:
    verify_mutation(request, user)
    leased_operator(request, payload, user)
    request_id(payload)
    robot_id = str(payload.get("robot_id", ""))
    if robot_id not in {"robot_1", "robot_2"}:
        raise HTTPException(status_code=404, detail="ROBOT_NOT_FOUND")
    try:
        return _service(request).stop(robot_id)
    except KeyError as error:
        raise HTTPException(status_code=409, detail="CONTROL_CONFLICT") from error


@router.get("/recordings")
async def recording_status(request: Request, user: UserInfo = Depends(operator)) -> dict[str, object]:
    return {"active": _service(request).active()}
