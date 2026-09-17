from __future__ import annotations

from collections.abc import Awaitable, Callable
from typing import Any

from fastapi import APIRouter, Depends, HTTPException, Request

from pinky_control_center.api.control import operator
from pinky_control_center.api.missions import leased_operator, request_id
from pinky_control_center.auth import current_user
from pinky_control_center.command_service import QueueFull

router = APIRouter(prefix="/api/v1")

_VALID_ACTIONS = {"start", "stop"}
_VALID_MODES = {"auto", "white", "black"}


@router.post("/robots/{robot_id}/line-follow/actions", status_code=202)
async def line_follow_action(robot_id: str, payload: dict, request: Request, user=Depends(operator)):
    if robot_id != "robot_1":
        raise HTTPException(404, detail="ROBOT_NOT_FOUND")
    leased_operator(request, payload, user)
    try:
        action = str(payload["action"])
    except KeyError as e:
        raise HTTPException(422, detail="INVALID_VALUE") from e
    if action not in _VALID_ACTIONS:
        raise HTTPException(422, detail="INVALID_VALUE")
    mode = str(payload.get("mode", "auto"))
    if mode not in _VALID_MODES:
        raise HTTPException(422, detail="INVALID_VALUE")
    try:
        return request.app.state.command_dispatcher.submit(user, request_id(payload), robot_id, f"mission_line_{action}", parameters={"robot_id": robot_id, "mode": mode})
    except QueueFull as e:
        raise HTTPException(503, detail="QUEUE_FULL") from e


@router.get("/robots/{robot_id}/line-follow")
async def line_follow_status(robot_id: str, request: Request, user=Depends(current_user)):
    if robot_id != "robot_1":
        raise HTTPException(404, detail="ROBOT_NOT_FOUND")
    return request.app.state.line_follow_service.status(robot_id)


Handler = Callable[[str, dict[str, Any]], Awaitable[tuple[bool, dict[str, Any]]]]


def build_line_start_handler(service) -> Handler:
    async def execute_line_start(operation: str, parameters: dict[str, Any], user=None) -> tuple[bool, dict[str, Any]]:
        params = parameters or {}
        robot_id = str(params.get("robot_id", "robot_1"))
        mode = str(params.get("mode", "auto"))
        if robot_id != "robot_1":
            return False, {"reason_code": "ROBOT_NOT_FOUND"}
        if mode not in _VALID_MODES:
            return False, {"reason_code": "INVALID_VALUE"}
        try:
            status = await service.start(robot_id, mode)
            return True, {"line_follow": status}
        except ValueError as error:
            return False, {"reason_code": str(error) or "INVALID_VALUE"}
        except RuntimeError as error:
            return False, {"reason_code": str(error) or "CAMERA_STALLED"}

    return execute_line_start


def build_line_stop_handler(service) -> Handler:
    async def execute_line_stop(operation: str, parameters: dict[str, Any], user=None) -> tuple[bool, dict[str, Any]]:
        params = parameters or {}
        robot_id = str(params.get("robot_id", "robot_1"))
        if robot_id != "robot_1":
            return False, {"reason_code": "ROBOT_NOT_FOUND"}
        status = await service.stop(robot_id)
        return True, {"line_follow": status}

    return execute_line_stop
