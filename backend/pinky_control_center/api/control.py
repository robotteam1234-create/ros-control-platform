from __future__ import annotations

from uuid import UUID
from fastapi import APIRouter, Depends, HTTPException, Request, Response, WebSocket
from starlette.websockets import WebSocketDisconnect

from pinky_control_center.auth import current_user, verify_mutation, websocket_user
from pinky_control_center.models import ControlLease, LeaseRequest, UserInfo, UserRole
from pinky_control_center.storage import LeaseConflict, LeaseNotFound
from pinky_control_center.command_service import QueueFull

router = APIRouter(prefix="/api/v1")


def operator(request: Request) -> UserInfo:
    user = current_user(request)
    if user.role not in {UserRole.OPERATOR, UserRole.ADMIN}:
        raise HTTPException(status_code=403, detail="FORBIDDEN")
    verify_mutation(request, user)
    return user


@router.post("/control-lease", response_model=ControlLease)
async def acquire(payload: LeaseRequest, request: Request, user: UserInfo = Depends(operator)) -> ControlLease:
    try:
        return request.app.state.storage.acquire_lease(user, payload.request_id, request.cookies.get("cc_session", ""))
    except LeaseConflict as error:
        raise HTTPException(status_code=409, detail="CONTROL_CONFLICT") from error


@router.patch("/control-lease/{lease_id}", response_model=ControlLease)
async def renew(lease_id: str, payload: LeaseRequest, request: Request, user: UserInfo = Depends(operator)) -> ControlLease:
    from uuid import UUID
    try:
        return request.app.state.storage.renew_lease(UUID(lease_id), user, payload.request_id, request.cookies.get("cc_session", ""))
    except (LeaseNotFound, ValueError) as error:
        raise HTTPException(status_code=409, detail="CONTROL_CONFLICT") from error


@router.delete("/control-lease/{lease_id}", status_code=204)
async def release(lease_id: str, request: Request, user: UserInfo = Depends(operator)) -> Response:
    from uuid import UUID
    try:
        request.app.state.storage.release_lease(UUID(lease_id), user, request.cookies.get("cc_session", ""))
    except (LeaseNotFound, ValueError) as error:
        raise HTTPException(status_code=409, detail="CONTROL_CONFLICT") from error
    return Response(status_code=204)


@router.post("/stop", status_code=202)
async def stop(payload: dict[str, object], request: Request, user: UserInfo = Depends(operator)) -> dict[str, object]:
    try:
        request_id, target = UUID(str(payload["request_id"])), str(payload["target"])
    except (KeyError, ValueError) as error:
        raise HTTPException(status_code=422, detail="INVALID_VALUE") from error
    if target not in {"all", "robot_1", "robot_2"}:
        raise HTTPException(status_code=404, detail="ROBOT_NOT_FOUND")
    try:
        result = request.app.state.command_dispatcher.submit(user, request_id, target, "stop", priority=True)
        robot_ids = ["robot_1", "robot_2"] if target == "all" else [target]
        request.app.state.safety_service.request(robot_ids)
        result["targets"] = [{"robot_id": robot_id, "state": "REQUESTED"} for robot_id in robot_ids]
        return result
    except QueueFull as error:
        raise HTTPException(status_code=503, detail="QUEUE_FULL") from error
    except ValueError as error:
        raise HTTPException(status_code=409, detail="REQUEST_ID_CONFLICT") from error


@router.post("/stop/reset", status_code=202)
async def reset_stop(payload: dict[str, object], request: Request, user: UserInfo = Depends(operator)) -> dict[str, object]:
    try:
        request_id, target, lease_id = UUID(str(payload["request_id"])), str(payload["target"]), UUID(str(payload["lease_id"]))
        if target not in {"all", "robot_1", "robot_2"}:
            raise ValueError
    except (KeyError, ValueError) as error:
        raise HTTPException(status_code=409, detail="CONTROL_CONFLICT") from error
    try:
        if not request.app.state.storage.owns_lease(lease_id, user, request.cookies.get("cc_session", "")):
            raise PermissionError("CONTROL_CONFLICT")
        return request.app.state.command_dispatcher.submit(user, request_id, target, "reset_stop")
    except PermissionError as error:
        raise HTTPException(status_code=409, detail="CONTROL_CONFLICT") from error
    except QueueFull as error:
        raise HTTPException(status_code=503, detail="QUEUE_FULL") from error


@router.post("/robots/{robot_id}/mode", status_code=202)
async def set_mode(robot_id: str, payload: dict[str, object], request: Request, user: UserInfo = Depends(operator)) -> dict[str, object]:
    if robot_id not in {"robot_1", "robot_2"}:
        raise HTTPException(status_code=404, detail="ROBOT_NOT_FOUND")
    try:
        request_id, mode = UUID(str(payload["request_id"])), str(payload["mode"])
    except (KeyError, ValueError) as error:
        raise HTTPException(status_code=422, detail="INVALID_VALUE") from error
    if mode not in {"IDLE", "AUTO", "FOLLOW", "MANUAL", "STOPPED"}:
        raise HTTPException(status_code=422, detail="INVALID_VALUE")
    if mode != "STOPPED":
        for robot in request.app.state.state_store.snapshot().robots:
            if robot.robot_id == robot_id and robot.stop_latched:
                raise HTTPException(status_code=409, detail="STOP_LATCHED")
    try:
        return request.app.state.command_dispatcher.submit(user, request_id, robot_id, "set_mode", parameters={"mode": mode})
    except ValueError as error:
        raise HTTPException(status_code=409, detail="REQUEST_ID_CONFLICT") from error
    except QueueFull as error:
        raise HTTPException(status_code=503, detail="QUEUE_FULL") from error


@router.get("/commands/{command_id}")
async def command(command_id: str, request: Request, user: UserInfo = Depends(current_user)) -> dict[str, object]:
    result = request.app.state.storage.command(command_id, user)
    if result is None:
        raise HTTPException(status_code=404, detail="COMMAND_NOT_FOUND")
    return result


@router.websocket("/ws/teleop")
async def teleop(websocket: WebSocket) -> None:
    user = await websocket_user(websocket)
    if user is None:
        return
    await websocket.accept()
    last_seq = -1
    active_robot_id = "robot_1"
    while True:
        try:
            payload = await websocket.receive_json()
        except WebSocketDisconnect:
            await websocket.app.state.protective_stop(active_robot_id)
            return
        try:
            lease_id = UUID(str(payload["lease_id"]))
            robot_id, seq = str(payload["robot_id"]), int(payload["seq"])
            linear, angular = float(payload["linear_mps"]), float(payload["angular_rps"])
        except (KeyError, ValueError, TypeError):
            await websocket.send_json({"type": "rejected", "reason_code": "INVALID_VALUE"})
            continue
        limits = websocket.app.state.settings_service.current()
        if robot_id not in {"robot_1", "robot_2"}:
            await websocket.send_json({"type": "rejected", "reason_code": "INVALID_VALUE"})
        elif abs(linear) > limits.max_linear_mps or abs(angular) > limits.max_angular_rps:
            await websocket.send_json({"type": "rejected", "reason_code": "SETTINGS_SPEED_LIMIT"})
        elif seq <= last_seq:
            await websocket.send_json({"type": "rejected", "reason_code": "OUT_OF_ORDER"})
        elif not websocket.app.state.storage.owns_lease(lease_id, user, websocket.cookies.get("cc_session", "")):
            await websocket.send_json({"type": "rejected", "reason_code": "CONTROL_CONFLICT"})
        else:
            last_seq = seq
            active_robot_id = robot_id
            service = websocket.app.state.teleop_service
            try:
                service.enter(robot_id, lease_valid=True)
            except PermissionError:
                await websocket.send_json({"type": "rejected", "reason_code": "CONTROL_CONFLICT"}); continue
            result = service.ingest(robot_id, seq, linear, angular, limits.max_linear_mps, limits.max_angular_rps)
            if result == "ACCEPTED":
                publish = getattr(websocket.app.state.adapter, "publish_manual_velocity", None)
                if publish is not None:
                    accepted = await publish(robot_id, linear, angular)
                    if not accepted.accepted:
                        service.protective_stop(robot_id)
                        await websocket.send_json({"type": "rejected", "reason_code": accepted.reason_code or "ROSBRIDGE_WRITE_FAILED"})
                        continue
            await websocket.send_json({"type": result.lower(), "seq": seq})
