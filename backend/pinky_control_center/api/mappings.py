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
        action = payload.get("action")
        if not isinstance(action, str) or not action:
            raise HTTPException(422, detail="INVALID_VALUE")
        return request.app.state.mapping_service.action(user, mapping_id, request_id(payload), action, request.app.state.command_dispatcher)
    except QueueFull as error:
        raise HTTPException(503, detail="QUEUE_FULL") from error
