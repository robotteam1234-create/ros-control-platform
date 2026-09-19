# backend/pinky_control_center/api/mapping_stream.py
from __future__ import annotations

from fastapi import APIRouter, WebSocket

from pinky_control_center.auth import websocket_user

SUPPORTED_ROBOTS = ("robot_1",)


def create_router(map_stream_service) -> APIRouter:
    router = APIRouter()

    @router.websocket("/ws/mapping/{robot_id}")
    async def mapping_socket(websocket: WebSocket, robot_id: str) -> None:
        if await websocket_user(websocket) is None:
            return
        if robot_id not in SUPPORTED_ROBOTS:
            await websocket.close(code=4404)
            return
        await websocket.accept()
        queue = map_stream_service.subscribe(robot_id)
        try:
            latest = map_stream_service.latest(robot_id)
            if latest is not None:
                await websocket.send_bytes(latest)
            while True:
                await websocket.send_bytes(await queue.get())
        except Exception:
            pass
        finally:
            map_stream_service.unsubscribe(robot_id, queue)

    return router
