"""Web control client (one function per concern): session, drive patterns, runner."""

from __future__ import annotations

import asyncio
import json
import uuid

import httpx
import websockets

BASE = "http://127.0.0.1:8081"
ORIGIN = "http://127.0.0.1:4173"


def new_request_id() -> str:
    return str(uuid.uuid4())


def teleop_frame(lease_id: str, robot_id: str, seq: int, linear: float, angular: float) -> str:
    return json.dumps(
        {
            "lease_id": lease_id,
            "robot_id": robot_id,
            "seq": seq,
            "linear_mps": linear,
            "angular_rps": angular,
        }
    )


async def login(client: httpx.AsyncClient, username: str, password: str) -> str:
    r = await client.post(
        "/api/v1/session",
        json={"username": username, "password": password},
        headers={"Origin": ORIGIN},
    )
    r.raise_for_status()
    return r.json()["csrf_token"]


async def acquire_lease(client: httpx.AsyncClient, headers: dict) -> str:
    r = await client.post("/api/v1/control-lease", json={"request_id": new_request_id()}, headers=headers)
    r.raise_for_status()
    return r.json()["lease_id"]


async def reset_and_manual(client: httpx.AsyncClient, headers: dict, lease_id: str, robot_id: str) -> None:
    r = await client.post(
        "/api/v1/stop/reset",
        json={"request_id": new_request_id(), "target": robot_id, "lease_id": lease_id},
        headers=headers,
    )
    r.raise_for_status()
    for _ in range(30):
        await asyncio.sleep(2)
        st = await client.get("/api/v1/state", headers=headers)
        st.raise_for_status()
        latches = {r["robot_id"]: r["stop_latched"] for r in st.json()["robots"]}
        if not latches.get(robot_id, True):
            break
    else:
        raise RuntimeError(f"{robot_id} latch did not clear")
    lease_id = await acquire_lease(client, headers)
    r = await client.post(
        f"/api/v1/robots/{robot_id}/mode",
        json={"request_id": new_request_id(), "mode": "MANUAL"},
        headers=headers,
    )
    r.raise_for_status()
    lease_id = await acquire_lease(client, headers)
    return lease_id


async def stream_velocity(uri: str, cookies: str, origin: str, lease_id: str, robot_id: str, linear: float, angular: float, secs: float) -> int:
    count = 0
    async with websockets.connect(uri, additional_headers=[("Cookie", cookies), ("Origin", origin)]) as ws:
        end = asyncio.get_event_loop().time() + secs
        seq = 0
        while asyncio.get_event_loop().time() < end:
            seq += 1
            await ws.send(teleop_frame(lease_id, robot_id, seq, linear, angular))
            try:
                await asyncio.wait_for(ws.recv(), timeout=2)
            except asyncio.TimeoutError:
                break
            count += 1
            await asyncio.sleep(0.1)
    return count
