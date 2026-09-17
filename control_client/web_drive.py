"""Run drive patterns on robots through the web API (lease -> MANUAL -> stream).

Usage: web_drive.py <robot_1|robot_2|both> <pattern> [args...]
  patterns: straight[/count/drive_secs/speed] square[/side/turn/speed/rate]
Examples:
  web_drive.py robot_2 straight
  web_drive.py both square/3/3.2/0.1/0.4
"""

from __future__ import annotations

import asyncio
import sys

import httpx

from control_client.client import (
    BASE,
    ORIGIN,
    acquire_lease,
    login,
    reset_and_manual,
    stream_velocity,
)
from control_client.patterns import square_legs, straight_legs


async def drive_robot(client: httpx.AsyncClient, headers: dict, cookies: str, lease_id: str, robot: str, legs: list) -> str:
    lease_id = await reset_and_manual(client, headers, lease_id, robot)
    uri = BASE.replace("http", "ws") + "/api/v1/ws/teleop"
    for linear, angular, secs in legs:
        n = await stream_velocity(uri, cookies, ORIGIN, lease_id, robot, linear, angular, secs)
        print(robot, f"v={linear} w={angular} t={secs}s msgs={n}", flush=True)
    return lease_id


async def main() -> None:
    target = sys.argv[1] if len(sys.argv) > 1 else "both"
    pattern = sys.argv[2] if len(sys.argv) > 2 else "straight"
    parts = pattern.split("/")
    if parts[0] == "square":
        legs = square_legs(*(float(x) for x in parts[1:])) if len(parts) > 1 else square_legs()
    else:
        legs = straight_legs(*([float(x) for x in parts[1:]] or [3, 2.0, 0.1])) if len(parts) > 1 else straight_legs()
    robots = ("robot_1", "robot_2") if target == "both" else (target,)
    async with httpx.AsyncClient(base_url=BASE) as client:
        csrf = await login(client, "operator", "pinky-operator-1")
        headers = {"Origin": ORIGIN, "X-CSRF-Token": csrf}
        lease_id = await acquire_lease(client, headers)
        cookies = "; ".join(f"{c.name}={c.value}" for c in client.cookies.jar)
        await asyncio.gather(*(drive_robot(client, headers, cookies, lease_id, r, legs) for r in robots))
        print("done.")


if __name__ == "__main__":
    asyncio.run(main())
