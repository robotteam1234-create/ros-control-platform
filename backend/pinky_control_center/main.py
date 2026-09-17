from __future__ import annotations

import argparse
import asyncio
import os
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Literal

import uvicorn
from fastapi import Depends, FastAPI, HTTPException, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse, Response
from starlette.exceptions import HTTPException as StarletteHTTPException

from pinky_control_center.adapters.mock import MockRobotAdapter
from pinky_control_center.adapters.ros import RosbridgeAdapter
from pinky_control_center.api import alerts, cameras, control, history, mappings, maps, missions, navigation, recordings, sensors, session, state, settings
from pinky_control_center.api import line_follow as line_follow_api
from pinky_control_center.auth import current_user, verify_mutation
from pinky_control_center.config import load_mock_config, load_ros_config
from pinky_control_center.models import FormationMode, MockScenario, MockScenarioRequest, UserInfo, UserRole
from pinky_control_center.map_service import MapService
from pinky_control_center.camera_service import CameraService
from pinky_control_center.recording_service import RecordingService
from pinky_control_center.command_service import CommandService
from pinky_control_center.teleop_service import TeleopService
from pinky_control_center.safety_service import SafetyService
from pinky_control_center.state_store import StateStore
from pinky_control_center.storage import Storage, utc_now
from pinky_control_center.mission_service import MissionService
from pinky_control_center.line_follow_service import LineFollowService
from pinky_control_center.mapping_service import MappingService
from pinky_control_center.alert_service import AlertService
from pinky_control_center.settings_service import SettingsService
from pinky_control_center.navigation_service import NavigationService


def default_database_path() -> Path:
    state_home = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local" / "state"))
    return state_home / "control-platform" / "control.db"


def create_app(mode: Literal["mock", "ros"] = "mock", config_path: Path | None = None, database_path: Path | None = None, allowed_origin: str = "http://localhost:5173", secure_cookies: bool = False, monotonic_clock=None, start_command_worker: bool = True, storage_clock=None, start_watchdog: bool = True) -> FastAPI:
    try:
        worker_count = int(os.environ.get("CONTROL_PLATFORM_WORKERS", "1"))
    except ValueError as error:
        raise ValueError("CONTROL_PLATFORM_WORKERS must declare a single worker") from error
    if worker_count != 1:
        raise ValueError("settings application requires a single worker")
    if mode == "mock":
        adapter = MockRobotAdapter(config=load_mock_config(config_path))
    elif mode == "ros":
        adapter = RosbridgeAdapter(load_ros_config(config_path))
    else:
        raise ValueError("mode must be mock or ros")
    lease_events: list[str] = []
    storage = Storage(database_path or default_database_path(), clock=storage_clock or utc_now, lease_end_hook=lease_events.append)
    db_path = Path(database_path or default_database_path())
    recording_service = RecordingService(db_path.parent / "recordings")
    state_store = StateStore(adapter.snapshot)
    map_service = MapService()
    camera_service = CameraService(adapter.frame)
    command_service = CommandService(storage, adapter)
    runtime_clock = monotonic_clock or time.monotonic
    teleop_service = TeleopService(runtime_clock)
    safety_service = SafetyService(runtime_clock)
    alert_service = AlertService(storage)
    settings_service = SettingsService(storage, adapter, map_service, state_store)
    navigation_service = NavigationService(storage, adapter, state_store, map_service, settings_service.current)
    mission_service = MissionService(storage, adapter, state_store, runtime_clock, settings_service.current)
    line_follow_service = LineFollowService(adapter)
    mapping_service = MappingService(storage, settings_service.current)
    state_store.alert_provider = lambda: alert_service.list(state="ACTIVE")
    for operation in ("formation_pair", "formation_start", "formation_pause", "formation_unpair", "formation_rejoin"):
        command_service.handlers[operation] = mission_service.execute_formation
    for operation in ("mission_validate", "mission_start", "mission_resume", "mission_pause", "mission_cancel"):
        command_service.handlers[operation] = mission_service.execute_mission
    for operation in ("mapping_validate", "mapping_start", "mapping_pause", "mapping_resume", "mapping_cancel"):
        command_service.handlers[operation] = mapping_service.execute_mapping
    command_service.handlers["mission_line_start"] = line_follow_api.build_line_start_handler(line_follow_service)
    command_service.handlers["mission_line_stop"] = line_follow_api.build_line_stop_handler(line_follow_service)
    command_service.handlers["navigate_from_map"] = navigation_service.execute

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        await adapter.connect()
        # Existing dashboard settings are mock-only until the robot-side
        # parameter/control contract is supplied.  ROS starts in observation
        # mode instead of claiming that those values reached hardware.
        if mode == "mock":
            await settings_service.apply_current()
        if start_command_worker:
            await command_service.start()
        watchdog_task = asyncio.create_task(watchdog()) if start_watchdog else None
        yield
        if watchdog_task:
            watchdog_task.cancel()
            try:
                await watchdog_task
            except asyncio.CancelledError:
                pass
        await command_service.close()
        await adapter.close()
        await camera_service.close()
        storage.close()

    app = FastAPI(title="Pinky Control Center", version="0.1.0", lifespan=lifespan)
    app.state.adapter = adapter
    app.state.mode = mode
    app.state.storage = storage
    app.state.command_service = command_service
    app.state.command_dispatcher = command_service
    app.state.teleop_service = teleop_service
    app.state.safety_service = safety_service
    app.state.mission_service = mission_service
    app.state.line_follow_service = line_follow_service
    app.state.mapping_service = mapping_service
    app.state.recording_service = recording_service
    app.state.alert_service = alert_service
    app.state.settings_service = settings_service
    app.state.navigation_service = navigation_service
    def refresh_stops() -> None:
        for robot in state_store.snapshot().robots:
            safety_service.observe(robot.robot_id, stop_latched=bool(robot.stop_latched), linear_mps=robot.linear_mps, angular_rps=robot.angular_rps, fresh=robot.pose_freshness.value == "FRESH",
                                   odom_linear_mps=robot.odom_linear_mps, odom_angular_rps=robot.odom_angular_rps, odom_fresh=robot.pose_freshness.value == "FRESH")
        command_service.refresh_stops(safety_service.states())
    app.state.allowed_origin = allowed_origin
    app.state.secure_cookies = secure_cookies
    app.state.state_store = state_store
    state_store.map_id_provider = lambda: settings_service.current().active_map_id

    async def protective_stop(robot_id: str) -> None:
        teleop_service.protective_stop(robot_id)
        state_store.disconnect(robot_id)
        await command_service.protective_stop(robot_id)

    async def zero_teleop_velocity(robot_id: str) -> None:
        """Clear an expired deadman input without latching an operator stop.

        A normal button release or a quiet teleop socket is different from a
        lost connection, lease expiry, or explicit stop.  The robot-side
        watchdog also zeros stale input, so the backend should only refresh
        that zero here.  If the zero cannot be delivered, fall back to the
        latched protective-stop path.
        """
        publish = getattr(adapter, "publish_manual_velocity", None)
        if publish is None:
            return
        try:
            accepted = await publish(robot_id, 0.0, 0.0)
        except Exception:
            accepted = None
        if accepted is None or not accepted.accepted:
            await protective_stop(robot_id)

    async def runtime_tick() -> None:
        refresh_stops()
        for event in getattr(adapter, "drain_events", lambda: [])():
            mission_service.consume_adapter_event(event)
            if event.kind == "command" and event.payload.get("state") == "REJECTED" and event.robot_id:
                alert_service.record_command_rejection(event.robot_id, event.payload.get("reason_code") if isinstance(event.payload.get("reason_code"), str) else None)
        observed = state_store.snapshot()
        # A transitional local override must not hide a fresh adapter LOST state.
        source_formation = state_store.snapshot_source().formation
        if source_formation.state is FormationMode.LOST and observed.formation.state is not FormationMode.LOST:
            observed = observed.model_copy(update={"formation": source_formation})
        newly_active = alert_service.evaluate(observed)
        for alert in newly_active:
            if alert.code in {"COMMUNICATION_LOSS", "COMMUNICATION_STALE", "FOLLOW_LOST", "TF_INVALID", "NAVIGATION_SENSOR_ERROR", "BATTERY_CRITICAL"}:
                stopped, unconfirmed = await mission_service.protective_pause(alert.code)
                if not stopped:
                    alert_service.record_protective_stop_unconfirmed(alert.code, unconfirmed)
        await mission_service.tick()
        for robot_id in line_follow_service.active_robot_ids():
            if not line_follow_service.tick_due(robot_id):
                continue
            await line_follow_service.tick_once(robot_id)
            if line_follow_service.take_safety_stop(robot_id):
                # LOST transition: zero already published by tick_once; route
                # through the latched protective-stop path (no auto-resume).
                storage.record_history_safe(event_type="LINE_FOLLOW_LOST", robot_id=robot_id, payload={"state": "LOST"})
                await protective_stop(robot_id)
        for robot_id in teleop_service.tick():
            await zero_teleop_velocity(robot_id)
        if storage.expire_security():
            # An expired control identity invalidates manual authority for both robots.
            await protective_stop("robot_1")

    async def watchdog() -> None:
        while True:
            await runtime_tick()
            await asyncio.sleep(0.05)

    app.state.runtime_tick = runtime_tick
    app.state.protective_stop = protective_stop
    # T02 records lease end causes only. T05 connects this hook to the safety stop path.
    app.state.lease_events = lease_events
    app.include_router(session.router)
    app.include_router(control.router)
    app.include_router(missions.router)
    app.include_router(line_follow_api.router)
    app.include_router(mappings.router)
    app.include_router(recordings.router)
    app.include_router(navigation.router)
    app.add_api_websocket_route("/ws/teleop", control.teleop)
    app.include_router(state.create_router(state_store))
    app.include_router(maps.create_router(map_service))
    app.include_router(settings.create_router())
    app.include_router(cameras.create_router(camera_service))
    app.include_router(alerts.router)
    app.include_router(history.router)
    app.include_router(sensors.router)

    @app.exception_handler(StarletteHTTPException)
    async def api_error(_request: Request, error: StarletteHTTPException) -> JSONResponse:
        detail = error.detail
        if isinstance(detail, dict):
            code = str(detail.get("code", "REQUEST_FAILED"))
            details = detail
        else:
            code = str(detail)
            details = None
        return JSONResponse(status_code=error.status_code, content={"error": {"code": code, "message": code, "details": details}, "request_id": None})

    @app.exception_handler(RequestValidationError)
    async def validation_error(_request: Request, error: RequestValidationError) -> JSONResponse:
        return JSONResponse(status_code=422, content={"error": {"code": "INVALID_VALUE", "message": "request validation failed", "details": {"validation_errors": error.errors()}}, "request_id": None})

    @app.get("/health")
    async def health() -> dict[str, str]:
        return {"status": "ok", "mode": mode}

    @app.get("/api/v1/cameras/{robot_id}")
    async def camera(robot_id: Literal["robot_1", "robot_2"], _user: UserInfo = Depends(current_user)) -> Response:
        frame = adapter.frame(robot_id)
        if frame is None:
            raise HTTPException(status_code=503, detail={"code": "CAMERA_STALLED", "robot_id": robot_id})
        return Response(content=frame.jpeg, media_type="image/jpeg", headers={"X-Frame-Id": frame.frame_id})

    @app.get("/api/v1/mock/scenario", response_model=MockScenarioRequest)
    async def get_scenario(_user: UserInfo = Depends(current_user)) -> MockScenarioRequest:
        if mode != "mock":
            raise HTTPException(status_code=404, detail="NOT_FOUND")
        return MockScenarioRequest(scenario=adapter.scenario)

    @app.post("/api/v1/mock/scenario", response_model=MockScenarioRequest)
    async def set_scenario(payload: MockScenarioRequest, request: Request, user: UserInfo = Depends(current_user)) -> MockScenarioRequest:
        if mode != "mock":
            raise HTTPException(status_code=404, detail="NOT_FOUND")
        if user.role not in {UserRole.OPERATOR, UserRole.ADMIN}:
            raise HTTPException(status_code=403, detail="FORBIDDEN")
        verify_mutation(request, user)
        adapter.set_scenario(payload.scenario)
        return payload

    return app


def cli() -> None:
    parser = argparse.ArgumentParser(description="Run the Pinky Pro control API")
    parser.add_argument("--mode", choices=("mock", "ros"), default="mock")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--config", type=Path, default=None, help="path to robot mapping YAML for the selected mode")
    parser.add_argument("--database", type=Path, default=default_database_path())
    parser.add_argument("--allowed-origin", default=os.environ.get("CONTROL_PLATFORM_ALLOWED_ORIGIN", "http://localhost:5173"), help="single browser origin allowed for API and WebSocket access")
    parser.add_argument("--secure-cookies", action=argparse.BooleanOptionalAction, default=os.environ.get("CONTROL_PLATFORM_SECURE_COOKIES", "0") == "1", help="mark session and CSRF cookies Secure (required behind HTTPS)")
    parser.add_argument("--reset-password", metavar="USERNAME")
    parser.add_argument("--password", help="password for --reset-password; never persisted in plaintext")
    parser.add_argument("--role", choices=[role.value for role in UserRole], default=UserRole.ADMIN.value)
    args = parser.parse_args()
    if args.reset_password:
        if not args.password:
            parser.error("--password is required with --reset-password")
        Storage(args.database).create_or_reset_user(args.reset_password, args.password, UserRole(args.role))
        return
    uvicorn.run(create_app(args.mode, args.config, args.database, allowed_origin=args.allowed_origin, secure_cookies=args.secure_cookies), host=args.host, port=args.port, workers=1)


if __name__ == "__main__":
    cli()
