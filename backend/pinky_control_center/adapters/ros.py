"""Per-robot rosbridge adapter.

This module deliberately owns two independent websocket clients.  A ROS 2
domain is selected by the rosbridge process, never by a browser request or a
dashboard command.  The adapter therefore uses the fixed, validated mapping
in :class:`RosbridgeConfig` as its only routing authority.
"""
from __future__ import annotations

import asyncio
import base64
import inspect
import json
import math
import os
import ssl
import time
from collections.abc import AsyncIterator, Callable
from dataclasses import dataclass
from datetime import UTC, datetime, timedelta
from io import BytesIO
from typing import Callable, Any, Literal
from uuid import uuid4

import websockets
from PIL import Image

from pinky_control_center.config import RosbridgeConfig, RosbridgeRobotConfig
from pinky_control_center.models import (
    AdapterEvent, CameraFrame, CommandAcceptance, CommandRequest, Connection,
    FormationMode, FormationState, Freshness, MapPoint, Pose, RobotMode, RobotState,
    SensorState, SensorStatus, StateSnapshot,
)

RobotId = Literal["robot_1", "robot_2"]
_RECONNECT_DELAYS = (1, 2, 4, 8)
_MAX_FRAGMENT_COUNT = 64
_MAX_FRAGMENT_BYTES = 4 * 1024 * 1024
_FRAGMENT_TTL_SECONDS = 5.0
_MAX_FRAGMENT_IDS = 16
_CAMERA_STALE_SECONDS = 2.0
_SCAN_STALE_SECONDS = 1.0
_MAX_SCAN_POINTS = 2000


@dataclass
class _FragmentBuffer:
    total: int
    created_at: float
    chunks: dict[int, str]
    byte_count: int = 0


@dataclass
class _PendingServiceCall:
    robot_id: RobotId
    socket: Any
    command: CommandRequest
    future: asyncio.Future[CommandAcceptance]


@dataclass(frozen=True)
class _Transform2D:
    """Planar part of a ROS transform, expressed as parent -> child."""

    x: float
    y: float
    yaw: float


class RosbridgeAdapter:
    """Translate rosbridge JSON into the existing adapter contract.

    `connect_factory` is injectable so protocol tests do not need ROS or a
    network endpoint.  A transport failure only changes the matching robot's
    state; it cannot redirect a command to the other robot.
    """

    def __init__(
        self,
        config: RosbridgeConfig,
        *,
        connect_factory: Callable[..., Any] = websockets.connect,
        clock: Callable[[], datetime] | None = None,
        reconnect_delays: tuple[int, ...] = _RECONNECT_DELAYS,
    ) -> None:
        self.config = config
        self._by_id: dict[RobotId, RosbridgeRobotConfig] = {robot.robot_id: robot for robot in config.robots}
        self._connect_factory = connect_factory
        self._clock = clock or (lambda: datetime.now(UTC))
        self._reconnect_delays = reconnect_delays
        self._sockets: dict[RobotId, Any] = {}
        self._tasks: dict[RobotId, asyncio.Task[None]] = {}
        self._connected = False
        self._event_queue: asyncio.Queue[AdapterEvent | None] = asyncio.Queue(maxsize=200)
        self._event_closed = False
        self._drained_events: list[AdapterEvent] = []
        self._frames: dict[RobotId, CameraFrame] = {}
        self._service_calls: dict[str, _PendingServiceCall] = {}
        self._fragments: dict[RobotId, dict[str, _FragmentBuffer]] = {"robot_1": {}, "robot_2": {}}
        self._states: dict[RobotId, RobotState] = {
            robot_id: self._blank_state(robot) for robot_id, robot in self._by_id.items()
        }
        self._pose_received_at: dict[RobotId, datetime | None] = {"robot_1": None, "robot_2": None}
        self._battery_received_at: dict[RobotId, datetime | None] = {"robot_1": None, "robot_2": None}
        self._camera_received_at: dict[RobotId, datetime | None] = {"robot_1": None, "robot_2": None}
        self._scan_received_at: dict[RobotId, datetime | None] = {"robot_1": None, "robot_2": None}
        self._scans: dict[RobotId, dict[str, object] | None] = {"robot_1": None, "robot_2": None}
        self._transforms: dict[RobotId, dict[tuple[str, str], _Transform2D]] = {"robot_1": {}, "robot_2": {}}
        self._odom_frames: dict[RobotId, tuple[str, str] | None] = {"robot_1": None, "robot_2": None}

    def _blank_state(self, robot: RosbridgeRobotConfig) -> RobotState:
        return RobotState(
            robot_id=robot.robot_id, name=robot.name, role=robot.role,
            connection=Connection.OFFLINE, received_at=None, pose=None,
            pose_freshness=Freshness.UNKNOWN, battery_freshness=Freshness.UNKNOWN,
            mode=RobotMode.UNKNOWN, tf_valid=False, tf_reason_code="ROSBRIDGE_OFFLINE",
            sensors=[SensorStatus(name="camera", state=SensorState.STALE)] if robot.camera.enabled else [],
        )

    async def connect(self) -> None:
        if self._connected:
            return
        if self._event_closed:
            self._event_queue = asyncio.Queue(maxsize=200)
            self._event_closed = False
        self._connected = True
        for robot_id in ("robot_1", "robot_2"):
            self._tasks[robot_id] = asyncio.create_task(self._run(robot_id))
        # Let immediately available test/live transports subscribe before a
        # caller inspects the snapshot, without waiting for unavailable robots.
        await asyncio.sleep(0)

    async def close(self) -> None:
        self._connected = False
        tasks = list(self._tasks.values())
        self._tasks.clear()
        for task in tasks:
            task.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        for socket in list(self._sockets.values()):
            closer = getattr(socket, "close", None)
            if closer:
                result = closer()
                if inspect.isawaitable(result):
                    await result
        self._sockets.clear()
        for pending in self._service_calls.values():
            if not pending.future.done():
                pending.future.set_result(CommandAcceptance(accepted=False, reason_code="ROSBRIDGE_CLOSED"))
        self._service_calls.clear()
        for robot_id in self._states:
            self._mark_disconnected(robot_id)
        self._close_event_stream()

    async def _open(self, robot_id: RobotId) -> Any:
        robot = self._by_id[robot_id]
        candidate = self._connect_factory(robot.bridge_url, open_timeout=5, **self._connection_kwargs(robot_id))
        if inspect.isawaitable(candidate):
            return await candidate
        return candidate

    async def _run(self, robot_id: RobotId) -> None:
        retry = 0
        while self._connected:
            socket: Any | None = None
            try:
                socket = await self._open(robot_id)
                self._sockets[robot_id] = socket
                self._mark_connected(robot_id)
                await self._subscribe(robot_id, socket)
                stable_connection = False
                while self._connected:
                    raw = await socket.recv()
                    if raw is None:
                        raise ConnectionError("rosbridge closed")
                    received_message = await self._handle_raw(robot_id, raw, socket)
                    # Do not repeatedly hammer an endpoint that accepts TCP
                    # and drops us before it carries a rosbridge message.
                    if received_message and not stable_connection:
                        stable_connection = True
                        retry = 0
            except asyncio.CancelledError:
                raise
            except Exception:
                self._mark_disconnected(robot_id)
                delay = self._reconnect_delays[min(retry, len(self._reconnect_delays) - 1)]
                retry += 1
                if self._connected:
                    await asyncio.sleep(delay)
            finally:
                if self._sockets.get(robot_id) is socket:
                    self._sockets.pop(robot_id, None)
                if socket is not None:
                    closer = getattr(socket, "close", None)
                    if closer:
                        try:
                            result = closer()
                            if inspect.isawaitable(result):
                                await result
                        except Exception:
                            pass

    async def _subscribe(self, robot_id: RobotId, socket: Any) -> None:
        robot = self._by_id[robot_id]
        topics = robot.topics
        subscriptions = [
            (topics.odom, "nav_msgs/msg/Odometry", 0),
            (topics.battery_percent, "std_msgs/msg/Float32", 0),
            (topics.battery_voltage, "std_msgs/msg/Float32", 0),
            (topics.control_status, None, 0),
            (topics.tf, "tf2_msgs/msg/TFMessage", 0),
            (topics.tf_static, "tf2_msgs/msg/TFMessage", 0),
            (topics.scan, "sensor_msgs/msg/LaserScan", 200),
        ]
        if robot.camera.enabled:
            subscriptions.append(
                (topics.camera_compressed, "sensor_msgs/msg/CompressedImage", robot.camera.throttle_rate_ms)
            )
        for topic, message_type, throttle_rate in subscriptions:
            payload: dict[str, object] = {"op": "subscribe", "topic": topic, "queue_length": 1}
            if message_type:
                payload["type"] = message_type
            if throttle_rate:
                payload.update({"throttle_rate": throttle_rate, "fragment_size": robot.camera.fragment_size})
            await socket.send(json.dumps(payload, separators=(",", ":")))
        if topics.path:
            await socket.send(json.dumps({"op": "subscribe", "topic": topics.path, "type": "nav_msgs/msg/Path", "queue_length": 1}, separators=(",", ":")))

    def scan_fresh(self, robot_id: RobotId, max_age: float = 2.0) -> bool:
        """True when a LaserScan arrived within max_age seconds."""
        received_at = self._scan_received_at.get(robot_id)
        return received_at is not None and self._clock() - received_at <= max_age

    async def set_map_streaming(self, robot_id: RobotId, enabled: bool) -> None:
        """Subscribe/unsubscribe /map for the live mapping view (robot-scoped)."""
        socket = self._sockets.get(robot_id)
        if socket is None:
            return
        op = "subscribe" if enabled else "unsubscribe"
        await socket.send(json.dumps({"op": op, "topic": "/map", "type": "nav_msgs/msg/OccupancyGrid", "queue_length": 1}, separators=(",", ":")))

    def _mark_connected(self, robot_id: RobotId) -> None:
        now = self._clock()
        current = self._states[robot_id]
        self._states[robot_id] = current.model_copy(update={
            "connection": Connection.ONLINE, "received_at": now,
            "tf_reason_code": "TF_UNAVAILABLE" if current.pose is None else current.tf_reason_code,
        })

    def _mark_disconnected(self, robot_id: RobotId) -> None:
        self._frames.pop(robot_id, None)
        self._fragments[robot_id].clear()
        self._pose_received_at[robot_id] = None
        self._battery_received_at[robot_id] = None
        self._camera_received_at[robot_id] = None
        self._scan_received_at[robot_id] = None
        self._scans[robot_id] = None
        self._transforms[robot_id].clear()
        self._odom_frames[robot_id] = None
        current = self._states[robot_id]
        self._states[robot_id] = current.model_copy(update={
            "connection": Connection.OFFLINE, "received_at": None,
            "pose_freshness": Freshness.UNKNOWN, "battery_freshness": Freshness.UNKNOWN,
            "pose": None, "linear_mps": None, "angular_rps": None,
            "odom_linear_mps": None, "odom_angular_rps": None,
            "battery_percent": None, "voltage_v": None,
            "mode": RobotMode.UNKNOWN, "stop_latched": None, "capabilities": [],
            "trail": [], "path": [], "goal": None,
            "tf_valid": False, "tf_reason_code": "ROSBRIDGE_OFFLINE",
            "sensors": [SensorStatus(name="camera", state=SensorState.STALE)]
            if self._by_id[robot_id].camera.enabled else [],
        })

    async def _handle_raw(self, robot_id: RobotId, raw: str | bytes, socket: Any | None = None) -> bool:
        try:
            payload = json.loads(raw.decode("utf-8") if isinstance(raw, bytes) else raw)
        except (UnicodeDecodeError, json.JSONDecodeError):
            return False
        if not isinstance(payload, dict):
            return False
        if payload.get("op") == "fragment":
            reassembled = self._accept_fragment(robot_id, payload)
            if reassembled is not None:
                return await self._handle_raw(robot_id, reassembled, socket)
            return False
        if payload.get("op") == "service_response":
            await self._complete_service_call(robot_id, socket, payload)
            return True
        if payload.get("op") != "publish" or not isinstance(payload.get("topic"), str) or not isinstance(payload.get("msg"), dict):
            return False
        await self.handle_publish(robot_id, payload["topic"], payload["msg"])
        return True

    def _accept_fragment(self, robot_id: RobotId, payload: dict[str, object]) -> str | None:
        """Safely retain only a small, short-lived rosbridge fragment set."""
        buffers = self._fragments[robot_id]
        now = time.monotonic()
        for fragment_id, buffer in list(buffers.items()):
            if now - buffer.created_at > _FRAGMENT_TTL_SECONDS:
                del buffers[fragment_id]
        fragment_id, number, total, data = payload.get("id"), payload.get("num"), payload.get("total"), payload.get("data")
        if not isinstance(fragment_id, str) or not fragment_id or len(fragment_id) > 128:
            return None
        if isinstance(number, bool) or not isinstance(number, int) or isinstance(total, bool) or not isinstance(total, int):
            return None
        if not isinstance(data, str) or total < 1 or total > _MAX_FRAGMENT_COUNT or number < 0 or number >= total:
            buffers.pop(fragment_id, None)
            return None
        buffer = buffers.get(fragment_id)
        if buffer is None:
            if len(buffers) >= _MAX_FRAGMENT_IDS:
                oldest = min(buffers, key=lambda value: buffers[value].created_at)
                del buffers[oldest]
            buffer = _FragmentBuffer(total=total, created_at=now, chunks={})
            buffers[fragment_id] = buffer
        elif buffer.total != total:
            del buffers[fragment_id]
            return None
        previous = buffer.chunks.get(number)
        size = len(data.encode("utf-8"))
        next_size = buffer.byte_count - (len(previous.encode("utf-8")) if previous is not None else 0) + size
        if next_size > _MAX_FRAGMENT_BYTES:
            del buffers[fragment_id]
            return None
        buffer.chunks[number] = data
        buffer.byte_count = next_size
        if len(buffer.chunks) != total:
            return None
        try:
            reassembled = "".join(buffer.chunks[index] for index in range(total))
        except KeyError:
            return None
        del buffers[fragment_id]
        return reassembled

    async def _complete_service_call(self, robot_id: RobotId, socket: Any | None, payload: dict[str, object]) -> None:
        call_id = payload.get("id")
        if not isinstance(call_id, str):
            return
        pending = self._service_calls.get(call_id)
        if pending is None or pending.robot_id != robot_id or pending.socket is not socket or pending.future.done():
            return
        self._service_calls.pop(call_id, None)
        values = payload.get("values")
        accepted = bool(payload.get("result", False))
        reason: str | None = None
        if isinstance(values, dict):
            accepted = bool(values.get("accepted", accepted))
            value_reason = values.get("reason_code")
            reason = value_reason if isinstance(value_reason, str) else None
        result = CommandAcceptance(accepted=accepted, reason_code=reason if not accepted else None)
        pending.future.set_result(result)
        if result.accepted and pending.command.operation == "navigate":
            try:
                goal = Pose.model_validate(pending.command.parameters["goal"])
                self._states[robot_id] = self._states[robot_id].model_copy(update={"goal": goal})
            except (KeyError, TypeError, ValueError):
                pass
        elif result.accepted and pending.command.operation in {"stop", "cancel_navigation"}:
            self._states[robot_id] = self._states[robot_id].model_copy(update={"goal": None})
        await self._emit("command", robot_id, {
            "command_id": str(pending.command.command_id),
            "state": "SUCCEEDED" if result.accepted else "REJECTED",
            "reason_code": result.reason_code,
            "operation": pending.command.operation,
            "parameters": pending.command.parameters,
        }, self._clock())

    map_handler: Callable[[RobotId, dict], None] | None = None

    async def handle_publish(self, robot_id: RobotId, topic: str, message: dict[str, object]) -> None:
        """Public protocol seam used by contract tests and rosbridge readers."""
        robot = self._by_id[robot_id]
        now = self._clock()
        if topic == robot.topics.odom:
            self._update_odom(robot_id, message, now)
            await self._emit("robot_state", robot_id, self._states[robot_id].model_dump(mode="json"), now)
        elif topic in {robot.topics.tf, robot.topics.tf_static}:
            self._update_tf(robot_id, message)
            self._refresh_map_pose(robot_id)
            await self._emit("robot_state", robot_id, self._states[robot_id].model_dump(mode="json"), now)
        elif topic in {robot.topics.battery_percent, robot.topics.battery_voltage}:
            self._update_battery(robot_id, topic, message, now)
            await self._emit("robot_state", robot_id, self._states[robot_id].model_dump(mode="json"), now)
        elif topic == robot.topics.control_status:
            self._update_control_status(robot_id, message, now)
            await self._emit("robot_state", robot_id, self._states[robot_id].model_dump(mode="json"), now)
        elif topic == robot.topics.camera_compressed:
            frame = self._decode_camera(robot_id, message, now)
            if frame:
                self._frames[robot_id] = frame
        elif robot.topics.path and topic == robot.topics.path:
            payload = self._path_payload(robot_id, message)
            self._states[robot_id] = self._states[robot_id].model_copy(update={
                "connection": Connection.ONLINE, "received_at": now,
                "path": [MapPoint.model_validate(point) for point in payload["points"]],
            })
            await self._emit("path", robot_id, payload, now)
        elif topic == "/map":
            if self.map_handler is not None:
                self.map_handler(robot_id, message)
        elif topic == robot.topics.scan:
            self._scans[robot_id] = message
            self._scan_received_at[robot_id] = now
            self._states[robot_id] = self._states[robot_id].model_copy(update={
                "connection": Connection.ONLINE, "received_at": now,
            })

    def _update_odom(self, robot_id: RobotId, message: dict[str, object], now: datetime) -> None:
        pose_part = _nested_dict(message, "pose", "pose")
        position = _nested_dict(pose_part, "position")
        orientation = _nested_dict(pose_part, "orientation")
        twist = _nested_dict(message, "twist", "twist")
        linear = _nested_dict(twist, "linear")
        angular = _nested_dict(twist, "angular")
        try:
            pose = Pose(x=float(position["x"]), y=float(position["y"]), yaw=_yaw(orientation), frame_id=_frame_id(message, "odom"))
        except (KeyError, TypeError, ValueError):
            return
        frame_id = _frame_id(message, "odom")
        child_frame = message.get("child_frame_id")
        if isinstance(child_frame, str) and child_frame:
            edge = (_frame(frame_id), _frame(child_frame))
            self._odom_frames[robot_id] = edge
            self._transforms[robot_id][edge] = _Transform2D(
                pose.x, pose.y, pose.yaw
            )
        self._states[robot_id] = self._states[robot_id].model_copy(update={
            "connection": Connection.ONLINE, "received_at": now, "pose": pose,
            "pose_freshness": Freshness.FRESH, "linear_mps": _number(linear.get("x")),
            "angular_rps": _number(angular.get("z")),
            "odom_linear_mps": _number(linear.get("x")), "odom_angular_rps": _number(angular.get("z")),
            "tf_valid": False,
            "tf_reason_code": "MAP_TF_UNVERIFIED",
        })
        self._pose_received_at[robot_id] = now
        self._refresh_map_pose(robot_id)

    def _update_tf(self, robot_id: RobotId, message: dict[str, object]) -> None:
        transforms = message.get("transforms")
        if not isinstance(transforms, list):
            return
        for item in transforms:
            if not isinstance(item, dict):
                continue
            header = _nested_dict(item, "header")
            parent = header.get("frame_id")
            child = item.get("child_frame_id")
            transform = _nested_dict(item, "transform")
            translation = _nested_dict(transform, "translation")
            rotation = _nested_dict(transform, "rotation")
            if not isinstance(parent, str) or not parent or not isinstance(child, str) or not child:
                continue
            x, y = _number(translation.get("x")), _number(translation.get("y"))
            if x is None or y is None:
                continue
            try:
                yaw = _yaw(rotation)
            except ValueError:
                continue
            self._transforms[robot_id][(_frame(parent), _frame(child))] = _Transform2D(x, y, yaw)

    def _refresh_map_pose(self, robot_id: RobotId) -> None:
        current = self._states[robot_id]
        if current.pose is None:
            return
        target = self._odom_frames[robot_id][1] if self._odom_frames[robot_id] else _frame("base_footprint")
        transform = _lookup_transform(self._transforms[robot_id], "map", target)
        if transform is None:
            self._states[robot_id] = current.model_copy(update={
                "tf_valid": False, "tf_reason_code": "MAP_TF_UNVERIFIED",
            })
            return
        updated_trail = [*current.trail, MapPoint(x=transform.x, y=transform.y)][-200:]
        self._states[robot_id] = current.model_copy(update={
            "pose": Pose(x=transform.x, y=transform.y, yaw=transform.yaw, frame_id="map"),
            "tf_valid": True, "tf_reason_code": None, "trail": updated_trail,
        })

    def _update_battery(self, robot_id: RobotId, topic: str, message: dict[str, object], now: datetime) -> None:
        value = _number(message.get("data"))
        if value is None:
            return
        updates: dict[str, object] = {"connection": Connection.ONLINE, "received_at": now, "battery_freshness": Freshness.FRESH}
        if topic == self._by_id[robot_id].topics.battery_percent:
            updates["battery_percent"] = max(0.0, min(100.0, value * 100 if value <= 1 else value))
        else:
            updates["voltage_v"] = max(0.0, value)
        self._states[robot_id] = self._states[robot_id].model_copy(update=updates)
        self._battery_received_at[robot_id] = now

    def _update_control_status(self, robot_id: RobotId, message: dict[str, object], now: datetime) -> None:
        current = self._states[robot_id]
        mode_raw = message.get("mode")
        try:
            mode = RobotMode(str(mode_raw)) if mode_raw is not None else current.mode
        except ValueError:
            mode = RobotMode.UNKNOWN
        capabilities = message.get("capabilities")
        self._states[robot_id] = current.model_copy(update={
            "connection": Connection.ONLINE, "received_at": now, "mode": mode,
            "stop_latched": message.get("stop_latched") if isinstance(message.get("stop_latched"), bool) else current.stop_latched,
            "linear_mps": _number(message.get("linear_mps")) if _number(message.get("linear_mps")) is not None else current.linear_mps,
            "angular_rps": _number(message.get("angular_rps")) if _number(message.get("angular_rps")) is not None else current.angular_rps,
            "capabilities": [value for value in capabilities if isinstance(value, str)] if isinstance(capabilities, list) else current.capabilities,
        })

    def _decode_camera(self, robot_id: RobotId, message: dict[str, object], now: datetime) -> CameraFrame | None:
        encoded = message.get("data")
        if not isinstance(encoded, str):
            return None
        try:
            jpeg = base64.b64decode(encoded, validate=True)
            if not jpeg.startswith(b"\xff\xd8"):
                return None
            with Image.open(BytesIO(jpeg)) as image:
                width, height = image.size
        except (ValueError, OSError):
            return None
        header = _nested_dict(message, "header")
        stamp = _nested_dict(header, "stamp")
        captured_at = _ros_stamp(stamp)
        frame = CameraFrame(robot_id=robot_id, frame_id=str(_nested_dict(header).get("seq", uuid4())), captured_at=captured_at, received_at=now, width=width, height=height, jpeg=jpeg)
        self._states[robot_id] = self._states[robot_id].model_copy(update={
            "sensors": [SensorStatus(name="camera", state=SensorState.OK, received_at=now)]
        })
        self._camera_received_at[robot_id] = now
        return frame

    def _path_payload(self, robot_id: RobotId, message: dict[str, object]) -> dict[str, object]:
        points: list[dict[str, float]] = []
        poses = message.get("poses")
        if isinstance(poses, list):
            for item in poses[:200]:
                if not isinstance(item, dict):
                    continue
                position = _nested_dict(item, "pose", "position")
                x, y = _number(position.get("x")), _number(position.get("y"))
                if x is not None and y is not None:
                    points.append({"x": x, "y": y})
        return {"robot_id": robot_id, "frame_id": _frame_id(message, ""), "points": points}

    async def _emit(self, kind: Literal["robot_state", "formation", "command", "map", "path", "scan", "costmap"], robot_id: RobotId | None, payload: dict[str, object], now: datetime) -> None:
        if self._event_closed:
            return
        event = AdapterEvent(kind=kind, robot_id=robot_id, payload=payload, received_at=now)
        self._drained_events.append(event)
        if self._event_queue.full():
            self._event_queue.get_nowait()
        self._event_queue.put_nowait(event)

    def snapshot(self) -> StateSnapshot:
        now = self._clock()
        robots = [self._freshness_state(robot_id, now) for robot_id in ("robot_1", "robot_2")]
        return StateSnapshot(
            robots=robots,
            formation=FormationState(state=FormationMode.UNPAIRED, master_id="robot_1", slave_id="robot_2", target_distance_m=0.8),
            mode="ros", seq=0, server_time=now, map_id="unknown",
        )

    def frame(self, robot_id: str) -> CameraFrame | None:
        if robot_id not in self._by_id:
            return None
        if _freshness(self._camera_received_at[robot_id], self._clock(), _CAMERA_STALE_SECONDS) is not Freshness.FRESH:
            self._frames.pop(robot_id, None)
            return None
        return self._frames.get(robot_id)

    def sensor_layers(self, robot_id: str) -> dict[str, object]:
        """Return bounded sensor overlays, with LaserScan points in map coordinates."""
        if robot_id not in self._by_id:
            raise ValueError("unknown robot")
        state = self._states[robot_id]
        scan = self._scan_layer(robot_id)
        unavailable = "STALE" if state.connection is not Connection.ONLINE else "UNSUPPORTED"
        return {
            "robot_id": robot_id,
            "scan": scan,
            "costmaps": [
                {"name": "local_costmap", "state": unavailable, "cells": []},
                {"name": "global_costmap", "state": unavailable, "cells": []},
            ],
        }

    def _scan_layer(self, robot_id: RobotId) -> dict[str, object]:
        received_at = self._scan_received_at[robot_id]
        message = self._scans[robot_id]
        base = {
            "frame_id": "map", "points": [],
            "source_at": None,
            "received_at": received_at.isoformat() if received_at else None,
        }
        if message is None or received_at is None:
            return {**base, "state": "STALE", "reason_code": "SCAN_UNAVAILABLE"}
        if _freshness(received_at, self._clock(), _SCAN_STALE_SECONDS) is not Freshness.FRESH:
            return {**base, "state": "STALE", "reason_code": "SCAN_STALE"}

        laser_frame = _frame_id(message, "")
        map_to_laser = _lookup_transform(self._transforms[robot_id], "map", laser_frame) if laser_frame else None
        if map_to_laser is None:
            return {**base, "state": "ERROR", "reason_code": "SCAN_MAP_TF_UNAVAILABLE"}

        ranges = message.get("ranges")
        angle_min = _number(message.get("angle_min"))
        angle_increment = _number(message.get("angle_increment"))
        range_min = _number(message.get("range_min"))
        range_max = _number(message.get("range_max"))
        if not isinstance(ranges, list) or angle_min is None or angle_increment is None:
            return {**base, "state": "ERROR", "reason_code": "SCAN_INVALID"}

        stride = max(1, math.ceil(len(ranges) / _MAX_SCAN_POINTS))
        points: list[dict[str, float]] = []
        minimum: float | None = None
        for index in range(0, len(ranges), stride):
            distance = _number(ranges[index])
            if distance is None or (range_min is not None and distance < range_min) or (range_max is not None and distance > range_max):
                continue
            angle = map_to_laser.yaw + angle_min + index * angle_increment
            points.append({
                "x": map_to_laser.x + math.cos(angle) * distance,
                "y": map_to_laser.y + math.sin(angle) * distance,
                "range_m": distance,
            })
            minimum = distance if minimum is None else min(minimum, distance)
        stamp = _ros_stamp(_nested_dict(message, "header", "stamp"))
        return {
            **base, "state": "OK", "reason_code": None, "points": points,
            "source_at": stamp.isoformat() if stamp else None,
            "point_count": len(points), "min_range_m": minimum,
        }

    def drain_events(self) -> list[AdapterEvent]:
        events, self._drained_events = self._drained_events, []
        return events

    async def events(self) -> AsyncIterator[AdapterEvent]:
        while True:
            event = await self._event_queue.get()
            if event is None:
                # Keep a terminal marker available for another consumer.
                if self._event_closed and not self._event_queue.full():
                    self._event_queue.put_nowait(None)
                return
            yield event

    async def frames(self, robot_id: str) -> AsyncIterator[CameraFrame]:
        last_id: str | None = None
        while self._connected and robot_id in self._by_id:
            frame = self.frame(robot_id)
            if frame and frame.frame_id != last_id:
                last_id = frame.frame_id
                yield frame
            await asyncio.sleep(0.05)

    async def execute(self, command: CommandRequest) -> CommandAcceptance:
        robot_id = command.robot_id
        robot = self._by_id[robot_id]
        socket = self._sockets.get(robot_id)
        if socket is None:
            return CommandAcceptance(accepted=False, reason_code="ROSBRIDGE_OFFLINE")
        if command.operation == "initial_pose":
            return await self._publish_initial_pose(robot_id, socket, command)
        service = robot.services
        if command.operation.startswith("follow_"):
            if not service.follow_available or not service.follow_command or not service.follow_command_type:
                return CommandAcceptance(accepted=False, reason_code="UNSUPPORTED")
            endpoint, service_type = service.follow_command, service.follow_command_type
        else:
            # The robot-side control mediator is mandatory: this adapter never
            # writes /cmd_vel directly.  Unconfirmed service contracts stay
            # explicitly unsupported even if a placeholder name is configured.
            if not service.control_available:
                return CommandAcceptance(accepted=False, reason_code="UNSUPPORTED")
            endpoint, service_type = service.control_command, service.control_command_type
        call_id = f"cc:{command.command_id}:{robot_id}"
        future: asyncio.Future[CommandAcceptance] = asyncio.get_running_loop().create_future()
        self._service_calls[call_id] = _PendingServiceCall(robot_id=robot_id, socket=socket, command=command, future=future)
        request = {
            "op": "call_service", "id": call_id, "service": endpoint,
            "type": service_type,
            "args": self._service_args(service_type, command),
        }
        try:
            await socket.send(json.dumps(request, separators=(",", ":")))
            return await asyncio.wait_for(future, timeout=2.0)
        except asyncio.TimeoutError:
            return CommandAcceptance(accepted=False, reason_code="ROSBRIDGE_TIMEOUT")
        except Exception:
            return CommandAcceptance(accepted=False, reason_code="ROSBRIDGE_WRITE_FAILED")
        finally:
            self._service_calls.pop(call_id, None)

    @staticmethod
    def _service_args(service_type: str, command: CommandRequest) -> dict[str, object]:
        if service_type.endswith("/ControlCommand"):
            return {
                "command_id": str(command.command_id),
                "operation": command.operation,
                "parameters_json": json.dumps(command.parameters, separators=(",", ":")),
            }
        return {
            "command_id": str(command.command_id),
            "operation": command.operation,
            "parameters": command.parameters,
        }

    async def _publish_initial_pose(self, robot_id: RobotId, socket: Any, command: CommandRequest) -> CommandAcceptance:
        topic = self._by_id[robot_id].topics.initial_pose
        if not topic:
            return CommandAcceptance(accepted=False, reason_code="UNSUPPORTED")
        try:
            pose = Pose.model_validate(command.parameters["pose"])
        except (KeyError, TypeError, ValueError):
            return CommandAcceptance(accepted=False, reason_code="INVALID_VALUE")
        message = {
            # A zero timestamp asks TF to use the latest available transform.
            # This avoids rejecting an initial pose when the robot and control
            # center clocks differ slightly or AMCL starts during publication.
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": pose.frame_id},
            "pose": {
                "pose": {
                    "position": {"x": pose.x, "y": pose.y, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(pose.yaw / 2), "w": math.cos(pose.yaw / 2)},
                },
                "covariance": [0.0] * 36,
            },
        }
        try:
            await socket.send(json.dumps({"op": "publish", "topic": topic, "type": "geometry_msgs/msg/PoseWithCovarianceStamped", "msg": message}, separators=(",", ":")))
        except Exception:
            return CommandAcceptance(accepted=False, reason_code="ROSBRIDGE_WRITE_FAILED")
        await self._emit("command", robot_id, {
            "command_id": str(command.command_id), "state": "SUCCEEDED", "reason_code": None,
            "operation": command.operation, "parameters": command.parameters,
        }, self._clock())
        return CommandAcceptance(accepted=True)

    async def publish_manual_velocity(self, robot_id: RobotId, linear_mps: float, angular_rps: float) -> CommandAcceptance:
        """Publish only to the robot-side safety mediator input, never /cmd_vel."""
        robot = self._by_id[robot_id]
        topic, socket = robot.topics.manual_velocity, self._sockets.get(robot_id)
        if not topic:
            return CommandAcceptance(accepted=False, reason_code="UNSUPPORTED")
        if socket is None:
            return CommandAcceptance(accepted=False, reason_code="ROSBRIDGE_OFFLINE")
        message = {"header": {}, "twist": {"linear": {"x": linear_mps, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": angular_rps}}}
        try:
            await socket.send(json.dumps({"op": "publish", "topic": topic, "type": "geometry_msgs/msg/TwistStamped", "msg": message}, separators=(",", ":")))
        except Exception:
            return CommandAcceptance(accepted=False, reason_code="ROSBRIDGE_WRITE_FAILED")
        return CommandAcceptance(accepted=True)

    async def apply_settings(self, values: dict[str, object]) -> bool:
        """ROS parameter/settings application has no confirmed T12 contract yet."""
        return False

    def _freshness_state(self, robot_id: RobotId, now: datetime) -> RobotState:
        current = self._states[robot_id]
        pose_at, battery_at = self._pose_received_at[robot_id], self._battery_received_at[robot_id]
        pose_freshness = _freshness(pose_at, now, 1.0)
        battery_freshness = _freshness(battery_at, now, 15.0)
        camera_freshness = _freshness(self._camera_received_at[robot_id], now, _CAMERA_STALE_SECONDS)
        sensors = [
            sensor.model_copy(update={"state": SensorState.OK if camera_freshness is Freshness.FRESH else SensorState.STALE})
            if sensor.name == "camera" else sensor
            for sensor in current.sensors
        ]
        return current.model_copy(update={"pose_freshness": pose_freshness, "battery_freshness": battery_freshness, "sensors": sensors})

    def _connection_kwargs(self, robot_id: RobotId) -> dict[str, object]:
        """Resolve environment references at connect time without exposing values."""
        robot = self._by_id[robot_id]
        security = robot.security
        if robot.bridge_url.startswith("ws://"):
            return {}
        context = ssl.create_default_context(cafile=_required_env_path(security.ca_cert_env) if security.ca_cert_env else None)
        if not security.verify_tls:
            context.check_hostname = False
            context.verify_mode = ssl.CERT_NONE
        if security.client_cert_env and security.client_key_env:
            context.load_cert_chain(_required_env_path(security.client_cert_env), _required_env_path(security.client_key_env))
        result: dict[str, object] = {"ssl": context}
        if security.authorization_token_env:
            result["additional_headers"] = {"Authorization": f"Bearer {_required_env_value(security.authorization_token_env)}"}
        return result

    def _close_event_stream(self) -> None:
        if self._event_closed:
            return
        self._event_closed = True
        if self._event_queue.full():
            self._event_queue.get_nowait()
        self._event_queue.put_nowait(None)


def _nested_dict(value: object, *keys: str) -> dict[str, object]:
    current = value
    for key in keys:
        if not isinstance(current, dict):
            return {}
        current = current.get(key)
    return current if isinstance(current, dict) else {}


def _number(value: object) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _freshness(received_at: datetime | None, now: datetime, max_age_seconds: float) -> Freshness:
    if received_at is None:
        return Freshness.UNKNOWN
    return Freshness.FRESH if now - received_at <= timedelta(seconds=max_age_seconds) else Freshness.STALE


def _required_env_value(name: str) -> str:
    value = os.environ.get(name)
    if not value:
        raise RuntimeError("configured rosbridge credential is unavailable")
    return value


def _required_env_path(name: str) -> str:
    return _required_env_value(name)


def _yaw(orientation: dict[str, object]) -> float:
    x, y, z, w = (_number(orientation.get(axis)) for axis in ("x", "y", "z", "w"))
    if None in {x, y, z, w}:
        raise ValueError("invalid quaternion")
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))  # type: ignore[operator]


def _frame_id(message: dict[str, object], fallback: str) -> str:
    value = _nested_dict(message, "header").get("frame_id")
    return value if isinstance(value, str) and value else fallback


def _frame(value: str) -> str:
    """ROS frame IDs are compared without the legacy leading slash."""
    return value.lstrip("/")


def _compose(parent_to_middle: _Transform2D, middle_to_child: _Transform2D) -> _Transform2D:
    cosine, sine = math.cos(parent_to_middle.yaw), math.sin(parent_to_middle.yaw)
    return _Transform2D(
        parent_to_middle.x + cosine * middle_to_child.x - sine * middle_to_child.y,
        parent_to_middle.y + sine * middle_to_child.x + cosine * middle_to_child.y,
        _normalize_yaw(parent_to_middle.yaw + middle_to_child.yaw),
    )


def _inverse(transform: _Transform2D) -> _Transform2D:
    cosine, sine = math.cos(transform.yaw), math.sin(transform.yaw)
    return _Transform2D(
        -cosine * transform.x - sine * transform.y,
        sine * transform.x - cosine * transform.y,
        _normalize_yaw(-transform.yaw),
    )


def _lookup_transform(transforms: dict[tuple[str, str], _Transform2D], source: str, target: str) -> _Transform2D | None:
    """Find and compose a short TF graph path from source to target."""
    source, target = _frame(source), _frame(target)
    if source == target:
        return _Transform2D(0.0, 0.0, 0.0)
    graph: dict[str, list[tuple[str, _Transform2D]]] = {}
    for (parent, child), transform in transforms.items():
        graph.setdefault(parent, []).append((child, transform))
        graph.setdefault(child, []).append((parent, _inverse(transform)))
    queue: list[tuple[str, _Transform2D]] = [(source, _Transform2D(0.0, 0.0, 0.0))]
    visited = {source}
    while queue:
        frame, accumulated = queue.pop(0)
        for neighbor, edge in graph.get(frame, []):
            if neighbor in visited:
                continue
            composed = _compose(accumulated, edge)
            if neighbor == target:
                return composed
            visited.add(neighbor)
            queue.append((neighbor, composed))
    return None


def _normalize_yaw(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _ros_stamp(stamp: dict[str, object]) -> datetime | None:
    seconds, nanoseconds = _number(stamp.get("sec")), _number(stamp.get("nanosec"))
    if seconds is None or nanoseconds is None:
        return None
    try:
        return datetime.fromtimestamp(seconds + nanoseconds / 1_000_000_000, tz=UTC)
    except (OverflowError, OSError, ValueError):
        return None
