from __future__ import annotations

import asyncio
import base64
import json
import math
import ssl
from datetime import UTC, datetime, timedelta
from uuid import uuid4

import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from pinky_control_center.adapters.mock import MockRobotAdapter
from pinky_control_center.adapters.ros import RosbridgeAdapter
from pinky_control_center.config import RosbridgeConfig, load_ros_config
from pinky_control_center.models import CommandRequest, Connection, Freshness
from pinky_control_center.main import create_app
from pinky_control_center.state_store import StateStore


class FakeSocket:
    def __init__(self, *, respond_to_calls: bool = True) -> None:
        self.sent: list[dict[str, object]] = []
        self.incoming: asyncio.Queue[str | None] = asyncio.Queue()
        self.closed = False
        self.respond_to_calls = respond_to_calls

    async def send(self, raw: str) -> None:
        payload = json.loads(raw)
        self.sent.append(payload)
        if payload["op"] == "call_service" and self.respond_to_calls:
            await self.incoming.put(json.dumps({"op": "service_response", "id": payload["id"], "result": True, "values": {"accepted": True}}))

    async def recv(self) -> str | None:
        return await self.incoming.get()

    async def close(self) -> None:
        self.closed = True


def enabled_config() -> RosbridgeConfig:
    data = load_ros_config().model_dump(mode="python")
    for robot in data["robots"]:
        robot["services"]["control_available"] = True
        robot["services"]["follow_available"] = True
    return RosbridgeConfig.model_validate(data)


def test_ros_config_locks_domain_ids_and_uses_two_endpoints() -> None:
    data = load_ros_config().model_dump(mode="python")
    data["robots"][0]["domain_id"] = 99
    with pytest.raises(ValidationError, match="domain_id must be 12"):
        RosbridgeConfig.model_validate(data)
    data = load_ros_config().model_dump(mode="python")
    data["robots"][1]["bridge_url"] = data["robots"][0]["bridge_url"]
    with pytest.raises(ValidationError, match="separate rosbridge endpoint"):
        RosbridgeConfig.model_validate(data)
    data = load_ros_config().model_dump(mode="python")
    data["robots"][0]["topics"]["odom"] = "/odom"
    assert RosbridgeConfig.model_validate(data).robots[0].topics.odom == "/odom"
    data = load_ros_config().model_dump(mode="python")
    data["robots"][0]["bridge_url"] = "wss://robot-1.local:9090"
    data["robots"][0]["security"] = {"client_cert_env": "ROBOT_CERT"}
    with pytest.raises(ValidationError, match="configured together"):
        RosbridgeConfig.model_validate(data)
    data = load_ros_config().model_dump(mode="python")
    data["robots"][0]["security"] = {"authorization_token_env": "ROBOT_TOKEN"}
    with pytest.raises(ValidationError, match="require a wss"):
        RosbridgeConfig.model_validate(data)


def test_rosbridge_routes_subscriptions_and_commands_to_the_matching_robot() -> None:
    async def exercise() -> None:
        sockets = {"ws://robot-1.local:9090": FakeSocket(), "ws://robot-2.local:9091": FakeSocket()}

        async def connect(url: str, **_kwargs):
            return sockets[url]

        adapter = RosbridgeAdapter(enabled_config(), connect_factory=connect)
        await adapter.connect()
        await asyncio.sleep(0)
        assert {item["topic"] for item in sockets["ws://robot-1.local:9090"].sent} >= {"/odom", "/camera/image_raw/compressed"}
        assert {item["topic"] for item in sockets["ws://robot-2.local:9091"].sent} >= {"/odom", "/camera/image_raw/compressed"}

        accepted = await adapter.execute(CommandRequest(command_id=uuid4(), robot_id="robot_2", operation="stop"))
        assert accepted.accepted is True
        calls_1 = [item for item in sockets["ws://robot-1.local:9090"].sent if item["op"] == "call_service"]
        calls_2 = [item for item in sockets["ws://robot-2.local:9091"].sent if item["op"] == "call_service"]
        assert calls_1 == []
        assert calls_2[0]["service"] == "/control/command"
        assert calls_2[0]["args"]["operation"] == "stop"
        events = adapter.drain_events()
        assert events[-1].kind == "command" and events[-1].payload["state"] == "SUCCEEDED"
        await adapter.close()

    asyncio.run(exercise())


def test_rosbridge_does_not_subscribe_when_camera_is_disabled() -> None:
    async def exercise() -> None:
        data = load_ros_config().model_dump(mode="python")
        for robot in data["robots"]:
            robot["camera"]["enabled"] = False
        config = RosbridgeConfig.model_validate(data)
        sockets = {robot.bridge_url: FakeSocket() for robot in config.robots}

        async def connect(url: str, **_kwargs):
            return sockets[url]

        adapter = RosbridgeAdapter(config, connect_factory=connect)
        await adapter.connect()
        await asyncio.sleep(0)
        for robot in config.robots:
            topics = {item["topic"] for item in sockets[robot.bridge_url].sent if item["op"] == "subscribe"}
            assert robot.topics.camera_compressed not in topics
        assert all(state.sensors == [] for state in adapter.snapshot().robots)
        await adapter.close()

    asyncio.run(exercise())


def test_rosbridge_decodes_compressed_camera_and_never_treats_odom_as_map_pose() -> None:
    async def exercise() -> None:
        adapter = RosbridgeAdapter(load_ros_config())
        jpeg = MockRobotAdapter().frame("robot_1").jpeg
        await adapter.handle_publish("robot_1", "/camera/image_raw/compressed", {"data": base64.b64encode(jpeg).decode(), "header": {"seq": 7}})
        await adapter.handle_publish("robot_1", "/odom", {
            "header": {"frame_id": "odom"},
            "pose": {"pose": {"position": {"x": 1.0, "y": 2.0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}},
            "twist": {"twist": {"linear": {"x": 0.1}, "angular": {"z": 0.2}}},
        })
        frame = adapter.frame("robot_1")
        state = adapter.snapshot().robots[0]
        assert frame is not None and frame.frame_id == "7" and frame.jpeg == jpeg
        assert state.connection is Connection.ONLINE
        assert state.pose is not None and state.pose.frame_id == "odom"
        assert state.tf_valid is False and state.tf_reason_code == "MAP_TF_UNVERIFIED"
        adapter._mark_disconnected("robot_1")
        assert adapter.frame("robot_1") is None

    asyncio.run(exercise())


def test_rosbridge_composes_map_tf_with_odom_and_exposes_map_pose() -> None:
    async def exercise() -> None:
        adapter = RosbridgeAdapter(load_ros_config())
        await adapter.handle_publish("robot_1", "/odom", {
            "header": {"frame_id": "odom"}, "child_frame_id": "base_footprint",
            "pose": {"pose": {"position": {"x": 0.5, "y": 0.0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}},
            "twist": {"twist": {"linear": {"x": 0}, "angular": {"z": 0}}},
        })
        await adapter.handle_publish("robot_1", "/tf", {"transforms": [{
            "header": {"frame_id": "map"}, "child_frame_id": "odom",
            "transform": {"translation": {"x": 1.0, "y": 2.0}, "rotation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        }]})
        state = adapter.snapshot().robots[0]
        assert state.pose is not None
        assert state.pose.model_dump() == {"x": 1.5, "y": 2.0, "yaw": 0.0, "frame_id": "map"}
        assert state.tf_valid is True and state.tf_reason_code is None

    asyncio.run(exercise())


def test_rosbridge_projects_laserscan_into_map_and_expires_old_points() -> None:
    async def exercise() -> None:
        now = [datetime(2026, 1, 1, tzinfo=UTC)]
        adapter = RosbridgeAdapter(load_ros_config(), clock=lambda: now[0])
        adapter._mark_connected("robot_2")
        await adapter.handle_publish("robot_2", "/tf_static", {"transforms": [{
            "header": {"frame_id": "map"}, "child_frame_id": "laser",
            "transform": {"translation": {"x": 1.0, "y": 2.0}, "rotation": {"x": 0, "y": 0, "z": 0, "w": 1}},
        }]})
        await adapter.handle_publish("robot_2", "/scan", {
            "header": {"frame_id": "laser", "stamp": {"sec": 1_767_225_600, "nanosec": 0}},
            "angle_min": 0.0, "angle_increment": math.pi / 2,
            "range_min": 0.05, "range_max": 8.0,
            "ranges": [0.1, 1.0, None, 9.0],
        })
        scan = adapter.sensor_layers("robot_2")["scan"]
        assert scan["state"] == "OK" and scan["frame_id"] == "map"
        assert scan["point_count"] == 2 and scan["min_range_m"] == pytest.approx(0.1)
        assert scan["points"][0] == pytest.approx({"x": 1.1, "y": 2.0, "range_m": 0.1})
        assert scan["points"][1] == pytest.approx({"x": 1.0, "y": 3.0, "range_m": 1.0})

        now[0] += timedelta(seconds=2)
        stale = adapter.sensor_layers("robot_2")["scan"]
        assert stale["state"] == "STALE" and stale["points"] == []
        assert stale["reason_code"] == "SCAN_STALE"

    asyncio.run(exercise())


def test_rosbridge_publishes_initial_pose_and_manual_velocity_to_mediator_topics() -> None:
    async def exercise() -> None:
        sockets = {"ws://robot-1.local:9090": FakeSocket(), "ws://robot-2.local:9091": FakeSocket()}

        async def connect(url: str, **_kwargs):
            return sockets[url]

        adapter = RosbridgeAdapter(load_ros_config(), connect_factory=connect)
        await adapter.connect()
        await asyncio.sleep(0)
        initial = await adapter.execute(CommandRequest(command_id=uuid4(), robot_id="robot_1", operation="initial_pose", parameters={"pose": {"x": 1.0, "y": 2.0, "yaw": 0.5, "frame_id": "map"}}))
        velocity = await adapter.publish_manual_velocity("robot_1", 0.1, 0.2)
        assert initial.accepted and velocity.accepted
        sent = sockets["ws://robot-1.local:9090"].sent
        initial_message = next(item for item in sent if item.get("op") == "publish" and item.get("topic") == "/initialpose")
        assert initial_message["type"] == "geometry_msgs/msg/PoseWithCovarianceStamped"
        assert initial_message["msg"]["header"]["stamp"] == {"sec": 0, "nanosec": 0}
        assert initial_message["msg"]["pose"]["pose"]["position"]["x"] == 1.0
        velocity_message = next(item for item in sent if item.get("op") == "publish" and item.get("topic") == "/control/manual_velocity")
        assert velocity_message["type"] == "geometry_msgs/msg/TwistStamped"
        assert velocity_message["msg"]["twist"]["linear"]["x"] == 0.1
        await adapter.close()

    asyncio.run(exercise())


def test_ros_mode_starts_in_observation_mode_without_applying_mock_settings(tmp_path) -> None:
    # Unreachable deployment endpoints must not prevent the API from exposing
    # its stale/offline state or cause a mock settings application to be claimed.
    with TestClient(create_app("ros", database_path=tmp_path / "control.db")) as client:
        assert client.get("/health").json() == {"status": "ok", "mode": "ros"}


def test_rosbridge_reassembles_bounded_camera_fragments_and_drops_invalid_sets() -> None:
    async def exercise() -> None:
        adapter = RosbridgeAdapter(load_ros_config())
        jpeg = MockRobotAdapter().frame("robot_1").jpeg
        published = json.dumps({"op": "publish", "topic": "/camera/image_raw/compressed", "msg": {"data": base64.b64encode(jpeg).decode(), "header": {"seq": 11}}})
        split = [published[:100], published[100:500], published[500:]]
        for number in (2, 0, 1):
            data = split[number]
            await adapter._handle_raw("robot_1", json.dumps({"op": "fragment", "id": "camera-11", "num": number, "total": len(split), "data": data}))
        assert adapter.frame("robot_1") is not None

        await adapter._handle_raw("robot_1", json.dumps({"op": "fragment", "id": "oversized", "num": 0, "total": 65, "data": "x"}))
        await adapter._handle_raw("robot_1", json.dumps({"op": "fragment", "id": "camera-12", "num": 0, "total": 2, "data": "{" * (4 * 1024 * 1024 + 1)}))
        assert "oversized" not in adapter._fragments["robot_1"]
        assert "camera-12" not in adapter._fragments["robot_1"]

    asyncio.run(exercise())


def test_ros_sensor_layers_are_stale_until_the_first_scan_arrives() -> None:
    adapter = RosbridgeAdapter(load_ros_config())
    available = adapter.sensor_layers("robot_1")
    assert available["scan"]["state"] == "STALE"
    assert {item["state"] for item in available["costmaps"]} == {"STALE"}
    adapter._mark_connected("robot_1")
    waiting = adapter.sensor_layers("robot_1")
    assert waiting["scan"]["state"] == "STALE"
    assert waiting["scan"]["reason_code"] == "SCAN_UNAVAILABLE"
    assert {item["state"] for item in waiting["costmaps"]} == {"UNSUPPORTED"}


def test_battery_updates_do_not_refresh_an_old_pose() -> None:
    async def exercise() -> None:
        now = [datetime(2026, 1, 1, tzinfo=UTC)]
        adapter = RosbridgeAdapter(load_ros_config(), clock=lambda: now[0])
        await adapter.handle_publish("robot_1", "/odom", {
            "header": {"frame_id": "odom"},
            "pose": {"pose": {"position": {"x": 1.0, "y": 2.0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}},
            "twist": {"twist": {"linear": {"x": 0.0}, "angular": {"z": 0.0}}},
        })
        now[0] += timedelta(seconds=2)
        await adapter.handle_publish("robot_1", "/battery/percent", {"data": 86.0})
        observed = StateStore(adapter.snapshot, clock=lambda: now[0]).snapshot().robots[0]
        assert observed.pose_freshness is Freshness.STALE
        assert observed.battery_freshness is Freshness.FRESH

    asyncio.run(exercise())


def test_disconnect_and_reconnect_do_not_revive_old_telemetry() -> None:
    async def exercise() -> None:
        now = [datetime(2026, 1, 1, tzinfo=UTC)]
        adapter = RosbridgeAdapter(load_ros_config(), clock=lambda: now[0])
        await adapter.handle_publish("robot_1", "/odom", {
            "header": {"frame_id": "odom"},
            "pose": {"pose": {"position": {"x": 1.0, "y": 2.0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}},
            "twist": {"twist": {"linear": {"x": 0.0}, "angular": {"z": 0.0}}},
        })
        await adapter.handle_publish("robot_1", "/battery/percent", {"data": 86.0})
        await adapter.handle_publish("robot_1", "/control/status", {"mode": "MANUAL", "stop_latched": True, "capabilities": ["navigate"]})
        adapter._mark_disconnected("robot_1")
        now[0] += timedelta(milliseconds=100)
        adapter._mark_connected("robot_1")
        robot = adapter.snapshot().robots[0]
        assert robot.pose is None
        assert robot.pose_freshness is Freshness.UNKNOWN
        assert robot.battery_percent is None and robot.battery_freshness is Freshness.UNKNOWN
        assert robot.mode.value == "UNKNOWN" and robot.stop_latched is None and robot.capabilities == []

    asyncio.run(exercise())


def test_camera_sensor_stales_while_odom_continues() -> None:
    async def exercise() -> None:
        now = [datetime(2026, 1, 1, tzinfo=UTC)]
        adapter = RosbridgeAdapter(load_ros_config(), clock=lambda: now[0])
        jpeg = MockRobotAdapter().frame("robot_1").jpeg
        await adapter.handle_publish("robot_1", "/camera/image_raw/compressed", {"data": base64.b64encode(jpeg).decode()})
        now[0] += timedelta(seconds=2, milliseconds=1)
        await adapter.handle_publish("robot_1", "/odom", {
            "header": {"frame_id": "odom"},
            "pose": {"pose": {"position": {"x": 1.0, "y": 2.0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}},
            "twist": {"twist": {"linear": {"x": 0.0}, "angular": {"z": 0.0}}},
        })
        camera = next(sensor for sensor in adapter.snapshot().robots[0].sensors if sensor.name == "camera")
        assert camera.state.value == "STALE"
        assert adapter.frame("robot_1") is None

    asyncio.run(exercise())


def test_path_publish_updates_bounded_robot_snapshot_and_emits_layer_event() -> None:
    async def exercise() -> None:
        adapter = RosbridgeAdapter(load_ros_config())
        await adapter.handle_publish("robot_1", "/plan", {
            "header": {"frame_id": "map"},
            "poses": [{"pose": {"position": {"x": float(index), "y": 2.0}}} for index in range(205)],
        })
        robot = adapter.snapshot().robots[0]
        assert len(robot.path) == 200
        assert robot.path[0].x == 0.0 and robot.path[-1].x == 199.0
        event = adapter.drain_events()[-1]
        assert event.kind == "path" and len(event.payload["points"]) == 200

    asyncio.run(exercise())


def test_fragment_buffers_limit_active_ids_and_event_consumers_close() -> None:
    async def exercise() -> None:
        adapter = RosbridgeAdapter(load_ros_config())
        for number in range(17):
            adapter._accept_fragment("robot_1", {"id": f"partial-{number}", "num": 0, "total": 2, "data": "{"})
        assert len(adapter._fragments["robot_1"]) == 16
        assert "partial-0" not in adapter._fragments["robot_1"]

        async def consume() -> bool:
            async for _event in adapter.events():
                pass
            return True

        consumer = asyncio.create_task(consume())
        await asyncio.sleep(0)
        await adapter.close()
        assert await asyncio.wait_for(consumer, timeout=1) is True

    asyncio.run(exercise())


def test_secure_bridge_uses_tls_context_and_environment_token(monkeypatch) -> None:
    async def exercise() -> None:
        data = load_ros_config().model_dump(mode="python")
        data["robots"][0]["bridge_url"] = "wss://robot-1.local:9090"
        data["robots"][0]["security"] = {"verify_tls": True, "authorization_token_env": "ROBOT_1_BRIDGE_TOKEN"}
        config = RosbridgeConfig.model_validate(data)
        captured: dict[str, object] = {}

        async def connect(url: str, **kwargs):
            captured.update({"url": url, **kwargs})
            return object()

        adapter = RosbridgeAdapter(config, connect_factory=connect)
        monkeypatch.setenv("ROBOT_1_BRIDGE_TOKEN", "test-only-token")
        await adapter._open("robot_1")
        assert captured["url"] == "wss://robot-1.local:9090"
        assert isinstance(captured["ssl"], ssl.SSLContext)
        assert captured["additional_headers"] == {"Authorization": "Bearer test-only-token"}
        assert "test-only-token" not in str(config.model_dump())

    asyncio.run(exercise())


def test_service_response_must_arrive_on_the_originating_robot_socket() -> None:
    async def exercise() -> None:
        sockets = {"ws://robot-1.local:9090": FakeSocket(respond_to_calls=False), "ws://robot-2.local:9091": FakeSocket(respond_to_calls=False)}

        async def connect(url: str, **_kwargs):
            return sockets[url]

        adapter = RosbridgeAdapter(enabled_config(), connect_factory=connect)
        await adapter.connect()
        await asyncio.sleep(0)
        command = CommandRequest(command_id=uuid4(), robot_id="robot_1", operation="stop")
        task = asyncio.create_task(adapter.execute(command))
        await asyncio.sleep(0)
        call_id = next(item["id"] for item in sockets["ws://robot-1.local:9090"].sent if item["op"] == "call_service")
        response = json.dumps({"op": "service_response", "id": call_id, "result": True, "values": {"accepted": True}})
        await adapter._handle_raw("robot_2", response, sockets["ws://robot-2.local:9091"])
        assert task.done() is False
        await adapter._handle_raw("robot_1", response, sockets["ws://robot-1.local:9090"])
        assert (await task).accepted is True
        await adapter.close()

    asyncio.run(exercise())


def test_rosbridge_map_streaming_subscribe_route_and_unsubscribe() -> None:
    async def exercise() -> None:
        config = load_ros_config()
        sockets = {robot.bridge_url: FakeSocket() for robot in config.robots}

        async def connect(url: str, **_kwargs):
            return sockets[url]

        adapter = RosbridgeAdapter(config, connect_factory=connect)
        await adapter.connect()
        await asyncio.sleep(0)
        seen: list[tuple[str, dict]] = []
        adapter.map_handler = lambda robot_id, msg: seen.append((robot_id, msg))
        robot_1_socket = sockets[config.robots[0].bridge_url]
        sent_topics = lambda: [item["topic"] for item in robot_1_socket.sent]

        await adapter.set_map_streaming("robot_1", True)
        assert "/map" in sent_topics()

        grid = {"info": {"width": 1, "height": 1, "resolution": 0.05, "origin": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}}}, "data": [0]}
        await robot_1_socket.incoming.put(json.dumps({"op": "publish", "topic": "/map", "msg": grid}))
        await asyncio.sleep(0)
        assert seen and seen[0][0] == "robot_1" and seen[0][1]["data"] == [0]

        await adapter.set_map_streaming("robot_1", False)
        unsubscribed = [item for item in robot_1_socket.sent if item["op"] == "unsubscribe"]
        assert any(item["topic"] == "/map" for item in unsubscribed)

        # robot_2 socket never received map ops (robot_1-only streaming)
        robot_2_socket = sockets[config.robots[1].bridge_url]
        assert "/map" not in [item["topic"] for item in robot_2_socket.sent]
        await adapter.close()

    asyncio.run(exercise())
