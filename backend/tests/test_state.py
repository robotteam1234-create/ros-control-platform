from __future__ import annotations

from datetime import UTC, datetime, timedelta

from pinky_control_center.adapters.mock import MockRobotAdapter
from pinky_control_center.models import Freshness
from pinky_control_center.state_store import StateStore


def test_pose_and_battery_freshness_use_independent_thresholds() -> None:
    base = datetime(2026, 1, 1, tzinfo=UTC)
    source = MockRobotAdapter().snapshot()
    robots = [robot.model_copy(update={"received_at": base}) for robot in source.robots]
    snapshot = source.model_copy(update={"robots": robots})
    current = base + timedelta(seconds=2)
    store = StateStore(lambda: snapshot, clock=lambda: current)
    stale_pose = store.snapshot().robots[0]
    assert stale_pose.pose_freshness is Freshness.STALE
    assert stale_pose.battery_freshness is Freshness.FRESH
    current = base + timedelta(seconds=16)
    stale_battery = store.snapshot().robots[0]
    assert stale_battery.battery_freshness is Freshness.STALE


def test_snapshot_sequence_restarts_client_from_a_full_snapshot() -> None:
    store = StateStore(MockRobotAdapter().snapshot)
    first = store.snapshot()
    second = store.snapshot()
    assert second.seq == first.seq + 1
    assert {robot.robot_id for robot in second.robots} == {"robot_1", "robot_2"}


def test_disconnected_robot_rejoins_when_fresh_data_arrives() -> None:
    base = datetime(2026, 1, 1, tzinfo=UTC)
    source = MockRobotAdapter().snapshot()
    robots = [robot.model_copy(update={"received_at": base}) for robot in source.robots]
    snapshot = source.model_copy(update={"robots": robots})
    now = base + timedelta(seconds=10)
    store = StateStore(lambda: snapshot, clock=lambda: now)
    store.disconnect("robot_1")
    assert store.snapshot().robots[0].connection.value == "OFFLINE"
    now = base
    rejoined = store.snapshot().robots[0]
    assert rejoined.connection.value == "ONLINE"
    assert "robot_1" not in store.disconnected
