import asyncio
import json
import time
from pathlib import Path

import pytest

from pinky_control_center.mapping_runner import MappingRunner, build_env, parse_event

FAKE = """#!/bin/bash
echo "===="
echo "[1단계  벽 따라가기 — 통로를 체계적으로 훑는다]"
echo "  (-1.0,-0.2) 이동  1.50m 앞 0.30"
echo "[1단계 결과] 관측 100 -> 200칸 (+100)"
echo "[4단계  출발 지점으로 복귀]"
echo "맵 저장 완료"
exit 0
"""
FAILER = "#!/bin/bash\necho '좌표계 손상'\necho '맵 저장 실패'\nexit 1\n"
LONGER = "#!/bin/bash\nsleep 30\n"


def _write(tmp_path: Path, body: str) -> Path:
    script = tmp_path / "run_lap_real.sh"
    script.write_text(body, encoding="utf-8")
    script.chmod(0o755)
    return tmp_path


def test_parse_event_stage_and_saved():
    assert parse_event("==== [1단계  벽 따라가기]")["stage_index"] == 0
    assert parse_event("맵 저장 완료")["kind"] == "saved"
    assert parse_event("hello") is None


def test_build_env_sets_domain_and_overlay(tmp_path):
    env = build_env(tmp_path / "nav_overlay", 12)
    assert env["ROS_DOMAIN_ID"] == "12"
    assert "ROS_LOCALHOST_ONLY" not in env
    assert env["PINKY_TRACK_W"] == "0"
    assert env["PINKY_TRACK_H"] == "0"
    assert env["ROS_AUTOMATIC_DISCOVERY_RANGE"] == "SUBNET"
    lib = str(tmp_path / "nav_overlay" / "opt" / "ros" / "jazzy" / "lib")
    assert lib in env["LD_LIBRARY_PATH"]
    bin_dir = str(tmp_path / "nav_overlay" / "opt" / "ros" / "jazzy" / "bin")
    assert env["PATH"].startswith(bin_dir)


def test_start_runs_to_completed(tmp_path):
    lap = _write(tmp_path, FAKE)
    done = []
    runner = MappingRunner(
        lap585_dir=lap,
        overlay_dir=tmp_path / "ov",
        state_dir=tmp_path / "st",
        on_complete=lambda: done.append(1),
    )
    out = asyncio.run(runner.start("robot_1"))
    assert out["state"] == "COMPLETED"
    assert runner.status()["stage_index"] == 3
    assert done == [1]
    log = (tmp_path / "st" / "mapping" / "last_run.log").read_text(encoding="utf-8")
    assert log.count("[1단계") >= 1


def test_corrupt_run_fails(tmp_path):
    lap = _write(tmp_path, FAILER)
    runner = MappingRunner(lap585_dir=lap, overlay_dir=tmp_path / "ov", state_dir=tmp_path / "st")
    out = asyncio.run(runner.start("robot_1"))
    assert out["state"] == "FAILED"
    assert out["reason"] == "MAP_CORRUPTED"


def test_cancel_kills_long_run(tmp_path):
    lap = _write(tmp_path, LONGER)
    runner = MappingRunner(lap585_dir=lap, overlay_dir=tmp_path / "ov", state_dir=tmp_path / "st")

    async def scenario():
        task = asyncio.ensure_future(runner.start("robot_1"))
        await asyncio.sleep(0.3)
        out = await runner.cancel()
        await asyncio.wait_for(task, 8)
        return out

    assert asyncio.run(scenario())["state"] == "PAUSED"
    assert runner.status()["state"] == "PAUSED"


def test_missing_script_rejected(tmp_path):
    runner = MappingRunner(lap585_dir=tmp_path / "nope", overlay_dir=tmp_path, state_dir=tmp_path / "st")
    with pytest.raises(ValueError):
        asyncio.run(runner.start("robot_1"))


def test_unsupported_robot_rejected(tmp_path):
    lap = _write(tmp_path, FAKE)
    runner = MappingRunner(lap585_dir=lap, overlay_dir=tmp_path, state_dir=tmp_path / "st")
    with pytest.raises(ValueError):
        asyncio.run(runner.start("robot_2"))


def test_preflight_offline_rejected(tmp_path):
    lap = _write(tmp_path, FAKE)
    runner = MappingRunner(
        lap585_dir=lap, overlay_dir=tmp_path, state_dir=tmp_path / "st",
        preflight_state=lambda: (False, True),
    )
    with pytest.raises(ValueError):
        asyncio.run(runner.start("robot_1"))


def test_preflight_scan_stale_rejected(tmp_path):
    lap = _write(tmp_path, FAKE)
    runner = MappingRunner(
        lap585_dir=lap, overlay_dir=tmp_path, state_dir=tmp_path / "st",
        preflight_state=lambda: (True, False),
    )
    with pytest.raises(ValueError):
        asyncio.run(runner.start("robot_1"))


def test_orphan_detected_and_killed(tmp_path):
    import subprocess as sp

    sleeper = sp.Popen(["sleep", "30"], start_new_session=True)
    try:
        mapping_dir = tmp_path / "st" / "mapping"
        mapping_dir.mkdir(parents=True)
        (mapping_dir / "run.json").write_text(
            json.dumps({"pid": sleeper.pid, "mapping_id": "m1"}), encoding="utf-8")
        runner = MappingRunner(lap585_dir=tmp_path, overlay_dir=tmp_path, state_dir=tmp_path / "st")
        orphan = runner.detect_orphan()
        assert orphan is not None and orphan["pid"] == sleeper.pid
        asyncio.run(runner.kill_orphan())
        assert sleeper.poll() is not None or True
        for _ in range(30):
            if sleeper.poll() is not None:
                break
            time.sleep(0.1)
        assert sleeper.poll() is not None
    finally:
        if sleeper.poll() is None:
            sleeper.kill()
