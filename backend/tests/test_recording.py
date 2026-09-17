"""RED: recording service stores camera frames + manifest for later learning."""

from pinky_control_center.camera_service import CameraFrame


def _frame(i: int) -> CameraFrame:
    from datetime import datetime, timezone

    now = datetime.now(timezone.utc)
    return CameraFrame(
        robot_id="robot_2",
        frame_id=f"f{i}",
        captured_at=now,
        received_at=now,
        width=640,
        height=480,
        jpeg=b"\xff\xd8fakejpeg" + bytes([i]),
    )


def test_recording_stores_frames_and_manifest(tmp_path):
    from pinky_control_center.recording_service import RecordingService

    svc = RecordingService(root=tmp_path / "recordings")
    run = svc.start("robot_2", label="test-run")
    assert run["robot_id"] == "robot_2"
    assert svc.record("robot_2", _frame(0)) is True
    assert svc.record("robot_2", _frame(1)) is True
    info = svc.stop("robot_2")
    assert info["frames"] == 2
    assert (tmp_path / "recordings" / "test-run" / "robot_2" / "meta.jsonl").exists()
    assert len(list((tmp_path / "recordings" / "test-run" / "robot_2").glob("*.jpg"))) == 2


def test_recording_ignores_unknown_robot(tmp_path):
    from pinky_control_center.recording_service import RecordingService

    svc = RecordingService(root=tmp_path / "recordings")
    assert svc.record("robot_9", _frame(0)) is False
    try:
        svc.stop("robot_9")
    except KeyError:
        pass
    else:
        raise AssertionError("stop on idle robot must raise KeyError")
