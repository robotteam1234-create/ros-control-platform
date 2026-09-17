"""Tests for learning helpers (pure functions, no hardware)."""

import numpy as np


def test_blob_steering_centers_target(tmp_path):
    import cv2

    from learning.opencv_baseline import blob_steering

    img = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(img, (200, 100), (260, 160), (0, 255, 255), -1)  # yellow right
    v, w = blob_steering(img)
    assert v > 0
    assert w < 0  # steer right (negative = clockwise toward blob)
    v0, w0 = blob_steering(np.zeros((240, 320, 3), dtype=np.uint8))
    assert (v0, w0) == (0.0, 0.0)


def test_dataset_stats_counts_labels(tmp_path):
    import json

    from learning.dataset_loader import dataset_stats

    run = tmp_path / "robot_2"
    run.mkdir(parents=True)
    rows = [
        {"frame_id": f"f{i}", "captured_at": None, "received_at": "2026-01-01T00:00:00+00:00", "width": 64, "height": 48}
        for i in range(3)
    ]
    (run / "meta.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    for i in range(3):
        (run / f"{i:06d}.jpg").write_bytes(b"\xff\xd8x")
    (run / "labels").mkdir()
    (run / "labels" / "000000.txt").write_text("0 0.5 0.5 0.2 0.2\n")
    stats = dataset_stats(run)
    assert stats == {"frames": 3, "labeled": 1, "boxes": 1}
