"""Dataset loader: recordings layout -> (image, label) pairs for learning."""

from __future__ import annotations

from pathlib import Path

from pinky_control_center.dataset import read_manifest


def iter_labeled_frames(run_dir: str | Path):
    """Yield (jpg_path, boxes) for frames that have a YOLO label file."""
    run_dir = Path(run_dir)
    for row in read_manifest(run_dir):
        stem = f"{int(row['frame_id'][1:]):06d}" if str(row["frame_id"]).startswith("f") else str(row["frame_id"])
        img = run_dir / f"{stem}.jpg"
        label = run_dir / "labels" / f"{stem}.txt"
        if not img.exists():
            continue
        boxes = []
        if label.exists():
            for line in label.read_text(encoding="utf-8").splitlines():
                parts = line.split()
                if len(parts) == 5:
                    boxes.append(tuple(map(float, parts)))
        yield img, boxes


def dataset_stats(run_dir: str | Path) -> dict:
    pairs = list(iter_labeled_frames(run_dir))
    labeled = sum(1 for _, boxes in pairs if boxes)
    return {"frames": len(pairs), "labeled": labeled, "boxes": sum(len(b) for _, b in pairs)}
