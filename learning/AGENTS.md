<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-09-17 | Updated: 2026-09-17 -->

# learning

## Purpose
Line-follow ML experiments feeding off platform dataset recordings: a classical OpenCV baseline (no training), a YOLO dataset converter/trainer, and the dataset loader they share. Exploratory area — not part of the deployed platform.

## Key Files
| File | Description |
|------|-------------|
| `dataset_loader.py` | Reads the platform `recordings/` layout (`meta.jsonl` manifest + YOLO-format `labels/*.txt`), yields `(jpg_path, boxes)` pairs; `dataset_stats()` returns `{frames, labeled, boxes}` |
| `opencv_baseline.py` | Classical baseline: `blob_steering()` pure function — largest HSV blob (default yellow 20–40 hue) → `(linear_mps, angular_rps)` steering; mirrors lap585 wall-follow logic for testability |
| `yolo_train.py` | Converts a recording run into an Ultralytics YOLO dataset (train/val split, `data.yaml`, single class `target`) then trains (`yolo11n.pt`) |
| `test_learning.py` | Pure-function pytest suite: blob steering + dataset stats from a synthetic recording |

## For AI Agents

### Working In This Directory
- Run tests from the **repo root** with the backend venv: `backend/.venv/bin/python -m pytest learning/test_learning.py -q` (imports are `from learning.…`; no `__init__.py` — pytest rootdir insertion handles it).
- Ultralytics is an optional extra, NOT in `backend/pyproject.toml` — install ad hoc (`pip install ultralytics`) only when training.
- Keep new logic as pure functions (like `blob_steering`) so it stays testable without frames on disk.

### Testing Requirements
- `backend/.venv/bin/python -m pytest learning/test_learning.py -q` from repo root.

## Dependencies

### External
- OpenCV (`opencv-python-headless`) and `numpy` — pinned backend deps
- Ultralytics YOLO (optional, for `yolo_train.py` only)

### Internal
- Recording layout produced by `../backend/pinky_control_center/recording_service.py` + `dataset.py`.
- `opencv_baseline.blob_steering` intentionally mirrors the steering logic in `../lap585/pinky_wallfollow.py`.

<!-- MANUAL: -->
