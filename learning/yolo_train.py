"""YOLO training entry: recordings layout -> ultralytics YOLO dataset + train.

Usage (PC, CPU-friendly default):
    backend/.venv/bin/python learning/yolo_train.py --run recordings/<label>/robot_2 --model yolo11n.pt --epochs 20

Steps: link images/labels into a YOLO dataset dir, write data.yaml,
train, report best.pt. Requires `pip install ultralytics` in the venv.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


def build_yolo_dataset(run_dir: Path, out_dir: Path, val_ratio: float = 0.2) -> Path:
    from pinky_control_center.dataset import read_manifest, split_rows

    rows = read_manifest(run_dir)
    train_rows, val_rows = split_rows(rows, val_ratio)
    for split, subset in (("train", train_rows), ("val", val_rows)):
        (out_dir / "images" / split).mkdir(parents=True, exist_ok=True)
        (out_dir / "labels" / split).mkdir(parents=True, exist_ok=True)
        for row in subset:
            stem = f"{int(row['frame_id'][1:]):06d}" if str(row["frame_id"]).startswith("f") else str(row["frame_id"])
            src_img = run_dir / f"{stem}.jpg"
            src_lbl = run_dir / "labels" / f"{stem}.txt"
            if src_img.exists():
                shutil.copy(src_img, out_dir / "images" / split / f"{stem}.jpg")
            if src_lbl.exists():
                shutil.copy(src_lbl, out_dir / "labels" / split / f"{stem}.txt")
    (out_dir / "data.yaml").write_text(
        f"path: {out_dir}\ntrain: images/train\nval: images/val\nnames: {{0: target}}\n",
        encoding="utf-8",
    )
    return out_dir / "data.yaml"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--out", default="learning/yolo_dataset")
    ap.add_argument("--model", default="yolo11n.pt")
    ap.add_argument("--epochs", type=int, default=20)
    ap.add_argument("--imgsz", type=int, default=640)
    args = ap.parse_args()
    data = build_yolo_dataset(Path(args.run), Path(args.out))
    from ultralytics import YOLO

    model = YOLO(args.model)
    model.train(data=str(data), epochs=args.epochs, imgsz=args.imgsz)
    print("best:", model.ckpt_path if hasattr(model, "ckpt_path") else "runs/detect/train/weights/best.pt")


if __name__ == "__main__":
    main()
