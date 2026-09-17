"""Dataset helpers (pure functions): manifest IO, splits, YOLO labels."""

from __future__ import annotations

import json
from pathlib import Path


def read_manifest(run_dir: str | Path) -> list[dict]:
    rows: list[dict] = []
    manifest = Path(run_dir) / "meta.jsonl"
    if not manifest.exists():
        return rows
    for line in manifest.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            rows.append(json.loads(line))
    return rows


def split_rows(rows: list[dict], val_ratio: float = 0.2) -> tuple[list[dict], list[dict]]:
    n = len(rows)
    n_val = max(0, min(n, round(n * val_ratio)))
    return rows[: n - n_val], rows[n - n_val :]


def write_yolo_label(run_dir: str | Path, stem: str, cls: int, cx: float, cy: float, w: float, h: float) -> Path:
    labels_dir = Path(run_dir) / "labels"
    labels_dir.mkdir(parents=True, exist_ok=True)
    path = labels_dir / f"{stem}.txt"
    path.write_text(f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n", encoding="utf-8")
    return path
