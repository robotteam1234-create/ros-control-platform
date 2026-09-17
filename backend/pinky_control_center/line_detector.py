from __future__ import annotations
from dataclasses import dataclass
import cv2
import numpy as np

@dataclass(frozen=True)
class LineDetectorResult:
    found: bool
    offset: float
    area_ratio: float
    polarity: str

MIN_AREA_RATIO = 0.005
MAX_AREA_RATIO = 0.35
MIN_CONTRAST_STD = 10.0

def _polarity_mask(blur, invert: bool) -> np.ndarray:
    flags = cv2.THRESH_OTSU | (cv2.THRESH_BINARY_INV if invert else cv2.THRESH_BINARY)
    _, mask = cv2.threshold(blur, 0, 255, flags)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

def _line_candidate(mask: np.ndarray, pixels: int) -> tuple[float, float, int, int] | None:
    """Best in-band blob: (area_ratio, centroid_x, bbox_x, bbox_width) or None."""
    found = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # OpenCV 3 returns (image, contours, hierarchy); OpenCV 4+ returns (contours, hierarchy).
    contours = found[0] if len(found) == 2 else found[1]
    for c in sorted(contours, key=cv2.contourArea, reverse=True):
        area_ratio = float(cv2.contourArea(c) / pixels)
        if area_ratio < MIN_AREA_RATIO:
            break
        if area_ratio > MAX_AREA_RATIO:
            continue
        m = cv2.moments(c)
        if m["m00"] > 0:
            bx, _, bw, _ = cv2.boundingRect(c)
            return area_ratio, float(m["m10"] / m["m00"]), int(bx), int(bw)
    return None

def detect_line(jpeg: bytes, mode: str = "auto") -> LineDetectorResult:
    arr = np.frombuffer(jpeg, dtype=np.uint8)
    gray = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        return LineDetectorResult(False, 0.0, 0.0, "unknown")
    if mode not in ("auto", "white", "black"):
        raise ValueError("INVALID_VALUE")
    h, w = gray.shape
    roi = gray[h // 2 :, :]
    blur = cv2.GaussianBlur(roi, (5, 5), 0)
    if float(blur.std()) < MIN_CONTRAST_STD:
        return LineDetectorResult(False, 0.0, 0.0, mode if mode != "auto" else "unknown")
    pixels = blur.shape[0] * blur.shape[1]
    if mode == "auto":
        candidates = []
        for polarity, invert in (("white", False), ("black", True)):
            c = _line_candidate(_polarity_mask(blur, invert), pixels)
            if c is not None:
                candidates.append((polarity, *c))
        if not candidates:
            return LineDetectorResult(False, 0.0, 0.0, "unknown")
        # Whole-floor blobs are rejected by MAX_AREA_RATIO. A followed line runs
        # from the robot toward the horizon, so it never hugs the left/right
        # frame edge: prefer interior blobs (beats floor fragments the line
        # splits off, and JPEG ghost rings lose on area), fall back to edges.
        interior = [t for t in candidates if t[3] > 2 and t[3] + t[4] < w - 2]
        polarity, area_ratio, cx, _, _ = max(interior or candidates, key=lambda t: t[1])
    else:
        polarity = mode
        c = _line_candidate(_polarity_mask(blur, mode == "black"), pixels)
        if c is None:
            return LineDetectorResult(False, 0.0, 0.0, polarity)
        area_ratio, cx = c[0], c[1]
    offset = max(-1.0, min(1.0, (cx - w / 2) / (w / 2)))
    return LineDetectorResult(True, offset, area_ratio, polarity)
