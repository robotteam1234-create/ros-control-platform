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

WHITE_THRESH = 200
BLACK_THRESH = 60
MIN_AREA_RATIO = 0.005

def detect_line(jpeg: bytes, mode: str = "auto") -> LineDetectorResult:
    arr = np.frombuffer(jpeg, dtype=np.uint8)
    gray = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        return LineDetectorResult(False, 0.0, 0.0, "unknown")
    h, w = gray.shape
    roi = gray[h // 2 :, :]
    blur = cv2.GaussianBlur(roi, (5, 5), 0)
    polarity = mode
    if mode == "auto":
        polarity = "white" if float(blur.mean()) < 127 else "black"
    if polarity == "white":
        _, mask = cv2.threshold(blur, WHITE_THRESH, 255, cv2.THRESH_BINARY)
    elif polarity == "black":
        _, mask = cv2.threshold(blur, BLACK_THRESH, 255, cv2.THRESH_BINARY_INV)
    else:
        raise ValueError("INVALID_VALUE")
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return LineDetectorResult(False, 0.0, 0.0, polarity)
    best = max(contours, key=cv2.contourArea)
    area_ratio = float(cv2.contourArea(best) / (roi.shape[0] * roi.shape[1]))
    if area_ratio < MIN_AREA_RATIO:
        return LineDetectorResult(False, 0.0, area_ratio, polarity)
    m = cv2.moments(best)
    if m["m00"] <= 0:
        return LineDetectorResult(False, 0.0, area_ratio, polarity)
    cx = float(m["m10"] / m["m00"])
    offset = max(-1.0, min(1.0, (cx - w / 2) / (w / 2)))
    return LineDetectorResult(True, offset, area_ratio, polarity)
