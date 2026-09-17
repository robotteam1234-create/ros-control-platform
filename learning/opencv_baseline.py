"""OpenCV baseline: color-blob follower producing velocity commands.

Classical (no training) counterpart to the YOLO path: finds the largest
blob in an HSV range and steers toward its horizontal offset. Mirrors the
lap585 wall-follow logic in a pure function for testability.
"""

from __future__ import annotations

import cv2
import numpy as np


def blob_steering(
    bgr,
    lower_hsv=(20, 100, 100),
    upper_hsv=(40, 255, 255),
    base_speed: float = 0.1,
    gain: float = 1.0,
) -> tuple[float, float]:
    """Return (linear_mps, angular_rps) steering toward the largest blob."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array(lower_hsv, dtype=np.uint8),
        np.array(upper_hsv, dtype=np.uint8),
    )
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0.0, 0.0
    biggest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(biggest) < 500:
        return 0.0, 0.0
    m = cv2.moments(biggest)
    cx = m["m10"] / (m["m00"] + 1e-9)
    err = (cx / bgr.shape[1]) - 0.5
    return base_speed, float(-gain * err)
