"""Drive patterns: mapping coverage legs as (linear, angular, seconds)."""

from __future__ import annotations


def square_legs(side_secs: float = 3.0, turn_secs: float = 3.2, speed: float = 0.1, turn_rate: float = 0.4) -> list:
    legs = []
    for _ in range(4):
        legs.append((speed, 0.0, side_secs))
        legs.append((0.0, turn_rate, turn_secs))
    return legs


def straight_legs(count: int = 3, drive_secs: float = 2.0, speed: float = 0.1) -> list:
    return [(speed, 0.0, drive_secs) for _ in range(int(count))]
