"""Pure joystick gesture detection (spec ARCHITECTURE.md §3.6).

Ported verbatim from ``flir_joy_bridge.py``'s ``CircleDetector``
(behaviour must not change — a JavaScript twin of this class runs in
the browser and the two must agree numerically). No I/O, no clock: the
detector only consumes stick samples via ``update(x, y)``.
"""

from __future__ import annotations

import math

CIRCLE_MIN_R = 0.4  # minimum stick radius to count for circle detection


class CircleDetector:
    """Detects a full 360 degree rotation of a joystick stick.

    Tracks accumulated (unwrapped) angle change over a sliding window;
    fires when the accumulated total reaches +/- 2*pi. Resets whenever
    the stick returns to the centre (radius < min_r).
    """

    def __init__(self, min_r: float = CIRCLE_MIN_R, window: int = 120):
        self._min_r = min_r
        self._window = window
        self._history: list[float] = []

    def update(self, x: float, y: float) -> bool:
        r = math.hypot(x, y)
        if r < self._min_r:
            self._history.clear()
            return False
        angle = math.atan2(y, x)
        self._history.append(angle)
        if len(self._history) > self._window:
            self._history.pop(0)
        if len(self._history) < 20:
            return False
        total = 0.0
        for i in range(1, len(self._history)):
            d = self._history[i] - self._history[i - 1]
            if d > math.pi:
                d -= 2 * math.pi
            if d < -math.pi:
                d += 2 * math.pi
            total += d
        return abs(total) >= 2 * math.pi

    def reset(self) -> None:
        self._history.clear()
