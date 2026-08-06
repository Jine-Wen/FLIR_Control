"""Pure joystick gesture detection (spec ARCHITECTURE.md §3.6).

A JavaScript twin of this class runs in the browser (``web/app.js``), and the
two must stay numerically in step — the same gesture has to unlock the console
whether it is drawn with a mouse or a physical stick.

No I/O, no clock: the detector only consumes stick samples via ``update``.
"""

from __future__ import annotations

import math

CIRCLE_MIN_R = 0.4  # stick radius below which samples are ignored
#: Radius under which sweep progress is discarded. Deliberately lower than
#: CIRCLE_MIN_R so brushing past the centre mid-gesture costs nothing, while a
#: deliberate return to rest still resets.
CIRCLE_RESET_R = 0.18


class CircleDetector:
    """Detects a full 360-degree rotation of a joystick stick.

    Accumulates the unwrapped angle swept by the stick and fires once the total
    reaches a full turn in either direction.

    The accumulation is incremental — a running total plus the previous angle —
    rather than a sliding window of past samples. The window version this
    replaces could not detect a slow circle at all: it kept the last 120
    samples and summed the deltas across them, so a gesture drawn carefully
    enough to produce more than 120 samples had its earliest deltas discarded
    faster than new ones arrived, and the total never reached 2*pi no matter how
    many turns were drawn. It also required at least 20 samples before it would
    report anything, rejecting a quick flick of a genuine full circle. The
    usable band was therefore "neither slow nor fast", and a first attempt —
    typically slow and careful — fell outside it.

    Progress survives brief dips toward the centre and is only discarded when
    the stick genuinely returns to rest (radius < ``CIRCLE_RESET_R``).
    """

    def __init__(
        self,
        min_r: float = CIRCLE_MIN_R,
        reset_r: float = CIRCLE_RESET_R,
        window: int = 0,  # accepted and ignored; kept so old callers still work
    ) -> None:
        self._min_r = min_r
        self._reset_r = reset_r
        self._prev_angle: float | None = None
        self._total = 0.0

    @property
    def progress(self) -> float:
        """Fraction of a full turn swept so far, 0.0–1.0. Lets a UI show how
        far along the gesture is instead of leaving the operator guessing."""
        return min(1.0, abs(self._total) / (2 * math.pi))

    def update(self, x: float, y: float) -> bool:
        r = math.hypot(x, y)

        if r < self._reset_r:
            self.reset()
            return False

        # Between reset_r and min_r: hold progress, but don't measure — the
        # angle is too noisy near the centre to be meaningful.
        if r < self._min_r:
            self._prev_angle = None
            return False

        angle = math.atan2(y, x)
        if self._prev_angle is not None:
            delta = angle - self._prev_angle
            # Unwrap to (-pi, pi] so crossing the +/-pi seam reads as a small
            # step rather than a full turn backwards.
            if delta > math.pi:
                delta -= 2 * math.pi
            elif delta < -math.pi:
                delta += 2 * math.pi
            self._total += delta
        self._prev_angle = angle

        return abs(self._total) >= 2 * math.pi

    def reset(self) -> None:
        self._prev_angle = None
        self._total = 0.0
