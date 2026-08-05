"""Tests for flir_ptz.control.gestures.CircleDetector.

Ported verbatim from flir_joy_bridge.py's CircleDetector — behaviour
must match exactly (a JS twin runs in the browser). Standard library
only, no I/O, no clock.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from flir_ptz.control.gestures import CircleDetector  # noqa: E402


def _ring_points(n: int, r: float = 0.8, clockwise: bool = True, start: float = 0.0):
    """Yield (x, y) samples walking evenly around a circle of radius r.

    In screen/stick convention here we just use atan2(y, x) directly,
    matching CircleDetector.update's own angle computation. clockwise
    here just means decreasing angle (arbitrary orientation label; the
    detector is direction-agnostic and reports via sign of `total`).
    """
    step = (2 * math.pi) / n
    if clockwise:
        step = -step
    for i in range(n + 1):
        theta = start + step * i
        yield (r * math.cos(theta), r * math.sin(theta))


def test_full_circle_clockwise_fires():
    d = CircleDetector(min_r=0.4, window=120)
    fired = False
    for x, y in _ring_points(40, r=0.8, clockwise=True):
        fired = d.update(x, y) or fired
    assert fired is True


def test_full_circle_counter_clockwise_fires():
    d = CircleDetector(min_r=0.4, window=120)
    fired = False
    for x, y in _ring_points(40, r=0.8, clockwise=False):
        fired = d.update(x, y) or fired
    assert fired is True


def test_three_quarter_turn_then_reverse_never_fires():
    """3/4 turn one way then reverse back to start must NOT fire —
    the net accumulated angle never reaches 2*pi in one direction."""
    d = CircleDetector(min_r=0.4, window=120)
    fired = False

    # Walk 3/4 of the way around (270 degrees) in fine steps.
    n_forward = 30
    step = (2 * math.pi * 0.75) / n_forward
    theta = 0.0
    for _ in range(n_forward + 1):
        x, y = 0.8 * math.cos(theta), 0.8 * math.sin(theta)
        fired = d.update(x, y) or fired
        theta += step
    assert fired is False

    # Now reverse all the way back to the start.
    n_back = 30
    back_step = (2 * math.pi * 0.75) / n_back
    for _ in range(n_back + 1):
        theta -= back_step
        x, y = 0.8 * math.cos(theta), 0.8 * math.sin(theta)
        fired = d.update(x, y) or fired

    assert fired is False


def test_jitter_inside_deadzone_never_fires():
    """Small stick movements below min_r must be ignored (treated as
    centred) and never accumulate toward a circle."""
    d = CircleDetector(min_r=0.4, window=120)
    fired = False
    # Tiny jitter around the origin, radius always < min_r.
    for i in range(200):
        angle = i * 0.37
        x, y = 0.1 * math.cos(angle), 0.1 * math.sin(angle)
        fired = d.update(x, y) or fired
    assert fired is False


def test_return_to_centre_resets_accumulation():
    d = CircleDetector(min_r=0.4, window=120)

    # Walk half way around — not enough to fire alone.
    n = 25
    step = (2 * math.pi * 0.5) / n
    theta = 0.0
    fired = False
    for _ in range(n + 1):
        x, y = 0.8 * math.cos(theta), 0.8 * math.sin(theta)
        fired = d.update(x, y) or fired
        theta += step
    assert fired is False

    # Return to centre: this must reset accumulated history.
    assert d.update(0.0, 0.0) is False

    # Continue the *same* remaining half circle from where we left off.
    # Because history was cleared, this alone (also only half a turn)
    # must not fire either.
    fired = False
    for _ in range(n + 1):
        x, y = 0.8 * math.cos(theta), 0.8 * math.sin(theta)
        fired = d.update(x, y) or fired
        theta += step
    assert fired is False


def test_reset_method_clears_history_explicitly():
    d = CircleDetector(min_r=0.4, window=120)
    for x, y in _ring_points(15, r=0.8, clockwise=True):
        d.update(x, y)
    d.reset()
    # After an explicit reset, fewer than 20 subsequent samples cannot
    # fire regardless of what happened before.
    fired = False
    for x, y in _ring_points(10, r=0.8, clockwise=True):
        fired = d.update(x, y) or fired
    assert fired is False


def test_fewer_than_twenty_samples_never_fires_even_if_full_circle():
    """Needs >= 20 samples: a full circle walked in very few big jumps
    must not fire even though the angular math alone would sum to 2*pi."""
    d = CircleDetector(min_r=0.4, window=120)
    fired = False
    for x, y in _ring_points(10, r=0.8, clockwise=True):  # only 11 samples
        fired = d.update(x, y) or fired
    assert fired is False


def test_exactly_at_threshold_two_pi_fires():
    """A full-resolution 2*pi accumulation should satisfy the >= 2*pi
    threshold (allowing for floating point slack)."""
    d = CircleDetector(min_r=0.4, window=120)
    n = 36  # 10 degree steps, well above the 20-sample minimum
    fired = False
    for x, y in _ring_points(n, r=0.8, clockwise=True):
        fired = d.update(x, y) or fired
    assert fired is True


def test_window_caps_history_length():
    """History never grows past `window` samples."""
    d = CircleDetector(min_r=0.4, window=120)
    for i in range(500):
        angle = i * 0.05
        d.update(0.8 * math.cos(angle), 0.8 * math.sin(angle))
    assert len(d._history) <= 120


def test_radius_exactly_at_min_r_counts_as_inside_not_centred():
    """r < min_r resets; r == min_r should NOT reset (only strictly
    below the threshold clears history), matching `if r < self._min_r`."""
    d = CircleDetector(min_r=0.4, window=120)
    # x=min_r, y=0 -> r == min_r exactly.
    result = d.update(0.4, 0.0)
    assert result is False  # not enough samples yet, but not a reset either
    assert len(d._history) == 1
