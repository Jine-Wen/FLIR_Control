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


def test_reset_discards_accumulated_progress():
    d = CircleDetector(min_r=0.4)
    for x, y in _ring_points(15, r=0.8, clockwise=True):
        d.update(x, y)
    assert d.progress > 0.0
    d.reset()
    assert d.progress == 0.0



def test_a_fast_full_circle_fires_even_with_few_samples():
    """Regression: the detector used to require >= 20 samples before it would
    report anything, so a quick flick of a genuine full circle was rejected and
    the operator had to keep going."""
    d = CircleDetector(min_r=0.4)
    fired = False
    for x, y in _ring_points(10, r=0.8, clockwise=True):  # 11 samples
        fired = d.update(x, y) or fired
    assert fired is True


def test_a_slow_full_circle_fires_however_many_samples_it_takes():
    """The worse half of the same regression. The old detector summed deltas
    across a 120-sample sliding window, so a circle drawn carefully enough to
    produce more than 120 samples had its earliest deltas dropped faster than
    new ones arrived -- the total could never reach 2*pi and the gesture was
    impossible, no matter how many turns were drawn."""
    d = CircleDetector(min_r=0.4)
    fired = False
    for x, y in _ring_points(400, r=0.8, clockwise=True):
        fired = d.update(x, y) or fired
        if fired:
            break
    assert fired is True



def test_exactly_at_threshold_two_pi_fires():
    """A full-resolution 2*pi accumulation should satisfy the >= 2*pi
    threshold (allowing for floating point slack)."""
    d = CircleDetector(min_r=0.4, window=120)
    n = 36  # 10 degree steps, well above the 20-sample minimum
    fired = False
    for x, y in _ring_points(n, r=0.8, clockwise=True):
        fired = d.update(x, y) or fired
    assert fired is True


def test_progress_reports_how_far_round_the_gesture_is():
    d = CircleDetector(min_r=0.4)
    for x, y in _ring_points(200, r=0.8, clockwise=True, start=0.0):
        if d.update(x, y):
            break
    assert d.progress == 1.0

    half = CircleDetector(min_r=0.4)
    for i in range(51):                      # sweep 0 -> pi, i.e. half a turn
        angle = math.pi * i / 50
        half.update(0.8 * math.cos(angle), 0.8 * math.sin(angle))
    assert 0.45 < half.progress < 0.55



def test_brushing_past_the_centre_does_not_wipe_progress():
    """Only a genuine return to rest resets. A gesture that dips toward the
    middle mid-sweep used to lose everything and start over, which is a large
    part of why unlocking took several attempts."""
    d = CircleDetector(min_r=0.4)
    for i in range(30):                      # most of a turn
        angle = 2 * math.pi * i / 40
        d.update(0.8 * math.cos(angle), 0.8 * math.sin(angle))
    before = d.progress
    assert before > 0.5

    d.update(0.25, 0.0)                      # brush past, above reset_r
    assert d.progress == before, "progress must survive a brief dip inward"

    d.update(0.05, 0.0)                      # genuine return to rest
    assert d.progress == 0.0


