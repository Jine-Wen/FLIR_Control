#!/usr/bin/env python3
"""Pure speed-profile math for PTZ axis control.

No I/O, no async, no rclpy. Must be importable with nothing but the Python
3.12 standard library.

``choose_speed`` is a hand-tuned, piecewise-linear speed curve validated on
real hardware. It is ported bit-for-bit from the original
``flir_ptz_node.py:_choose_speed`` / ``AZ_SPEED_PROFILE`` / ``EL_SPEED_PROFILE``
— do not "improve" it, do not change any constant.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class AxisProfile:
    """Piecewise speed-vs-error curve for one axis (azimuth or elevation)."""

    creep_err: float
    slow_err: float
    mid_err: float
    fast_err: float
    creep: float
    slow: float
    mid: float
    fast: float
    max_speed: float


# Hand-tuned on real hardware — values must match the original
# AZ_SPEED_PROFILE / EL_SPEED_PROFILE dicts in flir_ptz_node.py exactly.
AZ_PROFILE = AxisProfile(0.9, 3.5, 9.0, 20.0, 0.9, 3.2, 9.0, 18.0, 40.0)
EL_PROFILE = AxisProfile(0.9, 2.5, 6.0, 12.0, 0.8, 2.2, 6.0, 12.0, 40.0)


def normalize_az(az: float) -> float:
    """Wrap an azimuth angle into ``(-180, 180]``.

    Reimplemented with ``math.fmod`` instead of the original ``while`` loops,
    which would spin for an unbounded number of iterations on large inputs
    (e.g. ``1e9``).

    Non-finite input (``NaN`` or ``+/-inf``) has no well-defined wrapped
    value. Documented behaviour: any non-finite input returns ``NaN`` rather
    than raising (``math.fmod`` itself raises ``ValueError`` for infinite
    input, which we deliberately avoid propagating from a "pure" utility).
    """
    if not math.isfinite(az):
        return math.nan

    # Shift into (0, 360], then shift back into (-180, 180].
    r = math.fmod(az + 180.0, 360.0)
    if r <= 0.0:
        r += 360.0
    return r - 180.0


def az_error(target: float, current: float) -> float:
    """Shortest signed angular error from ``current`` to ``target``."""
    return normalize_az(target - current)


def sign(v: float) -> int:
    return 1 if v > 0 else (-1 if v < 0 else 0)


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _lerp(t: float, a: float, b: float) -> float:
    return a + t * (b - a)


def choose_speed(err: float, profile: AxisProfile, tol: float) -> float:
    """Piecewise-interpolated speed command from an angular error.

    Bit-for-bit port of the original ``_choose_speed`` in
    ``flir_ptz_node.py``. Do not alter the piecewise band boundaries or
    interpolation order — this curve was validated on real hardware.
    """
    e = abs(err)
    s = sign(err)
    p = profile

    if e <= tol:
        return 0.0
    if e <= p.creep_err:
        return s * p.creep
    if e <= p.fast_err:
        if e <= p.slow_err:
            t = clamp((e - p.creep_err) / ((p.slow_err - p.creep_err) or 1), 0, 1)
            v = _lerp(t, p.creep, p.slow)
        elif e <= p.mid_err:
            t = clamp((e - p.slow_err) / ((p.mid_err - p.slow_err) or 1), 0, 1)
            v = _lerp(t, p.slow, p.mid)
        else:
            t = clamp((e - p.mid_err) / ((p.fast_err - p.mid_err) or 1), 0, 1)
            v = _lerp(t, p.mid, p.fast)
        return s * v
    return s * p.max_speed
