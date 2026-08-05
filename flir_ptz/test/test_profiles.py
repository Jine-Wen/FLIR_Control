#!/usr/bin/env python3
"""Tests for flir_ptz.control.profiles.

Must run offline with nothing but the Python 3.12 standard library —
no ROS, no httpx.
"""

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)  # .../flir_ptz  (contains the flir_ptz package)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import pytest

from flir_ptz.control.profiles import (
    AZ_PROFILE,
    EL_PROFILE,
    AxisProfile,
    az_error,
    choose_speed,
    clamp,
    normalize_az,
    sign,
)


# ---------------------------------------------------------------------------
# Reference implementation — verbatim port of the ORIGINAL
# flir_ptz/flir_ptz_node.py:_choose_speed (dict-based profile), kept here so
# choose_speed() can be pinned against it byte-for-byte. Do not "fix" this
# reference to match new behaviour — if they disagree, the new code is wrong.
# ---------------------------------------------------------------------------

_OLD_AZ_SPEED_PROFILE = {
    "creepErr": 0.9, "slowErr": 3.5, "midErr": 9.0, "fastErr": 20.0,
    "creep": 0.9, "slow": 3.2, "mid": 9.0, "fast": 18.0, "max": 40.0,
}
_OLD_EL_SPEED_PROFILE = {
    "creepErr": 0.9, "slowErr": 2.5, "midErr": 6.0, "fastErr": 12.0,
    "creep": 0.8, "slow": 2.2, "mid": 6.0, "fast": 12.0, "max": 40.0,
}


def _old_sign(v: float) -> int:
    return 1 if v > 0 else (-1 if v < 0 else 0)


def _old_clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _old_lerp(t: float, a: float, b: float) -> float:
    return a + t * (b - a)


def _old_choose_speed(err: float, profile: dict, tol: float) -> float:
    e, s, p = abs(err), _old_sign(err), profile
    if e <= tol:
        return 0.0
    if e <= p["creepErr"]:
        return s * p["creep"]
    if e <= p["fastErr"]:
        if e <= p["slowErr"]:
            t = _old_clamp((e - p["creepErr"]) / ((p["slowErr"] - p["creepErr"]) or 1), 0, 1)
            v = _old_lerp(t, p["creep"], p["slow"])
        elif e <= p["midErr"]:
            t = _old_clamp((e - p["slowErr"]) / ((p["midErr"] - p["slowErr"]) or 1), 0, 1)
            v = _old_lerp(t, p["slow"], p["mid"])
        else:
            t = _old_clamp((e - p["midErr"]) / ((p["fastErr"] - p["midErr"]) or 1), 0, 1)
            v = _old_lerp(t, p["mid"], p["fast"])
        return s * v
    return s * p["max"]


# ---------------------------------------------------------------------------
# choose_speed: bit-for-bit equivalence with the old dict-based curve
# ---------------------------------------------------------------------------

AXIS_PAIRS = [
    (AZ_PROFILE, _OLD_AZ_SPEED_PROFILE, 0.5),
    (EL_PROFILE, _OLD_EL_SPEED_PROFILE, 0.5),
]


@pytest.mark.parametrize("new_profile,old_profile,tol", AXIS_PAIRS, ids=["az", "el"])
def test_choose_speed_matches_old_across_sweep(new_profile, old_profile, tol):
    # Dense sweep across and beyond every band, both signs.
    errs = [x / 10.0 for x in range(-300, 301)]  # -30.0 .. 30.0 step 0.1
    for err in errs:
        expected = _old_choose_speed(err, old_profile, tol)
        actual = choose_speed(err, new_profile, tol)
        assert actual == expected, f"err={err}: expected {expected!r}, got {actual!r}"


@pytest.mark.parametrize("new_profile,old_profile,tol", AXIS_PAIRS, ids=["az", "el"])
def test_choose_speed_matches_old_at_exact_band_boundaries(new_profile, old_profile, tol):
    boundaries = [
        old_profile["creepErr"], old_profile["slowErr"],
        old_profile["midErr"], old_profile["fastErr"],
    ]
    for b in boundaries:
        for err in (b, -b, b + 1e-9, b - 1e-9, -b + 1e-9, -b - 1e-9):
            expected = _old_choose_speed(err, old_profile, tol)
            actual = choose_speed(err, new_profile, tol)
            assert actual == expected, f"err={err}: expected {expected!r}, got {actual!r}"


def test_choose_speed_zero_within_tolerance():
    assert choose_speed(0.0, AZ_PROFILE, 0.5) == 0.0
    assert choose_speed(0.5, AZ_PROFILE, 0.5) == 0.0
    assert choose_speed(-0.5, AZ_PROFILE, 0.5) == 0.0


def test_choose_speed_creep_band():
    # Just above tol, at/under creep_err -> exactly creep speed, signed.
    assert choose_speed(0.9, AZ_PROFILE, 0.5) == pytest.approx(0.9)
    assert choose_speed(-0.9, AZ_PROFILE, 0.5) == pytest.approx(-0.9)


def test_choose_speed_saturates_at_max_beyond_fast_err():
    assert choose_speed(1000.0, AZ_PROFILE, 0.5) == AZ_PROFILE.max_speed
    assert choose_speed(-1000.0, AZ_PROFILE, 0.5) == -AZ_PROFILE.max_speed
    assert choose_speed(20.0001, AZ_PROFILE, 0.5) == AZ_PROFILE.max_speed


def test_choose_speed_degenerate_zero_width_band_no_div_by_zero():
    # Profile with a zero-width band must not raise ZeroDivisionError —
    # matches the `or 1` guard in the original implementation.
    degenerate = AxisProfile(
        creep_err=1.0, slow_err=1.0, mid_err=5.0, fast_err=10.0,
        creep=1.0, slow=1.0, mid=5.0, fast=10.0, max_speed=20.0,
    )
    # Should not raise; err=1.0 is exactly at the creep_err/slow_err boundary.
    result = choose_speed(1.0, degenerate, 0.1)
    assert math.isfinite(result)


# ---------------------------------------------------------------------------
# normalize_az — property tests
# ---------------------------------------------------------------------------

def test_normalize_az_result_always_in_range():
    samples = [
        0.0, 1.0, -1.0, 90.0, -90.0, 179.9, -179.9,
        180.0, -180.0, 180.0001, -180.0001,
        360.0, -360.0, 540.0, -540.0, 720.0, -720.0,
        1e9, -1e9, 1e12, -1e12, 12345.6789, -98765.4321,
    ]
    for x in samples:
        r = normalize_az(x)
        assert -180.0 < r <= 180.0, f"normalize_az({x}) = {r} out of (-180, 180]"


def test_normalize_az_exact_180_boundary():
    assert normalize_az(180.0) == 180.0
    # -180 wraps to +180 (matches the old inclusive `<=` while-loop condition)
    assert normalize_az(-180.0) == 180.0


def test_normalize_az_identity_within_range():
    for x in (0.0, 45.0, -45.0, 179.9, -179.9, 1.0, -1.0):
        assert normalize_az(x) == pytest.approx(x)


def test_normalize_az_idempotent():
    samples = [0.0, 45.0, -45.0, 180.0, -180.0, 360.0, 720.5, -720.5, 1e9, -1e9, 1e15, -1e15]
    for x in samples:
        once = normalize_az(x)
        twice = normalize_az(once)
        assert once == twice, f"normalize_az not idempotent for {x}: {once} != {twice}"


def test_normalize_az_matches_old_while_loop_semantics_small_inputs():
    def old_normalize(az):
        while az <= -180.0:
            az += 360.0
        while az > 180.0:
            az -= 360.0
        return az

    # Keep inputs small — the old implementation is what we're replacing
    # *because* it hangs on huge inputs, so we only compare where it's safe.
    for x in [x / 3.0 for x in range(-3000, 3001)]:  # -1000.0 .. 1000.0
        assert normalize_az(x) == pytest.approx(old_normalize(x))


def test_normalize_az_large_inputs_do_not_hang():
    # 1e9 would take ~2.7 million iterations of the old while loop.
    # This must return quickly.
    assert -180.0 < normalize_az(1e9) <= 180.0
    assert -180.0 < normalize_az(-1e9) <= 180.0


def test_normalize_az_non_finite_returns_nan():
    assert math.isnan(normalize_az(float("nan")))
    assert math.isnan(normalize_az(float("inf")))
    assert math.isnan(normalize_az(float("-inf")))


# ---------------------------------------------------------------------------
# az_error / sign / clamp
# ---------------------------------------------------------------------------

def test_az_error_basic():
    assert az_error(10.0, 5.0) == pytest.approx(5.0)
    assert az_error(-170.0, 170.0) == pytest.approx(20.0)  # wraps the short way
    assert az_error(170.0, -170.0) == pytest.approx(-20.0)


def test_sign():
    assert sign(5.0) == 1
    assert sign(-5.0) == -1
    assert sign(0.0) == 0


def test_clamp():
    assert clamp(5.0, 0.0, 10.0) == 5.0
    assert clamp(-5.0, 0.0, 10.0) == 0.0
    assert clamp(15.0, 0.0, 10.0) == 10.0
