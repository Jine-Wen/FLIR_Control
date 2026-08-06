#!/usr/bin/env python3
"""Tests for the pure zoom-magnification helper in
``flir_ptz.control.zoom_optics``: ``magnification(wide_fov, current_fov)``
and the measured ``EO_WIDE_FOV_DEG`` / ``IR_WIDE_FOV_DEG`` constants.

Pure stdlib; no ROS, no camera, no I/O of any kind.
"""

from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import pytest  # noqa: E402

from flir_ptz.control.zoom_optics import (  # noqa: E402
    EO_WIDE_FOV_DEG,
    IR_WIDE_FOV_DEG,
    magnification,
)


# ---------------------------------------------------------------------------
# Measured constants (worker brief, real FLIR 364C).
# ---------------------------------------------------------------------------


def test_measured_wide_fov_constants():
    assert EO_WIDE_FOV_DEG == pytest.approx(63.7)
    assert IR_WIDE_FOV_DEG == pytest.approx(18.0)


# ---------------------------------------------------------------------------
# Normal range: EO's measured wide/tele extremes and the ~30x range they imply.
# ---------------------------------------------------------------------------


def test_eo_widest_reading_is_1x():
    assert magnification(EO_WIDE_FOV_DEG, EO_WIDE_FOV_DEG) == pytest.approx(1.0)


def test_eo_tele_reading_is_about_30x():
    # Measured: tele Zoom = 2.12 deg at Zoom_pctg 100 -> ~30.0x range.
    mag = magnification(EO_WIDE_FOV_DEG, 2.12)
    assert mag == pytest.approx(63.7 / 2.12)
    assert mag == pytest.approx(30.05, abs=0.05)


def test_eo_midrange_reading():
    # 41.75 deg is a value actually observed in the field (see
    # test_zoom_session.py's DLTVLastNMEAGet fixtures).
    mag = magnification(EO_WIDE_FOV_DEG, 41.75)
    assert mag == pytest.approx(63.7 / 41.75)
    assert mag > 1.0


# ---------------------------------------------------------------------------
# IR's measured wide/tele extremes and the ~2.1x range they imply.
# ---------------------------------------------------------------------------


def test_ir_widest_reading_is_1x():
    assert magnification(IR_WIDE_FOV_DEG, IR_WIDE_FOV_DEG) == pytest.approx(1.0)


def test_ir_tele_reading_is_about_2x():
    # Measured: tele FOV = 8.62 deg at Zoom_Pctg 53.12, Electronic_zoom 2
    # -> ~2.09x range.
    mag = magnification(IR_WIDE_FOV_DEG, 8.62)
    assert mag == pytest.approx(18.0 / 8.62)
    assert mag == pytest.approx(2.09, abs=0.02)


# ---------------------------------------------------------------------------
# Edge cases -- the whole point of a dedicated pure helper: a divide-by-zero
# or a nonsensical "0.4x" reaching the UI is worse than showing nothing, so
# every one of these must floor at 1.0, never raise, never invert.
# ---------------------------------------------------------------------------


def test_zero_current_fov_floors_at_one():
    assert magnification(EO_WIDE_FOV_DEG, 0.0) == 1.0


def test_missing_current_fov_floors_at_one():
    assert magnification(EO_WIDE_FOV_DEG, None) == 1.0


def test_negative_current_fov_floors_at_one():
    assert magnification(EO_WIDE_FOV_DEG, -5.0) == 1.0


def test_current_fov_larger_than_wide_floors_at_one():
    """A bad reading (or a wide_fov that doesn't match this lens) must never
    produce a magnification below 1.0 -- "0.4x" is meaningless for a lens
    that can only ever zoom IN from its widest, never past it."""
    assert magnification(EO_WIDE_FOV_DEG, EO_WIDE_FOV_DEG + 10.0) == 1.0


def test_current_fov_equal_to_wide_is_exactly_one_not_floored_generically():
    """The boundary case: current_fov == wide_fov is a legitimate "fully
    zoomed out" reading, not an edge case to be floored away -- it must
    compute to exactly 1.0 via the real division, not just happen to equal
    the floor value."""
    assert magnification(50.0, 50.0) == 1.0


def test_zero_wide_fov_never_raises():
    """A misconfigured `eo_wide_fov_deg`/`ir_wide_fov_deg` ROS parameter
    (e.g. left at 0) must not crash the tick loop with a ZeroDivisionError."""
    assert magnification(0.0, 10.0) == 1.0


def test_negative_wide_fov_never_raises():
    assert magnification(-1.0, 10.0) == 1.0


def test_never_raises_for_any_combination_of_bad_inputs():
    bad_values = [0.0, -1.0, None, 1e9, float("nan")]
    for wide in [EO_WIDE_FOV_DEG, IR_WIDE_FOV_DEG, 0.0, -1.0]:
        for current in bad_values:
            magnification(wide, current)  # must not raise
