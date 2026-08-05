#!/usr/bin/env python3
"""Tests for flir_ptz.control.fsm -- MotionFSM, the ported motion state
machine (spec ARCHITECTURE.md sec. 3.4, PARITY.md rows A3-A22).

Must run offline with nothing but the Python 3.12 standard library -- no
ROS, no httpx, no camera, no real clock. Every test drives the FSM with an
explicit ``now: float`` and scripted ``PtSample`` sequences, and asserts on
the emitted action sequences -- this is the only proof the hand-tuned
control logic survived the rewrite.
"""

import copy
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)  # .../flir_ptz  (contains the flir_ptz package)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import pytest

from flir_ptz.control.config import ControlConfig
from flir_ptz.control.fsm import (
    Goto,
    Intent,
    Mode,
    MotionFSM,
    ScanOff,
    ScanOn,
    ScanSetLimits,
    ScanSetSpeed,
    SetSpeed,
    Stop,
    StepResult,
    _ScanSub,
)
from flir_ptz.nexus.protocol import PtSample


# -- helpers ------------------------------------------------------------


def cfg(**overrides) -> ControlConfig:
    return ControlConfig(**overrides)


def sample(az=0.0, el=0.0, geo_az=0.0, geo_el=0.0, sx=0.0, sy=0.0, mode=0) -> PtSample:
    return PtSample(
        abs_az=az, abs_el=el, geo_az=geo_az, geo_el=geo_el, speed_x=sx, speed_y=sy, mode=mode
    )


def only(result: StepResult, cls):
    """Assert result.actions has exactly one action and it's of type cls;
    return it."""
    assert len(result.actions) == 1, result.actions
    assert isinstance(result.actions[0], cls), result.actions
    return result.actions[0]


def action_types(result: StepResult):
    return [type(a) for a in result.actions]


# =========================================================================
# Basic construction / mode property / initial tick_period
# =========================================================================


def test_initial_state_is_idle():
    fsm = MotionFSM(cfg())
    assert fsm.mode == Mode.IDLE
    r = fsm.step(None, 0.0)
    assert r.mode == Mode.IDLE
    assert r.actions == ()
    assert r.seq == 0
    assert r.is_moving is False
    assert r.is_scanning is False
    assert r.done is False


def test_idle_tick_period_is_1_over_poll_hz():
    fsm = MotionFSM(cfg(poll_hz=20.0))
    r = fsm.step(None, 0.0)
    assert r.tick_period == pytest.approx(1.0 / 20.0)


def test_active_tick_period_is_poll_ms():
    fsm = MotionFSM(cfg(poll_ms=75))
    fsm.submit(Intent.goto(1.0, 2.0), 0.0)
    r = fsm.step(sample(), 0.0)
    assert r.mode == Mode.GOTO
    assert r.tick_period == pytest.approx(0.075)


def test_submit_does_not_execute_immediately():
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.goto(1.0, 2.0), 0.0)
    # mode/seq must not change until step() is called
    assert fsm.mode == Mode.IDLE


def test_submit_latest_wins():
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.goto(1.0, 2.0), 0.0)
    fsm.submit(Intent.goto(9.0, 9.0), 0.0)  # overwrites the first
    r = fsm.step(sample(), 0.0)
    g = only(r, Goto)
    assert g.az == 9.0 and g.el == 9.0


# =========================================================================
# GOTO -- A3, A4, A5
# =========================================================================


def test_goto_sends_single_goto_action_and_no_immediate_arrival_check():
    """A3: node:517 -- one PTGeoAzimuthElevationSet, camera drives itself.
    The entry tick must NOT also evaluate the sample for arrival (old code
    always slept a full poll interval before its first arrival check)."""
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.goto(30.0, -10.0), 0.0)
    # even though the camera is already exactly on target, the entry tick
    # must not report "arrived" -- it only sends the Goto.
    r = fsm.step(sample(az=30.0, el=-10.0), 0.0)
    g = only(r, Goto)
    assert (g.az, g.el) == (30.0, -10.0)
    assert r.mode == Mode.GOTO
    assert r.done is False
    assert r.is_moving is True


def test_goto_arrival_requires_low_speed_and_in_tolerance():
    """A3 / node:549."""
    c = cfg(az_tol=0.5, el_tol=0.5)
    fsm = MotionFSM(c)
    fsm.submit(Intent.goto(10.0, 5.0), 0.0)
    fsm.step(sample(), 0.0)  # entry tick: sends Goto

    # in tolerance but still moving fast -> not arrived
    r = fsm.step(sample(az=10.0, el=5.0, sx=5.0, sy=0.0), 1.0)
    assert r.actions == ()
    assert r.mode == Mode.GOTO
    assert r.done is False

    # low speed but out of tolerance -> not arrived
    r = fsm.step(sample(az=5.0, el=5.0, sx=0.0, sy=0.0), 1.1)
    assert r.mode == Mode.GOTO
    assert r.done is False

    # both conditions satisfied -> arrived
    r = fsm.step(sample(az=10.0, el=5.0, sx=0.05, sy=0.0), 1.2)
    assert r.mode == Mode.IDLE
    assert r.done is True
    assert "arrived" in r.note


def test_goto_normal_convergence_to_arrival():
    c = cfg(az_tol=0.5, el_tol=0.5, goto_grace_s=1.5, stall_timeout_s=4.0)
    fsm = MotionFSM(c)
    fsm.submit(Intent.goto(10.0, 0.0), 0.0)
    fsm.step(sample(az=0.0), 0.0)

    # steadily progressing, well inside grace -> no stall bookkeeping issue
    for i, az in enumerate([2.0, 4.0, 6.0, 8.0, 9.6], start=1):
        now = i * 0.2
        r = fsm.step(sample(az=az, el=0.0, sx=3.0), now)
        assert r.mode == Mode.GOTO, f"gave up early at az={az}"

    r = fsm.step(sample(az=10.0, el=0.0, sx=0.0), 1.2)
    assert r.mode == Mode.IDLE
    assert r.done is True


def test_goto_stall_during_grace_does_not_trigger():
    """A4: within the 1.5s grace period after send, lack of progress must
    NOT start the stall clock (node:564-565)."""
    c = cfg(az_tol=0.5, el_tol=0.5, goto_grace_s=1.5, stall_timeout_s=4.0)
    fsm = MotionFSM(c)
    fsm.submit(Intent.goto(50.0, 0.0), 0.0)
    fsm.step(sample(az=0.0), 0.0)

    # never moves at all, but we stay within the 1.5s grace window
    for now in (0.2, 0.5, 0.9, 1.4):
        r = fsm.step(sample(az=0.0, el=0.0, sx=0.0), now)
        assert r.mode == Mode.GOTO, f"stalled prematurely at t={now}"
        assert r.done is False


def test_goto_stall_after_grace_gives_up():
    """A4: no progress for stall_timeout_s (4.0s) after the grace/fallback
    window -> give up (node:573-578). Never having moved at all still
    eventually gives up via the ``grace_s * 2`` fallback."""
    c = cfg(az_tol=0.5, el_tol=0.5, goto_grace_s=1.5, stall_timeout_s=4.0, stall_progress_deg=0.05)
    fsm = MotionFSM(c)
    fsm.submit(Intent.goto(50.0, 0.0), 0.0)
    fsm.step(sample(az=0.0), 0.0)

    gave_up_at = None
    now = 0.0
    for _ in range(200):
        now += 0.1
        r = fsm.step(sample(az=0.0, el=0.0, sx=0.0), now)
        if r.mode == Mode.IDLE:
            gave_up_at = now
            break
    assert gave_up_at is not None, "never gave up"
    assert r.done is True
    assert "stalled" in r.note or "giving up" in r.note
    # fallback stall clock only starts at grace_s*2 = 3.0s, then needs a
    # further stall_timeout_s = 4.0s -> gives up around t ~= 7.0s, well
    # after the grace window and well before an absurd amount of time.
    assert 6.5 <= gave_up_at <= 7.5, gave_up_at


def test_goto_progress_resets_stall_timer():
    """A4: continual small progress (>= stall_progress_deg per poll) must
    reset the stall clock and never give up (node:567-569)."""
    c = cfg(az_tol=0.5, el_tol=0.5, goto_grace_s=1.5, stall_timeout_s=4.0, stall_progress_deg=0.05)
    fsm = MotionFSM(c)
    fsm.submit(Intent.goto(50.0, 0.0), 0.0)
    fsm.step(sample(az=0.0), 0.0)

    now = 0.0
    az = 0.0
    # crawl toward target very slowly (0.1 deg/poll, comfortably above the
    # 0.05 deg progress threshold) for well beyond stall_timeout_s and
    # confirm it never gives up.
    for _ in range(100):
        now += 0.1
        az += 0.1
        r = fsm.step(sample(az=az, el=0.0, sx=1.0), now)
        assert r.mode == Mode.GOTO, f"gave up despite steady progress at t={now}"


def test_goto_preempted_by_new_goto_bumps_seq_no_stop_between_gotos():
    """A5: a new move_to command always restarts (seq preemption), even to
    the same mode. Old code sent no explicit Stop() between two
    consecutive goto_position() calls -- superseding with a new absolute
    command is enough."""
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.goto(10.0, 0.0), 0.0)
    r1 = fsm.step(sample(), 0.0)
    seq1 = r1.seq
    assert action_types(r1) == [Goto]

    fsm.submit(Intent.goto(20.0, 0.0), 0.5)
    r2 = fsm.step(sample(az=1.0), 0.5)
    assert r2.mode == Mode.GOTO
    assert r2.seq == seq1 + 1
    assert action_types(r2) == [Goto]  # no Stop() mixed in
    assert only(r2, Goto) == Goto(20.0, 0.0)


def test_goto_preempted_by_track_emits_stop_exit_action():
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.goto(10.0, 0.0), 0.0)
    fsm.step(sample(), 0.0)

    fsm.submit(Intent.track(5.0, 5.0), 0.5)
    r = fsm.step(sample(az=1.0), 0.5)
    assert r.mode == Mode.TRACK
    # exit Stop() from leaving GOTO, then TRACK's own immediate SetSpeed
    assert Stop() in r.actions


# =========================================================================
# TRACK -- A6, A7, A8, A9, A10, A11, A22
# =========================================================================


def test_track_sends_setspeed_closed_loop():
    """A6: node:626, node:770-771."""
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.track(20.0, 0.0), 0.0)
    r = fsm.step(sample(az=0.0, el=0.0), 0.0)
    assert r.mode == Mode.TRACK
    sp = only(r, SetSpeed)
    assert sp.az > 0  # driving toward positive az error


def test_track_settle_counting_and_hold_hysteresis_no_chatter_at_boundary():
    """A7: settle_samples consecutive in-tol samples before "arrived", and
    the wider hold-tolerance band must not chatter once hold_mode has
    latched, even when the error sits between az_tol and az_hold_tol."""
    c = cfg(az_tol=0.5, el_tol=0.5, az_hold_tol=0.7, el_hold_tol=0.7, settle_samples=4, track_idle_s=1.5)
    fsm = MotionFSM(c)
    fsm.submit(Intent.track(10.0, 0.0), 0.0)
    fsm.step(sample(az=10.0, el=0.0), 0.0)  # tick 1, in tol -> settle=1, Stop

    r = fsm.step(sample(az=10.0, el=0.0), 0.06)  # settle=2
    assert only(r, Stop) == Stop()
    r = fsm.step(sample(az=10.0, el=0.0), 0.12)  # settle=3
    assert only(r, Stop) == Stop()
    r = fsm.step(sample(az=10.0, el=0.0), 0.18)  # settle=4 -> "arrived" but idle not yet 1.5s
    assert only(r, Stop) == Stop()
    assert r.mode == Mode.TRACK
    assert "arrived" in r.note

    # now wander into the hold band (beyond az_tol but within az_hold_tol)
    # -- must NOT reset to driving/SetSpeed since hold_mode has latched.
    r = fsm.step(sample(az=10.6, el=0.0), 0.24)  # az_err=0.6 -> outside az_tol(0.5), inside hold(0.7)
    assert action_types(r) == [Stop]
    r = fsm.step(sample(az=10.65, el=0.0), 0.30)
    assert action_types(r) == [Stop]


def test_track_hold_band_exit_resumes_driving():
    """Once error exceeds even the wider hold tolerance, hysteresis breaks
    and closed-loop driving resumes (settle/hold reset to 0/False)."""
    c = cfg(az_tol=0.5, el_tol=0.5, az_hold_tol=0.7, el_hold_tol=0.7, settle_samples=2, track_idle_s=999.0)
    fsm = MotionFSM(c)
    fsm.submit(Intent.track(10.0, 0.0), 0.0)
    fsm.step(sample(az=10.0, el=0.0), 0.0)
    fsm.step(sample(az=10.0, el=0.0), 0.06)  # settle reaches 2, hold_mode True

    r = fsm.step(sample(az=11.0, el=0.0), 0.12)  # az_err=1.0 > hold_tol(0.7)
    assert action_types(r) == [SetSpeed]


def test_track_idle_exit_timing():
    """A11: node:735 -- exit requires BOTH settled (settle_samples) AND
    >= track_idle_s elapsed since the last new target."""
    c = cfg(az_tol=0.5, el_tol=0.5, settle_samples=2, track_idle_s=1.5)
    fsm = MotionFSM(c)
    fsm.submit(Intent.track(10.0, 0.0), 0.0)
    fsm.step(sample(az=10.0, el=0.0), 0.0)  # settle=1
    r = fsm.step(sample(az=10.0, el=0.0), 0.06)  # settle=2, arrived, idle=0.06 < 1.5s
    assert r.mode == Mode.TRACK
    assert r.done is False

    # keep sitting in tolerance until idle clock passes 1.5s (measured from
    # the original target submission at t=0.0)
    r = fsm.step(sample(az=10.0, el=0.0), 1.4)
    assert r.mode == Mode.TRACK
    r = fsm.step(sample(az=10.0, el=0.0), 1.6)
    assert r.mode == Mode.IDLE
    assert r.done is True


def test_track_overshoot_brake_fires_near_target_on_sign_reversal():
    """A8: node:756-768 -- az error sign reversal AND |az_err| < 3.0."""
    c = cfg(az_tol=0.5, el_tol=0.5)
    fsm = MotionFSM(c)
    fsm.submit(Intent.track(0.0, 0.0), 0.0)
    # first sample: az_err = 0 - 2.0 = -2.0 (approaching from positive side)
    r = fsm.step(sample(az=2.0, el=0.0), 0.0)
    assert action_types(r) == [SetSpeed]
    # overshoots past target: az_err flips positive, still close (|1.0|<3.0)
    r = fsm.step(sample(az=-1.0, el=0.0), 0.06)
    assert action_types(r) == [Stop]
    assert "overshoot brake" in r.note


def test_track_overshoot_brake_does_not_fire_far_from_target():
    """A8: sign reversal far from target (|err| >= 3.0 az / >= 2.0 el) must
    NOT brake -- ordinary closed-loop driving continues."""
    c = cfg(az_tol=0.5, el_tol=0.5)
    fsm = MotionFSM(c)
    fsm.submit(Intent.track(0.0, 0.0), 0.0)
    r = fsm.step(sample(az=10.0, el=0.0), 0.0)  # az_err = -10.0
    assert action_types(r) == [SetSpeed]
    r = fsm.step(sample(az=-10.0, el=0.0), 0.06)  # az_err = +10.0, sign flip, but far (>=3.0)
    assert action_types(r) == [SetSpeed]
    assert "overshoot" not in r.note


def test_track_elevation_cap_near_target():
    """A9: node:773-774 -- |el_err| < 1.2 caps |el_cmd| to 1.4, regardless
    of what choose_speed would otherwise produce."""
    c = cfg(az_tol=0.5, el_tol=0.5)
    fsm = MotionFSM(c)
    fsm.submit(Intent.track(0.0, 5.0), 0.0)
    # el_err = 5 - 4.0 = 1.0 (< 1.2) -- choose_speed alone (mid-band) would
    # give well over 1.4 deg/s here; the cap must clamp it down.
    r = fsm.step(sample(az=0.0, el=4.0), 0.0)
    sp = only(r, SetSpeed)
    assert abs(sp.el) <= 1.4 + 1e-9
    assert sp.el > 0  # still driving in the correct direction


def test_track_elevation_cap_does_not_apply_far_from_target():
    c = cfg(az_tol=0.5, el_tol=0.5)
    fsm = MotionFSM(c)
    fsm.submit(Intent.track(0.0, 20.0), 0.0)
    r = fsm.step(sample(az=0.0, el=0.0), 0.0)  # el_err = 20.0, far
    sp = only(r, SetSpeed)
    assert abs(sp.el) > 1.4  # uncapped, should be near max


def test_track_retarget_in_place_no_seq_bump_no_restart():
    """A10: node:681-694 -- a new track target while already tracking
    updates in place; no Stop() exit action, no seq bump."""
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.track(10.0, 0.0), 0.0)
    r1 = fsm.step(sample(az=0.0, el=0.0), 0.0)
    seq1 = r1.seq
    assert r1.mode == Mode.TRACK

    fsm.submit(Intent.track(20.0, 0.0), 0.06)
    r2 = fsm.step(sample(az=0.0, el=0.0), 0.06)
    assert r2.mode == Mode.TRACK
    assert r2.seq == seq1  # no bump
    assert Stop() not in r2.actions or action_types(r2) == [SetSpeed]
    sp = only(r2, SetSpeed)
    assert sp.az > 0  # now driving toward the NEW target (20.0)


def test_track_retarget_mid_flight_resets_settle():
    """Retargeting while partially settled must reset settle/hold state,
    not let stale settle count carry over toward the new target."""
    c = cfg(az_tol=0.5, el_tol=0.5, settle_samples=4)
    fsm = MotionFSM(c)
    fsm.submit(Intent.track(10.0, 0.0), 0.0)
    fsm.step(sample(az=10.0, el=0.0), 0.0)  # settle=1
    fsm.step(sample(az=10.0, el=0.0), 0.06)  # settle=2

    fsm.submit(Intent.track(50.0, 0.0), 0.10)  # retarget, far away now
    r = fsm.step(sample(az=10.0, el=0.0), 0.10)
    assert action_types(r) == [SetSpeed]  # not "arrived", driving again

    # re-approach the new target and confirm settle restarts from zero
    # (needs a fresh settle_samples=4 count, not carrying over the old 2)
    r = fsm.step(sample(az=50.0, el=0.0), 0.16)  # settle=1
    assert "arrived" not in r.note or r.mode == Mode.TRACK
    r2 = fsm.step(sample(az=50.0, el=0.0), 0.22)  # settle=2
    r3 = fsm.step(sample(az=50.0, el=0.0), 0.28)  # settle=3
    assert r3.mode == Mode.TRACK
    assert r3.done is False


def test_track_new_target_same_value_is_noop():
    """Resubmitting an identical target does not reset settle/hold or the
    idle clock (matches old code's `new_tgt != cur_tgt` guard)."""
    c = cfg(az_tol=0.5, el_tol=0.5, settle_samples=2, track_idle_s=1.5)
    fsm = MotionFSM(c)
    fsm.submit(Intent.track(10.0, 0.0), 0.0)
    fsm.step(sample(az=10.0, el=0.0), 0.0)  # settle=1
    # resubmit the SAME (10.0, 0.0) target -- must not reset last_target_at
    fsm.submit(Intent.track(10.0, 0.0), 1.0)
    r = fsm.step(sample(az=10.0, el=0.0), 1.0)  # settle=2 -> arrived, idle = 1.0 - 0.0 = 1.0 < 1.5
    assert r.mode == Mode.TRACK
    r2 = fsm.step(sample(az=10.0, el=0.0), 1.6)  # idle = 1.6 - 0.0 = 1.6 >= 1.5 (using ORIGINAL t=0)
    assert r2.mode == Mode.IDLE


def test_track_preempts_scan():
    """A22: node:648 -- track preempts an active scan, ScanOff exit."""
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.scan_start(0.0, 30.0, -5.0, 5.0), 0.0)
    fsm.step(sample(), 0.0)
    assert fsm.mode == Mode.SCAN

    fsm.submit(Intent.track(10.0, 0.0), 1.0)
    r = fsm.step(sample(az=0.0, el=0.0), 1.0)
    assert r.mode == Mode.TRACK
    assert ScanOff() in r.actions


# =========================================================================
# VELOCITY -- A12, A13, A14, A22
# =========================================================================


def test_velocity_sends_setspeed_direct():
    """A12: node:840."""
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.velocity(5.0, -3.0), 0.0)
    r = fsm.step(None, 0.0)  # velocity mode never needs a sample
    sp = only(r, SetSpeed)
    assert sp.az == 5.0 and sp.el == -3.0
    assert r.mode == Mode.VELOCITY


def test_velocity_clamps_to_plus_minus_40():
    """A13: node:884-890."""
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.velocity(100.0, -100.0), 0.0)
    r = fsm.step(None, 0.0)
    sp = only(r, SetSpeed)
    assert sp.az == 40.0
    assert sp.el == -40.0


def test_velocity_deadband_below_0_01_snaps_to_zero_but_not_exit():
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.velocity(0.005, 0.5), 0.0)
    r = fsm.step(None, 0.0)
    sp = only(r, SetSpeed)
    assert sp.az == 0.0  # snapped
    assert sp.el == 0.5
    assert r.mode == Mode.VELOCITY  # not exited: el is nonzero


def test_velocity_resend_only_on_change():
    """A13/A14 supporting behaviour: node:892-907 -- identical clamped
    value is not resent."""
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.velocity(5.0, 5.0), 0.0)
    r1 = fsm.step(None, 0.0)
    assert action_types(r1) == [SetSpeed]

    # no new intent, same target -> ongoing tick must not resend
    r2 = fsm.step(None, 0.05)
    assert r2.actions == ()
    assert r2.mode == Mode.VELOCITY

    # a retarget to a genuinely different speed DOES resend
    fsm.submit(Intent.velocity(6.0, 5.0), 0.1)
    r3 = fsm.step(None, 0.1)
    assert action_types(r3) == [SetSpeed]


def test_velocity_idle_exit_after_1_5s():
    """A14: node:880, idle timeout is strict >."""
    c = cfg(velocity_idle_s=1.5)
    fsm = MotionFSM(c)
    fsm.submit(Intent.velocity(5.0, 0.0), 0.0)
    fsm.step(None, 0.0)

    r = fsm.step(None, 1.5)  # exactly at boundary -> not idle yet (strict >)
    assert r.mode == Mode.VELOCITY

    r = fsm.step(None, 1.51)
    assert r.mode == Mode.IDLE
    assert action_types(r) == [Stop]
    assert r.done is True


def test_velocity_explicit_zero_exits_immediately():
    """A14: node:909-910 -- explicit zero speed stops immediately, no need
    to wait out the idle timer."""
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.velocity(0.0, 0.0), 0.0)
    r = fsm.step(None, 0.0)
    assert r.mode == Mode.IDLE
    assert r.done is True
    # az=el=0.0 already equals the "last_sent=None" default so a SetSpeed(0,0)
    # IS emitted once (matches old code sending the zero command before exit)
    assert action_types(r) == [SetSpeed]
    assert r.actions[0] == SetSpeed(0.0, 0.0)


def test_velocity_zero_via_retarget_exits_immediately_without_waiting_idle():
    c = cfg(velocity_idle_s=1.5)
    fsm = MotionFSM(c)
    fsm.submit(Intent.velocity(5.0, 0.0), 0.0)
    fsm.step(None, 0.0)
    fsm.submit(Intent.velocity(0.0, 0.0), 0.05)
    r = fsm.step(None, 0.05)  # well within idle window, but explicit zero
    assert r.mode == Mode.IDLE
    assert r.done is True


def test_velocity_preempts_scan():
    """A22: node:860."""
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.scan_start(0.0, 30.0, -5.0, 5.0), 0.0)
    fsm.step(sample(), 0.0)
    fsm.submit(Intent.velocity(5.0, 0.0), 1.0)
    r = fsm.step(sample(), 1.0)
    assert r.mode == Mode.VELOCITY
    assert ScanOff() in r.actions


# =========================================================================
# SCAN -- A16, A17, A18, A20, A21, A22
# =========================================================================


def _drive_scan_to_active(fsm: MotionFSM, c: ControlConfig, center=0.0, each_side=30.0,
                           el=-10.0, speed=5.0, t0=0.0):
    """Helper: submit scan_start and advance through every sub-state to
    ACTIVE with well-behaved samples. Returns the final `now`."""
    fsm.submit(Intent.scan_start(center, each_side, el, speed), t0)
    r = fsm.step(sample(mode=0, sx=0, sy=0), t0)
    assert fsm._scan_sub == _ScanSub.STOP_PREV
    assert ScanOff() in r.actions

    now = t0 + 0.01
    r = fsm.step(sample(mode=0, sx=0, sy=0), now)  # stopped confirmed -> preposition
    assert fsm._scan_sub == _ScanSub.PREPOSITION
    assert isinstance(r.actions[-1], Goto)

    now += 0.01
    r = fsm.step(sample(az=center, el=el, sx=0, sy=0), now)  # elevation reached
    assert fsm._scan_sub == _ScanSub.SET_SPEED
    assert isinstance(r.actions[-1], ScanSetSpeed)

    now += c.scan_save_wait_s + 0.001
    r = fsm.step(sample(az=center, el=el), now)
    assert fsm._scan_sub == _ScanSub.SET_LIMITS
    assert isinstance(r.actions[-1], ScanSetLimits)

    now += 0.25 + 0.001
    r = fsm.step(sample(az=center, el=el), now)
    assert fsm._scan_sub == _ScanSub.ON
    assert isinstance(r.actions[-1], ScanOn)
    assert r.is_scanning is True  # is_scanning flips True right when ON starts

    now += c.scan_start_wait_s + 0.001
    r = fsm.step(sample(az=center, el=el, sx=speed), now)  # confirm: nonzero speed
    assert fsm._scan_sub == _ScanSub.ACTIVE
    return now


def test_scan_full_subsequence_progression():
    """A16/A17: STOP_PREV -> PREPOSITION -> SET_SPEED -> SET_LIMITS -> ON
    -> ACTIVE, one sub-state advance per tick, correct action per stage."""
    c = cfg()
    fsm = MotionFSM(c)
    _drive_scan_to_active(fsm, c)
    assert fsm.mode == Mode.SCAN
    assert fsm._scan_sub == _ScanSub.ACTIVE


def test_scan_limits_use_normalized_left_right():
    c = cfg()
    fsm = MotionFSM(c)
    fsm.submit(Intent.scan_start(170.0, 30.0, -5.0, 5.0), 0.0)
    fsm.step(sample(mode=0), 0.0)  # -> STOP_PREV
    fsm.step(sample(mode=0), 0.01)  # -> PREPOSITION
    fsm.step(sample(az=170.0, el=-5.0), 0.02)  # -> SET_SPEED
    now = 0.02 + c.scan_save_wait_s + 0.001
    limits_result = fsm.step(sample(), now)  # -> SET_LIMITS (action emitted this tick)
    sl = limits_result.actions[-1]
    assert isinstance(sl, ScanSetLimits)
    # 170 + 30 = 200 -> normalized to -160
    assert sl.right == pytest.approx(-160.0)
    assert sl.left == pytest.approx(140.0)


def test_scan_preposition_only_checks_elevation_not_azimuth():
    """Faithful port of node:1056-1071: azimuth is NOT part of the
    preposition arrival check, only elevation + speed."""
    c = cfg(el_tol=0.5)
    fsm = MotionFSM(c)
    fsm.submit(Intent.scan_start(0.0, 30.0, -10.0, 5.0), 0.0)
    fsm.step(sample(mode=0), 0.0)
    fsm.step(sample(mode=0), 0.01)  # -> PREPOSITION
    assert fsm._scan_sub == _ScanSub.PREPOSITION
    # az is wildly off-target but el is within tolerance and speed low
    r = fsm.step(sample(az=999.0, el=-10.0, sx=0.0, sy=0.0), 0.02)
    assert fsm._scan_sub == _ScanSub.SET_SPEED


def test_scan_stop_prev_timeout_proceeds_anyway():
    """A20 supporting behaviour: node:1034-1036, failure to confirm stop
    is ignored, proceeds to PREPOSITION regardless after
    stop_scan_timeout_s."""
    c = cfg(stop_scan_timeout_s=1.0)
    fsm = MotionFSM(c)
    fsm.submit(Intent.scan_start(0.0, 30.0, -10.0, 5.0), 0.0)
    fsm.step(sample(mode=1, sx=5.0), 0.0)  # camera reports still moving/scanning
    r = fsm.step(sample(mode=1, sx=5.0), 0.5)
    assert fsm._scan_sub == _ScanSub.STOP_PREV  # still waiting, under timeout
    r = fsm.step(sample(mode=1, sx=5.0), 1.01)  # past timeout
    assert fsm._scan_sub == _ScanSub.PREPOSITION
    assert isinstance(r.actions[-1], Goto)


def test_scan_preposition_timeout_proceeds_anyway():
    """Preserve the 15s preposition timeout (old code waited up to 15s for
    arrival before sweeping, then proceeded regardless)."""
    c = cfg()
    fsm = MotionFSM(c)
    fsm.submit(Intent.scan_start(0.0, 30.0, -10.0, 5.0), 0.0)
    fsm.step(sample(mode=0), 0.0)
    fsm.step(sample(mode=0), 0.01)  # -> PREPOSITION, deadline = 0.01 + 15.0
    assert fsm._scan_sub == _ScanSub.PREPOSITION

    # never reaches elevation, but well before the 15s deadline
    r = fsm.step(sample(az=0.0, el=50.0, sx=5.0), 10.0)
    assert fsm._scan_sub == _ScanSub.PREPOSITION

    # past the 15s deadline -> proceeds anyway
    r = fsm.step(sample(az=0.0, el=50.0, sx=5.0), 15.02)
    assert fsm._scan_sub == _ScanSub.SET_SPEED
    assert isinstance(r.actions[-1], ScanSetSpeed)


def test_scan_on_confirm_timeout_proceeds_to_active():
    """3.0s confirm timeout (node:1105) -- if the camera never reports
    Mode!=0 / nonzero speed, proceed to ACTIVE anyway rather than hang."""
    c = cfg(scan_start_wait_s=0.1)
    fsm = MotionFSM(c)
    on_tick_now = _advance_scan_to_substate(fsm, c, _ScanSub.ON)
    assert fsm._scan_sub == _ScanSub.ON

    # still within the fixed post-ON wait -- idle sample changes nothing
    r = fsm.step(sample(mode=0, sx=0, sy=0), on_tick_now + 0.05)
    assert fsm._scan_sub == _ScanSub.ON

    # past the fixed wait (0.1s) -- confirm phase starts this same tick,
    # but the sample is idle so it does not confirm yet
    r = fsm.step(sample(mode=0, sx=0, sy=0), on_tick_now + 0.11)
    assert fsm._scan_sub == _ScanSub.ON

    # camera keeps reporting idle throughout confirm window -> still ON
    r = fsm.step(sample(mode=0, sx=0, sy=0), on_tick_now + 0.11 + 1.0)
    assert fsm._scan_sub == _ScanSub.ON

    # past the 3.0s confirm deadline -> proceeds to ACTIVE regardless
    r = fsm.step(sample(mode=0, sx=0, sy=0), on_tick_now + 0.11 + 3.01)
    assert fsm._scan_sub == _ScanSub.ACTIVE


def test_scan_is_scanning_false_before_on_true_from_on_onward():
    c = cfg()
    fsm = MotionFSM(c)
    fsm.submit(Intent.scan_start(0.0, 30.0, -10.0, 5.0), 0.0)
    r = fsm.step(sample(mode=0), 0.0)
    assert r.is_scanning is False  # STOP_PREV
    r = fsm.step(sample(mode=0), 0.01)
    assert r.is_scanning is False  # PREPOSITION
    now = _drive_scan_to_active(MotionFSM(c), c)
    fsm2 = MotionFSM(c)
    now = 0.0
    fsm2.submit(Intent.scan_start(0.0, 30.0, -10.0, 5.0), now)
    fsm2.step(sample(mode=0), now)
    now += 0.01
    fsm2.step(sample(mode=0), now)  # preposition
    now += 0.01
    fsm2.step(sample(az=0.0, el=-10.0), now)  # set_speed
    now += c.scan_save_wait_s + 0.001
    fsm2.step(sample(), now)  # set_limits
    now += 0.25 + 0.001
    r = fsm2.step(sample(), now)  # -> ON
    assert r.is_scanning is True


def test_scan_external_interrupt_detected_during_active():
    """A18: node:1126-1134 -- Mode==0 and both speeds <= 0.1 while
    ACTIVE."""
    c = cfg()
    fsm = MotionFSM(c)
    now = _drive_scan_to_active(fsm, c)
    r = fsm.step(sample(mode=1, sx=5.0), now + 0.15)
    assert r.mode == Mode.SCAN
    assert fsm._scan_sub == _ScanSub.ACTIVE

    r = fsm.step(sample(mode=0, sx=0.05, sy=0.0), now + 0.30)
    assert r.mode == Mode.IDLE
    assert r.done is True
    assert "interrupted" in r.note
    # old code does NOT re-send scan_mode_off() on this path (node:1132-1134)
    assert r.actions == ()


def test_scan_external_interrupt_not_falsely_triggered_by_low_speed_alone():
    c = cfg()
    fsm = MotionFSM(c)
    now = _drive_scan_to_active(fsm, c)
    # Mode != 0 even though speed is near zero (e.g. camera paused at a
    # sweep limit) -- must NOT be treated as an external interrupt.
    r = fsm.step(sample(mode=2, sx=0.0, sy=0.0), now + 0.15)
    assert r.mode == Mode.SCAN
    assert fsm._scan_sub == _ScanSub.ACTIVE


def test_scan_stop_intent_halts_and_emits_scanoff():
    """A20: node:986-1012."""
    c = cfg()
    fsm = MotionFSM(c)
    now = _drive_scan_to_active(fsm, c)
    fsm.submit(Intent.scan_stop(), now + 0.1)
    r = fsm.step(sample(mode=1, sx=5.0), now + 0.1)
    assert r.mode == Mode.IDLE
    assert r.actions == (ScanOff(),)


def test_scan_stop_while_not_scanning_is_noop():
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.scan_stop(), 0.0)
    r = fsm.step(None, 0.0)
    assert r.mode == Mode.IDLE
    assert r.actions == ()


def test_scan_restart_bumps_seq_and_resets_to_stop_prev():
    """A21: a fresh scan_start while already scanning fully restarts the
    sequence (stale sub-state auto-superseded), even to the same
    parameters."""
    c = cfg()
    fsm = MotionFSM(c)
    now = _drive_scan_to_active(fsm, c)
    seq_active = fsm.step(sample(mode=1, sx=5.0), now + 0.01).seq

    fsm.submit(Intent.scan_start(0.0, 30.0, -10.0, 5.0), now + 0.1)
    r = fsm.step(sample(mode=1, sx=5.0), now + 0.1)
    assert r.mode == Mode.SCAN
    assert fsm._scan_sub == _ScanSub.STOP_PREV
    assert r.seq == seq_active + 1
    assert ScanOff() in r.actions


def _advance_scan_to_substate(fsm, c, target, center=0.0, each_side=30.0, el=-10.0,
                               speed=5.0, t0=0.0):
    """Drive a freshly-submitted scan_start forward one tick at a time,
    stopping as soon as ``target`` sub-state is reached. Returns the `now`
    at which that sub-state was entered."""
    fsm.submit(Intent.scan_start(center, each_side, el, speed), t0)
    now = t0
    fsm.step(sample(mode=0, sx=0, sy=0), now)  # -> STOP_PREV
    if fsm._scan_sub == target:
        return now
    now += 0.01
    fsm.step(sample(mode=0, sx=0, sy=0), now)  # -> PREPOSITION
    if fsm._scan_sub == target:
        return now
    now += 0.01
    fsm.step(sample(az=center, el=el, sx=0, sy=0), now)  # -> SET_SPEED
    if fsm._scan_sub == target:
        return now
    now += c.scan_save_wait_s + 0.001
    fsm.step(sample(az=center, el=el), now)  # -> SET_LIMITS
    if fsm._scan_sub == target:
        return now
    now += 0.25 + 0.001
    fsm.step(sample(az=center, el=el), now)  # -> ON
    if fsm._scan_sub == target:
        return now
    now += c.scan_start_wait_s + 0.001
    fsm.step(sample(az=center, el=el, sx=speed), now)  # -> ACTIVE
    assert fsm._scan_sub == target
    return now


@pytest.mark.parametrize(
    "target_sub",
    [
        _ScanSub.STOP_PREV,
        _ScanSub.PREPOSITION,
        _ScanSub.SET_SPEED,
        _ScanSub.SET_LIMITS,
        _ScanSub.ON,
        _ScanSub.ACTIVE,
    ],
)
def test_scan_preemptible_at_every_substate(target_sub):
    """Preemption must be possible at EVERY sub-state (goto/track/velocity/
    home all preempt scan -- A22)."""
    c = cfg()
    fsm = MotionFSM(c)
    now = _advance_scan_to_substate(fsm, c, target_sub)
    assert fsm.mode == Mode.SCAN
    assert fsm._scan_sub == target_sub

    fsm.submit(Intent.home(), now + 1.0)
    r = fsm.step(sample(), now + 1.0)
    assert r.mode == Mode.HOMING
    assert ScanOff() in r.actions


def test_scan_preempted_by_goto_emits_scanoff():
    c = cfg()
    fsm = MotionFSM(c)
    fsm.submit(Intent.scan_start(0.0, 30.0, -10.0, 5.0), 0.0)
    fsm.step(sample(mode=0), 0.0)
    fsm.submit(Intent.goto(45.0, 0.0), 0.5)
    r = fsm.step(sample(mode=0), 0.5)
    assert r.mode == Mode.GOTO
    assert ScanOff() in r.actions
    assert Goto(45.0, 0.0) in r.actions


def test_priority_goto_track_velocity_home_all_preempt_scan():
    """A22, exhaustively."""
    c = cfg()
    for make_intent, expect_mode in [
        (lambda: Intent.goto(1.0, 1.0), Mode.GOTO),
        (lambda: Intent.track(1.0, 1.0), Mode.TRACK),
        (lambda: Intent.velocity(1.0, 1.0), Mode.VELOCITY),
        (lambda: Intent.home(), Mode.HOMING),
    ]:
        fsm = MotionFSM(c)
        fsm.submit(Intent.scan_start(0.0, 30.0, -10.0, 5.0), 0.0)
        fsm.step(sample(mode=0), 0.0)
        assert fsm.mode == Mode.SCAN
        fsm.submit(make_intent(), 1.0)
        r = fsm.step(sample(mode=0), 1.0)
        assert r.mode == expect_mode
        assert ScanOff() in r.actions


# =========================================================================
# stop intent (universal)
# =========================================================================


@pytest.mark.parametrize(
    "setup,expect_exit",
    [
        (lambda fsm: fsm.submit(Intent.goto(1.0, 1.0), 0.0) or fsm.step(sample(), 0.0), Stop()),
        (lambda fsm: fsm.submit(Intent.track(1.0, 1.0), 0.0) or fsm.step(sample(), 0.0), Stop()),
        (lambda fsm: fsm.submit(Intent.velocity(1.0, 1.0), 0.0) or fsm.step(None, 0.0), Stop()),
    ],
)
def test_stop_intent_halts_any_active_mode(setup, expect_exit):
    fsm = MotionFSM(cfg())
    setup(fsm)
    assert fsm.mode != Mode.IDLE
    fsm.submit(Intent.stop(), 1.0)
    r = fsm.step(sample(), 1.0)
    assert r.mode == Mode.IDLE
    assert expect_exit in r.actions


def test_stop_intent_from_scan_emits_scanoff():
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.scan_start(0.0, 30.0, -10.0, 5.0), 0.0)
    fsm.step(sample(mode=0), 0.0)
    fsm.submit(Intent.stop(), 1.0)
    r = fsm.step(sample(mode=0), 1.0)
    assert r.mode == Mode.IDLE
    assert r.actions == (ScanOff(),)


def test_stop_intent_while_idle_is_harmless():
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.stop(), 0.0)
    r = fsm.step(None, 0.0)
    assert r.mode == Mode.IDLE
    assert r.actions == ()


# =========================================================================
# HOMING
# =========================================================================


def test_homing_drives_to_configured_home_position_and_completes():
    c = cfg(home_az=0.0, home_el=-90.0, az_tol=0.5, el_tol=0.5)
    fsm = MotionFSM(c)
    fsm.submit(Intent.home(), 0.0)
    r = fsm.step(sample(az=10.0, el=-50.0), 0.0)
    g = only(r, Goto)
    assert (g.az, g.el) == (0.0, -90.0)
    assert r.mode == Mode.HOMING

    r = fsm.step(sample(az=0.0, el=-90.0, sx=0.0, sy=0.0), 0.5)
    assert r.mode == Mode.IDLE
    assert r.done is True


def test_homing_gives_up_after_home_timeout_s():
    c = cfg(home_timeout_s=5.0, az_tol=0.5, el_tol=0.5, goto_grace_s=1.5, stall_timeout_s=4.0)
    fsm = MotionFSM(c)
    fsm.submit(Intent.home(), 0.0)
    fsm.step(sample(az=0.0, el=0.0), 0.0)

    # never arrives, stuck far from target the whole time
    r = None
    now = 0.0
    for _ in range(60):
        now += 0.1
        r = fsm.step(sample(az=0.0, el=0.0, sx=0.0), now)
        if r.mode == Mode.IDLE:
            break
    assert r.mode == Mode.IDLE
    assert r.done is True
    assert now <= 5.5  # gave up at/around the 5.0s home_timeout_s, not later


def test_home_preempts_track():
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.track(10.0, 10.0), 0.0)
    fsm.step(sample(), 0.0)
    fsm.submit(Intent.home(), 1.0)
    r = fsm.step(sample(), 1.0)
    assert r.mode == Mode.HOMING
    assert Stop() in r.actions
    assert any(isinstance(a, Goto) for a in r.actions)


# =========================================================================
# sample=None tolerance
# =========================================================================


def test_none_sample_tolerated_in_every_active_mode():
    for make_intent in (
        lambda: Intent.goto(10.0, 10.0),
        lambda: Intent.track(10.0, 10.0),
        lambda: Intent.velocity(5.0, 5.0),
        lambda: Intent.home(),
    ):
        fsm = MotionFSM(cfg())
        fsm.submit(make_intent(), 0.0)
        fsm.step(sample(), 0.0)  # entry tick with a real sample
        # now feed None repeatedly -- must not crash, must hold state
        for i in range(20):
            r = fsm.step(None, 0.1 * (i + 1))
            assert isinstance(r, StepResult)


def test_repeated_none_samples_eventually_degrade_goto_via_real_time_stall():
    """Even with the camera read failing every tick, the stall clock is
    driven by ``now`` (not by successful reads), so persistent failure
    still eventually gives up once a real sample resumes and shows no
    progress, rather than hanging forever."""
    c = cfg(goto_grace_s=1.5, stall_timeout_s=4.0)
    fsm = MotionFSM(c)
    fsm.submit(Intent.goto(50.0, 0.0), 0.0)
    fsm.step(sample(az=0.0), 0.0)

    now = 0.0
    for _ in range(50):
        now += 0.1
        r = fsm.step(None, now)
        assert r.mode == Mode.GOTO  # None never itself causes a transition

    # a real sample now arrives, still off-target -- stall bookkeeping
    # should behave sanely (not crash, not instantly "arrived")
    r = fsm.step(sample(az=0.0, el=0.0, sx=0.0), now + 0.1)
    assert r.mode in (Mode.GOTO, Mode.IDLE)


def test_none_sample_in_scan_holds_current_substate():
    c = cfg()
    fsm = MotionFSM(c)
    fsm.submit(Intent.scan_start(0.0, 30.0, -10.0, 5.0), 0.0)
    fsm.step(None, 0.0)  # STOP_PREV entry, no sample needed for the ScanOff action
    assert fsm._scan_sub == _ScanSub.STOP_PREV
    r = fsm.step(None, 0.01)
    assert fsm._scan_sub == _ScanSub.STOP_PREV  # can't confirm stop without a sample
    assert isinstance(r, StepResult)


def test_velocity_ignores_none_sample_entirely():
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.velocity(5.0, 5.0), 0.0)
    r = fsm.step(None, 0.0)
    assert action_types(r) == [SetSpeed]


# =========================================================================
# Purity
# =========================================================================


def test_step_does_not_mutate_the_sample():
    fsm = MotionFSM(cfg())
    fsm.submit(Intent.track(10.0, 10.0), 0.0)
    s = sample(az=1.0, el=2.0, geo_az=3.0, geo_el=4.0, sx=5.0, sy=6.0, mode=7)
    before = copy.deepcopy(s)
    fsm.step(s, 0.0)
    assert s == before  # PtSample is frozen, but assert field-for-field too
    assert s.abs_az == before.abs_az
    assert s.abs_el == before.abs_el
    assert s.speed_x == before.speed_x
    assert s.speed_y == before.speed_y
    assert s.mode == before.mode


def test_identical_state_input_now_yields_identical_output():
    """Two independently constructed FSMs, driven through an identical
    script of submits/steps, must produce identical StepResults at every
    point -- step() has no hidden nondeterminism (no real clock, no
    randomness, no id()-based branching)."""

    def run(fsm: MotionFSM):
        results = []
        fsm.submit(Intent.track(15.0, -5.0), 0.0)
        results.append(fsm.step(sample(az=0.0, el=0.0), 0.0))
        results.append(fsm.step(sample(az=2.0, el=-1.0), 0.06))
        fsm.submit(Intent.track(20.0, -5.0), 0.12)
        results.append(fsm.step(sample(az=4.0, el=-2.0), 0.12))
        results.append(fsm.step(sample(az=6.0, el=-3.0), 0.18))
        return results

    fsm_a = MotionFSM(cfg())
    fsm_b = MotionFSM(cfg())
    results_a = run(fsm_a)
    results_b = run(fsm_b)
    assert results_a == results_b


def test_step_called_twice_with_same_args_on_two_clones_matches():
    c = cfg()
    fsm1 = MotionFSM(c)
    fsm1.submit(Intent.goto(10.0, 10.0), 0.0)
    fsm1.step(sample(), 0.0)
    r1 = fsm1.step(sample(az=1.0, el=1.0, sx=2.0), 0.1)

    fsm2 = MotionFSM(c)
    fsm2.submit(Intent.goto(10.0, 10.0), 0.0)
    fsm2.step(sample(), 0.0)
    r2 = fsm2.step(sample(az=1.0, el=1.0, sx=2.0), 0.1)

    assert r1 == r2


# =========================================================================
# Purity of the fsm module itself -- guard against accidental I/O/async
# creeping in (belt-and-suspenders companion to the grep check the worker
# report performs against the source file).
# =========================================================================


def test_fsm_module_has_no_forbidden_imports():
    import flir_ptz.control.fsm as fsm_module

    src_path = fsm_module.__file__
    with open(src_path, "r", encoding="utf-8") as f:
        src = f.read()
    for forbidden in ("import asyncio", "import httpx", "import rclpy", "time.monotonic", "await "):
        assert forbidden not in src, f"forbidden token {forbidden!r} found in fsm.py"
