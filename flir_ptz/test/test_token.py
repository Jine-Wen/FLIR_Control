"""Tests for flir_ptz.nexus.token — pure token policy state machine.

Standard library only. No rclpy, no httpx, no network — every test
drives the tracker with an explicit fake clock (plain floats), never
the real clock.
"""

import sys
from pathlib import Path

# The Worker-A-owned __init__.py files may not exist yet; fall back to
# inserting the ament package dir onto sys.path so the nested
# `flir_ptz.nexus.token` namespace package resolves without them.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from flir_ptz.nexus.token import (  # noqa: E402
    TokenAction,
    TokenPolicy,
    TokenState,
    TokenTracker,
    needs_token,
)


# -- needs_token -------------------------------------------------------


def test_needs_token_reads_are_free():
    assert needs_token("PTLastNMEAGet") is False
    assert needs_token("SERVERLastNMEAGet") is False
    assert needs_token("SERVERWhoAmI") is False


def test_needs_token_writes_require_token():
    assert needs_token("PTAbsAzElSet") is True
    assert needs_token("PTSpeedSet") is True
    assert needs_token("PTAutoScanOn") is True
    assert needs_token("PTAutoScanSetSpeed") is True


# -- observe_token_id / required_before ---------------------------------


def test_foreign_token_requires_acquire():
    tr = TokenTracker(TokenPolicy())
    tr.observe_token_id(token_id=999, our_session=42, now=0.0)
    assert tr.state == TokenState.FOREIGN
    assert tr.required_before("PTAbsAzElSet", now=0.0) == TokenAction.ACQUIRE


def test_held_token_requires_nothing():
    tr = TokenTracker(TokenPolicy())
    tr.observe_token_id(token_id=42, our_session=42, now=0.0)
    assert tr.state == TokenState.HELD
    assert tr.required_before("PTAbsAzElSet", now=0.0) == TokenAction.NONE


def test_unknown_state_requires_acquire():
    tr = TokenTracker(TokenPolicy())
    assert tr.state == TokenState.UNKNOWN
    assert tr.required_before("PTSpeedSet", now=0.0) == TokenAction.ACQUIRE


def test_read_action_never_requires_token_regardless_of_state():
    tr = TokenTracker(TokenPolicy())
    assert tr.required_before("PTLastNMEAGet", now=0.0) == TokenAction.NONE


# -- RC handling: 15 -> REVIVE -> retry, 21 -> RELOGIN -> retry ----------


def test_rc15_then_revive_then_retry_scripted_sequence():
    tr = TokenTracker(TokenPolicy())
    tr.observe_token_id(token_id=42, our_session=42, now=0.0)
    assert tr.state == TokenState.HELD

    # Command sent, camera replies RC=15 (token idle but still ours).
    action = tr.observe_rc(15, now=1.0)
    assert action == TokenAction.REVIVE
    assert tr.state == TokenState.IDLE_OURS

    # Before retrying the write, the tracker must again say REVIVE.
    assert tr.required_before("PTAbsAzElSet", now=1.0) == TokenAction.REVIVE

    # A matching Token_ID is NOT proof the token is usable: the camera keeps
    # reporting Token_ID == our session for the whole RC=15 idle state. So
    # observing it must NOT clear IDLE_OURS, or the revive would be skipped
    # and the next write would just get RC=15 again, forever.
    tr.observe_token_id(token_id=42, our_session=42, now=1.1)
    assert tr.state == TokenState.IDLE_OURS
    assert tr.required_before("PTAbsAzElSet", now=1.1) == TokenAction.REVIVE

    # Only a confirmed revive clears it.
    tr.note_token_acquired(now=1.2)
    assert tr.state == TokenState.HELD
    assert tr.required_before("PTAbsAzElSet", now=1.2) == TokenAction.NONE


def test_rc15_deadlock_cannot_recur_without_confirmed_revive():
    """Regression: the old code's bug. Repeatedly observing a matching
    Token_ID must never be enough to make the tracker think it can write."""
    tr = TokenTracker(TokenPolicy())
    tr.observe_token_id(token_id=42, our_session=42, now=0.0)
    tr.observe_rc(15, now=1.0)

    for i in range(50):
        tr.observe_token_id(token_id=42, our_session=42, now=2.0 + i * 0.1)
        assert tr.state == TokenState.IDLE_OURS
        assert tr.required_before("PTSpeedModeSet", now=2.0 + i * 0.1) == TokenAction.REVIVE

    tr.note_token_acquired(now=10.0)
    assert tr.required_before("PTSpeedModeSet", now=10.0) == TokenAction.NONE


def test_foreign_token_overrides_idle_ours():
    """If someone else (e.g. the physical JCU) takes the token while we are
    in the RC=15 idle state, stickiness must not mask the takeover."""
    tr = TokenTracker(TokenPolicy())
    tr.observe_token_id(token_id=42, our_session=42, now=0.0)
    tr.observe_rc(15, now=1.0)
    assert tr.state == TokenState.IDLE_OURS

    tr.observe_token_id(token_id=99, our_session=42, now=2.0)
    assert tr.state == TokenState.FOREIGN
    assert tr.required_before("PTSpeedModeSet", now=2.0) == TokenAction.ACQUIRE


def test_rc21_then_relogin_then_retry_scripted_sequence():
    tr = TokenTracker(TokenPolicy())
    tr.observe_token_id(token_id=42, our_session=42, now=0.0)
    assert tr.state == TokenState.HELD

    action = tr.observe_rc(21, now=2.0)
    assert action == TokenAction.RELOGIN
    assert tr.state == TokenState.UNKNOWN

    # After re-login, control isn't automatically ours: must re-acquire.
    assert tr.required_before("PTAbsAzElSet", now=2.0) == TokenAction.ACQUIRE


def test_rc_other_codes_are_noop():
    tr = TokenTracker(TokenPolicy())
    tr.observe_token_id(token_id=42, our_session=42, now=0.0)
    assert tr.observe_rc(0, now=0.5) == TokenAction.NONE
    assert tr.observe_rc(None, now=0.5) == TokenAction.NONE
    assert tr.state == TokenState.HELD


# -- periodic(): keepalive due / not-due at boundary, idle -> release ---


def test_keepalive_fires_immediately_when_active_and_never_kept_alive():
    policy = TokenPolicy(grace_s=30.0, keepalive_s=10.0)
    tr = TokenTracker(policy)
    tr.observe_token_id(token_id=1, our_session=1, now=0.0)
    assert tr.periodic(now=0.0, mode_active=True) == TokenAction.KEEPALIVE


def test_keepalive_not_due_before_interval_elapsed():
    policy = TokenPolicy(grace_s=30.0, keepalive_s=10.0)
    tr = TokenTracker(policy)
    tr.observe_token_id(token_id=1, our_session=1, now=0.0)
    assert tr.periodic(now=0.0, mode_active=True) == TokenAction.KEEPALIVE
    # Just under the keepalive interval: not due yet.
    assert tr.periodic(now=9.999, mode_active=True) == TokenAction.NONE


def test_keepalive_due_exactly_at_boundary():
    policy = TokenPolicy(grace_s=30.0, keepalive_s=10.0)
    tr = TokenTracker(policy)
    tr.observe_token_id(token_id=1, our_session=1, now=0.0)
    assert tr.periodic(now=0.0, mode_active=True) == TokenAction.KEEPALIVE
    # Exactly keepalive_s later: due again.
    assert tr.periodic(now=10.0, mode_active=True) == TokenAction.KEEPALIVE


def test_keepalive_within_grace_of_last_command_even_when_mode_idle():
    policy = TokenPolicy(grace_s=30.0, keepalive_s=10.0)
    tr = TokenTracker(policy)
    tr.observe_token_id(token_id=1, our_session=1, now=0.0)
    tr.note_command(now=5.0)
    # mode_active=False but within grace window of the last command.
    assert tr.periodic(now=5.0, mode_active=False) == TokenAction.KEEPALIVE
    # 7s later: still within grace (7s < 30s) but keepalive interval
    # (10s) hasn't elapsed yet.
    assert tr.periodic(now=12.0, mode_active=False) == TokenAction.NONE
    # 11s after the last keepalive, still well within the 30s grace
    # window (11s since the command): keepalive fires again.
    assert tr.periodic(now=16.0, mode_active=False) == TokenAction.KEEPALIVE


def test_idle_past_grace_releases_token_this_is_the_bug_fix():
    """The core regression test: no more unconditional Forced=1 every 10s.

    Once idle (mode inactive, no recent command) past grace_s, the
    tracker must emit RELEASE exactly once so a physical JCU operator
    is left alone, instead of force-claiming forever.
    """
    policy = TokenPolicy(grace_s=30.0, keepalive_s=10.0)
    tr = TokenTracker(policy)
    tr.observe_token_id(token_id=1, our_session=1, now=0.0)
    tr.note_command(now=0.0)

    # Still within grace: keepalive, never a forced re-acquire.
    assert tr.periodic(now=10.0, mode_active=False) == TokenAction.KEEPALIVE

    # Past grace_s with mode inactive and no new command: release once.
    action = tr.periodic(now=31.0, mode_active=False)
    assert action == TokenAction.RELEASE
    assert tr.state == TokenState.RELEASED

    # Idempotent: once released, periodic keeps returning NONE, never
    # spams RELEASE nor silently re-claims via a forced ACQUIRE.
    assert tr.periodic(now=41.0, mode_active=False) == TokenAction.NONE
    assert tr.periodic(now=100.0, mode_active=False) == TokenAction.NONE


def test_idle_past_grace_boundary_exactly_at_grace_s_is_not_within_grace():
    policy = TokenPolicy(grace_s=30.0, keepalive_s=10.0)
    tr = TokenTracker(policy)
    tr.observe_token_id(token_id=1, our_session=1, now=0.0)
    tr.note_command(now=0.0)
    # now - last_command == grace_s exactly: grace has elapsed (< is strict).
    action = tr.periodic(now=30.0, mode_active=False)
    assert action == TokenAction.RELEASE


def test_no_token_held_periodic_is_noop():
    tr = TokenTracker(TokenPolicy())
    assert tr.state == TokenState.UNKNOWN
    assert tr.periodic(now=0.0, mode_active=True) == TokenAction.NONE
    assert tr.periodic(now=1000.0, mode_active=False) == TokenAction.NONE


def test_hold_token_true_never_releases():
    policy = TokenPolicy(hold_token=True, grace_s=30.0, keepalive_s=10.0)
    tr = TokenTracker(policy)
    tr.observe_token_id(token_id=1, our_session=1, now=0.0)

    # Way past any grace window, mode inactive, no commands ever sent:
    # hold_token=True means we keep the token alive forever via
    # keepalive, never RELEASE.
    assert tr.periodic(now=0.0, mode_active=False) == TokenAction.KEEPALIVE
    assert tr.periodic(now=1000.0, mode_active=False) == TokenAction.KEEPALIVE
    assert tr.periodic(now=1000.0, mode_active=False) == TokenAction.NONE  # not due yet
    assert tr.periodic(now=100_000.0, mode_active=False) == TokenAction.KEEPALIVE
    assert tr.state != TokenState.RELEASED


def test_note_command_resets_grace_window():
    policy = TokenPolicy(grace_s=30.0, keepalive_s=10.0)
    tr = TokenTracker(policy)
    tr.observe_token_id(token_id=1, our_session=1, now=0.0)
    tr.note_command(now=0.0)

    # A fresh command arrives just before the original grace deadline,
    # resetting the window.
    tr.note_command(now=29.0)

    # At t=30 (30s after the *first* command — which alone would have
    # elapsed grace, see the boundary test above — but only 1s after
    # the refreshed command) the tracker must still be within grace:
    # never RELEASE.
    action = tr.periodic(now=30.0, mode_active=False)
    assert action != TokenAction.RELEASE
    assert tr.state != TokenState.RELEASED


def test_never_calls_real_clock(monkeypatch):
    """TokenTracker must never read the wall/monotonic clock itself."""
    import time as _time

    def _boom():
        raise AssertionError("TokenTracker must not call time.monotonic()")

    monkeypatch.setattr(_time, "monotonic", _boom)
    tr = TokenTracker(TokenPolicy())
    tr.observe_token_id(token_id=1, our_session=1, now=0.0)
    tr.note_command(now=0.0)
    tr.observe_rc(15, now=1.0)
    tr.required_before("PTSpeedSet", now=1.0)
    tr.periodic(now=1.0, mode_active=True)
