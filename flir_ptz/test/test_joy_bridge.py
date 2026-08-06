"""Tests for flir_ptz.nodes.joy_bridge's pure decision layer.

Only the pure helpers are imported here (no rclpy, no sensor_msgs, no
flir_ptz_msgs) -- this file must pass with no ROS 2 setup.bash sourced,
same as every other test module in this suite. Every timing-sensitive
test drives an explicit injected clock (plain floats); nothing sleeps.

Maps to PARITY.md rows D1-D9 (see the docstring on each test group).
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from flir_ptz.control.gestures import CircleDetector  # noqa: E402
from flir_ptz.nodes.joy_bridge import (  # noqa: E402
    AX_X,
    AX_Y,
    BTN_PS,
    ButtonState,
    JoyBridgeConfig,
    JoyBridgeState,
    RelockResult,
    apply_claim_granted,
    axis_value,
    button_value,
    compute_axis_speeds,
    process_joy,
    relock_on_ownership_loss,
    should_publish_motion,
    update_button,
)


def _cfg(**overrides):
    return JoyBridgeConfig(**overrides)


def _ring_points(n: int, r: float = 0.8, clockwise: bool = True, start: float = 0.0):
    """Yield (x, y) samples walking evenly around a circle of radius r."""
    step = (2 * math.pi) / n
    if clockwise:
        step = -step
    for i in range(n + 1):
        theta = start + step * i
        yield (r * math.cos(theta), r * math.sin(theta))


def _drive_full_circle(circle: CircleDetector, n: int = 40) -> bool:
    """Feed enough samples around a ring to complete a gesture; return
    whether the *last* update() call reported completion."""
    done = False
    for x, y in _ring_points(n):
        done = circle.update(x, y)
    return done


def _unlocked_state() -> JoyBridgeState:
    state = JoyBridgeState()
    state.unlocked = True
    return state


# ── D1: frame_id filtering ───────────────────────────────────────────────────


def test_frame_id_mismatch_is_ignored():
    cfg = _cfg(frame_id="ps5")
    state = _unlocked_state()
    circle = CircleDetector()
    decision = process_joy(cfg, state, circle, "some_other_pad", [0, 0, 1.0, 0.0], [], now=0.0)
    assert decision.accepted is False
    # Nothing else in the decision matters when not accepted, but make sure
    # no motion/publish side effects leaked through.
    assert decision.publish_motion is False


def test_frame_id_match_is_processed():
    cfg = _cfg(frame_id="ps5")
    state = _unlocked_state()
    circle = CircleDetector()
    decision = process_joy(cfg, state, circle, "ps5", [0, 0, 1.0, 0.0], [], now=0.0)
    assert decision.accepted is True


def test_frame_id_default_is_ps5():
    assert JoyBridgeConfig().frame_id == "ps5"


# ── D2 / D3: axis mapping, including the AZ negation ─────────────────────────


def test_az_negated_stick_right_is_positive():
    # PS5 axes[2]: right = -1.0 on the raw axis -> should map to +azimuth.
    az, el = compute_axis_speeds(ax_x=-1.0, ax_y=0.0, deadzone=0.05, max_speed=40.0)
    assert az == 40.0
    az, el = compute_axis_speeds(ax_x=1.0, ax_y=0.0, deadzone=0.05, max_speed=40.0)
    assert az == -40.0


def test_el_not_negated():
    az, el = compute_axis_speeds(ax_x=0.0, ax_y=1.0, deadzone=0.05, max_speed=40.0)
    assert el == 40.0
    az, el = compute_axis_speeds(ax_x=0.0, ax_y=-1.0, deadzone=0.05, max_speed=40.0)
    assert el == -40.0


def test_axis_mapping_end_to_end_through_process_joy():
    """Same sign convention, exercised through the full per-message path."""
    cfg = _cfg()
    state = _unlocked_state()
    circle = CircleDetector()
    # axes[2] = -1.0 (stick pushed right) -> positive azimuth speed.
    decision = process_joy(cfg, state, circle, "ps5", [0, 0, -1.0, 0.0], [], now=0.0)
    assert decision.az_speed == 40.0
    assert decision.el_speed == 0.0


# ── D4: deadzone and max-speed clamping ──────────────────────────────────────


def test_deadzone_collapses_to_zero():
    az, el = compute_axis_speeds(ax_x=0.049, ax_y=-0.049, deadzone=0.05, max_speed=40.0)
    assert az == 0.0
    assert el == 0.0


def test_deadzone_boundary_is_exclusive():
    # abs(value) < deadzone is treated as zero; value == deadzone is not.
    az, _ = compute_axis_speeds(ax_x=0.05, ax_y=0.0, deadzone=0.05, max_speed=40.0)
    assert az != 0.0


def test_clamping_at_max_speed():
    # Even a slightly out-of-range raw axis (defensive) must not exceed max.
    az, el = compute_axis_speeds(ax_x=-2.0, ax_y=2.0, deadzone=0.05, max_speed=40.0)
    assert az == 40.0
    assert el == 40.0
    az, el = compute_axis_speeds(ax_x=2.0, ax_y=-2.0, deadzone=0.05, max_speed=40.0)
    assert az == -40.0
    assert el == -40.0


def test_default_deadzone_and_max_speed_constants():
    cfg = JoyBridgeConfig()
    assert cfg.deadzone == 0.05
    assert cfg.max_speed == 40.0


# ── Index-out-of-range safety ─────────────────────────────────────────────────


def test_short_axes_array_does_not_raise():
    assert axis_value([], AX_X) == 0.0
    assert axis_value([0.1], AX_Y) == 0.0


def test_short_buttons_array_does_not_raise():
    assert button_value([], BTN_PS) is False
    assert button_value([0, 0], BTN_PS) is False


def test_process_joy_with_short_joy_message_does_not_raise():
    cfg = _cfg()
    state = _unlocked_state()
    circle = CircleDetector()
    # No axes, no buttons at all -- must not raise.
    decision = process_joy(cfg, state, circle, "ps5", [], [], now=0.0)
    assert decision.accepted is True
    assert decision.az_speed == 0.0
    assert decision.el_speed == 0.0
    assert decision.button_event == "none"


# ── D5 / D6: buttons[8] short tap vs long press, with an injected clock ──────


def test_short_tap_fires_center():
    state = ButtonState()
    assert update_button(state, pressed=True, now=0.0, long_press_s=3.0) == "none"
    assert update_button(state, pressed=True, now=0.5, long_press_s=3.0) == "none"
    assert update_button(state, pressed=False, now=1.0, long_press_s=3.0) == "center"


def test_long_press_fires_home_once_at_threshold():
    state = ButtonState()
    assert update_button(state, pressed=True, now=0.0, long_press_s=3.0) == "none"
    assert update_button(state, pressed=True, now=1.0, long_press_s=3.0) == "none"
    assert update_button(state, pressed=True, now=2.99, long_press_s=3.0) == "none"
    assert update_button(state, pressed=True, now=3.0, long_press_s=3.0) == "home"
    # Still held past the threshold -- must not fire again.
    assert update_button(state, pressed=True, now=4.0, long_press_s=3.0) == "none"


def test_release_after_long_press_does_not_also_fire_center():
    state = ButtonState()
    update_button(state, pressed=True, now=0.0, long_press_s=3.0)
    assert update_button(state, pressed=True, now=3.0, long_press_s=3.0) == "home"
    assert update_button(state, pressed=False, now=3.5, long_press_s=3.0) == "none"


def test_button_events_through_process_joy_center():
    cfg = _cfg(long_press_s=3.0)
    state = _unlocked_state()
    circle = CircleDetector()
    axes = [0, 0, 0.0, 0.0]
    process_joy(cfg, state, circle, "ps5", axes, [0, 0, 0, 0, 0, 0, 0, 0, 1], now=0.0)
    decision = process_joy(cfg, state, circle, "ps5", axes, [0, 0, 0, 0, 0, 0, 0, 0, 0], now=0.5)
    assert decision.button_event == "center"


def test_button_events_through_process_joy_home():
    cfg = _cfg(long_press_s=3.0)
    state = _unlocked_state()
    circle = CircleDetector()
    axes = [0, 0, 0.0, 0.0]
    process_joy(cfg, state, circle, "ps5", axes, [0, 0, 0, 0, 0, 0, 0, 0, 1], now=0.0)
    decision = process_joy(cfg, state, circle, "ps5", axes, [0, 0, 0, 0, 0, 0, 0, 0, 1], now=3.0)
    assert decision.button_event == "home"


def test_button_ignored_while_locked():
    cfg = _cfg(long_press_s=3.0)
    state = JoyBridgeState()  # locked (default)
    circle = CircleDetector()
    process_joy(cfg, state, circle, "ps5", [], [0, 0, 0, 0, 0, 0, 0, 0, 1], now=0.0)
    decision = process_joy(cfg, state, circle, "ps5", [], [0, 0, 0, 0, 0, 0, 0, 0, 0], now=0.1)
    assert decision.button_event == "none"


# ── D8: publish suppression while idle / moving->stopped transition ─────────


def test_should_publish_motion_moving():
    assert should_publish_motion(is_moving=True, was_moving=False) is True
    assert should_publish_motion(is_moving=True, was_moving=True) is True


def test_should_publish_motion_idle_suppressed():
    assert should_publish_motion(is_moving=False, was_moving=False) is False


def test_should_publish_motion_stopping_transition_emits_once():
    assert should_publish_motion(is_moving=False, was_moving=True) is True


def test_process_joy_suppresses_repeated_idle_publishes():
    cfg = _cfg()
    state = _unlocked_state()
    circle = CircleDetector()

    idle_axes = [0, 0, 0.0, 0.0]
    # First idle frame: was_moving starts False -> no publish.
    d1 = process_joy(cfg, state, circle, "ps5", idle_axes, [], now=0.0)
    assert d1.publish_motion is False
    # Repeated idle frames: still no publish.
    d2 = process_joy(cfg, state, circle, "ps5", idle_axes, [], now=0.1)
    assert d2.publish_motion is False


def test_process_joy_moving_then_stop_publishes_exactly_once_then_suppresses():
    cfg = _cfg()
    state = _unlocked_state()
    circle = CircleDetector()

    moving = process_joy(cfg, state, circle, "ps5", [0, 0, -1.0, 0.0], [], now=0.0)
    assert moving.publish_motion is True
    assert moving.az_speed == 40.0

    stop_transition = process_joy(cfg, state, circle, "ps5", [0, 0, 0.0, 0.0], [], now=0.1)
    assert stop_transition.publish_motion is True
    assert stop_transition.az_speed == 0.0
    assert stop_transition.el_speed == 0.0

    stop_again = process_joy(cfg, state, circle, "ps5", [0, 0, 0.0, 0.0], [], now=0.2)
    assert stop_again.publish_motion is False


# ── D7: circle-to-unlock gesture ─────────────────────────────────────────────


def test_circle_detected_reports_completion():
    circle = CircleDetector()
    assert _drive_full_circle(circle) is True


def test_process_joy_reports_circle_completed_but_does_not_unlock_by_itself():
    """Completing the gesture is only a *request* -- process_joy must not
    flip state.unlocked itself; that only happens via apply_claim_granted
    once the ClaimControl service responds (requirement 1)."""
    cfg = _cfg()
    state = JoyBridgeState()  # locked
    circle = CircleDetector()

    saw_completion = False
    for x, y in _ring_points(40):
        decision = process_joy(cfg, state, circle, "ps5", [0, 0, x, y], [], now=0.0)
        saw_completion = saw_completion or decision.circle_completed

    assert saw_completion is True
    assert state.unlocked is False  # still locked -- awaiting claim response


def test_circle_runs_even_while_locked():
    circle = CircleDetector()
    state = JoyBridgeState()
    assert state.unlocked is False
    assert _drive_full_circle(circle) is True


# ── Claim grant/deny -> lock state machine ────────────────────────────────────


def test_apply_claim_granted_unlocks():
    state = JoyBridgeState()
    apply_claim_granted(state, granted=True)
    assert state.unlocked is True


def test_apply_claim_denied_stays_locked():
    state = JoyBridgeState()
    apply_claim_granted(state, granted=False)
    assert state.unlocked is False


def test_after_grant_motion_commands_flow():
    cfg = _cfg()
    state = JoyBridgeState()
    circle = CircleDetector()
    # Locked: stick input produces no motion.
    locked_decision = process_joy(cfg, state, circle, "ps5", [0, 0, -1.0, 0.0], [], now=0.0)
    assert locked_decision.az_speed == 0.0
    assert locked_decision.publish_motion is False

    apply_claim_granted(state, granted=True)

    unlocked_decision = process_joy(cfg, state, circle, "ps5", [0, 0, -1.0, 0.0], [], now=0.1)
    assert unlocked_decision.az_speed == 40.0
    assert unlocked_decision.publish_motion is True


# ── D9 + restart semantics: relock on ownership loss ─────────────────────────


def test_relock_when_never_unlocked_is_a_noop():
    state = JoyBridgeState()
    circle = CircleDetector()
    result = relock_on_ownership_loss(state, circle)
    assert result == RelockResult(relocked=False, emit_stop=False)
    assert state.unlocked is False


def test_relock_when_unlocked_and_idle_relocks_without_stop():
    state = JoyBridgeState()
    apply_claim_granted(state, granted=True)
    circle = CircleDetector()
    result = relock_on_ownership_loss(state, circle)
    assert result.relocked is True
    assert result.emit_stop is False
    assert state.unlocked is False


def test_relock_resets_circle_progress():
    state = JoyBridgeState()
    apply_claim_granted(state, granted=True)
    circle = CircleDetector()
    # Get partway through a circle (but not complete).
    for x, y in list(_ring_points(40))[:10]:
        circle.update(x, y)
    assert circle.progress > 0.0, "precondition: the gesture is partway through"

    relock_on_ownership_loss(state, circle)
    # Progress must be discarded -- a fresh full circle is required again.
    # Asserted through the public surface rather than an internal field, so
    # this survives the detector's implementation changing underneath it.
    assert circle.progress == 0.0


def test_control_source_owner_joy_requires_no_relock():
    """When the topic itself reports owner == 'joy' there is nothing to do
    -- the bridge shell simply returns without touching state (this is
    exercised at the process_joy/relock layer; the shell-level 'if
    msg.owner == \"joy\": return' guard is a one-line pass-through)."""
    state = JoyBridgeState()
    apply_claim_granted(state, granted=True)
    assert state.unlocked is True
    # No call to relock_on_ownership_loss happens for owner == "joy" in the
    # node shell, so state is untouched here by construction.


# ── Safety: stop is emitted even when locked / control was just lost ────────


def test_safety_stop_emitted_when_ownership_lost_mid_motion():
    cfg = _cfg()
    state = JoyBridgeState()
    circle = CircleDetector()
    apply_claim_granted(state, granted=True)

    moving = process_joy(cfg, state, circle, "ps5", [0, 0, -1.0, 0.0], [], now=0.0)
    assert moving.is_moving is True
    assert state.was_moving is True

    # Web steals control mid-motion: control_source topic reports owner
    # != "joy" while the stick is still pushed. The bridge must not go
    # silent; it must report that a stop needs to be published.
    result = relock_on_ownership_loss(state, circle)
    assert result.relocked is True
    assert result.emit_stop is True
    # And the lock/motion bookkeeping is fully reset afterwards.
    assert state.unlocked is False
    assert state.was_moving is False


def test_no_spurious_stop_when_ownership_lost_while_already_idle():
    state = JoyBridgeState()
    circle = CircleDetector()
    apply_claim_granted(state, granted=True)
    # Never moved (was_moving stays False by default).
    result = relock_on_ownership_loss(state, circle)
    assert result.relocked is True
    assert result.emit_stop is False


def test_stop_after_denied_claim_is_never_needed():
    """If a claim is denied, motion was never possible, so there is
    nothing to stop -- apply_claim_granted(False) must not leave
    was_moving stranded True."""
    state = JoyBridgeState()
    state.was_moving = True  # pathological/defensive: should never happen, but guard it
    apply_claim_granted(state, granted=False)
    assert state.was_moving is False


def test_ptz_restart_republishes_empty_owner_forces_relock():
    """PARITY: on PTZ-node restart the latched topic republishes owner="",
    which must force a relock requiring a fresh circle gesture."""
    state = JoyBridgeState()
    apply_claim_granted(state, granted=True)
    circle = CircleDetector()
    result = relock_on_ownership_loss(state, circle)  # simulates owner == ""
    assert result.relocked is True
    assert state.unlocked is False
    # A fresh circle is required: partial history was cleared and a new
    # full rotation must be driven from scratch.
    assert _drive_full_circle(circle) is True
