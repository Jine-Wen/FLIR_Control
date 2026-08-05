#!/usr/bin/env python3
"""PS5 Joy -> FLIR PTZ bridge node (spec ARCHITECTURE.md sec 3.5/3.6/5).

Subscribes: <joy_topic> (sensor_msgs/Joy, frame_id="ps5" by default)
            flir/control_source (flir_ptz_msgs/ControlSource, latched)
Calls:      flir/claim_control  (flir_ptz_msgs/ClaimControl)
Publishes:  flir/cmd/joy_stick_control (flir_ptz_msgs/JoyStickControlCmd)
            flir/cmd/move_to           (flir_ptz_msgs/MoveToCmd)

Mapping (PS5 DualSense, frame_id == "ps5"):
  axes[2]   -- Left stick X  -> azimuth speed  (negated: stick-right = +AZ)
  axes[3]   -- Left stick Y  -> elevation speed
  buttons[8]-- PS  button    -> single tap  -> Center (AZ=0, EL=0)
                                 long press (>= 3 s) -> Home (AZ=0, EL=-90)

Arbitration lives in the PTZ node now, not here and not in the web node.
This bridge:
  - runs the circle-to-unlock gesture locally (it must run even while
    locked, so the stick can reclaim control),
  - on a completed circle, calls the ``flir/claim_control`` service and
    only starts sending motion once the response says ``granted``,
  - subscribes to the latched ``flir/control_source`` topic to learn
    when ownership has moved away from "joy" (web claimed it, or the
    PTZ node restarted and republished an empty owner) and re-locks
    itself in that case, requiring a fresh circle gesture to regain
    control.

Safety (non-negotiable): a zero-speed command is a stop, and stops are
always accepted by the arbitration layer regardless of ownership. If
control is taken away *while the stick is pushed*, this bridge must
still emit one final zero-speed command rather than silently going
quiet and leaving the camera slewing -- see ``relock_on_ownership_loss``.

Section boundary: everything above the "ROS shell" marker below is pure
decision logic -- no I/O, no rclpy, no wall-clock reads (time is always
an injected ``now: float``, monotonic seconds) -- and is unit-testable
with nothing but the standard library. The ``rclpy``-dependent imports
are guarded so that importing this module (to get at the pure helpers)
works even when ROS is not sourced.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional, Sequence

from flir_ptz.control.gestures import CircleDetector
from flir_ptz.control.profiles import clamp

# ── Constants (ported verbatim from the old flir_joy_bridge.py) ───────────────
MAX_SPEED    = 40.0   # deg/s -- matches web UI MAX_SPEED
DEADZONE     = 0.05   # stick values below this treated as zero
LONG_PRESS_S = 3.0    # seconds to trigger Home instead of Centre

# Axis / button indices
AX_X   = 2   # axes[2] -> azimuth
AX_Y   = 3   # axes[3] -> elevation
BTN_PS = 8   # buttons[8] -> centre / home


# ═══════════════════════════════════════════════════════════════════════════
# Pure decision logic (no rclpy, no I/O, no clock reads)
# ═══════════════════════════════════════════════════════════════════════════


def axis_value(axes: Sequence[float], index: int) -> float:
    """Safe axes[index] lookup -- a short Joy message must never raise."""
    return float(axes[index]) if len(axes) > index else 0.0


def button_value(buttons: Sequence[int], index: int) -> bool:
    """Safe buttons[index] lookup -- a short Joy message must never raise."""
    return bool(buttons[index]) if len(buttons) > index else False


def compute_axis_speeds(
    ax_x: float, ax_y: float, deadzone: float, max_speed: float
) -> tuple[float, float]:
    """Map raw stick axes to (azimuth_speed, elevation_speed) in deg/s.

    PS5 axes[2]: left=+1, right=-1 -> negated to match the web
    convention (stick-right = positive azimuth). axes[3] is used as-is.
    Values inside the deadzone collapse to exactly 0.0; everything else
    is scaled by max_speed and clamped to +/- max_speed.
    """
    az = 0.0 if abs(ax_x) < deadzone else clamp(-ax_x * max_speed, -max_speed, max_speed)
    el = 0.0 if abs(ax_y) < deadzone else clamp(ax_y * max_speed, -max_speed, max_speed)
    return az, el


def should_publish_motion(is_moving: bool, was_moving: bool) -> bool:
    """Publish only while moving, or on the moving->stopped transition.

    This keeps the topic from being flooded with (0, 0) while idle, but
    still emits exactly one zero-speed command when the stick returns
    to centre.
    """
    return is_moving or was_moving


@dataclass
class ButtonState:
    """Mutable short-tap / long-press tracking for buttons[BTN_PS]."""

    pressed: bool = False
    press_ts: Optional[float] = None
    fired_long: bool = False


def update_button(state: ButtonState, pressed: bool, now: float, long_press_s: float) -> str:
    """Advance the button edge/hold state machine; returns an event.

    Returns "home" the instant a held press crosses ``long_press_s``,
    "center" on release of a press that never crossed the threshold, or
    "none" otherwise. Driven purely by (state, pressed, now) -- no
    threads, no timers, no real sleeping: this is evaluated on every
    incoming Joy message, which arrive continuously from any live
    joystick driver.
    """
    event = "none"
    if pressed and not state.pressed:
        # Press edge: start the hold timer.
        state.press_ts = now
        state.fired_long = False
    elif pressed and state.pressed:
        # Held: fire "home" exactly once when the threshold is crossed.
        if not state.fired_long and state.press_ts is not None and (now - state.press_ts) >= long_press_s:
            state.fired_long = True
            event = "home"
    elif not pressed and state.pressed:
        # Release edge: short tap -> center, unless long-press already fired.
        if not state.fired_long:
            event = "center"
        state.press_ts = None
    state.pressed = pressed
    return event


@dataclass
class JoyBridgeConfig:
    """Immutable-in-practice tuning knobs sourced from ROS parameters."""

    frame_id: str = "ps5"
    max_speed: float = MAX_SPEED
    deadzone: float = DEADZONE
    long_press_s: float = LONG_PRESS_S


@dataclass
class JoyBridgeState:
    """Mutable bridge state, advanced only through the pure functions below."""

    unlocked: bool = False       # True only after a granted claim response
    was_moving: bool = False     # last published command was non-zero speed
    button: ButtonState = field(default_factory=ButtonState)


@dataclass(frozen=True)
class JoyDecision:
    """What ``process_joy`` decided to do with one incoming Joy message."""

    accepted: bool                  # False if frame_id didn't match (message ignored)
    az_speed: float = 0.0
    el_speed: float = 0.0
    publish_motion: bool = False    # publish JoyStickControlCmd(az_speed, el_speed)?
    is_moving: bool = False
    circle_completed: bool = False  # a full circle just finished -> call ClaimControl
    button_event: str = "none"      # "none" | "center" | "home"


def process_joy(
    cfg: JoyBridgeConfig,
    state: JoyBridgeState,
    circle: CircleDetector,
    frame_id: str,
    axes: Sequence[float],
    buttons: Sequence[int],
    now: float,
) -> JoyDecision:
    """Pure per-message decision: frame filter, gesture, motion, buttons.

    ``state`` and ``circle`` are mutated in place (this mirrors the old
    node's field updates) but no I/O happens here -- callers decide what
    to publish / call based on the returned ``JoyDecision``.
    """
    if frame_id != cfg.frame_id:
        return JoyDecision(accepted=False)

    ax_x = axis_value(axes, AX_X)
    ax_y = axis_value(axes, AX_Y)

    # Circle detection ALWAYS runs, even while locked, so the stick can
    # reclaim control. Completing a circle only asks for a claim; the
    # caller must confirm it against ClaimControl before this bridge is
    # actually allowed to move the camera (state.unlocked flips via
    # apply_claim_granted, not here).
    circle_completed = circle.update(ax_x, ax_y)
    if circle_completed:
        circle.reset()

    az_speed = el_speed = 0.0
    is_moving = False
    publish_motion = False
    button_event = "none"

    if state.unlocked:
        az_speed, el_speed = compute_axis_speeds(ax_x, ax_y, cfg.deadzone, cfg.max_speed)
        is_moving = az_speed != 0.0 or el_speed != 0.0
        publish_motion = should_publish_motion(is_moving, state.was_moving)
        state.was_moving = is_moving

        button_event = update_button(state.button, button_value(buttons, BTN_PS), now, cfg.long_press_s)

    return JoyDecision(
        accepted=True,
        az_speed=az_speed,
        el_speed=el_speed,
        publish_motion=publish_motion,
        is_moving=is_moving,
        circle_completed=circle_completed,
        button_event=button_event,
    )


def apply_claim_granted(state: JoyBridgeState, granted: bool) -> None:
    """Apply a ``ClaimControl`` response: only a granted claim unlocks."""
    state.unlocked = bool(granted)
    if not state.unlocked:
        state.was_moving = False


@dataclass(frozen=True)
class RelockResult:
    """Outcome of observing that "joy" no longer owns control."""

    relocked: bool     # True if this transitioned unlocked -> locked
    emit_stop: bool     # True if a zero-speed safety stop must be published now


def relock_on_ownership_loss(state: JoyBridgeState, circle: CircleDetector) -> RelockResult:
    """Handle a ``flir/control_source`` update whose owner isn't "joy".

    Covers both PARITY D9 (web takes control -> lock the joystick) and
    the restart case (PTZ node republishes owner="") -- both require a
    fresh circle gesture to regain control. Idempotent: calling this
    repeatedly while already locked is a no-op.

    SAFETY: if the stick was mid-motion (``state.was_moving``) when
    ownership moved away, ``emit_stop`` comes back True so the caller
    publishes one final zero-speed command -- the arbitration layer
    always accepts stops, so this is never blocked, and the camera is
    never left silently slewing.
    """
    was_unlocked = state.unlocked
    was_moving = state.was_moving
    state.unlocked = False
    state.was_moving = False
    if was_unlocked:
        circle.reset()
    return RelockResult(relocked=was_unlocked, emit_stop=was_moving)


# ═══════════════════════════════════════════════════════════════════════════
# ROS shell (thin: all decisions above are delegated to the pure layer)
# ═══════════════════════════════════════════════════════════════════════════

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )
    from sensor_msgs.msg import Joy

    from flir_ptz_msgs.msg import ControlSource, JoyStickControlCmd, MoveToCmd
    from flir_ptz_msgs.srv import ClaimControl

    _ROS_AVAILABLE = True
except ImportError:  # pragma: no cover -- exercised by running tests with no ROS sourced
    Node = object  # type: ignore[assignment,misc]
    _ROS_AVAILABLE = False


if _ROS_AVAILABLE:

    _CONTROL_SOURCE_QOS = QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
    )

    class FlirJoyBridge(Node):
        def __init__(self) -> None:
            super().__init__("flir_joy_bridge")

            self.declare_parameter("joy_topic", "/joy")
            self.declare_parameter("frame_id", "ps5")
            self.declare_parameter("max_speed", MAX_SPEED)
            self.declare_parameter("deadzone", DEADZONE)
            self.declare_parameter("long_press_s", LONG_PRESS_S)

            joy_topic = str(self.get_parameter("joy_topic").value)
            self._cfg = JoyBridgeConfig(
                frame_id=str(self.get_parameter("frame_id").value),
                max_speed=float(self.get_parameter("max_speed").value),
                deadzone=float(self.get_parameter("deadzone").value),
                long_press_s=float(self.get_parameter("long_press_s").value),
            )
            self._state = JoyBridgeState()
            self._circle = CircleDetector()

            self._joy_pub = self.create_publisher(JoyStickControlCmd, "flir/cmd/joy_stick_control", 5)
            self._move_pub = self.create_publisher(MoveToCmd, "flir/cmd/move_to", 5)

            self._claim_client = self.create_client(ClaimControl, "flir/claim_control")

            self.create_subscription(
                ControlSource, "flir/control_source", self._on_control_source, _CONTROL_SOURCE_QOS
            )
            self.create_subscription(Joy, joy_topic, self._on_joy, 10)

            self.get_logger().info(
                f"flir_joy_bridge: listening on '{joy_topic}' (frame_id='{self._cfg.frame_id}')"
            )

        # ── Incoming Joy ────────────────────────────────────────────────────

        def _on_joy(self, msg: "Joy") -> None:
            decision = process_joy(
                self._cfg,
                self._state,
                self._circle,
                msg.header.frame_id,
                msg.axes,
                msg.buttons,
                time.monotonic(),
            )
            if not decision.accepted:
                return

            if decision.circle_completed:
                self.get_logger().info("Joy circle detected -- requesting control claim")
                self._send_claim_request()

            if decision.publish_motion:
                self._publish_speed(decision.az_speed, decision.el_speed)

            if decision.button_event == "center":
                self._publish_move(0.0, 0.0, "Center")
            elif decision.button_event == "home":
                self._publish_move(0.0, -90.0, "Home")

        # ── Control-source ownership (latched) ─────────────────────────────

        def _on_control_source(self, msg: "ControlSource") -> None:
            if msg.owner == "joy":
                return
            result = relock_on_ownership_loss(self._state, self._circle)
            if result.relocked:
                self.get_logger().info(
                    f"Joy locked: control owner is '{msg.owner or '(none)'}'"
                )
            if result.emit_stop:
                self.get_logger().info("Joy safety stop: control was taken while moving")
                self._publish_speed(0.0, 0.0)

        # ── Claim service ────────────────────────────────────────────────────

        def _send_claim_request(self) -> None:
            request = ClaimControl.Request()
            request.source = "joy"
            request.release = False
            future = self._claim_client.call_async(request)
            future.add_done_callback(self._on_claim_response)

        def _on_claim_response(self, future) -> None:
            try:
                response = future.result()
            except Exception as exc:  # service down, node restarted mid-call, ...
                self.get_logger().warning(f"flir/claim_control call failed: {exc}")
                return
            apply_claim_granted(self._state, bool(response.granted))
            if self._state.unlocked:
                self.get_logger().info("Joy unlocked / reclaimed control (circle detected)")
            else:
                self.get_logger().warning(
                    f"Joy claim denied; control held by '{response.owner}'"
                )

        # ── Publish helpers ──────────────────────────────────────────────────

        def _publish_speed(self, az_speed: float, el_speed: float) -> None:
            cmd = JoyStickControlCmd()
            cmd.azimuth_speed = az_speed
            cmd.elevation_speed = el_speed
            cmd.source = "joy"
            self._joy_pub.publish(cmd)

        def _publish_move(self, az: float, el: float, label: str) -> None:
            msg = MoveToCmd()
            msg.target_azimuth = az
            msg.target_elevation = el
            msg.source = "joy"
            self._move_pub.publish(msg)
            self.get_logger().info(f"Joy -> {label} (AZ={az}, EL={el})")

    def main(args=None) -> None:
        rclpy.init(args=args)
        node = FlirJoyBridge()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    if not _ROS_AVAILABLE:
        raise RuntimeError("flir_ptz.nodes.joy_bridge requires rclpy; source your ROS 2 setup.bash")
    main()
