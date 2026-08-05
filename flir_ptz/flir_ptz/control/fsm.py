#!/usr/bin/env python3
"""The PTZ motion state machine (spec ARCHITECTURE.md sec. 3.4) -- the core of
this rewrite.

Pure decision logic: no I/O, no coroutine scheduling, no ``rclpy``, no
``httpx``, and no reads of the wall clock. Every method that cares about
time takes an explicit ``now: float`` (monotonic seconds) supplied by the
caller. This is what makes it possible to test the whole hardware-tuned
control behaviour (GOTO stall detection, TRACK settle/hold hysteresis, the
SCAN sub-state sequence, ...) offline, deterministically, with no camera
and no clock.

Architectural background (see spec sec. 1 and sec. 8 rule 2): the old
implementation ran a background telemetry poller *and* separate per-command
background tasks on an event loop, all serialised behind a single HTTP
semaphore -- doubling camera load and letting closed-loop TRACK act on
stale data, and requiring
``Task.cancel()`` scattered through six different command handlers to
implement preemption. The new design is a single read-then-control tick
loop (``control/controller.py``): ``MotionFSM.step(sample, now)`` is called
once per tick with a sample taken microseconds earlier, and returns the
actions the caller should send -- it never sends anything itself. There is
nothing to cancel: ``submit()`` just replaces a "latest wins" pending
intent, and the next ``step()`` picks it up and runs the mode's transition
table, which emits the correct exit actions (``Stop`` and/or ``ScanOff``)
once, in one place, instead of being duplicated at every call site the way
the old ``_handle_move_to`` / ``_handle_track`` / ``_handle_scan_start`` /
``destroy_node`` all did.

The hand-tuned behaviour ported bit-for-bit from ``flir_ptz_node.py`` (do
not "improve" any constant -- see module docstring of ``control/profiles.py``
and spec sec. 8 rule 4):

======================================================================================
Behaviour                                                          Old location
======================================================================================
GOTO: single PTGeoAzimuthElevationSet, camera drives itself        node:517
GOTO stall detection (1.5s grace / 0.05deg progress / 4s give up)  node:484-486, 560-579
GOTO arrival (speed < 0.1 AND in az_tol/el_tol)                    node:549
TRACK: closed-loop choose_speed on both axes                       node:770-771
TRACK settle/hold hysteresis (settle_samples)                      node:722-750
TRACK overshoot brake (sign reversal, |az|<3.0 or |el|<2.0)        node:756-768
TRACK elevation cap near target (|el_err|<1.2 -> |cmd|<=1.4)       node:773-774
TRACK retarget in place, no restart                                node:681-694
TRACK idle exit after 1.5s with no new target                      node:631, 730-737
VELOCITY: clamp +-40, deadband 0.01, resend only on change          node:884-907
VELOCITY idle exit after 1.5s, immediate exit on explicit zero     node:880, 909-910
SCAN sequence                                                      node:1014-1147
SCAN external-interrupt detection (Mode==0, both speeds <= 0.1)    node:1126-1134
Priority: goto/track/velocity/home all preempt scan                node:498, 648, 860
======================================================================================

HOMING is a first-class mode, not a synchronous 120-line special case
bolted onto ``destroy_node()`` the way ``_sync_home`` /
``_check_we_have_control_sync`` were. It reuses the exact same
single-shot-goto-plus-stall-detection machinery as GOTO (see
``_tick_move``), bounded by an additional outer ``cfg.home_timeout_s``
deadline -- see the "Design decisions and ambiguities" note near
``_enter_homing`` for why.

Two rows of PARITY.md that this module deliberately does *not* implement:
A15 (joystick RC=15 -> re-acquire token and resend) and A19 (scan watches
for lost control token). Per spec sec. 4.1, only ``nexus/session.py`` is
allowed to see Nexus return codes at all -- its token-aware ``execute()``
handles RC=15 (revive) and RC=21 (re-login) transparently and retries once,
so "the control loop never sees a token error". ``MotionFSM.step()`` has no
input channel for command-execution outcomes (its signature is
``step(sample, now)``), so it structurally cannot implement A15/A19 the way
the old code's inline ``has_control()`` polling did. The nearest equivalent
available to the FSM is the external-interrupt check already required for
A18 (``Mode == 0`` and both speeds ~0), which also happens to be what a
camera-side token takeover would look like from here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

from flir_ptz.control.config import ControlConfig
from flir_ptz.control.profiles import (
    AZ_PROFILE,
    EL_PROFILE,
    az_error,
    choose_speed,
    clamp,
    normalize_az,
    sign,
)
from flir_ptz.nexus.protocol import PtSample

# -- Constants ported verbatim from flir_ptz_node.py that have no home in
# ControlConfig (fsm.py does not own control/config.py -- spec sec. "file
# ownership"). Hardcoded in the old code too; not derived from any launch
# parameter there either.
_SCAN_SET_LIMITS_WAIT_S = 0.25  # node:1090 -- old code slept a flat 0.25s here
_SCAN_PREPOSITION_TIMEOUT_S = 15.0  # node:1058 -- `_el_deadline = now + 15.0`
_SCAN_CONFIRM_TIMEOUT_S = 3.0  # node:1105 -- `_wait_scan_started(3.0)`


class Mode(Enum):
    IDLE = auto()
    GOTO = auto()
    TRACK = auto()
    VELOCITY = auto()
    SCAN = auto()
    HOMING = auto()


class _ScanSub(Enum):
    """Internal SCAN sub-states (spec sec. 3.4): one tick advances at most
    one sub-state. Not part of the public ``Mode`` enum -- ``MotionFSM.mode``
    always reports ``Mode.SCAN`` for all six of these; sub-state is only
    observable indirectly via ``StepResult.is_scanning`` (True from
    ``ON`` onward, matching the old ``_scan_mode_on`` flag)."""

    STOP_PREV = auto()
    PREPOSITION = auto()
    SET_SPEED = auto()
    SET_LIMITS = auto()
    ON = auto()
    ACTIVE = auto()


# -- Intents (submitted by ROS callbacks via MotionFSM.submit) --------------


@dataclass(frozen=True)
class Intent:
    """A request submitted to the FSM. Construct via the convenience
    static methods below rather than the raw constructor."""

    kind: str  # "goto"|"track"|"velocity"|"scan_start"|"scan_stop"|"home"|"stop"
    az: float = 0.0
    el: float = 0.0
    az_speed: float = 0.0
    el_speed: float = 0.0
    each_side: float = 0.0
    scan_speed: float = 0.0
    source: str = ""

    @staticmethod
    def goto(az: float, el: float, source: str = "") -> "Intent":
        return Intent("goto", az=az, el=el, source=source)

    @staticmethod
    def track(az: float, el: float, source: str = "") -> "Intent":
        return Intent("track", az=az, el=el, source=source)

    @staticmethod
    def velocity(az_speed: float, el_speed: float, source: str = "") -> "Intent":
        return Intent("velocity", az_speed=az_speed, el_speed=el_speed, source=source)

    @staticmethod
    def scan_start(
        center: float, each_side: float, el: float, speed: float, source: str = ""
    ) -> "Intent":
        return Intent(
            "scan_start", az=center, el=el, each_side=each_side, scan_speed=speed, source=source
        )

    @staticmethod
    def scan_stop(source: str = "") -> "Intent":
        return Intent("scan_stop", source=source)

    @staticmethod
    def home(source: str = "") -> "Intent":
        return Intent("home", source=source)

    @staticmethod
    def stop(source: str = "") -> "Intent":
        return Intent("stop", source=source)


# -- Actions (emitted by the FSM for the caller to send to the camera) ------


@dataclass(frozen=True)
class SetSpeed:
    az: float
    el: float


@dataclass(frozen=True)
class Goto:
    az: float
    el: float


@dataclass(frozen=True)
class Stop:
    pass


@dataclass(frozen=True)
class ScanSetSpeed:
    speed: float


@dataclass(frozen=True)
class ScanSetLimits:
    left: float
    right: float


@dataclass(frozen=True)
class ScanOn:
    pass


@dataclass(frozen=True)
class ScanOff:
    pass


Action = SetSpeed | Goto | Stop | ScanSetSpeed | ScanSetLimits | ScanOn | ScanOff


@dataclass(frozen=True)
class StepResult:
    actions: tuple  # tuple[Action, ...]
    mode: Mode
    seq: int
    tick_period: float
    is_moving: bool
    is_scanning: bool
    done: bool
    note: str


# -- The state machine -------------------------------------------------------


class MotionFSM:
    """Single-tick motion state machine. No I/O, no async, no clock reads.

    Usage (mirrors ``control/controller.py``'s tick loop, spec sec. 4.2): each
    tick, ``submit()`` the latest intent (latest-wins, not executed yet),
    then the caller asynchronously reads a fresh sample from the session,
    then calls ``step(sample, now)`` -- purely, no I/O -- and finally
    asynchronously sends every action in ``result.actions`` to the camera.
    """

    def __init__(self, cfg: ControlConfig) -> None:
        self._cfg = cfg
        self._mode: Mode = Mode.IDLE
        self._seq: int = 0
        self._pending: Optional[Intent] = None

        # GOTO / HOMING share one set of "single shot move" bookkeeping --
        # only one of the two modes is ever active at a time.
        self._move_target: tuple[float, float] = (0.0, 0.0)
        self._move_sent_at: float = 0.0
        self._move_stall_since: Optional[float] = None
        self._move_prev_err: float = math.inf
        self._move_motion_started: bool = False
        self._move_arrived_logged: bool = False
        self._move_deadline: Optional[float] = None  # HOMING only

        # TRACK
        self._track_target: tuple[float, float] = (0.0, 0.0)
        self._track_last_target_at: float = 0.0
        self._track_settle: int = 0
        self._track_hold_mode: bool = False
        self._track_prev_az_err: Optional[float] = None
        self._track_prev_el_err: Optional[float] = None

        # VELOCITY
        self._vel_target: tuple[float, float] = (0.0, 0.0)
        self._vel_target_at: float = 0.0
        self._vel_last_sent: Optional[tuple[float, float]] = None

        # SCAN
        self._scan_sub: Optional[_ScanSub] = None
        self._scan_center_az: float = 0.0
        self._scan_each_side: float = 0.0
        self._scan_elevation: float = 0.0
        self._scan_speed: float = 0.0
        self._scan_left: float = 0.0
        self._scan_right: float = 0.0
        self._scan_wait_until: float = 0.0
        self._scan_deadline: float = 0.0
        self._scan_confirm_deadline: Optional[float] = None

    @property
    def mode(self) -> Mode:
        return self._mode

    def submit(self, intent: Intent, now: float) -> None:
        """Store the latest intent. Latest wins -- a second ``submit()``
        before the next ``step()`` silently replaces the first. Does not
        execute anything; the next ``step()`` call picks it up."""
        self._pending = intent

    def step(self, sample: Optional[PtSample], now: float) -> StepResult:
        """Advance the state machine by one tick. Pure: no I/O, no clock
        reads, does not mutate ``sample``. ``sample=None`` means this tick's
        camera read failed -- every mode tolerates that by holding its
        current commanded state rather than crashing or resending blindly;
        modes with a real-time deadline (GOTO/HOMING stall, HOMING timeout,
        TRACK/VELOCITY idle, SCAN sub-state timeouts) keep counting down
        against ``now`` regardless, so persistent read failure still
        degrades to "give up" rather than hanging forever."""
        intent = self._pending
        self._pending = None

        actions: list = []
        notes: list[str] = []
        done = False
        run_tick = True

        if intent is not None:
            i_actions, i_note, i_done, run_tick = self._dispatch_intent(intent, now)
            actions.extend(i_actions)
            if i_note:
                notes.append(i_note)
            done = done or i_done

        if run_tick:
            t_actions, t_note, t_done = self._run_tick(sample, now)
            actions.extend(t_actions)
            if t_note:
                notes.append(t_note)
            done = done or t_done

        is_moving = self._mode in (Mode.GOTO, Mode.TRACK, Mode.VELOCITY, Mode.HOMING)
        is_scanning = self._mode == Mode.SCAN and self._scan_sub in (
            _ScanSub.ON,
            _ScanSub.ACTIVE,
        )

        return StepResult(
            actions=tuple(actions),
            mode=self._mode,
            seq=self._seq,
            tick_period=self._tick_period(),
            is_moving=is_moving,
            is_scanning=is_scanning,
            done=done,
            note="; ".join(notes),
        )

    # -- Mode transition table ------------------------------------------
    #
    # One place, run on every transition, matching spec sec. 3.4: "转换时
    # 先执行离开动作（Stop 且/或 ScanOff），编码在转换表里一次，不要散落各处."
    # Leaving a driving mode (GOTO/TRACK/VELOCITY/HOMING) sends Stop();
    # leaving SCAN sends ScanOff(). This table only fires when the mode is
    # actually changing (see ``_transition``) -- a same-mode retarget
    # (TRACK->TRACK, VELOCITY->VELOCITY) or a same-mode restart
    # (GOTO->GOTO, HOMING->HOMING, SCAN->SCAN) does not re-run it, matching
    # the old code, which never sent a spurious Stop() between two
    # consecutive goto_position() calls.

    def _exit_actions(self, mode: Mode) -> list:
        if mode in (Mode.GOTO, Mode.TRACK, Mode.VELOCITY, Mode.HOMING):
            return [Stop()]
        if mode == Mode.SCAN:
            return [ScanOff()]
        return []

    def _transition(self, new_mode: Mode, force_bump: bool = False) -> None:
        """Change ``self._mode``. Bumps ``seq`` whenever the mode actually
        changes, or unconditionally when ``force_bump`` is set (used by
        GOTO/HOMING/SCAN, which -- unlike TRACK/VELOCITY -- always restart
        fresh even when re-triggered from the same mode; see A5/A21)."""
        if force_bump or self._mode != new_mode:
            self._seq += 1
        self._mode = new_mode
        if new_mode != Mode.SCAN:
            self._scan_sub = None

    def _dispatch_intent(self, intent: Intent, now: float):
        """Handle a freshly-submitted intent. Returns
        ``(actions, note, done, run_tick_this_same_step)``.

        ``run_tick_this_same_step`` controls whether the mode's ordinary
        per-tick logic (``_run_tick``) also executes in the same call to
        ``step()`` that processed this intent:

        * GOTO / HOMING / SCAN start: **no** -- the old code always sent
          the one-shot command and wound up sleeping a full poll interval
          before ever looking at a fresh sample again (node:517-530,
          node:1051-1060). So here too: emit the entry action(s), then wait
          for the *next* tick to evaluate anything.
        * TRACK / VELOCITY: **yes** -- old ``_track_abs``/
          ``_joy_stick_control_mode`` evaluate the very first loop
          iteration immediately, with no artificial pre-sleep
          (node:665-717, node:872-880). So here too: the freshly (re)set
          target is evaluated this same tick.
        * stop / scan_stop / unknown: yes, but mode is now IDLE (or
          unchanged for an ignored scan_stop) so it is a no-op either way.
        """
        kind = intent.kind

        if kind == "stop":
            exit_acts = self._exit_actions(self._mode)
            self._transition(Mode.IDLE)
            return list(exit_acts), "stop: halted", False, True

        if kind == "scan_stop":
            if self._mode == Mode.SCAN:
                self._transition(Mode.IDLE)
                return [ScanOff()], "scan_stop: halted", False, True
            return [], "scan_stop: not scanning, ignored", False, True

        if kind == "goto":
            exit_acts = [] if self._mode == Mode.GOTO else self._exit_actions(self._mode)
            self._transition(Mode.GOTO, force_bump=True)
            self._enter_goto(intent.az, intent.el, now)
            note = f"goto -> az={intent.az:.2f} el={intent.el:.2f}"
            return list(exit_acts) + [Goto(intent.az, intent.el)], note, False, False

        if kind == "home":
            exit_acts = [] if self._mode == Mode.HOMING else self._exit_actions(self._mode)
            self._transition(Mode.HOMING, force_bump=True)
            self._enter_homing(now)
            az, el = self._cfg.home_az, self._cfg.home_el
            note = f"home -> az={az:.2f} el={el:.2f}"
            return list(exit_acts) + [Goto(az, el)], note, False, False

        if kind == "track":
            fresh = self._mode != Mode.TRACK
            if fresh:
                exit_acts = self._exit_actions(self._mode)
                self._transition(Mode.TRACK)
                self._enter_track(intent.az, intent.el, now)
                note = f"track start -> az={intent.az:.2f} el={intent.el:.2f}"
            else:
                exit_acts = []
                self._retarget_track(intent.az, intent.el, now)
                note = f"track retarget -> az={intent.az:.2f} el={intent.el:.2f}"
            return list(exit_acts), note, False, True

        if kind == "velocity":
            fresh = self._mode != Mode.VELOCITY
            if fresh:
                exit_acts = self._exit_actions(self._mode)
                self._transition(Mode.VELOCITY)
                self._enter_velocity(intent.az_speed, intent.el_speed, now)
                note = "velocity start"
            else:
                exit_acts = []
                self._retarget_velocity(intent.az_speed, intent.el_speed, now)
                note = "velocity update"
            return list(exit_acts), note, False, True

        if kind == "scan_start":
            prev_mode = self._mode
            exit_acts = [] if prev_mode == Mode.SCAN else self._exit_actions(prev_mode)
            self._transition(Mode.SCAN, force_bump=True)
            self._enter_scan(intent.az, intent.each_side, intent.el, intent.scan_speed, now)
            note = (
                f"scan start center={intent.az:.2f} +-{intent.each_side:.2f} "
                f"el={intent.el:.2f} speed={intent.scan_speed:.2f}"
            )
            return list(exit_acts) + [ScanOff()], note, False, False

        return [], f"unknown intent kind: {kind!r}", False, True

    def _run_tick(self, sample: Optional[PtSample], now: float):
        """Ordinary per-tick logic for whatever ``self._mode`` currently
        is. Returns ``(actions, note, done)``."""
        if self._mode == Mode.IDLE:
            return [], "", False
        if self._mode in (Mode.GOTO, Mode.HOMING):
            return self._tick_move(sample, now)
        if self._mode == Mode.TRACK:
            return self._tick_track(sample, now)
        if self._mode == Mode.VELOCITY:
            return self._tick_velocity(sample, now)
        if self._mode == Mode.SCAN:
            return self._tick_scan(sample, now)
        return [], "", False

    def _tick_period(self) -> float:
        if self._mode == Mode.IDLE:
            return 1.0 / self._cfg.poll_hz
        if self._mode == Mode.SCAN and self._scan_sub == _ScanSub.ACTIVE:
            return self._cfg.scan_poll_ms / 1000.0
        return self._cfg.poll_ms / 1000.0

    # -- GOTO / HOMING (shared: single Goto, camera drives itself) ------
    #
    # Ported from node:480-590 (_move_to_abs). HOMING reuses the exact same
    # machinery with one addition: an outer ``cfg.home_timeout_s`` deadline.
    #
    # Design decision (spec was silent on the exact mechanism -- flagging
    # per instructions rather than silently picking one): the old code's
    # shutdown homing (``_sync_home``, node:1334-1423) used a *different*,
    # simpler proportional controller (gain 2.5, caps +-20/+-15 deg/s,
    # arrival at |err|<0.8 both axes, no settle hysteresis, no stall
    # detection, flat 20s deadline) that had nothing to do with GOTO's
    # stall-aware single-shot approach. Architecture sec. 4.2 explicitly
    # deletes that whole synchronous reimplementation and says "HOMING is a
    # mode", which -- given the "one tick loop, transition table, no
    # special-cased duplicate logic" theme of the whole rewrite -- I read
    # as: HOMING should reuse the *standard* mode machinery rather than
    # reintroduce a second, differently-tuned closed-loop controller. GOTO
    # is the standard machinery for "drive to an absolute position", so
    # HOMING = GOTO with a fixed target and cfg.home_timeout_s as an
    # additional outer deadline (on top of, not instead of, GOTO's own
    # grace/stall detection, since a JCU operator could equally well steal
    # control back during shutdown homing). If this reasoning is wrong,
    # please say so -- this was a judgment call, not a literal port.

    def _enter_goto(self, az: float, el: float, now: float) -> None:
        self._move_target = (az, el)
        self._move_sent_at = now
        self._move_stall_since = None
        self._move_prev_err = math.inf
        self._move_motion_started = False
        self._move_arrived_logged = False
        self._move_deadline = None

    def _enter_homing(self, now: float) -> None:
        self._move_target = (self._cfg.home_az, self._cfg.home_el)
        self._move_sent_at = now
        self._move_stall_since = None
        self._move_prev_err = math.inf
        self._move_motion_started = False
        self._move_arrived_logged = False
        self._move_deadline = now + self._cfg.home_timeout_s

    def _tick_move(self, sample: Optional[PtSample], now: float):
        actions: list = []

        if sample is None:
            return actions, "", False

        cur_az, cur_el = sample.abs_az, sample.abs_el
        spd = sample.speed_mag

        az_err = abs(az_error(self._move_target[0], cur_az))
        el_err = abs(self._move_target[1] - cur_el)
        in_tol = az_err <= self._cfg.az_tol and el_err <= self._cfg.el_tol

        # node:549 -- arrival requires BOTH low speed AND in-tolerance.
        if spd < 0.1 and in_tol:
            note = ""
            if not self._move_arrived_logged:
                self._move_arrived_logged = True
                note = f"arrived az={cur_az:.3f} el={cur_el:.3f}"
            self._transition(Mode.IDLE)
            return actions, note or "arrived", True

        self._move_arrived_logged = False

        # node:560-579 -- stall detection: 1.5s grace, then look for at
        # least 0.05deg of combined-error progress per poll; once motion
        # has been observed at least once (or the grace*2 = 3.0s "never
        # even started moving" fallback elapses), 4.0s of no progress ->
        # give up (JCU took over).
        if not in_tol:
            cur_err = az_err + el_err
            if (now - self._move_sent_at) < self._cfg.goto_grace_s:
                self._move_prev_err = cur_err
            else:
                if cur_err <= self._move_prev_err - self._cfg.stall_progress_deg:
                    self._move_motion_started = True
                    self._move_stall_since = None
                elif self._move_motion_started or (
                    now - self._move_sent_at
                ) >= self._cfg.goto_grace_s * 2:
                    if self._move_stall_since is None:
                        self._move_stall_since = now
                    elif now - self._move_stall_since >= self._cfg.stall_timeout_s:
                        note = (
                            f"stalled off-target az_err={az_err:.2f} "
                            f"el_err={el_err:.2f} -- giving up"
                        )
                        self._transition(Mode.IDLE)
                        return actions, note, True
                self._move_prev_err = cur_err
        else:
            self._move_stall_since = None

        # HOMING-only outer deadline, layered on top of the stall logic
        # above (self._move_deadline is None for plain GOTO).
        if self._move_deadline is not None and now >= self._move_deadline:
            self._transition(Mode.IDLE)
            return actions, "home timeout -- giving up", True

        return actions, "", False

    # -- TRACK (closed-loop PTSpeedModeSet) ------------------------------
    # Ported from node:626-807 (_track_abs).

    def _enter_track(self, az: float, el: float, now: float) -> None:
        self._track_target = (az, el)
        self._track_last_target_at = now
        self._track_settle = 0
        self._track_hold_mode = False
        self._track_prev_az_err = None
        self._track_prev_el_err = None

    def _retarget_track(self, az: float, el: float, now: float) -> None:
        """node:684-694 -- only resets settle/hold state (and the idle
        clock) when the target actually *changes*; re-submitting the same
        target is a silent no-op, exactly like the old code's
        ``if new_tgt and new_tgt != cur_tgt``."""
        new_target = (az, el)
        if new_target != self._track_target:
            self._track_target = new_target
            self._track_last_target_at = now
            self._track_settle = 0
            self._track_hold_mode = False
            self._track_prev_az_err = None
            self._track_prev_el_err = None

    def _tick_track(self, sample: Optional[PtSample], now: float):
        actions: list = []

        if sample is None:
            return actions, "", False

        cur_az, cur_el = sample.abs_az, sample.abs_el
        az_err = az_error(self._track_target[0], cur_az)
        el_err = self._track_target[1] - cur_el

        in_tol = abs(az_err) <= self._cfg.az_tol and abs(el_err) <= self._cfg.el_tol
        in_hold = abs(az_err) <= self._cfg.az_hold_tol and abs(el_err) <= self._cfg.el_hold_tol

        # node:722-737 -- dual-tolerance settle/hold hysteresis: az_tol/
        # el_tol to *enter* the settled band, the wider az_hold_tol/
        # el_hold_tol to *stay* in it once hold_mode has latched (prevents
        # chatter right at the az_tol/el_tol boundary). settle_samples
        # consecutive in-band ticks, AND >= track_idle_s since the last new
        # target, before exiting back to IDLE.
        if in_tol:
            self._track_hold_mode = True
            self._track_settle += 1
            actions.append(Stop())
            note = ""
            if self._track_settle >= self._cfg.settle_samples:
                note = f"arrived az={cur_az:.3f} el={cur_el:.3f}"
                idle = now - self._track_last_target_at
                if idle >= self._cfg.track_idle_s:
                    self._transition(Mode.IDLE)
                    self._track_prev_az_err = az_err
                    self._track_prev_el_err = el_err
                    return actions, note + " -- idle, exiting", True
            self._track_prev_az_err = az_err
            self._track_prev_el_err = el_err
            return actions, note, False

        if self._track_hold_mode and in_hold:
            self._track_settle += 1
            actions.append(Stop())
            note = ""
            if self._track_settle >= self._cfg.settle_samples:
                note = "arrived (hold mode)"
                idle = now - self._track_last_target_at
                if idle >= self._cfg.track_idle_s:
                    self._transition(Mode.IDLE)
                    self._track_prev_az_err = az_err
                    self._track_prev_el_err = el_err
                    return actions, note + " -- idle, exiting", True
            self._track_prev_az_err = az_err
            self._track_prev_el_err = el_err
            return actions, note, False

        self._track_settle = 0
        self._track_hold_mode = False

        # node:756-768 -- overshoot brake: error sign reversal (crossed
        # through the target between polls) while close to it (|az|<3.0 or
        # |el|<2.0) -> send a bare Stop and skip issuing a speed command
        # this tick. The old code did an explicit ``stop(); sleep(0.05);
        # set_speed(...)`` all within one iteration; in the one-tick
        # architecture there is no intra-tick sleep, so the "brief pause"
        # is instead realised as: this tick sends only Stop(), and the
        # speed computation resumes on the *next* tick (poll_ms is 60ms by
        # default -- close to the old 0.05s pause). Documented deliberate
        # simplification, not a silent behaviour change.
        crossed_az = (
            self._track_prev_az_err is not None
            and sign(self._track_prev_az_err) != 0
            and sign(az_err) != 0
            and sign(self._track_prev_az_err) != sign(az_err)
        )
        crossed_el = (
            self._track_prev_el_err is not None
            and sign(self._track_prev_el_err) != 0
            and sign(el_err) != 0
            and sign(self._track_prev_el_err) != sign(el_err)
        )
        if (crossed_az and abs(az_err) < 3.0) or (crossed_el and abs(el_err) < 2.0):
            actions.append(Stop())
            note = f"overshoot brake az={az_err:.3f} el={el_err:.3f}"
            self._track_prev_az_err = az_err
            self._track_prev_el_err = el_err
            return actions, note, False

        # node:770-774 -- closed-loop speed command, with the near-target
        # elevation speed cap.
        az_cmd = choose_speed(az_err, AZ_PROFILE, self._cfg.az_tol)
        el_cmd = choose_speed(el_err, EL_PROFILE, self._cfg.el_tol)
        if abs(el_err) < 1.2 and el_cmd != 0.0:
            el_cmd = sign(el_cmd) * min(abs(el_cmd), 1.4)

        actions.append(SetSpeed(az_cmd, el_cmd))
        note = f"az_err={az_err:.2f} cmd={az_cmd:.2f} el_err={el_err:.2f} cmd={el_cmd:.2f}"

        self._track_prev_az_err = az_err
        self._track_prev_el_err = el_err
        return actions, note, False

    # -- VELOCITY (direct joystick/manual speed) -------------------------
    # Ported from node:840-926 (_joy_stick_control_mode). Note this mode
    # never reads ``sample`` at all -- old code drove purely off the last
    # commanded target and a clock, with no camera feedback.

    def _enter_velocity(self, az_speed: float, el_speed: float, now: float) -> None:
        self._vel_target = (az_speed, el_speed)
        self._vel_target_at = now
        self._vel_last_sent = None

    def _retarget_velocity(self, az_speed: float, el_speed: float, now: float) -> None:
        """node:445-447 -- unlike TRACK, every incoming velocity command
        refreshes the idle clock unconditionally, even if the speed values
        are unchanged."""
        self._vel_target = (az_speed, el_speed)
        self._vel_target_at = now

    def _tick_velocity(self, sample: Optional[PtSample], now: float):
        actions: list = []
        notes: list[str] = []

        # node:879-882 -- idle timeout (strict >, matching the old code).
        idle = now - self._vel_target_at
        if idle > self._cfg.velocity_idle_s:
            actions.append(Stop())
            self._transition(Mode.IDLE)
            return actions, "velocity idle timeout -- stop", True

        # node:884-890 -- clamp to +-max_speed, then 0.01 deadband to 0.0.
        az_speed = clamp(float(self._vel_target[0]), -AZ_PROFILE.max_speed, AZ_PROFILE.max_speed)
        el_speed = clamp(float(self._vel_target[1]), -EL_PROFILE.max_speed, EL_PROFILE.max_speed)
        if abs(az_speed) < 0.01:
            az_speed = 0.0
        if abs(el_speed) < 0.01:
            el_speed = 0.0

        # node:892-907 -- only resend when the clamped/deadbanded value
        # actually changes.
        if self._vel_last_sent != (az_speed, el_speed):
            actions.append(SetSpeed(az_speed, el_speed))
            self._vel_last_sent = (az_speed, el_speed)
            notes.append(f"velocity az={az_speed:.2f} el={el_speed:.2f}")

        # node:909-910 -- explicit zero speed exits immediately, on any
        # tick (including the very first), regardless of the idle clock.
        if az_speed == 0.0 and el_speed == 0.0:
            self._transition(Mode.IDLE)
            notes.append("zero speed -- exiting")
            return actions, "; ".join(notes), True

        return actions, "; ".join(notes), False

    # -- SCAN --------------------------------------------------------------
    # Ported from node:1014-1196 (_run_scan / _wait_scan_stopped /
    # _wait_scan_started), reshaped into six explicit sub-states, one
    # advanced per tick, replacing the old 150-line coroutine and its seven
    # ``_check_scan_seq`` call sites. Preemption at any sub-state falls out
    # for free: ``_dispatch_intent`` only ever looks at ``self._mode``, not
    # at ``self._scan_sub``, so a goto/track/velocity/home/stop intent
    # exits SCAN (emitting ScanOff -- see ``_exit_actions``) no matter which
    # of the six sub-states scanning is currently in.

    def _enter_scan(
        self, center_az: float, each_side: float, elevation: float, speed: float, now: float
    ) -> None:
        self._scan_center_az = center_az
        self._scan_each_side = each_side
        self._scan_elevation = elevation
        self._scan_speed = speed
        self._scan_left = normalize_az(center_az - each_side)
        self._scan_right = normalize_az(center_az + each_side)
        self._scan_sub = _ScanSub.STOP_PREV
        self._scan_wait_until = now
        self._scan_deadline = now + self._cfg.stop_scan_timeout_s
        self._scan_confirm_deadline = None

    def _advance_to_preposition(self, now: float):
        self._scan_sub = _ScanSub.PREPOSITION
        self._scan_deadline = now + _SCAN_PREPOSITION_TIMEOUT_S
        note = f"scan: prepositioning to az={self._scan_center_az:.2f} el={self._scan_elevation:.2f}"
        return [Goto(self._scan_center_az, self._scan_elevation)], note

    def _advance_to_set_speed(self, now: float):
        self._scan_sub = _ScanSub.SET_SPEED
        self._scan_wait_until = now + self._cfg.scan_save_wait_s
        return [ScanSetSpeed(self._scan_speed)], f"scan: set speed={self._scan_speed:.2f}"

    def _advance_to_set_limits(self, now: float):
        self._scan_sub = _ScanSub.SET_LIMITS
        self._scan_wait_until = now + _SCAN_SET_LIMITS_WAIT_S
        note = f"scan: set limits {self._scan_left:.2f}..{self._scan_right:.2f}"
        return [ScanSetLimits(self._scan_left, self._scan_right)], note

    def _advance_to_on(self, now: float):
        self._scan_sub = _ScanSub.ON
        self._scan_wait_until = now + self._cfg.scan_start_wait_s
        self._scan_confirm_deadline = None
        return [ScanOn()], "scan: on"

    def _tick_scan(self, sample: Optional[PtSample], now: float):
        sub = self._scan_sub
        actions: list = []
        note = ""

        if sub == _ScanSub.STOP_PREV:
            # node:1168-1181 (_wait_scan_stopped): Mode==0 and both speeds
            # <= 0.01, or stop_scan_timeout_s elapses (failure is ignored,
            # old code proceeds anyway).
            stopped = (
                sample is not None
                and sample.mode == 0
                and abs(sample.speed_x) <= 0.01
                and abs(sample.speed_y) <= 0.01
            )
            if stopped or now >= self._scan_deadline:
                actions, note = self._advance_to_preposition(now)
            return actions, note, False

        if sub == _ScanSub.PREPOSITION:
            # node:1056-1071 -- only elevation (and speed) are checked for
            # arrival, azimuth is deliberately not part of this check in
            # the old code. 15s timeout, then proceed anyway.
            reached = False
            if sample is not None:
                el_err = abs(sample.abs_el - self._scan_elevation)
                reached = el_err <= self._cfg.el_tol and sample.speed_mag < 0.1
            if reached or now >= self._scan_deadline:
                actions, note = self._advance_to_set_speed(now)
            return actions, note, False

        if sub == _ScanSub.SET_SPEED:
            if now >= self._scan_wait_until:
                actions, note = self._advance_to_set_limits(now)
            return actions, note, False

        if sub == _ScanSub.SET_LIMITS:
            if now >= self._scan_wait_until:
                actions, note = self._advance_to_on(now)
            return actions, note, False

        if sub == _ScanSub.ON:
            # node:1102-1106 -- fixed scan_start_wait_s pause after sending
            # ScanOn, then up to 3.0s confirming Mode!=0 or speed>0.01
            # (timeout is not a failure -- old code proceeds regardless).
            # Old ``_wait_scan_started`` reads a fresh sample on its very
            # first iteration with no extra sleep, so the tick that ends
            # the fixed wait also evaluates confirmation immediately,
            # rather than deferring the first confirmation check to the
            # following tick.
            if self._scan_confirm_deadline is None:
                if now < self._scan_wait_until:
                    return actions, note, False
                self._scan_confirm_deadline = now + _SCAN_CONFIRM_TIMEOUT_S
                note = "scan: confirming active"
            confirmed = sample is not None and (
                sample.mode != 0 or abs(sample.speed_x) > 0.01 or abs(sample.speed_y) > 0.01
            )
            if confirmed or now >= self._scan_confirm_deadline:
                self._scan_sub = _ScanSub.ACTIVE
                note = "scan: active"
            return actions, note, False

        if sub == _ScanSub.ACTIVE:
            # node:1122-1136 -- external interrupt: Mode==0 and both speeds
            # <= 0.1 while we believe we are scanning. The old code does
            # NOT call scan_mode_off() again here (the camera already
            # stopped scanning by itself), it just gives up and marks
            # scanning off -- so no ScanOff action is emitted on this path,
            # unlike a normal preempting transition.
            interrupted = (
                sample is not None
                and sample.mode == 0
                and abs(sample.speed_x) <= 0.1
                and abs(sample.speed_y) <= 0.1
            )
            if interrupted:
                self._scan_sub = None
                self._transition(Mode.IDLE)
                return actions, "scan: externally interrupted", True
            return actions, note, False

        return actions, note, False
