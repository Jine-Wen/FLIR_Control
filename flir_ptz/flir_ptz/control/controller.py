#!/usr/bin/env python3
"""The single tick loop that unifies telemetry and control (spec
ARCHITECTURE.md sec. 4.2) -- the heart of this rewrite.

    while running:
        intent = take_pending_intent()          # lock-free swap, may trigger FSM transition
        sample = await session.read_nmea()      # ALWAYS fresh: read and control on the same tick
        publish_snapshot(sample, fsm.mode, fsm.seq)
        result = fsm.step(sample, now)          # pure
        for action in result.actions:
            await session.execute(action)
        await wait_for(wakeup_event, timeout=result.tick_period)

This replaces the old design's background telemetry poller *plus* separate
per-command asyncio tasks contending on a single semaphore -- reads and
control commands stole channel slots from each other, and the closed-loop
TRACK logic acted on stale positions (see ``flir_ptz_node.py:_poll_forever``
/ ``_track_abs`` / ``_joy_stick_control_mode`` / ``_run_scan``, all racing
on ``FlirNexusClient._request_sem``). Here there is exactly one reader/
writer of the camera per tick, and preemption is *structural*: a ROS
callback just replaces the FSM's pending intent (see
``MotionFSM.submit``) and nudges ``wakeup_event`` -- there is nothing to
cancel (spec sec. 8 rule 2: the control plane must never call the task
cancellation API anywhere). The next tick's ``fsm.step()`` sees the new intent, runs
the mode transition table, and emits the correct exit action(s) itself.

Two things ``MotionFSM.step()`` structurally cannot do (its signature is
``step(sample, now)`` -- pure, no I/O, no channel for command outcomes),
and which this module exists to bridge:

  * PARITY A19 (scan aborts when the control token is lost): ``session.
    execute()`` raises :class:`~flir_ptz.nexus.session.TokenLost` --
    caught here, turned into a submitted ``Intent.stop()`` for the next
    tick (which the FSM's ordinary transition table turns into IDLE +
    the right exit action), and logged once. See ``_handle_token_lost``.
  * Token keepalive/release: driven every tick via a controller-owned
    ``TokenTracker.periodic(now, mode_active)`` (see "Token tracking"
    below) -- not a separate timer task.

Also implements the GEO/ABS coordinate-frame fix documented in the
worker brief: ``PTGeoAzimuthElevationSet`` commands the camera in the
GEO frame, but ``MotionFSM``'s GOTO/HOMING arrival & stall-detection
logic (ported bit-for-bit from the old code, see ``fsm.py``) reads
``sample.abs_az`` / ``sample.abs_el``. Those coincide only when the
platform's heading offset is zero. Rather than editing ``fsm.py`` (owned
by another worker) or silently flipping the default (spec: no hardware
to verify against), this module offers a ``goto_feedback_frame``
parameter -- default ``"abs"``, bit-for-bit today's behaviour -- and,
when set to ``"geo"``, feeds the FSM a *remapped* ``PtSample`` (only
while ``fsm.mode`` is GOTO or HOMING -- see ``_frame_sample_for_fsm``)
with ``abs_az``/``abs_el`` replaced by the sample's own ``geo_az``/
``geo_el``. ``publish_snapshot`` still always receives the true,
unmodified sample. A persistent divergence between the two frames is
also monitored and logged once (see ``_check_frame_divergence``).

Token tracking design note (why a *second* ``TokenTracker``): spec sec.
4.1 says the control plane must never see raw Nexus token errors -- and
indeed ``CameraSession`` keeps its own ``TokenTracker`` entirely private
(there is no public accessor beyond the read-only ``token_state``
property, and no ``periodic()`` is ever called from inside
``session.py``). Since this module owns neither ``session.py`` nor
``token.py`` and must not edit them, it drives keepalive/release by
instantiating its own :class:`~flir_ptz.nexus.token.TokenTracker`
purely for that *periodic* decision, synchronising its belief state from
``session.token_state`` every tick (that property is the ground truth --
it is updated from real Nexus Return Codes inside ``execute()``) and
feeding it ``note_command(now)`` whenever this tick successfully sent a
camera-affecting action. This reuses the exact same, already-tested
policy state machine (spec requirement) without needing any new surface
on ``CameraSession``.
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import dataclasses
import logging
import time
from typing import Awaitable, Callable, Optional

from flir_ptz.control.config import ControlConfig
from flir_ptz.control.fsm import (
    Action,
    Goto,
    Intent,
    MotionFSM,
    Mode,
    ScanOff,
    ScanOn,
    ScanSetLimits,
    ScanSetSpeed,
    SetSpeed,
    Stop,
    StepResult,
)
from flir_ptz.control.profiles import az_error
from flir_ptz.nexus.protocol import DltvSample, PtSample
from flir_ptz.nexus.session import CameraSession, TokenLost
from flir_ptz.nexus.token import TokenAction, TokenPolicy, TokenTracker

# -- a tiny logger-duck-type fallback ----------------------------------------
#
# ptz_node.py passes its rclpy node logger (``.info`` / ``.warn`` /
# ``.error`` / ``.debug``); tests and any other caller that doesn't care
# get this stdlib-backed default so the module never requires rclpy.


class _StdlibLoggerAdapter:
    def __init__(self, logger: logging.Logger) -> None:
        self._logger = logger

    def info(self, msg: str) -> None:
        self._logger.info(msg)

    def warn(self, msg: str) -> None:
        self._logger.warning(msg)

    def error(self, msg: str) -> None:
        self._logger.error(msg)

    def debug(self, msg: str) -> None:
        self._logger.debug(msg)


def _default_logger() -> _StdlibLoggerAdapter:
    return _StdlibLoggerAdapter(logging.getLogger("flir_ptz.controller"))


# Threshold + persistence window for the one-time GEO/ABS divergence
# warning (worker brief: "exceeds a threshold (say 1.0 deg) persistently").
_FRAME_DIVERGENCE_THRESHOLD_DEG = 1.0
_FRAME_DIVERGENCE_PERSIST_S = 3.0

# -- EO zoom: orthogonal to MotionFSM, driven straight from the tick -------
#
# Zoom is deliberately NOT a MotionFSM mode (it must never disturb whatever
# pan/tilt mode is currently running) and deliberately NOT executed from a
# ROS callback thread (the tick loop is the only thing allowed to touch the
# session -- see module docstring). ``submit_zoom()`` just records the
# latest requested direction; ``_tick()`` drains it every tick, exactly like
# ``MotionFSM.submit()``/``step()`` does for pan/tilt, but entirely outside
# the FSM.
#
# Because DLTVZoomCountsIncrement/Decrement are CONTINUOUS -- the lens keeps
# moving until DLTVZoomStop arrives (measured on a real 364C) -- a dead-man
# timer is mandatory: while zooming, if no further "in"/"out" request
# renews it within ``ZOOM_DEADMAN_S``, the tick issues DLTVZoomStop on its
# own. A dropped pointerup, a closed browser tab, or a network blip must
# never leave the lens driving to its mechanical limit unattended.
ZOOM_DEADMAN_S = 1.0

# DLTVLastNMEAGet shares the same single serial CGI channel as PTLastNMEAGet
# (read every tick already) -- polling it that often would just steal
# channel slots from pan/tilt for no benefit. Poll on this slow cadence
# while idle instead, and force one extra read right after a zoom command so
# the UI reflects a change quickly (see ``_maybe_poll_dltv``).
ZOOM_POLL_IDLE_S = 2.0


class TickController:
    """Owns the single read-then-control tick loop for one connected
    :class:`CameraSession`. No thread, no rclpy: ``run()`` is a plain
    coroutine, driven by whatever event loop the caller schedules it on
    (a ROS node's background asyncio thread in production, plain
    ``asyncio.run()`` in tests -- see ``test/test_controller.py``, which
    drives this class directly against a small in-process, pure-stdlib
    kinematic fake (no HTTP, no sockets -- the same ``client_factory``
    injection seam ``test/test_session.py`` uses) with no ROS involved
    at all).
    """

    def __init__(
        self,
        session: CameraSession,
        fsm: MotionFSM,
        cfg: ControlConfig,
        *,
        goto_feedback_frame: str = "abs",
        home_on_shutdown: str = "auto",   # "auto" | "always" | "never"
        reconnect_after_failures: int = 10,
        token_policy: Optional[TokenPolicy] = None,
        on_snapshot: Optional[Callable[[Optional[PtSample], StepResult], None]] = None,
        logger: object = None,
        clock: Callable[[], float] = time.monotonic,
        zoom_deadman_s: float = ZOOM_DEADMAN_S,
        zoom_poll_idle_s: float = ZOOM_POLL_IDLE_S,
    ) -> None:
        if goto_feedback_frame not in ("abs", "geo"):
            raise ValueError(f"goto_feedback_frame must be 'abs' or 'geo', got {goto_feedback_frame!r}")

        self._session = session
        self._fsm = fsm
        self._cfg = cfg
        self._goto_feedback_frame = goto_feedback_frame
        self._home_on_shutdown = (home_on_shutdown or "auto").lower()
        self._reconnect_after_failures = max(1, int(reconnect_after_failures))
        self._on_snapshot = on_snapshot
        self._log = logger if logger is not None else _default_logger()
        self._clock = clock

        # Controller-local token tracker driving periodic() only -- see
        # module docstring "Token tracking design note". Synced from
        # ``session.token_state`` every tick, never the source of truth.
        self._token_tracker = TokenTracker(token_policy or TokenPolicy())

        # -- loop/thread plumbing --------------------------------------
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._wakeup: Optional[asyncio.Event] = None
        self._running: bool = False

        # -- read-failure / reconnect bookkeeping ------------------------
        self._consecutive_read_failures: int = 0

        # -- GEO/ABS divergence one-time warning --------------------------
        self._divergence_since: Optional[float] = None
        self._divergence_warned: bool = False

        # -- shutdown homing coordination ---------------------------------
        self._home_done_event: Optional[asyncio.Event] = None

        # -- background keepalive/release tasks ---------------------------
        # Held onto (never cancelled -- see ``_schedule_fire_and_forget``)
        # purely so a running task can't be garbage-collected mid-flight;
        # discarded via a completion callback once done.
        self._background_tasks: set = set()

        # -- EO zoom: orthogonal to the FSM (see module docstring) --------
        self._zoom_deadman_s = max(0.0, float(zoom_deadman_s))
        self._zoom_poll_idle_s = max(0.0, float(zoom_poll_idle_s))
        self._pending_zoom: Optional[str] = None    # drained once per tick
        self._zoom_active: bool = False              # lens believed moving
        self._zoom_deadline: Optional[float] = None  # dead-man expiry
        self._dltv_sample: Optional[DltvSample] = None
        self._last_dltv_poll_at: Optional[float] = None
        # Force one DLTV read on the first opportunity so the UI has a
        # value to show without having to wait out the idle cadence.
        self._dltv_poll_due: bool = True

    # -- properties ------------------------------------------------------

    @property
    def running(self) -> bool:
        return self._running

    @property
    def fsm(self) -> MotionFSM:
        return self._fsm

    @property
    def zoom_state(self) -> Optional[DltvSample]:
        """The most recent DLTVLastNMEAGet reading, or ``None`` before the
        first successful poll. Read by the ROS node to populate
        ``PtzState.zoom_pctg``/``zoom_mm`` -- see ``_maybe_poll_dltv`` for
        the polling cadence."""
        return self._dltv_sample

    # -- submitting intents (the only thing ROS callbacks touch) --------

    def submit(self, intent: Intent) -> None:
        """Submit an intent from *the loop this controller is running
        on*. Also nudges the wakeup event so a pending Stop lands as fast
        as the in-flight HTTP call allows, instead of waiting out the
        tick period. Safe to call before ``run()`` starts (queued into
        the FSM directly; the wakeup event simply doesn't exist yet)."""
        self._fsm.submit(intent, self._clock())
        if self._wakeup is not None:
            self._wakeup.set()

    def submit_threadsafe(self, intent: Intent) -> None:
        """Submit an intent from *any other* thread -- this is what ROS
        subscription callbacks call (they run on rclpy's executor
        thread, not the controller's asyncio loop thread). Per the
        worker brief: ``loop.call_soon_threadsafe(controller.submit,
        intent)``."""
        if self._loop is None:
            # Loop not started yet (e.g. a command arrives before the
            # background thread is up) -- best-effort direct submit.
            self.submit(intent)
            return
        self._loop.call_soon_threadsafe(self.submit, intent)

    def submit_zoom(self, direction: str) -> None:
        """Record a pending EO zoom request for the next tick to drain.

        Thread-safe -- unlike :meth:`submit`, this may be called from *any*
        thread, including a ROS subscription callback (mirrors
        :meth:`submit_threadsafe`). It never touches the camera itself: it
        only ever writes a small piece of controller-local state plus the
        wakeup event, which is exactly what makes it safe to call from
        off-loop -- the tick loop remains the only thing that ever talks to
        ``session`` (see module docstring). Deliberately bypasses
        ``MotionFSM`` entirely: zoom must never become a mode and must never
        disturb whichever pan/tilt mode is currently running.
        """
        if direction not in ("in", "out", "stop"):
            raise ValueError(f"zoom direction must be 'in', 'out', or 'stop', got {direction!r}")
        if self._loop is None:
            self._set_pending_zoom(direction)
            return
        self._loop.call_soon_threadsafe(self._set_pending_zoom, direction)

    def _set_pending_zoom(self, direction: str) -> None:
        self._pending_zoom = direction
        if self._wakeup is not None:
            self._wakeup.set()

    # -- run loop ----------------------------------------------------------

    async def run(self) -> None:
        """The single tick loop (spec sec. 4.2). Runs until
        ``request_stop()`` is called (from any thread) or an unrecoverable
        error escapes (there should be none in normal operation -- every
        expected failure mode is caught per-tick)."""
        self._loop = asyncio.get_running_loop()
        self._wakeup = asyncio.Event()
        self._running = True
        try:
            while self._running:
                await self._tick()
        finally:
            self._running = False

    def request_stop(self) -> None:
        """Ask the tick loop to exit after its current tick. Safe to call
        from any thread."""
        if self._loop is None:
            self._running = False
            return
        self._loop.call_soon_threadsafe(self._request_stop_locked)

    def _request_stop_locked(self) -> None:
        self._running = False
        if self._wakeup is not None:
            self._wakeup.set()

    # -- one tick ------------------------------------------------------

    async def _tick(self) -> None:
        # Cleared here, at the very start of the tick -- *before* any of
        # this tick's I/O -- rather than immediately before the wait at
        # the bottom. A ROS callback's submit() (see `submit`) can land
        # at any point while this tick is busy awaiting the camera; if
        # the event were cleared right before the wait instead, a
        # submit() that arrived during this tick's I/O would be wiped
        # out by that late clear() before ever being waited on -- a lost
        # wakeup that would silently cost a full extra tick_period of
        # latency. Clearing up front means any set() that happens during
        # this tick's own I/O survives to make the bottom's wait_for
        # return immediately instead of sleeping out tick_period.
        assert self._wakeup is not None
        self._wakeup.clear()

        now = self._clock()
        mode_before = self._fsm.mode

        sample = await self._read_sample(now)
        self._check_frame_divergence(sample, now)

        fsm_sample = self._frame_sample_for_fsm(sample, mode_before)
        result = self._fsm.step(fsm_sample, now)

        self._publish_snapshot(sample, result)

        await self._execute_actions(result.actions, now)

        if (
            self._home_done_event is not None
            and mode_before == Mode.HOMING
            and result.mode != Mode.HOMING
        ):
            self._home_done_event.set()

        # EO zoom: fully orthogonal to the FSM step above -- deliberately
        # not routed through result.actions/MotionFSM (see module
        # docstring's "EO zoom" section).
        await self._drain_zoom(now)

        self._run_token_periodic(now)

        await self._wait_for_next_tick(result.tick_period)

    async def _read_sample(self, now: float) -> Optional[PtSample]:
        try:
            sample = await self._session.read_nmea()
        except TokenLost as exc:
            self._handle_token_lost(exc, now)
            sample = None
        except Exception as exc:  # NexusError, ConnectionFailure, SessionError, ...
            self._consecutive_read_failures += 1
            self._log.warn(
                f"[flir_ptz] read_nmea failed "
                f"(consecutive={self._consecutive_read_failures}): {exc}"
            )
            if self._consecutive_read_failures >= self._reconnect_after_failures:
                await self._attempt_reconnect()
            return None
        else:
            self._consecutive_read_failures = 0
            return sample

    async def _attempt_reconnect(self) -> None:
        self._log.warn(
            f"[flir_ptz] {self._consecutive_read_failures} consecutive read failures "
            "-- attempting reconnect"
        )
        try:
            await self._session.connect()
            self._log.info("[flir_ptz] reconnect succeeded")
        except Exception as exc:
            self._log.error(f"[flir_ptz] reconnect failed: {exc}")
        finally:
            # Whether or not it worked, don't spin reconnect attempts on
            # every subsequent failed tick -- reset and let the failure
            # counter build back up to the threshold again.
            self._consecutive_read_failures = 0

    def _frame_sample_for_fsm(
        self, sample: Optional[PtSample], mode_before: Mode
    ) -> Optional[PtSample]:
        """Coordinate-frame fix (see module docstring). Only remaps
        abs_az/abs_el -> geo_az/geo_el, and only while GOTO/HOMING is the
        mode that will actually run its tick logic this call (checked via
        the *pre-step* mode: GOTO/HOMING entry never runs ``_run_tick`` on
        the same step that dispatches the intent -- see fsm.py's
        ``_dispatch_intent`` docstring -- so by the time ``_tick_move``
        ever reads ``sample.abs_*``, ``fsm.mode`` has already been GOTO/
        HOMING since the *previous* tick, which is exactly what this
        pre-step check observes)."""
        if sample is None or self._goto_feedback_frame != "geo":
            return sample
        if mode_before not in (Mode.GOTO, Mode.HOMING):
            return sample
        return dataclasses.replace(sample, abs_az=sample.geo_az, abs_el=sample.geo_el)

    def _check_frame_divergence(self, sample: Optional[PtSample], now: float) -> None:
        """One-time warning when GEO and ABS readings persistently
        disagree by more than ``_FRAME_DIVERGENCE_THRESHOLD_DEG`` -- see
        the "COORDINATE-FRAME ISSUE" section of the worker brief. Runs
        regardless of mode/``goto_feedback_frame`` setting so an
        installation running the (default) "abs" setting still gets
        warned that "geo" is probably what it wants."""
        if self._divergence_warned or sample is None:
            return

        az_diff = abs(az_error(sample.geo_az, sample.abs_az))
        el_diff = abs(sample.geo_el - sample.abs_el)
        diverged = az_diff > _FRAME_DIVERGENCE_THRESHOLD_DEG or el_diff > _FRAME_DIVERGENCE_THRESHOLD_DEG

        if not diverged:
            self._divergence_since = None
            return

        if self._divergence_since is None:
            self._divergence_since = now
            return

        if now - self._divergence_since >= _FRAME_DIVERGENCE_PERSIST_S:
            self._divergence_warned = True
            self._log.warn(
                "[flir_ptz] GEO and ABS azimuth/elevation readings have diverged "
                f"persistently (az diff={az_diff:.2f} deg, el diff={el_diff:.2f} deg). "
                "goto/home arrival detection uses the 'abs' frame by default, which "
                "will not match a GEO-frame PTGeoAzimuthElevationSet command on a "
                "platform with a non-zero heading offset. If goto/home commands "
                "appear to move the camera correctly but are reported as stalled, "
                "set the 'goto_feedback_frame' parameter to 'geo' for this "
                "installation. (This warning is logged once.)"
            )

    async def _execute_actions(self, actions: tuple, now: float) -> None:
        for action in actions:
            try:
                await self._execute_action(action)
            except TokenLost as exc:
                self._handle_token_lost(exc, now)
                return
            except Exception as exc:
                self._log.warn(f"[flir_ptz] action {action!r} failed: {exc}")
                return
            else:
                self._token_tracker.note_command(now)

    async def _execute_action(self, action: Action) -> None:
        if isinstance(action, SetSpeed):
            await self._session.set_speed(action.az, action.el)
        elif isinstance(action, Goto):
            await self._session.goto_position(action.az, action.el)
        elif isinstance(action, Stop):
            await self._session.stop()
        elif isinstance(action, ScanSetSpeed):
            await self._session.scan_set_speed(action.speed)
        elif isinstance(action, ScanSetLimits):
            await self._session.scan_set_limits(action.left, action.right)
        elif isinstance(action, ScanOn):
            await self._session.scan_mode_on()
        elif isinstance(action, ScanOff):
            await self._session.scan_mode_off()
        else:  # pragma: no cover - defensive; fsm.py's Action union is closed
            raise TypeError(f"unknown control action: {action!r}")

    def _handle_token_lost(self, exc: TokenLost, now: float) -> None:
        """PARITY A19: graceful degrade instead of crashing or silently
        hanging. ``MotionFSM.step()`` has no channel for command-outcome
        errors (see module docstring), so the bridge is: submit a plain
        ``stop`` intent for the *next* tick. The FSM's own transition
        table (run purely from ``self._mode`` -- see ``fsm.py``'s
        ``_dispatch_intent``) then emits the correct exit action(s)
        (``Stop`` and/or ``ScanOff``) and flips ``mode`` back to IDLE on
        its own, independent of whether that exit action itself manages
        to reach the camera (which it may well not, if the token is
        genuinely gone) -- so this degrade is guaranteed to reach IDLE
        even under a persistent token loss."""
        self._log.warn(f"[flir_ptz] control token lost during command: {exc} -- degrading to IDLE")
        self._fsm.submit(Intent.stop(source="system:token_lost"), now)
        if self._wakeup is not None:
            self._wakeup.set()

    # -- EO zoom: drain pending request + dead-man timer + slow DLTV poll --

    async def _drain_zoom(self, now: float) -> None:
        """Once per tick: send whatever zoom direction was requested since
        the last tick, or -- if nothing new arrived and the lens is still
        believed to be moving -- check the dead-man timer. Then (cheaply)
        decide whether a DLTV poll is due. Never touches ``self._fsm``:
        this is the whole point of keeping zoom orthogonal to pan/tilt."""
        direction = self._pending_zoom
        self._pending_zoom = None

        if direction is not None:
            await self._apply_zoom_command(direction, now)
        elif self._zoom_active and self._zoom_deadline is not None and now >= self._zoom_deadline:
            # MANDATORY SAFETY: no renewing "in"/"out" arrived in time --
            # a dropped pointerup, a closed tab, a network blip -- stop the
            # lens ourselves rather than let it drive to its mechanical
            # limit unattended.
            self._log.warn(
                f"[flir_ptz] EO zoom dead-man timer expired ({self._zoom_deadman_s:.1f}s "
                "without renewal) -- stopping zoom"
            )
            await self._apply_zoom_command("stop", now)

        await self._maybe_poll_dltv(now)

    async def _apply_zoom_command(self, direction: str, now: float) -> None:
        try:
            if direction == "in":
                await self._session.zoom_in()
            elif direction == "out":
                await self._session.zoom_out()
            else:  # "stop" -- always forwarded, even if we didn't think we were zooming
                await self._session.zoom_stop()
        except TokenLost as exc:
            # Deliberately NOT the same bridge as _handle_token_lost: that
            # submits an FSM Intent, and zoom must never touch fsm.mode.
            # The camera-side lens state is now unknown to us; the safest
            # local belief is "not actively renewing", so a stale dead-man
            # timer can't fire a pointless extra stop later.
            self._log.warn(f"[flir_ptz] EO zoom {direction!r} failed (token lost): {exc}")
            self._zoom_active = False
            self._zoom_deadline = None
            return
        except Exception as exc:
            self._log.warn(f"[flir_ptz] EO zoom {direction!r} failed: {exc}")
            return

        self._token_tracker.note_command(now)
        if direction == "stop":
            self._zoom_active = False
            self._zoom_deadline = None
        else:
            self._zoom_active = True
            self._zoom_deadline = now + self._zoom_deadman_s
        # Refresh the DLTV reading promptly so the UI reflects the change
        # quickly, instead of waiting out the idle poll cadence.
        self._dltv_poll_due = True

    async def _maybe_poll_dltv(self, now: float) -> None:
        due = (
            self._dltv_poll_due
            or self._last_dltv_poll_at is None
            or (now - self._last_dltv_poll_at) >= self._zoom_poll_idle_s
        )
        if not due:
            return
        self._dltv_poll_due = False
        self._last_dltv_poll_at = now
        try:
            sample = await self._session.read_dltv()
        except Exception as exc:  # TokenLost can't occur here (read-only), but never risk the tick
            self._log.warn(f"[flir_ptz] DLTV read failed: {exc}")
            return
        self._dltv_sample = sample

    def _publish_snapshot(self, sample: Optional[PtSample], result: StepResult) -> None:
        """Hand the tick's telemetry to the observer, defensively.

        ``on_snapshot`` is caller-supplied (the ROS node publishes from it).
        An exception raised there must never abort the tick loop: doing so
        takes down the whole control plane, including the shutdown homing
        sequence, over what is only an observability concern. Observed for
        real on SIGTERM, where rclpy invalidates the publisher context while
        the camera still needs to be parked.
        """
        if self._on_snapshot is None:
            return
        try:
            self._on_snapshot(sample, result)
        except Exception as exc:  # pragma: no cover - defensive
            self._log.warn(f"[controller] snapshot observer raised (ignored): {exc}")

    def _run_token_periodic(self, now: float) -> None:
        """Drive keepalive/release from the tick -- see module docstring
        "Token tracking design note". No separate timer task."""
        self._token_tracker.state = self._session.token_state
        mode_active = self._fsm.mode != Mode.IDLE
        action = self._token_tracker.periodic(now, mode_active)
        if action == TokenAction.KEEPALIVE:
            self._schedule_fire_and_forget(self._keepalive())
        elif action == TokenAction.RELEASE:
            self._schedule_fire_and_forget(self._release())
        # NONE / ACQUIRE / REVIVE / RELOGIN: ACQUIRE/REVIVE/RELOGIN are
        # never returned by periodic() (only by required_before/
        # observe_rc, which session.execute() already handles
        # internally) -- nothing to do here.

    def _schedule_fire_and_forget(self, coro: Awaitable[None]) -> None:
        """Run a best-effort background coroutine (keepalive/release)
        without blocking this tick and without ever needing to cancel it
        (spec sec. 8 rule 2): it is a single bounded HTTP call that
        either finishes or raises, and any exception is swallowed by the
        wrapper coroutine itself -- there is nothing here for a future
        tick to preempt. The task is held in ``self._background_tasks``
        only to prevent it being garbage-collected while still pending
        (a well-known asyncio footgun), and is discarded -- never
        cancelled -- once it completes on its own."""
        assert self._loop is not None
        task = self._loop.create_task(coro)
        self._background_tasks.add(task)
        task.add_done_callback(self._background_tasks.discard)

    async def _keepalive(self) -> None:
        try:
            await self._session.keepalive_token()
        except Exception as exc:
            self._log.warn(f"[flir_ptz] token keepalive failed: {exc}")

    async def _release(self) -> None:
        try:
            await self._session.release_control()
        except Exception as exc:
            self._log.warn(f"[flir_ptz] token release failed: {exc}")

    async def _wait_for_next_tick(self, tick_period: float) -> None:
        # Deliberately no ``self._wakeup.clear()`` here -- see the comment
        # at the top of ``_tick()`` for why it must happen there instead.
        assert self._wakeup is not None
        try:
            await asyncio.wait_for(self._wakeup.wait(), timeout=max(0.0, tick_period))
        except asyncio.TimeoutError:
            pass

    # -- shutdown: HOMING-as-a-mode + logout (spec sec. 4.2, PARITY A26/A27) --

    def request_home(self, timeout: float = 25.0) -> "concurrent.futures.Future[bool]":
        """Submit ``Intent.home()`` to the still-running loop and return a
        ``concurrent.futures.Future`` that resolves once the FSM reports
        the homing attempt finished (``StepResult.done`` with
        ``mode == Mode.IDLE`` -- whether it arrived or gave up on its own
        ``cfg.home_timeout_s`` deadline) or ``timeout`` seconds elapse,
        whichever comes first. Resolves to ``False`` immediately, without
        submitting anything, if we do not currently hold the control
        token (PARITY A26: only home while we actually hold it).

        Safe to call from any thread (this is the replacement for the old
        ~120-line synchronous ``_sync_home`` / ``_check_we_have_control_
        sync`` -- see module docstring)."""
        if self._loop is None or not self._running:
            fut: concurrent.futures.Future = concurrent.futures.Future()
            fut.set_result(False)
            return fut
        return asyncio.run_coroutine_threadsafe(self._home_coro(timeout), self._loop)

    async def _acquire_token_for_homing(self) -> bool:
        """Decide whether we may take the control token in order to park.

        Why this exists: the old node force-claimed the token every 10 s
        regardless of activity, so at shutdown it essentially always held it
        and therefore essentially always homed. Removing that force-claim (it
        stole control from a physical JCU operator every 10 s) is correct, but
        it silently changed shutdown behaviour too -- an idle node no longer
        holds the token, so the camera would stop being parked on exit.

        ``home_on_shutdown`` restores the practical behaviour without
        restoring the rudeness:

        * ``"auto"`` (default) -- take the token only if nobody else holds it.
          A free or already-ours token means no operator is driving, so
          parking is safe. If another session holds it, that is a live
          operator: leave them alone and skip homing.
        * ``"always"`` -- force-claim and park regardless. For rigs where the
          camera must always end up stowed.
        * ``"never"`` -- never park on shutdown.
        """
        mode = self._home_on_shutdown
        if mode == "never":
            return False
        if mode == "always":
            try:
                await self._session.ensure_control()
                return True
            except Exception as exc:
                self._log.warn(f"[flir_ptz] shutdown: could not force control token ({exc})")
                return False

        # "auto": only claim a token nobody else owns.
        try:
            status = await self._session.server_status()
        except Exception as exc:
            self._log.warn(f"[flir_ptz] shutdown: token status unreadable ({exc}) -- not parking")
            return False

        token = status.get("Token_ID")
        free = token in (None, 0, "", "0")
        if not free:
            self._log.info(
                f"[flir_ptz] shutdown: control token held by session {token} "
                "(an operator is driving) -- leaving camera where it is"
            )
            return False

        self._log.info("[flir_ptz] shutdown: control token is free -- claiming it to park the camera")
        try:
            await self._session.ensure_control()
            return True
        except Exception as exc:
            self._log.warn(f"[flir_ptz] shutdown: could not claim free token ({exc})")
            return False

    async def _home_coro(self, timeout: float) -> bool:
        try:
            holding = await self._session.has_control()
        except Exception as exc:
            self._log.warn(f"[flir_ptz] shutdown: has_control check failed ({exc}) -- skipping homing")
            holding = False

        if not holding:
            holding = await self._acquire_token_for_homing()
        if not holding:
            self._log.info("[flir_ptz] shutdown: control token not ours -- skipping homing")
            return False

        self._log.info("[flir_ptz] shutdown: control token is ours -- homing camera")
        done_event = asyncio.Event()
        self._home_done_event = done_event
        self.submit(Intent.home(source="system:shutdown"))
        try:
            await asyncio.wait_for(done_event.wait(), timeout=timeout)
            arrived = self._fsm.mode == Mode.IDLE
            self._log.info("[flir_ptz] shutdown: homing complete" if arrived else "[flir_ptz] shutdown: homing did not settle cleanly")
            return arrived
        except asyncio.TimeoutError:
            self._log.warn(f"[flir_ptz] shutdown: homing did not complete within {timeout:.0f}s")
            return False
        finally:
            self._home_done_event = None

    def request_logout(self) -> "concurrent.futures.Future[None]":
        """PARITY A27: release the camera-side PHP session slot. Safe to
        call from any thread; call ``.result(timeout=...)`` to wait for
        it synchronously."""
        if self._loop is None:
            fut: concurrent.futures.Future = concurrent.futures.Future()
            fut.set_result(None)
            return fut
        return asyncio.run_coroutine_threadsafe(self._session.logout(), self._loop)
