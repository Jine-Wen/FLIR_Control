#!/usr/bin/env python3
"""Integration tests for flir_ptz.control.controller.TickController -- the
single tick loop -- driven against an in-process, pure-stdlib kinematic
fake of the Nexus wire protocol. No ROS anywhere in this file:
``controller.py`` is tested standalone, exactly as the worker brief
requires.

There is no real FLIR camera on this development machine (a real 364C is
now used for hardware-level verification instead -- see ARCHITECTURE.md),
so this file follows the same seam ``test/test_session.py`` already uses:
``CameraSession(config, client_factory=...)`` is handed a small,
hand-written async transport with no HTTP server and no sockets at all,
duck-typing only the tiny slice of ``httpx.AsyncClient`` CameraSession
actually calls (``get`` / ``post`` / ``delete`` / ``aclose``). This both
sidesteps the httpx dependency (httpx IS installed on this machine, but
these tests must not require it) *and* keeps the whole suite fast and
deterministic -- no real sockets, no thread-executor round trips.

Unlike ``test_session.py``'s ``NexusFake`` (which only needs to script
Return Codes and session bookkeeping), the tick loop under test here
actually drives motion, so ``KinematicNexusFake`` below also carries a
small kinematic model -- position integrates against commanded GOTO
targets / manual speeds / auto-scan sweeps -- the same physics an
earlier HTTP-server-based camera simulator used to provide, before a
real 364C became available for hardware-level verification. Real
kinematics, real token semantics (Token_ID / Forced=1 / RC=15), real
envelope shapes -- just no HTTP layer, since nothing here needs to
exercise the wire format itself (that is ``test_session.py``'s job).

No real IP address or password appears anywhere in this file (spec sec.
8, rule 1): the fake camera lives entirely in-process (no socket at all),
and CameraConfig's host/username/password fields are unused placeholder
strings on an RFC 2606 reserved ``.invalid`` domain.
"""

from __future__ import annotations

import asyncio
import dataclasses
import math
import os
import sys
import threading
import time
from typing import Any, Callable, Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import pytest  # noqa: E402

from flir_ptz.control.config import CameraConfig, ControlConfig  # noqa: E402
from flir_ptz.control.controller import TickController  # noqa: E402
from flir_ptz.control.fsm import Intent, Mode, MotionFSM  # noqa: E402
from flir_ptz.control.profiles import clamp, normalize_az  # noqa: E402
from flir_ptz.nexus.session import CameraSession  # noqa: E402
from flir_ptz.nexus.token import TokenPolicy  # noqa: E402

DEFAULT_USERNAME = "admin"
DEFAULT_PASSWORD = "changeme"  # noqa: S105 - obviously-fake test placeholder
_FAKE_HOST = "http://camera.example.invalid"

RC_OK = 0
RC_TOKEN_IDLE = 15
RC_NOT_AUTH = 21
RC_UNKNOWN_ACTION = 99


def _parse_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _truthy(value: Any) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


# ---------------------------------------------------------------------------
# In-process kinematic fake of the Nexus wire protocol (see module
# docstring). Follows the exact same shape as test_session.py's NexusFake,
# extended with a motion model so GOTO actually converges and SetSpeed
# actually moves the position -- the FSM's arrival/stall logic needs that
# to be meaningful.
# ---------------------------------------------------------------------------


class KinematicNexusFake:
    """A small, scriptable in-process simulation of a Nexus CGI camera,
    with real (if simplified) kinematics and control-token semantics.

    Only one client session is ever connected in these tests (one
    ``CameraSession`` per fake), so session bookkeeping is deliberately
    trivial: a single fixed ``session_id`` is handed out, and "foreign"
    control-token holders are modelled as arbitrary integers via
    :meth:`steal_token` rather than a second real login.
    """

    def __init__(
        self,
        *,
        session_id: int = 4200,
        slew_rate_deg_s: float = 30.0,
        max_speed_deg_s: float = 40.0,
        el_min: float = -90.0,
        el_max: float = 90.0,
        token_idle_timeout: float = 60.0,
        goto_tol_deg: float = 0.05,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.session_id = session_id
        self._slew_rate = float(slew_rate_deg_s)
        self._max_speed = float(max_speed_deg_s)
        self._el_min = float(el_min)
        self._el_max = float(el_max)
        self._token_idle_timeout = float(token_idle_timeout)
        self._goto_tol = float(goto_tol_deg)
        self._clock = clock

        # -- kinematic state --------------------------------------------
        self._az = 0.0
        self._el = 0.0
        self._az_speed = 0.0
        self._el_speed = 0.0
        self._goto_target: Optional[tuple] = None
        self._scan_enabled = False
        self._scan_speed = 0.0
        self._scan_left = -30.0
        self._scan_right = 30.0
        self._scan_left_uw = 0.0
        self._scan_right_uw = 0.0
        self._scan_pos = 0.0
        self._scan_dir = 1
        self._last_update = clock()

        # -- control token -------------------------------------------------
        self._token_holder: Optional[int] = None
        self._token_activity: Optional[float] = None

        # -- fault injection (spec req. 6) ---------------------------------
        self._inject_rc: Optional[int] = None
        self._inject_rc_count = 0
        self._raise_conn_error_for: dict = {}
        self._latency_s = 0.0

        self.calls: list = []

    # -- introspection for tests --------------------------------------------

    @property
    def position(self) -> tuple:
        self._advance(self._clock())
        return (self._az, self._el)

    @property
    def token_holder(self) -> Optional[int]:
        return self._token_holder

    @property
    def scanning(self) -> bool:
        return self._scan_enabled

    # -- fault injection API -------------------------------------------------

    def inject(self, rc: int, count: int = 1) -> None:
        """Make the next ``count`` Nexus calls (any action) return ``rc``."""
        self._inject_rc = int(rc)
        self._inject_rc_count = max(0, int(count))

    def inject_conn_error(self, action: str, count: int = 1) -> None:
        self._raise_conn_error_for[action] = max(0, int(count))

    def set_latency(self, ms: float) -> None:
        self._latency_s = max(0.0, float(ms)) / 1000.0

    def steal_token(self, by_session: int) -> None:
        """Simulate another operator (e.g. a physical JCU) grabbing the
        control token out from under whoever currently holds it."""
        now = self._clock()
        self._advance(now)
        self._token_holder = int(by_session)
        self._token_activity = now

    def clear_faults(self) -> None:
        self._inject_rc = None
        self._inject_rc_count = 0
        self._raise_conn_error_for = {}
        self._latency_s = 0.0

    # -- kinematics (same physics as the now-removed HTTP-server-based
    # camera simulator, just with no HTTP layer) -----------------------------

    def _advance(self, now: float) -> None:
        dt = now - self._last_update
        self._last_update = now
        if dt <= 0:
            return

        if self._scan_enabled and self._scan_speed > 0.0:
            remaining = dt
            speed = self._scan_speed
            guard = 0
            while remaining > 1e-9 and guard < 1_000_000:
                guard += 1
                if self._scan_dir > 0:
                    dist_to_edge = max(0.0, self._scan_right_uw - self._scan_pos)
                else:
                    dist_to_edge = max(0.0, self._scan_pos - self._scan_left_uw)
                t_to_edge = dist_to_edge / speed
                if t_to_edge >= remaining:
                    self._scan_pos += self._scan_dir * speed * remaining
                    remaining = 0.0
                else:
                    self._scan_pos += self._scan_dir * dist_to_edge
                    self._scan_dir *= -1
                    remaining -= t_to_edge
            self._az = normalize_az(self._scan_pos)
            return

        if self._goto_target is not None:
            tgt_az, tgt_el = self._goto_target
            az_err = normalize_az(tgt_az - self._az)
            el_err = tgt_el - self._el
            az_done = abs(az_err) <= self._goto_tol
            el_done = abs(el_err) <= self._goto_tol
            if not az_done:
                step = clamp(az_err, -self._slew_rate * dt, self._slew_rate * dt)
                self._az = normalize_az(self._az + step)
            if not el_done:
                step = clamp(el_err, -self._slew_rate * dt, self._slew_rate * dt)
                self._el = clamp(self._el + step, self._el_min, self._el_max)
            if az_done and el_done:
                self._goto_target = None
            return

        if self._az_speed != 0.0:
            self._az = normalize_az(self._az + self._az_speed * dt)
        if self._el_speed != 0.0:
            new_el = clamp(self._el + self._el_speed * dt, self._el_min, self._el_max)
            if new_el != self._el + self._el_speed * dt:
                self._el_speed = 0.0
            self._el = new_el

    def _current_speeds(self) -> tuple:
        if self._scan_enabled and self._scan_speed > 0.0:
            return (self._scan_dir * self._scan_speed, 0.0)
        if self._goto_target is not None:
            tgt_az, tgt_el = self._goto_target
            az_err = normalize_az(tgt_az - self._az)
            el_err = tgt_el - self._el
            sx = 0.0 if abs(az_err) <= self._goto_tol else math.copysign(self._slew_rate, az_err)
            sy = 0.0 if abs(el_err) <= self._goto_tol else math.copysign(self._slew_rate, el_err)
            return (sx, sy)
        return (self._az_speed, self._el_speed)

    def _nmea_fields(self) -> dict:
        sx, sy = self._current_speeds()
        return {
            "Abs_Azimuth": round(self._az, 4),
            "Abs_Elevation": round(self._el, 4),
            "Geo_Azimuth": round(self._az, 4),
            "Geo_Elevation": round(self._el, 4),
            "Speed_X": round(sx, 4),
            "Speed_Y": round(sy, 4),
            "Mode": 1 if self._scan_enabled else 0,
        }

    def _server_status_fields(self) -> dict:
        fields = self._nmea_fields()
        fields.update(
            {
                "Token_ID": self._token_holder if self._token_holder is not None else 0,
                "Remote_Request": 1 if self._token_holder is not None else 0,
            }
        )
        return fields

    def _check_motion_allowed(self, session_id: Optional[int], now: float) -> Optional[int]:
        if session_id != self.session_id:
            return RC_NOT_AUTH
        if self._token_holder != session_id:
            return RC_TOKEN_IDLE
        if self._token_activity is not None and (now - self._token_activity) > self._token_idle_timeout:
            return RC_TOKEN_IDLE
        self._token_activity = now
        return None

    @staticmethod
    def _ok(action: str, extra: dict) -> dict:
        payload = {"Return Code": RC_OK, "Return String": "OK"}
        payload.update(extra)
        return {action: payload}

    @staticmethod
    def _err(action: str, code: int, msg: str) -> dict:
        return {action: {"Return Code": code, "Return String": msg}}

    # -- action dispatch (mirrors GET /Nexus.cgi?action=...) -----------------

    def handle_action(self, params: dict) -> dict:
        action = params.get("action", "")
        self.calls.append((action, dict(params)))

        remaining = self._raise_conn_error_for.get(action, 0)
        if remaining > 0:
            self._raise_conn_error_for[action] = remaining - 1
            raise ConnectionError(f"simulated connection failure: {action}")

        if self._inject_rc_count > 0:
            self._inject_rc_count -= 1
            return self._err(action or "Unknown", self._inject_rc, "injected fault")

        now = self._clock()
        self._advance(now)
        session_id = params.get("session")
        session_id = int(session_id) if session_id is not None else None

        if action == "SERVERWhoAmI":
            return self._ok(action, {"Id": self.session_id, "Owner": "test"})

        if action == "SERVERLastNMEAGet":
            if session_id != self.session_id:
                return self._err(action, RC_NOT_AUTH, "not authenticated")
            if _truthy(params.get("tokenoverride")) and session_id == self._token_holder:
                self._token_activity = now
            return self._ok(action, self._server_status_fields())

        if action == "PTLastNMEAGet":
            if session_id != self.session_id:
                return self._err(action, RC_NOT_AUTH, "not authenticated")
            return self._ok(action, self._nmea_fields())

        if action == "SERVERRemoteControlRequestAsync":
            if session_id != self.session_id:
                return self._err(action, RC_NOT_AUTH, "not authenticated")
            forced = _truthy(params.get("Forced"))
            if forced or self._token_holder is None or self._token_holder == session_id:
                self._token_holder = session_id
                self._token_activity = now
            return self._ok(action, {"Token_ID": self._token_holder if self._token_holder is not None else 0})

        if action == "SERVERRemoteControlRelease":
            if session_id != self.session_id:
                return self._err(action, RC_NOT_AUTH, "not authenticated")
            if self._token_holder == session_id:
                self._token_holder = None
                self._token_activity = None
            return self._ok(action, {})

        if action == "PTGeoAzimuthElevationSet":
            rc = self._check_motion_allowed(session_id, now)
            if rc is not None:
                return self._err(action, rc, "motion not allowed")
            az = normalize_az(_parse_float(params.get("Geo_Azimuth")))
            el = clamp(_parse_float(params.get("Geo_Elevation")), self._el_min, self._el_max)
            self._goto_target = (az, el)
            self._az_speed = 0.0
            self._el_speed = 0.0
            self._scan_enabled = False
            return self._ok(action, {})

        if action == "PTSpeedModeSet":
            rc = self._check_motion_allowed(session_id, now)
            if rc is not None:
                return self._err(action, rc, "motion not allowed")
            self._az_speed = clamp(_parse_float(params.get("Azimuth_Speed")), -self._max_speed, self._max_speed)
            self._el_speed = clamp(_parse_float(params.get("Elevation_Speed")), -self._max_speed, self._max_speed)
            self._goto_target = None
            self._scan_enabled = False
            return self._ok(action, {})

        if action == "PTAutoScanSpeedSet":
            rc = self._check_motion_allowed(session_id, now)
            if rc is not None:
                return self._err(action, rc, "motion not allowed")
            self._scan_speed = clamp(abs(_parse_float(params.get("Speed"))), 0.0, self._max_speed)
            return self._ok(action, {})

        if action == "PTAutoScanLimitsSet":
            rc = self._check_motion_allowed(session_id, now)
            if rc is not None:
                return self._err(action, rc, "motion not allowed")
            self._scan_left = normalize_az(_parse_float(params.get("Left_Azimuth")))
            self._scan_right = normalize_az(_parse_float(params.get("Right_Azimuth")))
            return self._ok(action, {})

        if action == "PTAutoScanModeOn":
            rc = self._check_motion_allowed(session_id, now)
            if rc is not None:
                return self._err(action, rc, "motion not allowed")
            span = normalize_az(self._scan_right - self._scan_left)
            if span <= 0.0:
                span += 360.0
            self._scan_left_uw = self._scan_left
            self._scan_right_uw = self._scan_left + span
            cur_uw = self._scan_left_uw + normalize_az(self._az - self._scan_left)
            self._scan_pos = clamp(cur_uw, self._scan_left_uw, self._scan_right_uw)
            self._scan_dir = 1
            self._az_speed = 0.0
            self._el_speed = 0.0
            self._goto_target = None
            self._scan_enabled = True
            return self._ok(action, {})

        if action == "PTAutoScanModeOff":
            rc = self._check_motion_allowed(session_id, now)
            if rc is not None:
                return self._err(action, rc, "motion not allowed")
            self._scan_enabled = False
            self._az_speed = 0.0
            self._el_speed = 0.0
            return self._ok(action, {})

        return self._err(action or "Unknown", RC_UNKNOWN_ACTION, f"unknown action: {action!r}")

    # -- POST / DELETE (login/logout plumbing; only "post" login mode and
    # logout actually touch these) -------------------------------------------

    def handle_post(self, url: str, json_body: Any, data_body: Any) -> dict:
        from flir_ptz.nexus.protocol import LOGIN_AUTH, SESSION_SAVE

        if url == LOGIN_AUTH:
            return {"status": "0"}
        if url == SESSION_SAVE:
            return {"ok": True}
        return {}

    def handle_delete(self, url: str) -> dict:
        return {"status": "0"}


class _FakeResp:
    """Duck-types the tiny slice of httpx.Response CameraSession uses."""

    def __init__(self, payload: dict) -> None:
        self._payload = payload
        self.status_code = 200
        self.text = "{}"  # never actually parsed -- .json() is used directly

    def json(self) -> Any:
        return self._payload

    def raise_for_status(self) -> None:
        pass


class FakeTransport:
    """Pure-stdlib duck-type of the httpx.AsyncClient slice CameraSession
    relies on (``get`` / ``post`` / ``delete`` / ``aclose``), dispatching
    in-process to a :class:`KinematicNexusFake`. No sockets, no HTTP --
    the same seam ``test_session.py``'s ``FakeTransport`` uses."""

    def __init__(self, fake: KinematicNexusFake) -> None:
        self._fake = fake
        self.closed = False

    async def get(self, url: str, *, params: dict) -> _FakeResp:
        latency = self._fake._latency_s
        if latency:
            await asyncio.sleep(latency)
        await asyncio.sleep(0)
        return _FakeResp(self._fake.handle_action(params))

    async def post(self, url: str, *, json: Any = None, data: Any = None) -> _FakeResp:
        await asyncio.sleep(0)
        return _FakeResp(self._fake.handle_post(url, json, data))

    async def delete(self, url: str) -> _FakeResp:
        await asyncio.sleep(0)
        return _FakeResp(self._fake.handle_delete(url))

    async def aclose(self) -> None:
        self.closed = True


def make_camera(**kwargs: Any) -> KinematicNexusFake:
    return KinematicNexusFake(**kwargs)


def make_session(
    cam: KinematicNexusFake,
    *,
    login_mode: str = "basic",
    token_policy: Optional[TokenPolicy] = None,
) -> CameraSession:
    cfg = CameraConfig(
        host=_FAKE_HOST,
        username=DEFAULT_USERNAME,
        password=DEFAULT_PASSWORD,
        login_mode=login_mode,
        model="364c",
    )

    def factory() -> FakeTransport:
        return FakeTransport(cam)

    return CameraSession(
        cfg,
        client_factory=factory,
        token_policy=token_policy or TokenPolicy(),
    )


def fast_cfg(**overrides: Any) -> ControlConfig:
    """A ControlConfig with tighter tick periods than the (hand-tuned,
    do-not-touch) defaults, purely to keep these tests fast. Never
    touches the choose_speed profile curves or tolerance defaults --
    only tick-period / timing knobs, which are ordinary runtime config."""
    base = dict(poll_ms=20, scan_poll_ms=40)
    base.update(overrides)
    return dataclasses.replace(ControlConfig(), **base)


class _Harness:
    """One KinematicNexusFake + one connected CameraSession + one
    MotionFSM + one TickController, all driven on a single asyncio loop
    (the loop the calling test's ``asyncio.run()`` created). Captures
    every ``publish_snapshot`` call so tests can assert on the sequence
    of (sample, StepResult) the tick loop produced."""

    def __init__(
        self,
        cam: KinematicNexusFake,
        cfg: Optional[ControlConfig] = None,
        *,
        session_kwargs: Optional[dict] = None,
        controller_kwargs: Optional[dict] = None,
    ):
        self.cam = cam
        self.cfg = cfg or fast_cfg()
        self.session = make_session(cam, **(session_kwargs or {}))
        self.fsm = MotionFSM(self.cfg)
        self.snapshots: list = []
        ck = dict(controller_kwargs or {})
        ck.setdefault("reconnect_after_failures", 5)
        self.controller = TickController(
            self.session,
            self.fsm,
            self.cfg,
            on_snapshot=lambda sample, result: self.snapshots.append((sample, result)),
            **ck,
        )
        self._task: Optional[asyncio.Task] = None

    async def start(self) -> None:
        await self.session.connect()
        self._task = asyncio.create_task(self.controller.run())
        # Give the loop a moment to actually enter run() / take its first tick.
        for _ in range(50):
            if self.controller.running:
                return
            await asyncio.sleep(0.01)

    async def stop(self) -> None:
        self.controller.request_stop()
        if self._task is not None:
            await asyncio.wait_for(self._task, timeout=5.0)

    async def wait_until(self, predicate: Callable[[], bool], timeout: float = 5.0) -> bool:
        """Poll ``predicate`` until it is True or ``timeout`` elapses.
        Always yields to the loop at least once *before* the first check
        -- callers typically call this immediately after a synchronous
        ``submit()``, and without an initial yield the very first check
        could observe stale pre-submit state (the controller's task
        never got a chance to run yet)."""
        deadline = time.monotonic() + timeout
        while True:
            await asyncio.sleep(0.01)
            if predicate():
                return True
            if time.monotonic() >= deadline:
                return predicate()

    def last_mode(self) -> Optional[Mode]:
        return self.snapshots[-1][1].mode if self.snapshots else None


def run(coro: Any) -> Any:
    return asyncio.run(coro)


# ---------------------------------------------------------------------------
# A1/A3/A6 -- the tick loop reaching a goto target
# ---------------------------------------------------------------------------


def test_goto_reaches_target():
    async def body():
        cam = make_camera()
        h = _Harness(cam)
        await h.start()
        try:
            h.controller.submit(Intent.goto(10.0, -5.0, source="test"))
            reached = await h.wait_until(lambda: h.fsm.mode == Mode.IDLE and h.snapshots, timeout=5.0)
            assert reached, "goto never returned to IDLE"
            az, el = cam.position
            assert az == pytest.approx(10.0, abs=0.3)
            assert el == pytest.approx(-5.0, abs=0.3)
            # A1: snapshots were actually published along the way.
            assert len(h.snapshots) > 3
            assert any(s is not None for s, _ in h.snapshots)
        finally:
            await h.stop()

    run(body())


# ---------------------------------------------------------------------------
# Preemption latency: a stop intent submitted mid-move lands within ~one
# tick, not the full move duration -- because the wakeup event fires.
# ---------------------------------------------------------------------------


def test_stop_preemption_is_fast_not_full_tick_period():
    # Deliberately coarse tick period (0.8s) and a slow slew rate (the
    # move alone would take ~35s to complete): the in-process fake's
    # per-call overhead is negligible and constant regardless of
    # tick_period, so making the tick period generously large is what
    # makes "preemption lands in about one round trip, not one whole
    # tick_period" a robust, non-flaky signal rather than a razor-thin
    # timing race.
    tick_period_s = 0.8

    async def body():
        cam = make_camera(slew_rate_deg_s=5.0)
        h = _Harness(cam, cfg=fast_cfg(poll_ms=int(tick_period_s * 1000)))
        await h.start()
        try:
            h.controller.submit(Intent.goto(170.0, 80.0, source="test"))
            started = await h.wait_until(lambda: h.fsm.mode == Mode.GOTO, timeout=2.0)
            assert started

            t_submit = time.monotonic()
            h.controller.submit(Intent.stop(source="test"))
            landed = await h.wait_until(lambda: h.fsm.mode == Mode.IDLE, timeout=2.0)
            elapsed = time.monotonic() - t_submit

            assert landed
            # If the wakeup event were not firing (or were being lost
            # to the clear()-race this test also guards against -- see
            # controller.py's ``_tick`` comment), landing would take
            # close to a full tick_period (0.8s) or more. The wakeup
            # event makes it land in roughly one round trip instead --
            # bound at half the tick period, which leaves an enormous
            # margin over the near-zero overhead of the in-process fake
            # while still failing hard if preemption regresses to
            # "wait out the tick".
            assert elapsed < tick_period_s / 2, (
                f"stop took {elapsed:.3f}s to land (tick_period={tick_period_s}s) "
                "-- wakeup event not firing (or being lost to a re-clear race)?"
            )
        finally:
            await h.stop()

    run(body())


# ---------------------------------------------------------------------------
# Read failures must not kill the loop, and must trigger reconnect after
# the configured threshold of consecutive failures.
# ---------------------------------------------------------------------------


def test_read_failures_do_not_kill_the_loop_and_recover():
    async def body():
        cam = make_camera()
        h = _Harness(cam)
        await h.start()
        try:
            # A handful of injected faults -- fewer than the reconnect
            # threshold (5) -- the loop must keep ticking and recover
            # on its own without ever needing a reconnect.
            cam.inject(rc=5, count=3)
            recovered = await h.wait_until(
                lambda: any(s is not None for s, _ in h.snapshots[-3:]) if h.snapshots else False,
                timeout=5.0,
            )
            assert recovered
            assert h.controller.running is True
        finally:
            await h.stop()

    run(body())


def test_reconnect_attempted_after_threshold_consecutive_failures():
    async def body():
        cam = make_camera()
        h = _Harness(cam, cfg=fast_cfg())
        await h.start()

        connect_calls = []
        orig_connect = h.session.connect

        async def spy_connect():
            connect_calls.append(time.monotonic())
            return await orig_connect()

        h.session.connect = spy_connect

        try:
            # Threshold is 5 (see _Harness) -- inject more faults than
            # that so a reconnect attempt is forced.
            cam.inject(rc=5, count=8)
            triggered = await h.wait_until(lambda: len(connect_calls) >= 1, timeout=5.0)
            assert triggered, "reconnect was never attempted after repeated read failures"
            # The loop must still be alive and eventually reading again.
            recovered = await h.wait_until(
                lambda: h.snapshots and h.snapshots[-1][0] is not None, timeout=5.0
            )
            assert recovered
        finally:
            await h.stop()

    run(body())


# ---------------------------------------------------------------------------
# A19 -- TokenLost degrades to IDLE instead of crashing the loop.
# ---------------------------------------------------------------------------


def test_token_lost_degrades_track_to_idle_without_crashing():
    """A6/A19-style bridge test: once we genuinely hold the token (HELD --
    established by the FSM's own natural ACQUIRE-via-Forced=1 dance), a
    foreign session stealing it produces a *persistent* RC=15 on our next
    write (the fake's ``_check_motion_allowed`` now sees a different
    Token_ID holder). ``CameraSession.execute()`` turns that into
    ``TokenLost`` after its one permitted revive+retry (revive itself
    cannot help here -- ``tokenoverride=1`` only revives an idle token
    that is still *ours*, not one someone else now holds -- see
    nexus/session.py). ``TickController`` must catch it and degrade to
    IDLE instead of crashing the loop -- this is the bridge described in
    the worker brief (fsm.py's own docstring notes it structurally cannot
    see command-execution outcomes at all).

    TRACK is used here rather than SCAN because it keeps re-sending
    ``SetSpeed`` every tick (SCAN's ``ACTIVE`` sub-state deliberately
    sends nothing -- see fsm.py -- so there would be no write left to
    fail once scanning is fully under way); the resulting bridge code in
    controller.py is generic across every FSM mode, SCAN included.
    """

    async def body():
        cam = make_camera()
        h = _Harness(cam, cfg=fast_cfg())
        await h.start()
        try:
            h.controller.submit(Intent.track(150.0, 60.0, source="test"))
            tracking = await h.wait_until(lambda: h.fsm.mode == Mode.TRACK, timeout=2.0)
            assert tracking

            # Wait until we have genuinely, successfully acquired the
            # token (not just entered TRACK mode) before stealing it --
            # stealing any earlier would just be reclaimed immediately
            # by the FSM's own next write via Forced=1.
            held = await h.wait_until(lambda: h.session.token_state.name == "HELD", timeout=3.0)
            assert held, "never observed HELD token state"

            cam.steal_token(by_session=999999)

            degraded = await h.wait_until(lambda: h.fsm.mode == Mode.IDLE, timeout=5.0)
            assert degraded, "TokenLost did not degrade the FSM back to IDLE"
            # The loop must still be running -- this is the "does not
            # crash" half of the requirement.
            assert h.controller.running is True

            # And it must still be able to do useful work afterward
            # (steal the token back to prove the loop is really alive).
            cam.steal_token(by_session=h.session.session_id)
            h.controller.submit(Intent.goto(3.0, 1.0, source="test"))
            still_alive = await h.wait_until(lambda: h.fsm.mode == Mode.GOTO, timeout=5.0)
            assert still_alive
        finally:
            await h.stop()

    run(body())


# ---------------------------------------------------------------------------
# Token keepalive fires while a mode is active; release fires once idle
# past the grace window. Driven via TokenTracker.periodic() from the tick
# (see controller.py's "Token tracking design note").
# ---------------------------------------------------------------------------


def test_keepalive_fires_while_active_then_release_after_grace():
    async def body():
        cam = make_camera()
        cfg = fast_cfg(poll_ms=15)
        # Tight controller-side periodic policy so the test runs fast.
        h = _Harness(
            cam,
            cfg=cfg,
            controller_kwargs=dict(
                token_policy=TokenPolicy(hold_token=False, grace_s=0.25, keepalive_s=0.06)
            ),
        )
        await h.start()
        try:
            keepalive_calls = []
            release_calls = []
            orig_keepalive = h.session.keepalive_token
            orig_release = h.session.release_control

            async def spy_keepalive():
                keepalive_calls.append(time.monotonic())
                return await orig_keepalive()

            async def spy_release():
                release_calls.append(time.monotonic())
                return await orig_release()

            h.session.keepalive_token = spy_keepalive
            h.session.release_control = spy_release

            # Get HELD by driving a short move (natural ACQUIRE dance),
            # then keep TRACK active for a while so mode_active stays
            # True and keepalive has time to fire more than once.
            h.controller.submit(Intent.track(150.0, 30.0, source="test"))
            tracking = await h.wait_until(lambda: h.fsm.mode == Mode.TRACK, timeout=2.0)
            assert tracking

            fired_twice = await h.wait_until(lambda: len(keepalive_calls) >= 2, timeout=3.0)
            assert fired_twice, f"expected >=2 keepalives while active, got {len(keepalive_calls)}"
            assert len(release_calls) == 0, "must not release while still active"

            # Now stop and wait past the grace window -- release should
            # fire exactly once, and keepalive must not resume after it.
            h.controller.submit(Intent.stop(source="test"))
            idle = await h.wait_until(lambda: h.fsm.mode == Mode.IDLE, timeout=2.0)
            assert idle

            released = await h.wait_until(lambda: len(release_calls) >= 1, timeout=2.0)
            assert released, "release never fired after the grace window elapsed"

            keepalive_count_at_release = len(keepalive_calls)
            await asyncio.sleep(0.2)
            assert len(keepalive_calls) == keepalive_count_at_release, (
                "keepalive kept firing after release -- token policy should have "
                "gone quiet"
            )
            assert len(release_calls) == 1, "release should only fire once, not repeatedly"
        finally:
            await h.stop()

    run(body())


# ---------------------------------------------------------------------------
# A26/A27 -- shutdown homing (as a mode, not a reimplementation) + logout.
#
# home_on_shutdown policy (controller.py's _acquire_token_for_homing):
#   "auto"   (default) -- claim a token only if it is currently free; skip
#            (leave the camera alone) if another session holds it.
#   "always" -- force-claim and park regardless of who holds the token.
#   "never"  -- never park on shutdown.
# ---------------------------------------------------------------------------


async def _wait_future(fut, timeout: float = 6.0):
    """Poll a concurrent.futures.Future from within a coroutine on the
    same loop that (indirectly, via run_coroutine_threadsafe) is
    resolving it -- safe because we only ever read `.done()`/call
    `.result()` once it is done, never block synchronously on it."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if fut.done():
            return fut.result()
        await asyncio.sleep(0.01)
    raise TimeoutError("future did not complete in time")


def test_homing_completes_and_reports_done_when_holding_token():
    async def body():
        cam = make_camera(slew_rate_deg_s=60.0)
        h = _Harness(cam, cfg=fast_cfg(home_timeout_s=5.0))
        await h.start()
        try:
            # Acquire the token via a normal command first (mirrors a
            # real session that has been actively controlling the
            # camera before shutdown).
            h.controller.submit(Intent.velocity(1.0, 0.0, source="test"))
            await h.wait_until(lambda: h.fsm.mode == Mode.VELOCITY, timeout=2.0)
            h.controller.submit(Intent.stop(source="test"))
            await h.wait_until(lambda: h.fsm.mode == Mode.IDLE, timeout=2.0)
            assert await h.session.has_control() is True

            fut = h.controller.request_home(timeout=5.0)
            result = await _wait_future(fut, timeout=6.0)
            assert result is True
            assert h.fsm.mode == Mode.IDLE
            az, el = cam.position
            assert az == pytest.approx(h.cfg.home_az, abs=0.5)
            assert el == pytest.approx(h.cfg.home_el, abs=0.5)
        finally:
            await h.stop()

    run(body())


def test_homing_auto_claims_free_token_and_parks():
    """"auto" (the default) is not a no-op when we don't already hold the
    token: a *free* token means nobody is driving, so it is safe (and
    useful) to claim it and park the camera on shutdown."""

    async def body():
        cam = make_camera(slew_rate_deg_s=60.0)
        h = _Harness(cam, cfg=fast_cfg(home_timeout_s=5.0))
        await h.start()
        try:
            assert cam.token_holder is None
            assert await h.session.has_control() is False

            fut = h.controller.request_home(timeout=5.0)
            result = await _wait_future(fut, timeout=6.0)
            assert result is True
            assert h.fsm.mode == Mode.IDLE
            az, el = cam.position
            assert az == pytest.approx(h.cfg.home_az, abs=0.5)
            assert el == pytest.approx(h.cfg.home_el, abs=0.5)
        finally:
            await h.stop()

    run(body())


def test_homing_auto_skips_when_token_held_by_someone_else():
    """"auto" must never steal control from a live operator: if another
    session already holds the token, homing is skipped entirely."""

    async def body():
        cam = make_camera()
        h = _Harness(cam, cfg=fast_cfg(home_timeout_s=3.0))
        await h.start()
        try:
            cam.steal_token(by_session=555555)
            assert await h.session.has_control() is False

            fut = h.controller.request_home(timeout=3.0)
            result = await _wait_future(fut, timeout=4.0)
            assert result is False
            assert h.fsm.mode == Mode.IDLE  # never even entered HOMING
            assert cam.token_holder == 555555, "the foreign operator's token must be left alone"
        finally:
            await h.stop()

    run(body())


def test_homing_always_force_claims_even_when_held_by_someone_else():
    """"always" restores the old rude-but-reliable behaviour: park
    regardless of who currently holds the token."""

    async def body():
        cam = make_camera(slew_rate_deg_s=60.0)
        h = _Harness(
            cam,
            cfg=fast_cfg(home_timeout_s=5.0),
            controller_kwargs=dict(home_on_shutdown="always"),
        )
        await h.start()
        try:
            cam.steal_token(by_session=555555)

            fut = h.controller.request_home(timeout=5.0)
            result = await _wait_future(fut, timeout=6.0)
            assert result is True
            assert h.fsm.mode == Mode.IDLE
            az, el = cam.position
            assert az == pytest.approx(h.cfg.home_az, abs=0.5)
            assert el == pytest.approx(h.cfg.home_el, abs=0.5)
        finally:
            await h.stop()

    run(body())


def test_homing_never_skips_even_with_a_free_token():
    """"never" must not park even though a free token is available and
    homing would otherwise (under "auto") succeed."""

    async def body():
        cam = make_camera()
        h = _Harness(
            cam,
            cfg=fast_cfg(home_timeout_s=3.0),
            controller_kwargs=dict(home_on_shutdown="never"),
        )
        await h.start()
        try:
            assert cam.token_holder is None

            fut = h.controller.request_home(timeout=3.0)
            result = await _wait_future(fut, timeout=4.0)
            assert result is False
            assert h.fsm.mode == Mode.IDLE
            assert cam.token_holder is None, "must not have claimed the free token"
        finally:
            await h.stop()

    run(body())


def test_logout_releases_session_slot():
    async def body():
        cam = make_camera()
        h = _Harness(cam)
        await h.start()
        try:
            sid = h.session.session_id
            assert sid is not None

            fut = h.controller.request_logout()
            await _wait_future(fut, timeout=5.0)

            # A27: the camera-side transport must have been closed as part
            # of logout.
            assert h.session._client is None or getattr(h.session._client, "closed", False)
        finally:
            h.controller.request_stop()
            await asyncio.wait_for(h._task, timeout=5.0)

    run(body())


# ---------------------------------------------------------------------------
# ROS-callback-thread contract: submit_threadsafe() is safe to call from a
# real foreign OS thread (mirrors how ptz_node.py's subscription callbacks,
# running on rclpy's executor thread, drive the controller).
# ---------------------------------------------------------------------------


def test_submit_threadsafe_from_a_real_foreign_thread():
    async def body():
        cam = make_camera()
        h = _Harness(cam)
        await h.start()
        try:
            errors = []

            def worker():
                try:
                    h.controller.submit_threadsafe(Intent.goto(5.0, 2.0, source="joy"))
                except Exception as exc:  # pragma: no cover - failure path
                    errors.append(exc)

            t = threading.Thread(target=worker)
            t.start()
            t.join(timeout=5.0)
            assert not errors

            moved = await h.wait_until(lambda: h.fsm.mode == Mode.GOTO, timeout=2.0)
            assert moved
        finally:
            await h.stop()

    run(body())


# ---------------------------------------------------------------------------
# Coordinate-frame handling: goto_feedback_frame threads the choice through
# without editing fsm.py -- verified via the constructor guard and the
# (default) unchanged "abs" behaviour.
# ---------------------------------------------------------------------------


def test_goto_feedback_frame_rejects_invalid_value():
    cam = make_camera()
    session = make_session(cam)
    fsm = MotionFSM(fast_cfg())
    with pytest.raises(ValueError):
        TickController(session, fsm, fast_cfg(), goto_feedback_frame="not-a-real-frame")


def test_goto_feedback_frame_default_is_abs_bit_for_bit():
    async def body():
        cam = make_camera()
        h = _Harness(cam)
        assert h.controller._goto_feedback_frame == "abs"
        await h.start()
        try:
            h.controller.submit(Intent.goto(8.0, -2.0, source="test"))
            reached = await h.wait_until(lambda: h.fsm.mode == Mode.IDLE, timeout=5.0)
            assert reached
            az, el = cam.position
            assert az == pytest.approx(8.0, abs=0.3)
            assert el == pytest.approx(-2.0, abs=0.3)
        finally:
            await h.stop()

    run(body())


# ---------------------------------------------------------------------------
# Static guard: no asyncio.Task.cancel() anywhere in the control plane
# (spec sec. 8 rule 2). Behavioural coverage of the same rule lives in
# test_stop_preemption_is_fast_not_full_tick_period above (preemption
# works without ever cancelling an in-flight command).
# ---------------------------------------------------------------------------


def test_no_task_cancel_in_controller_source():
    controller_path = os.path.join(_PKG_ROOT, "flir_ptz", "control", "controller.py")
    with open(controller_path, "r", encoding="utf-8") as f:
        source = f.read()
    assert "Task.cancel" not in source
    assert ".cancel()" not in source


# ---------------------------------------------------------------------------
# Module sanity: importable without httpx being installed.
# ---------------------------------------------------------------------------


def test_module_imports_without_httpx_installed():
    """controller.py must load where httpx is unavailable.

    Masks httpx rather than asserting the machine lacks it, so the test keeps
    verifying the actual property once httpx is installed. Runs in a
    subprocess so the masked import state cannot leak into other tests.
    """
    import subprocess
    import sys as _sys
    import textwrap

    code = textwrap.dedent(
        """
        import sys
        sys.modules["httpx"] = None   # makes `import httpx` raise ImportError
        import flir_ptz.control.controller as c
        assert hasattr(c, "TickController")
        print("OK")
        """
    )
    proc = subprocess.run(
        [_sys.executable, "-c", code],
        cwd=str(_PKG_ROOT),
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert proc.returncode == 0, proc.stderr
    assert "OK" in proc.stdout
