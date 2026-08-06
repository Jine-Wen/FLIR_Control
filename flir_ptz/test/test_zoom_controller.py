#!/usr/bin/env python3
"""Tests for the EO (daylight) and IR (thermal) zoom paths added to
flir_ptz.control.controller.TickController: ``submit_zoom(direction,
device)`` / the tick's per-device dead-man timers / the adaptive poll
cadence / the orthogonality invariant (zoom never touches ``fsm.mode``) /
EO-IR independence (stopping/zooming one must never affect the other).

Same seam as test_controller.py (whose in-process kinematic fake this file
deliberately does not import, to keep test files collection-order
independent): a small hand-written async transport with no HTTP server and
no sockets, duck-typing only the tiny slice of httpx.AsyncClient
CameraSession actually calls. Must run with no ROS, no camera, no httpx.

No real IP address or password appears anywhere in this file (spec sec. 8
rule 1): the fake camera lives entirely in-process, and CameraConfig's
host/username/password fields are unused placeholder strings on an RFC 2606
reserved ``.invalid`` domain.
"""

from __future__ import annotations

import asyncio
import os
import sys
import time
from typing import Any, Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import pytest  # noqa: E402

from flir_ptz.control.arbitration import Arbiter  # noqa: E402
from flir_ptz.control.config import CameraConfig, ControlConfig  # noqa: E402
from flir_ptz.control.controller import TickController  # noqa: E402
from flir_ptz.control.fsm import Mode, MotionFSM  # noqa: E402
from flir_ptz.nexus.session import CameraSession  # noqa: E402

_FAKE_HOST = "http://camera.example.invalid"


class _FakeResponse:
    def __init__(self, payload: dict) -> None:
        self._payload = payload
        self.text = "{}"

    def json(self) -> Any:
        return self._payload

    def raise_for_status(self) -> None:
        pass


class ZoomFake:
    """A minimal in-process Nexus fake: just enough for the tick loop's
    every-tick PTLastNMEAGet read, the token dance, and the DLTVZoom*/
    DLTVLastNMEAGet actions under test. No kinematics -- these tests never
    submit a pan/tilt Intent, so the FSM stays IDLE throughout, which is
    exactly the orthogonality invariant under test."""

    def __init__(self, *, session_id: int = 909) -> None:
        self.session_id = session_id
        self.token_holder: Optional[int] = None
        self.zoom_calls: list[str] = []  # "in" | "out" | "stop", call order
        self.dltv_calls = 0
        self.zoom_mm = 40.0
        self.zoom_pctg = 20.0
        # IR: independent state from EO above -- separate call list, separate
        # poll counter, separate reading.
        self.ir_zoom_calls: list[str] = []  # "in" | "out" | "stop", call order
        self.ir_calls = 0
        self.ir_fov = 11.67
        self.ir_zoom_pctg = 31.25
        self.calls: list[tuple[str, dict]] = []

    def handle_action(self, params: dict) -> dict:
        action = params.get("action", "")
        self.calls.append((action, dict(params)))

        if action == "SERVERWhoAmI":
            return {action: {"Return Code": 0, "Id": self.session_id, "Owner": "test"}}

        if action == "SERVERRemoteControlRequestAsync":
            self.token_holder = self.session_id
            return {action: {"Return Code": 0, "Token_ID": self.token_holder}}

        if action == "SERVERRemoteControlRelease":
            self.token_holder = None
            return {action: {"Return Code": 0}}

        if action == "SERVERLastNMEAGet":
            return {action: {"Return Code": 0, "Token_ID": self.token_holder or 0}}

        if action == "PTLastNMEAGet":
            return {action: {
                "Return Code": 0, "Abs_Azimuth": 0.0, "Abs_Elevation": 0.0,
                "Geo_Azimuth": 0.0, "Geo_Elevation": 0.0,
                "Speed_X": 0.0, "Speed_Y": 0.0, "Mode": 0,
            }}

        if action == "DLTVZoomCountsIncrement":
            self.zoom_calls.append("in")
            return {action: {"Return Code": 0}}

        if action == "DLTVZoomCountsDecrement":
            self.zoom_calls.append("out")
            return {action: {"Return Code": 0}}

        if action == "DLTVZoomStop":
            self.zoom_calls.append("stop")
            return {action: {"Return Code": 0}}

        if action == "DLTVLastNMEAGet":
            self.dltv_calls += 1
            return {action: {
                "Return Code": 0,
                "Zoom": self.zoom_mm,
                "Zoom_pctg": self.zoom_pctg,
                "Focus_pctg": 50.0,
                "Autofocus": 1.0,
            }}

        if action == "IRZoomIn":
            self.ir_zoom_calls.append("in")
            return {action: {"Return Code": 0}}

        if action == "IRZoomOut":
            self.ir_zoom_calls.append("out")
            return {action: {"Return Code": 0}}

        if action == "IRZoomStop":
            self.ir_zoom_calls.append("stop")
            return {action: {"Return Code": 0}}

        if action == "IRLastNMEAGet":
            self.ir_calls += 1
            return {action: {
                "Return Code": 0,
                "FOV": self.ir_fov,
                "Zoom_Pctg": self.ir_zoom_pctg,
                "FOV_Index": 0,
            }}

        return {action: {"Return Code": 0}}

    def handle_post(self, url: str, json_body: Any, data_body: Any) -> dict:
        return {}

    def handle_delete(self, url: str) -> dict:
        return {}


class FakeTransport:
    def __init__(self, fake: ZoomFake) -> None:
        self._fake = fake
        self.closed = False

    async def get(self, url: str, *, params: dict) -> _FakeResponse:
        await asyncio.sleep(0)
        return _FakeResponse(self._fake.handle_action(params))

    async def post(self, url: str, *, json: Any = None, data: Any = None) -> _FakeResponse:
        await asyncio.sleep(0)
        return _FakeResponse(self._fake.handle_post(url, json, data))

    async def delete(self, url: str) -> _FakeResponse:
        await asyncio.sleep(0)
        return _FakeResponse(self._fake.handle_delete(url))

    async def aclose(self) -> None:
        self.closed = True


def fast_cfg(**overrides: Any) -> ControlConfig:
    base = dict(poll_ms=20, scan_poll_ms=40)
    base.update(overrides)
    return ControlConfig(**base)


class _Harness:
    def __init__(
        self,
        cam: ZoomFake,
        cfg: Optional[ControlConfig] = None,
        *,
        zoom_deadman_s: float = 0.2,
        zoom_poll_idle_s: float = 10.0,
        zoom_poll_active_s: Optional[float] = None,
    ) -> None:
        self.cam = cam
        self.cfg = cfg or fast_cfg()
        session_cfg = CameraConfig(host=_FAKE_HOST, username="u", password="p", login_mode="basic", model="364c")
        self.session = CameraSession(session_cfg, client_factory=lambda: FakeTransport(cam))
        self.fsm = MotionFSM(self.cfg)
        self.snapshots: list = []
        controller_kwargs: dict[str, Any] = dict(
            on_snapshot=lambda sample, result: self.snapshots.append((sample, result)),
            reconnect_after_failures=5,
            zoom_deadman_s=zoom_deadman_s,
            zoom_poll_idle_s=zoom_poll_idle_s,
        )
        if zoom_poll_active_s is not None:
            controller_kwargs["zoom_poll_active_s"] = zoom_poll_active_s
        self.controller = TickController(
            self.session,
            self.fsm,
            self.cfg,
            **controller_kwargs,
        )
        self._task: Optional[asyncio.Task] = None

    async def start(self) -> None:
        await self.session.connect()
        self._task = asyncio.create_task(self.controller.run())
        for _ in range(50):
            if self.controller.running:
                return
            await asyncio.sleep(0.01)

    async def stop(self) -> None:
        self.controller.request_stop()
        if self._task is not None:
            await asyncio.wait_for(self._task, timeout=5.0)

    async def wait_until(self, predicate, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while True:
            await asyncio.sleep(0.01)
            if predicate():
                return True
            if time.monotonic() >= deadline:
                return predicate()


def run(coro: Any) -> Any:
    return asyncio.run(coro)


# ---------------------------------------------------------------------------
# MANDATORY SAFETY: the dead-man timer stops an unrenewed zoom on its own.
# ---------------------------------------------------------------------------


def test_deadman_timer_fires_a_stop_when_hold_is_not_renewed():
    async def body():
        cam = ZoomFake()
        h = _Harness(cam, zoom_deadman_s=0.15)
        await h.start()
        try:
            h.controller.submit_zoom("in")
            started = await h.wait_until(lambda: cam.zoom_calls == ["in"], timeout=2.0)
            assert started, "zoom_in never reached the camera"

            # Nothing renews it -- the tick loop must stop it on its own
            # well within a couple of dead-man windows.
            stopped = await h.wait_until(lambda: "stop" in cam.zoom_calls, timeout=2.0)
            assert stopped, "dead-man timer never fired a stop"
            assert cam.zoom_calls == ["in", "stop"], cam.zoom_calls
        finally:
            await h.stop()

    run(body())


def test_renewed_hold_does_not_stop():
    async def body():
        cam = ZoomFake()
        deadman_s = 0.15
        h = _Harness(cam, zoom_deadman_s=deadman_s)
        await h.start()
        try:
            # Keep re-submitting "in" faster than the dead-man window, for
            # well longer than one window's worth of time.
            deadline = time.monotonic() + deadman_s * 6
            while time.monotonic() < deadline:
                h.controller.submit_zoom("in")
                await asyncio.sleep(deadman_s / 4)

            assert "stop" not in cam.zoom_calls, (
                f"a renewed hold must never be auto-stopped, got {cam.zoom_calls}"
            )
            assert cam.zoom_calls, "zoom_in should have reached the camera at least once"
            assert set(cam.zoom_calls) == {"in"}
        finally:
            await h.stop()

    run(body())


def test_explicit_stop_is_immediate_and_no_later_deadman_stop_follows():
    async def body():
        cam = ZoomFake()
        deadman_s = 0.2
        h = _Harness(cam, zoom_deadman_s=deadman_s)
        await h.start()
        try:
            h.controller.submit_zoom("in")
            await h.wait_until(lambda: cam.zoom_calls == ["in"], timeout=2.0)

            t0 = time.monotonic()
            h.controller.submit_zoom("stop")
            landed = await h.wait_until(lambda: cam.zoom_calls == ["in", "stop"], timeout=1.0)
            elapsed = time.monotonic() - t0
            assert landed
            # Lands in about one tick round trip, not out the dead-man window.
            assert elapsed < deadman_s, f"explicit stop took {elapsed:.3f}s to land"

            # Wait out (and past) what the dead-man timer's window would
            # have been -- no redundant extra stop should ever be sent.
            await asyncio.sleep(deadman_s * 3)
            assert cam.zoom_calls == ["in", "stop"], (
                f"an explicit stop must not be followed by a spurious dead-man stop, got {cam.zoom_calls}"
            )
        finally:
            await h.stop()

    run(body())


def test_zoom_out_direction_also_covered_by_deadman():
    async def body():
        cam = ZoomFake()
        h = _Harness(cam, zoom_deadman_s=0.15)
        await h.start()
        try:
            h.controller.submit_zoom("out")
            started = await h.wait_until(lambda: cam.zoom_calls == ["out"], timeout=2.0)
            assert started
            stopped = await h.wait_until(lambda: "stop" in cam.zoom_calls, timeout=2.0)
            assert stopped
            assert cam.zoom_calls == ["out", "stop"]
        finally:
            await h.stop()

    run(body())


# ---------------------------------------------------------------------------
# Orthogonality: zoom must never become an FSM mode, and must never disturb
# whichever pan/tilt mode is running.
# ---------------------------------------------------------------------------


def test_zoom_never_changes_fsm_mode():
    async def body():
        cam = ZoomFake()
        h = _Harness(cam, zoom_deadman_s=0.15)
        await h.start()
        try:
            assert h.fsm.mode == Mode.IDLE

            h.controller.submit_zoom("in")
            await h.wait_until(lambda: cam.zoom_calls == ["in"], timeout=2.0)
            assert h.fsm.mode == Mode.IDLE

            await h.wait_until(lambda: "stop" in cam.zoom_calls, timeout=2.0)
            assert h.fsm.mode == Mode.IDLE

            h.controller.submit_zoom("out")
            await h.wait_until(lambda: cam.zoom_calls.count("out") >= 1, timeout=2.0)
            assert h.fsm.mode == Mode.IDLE

            h.controller.submit_zoom("stop")
            await h.wait_until(lambda: cam.zoom_calls[-1] == "stop", timeout=2.0)
            assert h.fsm.mode == Mode.IDLE

            # Every published snapshot along the way must also show IDLE --
            # zoom must not sneak into any transient FSM state either.
            assert all(result.mode == Mode.IDLE for _, result in h.snapshots)
        finally:
            await h.stop()

    run(body())


# ---------------------------------------------------------------------------
# DLTV state polling: slow cadence, not every tick -- and a prompt refresh
# right after a zoom command.
# ---------------------------------------------------------------------------


def test_dltv_polling_stays_on_its_slow_cadence_while_idle():
    async def body():
        cam = ZoomFake()
        tick_period_s = 0.01
        poll_idle_s = 0.3
        h = _Harness(cam, cfg=fast_cfg(poll_ms=int(tick_period_s * 1000)), zoom_poll_idle_s=poll_idle_s)
        await h.start()
        try:
            # Let a good number of ticks elapse with zero zoom activity.
            run_for_s = poll_idle_s * 3.0
            await asyncio.sleep(run_for_s)

            ticks_elapsed = len(h.snapshots)
            assert ticks_elapsed > 5, "test setup problem: too few ticks observed"

            # If DLTV were polled every tick, dltv_calls would track
            # ticks_elapsed almost 1:1. On the slow idle cadence it should be
            # a small handful (1 immediate poll at startup + ~2-3 more over
            # ~3x the idle window), nowhere near ticks_elapsed.
            assert cam.dltv_calls < ticks_elapsed / 2, (
                f"DLTV polled {cam.dltv_calls} times over {ticks_elapsed} ticks -- "
                "looks like every-tick polling, not the slow idle cadence"
            )
            assert 1 <= cam.dltv_calls <= 6, cam.dltv_calls
        finally:
            await h.stop()

    run(body())


def test_dltv_polled_promptly_right_after_a_zoom_command():
    async def body():
        cam = ZoomFake()
        # Idle cadence deliberately huge -- if a poll happens anyway, it's
        # because the zoom command forced it, not the idle timer.
        h = _Harness(cam, zoom_deadman_s=0.15, zoom_poll_idle_s=60.0)
        await h.start()
        try:
            # Absorb the one forced startup poll before measuring.
            await h.wait_until(lambda: cam.dltv_calls >= 1, timeout=2.0)
            baseline = cam.dltv_calls

            h.controller.submit_zoom("in")
            await h.wait_until(lambda: cam.zoom_calls == ["in"], timeout=2.0)

            polled_again = await h.wait_until(lambda: cam.dltv_calls > baseline, timeout=1.0)
            assert polled_again, "DLTV was not refreshed promptly after the zoom command"
        finally:
            await h.stop()

    run(body())


def test_zoom_state_property_reflects_latest_dltv_reading():
    async def body():
        cam = ZoomFake()
        cam.zoom_mm = 41.75
        cam.zoom_pctg = 18.59
        h = _Harness(cam, zoom_poll_idle_s=60.0)
        await h.start()
        try:
            got = await h.wait_until(lambda: h.controller.zoom_state is not None, timeout=2.0)
            assert got
            sample = h.controller.zoom_state
            assert sample.zoom == pytest.approx(41.75)
            assert sample.zoom_pctg == pytest.approx(18.59)
        finally:
            await h.stop()

    run(body())


# ---------------------------------------------------------------------------
# submit_zoom() is thread-safe -- callable from a real foreign OS thread,
# mirroring how ptz_node.py's ZoomCmd subscription callback (rclpy executor
# thread) drives it.
# ---------------------------------------------------------------------------


def test_submit_zoom_from_a_real_foreign_thread():
    import threading

    async def body():
        cam = ZoomFake()
        h = _Harness(cam, zoom_deadman_s=0.15)
        await h.start()
        try:
            errors = []

            def worker():
                try:
                    h.controller.submit_zoom("in")
                except Exception as exc:  # pragma: no cover - failure path
                    errors.append(exc)

            t = threading.Thread(target=worker)
            t.start()
            t.join(timeout=5.0)
            assert not errors

            reached = await h.wait_until(lambda: cam.zoom_calls == ["in"], timeout=2.0)
            assert reached
        finally:
            await h.stop()

    run(body())


def test_submit_zoom_rejects_invalid_direction():
    async def body():
        cam = ZoomFake()
        h = _Harness(cam)
        await h.start()
        try:
            with pytest.raises(ValueError):
                h.controller.submit_zoom("sideways")
        finally:
            await h.stop()

    run(body())


# ---------------------------------------------------------------------------
# Arbitration invariant zoom depends on (nodes/ptz_node.py's _on_zoom):
# a stop is accepted from anyone, even while another source owns the lease.
# Exercised directly against the shared Arbiter -- the exact rule
# `_arbitrate()`/`_on_zoom` delegate to (no rclpy needed to prove it).
# ---------------------------------------------------------------------------


def test_zoom_stop_is_accepted_while_another_source_owns_the_lease():
    arb = Arbiter(lease_s=60.0)
    now = 1000.0
    arb.claim("joy", now)
    assert arb.owner(now) == "joy"

    # A non-stop zoom command from "web" must be refused...
    assert arb.allows("web", False, now) is False
    # ...but a zoom "stop" from "web" must ALWAYS be accepted, exactly like
    # a zero-speed joystick or scan(stop=true) -- non-negotiable safety rule
    # for a lens that keeps moving until told to stop.
    assert arb.allows("web", True, now) is True


def test_zoom_non_stop_is_still_gated_by_the_lease():
    """The other half of the same invariant: only "stop" gets the safety
    exemption -- "in"/"out" are ordinary arbitrated commands."""
    arb = Arbiter(lease_s=60.0)
    now = 1000.0
    arb.claim("joy", now)
    assert arb.allows("web", False, now) is False
    arb.claim("web", now)
    assert arb.allows("web", False, now) is True


# ---------------------------------------------------------------------------
# IR zoom: submit_zoom(direction, "ir") issues the right IR actions, has its
# own independent dead-man timer, and never touches fsm.mode -- mirrors the
# EO tests above exactly, using IRZoomIn/IRZoomOut/IRZoomStop.
# ---------------------------------------------------------------------------


def test_ir_deadman_timer_fires_a_stop_when_hold_is_not_renewed():
    async def body():
        cam = ZoomFake()
        h = _Harness(cam, zoom_deadman_s=0.15)
        await h.start()
        try:
            h.controller.submit_zoom("in", "ir")
            started = await h.wait_until(lambda: cam.ir_zoom_calls == ["in"], timeout=2.0)
            assert started, "ir_zoom_in never reached the camera"

            stopped = await h.wait_until(lambda: "stop" in cam.ir_zoom_calls, timeout=2.0)
            assert stopped, "IR dead-man timer never fired a stop"
            assert cam.ir_zoom_calls == ["in", "stop"], cam.ir_zoom_calls
            # EO must be completely untouched.
            assert cam.zoom_calls == []
        finally:
            await h.stop()

    run(body())


def test_ir_zoom_never_changes_fsm_mode():
    async def body():
        cam = ZoomFake()
        h = _Harness(cam, zoom_deadman_s=0.15)
        await h.start()
        try:
            assert h.fsm.mode == Mode.IDLE
            h.controller.submit_zoom("in", "ir")
            await h.wait_until(lambda: cam.ir_zoom_calls == ["in"], timeout=2.0)
            assert h.fsm.mode == Mode.IDLE
            await h.wait_until(lambda: "stop" in cam.ir_zoom_calls, timeout=2.0)
            assert h.fsm.mode == Mode.IDLE
            assert all(result.mode == Mode.IDLE for _, result in h.snapshots)
        finally:
            await h.stop()

    run(body())


def test_ir_zoom_state_property_reflects_latest_ir_reading_independent_of_eo():
    async def body():
        cam = ZoomFake()
        cam.ir_fov = 10.70
        cam.ir_zoom_pctg = 37.50
        cam.zoom_mm = 41.75
        cam.zoom_pctg = 18.59
        h = _Harness(cam, zoom_poll_idle_s=60.0)
        await h.start()
        try:
            got = await h.wait_until(lambda: h.controller.ir_zoom_state is not None, timeout=2.0)
            assert got
            ir_sample = h.controller.ir_zoom_state
            assert ir_sample.fov == pytest.approx(10.70)
            assert ir_sample.zoom_pctg == pytest.approx(37.50)

            eo_sample = h.controller.zoom_state
            assert eo_sample is not None
            assert eo_sample.zoom_pctg == pytest.approx(18.59)
            # The two properties must never be confused with each other.
            assert ir_sample.zoom_pctg != eo_sample.zoom_pctg
        finally:
            await h.stop()

    run(body())


# ---------------------------------------------------------------------------
# EO / IR independence: zooming or stopping one device must never affect
# the other's pending command, dead-man deadline, or active flag.
# ---------------------------------------------------------------------------


def test_stopping_eo_leaves_a_running_ir_zoom_untouched():
    async def body():
        cam = ZoomFake()
        # IR's dead-man window is generous so it can't fire on its own
        # during this test -- only the EO stop below should ever move it.
        h = _Harness(cam, zoom_deadman_s=5.0)
        await h.start()
        try:
            h.controller.submit_zoom("in", "eo")
            h.controller.submit_zoom("in", "ir")
            await h.wait_until(lambda: cam.zoom_calls == ["in"] and cam.ir_zoom_calls == ["in"], timeout=2.0)

            h.controller.submit_zoom("stop", "eo")
            await h.wait_until(lambda: cam.zoom_calls == ["in", "stop"], timeout=2.0)

            # Give the tick loop a few more rounds to prove IR was never
            # touched by the EO stop.
            await asyncio.sleep(0.2)
            assert cam.ir_zoom_calls == ["in"], (
                f"stopping EO must not affect IR's running zoom, got {cam.ir_zoom_calls}"
            )
        finally:
            await h.stop()

    run(body())


def test_stopping_ir_leaves_a_running_eo_zoom_untouched():
    async def body():
        cam = ZoomFake()
        h = _Harness(cam, zoom_deadman_s=5.0)
        await h.start()
        try:
            h.controller.submit_zoom("out", "eo")
            h.controller.submit_zoom("out", "ir")
            await h.wait_until(lambda: cam.zoom_calls == ["out"] and cam.ir_zoom_calls == ["out"], timeout=2.0)

            h.controller.submit_zoom("stop", "ir")
            await h.wait_until(lambda: cam.ir_zoom_calls == ["out", "stop"], timeout=2.0)

            await asyncio.sleep(0.2)
            assert cam.zoom_calls == ["out"], (
                f"stopping IR must not affect EO's running zoom, got {cam.zoom_calls}"
            )
        finally:
            await h.stop()

    run(body())


def test_each_devices_deadman_fires_independently_stopping_one_leaves_the_other_running():
    """The core independence guarantee: give EO a short dead-man window and
    IR a long one. EO's timer must fire on its own while IR, never
    renewed but still within its own (longer) window, keeps running."""

    async def body():
        cam = ZoomFake()
        h = _Harness(cam, zoom_deadman_s=0.15)
        await h.start()
        try:
            h.controller.submit_zoom("in", "eo")
            h.controller.submit_zoom("in", "ir")
            await h.wait_until(lambda: cam.zoom_calls == ["in"] and cam.ir_zoom_calls == ["in"], timeout=2.0)

            # Keep renewing IR only -- EO's hold must still expire on its own.
            deadline = time.monotonic() + 0.6
            while time.monotonic() < deadline:
                h.controller.submit_zoom("in", "ir")
                await asyncio.sleep(0.04)

            assert cam.zoom_calls == ["in", "stop"], (
                f"EO dead-man must fire on its own even while IR is being renewed, got {cam.zoom_calls}"
            )
            # IR was resubmitted repeatedly (each renewal re-issues "in"),
            # but a renewed hold must never be auto-stopped.
            assert "stop" not in cam.ir_zoom_calls, (
                f"a renewed IR hold must never be auto-stopped, got {cam.ir_zoom_calls}"
            )
            assert set(cam.ir_zoom_calls) == {"in"}
        finally:
            await h.stop()

    run(body())


# ---------------------------------------------------------------------------
# device defaults to "eo" when absent or unrecognised (ZoomCmd.device /
# submit_zoom's device parameter).
# ---------------------------------------------------------------------------


def test_submit_zoom_device_defaults_to_eo_when_omitted():
    async def body():
        cam = ZoomFake()
        h = _Harness(cam, zoom_deadman_s=0.15)
        await h.start()
        try:
            h.controller.submit_zoom("in")  # no device argument at all
            reached = await h.wait_until(lambda: cam.zoom_calls == ["in"], timeout=2.0)
            assert reached
            assert cam.ir_zoom_calls == []
        finally:
            await h.stop()

    run(body())


def test_submit_zoom_unrecognised_device_defaults_to_eo():
    async def body():
        cam = ZoomFake()
        h = _Harness(cam, zoom_deadman_s=0.15)
        await h.start()
        try:
            h.controller.submit_zoom("in", "thermal-camera-2")
            reached = await h.wait_until(lambda: cam.zoom_calls == ["in"], timeout=2.0)
            assert reached, "an unrecognised device must fall back to eo, not be silently dropped"
            assert cam.ir_zoom_calls == []
        finally:
            await h.stop()

    run(body())


# ---------------------------------------------------------------------------
# Adaptive polling cadence (jittery-readout fix): fast (~ZOOM_POLL_ACTIVE_S)
# while a device is actively zooming, slow (ZOOM_POLL_IDLE_S) while idle --
# including a device that has never been zoomed at all. Counted over
# simulated (real, but short) time, mirroring the existing idle-cadence test.
# ---------------------------------------------------------------------------


def test_ir_polling_stays_on_its_slow_cadence_while_never_zoomed():
    async def body():
        cam = ZoomFake()
        tick_period_s = 0.01
        poll_idle_s = 0.3
        h = _Harness(cam, cfg=fast_cfg(poll_ms=int(tick_period_s * 1000)), zoom_poll_idle_s=poll_idle_s)
        await h.start()
        try:
            run_for_s = poll_idle_s * 3.0
            await asyncio.sleep(run_for_s)

            ticks_elapsed = len(h.snapshots)
            assert ticks_elapsed > 5, "test setup problem: too few ticks observed"
            assert cam.ir_calls < ticks_elapsed / 2, (
                f"IR polled {cam.ir_calls} times over {ticks_elapsed} ticks while never "
                "zoomed -- looks like every-tick polling, not the slow idle cadence"
            )
            assert 1 <= cam.ir_calls <= 6, cam.ir_calls
        finally:
            await h.stop()

    run(body())


def test_polling_speeds_up_while_zooming_and_slows_back_down_once_idle():
    """The adaptive-cadence contract directly: count DLTV polls over a fixed
    wall-clock window while actively zooming vs. an equal window once the
    zoom has stopped (dead-man expired) and stayed idle. The active window
    must show meaningfully more polls than the idle window."""

    async def body():
        cam = ZoomFake()
        poll_active_s = 0.03
        poll_idle_s = 0.3
        deadman_s = 0.2
        h = _Harness(
            cam,
            cfg=fast_cfg(poll_ms=5),
            zoom_deadman_s=deadman_s,
            zoom_poll_idle_s=poll_idle_s,
            zoom_poll_active_s=poll_active_s,
        )
        await h.start()
        try:
            # Absorb the forced startup poll.
            await h.wait_until(lambda: cam.dltv_calls >= 1, timeout=2.0)

            # -- active window: hold "in" continuously for a fixed duration --
            window_s = 0.3
            active_deadline = time.monotonic() + window_s
            while time.monotonic() < active_deadline:
                h.controller.submit_zoom("in", "eo")
                await asyncio.sleep(poll_active_s / 2)
            active_count = cam.dltv_calls

            # Let the dead-man expire and the channel settle into idle.
            await h.wait_until(lambda: cam.zoom_calls and cam.zoom_calls[-1] == "stop", timeout=2.0)
            idle_baseline = cam.dltv_calls

            # -- idle window: same wall-clock duration, no zoom activity --
            await asyncio.sleep(window_s)
            idle_count = cam.dltv_calls - idle_baseline

            assert active_count > idle_count, (
                f"expected more DLTV polls while actively zooming ({active_count}) than "
                f"while idle over an equal window ({idle_count})"
            )
            assert idle_count <= 2, f"idle window polled too often: {idle_count}"
        finally:
            await h.stop()

    run(body())
