#!/usr/bin/env python3
"""Tests for flir_ptz.nexus.session — the async Nexus camera-session layer.

Hard requirement: this file MUST pass on a machine with httpx **not**
installed (see ARCHITECTURE.md sec. 1). It never imports httpx and never
needs it — CameraSession's transport is fully dependency-injected via
``client_factory``, driven here by a small hand-written, pure-stdlib async
fake (``FakeTransport`` / ``NexusFake`` below). This is deliberately
smaller than ``test/test_controller.py``'s in-process kinematic fake
(no motion model here, just enough scripted RC/session behaviour to
drive every ``CameraSession`` code path) — no HTTP server, no sockets,
just an in-process fake matching the tiny duck-typed slice of
httpx.AsyncClient's API that CameraSession actually calls.

No real IP address or password appears anywhere in this file (spec sec.
8, rule 1) — the host below is an RFC 2606 reserved ``.invalid`` domain
and the credentials are obviously-fake test placeholders.
"""

from __future__ import annotations

import asyncio
import json
import os
import sys
from typing import Any, Callable, Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import pytest  # noqa: E402

from flir_ptz.control.config import CameraConfig  # noqa: E402
from flir_ptz.nexus.protocol import LOGIN_AUTH, SESSION_SAVE, NexusError  # noqa: E402
from flir_ptz.nexus.token import TokenState  # noqa: E402
from flir_ptz.nexus.session import (  # noqa: E402
    CameraSession,
    ConnectionFailure,
    EnsureControlTimeout,
    LoginError,
    TokenLost,
)


# ---------------------------------------------------------------------------
# Fakes — pure stdlib, no httpx
# ---------------------------------------------------------------------------


class FakeResponse:
    """Duck-types the tiny slice of httpx.Response CameraSession uses."""

    def __init__(self, payload: dict, status_code: int = 200):
        self._payload = payload
        self.status_code = status_code
        self.text = json.dumps(payload)

    def json(self) -> Any:
        return self._payload

    def raise_for_status(self) -> None:
        if self.status_code >= 400:
            raise RuntimeError(f"HTTP {self.status_code}")


class NexusFake:
    """A small, scriptable in-process simulation of a Nexus CGI camera.

    Not a full HTTP server, and no motion model (that's
    ``test/test_controller.py``'s in-process kinematic fake) — just
    enough state and scripting hooks to drive every CameraSession code
    path under test.
    """

    def __init__(self, *, session_id: int = 777):
        self.session_id = session_id
        self.token_holder: Optional[int] = None
        # Report this Token_ID instead of token_holder if set (simulates a
        # foreign / JCU-held token that never matches our session).
        self.foreign_token_id: Optional[int] = None

        self.basic_whoami_fail_times = 0
        self.post_login_fail_times = 0

        # action -> queue of RC codes to return (in order) before falling
        # back to normal simulated behaviour once the queue is empty.
        self.rc_script: dict[str, list[int]] = {}
        # action -> number of times to raise a connection error before
        # falling back to normal behaviour.
        self.raise_conn_error_for: dict[str, int] = {}

        self.calls: list[tuple[str, dict]] = []
        self.post_calls: list[tuple[str, Any, Any]] = []
        self.delete_calls: list[str] = []
        self._whoami_calls = 0
        self._post_auth_calls = 0

    # -- GET /Nexus.cgi ---------------------------------------------------

    def handle_get(self, params: dict) -> FakeResponse:
        action = params["action"]
        self.calls.append((action, dict(params)))

        remaining = self.raise_conn_error_for.get(action, 0)
        if remaining > 0:
            self.raise_conn_error_for[action] = remaining - 1
            raise ConnectionError(f"simulated connection failure: {action}")

        queue = self.rc_script.get(action)
        if queue:
            code = queue.pop(0)
            if code != 0:
                return FakeResponse(
                    {action: {"Return Code": code, "Return String": f"scripted {code}"}}
                )

        return FakeResponse({action: self._default_payload(action, params)})

    def _default_payload(self, action: str, params: dict) -> dict:
        if action == "SERVERWhoAmI":
            self._whoami_calls += 1
            if self._whoami_calls <= self.basic_whoami_fail_times:
                return {"Return Code": 5, "Return String": "not ready yet"}
            return {"Return Code": 0, "Id": self.session_id, "Owner": "test"}

        if action == "SERVERLastNMEAGet":
            if params.get("tokenoverride") == 1:
                self.token_holder = self.session_id
                return {"Return Code": 0, "Token_ID": self._reported_token()}
            sid = params.get("session")
            if sid != self.session_id:
                return {"Return Code": 21, "Return String": "invalid session"}
            return {"Return Code": 0, "Token_ID": self._reported_token()}

        if action == "SERVERRemoteControlRequestAsync":
            self.token_holder = self.session_id
            return {"Return Code": 0}

        if action == "SERVERRemoteControlRelease":
            self.token_holder = None
            return {"Return Code": 0}

        if action == "PTLastNMEAGet":
            return {
                "Return Code": 0,
                "Abs_Azimuth": 11.0,
                "Abs_Elevation": -2.0,
                "Geo_Azimuth": 10.0,
                "Geo_Elevation": -1.0,
                "Speed_X": 0.0,
                "Speed_Y": 0.0,
                "Mode": 0,
            }

        # PT*Set / PTAutoScan* / anything else not special-cased above.
        return {"Return Code": 0}

    def _reported_token(self) -> Optional[int]:
        if self.foreign_token_id is not None:
            return self.foreign_token_id
        return self.token_holder

    # -- POST ---------------------------------------------------------------

    def handle_post(self, url: str, json_body: Any, data_body: Any) -> FakeResponse:
        self.post_calls.append((url, json_body, data_body))
        if url == LOGIN_AUTH:
            self._post_auth_calls += 1
            if self._post_auth_calls <= self.post_login_fail_times:
                return FakeResponse({"status": "1", "message": "bad credentials"})
            return FakeResponse({"status": "0"})
        if url == SESSION_SAVE:
            return FakeResponse({"ok": True})
        return FakeResponse({})

    # -- DELETE ---------------------------------------------------------------

    def handle_delete(self, url: str) -> FakeResponse:
        self.delete_calls.append(url)
        return FakeResponse({"status": "0"})


class FakeTransport:
    """Pure-stdlib duck-type of the httpx.AsyncClient slice CameraSession
    relies on (``get`` / ``post`` / ``delete`` / ``aclose``). Tracks
    concurrent-call depth so tests can prove CameraSession serialises
    every network call through its Semaphore(1)."""

    def __init__(self, fake: NexusFake):
        self._fake = fake
        self.closed = False
        self._depth = 0
        self.max_depth = 0

    async def get(self, url: str, *, params: dict) -> FakeResponse:
        return await self._dispatch(lambda: self._fake.handle_get(params))

    async def post(self, url: str, *, json: Any = None, data: Any = None) -> FakeResponse:
        return await self._dispatch(lambda: self._fake.handle_post(url, json, data))

    async def delete(self, url: str) -> FakeResponse:
        return await self._dispatch(lambda: self._fake.handle_delete(url))

    async def aclose(self) -> None:
        self.closed = True

    async def _dispatch(self, call: Callable[[], FakeResponse]) -> FakeResponse:
        self._depth += 1
        self.max_depth = max(self.max_depth, self._depth)
        try:
            # Yield control here so that, if CameraSession ever failed to
            # serialise callers, a second concurrent dispatch could
            # observe (and bump) self._depth above 1 while this one is
            # suspended.
            await asyncio.sleep(0)
            return call()
        finally:
            self._depth -= 1


async def _instant_sleep(_seconds: float) -> None:
    """Replaces asyncio.sleep in tests so login-retry loops (which sleep
    2s between attempts by default) run instantly."""
    return None


def make_client_factory(fake: NexusFake) -> tuple[Callable[[], FakeTransport], list]:
    created: list[FakeTransport] = []

    def factory() -> FakeTransport:
        t = FakeTransport(fake)
        created.append(t)
        return t

    return factory, created


_FAKE_HOST = "http://camera.example.invalid"
_FAKE_USER = "test-user"
_FAKE_PASS = "test-pass-not-real"  # noqa: S105 - obviously-fake test placeholder


def make_session(
    *,
    login_mode: str = "basic",
    session_id: Optional[int] = None,
    fake: Optional[NexusFake] = None,
    **kwargs: Any,
) -> tuple[CameraSession, NexusFake, list]:
    fake = fake if fake is not None else NexusFake()
    factory, created = make_client_factory(fake)
    cfg = CameraConfig(
        host=_FAKE_HOST,
        username=_FAKE_USER,
        password=_FAKE_PASS,
        login_mode=login_mode,
        model="364c",
    )
    session = CameraSession(
        cfg,
        client_factory=factory,
        session_id=session_id,
        sleep=_instant_sleep,
        **kwargs,
    )
    return session, fake, created


def run(coro):
    return asyncio.run(coro)


# ---------------------------------------------------------------------------
# B1 — basic (364C) login
# ---------------------------------------------------------------------------


def test_login_basic_success():
    async def body():
        session, fake, _ = make_session(login_mode="basic")
        await session.connect()
        assert session.session_id == fake.session_id
        assert fake._whoami_calls == 1

    run(body())


def test_login_basic_retry_then_success():
    async def body():
        fake = NexusFake()
        fake.basic_whoami_fail_times = 3
        session, fake, _ = make_session(login_mode="basic", fake=fake)
        await session.connect()
        assert session.session_id == fake.session_id
        assert fake._whoami_calls == 4

    run(body())


def test_login_basic_all_attempts_fail_raises_login_error():
    async def body():
        fake = NexusFake()
        fake.basic_whoami_fail_times = 999
        session, fake, _ = make_session(
            login_mode="basic", fake=fake, login_retry_attempts=3
        )
        with pytest.raises(LoginError):
            await session.connect()
        assert fake._whoami_calls == 3

    run(body())


# ---------------------------------------------------------------------------
# B2 — post (M232) login
# ---------------------------------------------------------------------------


def test_login_post_success():
    async def body():
        session, fake, _ = make_session(login_mode="post")
        await session.connect()
        assert session.session_id == fake.session_id
        assert fake._post_auth_calls == 1
        # Best-effort session save must have been attempted.
        assert any(url == SESSION_SAVE for url, _, _ in fake.post_calls)

    run(body())


def test_login_post_status_nonzero_rejected_then_retry_then_success():
    async def body():
        fake = NexusFake()
        fake.post_login_fail_times = 2
        session, fake, _ = make_session(login_mode="post", fake=fake)
        await session.connect()
        assert session.session_id == fake.session_id
        assert fake._post_auth_calls == 3

    run(body())


def test_login_post_all_attempts_fail_raises_login_error():
    async def body():
        fake = NexusFake()
        fake.post_login_fail_times = 999
        session, fake, _ = make_session(
            login_mode="post", fake=fake, login_retry_attempts=3
        )
        with pytest.raises(LoginError):
            await session.connect()
        assert fake._post_auth_calls == 3

    run(body())


# ---------------------------------------------------------------------------
# B3 — session_id reuse
# ---------------------------------------------------------------------------


def test_session_reuse_accepted():
    async def body():
        fake = NexusFake(session_id=777)
        session, fake, _ = make_session(fake=fake, session_id=777)
        await session.connect()
        assert session.session_id == 777
        # No fresh login should have happened at all.
        assert fake._whoami_calls == 0

    run(body())


def test_session_reuse_rejected_then_relogin():
    async def body():
        fake = NexusFake(session_id=777)
        # We hand in a stale/foreign session id that doesn't match the
        # camera's canonical one -> probe must be rejected (RC=21) and a
        # fresh login performed.
        session, fake, _ = make_session(fake=fake, session_id=999)
        await session.connect()
        assert session.session_id == 777
        assert fake._whoami_calls == 1

    run(body())


# ---------------------------------------------------------------------------
# B4 — RC=21 -> relogin -> retry once
# ---------------------------------------------------------------------------


def test_rc21_triggers_one_relogin_and_one_retry():
    async def body():
        session, fake, _ = make_session(login_mode="basic")
        await session.connect()
        assert fake._whoami_calls == 1

        # First write acquires control (natural ACQUIRE dance) and
        # succeeds, driving the tracker into HELD.
        await session.set_speed(1.0, 0.0)
        assert session.token_state == TokenState.HELD

        # Script exactly one RC=21 for the *next* write.
        fake.rc_script["PTSpeedModeSet"] = [21]
        await session.set_speed(2.0, -1.0)

        speed_calls = [c for c in fake.calls if c[0] == "PTSpeedModeSet"]
        # First set_speed call (attempt) + the RC=21 attempt + the
        # successful retry = 3 total PTSpeedModeSet calls across the test.
        assert len(speed_calls) == 3
        # Exactly one extra SERVERWhoAmI call was triggered by the relogin.
        assert fake._whoami_calls == 2

    run(body())


# ---------------------------------------------------------------------------
# B5 — connection error -> reconnect -> retry once
# ---------------------------------------------------------------------------


def test_connection_error_triggers_one_reconnect_and_one_retry():
    async def body():
        session, fake, _ = make_session(login_mode="basic")
        await session.connect()
        assert fake._whoami_calls == 1

        fake.raise_conn_error_for["PTLastNMEAGet"] = 1
        sample = await session.read_nmea()

        assert sample.abs_az == 11.0
        assert fake._whoami_calls == 2  # exactly one reconnect happened
        read_calls = [c for c in fake.calls if c[0] == "PTLastNMEAGet"]
        assert len(read_calls) == 2  # failed attempt + successful retry

    run(body())


def test_connection_error_exhausted_raises_connection_failure():
    async def body():
        session, fake, _ = make_session(login_mode="basic")
        await session.connect()

        fake.raise_conn_error_for["PTLastNMEAGet"] = 2
        with pytest.raises(ConnectionFailure):
            await session.read_nmea()

        read_calls = [c for c in fake.calls if c[0] == "PTLastNMEAGet"]
        assert len(read_calls) == 2  # bounded: never more than one retry

    run(body())


# ---------------------------------------------------------------------------
# RC=15 -> revive -> retry once, and must never loop forever
# ---------------------------------------------------------------------------


def test_rc15_triggers_revive_then_retry_succeeds():
    async def body():
        session, fake, _ = make_session(login_mode="basic")
        await session.connect()
        await session.set_speed(1.0, 0.0)  # get into HELD via natural ACQUIRE
        assert session.token_state == TokenState.HELD

        fake.rc_script["PTSpeedModeSet"] = [15]
        await session.set_speed(3.0, 3.0)  # must not raise

        assert session.token_state == TokenState.HELD
        speed_calls = [c for c in fake.calls if c[0] == "PTSpeedModeSet"]
        assert len(speed_calls) == 3  # initial + RC15 attempt + retry

    run(body())


def test_rc15_persistent_does_not_loop_forever_raises_token_lost():
    async def body():
        session, fake, _ = make_session(login_mode="basic")
        await session.connect()
        await session.set_speed(1.0, 0.0)  # HELD
        assert session.token_state == TokenState.HELD

        # Both the first attempt AND the one permitted retry get RC=15.
        fake.rc_script["PTSpeedModeSet"] = [15, 15]
        with pytest.raises(TokenLost):
            await session.set_speed(9.0, 9.0)

        speed_calls = [c for c in fake.calls if c[0] == "PTSpeedModeSet"]
        # initial successful call (1) + exactly 2 more attempts for the
        # failing command = 3 total; never an unbounded number.
        assert len(speed_calls) == 3

    run(body())


def test_foreign_token_takeover_requires_reacquire_not_infinite_idle():
    """Regression guard mirroring test_token.py's stickiness tests, but at
    the session integration level: once we're HELD and a real RC=15 comes
    in, we must not be fooled by a merely-matching Token_ID into skipping
    the revive (see token.py docstring on IDLE_OURS)."""

    async def body():
        session, fake, _ = make_session(login_mode="basic")
        await session.connect()
        await session.set_speed(1.0, 0.0)
        assert session.token_state == TokenState.HELD

        fake.rc_script["PTSpeedModeSet"] = [15]
        await session.set_speed(4.0, 4.0)
        # A confirmed revive must have cleared IDLE_OURS back to HELD.
        assert session.token_state == TokenState.HELD

    run(body())


# ---------------------------------------------------------------------------
# Genuine (non-token) camera failure -> NexusError, not a hang
# ---------------------------------------------------------------------------


def test_genuine_failure_raises_nexus_error():
    async def body():
        session, fake, _ = make_session(login_mode="basic")
        await session.connect()

        fake.rc_script["PTAutoScanModeOn"] = [7]
        with pytest.raises(NexusError) as excinfo:
            await session.scan_mode_on()
        assert excinfo.value.code == 7

    run(body())


# ---------------------------------------------------------------------------
# B6 — ensure_control
# ---------------------------------------------------------------------------


def test_ensure_control_succeeds():
    async def body():
        session, fake, _ = make_session(login_mode="basic")
        await session.connect()

        await session.ensure_control(max_attempts=5, interval=0.0)
        assert session.token_state == TokenState.HELD

    run(body())


def test_ensure_control_times_out():
    async def body():
        fake = NexusFake()
        fake.foreign_token_id = 424242  # JCU holds it forever
        session, fake, _ = make_session(fake=fake, login_mode="basic")
        await session.connect()

        with pytest.raises(EnsureControlTimeout):
            await session.ensure_control(max_attempts=3, interval=0.0)

        requests = [c for c in fake.calls if c[0] == "SERVERRemoteControlRequestAsync"]
        assert len(requests) == 3  # bounded, never hangs

    run(body())


# ---------------------------------------------------------------------------
# B7 / B8 / B9 — release_control / has_control / keepalive_token
# ---------------------------------------------------------------------------


def test_release_control_ignores_nexus_error():
    async def body():
        session, fake, _ = make_session(login_mode="basic")
        await session.connect()
        fake.rc_script["SERVERRemoteControlRelease"] = [7]
        await session.release_control()  # must not raise

    run(body())


def test_release_control_marks_state_released_on_success():
    async def body():
        session, fake, _ = make_session(login_mode="basic")
        await session.connect()
        await session.set_speed(1.0, 0.0)
        assert session.token_state == TokenState.HELD
        await session.release_control()
        assert session.token_state == TokenState.RELEASED

    run(body())


def test_has_control_true_when_token_matches():
    async def body():
        session, fake, _ = make_session(login_mode="basic")
        await session.connect()
        await session.set_speed(1.0, 0.0)  # acquires
        assert await session.has_control() is True

    run(body())


def test_has_control_false_when_foreign():
    async def body():
        fake = NexusFake()
        fake.foreign_token_id = 999
        session, fake, _ = make_session(fake=fake, login_mode="basic")
        await session.connect()
        assert await session.has_control() is False

    run(body())


def test_keepalive_token_revives_and_updates_tracker():
    async def body():
        session, fake, _ = make_session(login_mode="basic")
        await session.connect()

        await session.keepalive_token()
        keepalive_calls = [
            c for c in fake.calls
            if c[0] == "SERVERLastNMEAGet" and c[1].get("tokenoverride") == 1
        ]
        assert len(keepalive_calls) == 1
        assert session.token_state == TokenState.HELD

    run(body())


# ---------------------------------------------------------------------------
# B10 / B11 / B12 — camera API surface
# ---------------------------------------------------------------------------


def test_read_nmea_set_speed_goto_stop():
    async def body():
        session, fake, _ = make_session(login_mode="basic")
        await session.connect()

        sample = await session.read_nmea()
        assert sample.abs_az == 11.0
        assert sample.geo_el == -1.0

        await session.set_speed(5.0, -5.0)
        speed_call = next(c for c in fake.calls if c[0] == "PTSpeedModeSet")
        assert speed_call[1]["Azimuth_Speed"] == 5.0
        assert speed_call[1]["Elevation_Speed"] == -5.0

        await session.goto_position(45.0, -10.0)
        goto_call = next(c for c in fake.calls if c[0] == "PTGeoAzimuthElevationSet")
        assert goto_call[1]["Geo_Azimuth"] == 45.0
        assert goto_call[1]["Geo_Elevation"] == -10.0

        await session.stop()
        zero_calls = [
            c for c in fake.calls
            if c[0] == "PTSpeedModeSet"
            and c[1]["Azimuth_Speed"] == 0.0
            and c[1]["Elevation_Speed"] == 0.0
        ]
        assert len(zero_calls) == 1

    run(body())


def test_scan_set_speed_limits_mode_on_off():
    async def body():
        session, fake, _ = make_session(login_mode="basic")
        await session.connect()

        await session.scan_set_speed(12.5)
        assert next(c for c in fake.calls if c[0] == "PTAutoScanSpeedSet")[1]["Speed"] == 12.5

        await session.scan_set_limits(30.0, -30.0)
        limits_call = next(c for c in fake.calls if c[0] == "PTAutoScanLimitsSet")
        assert limits_call[1]["Left_Azimuth"] == 30.0
        assert limits_call[1]["Right_Azimuth"] == -30.0

        await session.scan_mode_on()
        assert any(c[0] == "PTAutoScanModeOn" for c in fake.calls)

        await session.scan_mode_off()
        assert any(c[0] == "PTAutoScanModeOff" for c in fake.calls)

    run(body())


def test_scan_set_limits_normalizes_azimuth():
    async def body():
        session, fake, _ = make_session(login_mode="basic")
        await session.connect()

        await session.scan_set_limits(200.0, -200.0)
        limits_call = next(c for c in fake.calls if c[0] == "PTAutoScanLimitsSet")
        assert limits_call[1]["Left_Azimuth"] == pytest.approx(-160.0)
        assert limits_call[1]["Right_Azimuth"] == pytest.approx(160.0)

    run(body())


# ---------------------------------------------------------------------------
# B13 — single transport reused (no wasted camera session slots)
# ---------------------------------------------------------------------------


def test_single_client_reused_across_calls():
    async def body():
        session, fake, created = make_session(login_mode="basic")
        await session.connect()
        await session.read_nmea()
        await session.set_speed(1.0, 1.0)
        await session.server_status()
        assert len(created) == 1

    run(body())


def test_logout_releases_session_slot_and_closes_transport():
    async def body():
        session, fake, created = make_session(login_mode="basic")
        await session.connect()
        transport = created[-1]

        await session.logout()

        assert fake.delete_calls == [LOGIN_AUTH]
        assert transport.closed is True

    run(body())


# ---------------------------------------------------------------------------
# Read-only actions never touch the token
# ---------------------------------------------------------------------------


def test_read_only_actions_never_trigger_token_acquisition():
    async def body():
        session, fake, _ = make_session(login_mode="basic")
        await session.connect()
        assert session.token_state == TokenState.UNKNOWN

        await session.read_nmea()
        await session.server_status()

        assert not any(
            c[0] == "SERVERRemoteControlRequestAsync" for c in fake.calls
        )
        assert session.token_state == TokenState.UNKNOWN

    run(body())


# ---------------------------------------------------------------------------
# Concurrency: every network call is serialised
# ---------------------------------------------------------------------------


def test_concurrent_callers_are_serialized():
    async def body():
        session, fake, created = make_session(login_mode="basic")
        await session.connect()

        await asyncio.gather(
            session.read_nmea(),
            session.set_speed(1.0, 1.0),
            session.read_nmea(),
            session.server_status(),
        )

        transport = created[-1]
        assert transport.max_depth == 1

    run(body())


# ---------------------------------------------------------------------------
# Module-level sanity: importable without httpx
# ---------------------------------------------------------------------------


def test_module_imports_without_httpx_installed():
    """The guarantee: session.py loads even where httpx is unavailable.

    This must test the CODE, not the machine. An earlier version asserted
    `find_spec("httpx") is None`, i.e. that the environment happened to lack
    httpx — so it started failing the moment httpx was installed, despite the
    property under test still holding. Here httpx is actively masked instead,
    in a subprocess so no import-state leaks into the rest of the suite.
    """
    import subprocess
    import sys as _sys
    import textwrap

    code = textwrap.dedent(
        """
        import sys
        sys.modules["httpx"] = None   # makes `import httpx` raise ImportError
        import flir_ptz.nexus.session as s
        assert s.httpx is None, "guarded import should leave httpx as None"
        assert hasattr(s, "CameraSession")
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
