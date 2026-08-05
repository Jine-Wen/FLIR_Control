#!/usr/bin/env python3
"""Async camera-session layer for the FLIR Nexus CGI API.

This is the **only** module in the whole project allowed to ``import
httpx`` (spec ARCHITECTURE.md sec. 1/8). The import is guarded so that
``import flir_ptz.nexus.session`` always succeeds even when httpx is not
installed — the real HTTP transport is only ever touched lazily, inside
:meth:`CameraSession._default_client_factory`, and only if the caller did
not inject a ``client_factory`` of their own.

Testability (hard requirement, see the worker brief): the transport is
fully dependency-injected. ``CameraSession`` never constructs an
``httpx.AsyncClient`` unless nothing was injected. Tests pass a
hand-written, pure-stdlib async fake (see ``test/test_session.py``) via
``client_factory=lambda: fake`` and this module never touches httpx at
all in that mode — so the whole test file runs with httpx absent.

The transport object handed back by ``client_factory()`` only needs to
duck-type a tiny slice of ``httpx.AsyncClient``'s interface:

    async def get(self, url: str, *, params: dict) -> Response: ...
    async def post(self, url: str, *, json=None, data=None) -> Response: ...
    async def delete(self, url: str) -> Response: ...
    async def aclose(self) -> None: ...

and ``Response`` only needs ``.text: str``, ``.json() -> Any`` and
(optionally) ``.raise_for_status() -> None``. A real ``httpx.AsyncClient``
/ ``httpx.Response`` already satisfies this exactly.

Token bookkeeping is delegated entirely to
:class:`flir_ptz.nexus.token.TokenTracker` — this module drives it
(``required_before`` / ``observe_rc`` / ``note_token_acquired`` /
``note_command``) but owns none of the policy itself. Per
ARCHITECTURE.md sec. 4.1, the control plane must never see raw token
errors: :meth:`CameraSession.execute` either returns the successful
payload or raises :class:`TokenLost` — never a bare RC=15/21
:class:`~flir_ptz.nexus.protocol.NexusError`.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Any, Awaitable, Callable, Mapping, Optional

logger = logging.getLogger("flir_ptz.nexus.session")

from flir_ptz.control.config import CameraConfig
from flir_ptz.control.profiles import normalize_az
from flir_ptz.nexus.protocol import (
    LOGIN_AUTH,
    NEXUS_CGI,
    SESSION_SAVE,
    RC,
    NexusError,
    PtSample,
    build_query,
    classify,
    normalize_base_url,
    parse_nmea,
    unwrap,
)
from flir_ptz.nexus.token import (
    TokenAction,
    TokenPolicy,
    TokenState,
    TokenTracker,
    needs_token,
)

# httpx is only imported (lazily, guarded) so that this module — and every
# test that injects its own transport — imports cleanly on a machine where
# httpx is not installed (spec sec. 1: "純函式模組不得 import httpx"; this
# module is the one exception, but even here the import must not be
# required merely to *load* the module).
try:
    import httpx  # type: ignore
except ImportError:  # pragma: no cover - exercised on this dev machine
    httpx = None  # type: ignore


if httpx is not None:  # pragma: no cover - depends on local environment
    _CONNECTION_ERRORS: tuple = (
        httpx.RemoteProtocolError,
        httpx.ConnectError,
        httpx.ReadError,
        httpx.TimeoutException,
        asyncio.TimeoutError,
        ConnectionError,
    )
else:
    # Pure-stdlib fallback so fakes (and this whole test suite) can
    # simulate a connection failure with nothing but builtin exceptions.
    _CONNECTION_ERRORS = (asyncio.TimeoutError, ConnectionError)


# ---------------------------------------------------------------------------
# Typed outcomes — the control plane only ever sees these, never raw RC codes
# ---------------------------------------------------------------------------


class SessionError(Exception):
    """Base class for CameraSession-level failures."""


class LoginError(SessionError):
    """Authentication could not be established after all retry attempts."""


class ConnectionFailure(SessionError):
    """A request failed with a connection-level error even after the one
    permitted reconnect+retry (PARITY B5)."""


class EnsureControlTimeout(SessionError):
    """``ensure_control`` polled until ``max_attempts`` without the camera
    ever reporting our session as the Token_ID holder."""


class TokenLost(SessionError):
    """The control token could not be secured or recovered, even after the
    one permitted acquire/revive/relogin + retry.

    Per ARCHITECTURE.md sec. 4.1 this is the *only* token-shaped failure
    the control plane ever sees from :meth:`CameraSession.execute` — no
    raw RC=15/RC=21 ever escapes this module.
    """

    def __init__(self, cmd: str, code: int, msg: str) -> None:
        super().__init__(f"{cmd}: token lost/unavailable [{code}] {msg}")
        self.cmd = cmd
        self.code = code
        self.msg = msg


class CameraSession:
    """Async client over ``GET /Nexus.cgi``, built on an injectable transport.

    Reuses a single transport/client for the object's lifetime (PARITY
    B13) and serialises every network call through one
    ``asyncio.Semaphore(1)`` (spec sec. 4.1 / old client's
    ``_request_sem``) so concurrent callers can never interleave requests
    against the single serial camera resource.
    """

    def __init__(
        self,
        config: CameraConfig,
        *,
        client_factory: Optional[Callable[[], Any]] = None,
        device_id: int = 0,
        timeout: float = 8.0,
        session_id: Optional[int] = None,
        token_policy: TokenPolicy = TokenPolicy(),
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], Awaitable[None]] = asyncio.sleep,
        login_retry_attempts: int = 10,
        login_retry_delay: float = 2.0,
    ) -> None:
        self._config = config
        self._device_id = device_id
        self._timeout = timeout
        self._session_id = session_id
        self._tracker = TokenTracker(token_policy)
        self._clock = clock
        self._sleep = sleep
        self._login_retry_attempts = login_retry_attempts
        self._login_retry_delay = login_retry_delay

        # Dependency-injected transport factory — this is the seam that
        # lets the whole test suite run without httpx installed. Real
        # (non-test) callers simply omit client_factory and get a lazily
        # constructed httpx.AsyncClient the first time one is needed.
        self._client_factory: Callable[[], Any] = client_factory or self._default_client_factory
        self._client: Optional[Any] = None

        self._sem = asyncio.Semaphore(1)

    # -- properties ----------------------------------------------------

    @property
    def session_id(self) -> Optional[int]:
        return self._session_id

    @property
    def token_state(self) -> TokenState:
        return self._tracker.state

    @property
    def _login_mode(self) -> str:
        return (self._config.login_mode or "basic").lower()

    # -- transport construction -----------------------------------------

    def _default_client_factory(self) -> Any:
        if httpx is None:
            raise RuntimeError(
                "httpx is not installed; either `apt install python3-httpx` "
                "or pass client_factory=... to CameraSession() (used for "
                "testing/offline use)."
            )
        auth = (
            (self._config.username, self._config.password)
            if self._login_mode == "basic"
            else None
        )
        return httpx.AsyncClient(
            base_url=normalize_base_url(self._config.host),
            auth=auth,
            timeout=httpx.Timeout(self._timeout),
            follow_redirects=True,
            limits=httpx.Limits(max_connections=4, max_keepalive_connections=4),
        )

    @staticmethod
    async def _safe_close(client: Any) -> None:
        try:
            await client.aclose()
        except Exception:
            pass

    # -- connection management -------------------------------------------

    async def connect(self) -> None:
        """Establish the HTTP session, reusing an existing ``session_id``
        (probed via ``SERVERLastNMEAGet``, PARITY B3) if one was passed in
        and it is still valid; otherwise perform a fresh login (B1/B2)."""
        async with self._sem:
            await self._connect_locked()

    async def _connect_locked(self) -> None:
        if self._client is not None:
            await self._safe_close(self._client)
        self._client = self._client_factory()

        if self._session_id is not None:
            if await self._try_reuse_session():
                return
            self._session_id = None

        await self._login()

    async def _try_reuse_session(self) -> bool:
        try:
            raw = await self._raw_get(
                "SERVERLastNMEAGet", {}, session_override=self._session_id
            )
        except Exception:
            return False
        payload = unwrap(raw, "SERVERLastNMEAGet")
        code, _ = classify(payload)
        return code == 0

    async def _reconnect_locked(self) -> None:
        """Re-create the transport and re-authenticate from scratch.
        Called on RC=21 (RELOGIN) or on a connection error — never tries
        to reuse the old session_id (old client behaviour, lines 213-227)."""
        if self._client is not None:
            await self._safe_close(self._client)
        self._session_id = None
        self._client = self._client_factory()
        await self._login()

    async def _login(self) -> None:
        if self._login_mode == "post":
            await self._login_post()
        else:
            await self._login_basic()

    async def _login_basic(self) -> None:
        """364C — Basic Auth is carried by the client itself; just ask
        SERVERWhoAmI for the numeric session id (old client lines 183-211).

        Every attempt's failure reason is recorded and surfaced in the final
        :class:`LoginError`. Swallowing them (as the original did) makes a
        misconfigured host, a wrong password and an unplugged cable all look
        identical from the outside: a node that simply sits there for the
        whole retry budget saying nothing.
        """
        last: str = "no attempt completed"
        for attempt in range(self._login_retry_attempts):
            try:
                raw = await self._raw_get("SERVERWhoAmI", {}, no_session=True)
                who = unwrap(raw, "SERVERWhoAmI")
                code, msg = classify(who)
                new_id = who.get("Id")
                if code == 0 and new_id is not None:
                    self._session_id = int(new_id)
                    return
                last = f"RC={code} {msg!r} Id={new_id!r}"
            except Exception as exc:
                last = f"{type(exc).__name__}: {exc}"
            logger.warning(
                "basic login attempt %d/%d failed: %s",
                attempt + 1, self._login_retry_attempts, last,
            )
            if attempt < self._login_retry_attempts - 1:
                await self._sleep(self._login_retry_delay)
        raise LoginError(
            f"basic (364C) login failed after {self._login_retry_attempts} "
            f"attempts; last failure: {last}"
        )

    async def _login_post(self) -> None:
        """M232 — POST JSON credentials, then SERVERWhoAmI, then a
        best-effort session save (old client lines 126-181)."""
        logged_in = False
        for attempt in range(self._login_retry_attempts):
            try:
                resp = await self._client.post(
                    LOGIN_AUTH,
                    json={
                        "username": self._config.username,
                        "password": self._config.password,
                    },
                )
                if hasattr(resp, "raise_for_status"):
                    resp.raise_for_status()
                body = resp.json() if getattr(resp, "text", "") else {}
                if str(body.get("status", "")) == "0":
                    logged_in = True
                    break
            except Exception:
                pass
            if attempt < self._login_retry_attempts - 1:
                await self._sleep(self._login_retry_delay)

        if not logged_in:
            raise LoginError(
                f"POST {LOGIN_AUTH} login failed after "
                f"{self._login_retry_attempts} attempts"
            )

        raw = await self._raw_get("SERVERWhoAmI", {}, no_session=True)
        who = unwrap(raw, "SERVERWhoAmI")
        code, _ = classify(who)
        new_id = who.get("Id")
        if code != 0 or new_id is None:
            raise LoginError("SERVERWhoAmI after POST login returned no Id")
        self._session_id = int(new_id)

        # Best-effort session save — a failure here must not fail login.
        try:
            await self._client.post(
                SESSION_SAVE, data={"ses[cgisession]": str(self._session_id)}
            )
        except Exception:
            pass

    async def logout(self) -> None:
        """Release the camera-side PHP session slot (old node line 1417,
        ``DELETE /api/login/auth``) and close the transport. Best-effort:
        errors are swallowed, matching the old shutdown handler."""
        async with self._sem:
            if self._client is not None:
                try:
                    await self._client.delete(LOGIN_AUTH)
                except Exception:
                    pass
            await self._close_locked()

    async def close(self) -> None:
        """Close the transport without touching the camera-side session
        slot (use :meth:`logout` for that)."""
        async with self._sem:
            await self._close_locked()

    async def _close_locked(self) -> None:
        if self._client is not None:
            await self._safe_close(self._client)
            self._client = None

    # -- low-level HTTP ----------------------------------------------------

    async def _raw_get(
        self,
        action: str,
        params: Mapping[str, Any],
        *,
        session_override: Optional[int] = None,
        no_session: bool = False,
    ) -> dict:
        """Issue one raw GET. ``no_session=True`` omits the ``session``
        parameter entirely, which is required for the pre-login
        ``SERVERWhoAmI`` call — the camera rejects ``session=0``
        (see :func:`build_query`)."""
        if self._client is None:
            raise SessionError("connect() has not been called")

        sid = (
            None
            if no_session
            else (session_override if session_override is not None else self._session_id)
        )
        ts_ms = int(time.time() * 1000)
        query = build_query(action, params, sid, self._device_id, ts_ms)

        resp = await asyncio.wait_for(
            self._client.get(NEXUS_CGI, params=query), timeout=self._timeout + 2.0
        )
        if hasattr(resp, "raise_for_status"):
            resp.raise_for_status()
        return self._parse_response(resp)

    @staticmethod
    def _parse_response(resp: Any) -> dict:
        try:
            raw = resp.json()
            if isinstance(raw, dict):
                return raw
        except Exception:
            pass
        from flir_ptz.nexus.protocol import extract_json

        return extract_json(getattr(resp, "text", ""))

    # -- execute(): the token-aware command path --------------------------

    async def execute(self, action: str, params: Optional[Mapping[str, Any]] = None) -> dict:
        """Send one Nexus command with full token-tracker integration.

        Returns the unwrapped payload dict on success. Raises
        :class:`TokenLost` (never a raw NexusError with RC 15/21) if the
        token could not be secured/recovered, :class:`~flir_ptz.nexus.
        protocol.NexusError` for any other camera-reported failure, and
        :class:`ConnectionFailure` if the connection could not be
        recovered after one reconnect+retry. Exactly one retry is ever
        performed, whatever the trigger — so a persistent RC=15 (or a
        persistent connection error) can never loop forever.
        """
        async with self._sem:
            return await self._execute_locked(action, dict(params or {}))

    async def _execute_locked(self, action: str, params: dict) -> dict:
        if self._client is None:
            raise SessionError("connect() has not been called")

        proactive = self._tracker.required_before(action, self._clock())
        if proactive != TokenAction.NONE:
            await self._apply_token_action_locked(proactive, cmd=action)

        for attempt in range(2):
            try:
                raw = await self._raw_get(action, params)
            except _CONNECTION_ERRORS as exc:
                if attempt == 0:
                    await self._reconnect_locked()
                    continue
                raise ConnectionFailure(
                    f"{action}: connection failed after reconnect+retry"
                ) from exc

            payload = unwrap(raw, action)
            code, msg = classify(payload)

            if code is None or code == 0:
                if needs_token(action):
                    self._tracker.note_command(self._clock())
                return payload

            reactive = self._tracker.observe_rc(code, self._clock())
            if reactive in (TokenAction.REVIVE, TokenAction.RELOGIN) and attempt == 0:
                await self._apply_token_action_locked(reactive, cmd=action)
                continue

            if code in (RC.TOKEN_IDLE, RC.NOT_AUTH):
                raise TokenLost(action, code, str(msg))

            raise NexusError(action, code, str(msg))

        # Unreachable in practice (every branch above returns/raises/continues
        # exactly twice), kept as a defensive backstop against infinite loops.
        raise TokenLost(action, -1, "retry exhausted")

    async def _apply_token_action_locked(self, action: TokenAction, *, cmd: str) -> None:
        """Perform ACQUIRE / REVIVE / RELOGIN and, on genuine success, tell
        the tracker via ``note_token_acquired`` — the only thing that
        clears the sticky IDLE_OURS state (see token.py docstring). Any
        failure collapses to :class:`TokenLost` so callers only ever have
        to handle one token-shaped exception."""
        try:
            if action == TokenAction.ACQUIRE:
                await self._ensure_control_locked()
                self._tracker.note_token_acquired(self._clock())
            elif action == TokenAction.REVIVE:
                raw = await self._raw_get("SERVERLastNMEAGet", {"tokenoverride": 1})
                payload = unwrap(raw, "SERVERLastNMEAGet")
                code, msg = classify(payload)
                if code not in (None, 0):
                    raise NexusError("SERVERLastNMEAGet", code, msg)
                self._tracker.note_token_acquired(self._clock())
            elif action == TokenAction.RELOGIN:
                await self._reconnect_locked()
            # NONE / KEEPALIVE / RELEASE are not driven from execute().
        except _CONNECTION_ERRORS as exc:
            raise TokenLost(cmd, -1, f"{action.name} failed: connection error: {exc}") from exc
        except NexusError as exc:
            raise TokenLost(cmd, exc.code, f"{action.name} failed: {exc.msg}") from exc
        except SessionError as exc:
            raise TokenLost(cmd, -1, f"{action.name} failed: {exc}") from exc

    # -- ensure_control (PARITY B6) ----------------------------------------

    async def ensure_control(self, max_attempts: int = 120, interval: float = 0.2) -> None:
        """Poll ``SERVERRemoteControlRequestAsync(Forced=1)`` until
        ``Token_ID`` matches our session (default timeout 120*0.2s = 24s,
        matching the old client). Raises :class:`EnsureControlTimeout` if
        it gives up, never hangs indefinitely."""
        async with self._sem:
            await self._ensure_control_locked(max_attempts, interval)

    async def _ensure_control_locked(self, max_attempts: int = 120, interval: float = 0.2) -> None:
        if self._session_id is None:
            raise SessionError("ensure_control: not connected")

        tid: Optional[int] = None
        for attempt in range(1, max_attempts + 1):
            raw = await self._raw_get("SERVERRemoteControlRequestAsync", {"Forced": 1})
            payload = unwrap(raw, "SERVERRemoteControlRequestAsync")
            code, msg = classify(payload)
            if code not in (None, 0):
                raise NexusError("SERVERRemoteControlRequestAsync", code, msg)

            await self._sleep(interval)

            raw2 = await self._raw_get("SERVERLastNMEAGet", {})
            status = unwrap(raw2, "SERVERLastNMEAGet")
            tid_raw = status.get("Token_ID")
            tid = int(tid_raw) if tid_raw is not None else None
            self._tracker.observe_token_id(tid, self._session_id, self._clock())
            if tid == self._session_id:
                return

        raise EnsureControlTimeout(
            f"ensure_control: gave up after {max_attempts} attempts "
            f"(session={self._session_id}, last Token_ID={tid})"
        )

    # -- camera API (PARITY B7-B12) -----------------------------------------

    async def read_nmea(self) -> PtSample:
        payload = await self.execute("PTLastNMEAGet")
        return parse_nmea(payload)

    async def set_speed(self, az_speed: float, el_speed: float) -> None:
        await self.execute(
            "PTSpeedModeSet",
            {"Azimuth_Speed": az_speed, "Elevation_Speed": el_speed},
        )

    async def goto_position(self, geo_az: float, geo_el: float) -> None:
        await self.execute(
            "PTGeoAzimuthElevationSet",
            {"Geo_Azimuth": geo_az, "Geo_Elevation": geo_el},
        )

    async def stop(self) -> None:
        await self.set_speed(0.0, 0.0)

    async def scan_set_speed(self, speed: float) -> None:
        await self.execute("PTAutoScanSpeedSet", {"Speed": speed})

    async def scan_set_limits(self, left_az: float, right_az: float) -> None:
        # PARITY B12 — normalize into (-180, 180]. Reuses the already
        # hardened flir_ptz.control.profiles.normalize_az (fmod-based, no
        # unbounded while-loop) rather than re-implementing it here.
        await self.execute(
            "PTAutoScanLimitsSet",
            {
                "Left_Azimuth": normalize_az(left_az),
                "Right_Azimuth": normalize_az(right_az),
            },
        )

    async def scan_mode_on(self) -> None:
        await self.execute("PTAutoScanModeOn")

    async def scan_mode_off(self) -> None:
        await self.execute("PTAutoScanModeOff")

    async def server_status(self) -> dict:
        return await self.execute("SERVERLastNMEAGet")

    async def has_control(self) -> bool:
        status = await self.execute("SERVERLastNMEAGet")
        token = status.get("Token_ID")
        if token is None or self._session_id is None:
            return False
        return int(token) == self._session_id

    async def release_control(self) -> None:
        """Best-effort release so the JCU/Web can take over without
        Forced=1 (old client lines 417-426). RC=7 ("not supported"/"no
        token held") and any other failure are ignored, matching the old
        client exactly."""
        try:
            await self.execute("SERVERRemoteControlRelease")
            self._tracker.state = TokenState.RELEASED
        except Exception:
            pass

    async def keepalive_token(self) -> None:
        """Non-destructive keepalive: ``SERVERLastNMEAGet?tokenoverride=1``
        — exactly what the official FLIR web UI does (old client lines
        394-398). Bypasses ``execute()``'s proactive token check
        deliberately: this call *is* the revive mechanism, driven by the
        controller's own ``TokenTracker.periodic()`` decision, not by a
        write command."""
        async with self._sem:
            raw = await self._raw_get("SERVERLastNMEAGet", {"tokenoverride": 1})
            payload = unwrap(raw, "SERVERLastNMEAGet")
            code, _ = classify(payload)
            tid = payload.get("Token_ID")
            self._tracker.observe_token_id(
                int(tid) if tid is not None else None, self._session_id, self._clock()
            )
            if code == 0:
                self._tracker.note_token_acquired(self._clock())
