#!/usr/bin/env python3
"""Tests for the EO (daylight) zoom methods on flir_ptz.nexus.session.CameraSession:
``zoom_in`` / ``zoom_out`` / ``zoom_stop`` / ``read_dltv``.

Same seam as test_session.py (whose fakes this file deliberately does not
import, to keep each test file collection-order-independent): a small,
hand-written, pure-stdlib async fake duck-typing the tiny slice of
httpx.AsyncClient CameraSession actually calls. Must run with no ROS, no
camera, no httpx installed.

No real IP address or password appears anywhere in this file (spec sec. 8
rule 1) -- the host is an RFC 2606 reserved ``.invalid`` domain and the
credentials are obviously-fake test placeholders.
"""

from __future__ import annotations

import asyncio
import json
import os
import sys
from typing import Any, Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import pytest  # noqa: E402

from flir_ptz.control.config import CameraConfig  # noqa: E402
from flir_ptz.nexus.protocol import DltvSample  # noqa: E402
from flir_ptz.nexus.token import TokenState  # noqa: E402
from flir_ptz.nexus.session import CameraSession  # noqa: E402

_FAKE_HOST = "http://camera.example.invalid"
_FAKE_USER = "test-user"
_FAKE_PASS = "test-pass-not-real"  # noqa: S105 - obviously-fake test placeholder


class _FakeResponse:
    def __init__(self, payload: dict) -> None:
        self._payload = payload
        self.text = json.dumps(payload)

    def json(self) -> Any:
        return self._payload

    def raise_for_status(self) -> None:
        pass


class ZoomNexusFake:
    """Just enough of the Nexus wire protocol to drive zoom_in/out/stop and
    read_dltv through CameraSession.execute()'s full token-aware path:
    login, SERVERRemoteControlRequestAsync (ACQUIRE), the DLTVZoom* actions
    themselves, and DLTVLastNMEAGet."""

    def __init__(self, *, session_id: int = 321) -> None:
        self.session_id = session_id
        self.token_holder: Optional[int] = None
        self.zoom_calls: list[str] = []  # "in" | "out" | "stop", in call order
        self.zoom_mm = 41.75
        self.zoom_pctg = 18.59
        self.focus_pctg = 50.0
        self.autofocus = 1.0
        self.calls: list[tuple[str, dict]] = []

    def handle_get(self, params: dict) -> _FakeResponse:
        action = params["action"]
        self.calls.append((action, dict(params)))
        sid = params.get("session")

        if action == "SERVERWhoAmI":
            return _FakeResponse({action: {"Return Code": 0, "Id": self.session_id, "Owner": "test"}})

        if action == "SERVERRemoteControlRequestAsync":
            self.token_holder = self.session_id
            return _FakeResponse({action: {"Return Code": 0, "Token_ID": self.token_holder}})

        if action == "SERVERLastNMEAGet":
            return _FakeResponse({action: {"Return Code": 0, "Token_ID": self.token_holder or 0}})

        if action == "DLTVZoomCountsIncrement":
            self.zoom_calls.append("in")
            return _FakeResponse({action: {"Return Code": 0}})

        if action == "DLTVZoomCountsDecrement":
            self.zoom_calls.append("out")
            return _FakeResponse({action: {"Return Code": 0}})

        if action == "DLTVZoomStop":
            self.zoom_calls.append("stop")
            return _FakeResponse({action: {"Return Code": 0}})

        if action == "DLTVLastNMEAGet":
            return _FakeResponse({
                action: {
                    "Return Code": 0,
                    "Zoom": self.zoom_mm,
                    "Zoom_pctg": self.zoom_pctg,
                    "Focus_pctg": self.focus_pctg,
                    "Autofocus": self.autofocus,
                }
            })

        return _FakeResponse({action: {"Return Code": 0}})


class FakeTransport:
    def __init__(self, fake: ZoomNexusFake) -> None:
        self._fake = fake
        self.closed = False

    async def get(self, url: str, *, params: dict) -> _FakeResponse:
        await asyncio.sleep(0)
        return self._fake.handle_get(params)

    async def post(self, url: str, *, json: Any = None, data: Any = None) -> _FakeResponse:
        await asyncio.sleep(0)
        return _FakeResponse({})

    async def delete(self, url: str) -> _FakeResponse:
        await asyncio.sleep(0)
        return _FakeResponse({})

    async def aclose(self) -> None:
        self.closed = True


def make_session(fake: Optional[ZoomNexusFake] = None) -> tuple[CameraSession, ZoomNexusFake]:
    fake = fake if fake is not None else ZoomNexusFake()
    cfg = CameraConfig(host=_FAKE_HOST, username=_FAKE_USER, password=_FAKE_PASS, login_mode="basic", model="364c")
    session = CameraSession(cfg, client_factory=lambda: FakeTransport(fake))
    return session, fake


def run(coro):
    return asyncio.run(coro)


# ---------------------------------------------------------------------------
# zoom_in / zoom_out / zoom_stop issue the right actions through the
# token-aware execute() path -- not bypassed.
# ---------------------------------------------------------------------------


def test_zoom_in_sends_dltv_increment():
    async def body():
        session, fake = make_session()
        await session.connect()
        await session.zoom_in()
        assert fake.zoom_calls == ["in"]
        assert any(c[0] == "DLTVZoomCountsIncrement" for c in fake.calls)

    run(body())


def test_zoom_out_sends_dltv_decrement():
    async def body():
        session, fake = make_session()
        await session.connect()
        await session.zoom_out()
        assert fake.zoom_calls == ["out"]
        assert any(c[0] == "DLTVZoomCountsDecrement" for c in fake.calls)

    run(body())


def test_zoom_stop_sends_dltv_stop():
    async def body():
        session, fake = make_session()
        await session.connect()
        await session.zoom_stop()
        assert fake.zoom_calls == ["stop"]
        assert any(c[0] == "DLTVZoomStop" for c in fake.calls)

    run(body())


# ---------------------------------------------------------------------------
# needs_token is honoured: zoom_in/out/stop must acquire control FIRST,
# exactly like a PT*Set command -- never bypass the token-aware path.
# ---------------------------------------------------------------------------


def test_zoom_in_acquires_control_token_before_sending():
    async def body():
        session, fake = make_session()
        await session.connect()
        assert session.token_state == TokenState.UNKNOWN
        assert fake.token_holder is None

        await session.zoom_in()

        # The token must have been acquired (SERVERRemoteControlRequestAsync)
        # BEFORE the zoom command reached the camera -- not after, and not
        # skipped entirely.
        actions = [c[0] for c in fake.calls]
        acquire_idx = actions.index("SERVERRemoteControlRequestAsync")
        zoom_idx = actions.index("DLTVZoomCountsIncrement")
        assert acquire_idx < zoom_idx
        assert session.token_state == TokenState.HELD
        assert fake.token_holder == fake.session_id

    run(body())


def test_zoom_stop_also_requires_the_token_not_bypassed():
    """DLTVZoomStop does not end in "Get" and isn't a SERVERRemoteControl*
    call, so token.needs_token()'s deny-by-default rule requires it too --
    verify session.py doesn't special-case "stop" to skip execute()'s
    token-aware path."""

    async def body():
        session, fake = make_session()
        await session.connect()
        assert fake.token_holder is None

        await session.zoom_stop()

        assert any(c[0] == "SERVERRemoteControlRequestAsync" for c in fake.calls)
        assert fake.zoom_calls == ["stop"]

    run(body())


def test_second_zoom_command_does_not_reacquire_once_held():
    async def body():
        session, fake = make_session()
        await session.connect()
        await session.zoom_in()
        acquire_count_after_first = sum(1 for c in fake.calls if c[0] == "SERVERRemoteControlRequestAsync")

        await session.zoom_out()
        acquire_count_after_second = sum(1 for c in fake.calls if c[0] == "SERVERRemoteControlRequestAsync")

        assert acquire_count_after_first == 1
        assert acquire_count_after_second == 1  # still held -- no redundant re-acquire
        assert fake.zoom_calls == ["in", "out"]

    run(body())


# ---------------------------------------------------------------------------
# read_dltv: a plain read, never touches the token (ends in "Get").
# ---------------------------------------------------------------------------


def test_read_dltv_parses_measured_field_shape():
    async def body():
        fake = ZoomNexusFake()
        fake.zoom_mm = 41.75
        fake.zoom_pctg = 18.59
        fake.focus_pctg = 63.2
        fake.autofocus = 1.0
        session, fake = make_session(fake)
        await session.connect()

        sample = await session.read_dltv()

        assert sample == DltvSample(zoom=41.75, zoom_pctg=18.59, focus_pctg=63.2, autofocus=1.0)

    run(body())


def test_read_dltv_never_requires_the_control_token():
    async def body():
        session, fake = make_session()
        await session.connect()
        assert session.token_state == TokenState.UNKNOWN

        await session.read_dltv()

        assert not any(c[0] == "SERVERRemoteControlRequestAsync" for c in fake.calls)
        assert session.token_state == TokenState.UNKNOWN

    run(body())


def test_read_dltv_reflects_measured_zoom_pctg_change_in_then_out():
    """Mirrors the hardware measurement in the worker brief: Zoom_pctg went
    18.59 -> 43.49 (in) -> 16.80 (out)."""

    async def body():
        session, fake = make_session()
        await session.connect()

        fake.zoom_pctg = 18.59
        assert (await session.read_dltv()).zoom_pctg == pytest.approx(18.59)

        fake.zoom_pctg = 43.49
        assert (await session.read_dltv()).zoom_pctg == pytest.approx(43.49)

        fake.zoom_pctg = 16.80
        assert (await session.read_dltv()).zoom_pctg == pytest.approx(16.80)

    run(body())
