#!/usr/bin/env python3
"""Pure parsing and URL/query construction for the FLIR Nexus CGI protocol.

No I/O, no async, no rclpy, and — critically — no ``httpx``. Only
``nexus/session.py`` is allowed to import httpx (spec sec. 1/8); this module
must import cleanly with nothing but the Python 3.12 standard library.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Mapping, Optional

NEXUS_CGI = "/Nexus.cgi"
LOGIN_AUTH = "/api/login/auth"
SESSION_SAVE = "/api/login/session/save"


class RC(IntEnum):
    """Nexus CGI "Return Code" values that the control plane needs to react to."""

    OK = 0
    TOKEN_IDLE = 15  # token is still ours but the camera let it go idle
    NOT_AUTH = 21  # session invalid — must re-login


@dataclass(frozen=True)
class PtSample:
    """A single parsed PTLastNMEAGet / SERVERLastNMEAGet-style sample."""

    abs_az: float
    abs_el: float
    geo_az: float
    geo_el: float
    speed_x: float
    speed_y: float
    mode: int

    @property
    def speed_mag(self) -> float:
        return max(abs(self.speed_x), abs(self.speed_y))


class NexusError(Exception):
    """Raised when the camera returns a non-zero Return Code."""

    def __init__(self, cmd: str, code: int, msg: str) -> None:
        super().__init__(f"{cmd} failed: [{code}] {msg}")
        self.cmd = cmd
        self.code = code
        self.msg = msg


def build_query(
    action: str,
    params: Mapping[str, Any],
    session_id: Optional[int],
    device_id: int,
    ts_ms: int,
) -> dict[str, Any]:
    """Assemble the GET /Nexus.cgi query dict for one CGI call.

    ``session_id=None`` means "we do not have a session yet" and OMITS the
    ``session`` parameter entirely. Do not substitute ``0`` for it: verified
    against a FLIR 364C, the camera treats ``session=0`` as an invalid
    session and rejects the request, while omitting the parameter succeeds::

        ?action=SERVERWhoAmI&DeviceID=0            -> {"SERVERWhoAmI": {"Return Code": 0, "Id": 103, ...}}
        ?action=SERVERWhoAmI&session=0&DeviceID=0  -> {"error": {"Return Code": 21, "Return String": "Network client not authorized"}}

    Since ``SERVERWhoAmI`` is exactly the call used to obtain the session id
    in the first place, sending ``session=0`` there makes login impossible.
    """
    query: dict[str, Any] = {"action": action}
    query.update(params)
    if session_id is not None:
        query["session"] = session_id
    query["DeviceID"] = device_id
    query["_"] = ts_ms
    return query


def _find_matching_brace(text: str, start: int) -> Optional[int]:
    """Return the index of the ``}`` that closes the ``{`` at ``start``.

    String-literal aware: braces inside JSON string values (including
    escaped quotes) do not affect nesting depth. Returns ``None`` if no
    matching close brace is found.
    """
    depth = 0
    in_string = False
    escape = False
    n = len(text)
    i = start
    while i < n:
        ch = text[i]
        if in_string:
            if escape:
                escape = False
            elif ch == "\\":
                escape = True
            elif ch == '"':
                in_string = False
        else:
            if ch == '"':
                in_string = True
            elif ch == "{":
                depth += 1
            elif ch == "}":
                depth -= 1
                if depth == 0:
                    return i
        i += 1
    return None


def extract_json(text: str) -> dict:
    """Extract the first JSON object embedded in ``text``.

    Strategy: try ``json.loads`` on the whole text first (the common case —
    a clean JSON response). If that fails or does not yield an object, fall
    back to a brace-matching scan that is aware of string literals and
    escapes, so braces inside string values and trailing/leading noise
    around the object do not break extraction. Returns ``{}`` if no valid
    JSON object can be found.
    """
    if not isinstance(text, str):
        return {}

    try:
        obj = json.loads(text)
        if isinstance(obj, dict):
            return obj
    except json.JSONDecodeError:
        pass

    n = len(text)
    i = 0
    while i < n:
        if text[i] == "{":
            end = _find_matching_brace(text, i)
            if end is not None:
                candidate = text[i : end + 1]
                try:
                    obj = json.loads(candidate)
                except json.JSONDecodeError:
                    obj = None
                if isinstance(obj, dict):
                    return obj
        i += 1
    return {}


def normalize_base_url(host: str) -> str:
    """Turn a user-supplied camera host into a usable HTTP base URL.

    Accepts what people actually type — ``192.168.1.50``, ``10.0.0.5:8080``,
    ``http://cam.local/`` — and always yields something with a scheme and no
    trailing slash. Returns ``""`` unchanged so "not configured yet" stays
    distinguishable from a real address.

    This exists because a bare host is silently useless: handing
    ``"192.168.1.50"`` to httpx as a base URL raises ``UnsupportedProtocol``
    ("Request URL is missing an 'http://' or 'https://' protocol") on the
    first request, and if that error is caught by a retry loop the symptom
    is a login that hangs for the full retry budget rather than an obvious
    configuration error.

    Note the deliberate avoidance of ``urlparse`` here: it parses
    ``"127.0.0.1:8080"`` as scheme ``"127.0.0.1"``, which is exactly the
    host:port form we need to support.
    """
    host = (host or "").strip()
    if not host:
        return ""
    if "://" not in host:
        host = "http://" + host
    return host.rstrip("/")


#: Key the camera uses to wrap FAILED responses, instead of the command name.
ERROR_KEY = "error"


def unwrap(raw: dict, cmd: str) -> dict:
    """Unwrap a Nexus CGI response envelope.

    Successful responses are keyed by the command name::

        {"PTLastNMEAGet": {"Return Code": 0, "Abs_Azimuth": -51.41, ...}}

    Failures are keyed by ``"error"`` instead — verified against a FLIR 364C::

        {"error": {"Return Code": 7,  "Return String": "Invalid function"}}
        {"error": {"Return Code": 21, "Return String": "Network client not authorized"}}

    Handling only the first form (``raw.get(cmd, raw)``, as the original client
    did) silently turns every failure into a success: the command-name lookup
    misses, the whole envelope is returned, and :func:`classify` then finds no
    top-level ``Return Code`` and reports ``None`` — which callers treat as
    "no error". Downstream, :func:`parse_nmea` defaults every missing field to
    ``0.0``, so an unauthorised session reads as a camera parked at AZ=0 EL=0
    rather than as an error. RC=21 re-login and RC=15 token-revive could never
    fire through this path.
    """
    if cmd in raw:
        return raw[cmd]
    err = raw.get(ERROR_KEY)
    if isinstance(err, dict):
        return err
    return raw


def classify(payload: dict) -> tuple[Optional[int], str]:
    """Extract ``(Return Code, Return String)`` from an unwrapped payload."""
    code = payload.get("Return Code")
    msg = str(payload.get("Return String", ""))
    if code is None:
        return None, msg
    return int(code), msg


def parse_nmea(payload: dict) -> PtSample:
    """Parse a PTLastNMEAGet-shaped payload; missing fields default to 0."""
    return PtSample(
        abs_az=float(payload.get("Abs_Azimuth", 0.0)),
        abs_el=float(payload.get("Abs_Elevation", 0.0)),
        geo_az=float(payload.get("Geo_Azimuth", 0.0)),
        geo_el=float(payload.get("Geo_Elevation", 0.0)),
        speed_x=float(payload.get("Speed_X", 0.0)),
        speed_y=float(payload.get("Speed_Y", 0.0)),
        mode=int(payload.get("Mode", 0)),
    )


@dataclass(frozen=True)
class DltvSample:
    """A single parsed DLTVLastNMEAGet sample -- EO (daylight) lens zoom/focus
    telemetry. Measured on a real FLIR 364C: despite its name suggesting a
    focal length in mm, ``Zoom`` is actually the lens's current HORIZONTAL
    FIELD OF VIEW IN DEGREES (e.g. 41.75 deg between the measured wide 63.70
    deg and tele 2.12 deg extremes -- see ``control/zoom_optics.py``),
    ``Zoom_pctg`` is percent of zoom travel (e.g. 18.59), ``Focus_pctg`` is
    percent of focus travel, and ``Autofocus`` is the camera's autofocus
    state flag."""

    zoom: float
    zoom_pctg: float
    focus_pctg: float
    autofocus: float


def parse_dltv(payload: dict) -> DltvSample:
    """Parse a DLTVLastNMEAGet-shaped payload; missing fields default to 0."""
    return DltvSample(
        zoom=float(payload.get("Zoom", 0.0)),
        zoom_pctg=float(payload.get("Zoom_pctg", 0.0)),
        focus_pctg=float(payload.get("Focus_pctg", 0.0)),
        autofocus=float(payload.get("Autofocus", 0.0)),
    )


@dataclass(frozen=True)
class IrSample:
    """A single parsed IRLastNMEAGet sample -- IR (thermal) lens zoom
    telemetry. Measured on a real FLIR 364C: ``FOV`` is the field of view in
    degrees (e.g. 11.67), ``Zoom_Pctg`` is percent of zoom travel -- unlike
    EO's continuous readout, IR is quantised to multiples of 6.25 (e.g.
    25.0 / 31.25 / 37.5) -- and ``FOV_Index`` is the discrete FOV step index.

    *** CAPITALISATION TRAP (measured on hardware) ***: DLTV (EO) reports
    ``Zoom_pctg`` (lower-case p); IR reports ``Zoom_Pctg`` (upper-case P).
    Reusing :func:`parse_dltv` verbatim for IR reads nothing and silently
    yields 0.0 -- that is exactly why IR gets its own dataclass and parse
    function instead of sharing :class:`DltvSample`/:func:`parse_dltv`.
    """

    fov: float
    zoom_pctg: float
    fov_index: int


def parse_ir(payload: dict) -> IrSample:
    """Parse an IRLastNMEAGet-shaped payload; missing fields default to 0.

    Deliberately reads ``Zoom_Pctg`` (capital P) -- see :class:`IrSample`'s
    docstring for the capitalisation trap this exists to avoid.
    """
    return IrSample(
        fov=float(payload.get("FOV", 0.0)),
        zoom_pctg=float(payload.get("Zoom_Pctg", 0.0)),
        fov_index=int(payload.get("FOV_Index", 0)),
    )
