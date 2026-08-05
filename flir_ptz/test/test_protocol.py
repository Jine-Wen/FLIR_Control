#!/usr/bin/env python3
"""Tests for flir_ptz.nexus.protocol.

Must run offline with nothing but the Python 3.12 standard library —
no ROS, no httpx. This test file itself must not import httpx either.
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import pytest

from flir_ptz.nexus.protocol import (
    LOGIN_AUTH,
    NEXUS_CGI,
    SESSION_SAVE,
    RC,
    NexusError,
    PtSample,
    build_query,
    classify,
    extract_json,
    parse_nmea,
    unwrap,
)


# ---------------------------------------------------------------------------
# constants
# ---------------------------------------------------------------------------

def test_endpoint_constants():
    assert NEXUS_CGI == "/Nexus.cgi"
    assert LOGIN_AUTH == "/api/login/auth"
    assert SESSION_SAVE == "/api/login/session/save"


def test_rc_values():
    assert RC.OK == 0
    assert RC.TOKEN_IDLE == 15
    assert RC.NOT_AUTH == 21


# ---------------------------------------------------------------------------
# NexusError
# ---------------------------------------------------------------------------

def test_nexus_error_attributes():
    err = NexusError("PTSpeedModeSet", 15, "token idle")
    assert err.cmd == "PTSpeedModeSet"
    assert err.code == 15
    assert err.msg == "token idle"
    assert "PTSpeedModeSet" in str(err)
    assert "15" in str(err)
    assert "token idle" in str(err)


# ---------------------------------------------------------------------------
# PtSample
# ---------------------------------------------------------------------------

def test_ptsample_speed_mag():
    s = PtSample(abs_az=1, abs_el=2, geo_az=3, geo_el=4, speed_x=-3.0, speed_y=2.0, mode=1)
    assert s.speed_mag == 3.0

    s2 = PtSample(abs_az=0, abs_el=0, geo_az=0, geo_el=0, speed_x=1.0, speed_y=-9.0, mode=0)
    assert s2.speed_mag == 9.0

    s3 = PtSample(abs_az=0, abs_el=0, geo_az=0, geo_el=0, speed_x=0.0, speed_y=0.0, mode=0)
    assert s3.speed_mag == 0.0


def test_ptsample_is_frozen():
    s = PtSample(abs_az=0, abs_el=0, geo_az=0, geo_el=0, speed_x=0, speed_y=0, mode=0)
    with pytest.raises(Exception):
        s.abs_az = 5.0  # type: ignore[misc]


# ---------------------------------------------------------------------------
# build_query
# ---------------------------------------------------------------------------

def test_build_query_basic():
    q = build_query("PTSpeedModeSet", {"Azimuth_Speed": 1.5, "Elevation_Speed": -2.0}, 42, 0, 123456)
    assert q["action"] == "PTSpeedModeSet"
    assert q["Azimuth_Speed"] == 1.5
    assert q["Elevation_Speed"] == -2.0
    assert q["session"] == 42
    assert q["DeviceID"] == 0
    assert q["_"] == 123456


def test_build_query_no_extra_params():
    q = build_query("SERVERWhoAmI", {}, None, 0, 1)
    assert q["action"] == "SERVERWhoAmI"
    assert q["DeviceID"] == 0
    assert q["_"] == 1


def test_build_query_omits_session_when_none():
    """Verified against real 364C hardware: `session=0` is rejected with
    RC=21, while omitting the parameter succeeds. Since SERVERWhoAmI is how
    the session id is obtained in the first place, substituting 0 here makes
    login impossible."""
    q = build_query("SERVERWhoAmI", {}, None, 0, 1)
    assert "session" not in q


def test_build_query_includes_session_when_known():
    q = build_query("PTLastNMEAGet", {}, 103, 0, 1)
    assert q["session"] == 103


def test_build_query_includes_session_zero_only_if_explicitly_given():
    """0 is a legitimate value to pass through if a caller really means it;
    the omission rule is driven by None, not by falsiness."""
    q = build_query("PTLastNMEAGet", {}, 0, 0, 1)
    assert q["session"] == 0


def test_build_query_does_not_leak_credentials():
    q = build_query("PTGeoAzimuthElevationSet", {"Geo_Azimuth": 1.0, "Geo_Elevation": 2.0}, 7, 0, 999)
    assert "username" not in q
    assert "password" not in q


# ---------------------------------------------------------------------------
# extract_json
# ---------------------------------------------------------------------------

def test_extract_json_pure():
    text = '{"a": 1, "b": {"c": 2}}'
    assert extract_json(text) == {"a": 1, "b": {"c": 2}}


def test_extract_json_leading_noise():
    text = 'garbage-before-json {"Return Code": 0, "Return String": "OK"}'
    assert extract_json(text) == {"Return Code": 0, "Return String": "OK"}


def test_extract_json_trailing_noise():
    # This is exactly the case the old greedy `re.search(r"\{.*\}", DOTALL)`
    # gets wrong when there's noise *after* a well-formed object followed by
    # something that looks like it could extend the match.
    text = '{"Return Code": 0, "Return String": "OK"} some trailing garbage'
    assert extract_json(text) == {"Return Code": 0, "Return String": "OK"}


def test_extract_json_leading_and_trailing_noise():
    text = 'HTTP/1.1 200 OK\r\n\r\n{"Return Code": 0} <html>ignored</html>'
    assert extract_json(text) == {"Return Code": 0}


def test_extract_json_nested_objects():
    text = '{"SERVERWhoAmI": {"Return Code": 0, "Id": 5, "nested": {"x": 1}}}'
    result = extract_json(text)
    assert result == {"SERVERWhoAmI": {"Return Code": 0, "Id": 5, "nested": {"x": 1}}}


def test_extract_json_braces_inside_string_values():
    text = '{"Return Code": 0, "Return String": "unexpected } brace { inside"}'
    result = extract_json(text)
    assert result == {"Return Code": 0, "Return String": "unexpected } brace { inside"}


def test_extract_json_escaped_quote_inside_string():
    text = r'{"msg": "she said \"hi\" with a } inside too"}'
    result = extract_json(text)
    assert result == {"msg": 'she said "hi" with a } inside too'}


def test_extract_json_malformed_returns_empty_dict():
    assert extract_json("not json at all") == {}
    assert extract_json("") == {}
    assert extract_json("{unterminated") == {}
    assert extract_json("{'single': 'quotes'}") == {}  # not valid JSON


def test_extract_json_top_level_non_object_falls_back_to_embedded_object():
    # A bare JSON array at top level is valid JSON but not a dict — the
    # nested object should still be extracted by the brace-matching fallback.
    text = '[1, 2, {"Return Code": 0}]'
    assert extract_json(text) == {"Return Code": 0}


def test_extract_json_two_top_level_objects_returns_first():
    text = 'noise {"first": 1} middle {"second": 2}'
    assert extract_json(text) == {"first": 1}


# ---------------------------------------------------------------------------
# unwrap
# ---------------------------------------------------------------------------

def test_unwrap_present():
    raw = {"PTSpeedModeSet": {"Return Code": 0}}
    assert unwrap(raw, "PTSpeedModeSet") == {"Return Code": 0}


def test_unwrap_absent_returns_raw():
    raw = {"Return Code": 0}
    assert unwrap(raw, "SomeOtherCmd") == raw


# ---------------------------------------------------------------------------
# classify
# ---------------------------------------------------------------------------

def test_classify_ok():
    assert classify({"Return Code": 0, "Return String": "OK"}) == (0, "OK")


def test_classify_error_code():
    assert classify({"Return Code": 21, "Return String": "not authorized"}) == (21, "not authorized")


def test_classify_missing_rc_returns_none():
    code, msg = classify({"Abs_Azimuth": 1.0})
    assert code is None
    assert msg == ""


def test_classify_missing_return_string_defaults_empty():
    code, msg = classify({"Return Code": 0})
    assert code == 0
    assert msg == ""


def test_classify_string_code_is_coerced_to_int():
    assert classify({"Return Code": "15", "Return String": "idle"}) == (15, "idle")


# ---------------------------------------------------------------------------
# parse_nmea
# ---------------------------------------------------------------------------

def test_parse_nmea_full_payload():
    payload = {
        "Abs_Azimuth": 12.5, "Abs_Elevation": -3.25,
        "Geo_Azimuth": 10.0, "Geo_Elevation": -1.0,
        "Speed_X": 2.0, "Speed_Y": -1.5,
        "Mode": 1,
    }
    sample = parse_nmea(payload)
    assert sample == PtSample(
        abs_az=12.5, abs_el=-3.25, geo_az=10.0, geo_el=-1.0,
        speed_x=2.0, speed_y=-1.5, mode=1,
    )


def test_parse_nmea_missing_fields_default_to_zero():
    sample = parse_nmea({})
    assert sample == PtSample(
        abs_az=0.0, abs_el=0.0, geo_az=0.0, geo_el=0.0,
        speed_x=0.0, speed_y=0.0, mode=0,
    )


def test_parse_nmea_partial_payload():
    sample = parse_nmea({"Abs_Azimuth": 99.0, "Mode": 2})
    assert sample.abs_az == 99.0
    assert sample.mode == 2
    assert sample.abs_el == 0.0
    assert sample.speed_x == 0.0
    assert sample.speed_y == 0.0


# ── Real FLIR 364C wire captures ─────────────────────────────────────────────
#
# Everything below was captured from actual hardware (FLIR 364C) rather than
# assumed. The error envelope in particular contradicted the original client's
# assumption and was silently turning every failure into a success.

REAL_SUCCESS_NMEA = """{ "PTLastNMEAGet": { "Return Code" : 0,
\t\t"Return String" : "No Error",
\t\t"Device Type" : "PT",
\t\t\t"Device Id" : 0,
\t\t\t"Mode" : 0,
\t\t\t"Abs_Azimuth" : -51.41,
\t\t\t"Abs_Elevation" : 1.80,
\t\t\t"Geo_Azimuth" : 308.59,
\t\t\t"Geo_Elevation" : 1.80,
\t\t\t"Speed_X" : 0.00,
\t\t\t"Speed_Y" : 0.00 }
}"""

REAL_ERROR_INVALID_FUNCTION = """{ "error": { "Return Code" : 7,
\t\t"Return String" : "Invalid function"
\t\t }
}"""

REAL_ERROR_NOT_AUTHORIZED = """{ "error": { "Return Code" : 21,
\t\t"Return String" : "Network client not authorized"
\t\t }
}"""

REAL_SERVER_STATUS = """{ "SERVERLastNMEAGet": { "Return Code" : 0,
\t\t"Return String" : "No Error",
\t\t\t"Token_ID" : 103,
\t\t\t"Owner" : "",
\t\t\t"Remote_Request" : 0 }
}"""


def test_real_success_envelope_round_trips():
    raw = extract_json(REAL_SUCCESS_NMEA)
    payload = unwrap(raw, "PTLastNMEAGet")
    code, msg = classify(payload)
    assert code == 0
    assert msg == "No Error"
    sample = parse_nmea(payload)
    assert sample.abs_az == -51.41
    assert sample.geo_az == 308.59
    assert sample.mode == 0


def test_real_error_envelope_is_keyed_by_error_not_command_name():
    """The bug this pins: failures arrive under "error", so a command-name
    lookup misses and the failure is reported as success."""
    raw = extract_json(REAL_ERROR_INVALID_FUNCTION)
    assert "PTSpeedModeSet" not in raw and "error" in raw
    payload = unwrap(raw, "PTSpeedModeSet")
    code, msg = classify(payload)
    assert code == 7
    assert msg == "Invalid function"


def test_real_rc21_is_detected_so_relogin_can_fire():
    raw = extract_json(REAL_ERROR_NOT_AUTHORIZED)
    code, _ = classify(unwrap(raw, "PTLastNMEAGet"))
    assert code == RC.NOT_AUTH == 21


def test_unauthorised_read_must_not_look_like_a_camera_parked_at_zero():
    """Regression: with the old unwrap, an RC=21 body fell through to
    parse_nmea, whose defaults report AZ=0 EL=0 -- an error indistinguishable
    from a camera genuinely pointing at the origin."""
    raw = extract_json(REAL_ERROR_NOT_AUTHORIZED)
    payload = unwrap(raw, "PTLastNMEAGet")
    code, _ = classify(payload)
    assert code is not None and code != 0, "error must be visible to the caller"


def test_real_server_status_exposes_token_id():
    payload = unwrap(extract_json(REAL_SERVER_STATUS), "SERVERLastNMEAGet")
    assert classify(payload)[0] == 0
    assert payload["Token_ID"] == 103


def test_abs_and_geo_azimuth_are_the_same_angle_in_different_ranges():
    """Real capture: abs is (-180,180], geo is [0,360). Not a heading offset --
    normalising the difference must give zero."""
    payload = unwrap(extract_json(REAL_SUCCESS_NMEA), "PTLastNMEAGet")
    sample = parse_nmea(payload)
    from flir_ptz.control.profiles import az_error
    assert abs(az_error(sample.geo_az, sample.abs_az)) < 1e-9
