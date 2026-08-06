#!/usr/bin/env python3
"""Tests for flir_ptz.nodes.web_node.

Must run fully offline with nothing but the Python 3.12 standard library:
no ROS, no httpx (ARCHITECTURE.md sec 0). ``rclpy`` is not importable in
this environment, so ``flir_ptz.nodes.web_node._ROS_AVAILABLE`` is False
and ``FlirPtzWebNode`` (the actual rclpy.Node subclass) is never defined
or exercised here -- exactly like ``test_joy_bridge.py`` for the sibling
joy-bridge node.

Two layers are covered instead (per the worker brief):

  1. The pure module-level conversion/decision functions directly (no
     ROS message types needed -- ``SimpleNamespace`` stands in for
     ``PtzState``/``ControlSource``/``builtin_interfaces/Time`` wherever a
     duck-typed object is enough).
  2. An end-to-end pass through the REAL ``flir_ptz.webui.server`` HTTP
     route table (API.md sec 1-6), using a fake ``WebAdapter`` built out
     of THIS module's pure functions the same way ``FlirPtzWebNode``
     wires them -- proving the conversion functions produce API.md-shaped
     JSON when actually served over a socket, not just in isolation.

No real IP address or password appears anywhere in this file (spec sec 8
rule 1): all hosts below are RFC 5737/2606 documentation placeholders
(``192.0.2.0/24``, ``cam.invalid``) and credentials are obvious fake
placeholders.
"""

from __future__ import annotations

import http.client
import json
import os
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import pytest  # noqa: E402

import flir_ptz.nodes.web_node as web_node  # noqa: E402
from flir_ptz.control.config import ControlConfig  # noqa: E402
from flir_ptz.webui.server import ServerConfig, make_server  # noqa: E402
from flir_ptz.webui.sse import SSEHub  # noqa: E402

DEFAULT_TIMEOUT = 2.0
FAKE_HOST = "192.0.2.10"  # RFC 5737 TEST-NET-1 -- never a real camera IP


# ═══════════════════════════════════════════════════════════════════════════
# Module importability without ROS (spec sec 6 testability requirement)
# ═══════════════════════════════════════════════════════════════════════════


def test_module_imports_without_rclpy():
    """web_node must import cleanly where rclpy is unavailable.

    Masks rclpy in a subprocess rather than asserting the machine lacks it.
    The earlier version asserted `_ROS_AVAILABLE is False`, which passed only
    in a shell with no ROS sourced and failed in the environment the package
    actually runs in -- the opposite of useful.
    """
    import subprocess
    import textwrap

    code = textwrap.dedent(
        """
        import sys
        sys.modules["rclpy"] = None   # makes `import rclpy` raise ImportError
        import flir_ptz.nodes.web_node as w
        assert w._ROS_AVAILABLE is False
        assert not hasattr(w, "FlirPtzWebNode")
        # The pure helpers must remain available with no ROS at all.
        assert w.streams_for_model("364c", "10.0.0.1")["ir"].startswith("rtsp://")
        print("OK")
        """
    )
    proc = subprocess.run(
        [sys.executable, "-c", code],
        cwd=str(_PKG_ROOT), capture_output=True, text=True, timeout=60,
    )
    assert proc.returncode == 0, proc.stderr
    assert "OK" in proc.stdout


def test_ros_node_class_is_defined_when_rclpy_is_available():
    """The other half of the contract: where rclpy IS importable, the node
    class must actually exist. Without this, a module that silently failed to
    define FlirPtzWebNode would look 'fine' to the suite above."""
    if web_node._ROS_AVAILABLE:
        assert hasattr(web_node, "FlirPtzWebNode")
    else:
        pytest.skip("rclpy not importable here; covered by the masked-import test")


# ═══════════════════════════════════════════════════════════════════════════
# Pure functions: model / stream config (PARITY C14/C15)
# ═══════════════════════════════════════════════════════════════════════════


def test_streams_for_model_364c_has_ir_and_eo():
    streams = web_node.streams_for_model("364c", FAKE_HOST)
    assert streams == {
        "ir": f"rtsp://{FAKE_HOST}:8554/ir.0",
        "eo": f"rtsp://{FAKE_HOST}:8554/vis.0",
    }


def test_streams_for_model_m232_has_no_eo_key_at_all():
    streams = web_node.streams_for_model("m232", FAKE_HOST)
    assert streams == {"ir": f"rtsp://{FAKE_HOST}:8554/ir"}
    assert "eo" not in streams


def test_streams_for_model_unknown_model_falls_back_to_364c():
    assert web_node.streams_for_model("bogus", FAKE_HOST) == web_node.streams_for_model("364c", FAKE_HOST)


def test_mediamtx_patch_url_and_body():
    assert web_node.mediamtx_patch_url(9997, "ir") == "http://localhost:9997/v3/config/paths/patch/ir"
    assert web_node.mediamtx_patch_body("rtsp://x:8554/ir") == {"source": "rtsp://x:8554/ir"}


def test_apply_model_to_mediamtx_patches_every_stream_for_the_model():
    calls: list[tuple[str, dict]] = []
    web_node.apply_model_to_mediamtx("364c", FAKE_HOST, 9997, patch_fn=lambda ep, body: calls.append((ep, body)))
    assert calls == [
        ("http://localhost:9997/v3/config/paths/patch/ir", {"source": f"rtsp://{FAKE_HOST}:8554/ir.0"}),
        ("http://localhost:9997/v3/config/paths/patch/eo", {"source": f"rtsp://{FAKE_HOST}:8554/vis.0"}),
    ]


def test_apply_model_to_mediamtx_m232_patches_only_ir():
    calls: list[tuple[str, dict]] = []
    web_node.apply_model_to_mediamtx("m232", FAKE_HOST, 9997, patch_fn=lambda ep, body: calls.append((ep, body)))
    assert len(calls) == 1
    assert calls[0][0] == "http://localhost:9997/v3/config/paths/patch/ir"


def test_apply_model_to_mediamtx_never_raises_when_mediamtx_is_down():
    """PARITY C15 resilience clause: mediamtx unreachable must log a
    warning, never crash the caller or fail the HTTP request."""

    def _boom(endpoint: str, body: dict) -> None:
        raise OSError("connection refused")

    errors: list[tuple[str, str]] = []
    # Must not raise.
    web_node.apply_model_to_mediamtx(
        "364c", FAKE_HOST, 9997, patch_fn=_boom, on_error=lambda name, exc: errors.append((name, str(exc)))
    )
    assert {name for name, _ in errors} == {"ir", "eo"}


def test_apply_model_to_mediamtx_reports_success_per_path():
    successes: list[tuple[str, str]] = []
    web_node.apply_model_to_mediamtx(
        "364c", FAKE_HOST, 9997, patch_fn=lambda ep, body: None, on_success=lambda name, url: successes.append((name, url))
    )
    assert dict(successes) == web_node.streams_for_model("364c", FAKE_HOST)


def test_apply_model_to_mediamtx_partial_failure_still_reports_the_other_success():
    """One path down (mediamtx has no 'eo' path configured, say) must not
    prevent the other path's patch from being attempted and reported."""

    def _patch(endpoint: str, body: dict) -> None:
        if endpoint.endswith("/eo"):
            raise OSError("no such path")

    successes: list[str] = []
    errors: list[str] = []
    web_node.apply_model_to_mediamtx(
        "364c",
        FAKE_HOST,
        9997,
        patch_fn=_patch,
        on_success=lambda name, url: successes.append(name),
        on_error=lambda name, exc: errors.append(name),
    )
    assert successes == ["ir"]
    assert errors == ["eo"]


# ═══════════════════════════════════════════════════════════════════════════
# Pure functions: PtzState / ControlSource / camera_status conversion
# ═══════════════════════════════════════════════════════════════════════════


def _fake_ptz_state(**overrides: Any) -> SimpleNamespace:
    defaults = dict(
        abs_azimuth=10.0,
        abs_elevation=-5.0,
        geo_azimuth=11.0,
        geo_elevation=-4.5,
        speed_x=1.5,
        speed_y=0.0,
        mode=0,
        is_moving=True,
        is_scanning=False,
        active_move_seq=7,
        active_scan_seq=0,
        zoom_pctg=18.59,
        zoom_mm=41.75,
        ir_zoom_pctg=31.25,
        ir_fov=11.67,
    )
    defaults.update(overrides)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=250_000_000)), **defaults)


def test_ptz_state_msg_to_state_dict_matches_api_md_shape():
    msg = _fake_ptz_state()
    state = web_node.ptz_state_msg_to_state_dict(msg)
    assert state == {
        "abs_azimuth": 10.0,
        "abs_elevation": -5.0,
        "geo_azimuth": 11.0,
        "geo_elevation": -4.5,
        "speed_x": 1.5,
        "speed_y": 0.0,
        "mode": 0,
        "is_moving": True,
        "is_scanning": False,
        "active_move_seq": 7,
        "active_scan_seq": 0,
        "zoom_pctg": 18.59,
        "zoom_mm": 41.75,
        "ir_zoom_pctg": 31.25,
        "ir_fov": 11.67,
        "stamp": 100.25,
    }


def test_build_state_payload_shape_matches_api_md_sec2_state_event():
    state = web_node.ptz_state_msg_to_state_dict(_fake_ptz_state())
    payload = web_node.build_state_payload(
        connected=True,
        state_age_sec=0.05,
        model="364c",
        streams={"ir": "rtsp://cam.invalid:8554/ir.0", "eo": "rtsp://cam.invalid:8554/vis.0"},
        control_source="web",
        state=state,
    )
    assert set(payload.keys()) == {"connected", "state_age_sec", "model", "streams", "control_source", "state"}
    assert payload["connected"] is True
    assert payload["control_source"] == "web"
    assert payload["state"] == state


def test_build_state_payload_state_may_be_none_before_first_ptzstate():
    payload = web_node.build_state_payload(False, None, "364c", {"ir": "x"}, "", None)
    assert payload["state"] is None
    assert payload["state_age_sec"] is None
    assert payload["connected"] is False


def test_ros_time_to_float():
    assert web_node.ros_time_to_float(SimpleNamespace(sec=5, nanosec=500_000_000)) == 5.5
    assert web_node.ros_time_to_float(SimpleNamespace(sec=0, nanosec=0)) == 0.0


@pytest.mark.parametrize(
    "raw,expected",
    [
        ('{"status":"connected","host":"cam.invalid","error":""}', {"status": "connected", "host": "cam.invalid", "error": ""}),
        ('{"status":"error","host":"","error":"timeout"}', {"status": "error", "host": "", "error": "timeout"}),
        ("not json at all", {"status": "error", "host": "", "error": ""}),
        ("[]", {"status": "error", "host": "", "error": ""}),  # valid JSON, not a dict
        ("{}", {"status": "error", "host": "", "error": ""}),  # dict, but missing fields
        (None, {"status": "error", "host": "", "error": ""}),  # never raises even on wrong type
    ],
)
def test_camera_status_json_to_dict(raw, expected):
    assert web_node.camera_status_json_to_dict(raw) == expected


# ═══════════════════════════════════════════════════════════════════════════
# Pure functions: mirrored (non-authoritative) arbitration check
# ═══════════════════════════════════════════════════════════════════════════


def test_mirrored_allows_stop_command_always_allowed_no_owner():
    assert web_node.mirrored_allows("", 0.0, "web", is_stop=True, now=1000.0) is True


def test_mirrored_allows_stop_command_always_allowed_even_when_locked_by_other():
    assert web_node.mirrored_allows("joy", 2000.0, "web", is_stop=True, now=1000.0) is True


def test_mirrored_allows_non_stop_accepted_when_unclaimed():
    """An unowned lease falls open. See control.arbitration.allows_command --
    this mirror delegates to that single shared rule precisely so it cannot
    drift from the authority again."""
    assert web_node.mirrored_allows("", 0.0, "web", is_stop=False, now=1000.0) is True


def test_mirrored_allows_non_stop_rejected_when_owned_by_someone_else():
    assert web_node.mirrored_allows("joy", 2000.0, "web", is_stop=False, now=1000.0) is False


def test_mirrored_allows_non_stop_allowed_for_current_owner_before_expiry():
    assert web_node.mirrored_allows("web", 2000.0, "web", is_stop=False, now=1000.0) is True


def test_mirrored_allows_non_stop_accepted_once_lease_expired():
    """The deadlock this whole change exists to remove: an expired lease used
    to reject every command here, before it could even reach the authority, so
    the joystick silently stopped working until the operator repeated the
    unlock gesture."""
    assert web_node.mirrored_allows("web", 500.0, "web", is_stop=False, now=1000.0) is True
    # ...and a different source may take over an expired lease too.
    assert web_node.mirrored_allows("web", 500.0, "joy", is_stop=False, now=1000.0) is True


def test_mirrored_allows_still_rejects_against_a_live_foreign_lease():
    """Contention is unchanged: a live owner still excludes everyone else."""
    assert web_node.mirrored_allows("web", 1000.0, "joy", is_stop=False, now=500.0) is False
    assert web_node.mirrored_allows("web", 1000.0, "web", is_stop=False, now=500.0) is True
    # A stop is never blocked, even against a live foreign lease.
    assert web_node.mirrored_allows("web", 1000.0, "joy", is_stop=True, now=500.0) is True


def test_is_zero_speed():
    assert web_node.is_zero_speed(0.0, 0.0) is True
    assert web_node.is_zero_speed(0.0, 0.1) is False
    assert web_node.is_zero_speed(-0.0, 0.0) is True


def test_to_float_never_raises():
    assert web_node.to_float("3.5", 0.0) == 3.5
    assert web_node.to_float(None, 9.0) == 9.0
    assert web_node.to_float("not a number", 9.0) == 9.0
    assert web_node.to_float({"nope": True}, 9.0) == 9.0
    assert web_node.to_float(4, 0.0) == 4.0


# ═══════════════════════════════════════════════════════════════════════════
# Pure-ish: web_root resolution (asset root, PARITY note)
# ═══════════════════════════════════════════════════════════════════════════


def test_resolve_web_root_explicit_param_wins(tmp_path: Path):
    custom = tmp_path / "custom_web"
    custom.mkdir()
    resolved = web_node.resolve_web_root(str(custom))
    assert resolved == custom.resolve()


def test_resolve_web_root_explicit_param_expands_user_and_resolves(tmp_path: Path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    (tmp_path / "web_home").mkdir()
    resolved = web_node.resolve_web_root("~/web_home")
    assert resolved == (tmp_path / "web_home").resolve()


def test_resolve_web_root_falls_back_to_source_tree_when_no_param_and_no_ament(monkeypatch):
    """Graceful degradation when ament_index_python cannot be imported.

    This must test the CODE, not the machine. An earlier version asserted that
    the module was genuinely absent, which made the test pass only in a shell
    with no ROS sourced -- and fail in exactly the environment the package
    actually runs in, since sourcing setup.bash puts ament_index_python on the
    path. Mask the import instead so the degradation path is exercised
    regardless of what happens to be installed.
    """
    monkeypatch.setitem(sys.modules, "ament_index_python", None)
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", None)

    resolved = web_node.resolve_web_root("")
    assert resolved == Path(web_node.__file__).resolve().parent.parent / "web"
    assert (resolved / "index.html").exists()


def test_resolve_web_root_swallows_ament_index_failures(monkeypatch):
    """Even if ament_index_python WERE importable but the package can't be
    found (not installed / AMENT_PREFIX_PATH wrong), resolution must still
    fall through to the dev-tree fallback rather than raising."""
    import types

    fake_pkg_mod = types.ModuleType("ament_index_python.packages")

    def _boom(_name: str) -> str:
        raise LookupError("package 'flir_ptz' not found")

    fake_pkg_mod.get_package_share_directory = _boom  # type: ignore[attr-defined]
    fake_ament_mod = types.ModuleType("ament_index_python")
    monkeypatch.setitem(sys.modules, "ament_index_python", fake_ament_mod)
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", fake_pkg_mod)

    resolved = web_node.resolve_web_root("")
    assert resolved == Path(web_node.__file__).resolve().parent.parent / "web"


def test_resolve_web_root_uses_ament_share_dir_when_available(tmp_path: Path, monkeypatch):
    import types

    share_web = tmp_path / "share" / "flir_ptz" / "web"
    share_web.mkdir(parents=True)

    fake_pkg_mod = types.ModuleType("ament_index_python.packages")
    fake_pkg_mod.get_package_share_directory = lambda name: str(tmp_path / "share" / name)  # type: ignore[attr-defined]
    fake_ament_mod = types.ModuleType("ament_index_python")
    monkeypatch.setitem(sys.modules, "ament_index_python", fake_ament_mod)
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", fake_pkg_mod)

    resolved = web_node.resolve_web_root("")
    assert resolved == share_web


# ═══════════════════════════════════════════════════════════════════════════
# End-to-end through the REAL webui.server, with a fake WebAdapter built
# from THIS module's pure functions (same wiring FlirPtzWebNode uses).
# ═══════════════════════════════════════════════════════════════════════════


class _NodeLikeAdapter:
    """Implements ``webui.server.WebAdapter`` using web_node.py's pure
    functions exactly the way ``FlirPtzWebNode`` wires them, but with
    plain Python calls standing in for ROS pub/sub/service so this can run
    with no rclpy at all. Proves the pure layer produces API.md-shaped
    output when actually served by the real HTTP/SSE stack.
    """

    def __init__(self, camera_host: str = FAKE_HOST, model: str = "364c") -> None:
        self.camera_host = camera_host
        self.model = model
        self.latest_ptz_state: Optional[dict] = None
        self.state_received_at: Optional[float] = None
        self.control_source = {"owner": "", "expires": 0.0}
        self.camera_status = {"status": "connecting", "host": "", "error": ""}
        self.connect_calls: list[tuple[str, str, str, str]] = []
        self.published: list[tuple[str, Any]] = []
        # stand-in for the flir/claim_control service
        self.claim_responses: list[dict] = []

    # -- simulate a ROS callback arriving --
    def push_state(self, **overrides: Any) -> dict:
        msg = _fake_ptz_state(**overrides)
        self.latest_ptz_state = web_node.ptz_state_msg_to_state_dict(msg)
        self.state_received_at = time.monotonic()
        return self.state_snapshot()

    def push_camera_status(self, raw_json: str) -> dict:
        self.camera_status = web_node.camera_status_json_to_dict(raw_json)
        return dict(self.camera_status)

    def push_control_source(self, owner: str, expires: float) -> dict:
        self.control_source = {"owner": owner, "expires": expires}
        return dict(self.control_source)

    # -- WebAdapter: snapshots --
    def state_snapshot(self) -> dict:
        age = None if self.state_received_at is None else max(0.0, time.monotonic() - self.state_received_at)
        connected = self.camera_status.get("status") == "connected"
        streams = web_node.streams_for_model(self.model, self.camera_host)
        return web_node.build_state_payload(
            connected, age, self.model, streams, self.control_source.get("owner", ""), self.latest_ptz_state
        )

    def control_source_snapshot(self) -> dict:
        return dict(self.control_source)

    def camera_status_snapshot(self) -> dict:
        return dict(self.camera_status)

    def model_snapshot(self) -> dict:
        return {
            "model": self.model,
            "streams": web_node.streams_for_model(self.model, self.camera_host),
            "models": list(web_node.MODEL_CONFIGS.keys()),
        }

    def stream_url(self, key: str) -> Optional[str]:
        return web_node.streams_for_model(self.model, self.camera_host).get(key)

    # -- WebAdapter: commands (mirrors FlirPtzWebNode._allowed / cmd_*) --
    def _allowed(self, source: str, is_stop: bool) -> bool:
        return web_node.mirrored_allows(
            self.control_source.get("owner", ""), self.control_source.get("expires", 0.0), source, is_stop, time.time()
        )

    def cmd_move_to(self, body: dict) -> dict:
        az = web_node.to_float(body.get("target_azimuth"), 0.0)
        el = web_node.to_float(body.get("target_elevation"), 0.0)
        source = str(body.get("source") or web_node.DEFAULT_SOURCE)
        if not self._allowed(source, is_stop=False):
            return {"ok": False, "locked": self.control_source.get("owner", "")}
        self.published.append(("move_to", {"target_azimuth": az, "target_elevation": el, "source": source}))
        return {"ok": True, "target_azimuth": az, "target_elevation": el}

    def cmd_track(self, body: dict) -> dict:
        return self.cmd_move_to(body)  # same shape/rules for this test double

    def cmd_joy_stick_control(self, body: dict) -> dict:
        az = web_node.to_float(body.get("azimuth_speed"), 0.0)
        el = web_node.to_float(body.get("elevation_speed"), 0.0)
        source = str(body.get("source") or web_node.DEFAULT_SOURCE)
        if not self._allowed(source, is_stop=web_node.is_zero_speed(az, el)):
            return {"ok": False, "locked": self.control_source.get("owner", "")}
        self.published.append(("joy_stick_control", {"azimuth_speed": az, "elevation_speed": el, "source": source}))
        return {"ok": True, "azimuth_speed": az, "elevation_speed": el}

    def cmd_scan(self, body: dict) -> dict:
        stop = bool(body.get("stop", False))
        source = str(body.get("source") or web_node.DEFAULT_SOURCE)
        if not self._allowed(source, is_stop=stop):
            return {"ok": False, "locked": self.control_source.get("owner", ""), "stop": stop}
        self.published.append(("scan", {"stop": stop, "source": source}))
        return {"ok": True, "stop": stop}

    def cmd_zoom(self, body: dict) -> dict:
        direction = web_node.normalize_zoom_direction(str(body.get("direction", "stop")))
        device = web_node.normalize_zoom_device(str(body.get("device", "eo")))
        source = str(body.get("source") or web_node.DEFAULT_SOURCE)
        if not self._allowed(source, is_stop=direction == "stop"):
            return {"ok": False, "locked": self.control_source.get("owner", ""), "direction": direction, "device": device}
        self.published.append(("zoom", {"direction": direction, "device": device, "source": source}))
        return {"ok": True, "direction": direction, "device": device}

    def cmd_home(self, body: dict) -> dict:
        source = str(body.get("source") or web_node.DEFAULT_SOURCE)
        if not self._allowed(source, is_stop=False):
            return {"ok": False, "locked": self.control_source.get("owner", "")}
        cfg = ControlConfig()
        self.published.append(("home", {"target_azimuth": cfg.home_az, "target_elevation": cfg.home_el, "source": source}))
        return {"ok": True, "target_azimuth": cfg.home_az, "target_elevation": cfg.home_el}

    # -- WebAdapter: arbitration client (stand-in for ClaimControl.srv) --
    def claim(self, source: str) -> dict:
        response = self.claim_responses.pop(0) if self.claim_responses else {"granted": True, "owner": source}
        if response.get("granted"):
            self.control_source = {"owner": source, "expires": time.time() + 60.0}
        return {"ok": True, "granted": bool(response.get("granted")), "owner": response.get("owner", "")}

    def unlock(self) -> dict:
        self.control_source = {"owner": "", "expires": 0.0}
        return {"ok": True}

    # -- WebAdapter: setup --
    def connect(self, host: str, model: str, username: str, password: str) -> None:
        # Must return immediately -- no sleep, no waiting on a response.
        self.connect_calls.append((host, model, username, password))

    def set_model(self, model: str) -> dict:
        if model not in web_node.MODEL_CONFIGS:
            return {"ok": False, "model": self.model, "streams": web_node.streams_for_model(self.model, self.camera_host)}
        self.model = model
        return {"ok": True, "model": model, "streams": web_node.streams_for_model(model, self.camera_host)}


def _make_web_root(tmp_path: Path) -> Path:
    root = tmp_path / "web"
    root.mkdir()
    for name in ("index.html", "setup.html", "styles.css", "app.js", "setup.js"):
        (root / name).write_text(f"<!-- {name} -->")
    return root


class _RunningServer:
    def __init__(self, tmp_path: Path, adapter: _NodeLikeAdapter, **config_kwargs: Any) -> None:
        self.web_root = _make_web_root(tmp_path)
        self.adapter = adapter
        self.hub = SSEHub()
        cfg_kwargs: dict[str, Any] = dict(bind_host="127.0.0.1", port=0)
        cfg_kwargs.update(config_kwargs)
        self.config = ServerConfig(web_root=self.web_root, **cfg_kwargs)
        self.server = make_server(self.config, self.adapter, self.hub)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    def __enter__(self) -> "_RunningServer":
        self.thread.start()
        self.host, self.port = self.server.server_address[0], self.server.server_address[1]
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=DEFAULT_TIMEOUT)
        return False

    def connection(self) -> http.client.HTTPConnection:
        return http.client.HTTPConnection(self.host, self.port, timeout=DEFAULT_TIMEOUT)


def _get(conn: http.client.HTTPConnection, path: str):
    conn.request("GET", path)
    resp = conn.getresponse()
    body = resp.read()
    return resp.status, json.loads(body) if body else None


def _post(conn: http.client.HTTPConnection, path: str, body: Optional[dict] = None):
    data = json.dumps(body or {}).encode("utf-8")
    conn.request("POST", path, body=data, headers={"Content-Type": "application/json", "Content-Length": str(len(data))})
    resp = conn.getresponse()
    raw = resp.read()
    return resp.status, json.loads(raw) if raw else None


# -- C1: setup page connect flow (host/model/username/password) --------------


def test_c1_connect_is_accepted_and_forwarded_to_adapter(tmp_path: Path):
    adapter = _NodeLikeAdapter()
    with _RunningServer(tmp_path, adapter) as srv:
        status, resp = _post(
            srv.connection(),
            "/api/connect",
            {"host": FAKE_HOST, "model": "m232", "username": "op", "password": "s3cret-placeholder"},
        )
    assert status == 200
    assert resp == {"ok": True, "pending": True}
    assert adapter.connect_calls == [(FAKE_HOST, "m232", "op", "s3cret-placeholder")]


def test_connect_returns_immediately_even_though_outcome_is_async(tmp_path: Path):
    """API.md sec 5 / PARITY: /api/connect MUST NOT block on the camera.
    The fake adapter's connect() never sleeps, so a slow response here
    would indicate the server itself is blocking somewhere it shouldn't."""
    adapter = _NodeLikeAdapter()
    with _RunningServer(tmp_path, adapter) as srv:
        started = time.monotonic()
        status, resp = _post(srv.connection(), "/api/connect", {"host": FAKE_HOST, "password": "x"})
        elapsed = time.monotonic() - started
    assert status == 200
    assert resp["pending"] is True
    assert elapsed < 1.0  # generous bound; old code could take up to 15s


# -- C2: connection_status for the setup page's "already connected?" check --


def test_c2_connection_status_reflects_camera_status_snapshot(tmp_path: Path):
    adapter = _NodeLikeAdapter()
    adapter.push_camera_status(json.dumps({"status": "connected", "host": FAKE_HOST, "error": ""}))
    with _RunningServer(tmp_path, adapter) as srv:
        status, resp = _get(srv.connection(), "/api/connection_status")
    assert status == 200
    assert resp["ok"] is True
    assert resp["connected"] is True
    assert resp["status"]["host"] == FAKE_HOST


def test_c2_connection_status_false_before_any_camera_status_arrives(tmp_path: Path):
    adapter = _NodeLikeAdapter()
    with _RunningServer(tmp_path, adapter) as srv:
        status, resp = _get(srv.connection(), "/api/connection_status")
    assert resp["connected"] is False
    assert resp["status"]["status"] == "connecting"


# -- C13: camera disconnect banner --------------------------------------------


def test_c13_camera_status_error_is_visible_via_snapshot_and_sse(tmp_path: Path):
    adapter = _NodeLikeAdapter()
    with _RunningServer(tmp_path, adapter) as srv:
        adapter.push_camera_status(json.dumps({"status": "error", "host": FAKE_HOST, "error": "auth failed"}))
        srv.hub.broadcast("camera_status", adapter.camera_status_snapshot())
        status, resp = _get(srv.connection(), "/api/connection_status")
        assert resp["status"]["status"] == "error"
        assert resp["status"]["error"] == "auth failed"
        assert srv.hub.last_value("camera_status") == {"status": "error", "host": FAKE_HOST, "error": "auth failed"}


# -- C14/C15: model switch (EO visibility + mediamtx patch) -------------------


def test_c14_model_endpoint_reports_eo_stream_for_364c(tmp_path: Path):
    adapter = _NodeLikeAdapter(model="364c")
    with _RunningServer(tmp_path, adapter) as srv:
        status, resp = _get(srv.connection(), "/api/model")
    assert resp["model"] == "364c"
    assert "eo" in resp["streams"]
    assert set(resp["models"]) == {"364c", "m232"}


def test_c14_model_endpoint_omits_eo_stream_for_m232(tmp_path: Path):
    adapter = _NodeLikeAdapter(model="m232")
    with _RunningServer(tmp_path, adapter) as srv:
        status, resp = _get(srv.connection(), "/api/model")
    assert resp["model"] == "m232"
    assert "eo" not in resp["streams"]


def test_c14_post_model_switches_and_updates_streams(tmp_path: Path):
    adapter = _NodeLikeAdapter(model="364c")
    with _RunningServer(tmp_path, adapter) as srv:
        status, resp = _post(srv.connection(), "/api/model", {"model": "m232"})
    assert resp["ok"] is True
    assert resp["model"] == "m232"
    assert "eo" not in resp["streams"]
    assert adapter.model == "m232"


def test_c15_mediamtx_patch_targets_match_post_model_streams():
    """The URLs apply_model_to_mediamtx() PATCHes must be exactly the
    stream URLs /api/model just switched to (PARITY C15)."""
    calls: list[tuple[str, dict]] = []
    web_node.apply_model_to_mediamtx("m232", FAKE_HOST, 9997, patch_fn=lambda ep, body: calls.append((ep, body)))
    expected_url = web_node.streams_for_model("m232", FAKE_HOST)["ir"]
    assert calls == [("http://localhost:9997/v3/config/paths/patch/ir", {"source": expected_url})]


# -- C17: claim / unlock -------------------------------------------------------


def test_c17_claim_grants_and_is_visible_in_control_source_snapshot(tmp_path: Path):
    adapter = _NodeLikeAdapter()
    with _RunningServer(tmp_path, adapter) as srv:
        status, resp = _post(srv.connection(), "/api/claim", {"source": "web"})
        assert status == 200
        assert resp == {"ok": True, "granted": True, "owner": "web"}
        _, cs = _get(srv.connection(), "/api/control_source")
        assert cs == {"ok": True, "source": "web"}


def test_c17_unlock_clears_control_source(tmp_path: Path):
    adapter = _NodeLikeAdapter()
    adapter.push_control_source("web", time.time() + 60.0)
    with _RunningServer(tmp_path, adapter) as srv:
        status, resp = _post(srv.connection(), "/api/unlock", {})
        assert status == 200
        assert resp == {"ok": True}
        _, cs = _get(srv.connection(), "/api/control_source")
        assert cs == {"ok": True, "source": ""}


def test_c17_claim_denied_reflects_service_response(tmp_path: Path):
    adapter = _NodeLikeAdapter()
    adapter.push_control_source("joy", time.time() + 60.0)
    adapter.claim_responses.append({"granted": False, "owner": "joy"})
    with _RunningServer(tmp_path, adapter) as srv:
        status, resp = _post(srv.connection(), "/api/claim", {"source": "web"})
    assert resp == {"ok": True, "granted": False, "owner": "joy"}


# -- arbitration mirroring: cmd_* endpoints honour the mirrored latch --------


def test_cmd_move_to_accepted_when_lease_is_unclaimed():
    """A command against an unowned lease must be forwarded, not rejected.

    This asserted the opposite before. Rejecting here was the second half of
    the deadlock: even after the authority was fixed to let an idle lease fall
    open, this mirror kept answering {"ok": false, "locked": ""} and the
    command never reached the PTZ node at all, so fixing the real rule looked
    like it had done nothing.
    """
    adapter = _NodeLikeAdapter()
    resp = adapter.cmd_move_to({"target_azimuth": 1.0, "target_elevation": 2.0, "source": "web"})
    assert resp.get("ok") is True
    assert "locked" not in resp


def test_cmd_move_to_allowed_once_web_holds_the_mirrored_lease():
    adapter = _NodeLikeAdapter()
    adapter.push_control_source("web", time.time() + 60.0)
    resp = adapter.cmd_move_to({"target_azimuth": 1.0, "target_elevation": 2.0, "source": "web"})
    assert resp == {"ok": True, "target_azimuth": 1.0, "target_elevation": 2.0}
    assert adapter.published[-1][0] == "move_to"
    assert adapter.published[-1][1]["source"] == "web"


def test_cmd_move_to_locked_when_joy_holds_lease():
    adapter = _NodeLikeAdapter()
    adapter.push_control_source("joy", time.time() + 60.0)
    resp = adapter.cmd_move_to({"target_azimuth": 1.0, "target_elevation": 2.0, "source": "web"})
    assert resp == {"ok": False, "locked": "joy"}
    assert adapter.published == []


def test_cmd_joy_stick_control_zero_speed_always_allowed_even_when_locked_by_other():
    adapter = _NodeLikeAdapter()
    adapter.push_control_source("joy", time.time() + 60.0)
    resp = adapter.cmd_joy_stick_control({"azimuth_speed": 0.0, "elevation_speed": 0.0, "source": "web"})
    assert resp == {"ok": True, "azimuth_speed": 0.0, "elevation_speed": 0.0}


def test_cmd_scan_stop_always_allowed_even_when_locked_by_other():
    adapter = _NodeLikeAdapter()
    adapter.push_control_source("joy", time.time() + 60.0)
    resp = adapter.cmd_scan({"stop": True, "source": "web"})
    assert resp == {"ok": True, "stop": True}


def test_cmd_scan_start_locked_when_joy_holds_lease():
    adapter = _NodeLikeAdapter()
    adapter.push_control_source("joy", time.time() + 60.0)
    resp = adapter.cmd_scan({"stop": False, "center_azimuth": 0.0, "source": "web"})
    assert resp == {"ok": False, "locked": "joy", "stop": False}


def test_normalize_zoom_direction_passes_through_in_and_out():
    assert web_node.normalize_zoom_direction("in") == "in"
    assert web_node.normalize_zoom_direction("out") == "out"


def test_normalize_zoom_direction_fails_safe_to_stop():
    """The lens is CONTINUOUS -- an unrecognised value must never be
    silently dropped (which for zoom would mean "leave the lens running"),
    so it degrades to the one direction that is always safe: stop."""
    assert web_node.normalize_zoom_direction("stop") == "stop"
    assert web_node.normalize_zoom_direction("") == "stop"
    assert web_node.normalize_zoom_direction("sideways") == "stop"
    assert web_node.normalize_zoom_direction(None) == "stop"


def test_cmd_zoom_accepted_when_lease_is_unclaimed():
    adapter = _NodeLikeAdapter()
    resp = adapter.cmd_zoom({"direction": "in", "source": "web"})
    assert resp == {"ok": True, "direction": "in", "device": "eo"}
    assert adapter.published[-1] == ("zoom", {"direction": "in", "device": "eo", "source": "web"})


def test_cmd_zoom_in_locked_when_joy_holds_lease():
    adapter = _NodeLikeAdapter()
    adapter.push_control_source("joy", time.time() + 60.0)
    resp = adapter.cmd_zoom({"direction": "in", "source": "web"})
    assert resp == {"ok": False, "locked": "joy", "direction": "in", "device": "eo"}
    assert adapter.published == []


def test_cmd_zoom_stop_always_allowed_even_when_locked_by_other():
    adapter = _NodeLikeAdapter()
    adapter.push_control_source("joy", time.time() + 60.0)
    resp = adapter.cmd_zoom({"direction": "stop", "source": "web"})
    assert resp == {"ok": True, "direction": "stop", "device": "eo"}


def test_cmd_zoom_unrecognised_direction_treated_as_stop_and_always_allowed():
    adapter = _NodeLikeAdapter()
    adapter.push_control_source("joy", time.time() + 60.0)
    resp = adapter.cmd_zoom({"direction": "sideways", "source": "web"})
    assert resp == {"ok": True, "direction": "stop", "device": "eo"}


# -- device: carried through /api/cmd/zoom, defaulting to "eo" ---------------


def test_normalize_zoom_device_passes_through_eo_and_ir():
    assert web_node.normalize_zoom_device("eo") == "eo"
    assert web_node.normalize_zoom_device("ir") == "ir"


def test_normalize_zoom_device_defaults_to_eo_when_absent_or_unrecognised():
    assert web_node.normalize_zoom_device("") == "eo"
    assert web_node.normalize_zoom_device("thermal") == "eo"
    assert web_node.normalize_zoom_device(None) == "eo"


def test_cmd_zoom_ir_device_is_carried_through():
    adapter = _NodeLikeAdapter()
    resp = adapter.cmd_zoom({"direction": "in", "device": "ir", "source": "web"})
    assert resp == {"ok": True, "direction": "in", "device": "ir"}
    assert adapter.published[-1] == ("zoom", {"direction": "in", "device": "ir", "source": "web"})


def test_cmd_zoom_omitted_device_defaults_to_eo():
    adapter = _NodeLikeAdapter()
    resp = adapter.cmd_zoom({"direction": "out", "source": "web"})
    assert resp["device"] == "eo"
    assert adapter.published[-1][1]["device"] == "eo"


def test_cmd_home_uses_control_config_home_position():
    adapter = _NodeLikeAdapter()
    adapter.push_control_source("web", time.time() + 60.0)
    resp = adapter.cmd_home({"source": "web"})
    cfg = ControlConfig()
    assert resp == {"ok": True, "target_azimuth": cfg.home_az, "target_elevation": cfg.home_el}


# -- state event: full round trip via SSE + GET /api/state -------------------


def test_state_snapshot_matches_get_api_state_and_sse_broadcast(tmp_path: Path):
    adapter = _NodeLikeAdapter()
    with _RunningServer(tmp_path, adapter) as srv:
        payload = adapter.push_state()
        srv.hub.broadcast("state", payload)
        status, resp = _get(srv.connection(), "/api/state")
    assert resp["state_age_sec"] == pytest.approx(payload["state_age_sec"], abs=0.5)
    assert {k: v for k, v in resp.items() if k != "state_age_sec"} == {
        k: v for k, v in payload.items() if k != "state_age_sec"
    }
    assert srv.hub.last_value("state") == payload
    assert resp["state"]["abs_azimuth"] == 10.0
    assert resp["model"] == "364c"
    assert set(resp["streams"].keys()) == {"ir", "eo"}


def test_state_snapshot_state_age_grows_and_connected_tracks_camera_status(tmp_path: Path):
    adapter = _NodeLikeAdapter()
    adapter.push_state()
    snap_before = adapter.state_snapshot()
    assert snap_before["connected"] is False  # camera_status still "connecting"
    adapter.push_camera_status(json.dumps({"status": "connected", "host": FAKE_HOST, "error": ""}))
    snap_after = adapter.state_snapshot()
    assert snap_after["connected"] is True
    assert snap_after["state_age_sec"] is not None and snap_after["state_age_sec"] >= 0.0


# -- new SSE client gets an immediate replay of the latest values ------------


def test_new_sse_client_gets_current_values_immediately(tmp_path: Path):
    adapter = _NodeLikeAdapter()
    with _RunningServer(tmp_path, adapter) as srv:
        srv.hub.broadcast("state", adapter.push_state())
        srv.hub.broadcast("control_source", adapter.push_control_source("web", time.time() + 60.0))
        srv.hub.broadcast("camera_status", adapter.push_camera_status(json.dumps({"status": "connected", "host": FAKE_HOST, "error": ""})))

        client_id, q = srv.hub.register()
        try:
            seen_events = set()
            for _ in range(3):
                frame = q.get(timeout=DEFAULT_TIMEOUT)
                assert frame is not None
                text = frame.decode("utf-8")
                event_line = text.splitlines()[0]
                seen_events.add(event_line.split(": ", 1)[1])
            assert seen_events == {"state", "control_source", "camera_status"}
        finally:
            srv.hub.unregister(client_id)


# ── is_connected: liveness must require BOTH signals ─────────────────────────
#
# Regression guard. The first draft of this node derived `connected` purely
# from `camera_status.status`, which is latched (TRANSIENT_LOCAL). If the PTZ
# node dies, the web node keeps its last "connected" sample indefinitely and
# the dashboard would show a green light over a dead system. The old dashboard
# had the opposite half-answer: it used only telemetry staleness, which cannot
# distinguish "camera unreachable" from "node still starting up".


def test_is_connected_requires_camera_status_connected():
    assert web_node.is_connected({"status": "error"}, 0.1) is False
    assert web_node.is_connected({"status": "connecting"}, 0.1) is False
    assert web_node.is_connected({}, 0.1) is False


def test_is_connected_requires_fresh_telemetry():
    """The failure this exists to catch: PTZ node dead, latched status stale."""
    fresh = {"status": "connected"}
    assert web_node.is_connected(fresh, 0.1) is True
    assert web_node.is_connected(fresh, web_node.STATE_STALE_AFTER_S + 0.01) is False


def test_is_connected_false_before_any_telemetry_arrives():
    assert web_node.is_connected({"status": "connected"}, None) is False


def test_is_connected_boundary_is_exclusive():
    fresh = {"status": "connected"}
    assert web_node.is_connected(fresh, web_node.STATE_STALE_AFTER_S) is False
    assert web_node.is_connected(fresh, web_node.STATE_STALE_AFTER_S - 1e-9) is True


# ── stream host follows live reconfiguration ─────────────────────────────────
#
# Regression guard (PARITY A23 + C14/C15). Stream URLs were built solely from
# the launch-time `camera_host` parameter, so after switching cameras through
# the setup page the dashboard showed correct telemetry from the new camera
# while the video pane still pointed at the old one.


def test_host_from_camera_status_strips_scheme():
    assert web_node.host_from_camera_status({"host": "http://192.0.2.10"}) == "192.0.2.10"
    assert web_node.host_from_camera_status({"host": "https://cam.example"}) == "cam.example"
    assert web_node.host_from_camera_status({"host": "192.0.2.10:8080"}) == "192.0.2.10:8080"
    assert web_node.host_from_camera_status({"host": "http://192.0.2.10/"}) == "192.0.2.10"


def test_host_from_camera_status_empty_lets_caller_keep_its_own():
    """Must not yield '' as a *usable* host -- otherwise stream URLs degrade
    to rtsp://:8554/... which points nowhere."""
    assert web_node.host_from_camera_status({}) == ""
    assert web_node.host_from_camera_status({"host": ""}) == ""
    assert web_node.host_from_camera_status({"host": None}) == ""


def test_streams_never_built_with_an_empty_host():
    streams = web_node.streams_for_model("364c", "192.0.2.10")
    assert all("rtsp://192.0.2.10:" in url for url in streams.values())
    assert not any(url.startswith("rtsp://:") for url in streams.values())
