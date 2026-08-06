#!/usr/bin/env python3
"""FLIR PTZ web bridge node (spec ARCHITECTURE.md sec 5/6, API.md, PARITY.md
rows C1/C2/C13/C14/C15/C17).

Subscribes: flir/ptz/state       (flir_ptz_msgs/PtzState, depth 10)
            flir/camera_status   (std_msgs/String JSON, latched)
            flir/control_source  (flir_ptz_msgs/ControlSource, latched)
Calls:      flir/claim_control   (flir_ptz_msgs/ClaimControl)
Publishes:  flir/cmd/move_to             (flir_ptz_msgs/MoveToCmd)
            flir/cmd/track                (flir_ptz_msgs/MoveToCmd)
            flir/cmd/joy_stick_control    (flir_ptz_msgs/JoyStickControlCmd)
            flir/cmd/scan                 (flir_ptz_msgs/ScanCmd)
            flir/cmd/zoom                  (flir_ptz_msgs/ZoomCmd)  -- EO only
            flir/camera_config             (std_msgs/String JSON)

This node is a thin ROS shell around the stdlib-only HTTP/SSE server in
``webui/server.py`` + ``webui/sse.py``: it implements the ``WebAdapter``
Protocol declared at the top of ``webui/server.py`` directly (every method
that module documents), and pushes ROS subscription data into the
``SSEHub`` it owns.

Arbitration ownership has moved to the PTZ node (spec sec. 5 "重啟語意" /
sec 3.5). This node is NOT an authority: it never runs its own lease timer
and never invents an owner. It only ever does two things around control:

  * mirrors the latched ``flir/control_source`` topic into local state
    (``control_source_snapshot()`` + the optimistic ``locked`` rejection in
    the ``cmd_*`` methods -- purely a UX nicety so a browser gets an
    instant "locked" response instead of a silent drop; the PTZ node's own
    enforcement is what actually matters and happens regardless of what
    this node returns here);
  * forwards ``/api/claim`` and ``/api/unlock`` to the ``flir/claim_control``
    service as a client.

Section boundary: everything above the "ROS shell" marker is pure decision
/ conversion logic -- no I/O, no rclpy import, no wall-clock reads baked
in (time is always an explicit parameter) -- and is unit-testable with
nothing but the standard library, exactly like ``nodes/joy_bridge.py``.
The ``rclpy``-dependent imports are guarded so importing this module (to
get at the pure helpers, or just to import the package) works even when
ROS is not sourced.
"""

from __future__ import annotations

import json
import sys
import threading
import time
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Optional

from flir_ptz.control.arbitration import allows_command
from flir_ptz.control.config import ControlConfig
from flir_ptz.webui.server import PortInUse, ServerConfig, make_server
from flir_ptz.webui.sse import SSEHub

# ═══════════════════════════════════════════════════════════════════════════
# Pure decision / conversion logic (no rclpy, no I/O, no clock reads)
# ═══════════════════════════════════════════════════════════════════════════

# -- model -> RTSP stream config (ported verbatim from flir_ptz_web.py:34-45,
#    PARITY C14/C15). Keys must stay lowercase -- callers always .lower() the
#    model string before looking this up.
MODEL_CONFIGS: dict[str, dict[str, Any]] = {
    "364c": {
        "rtsp_port": 8554,
        "ir_path": "ir.0",
        "eo_path": "vis.0",  # EO available
    },
    "m232": {
        "rtsp_port": 8554,
        "ir_path": "ir",  # rtsp://<ip>:8554/ir
        "eo_path": None,  # M232 has no EO stream
    },
}

DEFAULT_SOURCE = "web"
CLAIM_TIMEOUT_S = 3.0


def streams_for_model(model: str, camera_host: str) -> dict[str, str]:
    """RTSP source URLs for ``model`` (falls back to 364c config for an
    unknown model, matching the old ``_streams`` property). ``eo`` is
    omitted entirely for models with no EO stream (M232) rather than being
    present with a null value -- API.md sec 2's ``streams`` shape is
    ``{"ir": "rtsp://...", "eo": "rtsp://..."}`` with eo optional."""
    cfg = MODEL_CONFIGS.get(model, MODEL_CONFIGS["364c"])
    port = cfg["rtsp_port"]
    result: dict[str, str] = {"ir": f"rtsp://{camera_host}:{port}/{cfg['ir_path']}"}
    if cfg["eo_path"] is not None:
        result["eo"] = f"rtsp://{camera_host}:{port}/{cfg['eo_path']}"
    return result


def mediamtx_patch_url(api_port: int, name: str) -> str:
    """URL for mediamtx's REST config-patch endpoint for path ``name``
    (PARITY C15, ported from flir_ptz_web.py:178)."""
    return f"http://localhost:{api_port}/v3/config/paths/patch/{name}"


def mediamtx_patch_body(url: str) -> dict[str, str]:
    return {"source": url}


def _default_mediamtx_patch(endpoint: str, body: dict[str, str], timeout: float = 2.0) -> None:
    """Real HTTP PATCH via stdlib ``urllib`` (not ``httpx`` -- per
    ARCHITECTURE.md sec 1, only ``nexus/session.py`` may import ``httpx``;
    ``urllib`` is fine and matches the old implementation)."""
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        endpoint, data=data, method="PATCH", headers={"Content-Type": "application/json"}
    )
    urllib.request.urlopen(req, timeout=timeout)


def apply_model_to_mediamtx(
    model: str,
    camera_host: str,
    api_port: int,
    patch_fn: Callable[[str, dict[str, str]], None] = _default_mediamtx_patch,
    on_success: Optional[Callable[[str, str], None]] = None,
    on_error: Optional[Callable[[str, Exception], None]] = None,
) -> None:
    """PARITY C15: PATCH mediamtx's REST config API so its RTSP source
    paths follow ``model``. Resilient by construction -- ``patch_fn``
    failing for a path (mediamtx down/unreachable) is always caught and
    reported via ``on_error`` rather than raised, so this can never crash
    the caller or fail whatever HTTP request triggered the model switch.

    ``patch_fn`` is injectable (defaults to a real ``urllib`` PATCH) so
    this whole resilience contract is unit-testable with no real mediamtx
    process and no network at all."""
    targets = streams_for_model(model, camera_host)
    for name, url in targets.items():
        endpoint = mediamtx_patch_url(api_port, name)
        body = mediamtx_patch_body(url)
        try:
            patch_fn(endpoint, body)
        except Exception as exc:  # mediamtx down/unreachable -- never propagate
            if on_error:
                on_error(name, exc)
        else:
            if on_success:
                on_success(name, url)


def ptz_state_msg_to_state_dict(msg: Any) -> dict[str, Any]:
    """Convert a ``PtzState`` message (or any duck-typed object with the
    same attributes -- this deliberately does not require the real ROS
    message type so it is testable with a plain ``SimpleNamespace``) into
    the inner ``state`` object of the API.md sec 2 ``state`` event
    payload."""
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1_000_000_000
    return {
        "abs_azimuth": msg.abs_azimuth,
        "abs_elevation": msg.abs_elevation,
        "geo_azimuth": msg.geo_azimuth,
        "geo_elevation": msg.geo_elevation,
        "speed_x": msg.speed_x,
        "speed_y": msg.speed_y,
        "mode": msg.mode,
        "is_moving": msg.is_moving,
        "is_scanning": msg.is_scanning,
        "active_move_seq": msg.active_move_seq,
        "active_scan_seq": msg.active_scan_seq,
        "zoom_pctg": msg.zoom_pctg,
        "zoom_mm": msg.zoom_mm,
        "zoom_mag": msg.zoom_mag,
        "ir_zoom_pctg": msg.ir_zoom_pctg,
        "ir_fov": msg.ir_fov,
        "ir_zoom_mag": msg.ir_zoom_mag,
        "stamp": stamp,
    }


def build_state_payload(
    connected: bool,
    state_age_sec: Optional[float],
    model: str,
    streams: dict[str, str],
    control_source: str,
    state: Optional[dict[str, Any]],
) -> dict[str, Any]:
    """Assemble the full API.md sec 2 ``state`` event / ``GET /api/state``
    envelope from already-converted pieces."""
    return {
        "connected": connected,
        "state_age_sec": state_age_sec,
        "model": model,
        "streams": streams,
        "control_source": control_source,
        "state": state,
    }


#: Telemetry older than this is treated as "not live" by :func:`is_connected`.
#: Matches the old dashboard's 2.0 s staleness window.
STATE_STALE_AFTER_S = 2.0


def is_connected(camera_status: dict[str, Any], state_age_sec: Optional[float]) -> bool:
    """Is the dashboard actually seeing a live camera?

    BOTH conditions must hold, and each catches a failure the other misses:

    * ``camera_status.status == "connected"`` — the PTZ node reports an
      authenticated session. On its own this is not enough: that topic is
      latched (TRANSIENT_LOCAL), so if the PTZ node dies, the web node keeps
      its last received "connected" sample forever and the UI would show a
      green light over a dead system.
    * telemetry is fresh — a ``PtzState`` arrived within
      :data:`STATE_STALE_AFTER_S`. On its own this is not enough either: it
      cannot distinguish "camera unreachable" from "node still starting up",
      which is exactly what the setup page needs to tell apart.

    The old dashboard used only the staleness half (flir_ptz_web.py:230).
    """
    if camera_status.get("status") != "connected":
        return False
    if state_age_sec is None:
        return False
    return state_age_sec < STATE_STALE_AFTER_S


def ros_time_to_float(time_msg: Any) -> float:
    """``builtin_interfaces/Time`` (or duck-typed ``sec``/``nanosec``
    object) -> float seconds."""
    return float(time_msg.sec) + float(time_msg.nanosec) / 1_000_000_000.0


def host_from_camera_status(status: dict[str, Any]) -> str:
    """Extract a bare host[:port] from a ``camera_status`` payload.

    The PTZ node reports the base URL it is actually connected to
    (``"http://192.0.2.10"``), while RTSP/WebRTC URLs need the bare host.
    Returns ``""`` when the status carries no usable host, so callers can
    fall back to their configured value rather than building
    ``rtsp://:8554/...``.
    """
    host = str(status.get("host") or "").strip()
    if not host:
        return ""
    for scheme in ("http://", "https://"):
        if host.startswith(scheme):
            host = host[len(scheme):]
            break
    return host.strip("/")


def camera_status_json_to_dict(raw: Any) -> dict[str, str]:
    """Parse the JSON string carried by ``flir/camera_status`` into the
    API.md sec 2 ``camera_status`` event shape. Never raises -- malformed
    JSON, a non-dict payload, or missing fields all degrade to sane
    defaults rather than dropping the update."""
    data: Any = {}
    try:
        data = json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        data = {}
    if not isinstance(data, dict):
        data = {}
    return {
        "status": str(data.get("status", "error")),
        "host": str(data.get("host", "")),
        "error": str(data.get("error", "")),
    }


def mirrored_allows(owner: str, expires_at: float, source: str, is_stop: bool, now: float) -> bool:
    """Optimistic, non-authoritative arbitration check against the *mirrored*
    ``flir/control_source`` latch.

    The PTZ node is the real authority and enforces this independently; this
    exists only so the browser gets an immediate ``locked`` response instead
    of a silently-dropped publish. It therefore delegates to the single shared
    rule rather than re-implementing it -- a previous hand-written copy went
    out of sync the moment the rule changed and blocked every command before
    it could reach the authority.
    """
    return allows_command(owner, expires_at, source, is_stop, now)


def is_zero_speed(az_speed: float, el_speed: float) -> bool:
    """A zero-speed ``joy_stick_control`` is a stop command and must always
    be let through arbitration (API.md sec 3 safety rule)."""
    return az_speed == 0.0 and el_speed == 0.0


def normalize_zoom_direction(direction: Any) -> str:
    """Fail safe, same rule as ``nodes/ptz_node.py``'s ``_on_zoom``: any
    value other than ``"in"``/``"out"`` -- missing, malformed, or simply
    unrecognised -- becomes ``"stop"``. The lens is CONTINUOUS, so "do
    nothing" is never a safe default for a value we don't understand."""
    return direction if direction in ("in", "out") else "stop"


def normalize_zoom_device(device: Any) -> str:
    """Same rule as ``nodes/ptz_node.py``'s ``_on_zoom``: any value other
    than ``"eo"``/``"ir"`` -- missing, malformed, or simply unrecognised --
    defaults to ``"eo"``, so requests that predate this field keep working
    unchanged."""
    return device if device in ("eo", "ir") else "eo"


def to_float(value: Any, default: float) -> float:
    """Best-effort ``float(value)`` that never raises -- POST bodies are
    untrusted JSON and may contain any type."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def resolve_web_root(web_root_param: str) -> Path:
    """Resolve the directory the static web assets are served from
    (PARITY.md's asset-root note).

    Precedence:
      1. ``web_root_param`` if non-empty (the ``web_root`` ROS parameter --
         explicit operator override always wins).
      2. ``ament_index_python``'s installed share directory
         (``share/flir_ptz/web``) -- the normal colcon-installed case.
      3. The source tree next to this file (``flir_ptz/flir_ptz/web``) --
         development fallback when running un-installed, and also what
         keeps this function usable with no ROS/ament present at all.

    Never raises: any failure resolving ament_index (not installed,
    package not found, AMENT_PREFIX_PATH unset) is swallowed and falls
    through to the next candidate.
    """
    if web_root_param:
        return Path(web_root_param).expanduser().resolve()

    try:
        from ament_index_python.packages import get_package_share_directory

        share_web = Path(get_package_share_directory("flir_ptz")) / "web"
        if share_web.is_dir():
            return share_web
    except Exception:
        pass

    # Development fallback: this file lives at flir_ptz/flir_ptz/nodes/web_node.py
    # -> flir_ptz/flir_ptz/web is two levels up.
    return Path(__file__).resolve().parent.parent / "web"


@dataclass
class _MutableWebState:
    """In-memory mirror of everything the HTTP layer needs to answer a GET
    instantly, guarded by ``FlirPtzWebNode._lock``. Never authoritative --
    every field here is just the most recent value observed from a ROS
    subscription (or, for ``model``, an operator's ``/api/model`` request)."""

    model: str = "364c"
    latest_ptz_state: Optional[dict[str, Any]] = None
    state_received_at: Optional[float] = None
    control_source: dict[str, Any] = field(default_factory=lambda: {"owner": "", "expires": 0.0})
    camera_status: dict[str, Any] = field(
        default_factory=lambda: {"status": "connecting", "host": "", "error": ""}
    )


# ═══════════════════════════════════════════════════════════════════════════
# ROS shell (thin: all decisions above are delegated to the pure layer)
# ═══════════════════════════════════════════════════════════════════════════

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )
    from std_msgs.msg import String

    from flir_ptz_msgs.msg import ControlSource, JoyStickControlCmd, MoveToCmd, PtzState, ScanCmd, ZoomCmd
    from flir_ptz_msgs.srv import ClaimControl

    _ROS_AVAILABLE = True
except ImportError:  # pragma: no cover -- exercised by running tests with no ROS sourced
    Node = object  # type: ignore[assignment,misc]
    _ROS_AVAILABLE = False


if _ROS_AVAILABLE:

    # flir/control_source and flir/camera_status are latched facts (spec
    # sec 5 "QoS"): TRANSIENT_LOCAL/RELIABLE/KEEP_LAST(1) so a late-joining
    # subscriber (this node restarting) immediately gets the current value
    # instead of waiting for the next change.
    _LATCHED_QOS = QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
    )

    class FlirPtzWebNode(Node):
        """Implements ``webui.server.WebAdapter`` directly -- every public
        method below is one the Protocol in ``webui/server.py`` declares."""

        def __init__(self) -> None:
            super().__init__("flir_ptz_web")

            self.declare_parameter("bind_host", "0.0.0.0")
            self.declare_parameter("port", 8080)
            # Never a real IP by default (spec sec 8 rule 1) -- empty until
            # a setup-page /api/connect or launch override supplies one.
            self.declare_parameter("camera_host", "")
            self.declare_parameter("login_mode", "basic")
            self.declare_parameter("mediamtx_api_port", 9997)
            self.declare_parameter("enable_ffplay", False)
            # Empty = auto-resolve via resolve_web_root().
            self.declare_parameter("web_root", "")

            bind_host = str(self.get_parameter("bind_host").value)
            port = int(self.get_parameter("port").value)
            self._camera_host = str(self.get_parameter("camera_host").value)
            login_mode = str(self.get_parameter("login_mode").value).lower()
            self._mediamtx_api_port = int(self.get_parameter("mediamtx_api_port").value)
            enable_ffplay = bool(self.get_parameter("enable_ffplay").value)
            web_root_param = str(self.get_parameter("web_root").value)

            self._lock = threading.Lock()
            self._mstate = _MutableWebState(model="m232" if login_mode == "post" else "364c")

            # -- command publishers (source field carried straight through
            #    from the HTTP request body -- API.md sec 3) --
            self._move_pub = self.create_publisher(MoveToCmd, "flir/cmd/move_to", 10)
            self._track_pub = self.create_publisher(MoveToCmd, "flir/cmd/track", 10)
            self._joy_pub = self.create_publisher(JoyStickControlCmd, "flir/cmd/joy_stick_control", 10)
            self._scan_pub = self.create_publisher(ScanCmd, "flir/cmd/scan", 10)
            self._zoom_pub = self.create_publisher(ZoomCmd, "flir/cmd/zoom", 10)
            self._config_pub = self.create_publisher(String, "flir/camera_config", 5)

            self.create_subscription(PtzState, "flir/ptz/state", self._on_state, 10)
            self.create_subscription(String, "flir/camera_status", self._on_camera_status, _LATCHED_QOS)
            self.create_subscription(
                ControlSource, "flir/control_source", self._on_control_source, _LATCHED_QOS
            )

            self._claim_client = self.create_client(ClaimControl, "flir/claim_control")

            self._sse_hub = SSEHub()
            web_root = resolve_web_root(web_root_param)
            self._http_config = ServerConfig(
                web_root=web_root,
                bind_host=bind_host,
                port=port,
                enable_ffplay=enable_ffplay,
            )
            self._http_server = make_server(
                self._http_config, self, self._sse_hub, log=self.get_logger().debug
            )
            self._http_thread = threading.Thread(
                target=self._http_server.serve_forever,
                daemon=True,
                name="flir_ptz_web_http",
            )
            self._http_thread.start()
            self.get_logger().info(
                f"FLIR PTZ web UI listening on http://{bind_host}:{port} "
                f"(web_root={web_root}, camera_host={self._camera_host or '(unset)'})"
            )

        # ── snapshots (WebAdapter protocol) ─────────────────────────────

        def state_snapshot(self) -> dict[str, Any]:
            with self._lock:
                state = dict(self._mstate.latest_ptz_state) if self._mstate.latest_ptz_state else None
                received_at = self._mstate.state_received_at
                model = self._mstate.model
                owner = self._mstate.control_source.get("owner", "")
                camera_status = dict(self._mstate.camera_status)
            age = None if received_at is None else max(0.0, time.monotonic() - received_at)
            connected = is_connected(camera_status, age)
            streams = streams_for_model(model, self._camera_host)
            return build_state_payload(connected, age, model, streams, owner, state)

        def control_source_snapshot(self) -> dict[str, Any]:
            with self._lock:
                return dict(self._mstate.control_source)

        def camera_status_snapshot(self) -> dict[str, Any]:
            with self._lock:
                return dict(self._mstate.camera_status)

        def model_snapshot(self) -> dict[str, Any]:
            with self._lock:
                model = self._mstate.model
            return {
                "model": model,
                "streams": streams_for_model(model, self._camera_host),
                "models": list(MODEL_CONFIGS.keys()),
            }

        def stream_url(self, key: str) -> Optional[str]:
            with self._lock:
                model = self._mstate.model
            return streams_for_model(model, self._camera_host).get(key)

        # ── ROS subscriptions -> SSE broadcast ──────────────────────────

        def _on_state(self, msg: "PtzState") -> None:
            state_dict = ptz_state_msg_to_state_dict(msg)
            with self._lock:
                self._mstate.latest_ptz_state = state_dict
                self._mstate.state_received_at = time.monotonic()
            # state_snapshot() re-reads the lock, which is fine -- this
            # callback is not itself hot enough (~10Hz) to need to avoid
            # the extra acquire, and it keeps GET /api/state and the SSE
            # event byte-for-byte identical by construction.
            self._sse_hub.broadcast("state", self.state_snapshot())

        def _on_camera_status(self, msg: "String") -> None:
            status = camera_status_json_to_dict(msg.data)
            with self._lock:
                self._mstate.camera_status = status
                # Track the camera the PTZ node is *actually* talking to.
                #
                # Stream URLs used to be built solely from the launch-time
                # `camera_host` parameter, so after a live reconfigure through
                # the setup page (PARITY A23) the RTSP/WebRTC URLs still
                # pointed at the previous camera -- the dashboard would show
                # correct telemetry from the new one while the video pane
                # silently streamed the old one, or nothing at all.
                host = host_from_camera_status(status)
                if host:
                    self._camera_host = host
            self._sse_hub.broadcast("camera_status", status)

        def _on_control_source(self, msg: "ControlSource") -> None:
            payload = {"owner": msg.owner, "expires": ros_time_to_float(msg.expires)}
            with self._lock:
                self._mstate.control_source = payload
            self._sse_hub.broadcast("control_source", payload)

        # ── commands (WebAdapter protocol) ──────────────────────────────
        # None of these are the arbitration authority (see module
        # docstring): the "locked" rejection below is an optimistic check
        # against the mirrored latch purely so the browser gets instant
        # feedback instead of a silently-dropped publish. The PTZ node
        # enforces the real rule independently.

        def _allowed(self, source: str, is_stop: bool) -> bool:
            with self._lock:
                owner = self._mstate.control_source.get("owner", "")
                expires = self._mstate.control_source.get("expires", 0.0)
            return mirrored_allows(owner, expires, source, is_stop, time.time())

        def _locked_response(self, extra: Optional[dict[str, Any]] = None) -> dict[str, Any]:
            with self._lock:
                owner = self._mstate.control_source.get("owner", "")
            resp: dict[str, Any] = {"ok": False, "locked": owner}
            if extra:
                resp.update(extra)
            return resp

        def cmd_move_to(self, body: dict[str, Any]) -> dict[str, Any]:
            az = to_float(body.get("target_azimuth"), 0.0)
            el = to_float(body.get("target_elevation"), 0.0)
            source = str(body.get("source") or DEFAULT_SOURCE)
            if not self._allowed(source, is_stop=False):
                return self._locked_response()
            msg = MoveToCmd()
            msg.target_azimuth = az
            msg.target_elevation = el
            msg.source = source
            self._move_pub.publish(msg)
            return {"ok": True, "target_azimuth": az, "target_elevation": el}

        def cmd_track(self, body: dict[str, Any]) -> dict[str, Any]:
            az = to_float(body.get("target_azimuth"), 0.0)
            el = to_float(body.get("target_elevation"), 0.0)
            source = str(body.get("source") or DEFAULT_SOURCE)
            if not self._allowed(source, is_stop=False):
                return self._locked_response()
            msg = MoveToCmd()
            msg.target_azimuth = az
            msg.target_elevation = el
            msg.source = source
            self._track_pub.publish(msg)
            return {"ok": True, "target_azimuth": az, "target_elevation": el}

        def cmd_joy_stick_control(self, body: dict[str, Any]) -> dict[str, Any]:
            az = to_float(body.get("azimuth_speed"), 0.0)
            el = to_float(body.get("elevation_speed"), 0.0)
            source = str(body.get("source") or DEFAULT_SOURCE)
            if not self._allowed(source, is_stop=is_zero_speed(az, el)):
                return self._locked_response()
            msg = JoyStickControlCmd()
            msg.azimuth_speed = az
            msg.elevation_speed = el
            msg.source = source
            self._joy_pub.publish(msg)
            return {"ok": True, "azimuth_speed": az, "elevation_speed": el}

        def cmd_scan(self, body: dict[str, Any]) -> dict[str, Any]:
            stop = bool(body.get("stop", False))
            source = str(body.get("source") or DEFAULT_SOURCE)
            if not self._allowed(source, is_stop=stop):
                return self._locked_response({"stop": stop})
            msg = ScanCmd()
            msg.stop = stop
            if not stop:
                msg.center_azimuth = to_float(body.get("center_azimuth"), 0.0)
                msg.each_side_deg = to_float(body.get("each_side_deg"), 20.0)
                msg.elevation = to_float(body.get("elevation"), 0.0)
                msg.speed = to_float(body.get("speed"), 5.0)
            msg.source = source
            self._scan_pub.publish(msg)
            return {"ok": True, "stop": stop}

        def cmd_zoom(self, body: dict[str, Any]) -> dict[str, Any]:
            direction = normalize_zoom_direction(str(body.get("direction", "stop")))
            device = normalize_zoom_device(str(body.get("device", "eo")))
            source = str(body.get("source") or DEFAULT_SOURCE)
            if not self._allowed(source, is_stop=direction == "stop"):
                return self._locked_response({"direction": direction, "device": device})
            msg = ZoomCmd()
            msg.direction = direction
            msg.device = device
            msg.source = source
            self._zoom_pub.publish(msg)
            return {"ok": True, "direction": direction, "device": device}

        def cmd_home(self, body: dict[str, Any]) -> dict[str, Any]:
            source = str(body.get("source") or DEFAULT_SOURCE)
            if not self._allowed(source, is_stop=False):
                return self._locked_response()
            cfg = ControlConfig()
            msg = MoveToCmd()
            msg.target_azimuth = cfg.home_az
            msg.target_elevation = cfg.home_el
            msg.source = source
            self._move_pub.publish(msg)
            return {"ok": True, "target_azimuth": cfg.home_az, "target_elevation": cfg.home_el}

        # ── control-source arbitration client (WebAdapter protocol) ────

        def claim(self, source: str) -> dict[str, Any]:
            result = self._call_claim_service(source, release=False)
            return {"ok": True, "granted": result["granted"], "owner": result["owner"]}

        def unlock(self) -> dict[str, Any]:
            with self._lock:
                owner = self._mstate.control_source.get("owner", "") or DEFAULT_SOURCE
            self._call_claim_service(owner, release=True)
            return {"ok": True}

        def _call_claim_service(self, source: str, release: bool) -> dict[str, Any]:
            with self._lock:
                fallback_owner = self._mstate.control_source.get("owner", "")
            fallback = {"granted": False, "owner": fallback_owner, "expires": 0.0}

            if not self._claim_client.service_is_ready():
                self.get_logger().warning("flir/claim_control service not available")
                return fallback

            request = ClaimControl.Request()
            request.source = source
            request.release = release
            future = self._claim_client.call_async(request)
            done = threading.Event()
            future.add_done_callback(lambda _f: done.set())
            if not done.wait(timeout=CLAIM_TIMEOUT_S):
                self.get_logger().warning("flir/claim_control call timed out")
                return fallback
            try:
                response = future.result()
            except Exception as exc:  # service down, node restarted mid-call, ...
                self.get_logger().warning(f"flir/claim_control call failed: {exc}")
                return fallback
            return {
                "granted": bool(response.granted),
                "owner": response.owner,
                "expires": ros_time_to_float(response.expires),
            }

        # ── setup (WebAdapter protocol) ──────────────────────────────────

        def connect(self, host: str, model: str, username: str, password: str) -> None:
            """Non-blocking (PARITY.md, API.md sec 5 -- fixes the old 15s
            ``threading.Event().wait()`` defect at flir_ptz_web.py:421):
            publish the config and return immediately. The PTZ node
            reconnects asynchronously and the outcome reaches the browser
            later purely via the ``camera_status`` SSE event."""
            payload = {"host": host, "model": model, "username": username, "password": password}
            msg = String()
            msg.data = json.dumps(payload)
            self._config_pub.publish(msg)

        def set_model(self, model: str) -> dict[str, Any]:
            model = model.lower()
            if model not in MODEL_CONFIGS:
                with self._lock:
                    current = self._mstate.model
                return {"ok": False, "model": current, "streams": streams_for_model(current, self._camera_host)}
            with self._lock:
                self._mstate.model = model
            self.get_logger().info(f"Model switched to: {model.upper()}")
            self._apply_model_to_mediamtx(model)
            return {"ok": True, "model": model, "streams": streams_for_model(model, self._camera_host)}

        def _apply_model_to_mediamtx(self, model: str) -> None:
            """PARITY C15: PATCH mediamtx's REST config API so its RTSP
            source paths follow the new model. mediamtx being unreachable
            must only ever log a warning -- never raise (this runs inside
            an HTTP request handler thread and must not fail the
            /api/model response, nor crash the node). See
            ``apply_model_to_mediamtx`` for the resilient, testable core."""
            apply_model_to_mediamtx(
                model,
                self._camera_host,
                self._mediamtx_api_port,
                on_success=lambda name, url: self.get_logger().info(f"mediamtx: {name} -> {url}"),
                on_error=lambda name, exc: self.get_logger().warning(f"mediamtx API ({name}): {exc}"),
            )

        # ── shutdown ─────────────────────────────────────────────────────

        def destroy_node(self) -> None:
            self._http_server.shutdown()
            self._http_server.server_close()
            self._http_thread.join(timeout=2.0)
            super().destroy_node()

    def main(args: Optional[list] = None) -> None:
        rclpy.init(args=args)
        try:
            node = FlirPtzWebNode()
        except PortInUse as exc:
            # Report the one actionable line rather than a traceback through
            # socketserver internals. Nothing here is a bug to be debugged --
            # a port is busy, and the message says which and what to do.
            print(f"\n[flir_ptz_web] {exc.strerror}\n", file=sys.stderr)
            rclpy.try_shutdown()
            raise SystemExit(1)
        # MultiThreadedExecutor (not plain rclpy.spin): claim()/unlock()
        # block an HTTP worker thread on a ClaimControl future via a
        # threading.Event -- that future's done-callback only fires once
        # the executor services the client's response, which requires a
        # spin thread distinct from whichever thread is blocked waiting.
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    if not _ROS_AVAILABLE:
        raise RuntimeError("flir_ptz.nodes.web_node requires rclpy; source your ROS 2 setup.bash")
    main()
