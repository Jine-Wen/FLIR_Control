#!/usr/bin/env python3
"""HTTP server for the FLIR PTZ web console (spec ARCHITECTURE.md sec. 6,
API.md). Replaces the polling dashboard bridge in the old
``flir_ptz_web.py`` with a thin, stdlib-only transport layer built around
the SSE push channel in ``webui/sse.py``.

Hard rule: this module (like ``webui/sse.py``) MUST NOT import ``rclpy`` or
``httpx``. The whole HTTP layer must be constructible and testable with no
ROS installed and no camera present. The bridge to ROS is the
:class:`WebAdapter` protocol below -- ``nodes/web_node.py`` (owned by
another worker) implements it and hands an instance, plus an
:class:`~flir_ptz.webui.sse.SSEHub` it pushes into on every ROS callback, to
:func:`make_server`.

Fixed defects relative to the old implementation (see PARITY.md, API.md
sec. 5):

* ``/api/connect`` no longer blocks an HTTP worker thread on a
  ``threading.Event`` for up to 15s (old ``flir_ptz_web.py:421``). It
  returns ``{"ok": true, "pending": true}`` immediately; the real outcome
  reaches the browser later via the ``camera_status`` SSE event.
* The MJPEG fallback's ffmpeg subprocess is *always* reaped (terminate,
  then kill if it ignores that) when the client disconnects, instead of the
  old code's bare ``except Exception: pass`` around a single
  ``wait(timeout=2)`` that could leave an orphaned ffmpeg running forever
  if the process ignored SIGTERM.
"""

from __future__ import annotations

import errno
import json
import mimetypes
import os
import subprocess
import threading
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path, PurePosixPath
from typing import Any, Callable, Optional, Protocol
from urllib.parse import unquote, urlparse

from flir_ptz.webui.sse import KEEPALIVE_INTERVAL_S, SSEHub, format_keepalive

class PortInUse(OSError):
    """The web console's port is already bound by something else.

    Its own message is the whole point: the stdlib raises a bare
    ``OSError: [Errno 98] Address already in use`` from deep inside
    ``socketserver``, which never mentions which port, which process, or what
    to do about it.
    """

    def __init__(self, host: str, port: int) -> None:
        super().__init__(
            errno.EADDRINUSE,
            f"web console port {port} is already in use on {host}.\n"
            f"  Another instance is probably still running. Check with:\n"
            f"      ss -tlnp | grep {port}\n"
            f"  Stop it, or start this one on a different port:\n"
            f"      ros2 launch flir_ptz flir_ptz.launch.py web_port:=<other>",
        )
        self.host = host
        self.port = port


# -- The ROS-side contract ---------------------------------------------------
#
# Everything below this line down to `class WebAdapter` is pure interface
# documentation: `nodes/web_node.py` implements this duck-typed object and
# passes it into `make_server`. Keep it minimal -- every method here is
# something an HTTP route in this module actually calls.
#
# Threading: every method is called from an arbitrary ThreadingHTTPServer
# worker thread and MUST be safe to call concurrently from many threads at
# once. None of them may block on camera/network I/O -- `connect()` in
# particular must return near-instantly (see its docstring); publishing a
# ROS message and returning is fine, waiting for a reply is not.


class WebAdapter(Protocol):
    """Duck-typed adapter between the HTTP layer and the ROS web node.

    A real implementation typically wraps ROS publishers/services on
    ``nodes/web_node.py``; nothing in this module ever imports ``rclpy``, so
    tests can implement this Protocol with a plain Python object (see
    ``test/test_sse.py``'s ``FakeAdapter``) and exercise the whole HTTP
    surface with no ROS present.
    """

    # -- snapshots ------------------------------------------------------
    # Used both for the "reply to a synchronous debug GET" endpoints
    # (API.md sec. 5) and, by the caller of SSEHub.broadcast (the ROS
    # node, not this module), as the shape of the corresponding SSE event
    # payload. Each must return a plain JSON-serializable dict matching the
    # shape documented in API.md sec. 2 for the same-named event.

    def state_snapshot(self) -> dict[str, Any]:
        """Same shape as the SSE ``state`` event payload: ``{"connected",
        "state_age_sec", "model", "streams", "control_source", "state"}``."""
        ...

    def control_source_snapshot(self) -> dict[str, Any]:
        """Same shape as the SSE ``control_source`` event payload:
        ``{"owner", "expires"}``."""
        ...

    def camera_status_snapshot(self) -> dict[str, Any]:
        """Same shape as the SSE ``camera_status`` event payload:
        ``{"status", "host", "error"}``."""
        ...

    def model_snapshot(self) -> dict[str, Any]:
        """``{"model": str, "streams": {"ir": url, "eo": url?}, "models":
        [str, ...]}``."""
        ...

    def stream_url(self, key: str) -> Optional[str]:
        """RTSP source URL for ``key`` (``"ir"`` or ``"eo"``), or ``None``
        if that stream doesn't exist for the current model (e.g. ``eo`` on
        an M232). Backs the MJPEG fallback endpoints."""
        ...

    # -- commands ---------------------------------------------------------
    # `body` is the already-JSON-parsed POST body (``{}`` if the request
    # had no/invalid body). Each returns the dict to send back verbatim as
    # the JSON response -- including the `{"ok": false, "locked": "joy"}`
    # arbitration-rejection shape (API.md sec. 3), which is still an HTTP
    # 200, not an error status.

    def cmd_move_to(self, body: dict[str, Any]) -> dict[str, Any]: ...

    def cmd_track(self, body: dict[str, Any]) -> dict[str, Any]: ...

    def cmd_joy_stick_control(self, body: dict[str, Any]) -> dict[str, Any]: ...

    def cmd_scan(self, body: dict[str, Any]) -> dict[str, Any]: ...

    def cmd_zoom(self, body: dict[str, Any]) -> dict[str, Any]:
        """EO (daylight) or IR (thermal) lens zoom -- ``body["direction"]``
        is ``"in"`` | ``"out"`` | ``"stop"`` and ``body["device"]`` is
        ``"eo"`` | ``"ir"`` (absent/unrecognised defaults to ``"eo"``).
        This module never inspects ``device`` itself -- it is opaque body
        content forwarded verbatim to the adapter, exactly like every other
        POST field (see the route table below)."""
        ...

    def cmd_home(self, body: dict[str, Any]) -> dict[str, Any]: ...

    # -- control-source arbitration (API.md sec. 4) ------------------------

    def claim(self, source: str) -> dict[str, Any]:
        """``{"ok": true, "granted": bool, "owner": str}``."""
        ...

    def unlock(self) -> dict[str, Any]:
        """``{"ok": true}``."""
        ...

    # -- setup (API.md sec. 5) ---------------------------------------------

    def connect(self, host: str, model: str, username: str, password: str) -> None:
        """Kick off a (re)connect to the camera and return immediately --
        MUST NOT block on the camera responding. The outcome is delivered
        later purely via the ``camera_status`` SSE event (and reflected in
        the next ``camera_status_snapshot()``/``model_snapshot()`` call);
        this method's return value is not consulted by the caller."""
        ...

    def set_model(self, model: str) -> dict[str, Any]:
        """``{"ok": bool, "model": str, "streams": {...}}``."""
        ...


# -- ffplay (server-local subprocess launcher; no ROS involved) -------------
# Ported from flir_ptz_web.py:47-64, gated behind `enable_ffplay` (default
# False) per API.md sec. 6 / PARITY.md's "retained but downgraded" table:
# launching a GUI window from the server process is bad architecture, kept
# only as an explicit opt-in so it doesn't regress a feature that existed.

FFPLAY_ARGS = [
    "ffplay",
    "-rtsp_transport", "udp",
    "-fflags", "nobuffer+discardcorrupt",
    "-flags", "low_delay",
    "-probesize", "32",
    "-analyzeduration", "0",
    "-max_delay", "0",
    "-sync", "ext",
    "-framedrop",
    "-vf", "setpts=0",
    "-an",
]

FFPLAY_WINDOWS = {
    "ir": {"title": "FLIR IR", "left": "40", "top": "80"},
    "eo": {"title": "FLIR EO", "left": "960", "top": "80"},
}


def _build_mjpeg_cmd(stream_key: str, url: str) -> list[str]:
    """ffmpeg command for the MJPEG fallback endpoints. Ported verbatim from
    flir_ptz_web.py:498-528 (IR gets one extra low-latency flag,
    ``-avioflags direct``, that EO doesn't)."""
    base = [
        "ffmpeg",
        "-fflags", "nobuffer+discardcorrupt",
        "-flags", "low_delay",
        "-rtsp_transport", "udp",
    ]
    if stream_key == "ir":
        base += ["-avioflags", "direct"]
    base += [
        "-probesize", "32",
        "-analyzeduration", "0",
        "-max_delay", "0",
        "-i", url,
        "-an",
        "-vf", "setpts=0,scale=640:360",
        "-f", "mjpeg", "-q:v", "3", "-r", "25",
        "pipe:1",
    ]
    return base


def _reap(proc: subprocess.Popen) -> None:
    """Make sure ``proc`` is dead and waited-on, no matter what. The old
    code's cleanup (flir_ptz_web.py:564-569) was a bare ``terminate()`` +
    ``wait(timeout=2)`` wrapped in ``except Exception: pass`` -- if the
    process ignored SIGTERM the ``wait`` raised ``TimeoutExpired``, was
    swallowed, and ffmpeg kept running orphaned forever. This always
    escalates to SIGKILL and waits again before giving up."""
    if proc.poll() is not None:
        return
    try:
        proc.terminate()
        proc.wait(timeout=2.0)
        return
    except subprocess.TimeoutExpired:
        pass
    except Exception:
        pass
    try:
        proc.kill()
        proc.wait(timeout=2.0)
    except Exception:
        pass


class _FfplayManager:
    """Server-local ffplay process bookkeeping. Independent of WebAdapter --
    stream URLs still come from the adapter (they depend on the camera
    model), but launching/stopping a local GUI window is a pure HTTP-layer
    concern."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._procs: dict[str, subprocess.Popen] = {}

    def launch(self, stream: str, url: str) -> dict[str, Any]:
        with self._lock:
            current = self._procs.get(stream)
            if current and current.poll() is None:
                return {"ok": True, "stream": stream, "pid": current.pid, "already_running": True}
            window = FFPLAY_WINDOWS.get(stream, {})
            window_args = [
                "-window_title", window.get("title", f"FLIR {stream.upper()}"),
                "-x", "880",
                "-y", "560",
                "-left", window.get("left", "40"),
                "-top", window.get("top", "80"),
            ]
            proc = subprocess.Popen(FFPLAY_ARGS + window_args + [url])
            self._procs[stream] = proc
            return {"ok": True, "stream": stream, "pid": proc.pid, "url": url, "window": window}

    def stop(self, stream: Optional[str] = None) -> dict[str, Any]:
        stopped: list[str] = []
        with self._lock:
            for name, proc in list(self._procs.items()):
                if stream is not None and name != stream:
                    continue
                _reap(proc)
                stopped.append(name)
                self._procs.pop(name, None)
        return {"ok": True, "stopped": stopped}

    def stop_all(self) -> None:
        self.stop(None)


# -- server configuration -----------------------------------------------------


@dataclass
class ServerConfig:
    web_root: Path
    bind_host: str = "0.0.0.0"
    port: int = 8080
    enable_ffplay: bool = False
    keepalive_interval_s: float = KEEPALIVE_INTERVAL_S
    static_index: dict[str, str] = field(
        default_factory=lambda: {"/setup": "setup.html", "/control": "index.html"}
    )


def _read_json_body(handler: BaseHTTPRequestHandler) -> dict[str, Any]:
    length = int(handler.headers.get("Content-Length", "0") or "0")
    if length <= 0:
        return {}
    raw = handler.rfile.read(length)
    try:
        parsed = json.loads(raw.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError):
        return {}
    return parsed if isinstance(parsed, dict) else {}


def make_server(
    config: ServerConfig,
    adapter: WebAdapter,
    sse_hub: SSEHub,
    log: Optional[Callable[[str], None]] = None,
) -> ThreadingHTTPServer:
    """Build (but do not start) a ``ThreadingHTTPServer`` implementing the
    whole route table in API.md sec. 1-6.

    ``log``, if given, receives one-line strings for request logging
    (``nodes/web_node.py`` typically passes ``node.get_logger().debug``);
    default is silent, matching the old handler's behaviour of routing logs
    through the ROS logger rather than stderr.
    """
    web_root = config.web_root.resolve()
    ffplay = _FfplayManager()
    _noop_log: Callable[[str], None] = lambda _msg: None
    logf = log or _noop_log

    class Handler(BaseHTTPRequestHandler):
        server_version = "FlirPtzWeb/2.0"
        protocol_version = "HTTP/1.1"

        # -- routing ------------------------------------------------------

        def do_GET(self) -> None:  # noqa: N802 (stdlib override)
            parsed = urlparse(self.path)
            path = parsed.path

            if path == "/":
                self.send_response(302)
                self.send_header("Location", "/setup")
                self.send_header("Content-Length", "0")
                self.end_headers()
                return

            if path in config.static_index:
                self._send_static(config.static_index[path])
                return

            if path == "/api/events":
                self._handle_sse()
                return

            if path == "/api/state":
                self._send_json(adapter.state_snapshot())
                return

            if path == "/api/control_source":
                snap = adapter.control_source_snapshot()
                self._send_json({"ok": True, "source": snap.get("owner", "")})
                return

            if path == "/api/connection_status":
                status = adapter.camera_status_snapshot()
                connected = status.get("status") == "connected"
                self._send_json({"ok": True, "connected": connected, "status": status})
                return

            if path == "/api/model":
                snap = adapter.model_snapshot()
                self._send_json({"ok": True, **snap})
                return

            if path in ("/api/stream/ir", "/api/stream/eo"):
                self._send_mjpeg(path.rsplit("/", 1)[-1])
                return

            self._send_static(unquote(path))

        def do_POST(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            path = parsed.path
            body = _read_json_body(self)

            if path == "/api/connect":
                self._handle_connect(body)
                return

            if path == "/api/cmd/move_to":
                self._send_json(adapter.cmd_move_to(body))
                return
            if path == "/api/cmd/track":
                self._send_json(adapter.cmd_track(body))
                return
            if path == "/api/cmd/joy_stick_control":
                self._send_json(adapter.cmd_joy_stick_control(body))
                return
            if path == "/api/cmd/scan":
                self._send_json(adapter.cmd_scan(body))
                return
            if path == "/api/cmd/zoom":
                self._send_json(adapter.cmd_zoom(body))
                return
            if path == "/api/cmd/home":
                self._send_json(adapter.cmd_home(body))
                return

            if path == "/api/claim":
                src = str(body.get("source", "")).lower()
                if src not in ("web", "joy"):
                    self._send_json({"ok": False, "error": "source must be 'web' or 'joy'"}, status=400)
                    return
                self._send_json(adapter.claim(src))
                return

            if path == "/api/unlock":
                self._send_json(adapter.unlock())
                return

            if path == "/api/model":
                model = str(body.get("model", "")).lower()
                self._send_json(adapter.set_model(model))
                return

            if path == "/api/video/launch":
                self._handle_video_launch(body)
                return
            if path == "/api/video/stop":
                self._handle_video_stop(body)
                return

            self._send_json({"ok": False, "error": "not found"}, status=404)

        # -- /api/connect: MUST be non-blocking (PARITY, API.md sec. 5) ---

        def _handle_connect(self, body: dict[str, Any]) -> None:
            host = str(body.get("host", "")).strip()
            model = str(body.get("model", "364c")).strip()
            username = str(body.get("username", "")).strip()
            password = str(body.get("password", "")).strip()
            if not host:
                self._send_json({"ok": False, "error": "host is required"}, status=400)
                return
            if not password:
                self._send_json({"ok": False, "error": "password is required"}, status=400)
                return
            # Fire-and-forget: adapter.connect() must itself be non-blocking
            # (publish a ROS message and return). The real outcome arrives
            # later via the camera_status SSE event -- never wait on it here.
            adapter.connect(host, model, username, password)
            self._send_json({"ok": True, "pending": True})

        # -- SSE ------------------------------------------------------------

        def _handle_sse(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            client_id, client_q = sse_hub.register()
            try:
                while True:
                    frame = client_q.get(timeout=config.keepalive_interval_s)
                    if frame is None:
                        frame = format_keepalive()
                    self.wfile.write(frame)
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                # Client went away -- expected and harmless. Registry
                # cleanup happens in `finally` below regardless of why the
                # loop stopped.
                pass
            finally:
                sse_hub.unregister(client_id)

        # -- MJPEG fallback --------------------------------------------------

        def _send_mjpeg(self, stream_key: str) -> None:
            url = adapter.stream_url(stream_key)
            if not url:
                self.send_error(404)
                return
            cmd = _build_mjpeg_cmd(stream_key, url)
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            try:
                buf = b""
                while True:
                    assert proc.stdout is not None
                    chunk = proc.stdout.read(4096)
                    if not chunk:
                        break
                    buf += chunk
                    while True:
                        start = buf.find(b"\xff\xd8")
                        if start == -1:
                            break
                        end = buf.find(b"\xff\xd9", start + 2)
                        if end == -1:
                            break
                        frame = buf[start:end + 2]
                        buf = buf[end + 2:]
                        header = (
                            f"--frame\r\nContent-Type: image/jpeg\r\n"
                            f"Content-Length: {len(frame)}\r\n\r\n"
                        ).encode()
                        self.wfile.write(header)
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                # Always reaped, whether the client disconnected (write
                # raised above), ffmpeg exited/crashed, or the loop broke
                # normally -- see `_reap`'s docstring for the defect this
                # fixes relative to the old code.
                if proc.stdout is not None:
                    try:
                        proc.stdout.close()
                    except Exception:
                        pass
                _reap(proc)

        # -- ffplay (gated) ---------------------------------------------------

        def _handle_video_launch(self, body: dict[str, Any]) -> None:
            if not config.enable_ffplay:
                self._send_json({"ok": False, "error": "ffplay disabled"})
                return
            stream = str(body.get("stream", ""))
            url = adapter.stream_url(stream)
            if url is None:
                self._send_json({"ok": False, "error": "unknown stream"})
                return
            self._send_json(ffplay.launch(stream, url))

        def _handle_video_stop(self, body: dict[str, Any]) -> None:
            if not config.enable_ffplay:
                self._send_json({"ok": False, "error": "ffplay disabled"})
                return
            stream = body.get("stream")
            self._send_json(ffplay.stop(str(stream) if stream else None))

        # -- static assets, with path-traversal guard (PARITY C18) -----------
        # Ported from flir_ptz_web.py:580-597. `unquote()` is applied by the
        # caller (do_GET) before this is reached, so percent-encoded escapes
        # (e.g. "%2e%2e") are resolved before the realpath check runs.

        def _send_static(self, request_path: str) -> None:
            rel = "index.html" if request_path in ("", "/") else request_path.lstrip("/")

            # Containment check is LEXICAL (normpath), not physical (resolve).
            #
            # Resolving first breaks `colcon build --symlink-install`, which
            # installs each web asset as an individual symlink back into the
            # source tree: `(web_root / "setup.html").resolve()` then lands in
            # .../flir_ptz/flir_ptz/web/, whose parents no longer include
            # web_root, and every page 403s. Normalising lexically still
            # collapses any "..", so traversal out of the root is rejected,
            # while a symlinked asset inside the root is served normally.
            if PurePosixPath(rel).is_absolute() or ".." in PurePosixPath(rel).parts:
                self.send_error(403)
                return
            target = Path(os.path.normpath(str(web_root / rel)))
            if web_root not in target.parents and target != web_root:
                self.send_error(403)
                return
            if not target.exists() or not target.is_file():
                self.send_error(404)
                return
            ctype = mimetypes.guess_type(str(target))[0] or "application/octet-stream"
            data = target.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(data)

        # -- helpers ----------------------------------------------------------

        def _send_json(self, payload: dict[str, Any], status: int = 200) -> None:
            data = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, fmt: str, *args: Any) -> None:  # noqa: N802
            logf(fmt % args)

    try:
        server = ThreadingHTTPServer((config.bind_host, config.port), Handler)
    except OSError as exc:
        # A busy port is an ordinary operator mistake -- usually a previous
        # instance still running -- and deserves a sentence, not a twenty-line
        # traceback through socketserver internals that never names the port.
        if exc.errno == errno.EADDRINUSE:
            raise PortInUse(config.bind_host, config.port) from exc
        raise
    server.daemon_threads = True
    server._ffplay_manager = ffplay  # type: ignore[attr-defined]  # for shutdown, see below

    _orig_close = server.server_close

    def _server_close() -> None:
        ffplay.stop_all()
        _orig_close()

    server.server_close = _server_close  # type: ignore[method-assign]
    return server
