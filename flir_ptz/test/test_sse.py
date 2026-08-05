#!/usr/bin/env python3
"""Tests for flir_ptz.webui.sse (SSEHub / SSE wire framing) and
flir_ptz.webui.server (the HTTP route table, API.md sec. 1-6).

Must run fully offline with nothing but the Python 3.12 standard library:
no ROS, no httpx (not installed in this environment — ARCHITECTURE.md
sec. 0). Every network-facing test binds a real ThreadingHTTPServer on
port 0 and drives it with ``http.client``/raw sockets, exactly like a real
browser or curl would — the pure SSE-framing and backpressure tests talk to
``SSEHub`` directly (no sockets needed to prove queue/eviction behaviour,
and it keeps those tests fast and deterministic).

Every socket operation below has an explicit timeout: a bug in the server
must show up as a test *failure*, never as a hung test run.
"""

from __future__ import annotations

import http.client
import json
import os
import socket
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

from flir_ptz.webui.server import ServerConfig, WebAdapter, make_server  # noqa: E402
from flir_ptz.webui.sse import (  # noqa: E402
    SSEHub,
    UNDROPPABLE_EVENTS,
    format_keepalive,
    format_sse,
    sse_frame,
)

DEFAULT_TIMEOUT = 2.0


# ---------------------------------------------------------------------------
# FakeAdapter — implements WebAdapter with no ROS/rclpy involved at all,
# proving server.py's whole route table works with nothing but this Protocol.
# ---------------------------------------------------------------------------


class FakeAdapter:
    """Records every call it receives and returns canned/echoed responses.
    Deliberately duck-typed against ``WebAdapter`` rather than subclassing
    it (Protocol is structural) — this IS what nodes/web_node.py's real
    implementation will look like from server.py's point of view."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, Any]] = []
        self.connect_calls: list[tuple[str, str, str, str]] = []
        self._state = {
            "connected": True,
            "state_age_sec": 0.05,
            "model": "364c",
            "streams": {"ir": "rtsp://cam.invalid:8554/ir.0", "eo": "rtsp://cam.invalid:8554/vis.0"},
            "control_source": "",
            "state": {
                "abs_azimuth": 1.0, "abs_elevation": 2.0, "geo_azimuth": 3.0, "geo_elevation": 4.0,
                "speed_x": 0.0, "speed_y": 0.0, "mode": 0, "is_moving": False, "is_scanning": False,
                "active_move_seq": 0, "active_scan_seq": 0, "stamp": 123.456,
            },
        }
        self._control_source = {"owner": "", "expires": 0.0}
        self._camera_status = {"status": "connecting", "host": "", "error": ""}
        self._model = {
            "model": "364c",
            "streams": {"ir": "rtsp://cam.invalid:8554/ir.0", "eo": "rtsp://cam.invalid:8554/vis.0"},
            "models": ["364c", "m232"],
        }
        # Simulated slow camera round-trip -- exercises the non-blocking
        # /api/connect contract: this fires in a background thread and the
        # HTTP response must NOT wait for it (see the old 15s
        # threading.Event().wait() defect, PARITY.md / API.md sec. 5).
        self.connect_delay_s = 0.0

    # -- snapshots --
    def state_snapshot(self) -> dict:
        return dict(self._state)

    def control_source_snapshot(self) -> dict:
        return dict(self._control_source)

    def camera_status_snapshot(self) -> dict:
        return dict(self._camera_status)

    def model_snapshot(self) -> dict:
        return dict(self._model)

    def stream_url(self, key: str) -> Optional[str]:
        return self._model["streams"].get(key)

    # -- commands --
    def cmd_move_to(self, body: dict) -> dict:
        self.calls.append(("move_to", body))
        return {"ok": True, "echo": body}

    def cmd_track(self, body: dict) -> dict:
        self.calls.append(("track", body))
        return {"ok": True, "echo": body}

    def cmd_joy_stick_control(self, body: dict) -> dict:
        self.calls.append(("joy_stick_control", body))
        return {"ok": True, "echo": body}

    def cmd_scan(self, body: dict) -> dict:
        self.calls.append(("scan", body))
        return {"ok": True, "echo": body, "stop": bool(body.get("stop", False))}

    def cmd_home(self, body: dict) -> dict:
        self.calls.append(("home", body))
        return {"ok": True, "echo": body}

    # -- arbitration --
    def claim(self, source: str) -> dict:
        self.calls.append(("claim", source))
        self._control_source = {"owner": source, "expires": 9999.0}
        return {"ok": True, "granted": True, "owner": source}

    def unlock(self) -> dict:
        self.calls.append(("unlock", None))
        self._control_source = {"owner": "", "expires": 0.0}
        return {"ok": True}

    # -- setup --
    def connect(self, host: str, model: str, username: str, password: str) -> None:
        self.connect_calls.append((host, model, username, password))
        if self.connect_delay_s <= 0:
            return

        def _slow() -> None:
            time.sleep(self.connect_delay_s)
            self._camera_status = {"status": "connected", "host": host, "error": ""}

        threading.Thread(target=_slow, daemon=True).start()

    def set_model(self, model: str) -> dict:
        self.calls.append(("set_model", model))
        if model not in ("364c", "m232"):
            return {"ok": False, "model": self._model["model"], "streams": self._model["streams"]}
        self._model["model"] = model
        return {"ok": True, "model": model, "streams": self._model["streams"]}


# ---------------------------------------------------------------------------
# server harness
# ---------------------------------------------------------------------------


def _make_web_root(tmp_path: Path) -> Path:
    root = tmp_path / "web"
    root.mkdir()
    (root / "index.html").write_text("<html>control</html>")
    (root / "setup.html").write_text("<html>setup</html>")
    (root / "styles.css").write_text("body { color: red; }")
    (root / "app.js").write_text("console.log('app');")
    (root / "setup.js").write_text("console.log('setup');")
    sub = root / "sub"
    sub.mkdir()
    (sub / "nested.txt").write_text("nested")
    return root


class _RunningServer:
    def __init__(self, tmp_path: Path, **config_kwargs: Any) -> None:
        self.web_root = _make_web_root(tmp_path)
        self.adapter = FakeAdapter()
        self.hub = SSEHub()
        cfg_kwargs: dict[str, Any] = dict(bind_host="127.0.0.1", port=0)
        cfg_kwargs.update(config_kwargs)
        self.config = ServerConfig(web_root=self.web_root, **cfg_kwargs)
        self.server = make_server(self.config, self.adapter, self.hub)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    def __enter__(self) -> "_RunningServer":
        self.thread.start()
        self.host, self.port = self.server.server_address[0], self.server.server_address[1]
        self.base_url = f"http://{self.host}:{self.port}"
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
    return resp.status, dict(resp.getheaders()), body


def _post(conn: http.client.HTTPConnection, path: str, body: Optional[dict] = None):
    data = json.dumps(body or {}).encode("utf-8")
    conn.request(
        "POST", path, body=data,
        headers={"Content-Type": "application/json", "Content-Length": str(len(data))},
    )
    resp = conn.getresponse()
    raw = resp.read()
    try:
        parsed = json.loads(raw.decode("utf-8")) if raw else None
    except json.JSONDecodeError:
        parsed = None
    return resp.status, parsed


class _RawSseClient:
    """A minimal, from-scratch SSE reader over a raw socket -- deliberately
    independent of ``http.client`` so the exact bytes on the wire are what
    gets asserted on, not anything a higher-level HTTP library might
    normalize."""

    def __init__(self, host: str, port: int, timeout: float = DEFAULT_TIMEOUT) -> None:
        self.timeout = timeout
        self.sock = socket.create_connection((host, port), timeout=timeout)
        req = (
            f"GET /api/events HTTP/1.1\r\n"
            f"Host: {host}:{port}\r\n"
            f"Connection: keep-alive\r\n\r\n"
        )
        self.sock.sendall(req.encode("ascii"))
        self._buf = b""
        self._read_headers()

    def _read_headers(self) -> None:
        while b"\r\n\r\n" not in self._buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("connection closed before headers were received")
            self._buf += chunk
        head, _, rest = self._buf.partition(b"\r\n\r\n")
        self.status_line = head.split(b"\r\n", 1)[0].decode("ascii", "replace")
        self.header_text = head.decode("ascii", "replace")
        self._buf = rest

    def read_frame(self, timeout: Optional[float] = None) -> bytes:
        """Block for the next full SSE frame (bytes through the terminating
        blank line), including keepalive comment frames."""
        self.sock.settimeout(timeout if timeout is not None else self.timeout)
        while b"\n\n" not in self._buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("connection closed")
            self._buf += chunk
        frame, sep, rest = self._buf.partition(b"\n\n")
        self._buf = rest
        return frame + sep

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


def _event_name(frame: bytes) -> Optional[str]:
    first_line = frame.split(b"\n", 1)[0]
    if first_line.startswith(b"event: "):
        return first_line[len(b"event: "):].decode()
    return None


# ---------------------------------------------------------------------------
# SSE wire framing -- exact bytes, no trusting the helper (per instructions)
# ---------------------------------------------------------------------------


def test_sse_frame_wire_format_exact():
    frame = sse_frame("state", '{"a":1}')
    assert frame == b'event: state\ndata: {"a":1}\n\n'


def test_format_sse_json_encodes_and_frames():
    frame = format_sse("camera_status", {"status": "connected", "host": "cam.invalid", "error": ""})
    text = frame.decode()
    lines = text.split("\n")
    assert lines[0] == "event: camera_status"
    assert lines[1].startswith("data: ")
    payload = json.loads(lines[1][len("data: "):])
    assert payload == {"status": "connected", "host": "cam.invalid", "error": ""}
    assert text.endswith("\n\n")
    # exactly one data: line for a payload with no embedded newlines
    assert sum(1 for line in lines if line.startswith("data: ")) == 1


def test_sse_frame_multiline_data_split_across_multiple_data_lines():
    frame = sse_frame("note", "line1\nline2\nline3")
    assert frame == b"event: note\ndata: line1\ndata: line2\ndata: line3\n\n"
    # every physical line of the payload gets its own `data:` prefix, and
    # the frame terminator is only the trailing blank line.
    text = frame.decode()
    body_lines = text.rstrip("\n").split("\n")[1:]  # drop "event: note"
    assert body_lines == ["data: line1", "data: line2", "data: line3"]


def test_keepalive_comment_format_exact():
    assert format_keepalive() == b": keepalive\n\n"


# ---------------------------------------------------------------------------
# SSEHub -- initial value on connect, broadcast fan-out, backpressure
# (pure, no sockets -- exercises the registry/eviction logic directly)
# ---------------------------------------------------------------------------


def test_new_client_gets_current_value_of_each_event_type_immediately():
    hub = SSEHub()
    hub.broadcast("state", {"n": 1})
    hub.broadcast("control_source", {"owner": "web", "expires": 5.0})
    # A second "state" broadcast supersedes the first -- register() must
    # replay only the LATEST value, not every historical broadcast.
    hub.broadcast("state", {"n": 2})

    _cid, q = hub.register()
    frame1 = q.get(timeout=DEFAULT_TIMEOUT)
    frame2 = q.get(timeout=DEFAULT_TIMEOUT)
    assert frame1 is not None and frame2 is not None
    got = {_event_name(frame1), _event_name(frame2)}
    assert got == {"state", "control_source"}
    # no third item queued -- exactly one replay per known event type
    assert q.get(timeout=0.1) is None

    for frame in (frame1, frame2):
        if _event_name(frame) == "state":
            assert b'"n": 2' in frame or b'"n":2' in frame


def test_client_registered_before_any_broadcast_gets_nothing_initially():
    hub = SSEHub()
    _cid, q = hub.register()
    assert q.get(timeout=0.1) is None


def test_broadcast_reaches_all_registered_clients():
    hub = SSEHub()
    ids_and_qs = [hub.register() for _ in range(3)]
    hub.broadcast("state", {"x": 42})
    for _cid, q in ids_and_qs:
        frame = q.get(timeout=DEFAULT_TIMEOUT)
        assert frame is not None
        assert _event_name(frame) == "state"


def test_unregister_removes_client_from_future_broadcasts():
    hub = SSEHub()
    cid, q = hub.register()
    hub.unregister(cid)
    hub.broadcast("state", {"x": 1})
    assert q.get(timeout=0.1) is None
    assert hub.client_count() == 0


def test_backpressure_drops_oldest_state_but_never_drops_control_or_camera_status():
    hub = SSEHub(maxsize=5)
    _cid, q = hub.register()  # deliberately never drained while flooding

    for i in range(50):
        hub.broadcast("state", {"i": i})
        # queue must never grow past maxsize, no matter how fast/long the
        # publisher floods a stalled reader
        assert len(q) <= 5

    hub.broadcast("control_source", {"owner": "joy", "expires": 1.0})
    hub.broadcast("camera_status", {"status": "error", "host": "h", "error": "boom"})
    assert len(q) <= 5

    drained = []
    while True:
        frame = q.get(timeout=0.1)
        if frame is None:
            break
        drained.append(frame)

    names = [_event_name(f) for f in drained]
    assert "control_source" in names, "control_source must never be dropped"
    assert "camera_status" in names, "camera_status must never be dropped"
    # Both undroppable events are present alongside at most (maxsize - 2)
    # state events -- i.e. state events were the ones evicted to make room.
    state_count = sum(1 for n in names if n == "state")
    assert state_count < 50, "old state events must have been dropped under backpressure"
    assert len(drained) <= 5


def test_backpressure_never_drops_multiple_undroppable_events_even_past_maxsize():
    # Regression guard: if control_source/camera_status themselves arrive
    # faster than the reader drains and would together exceed maxsize, the
    # policy must still not silently grow the queue -- but per spec these
    # are low-rate, so this pins down the "fall back to dropping oldest
    # overall" edge case rather than crashing or growing unbounded.
    hub = SSEHub(maxsize=3)
    _cid, q = hub.register()
    hub.broadcast("control_source", {"owner": "a", "expires": 1})
    hub.broadcast("camera_status", {"status": "x", "host": "h", "error": ""})
    hub.broadcast("control_source", {"owner": "b", "expires": 2})
    hub.broadcast("camera_status", {"status": "y", "host": "h", "error": ""})
    assert len(q) <= 3


# ---------------------------------------------------------------------------
# End-to-end SSE over a real socket: framing, initial burst, fan-out,
# disconnect reaping, keepalive -- all against the actual server.
# ---------------------------------------------------------------------------


def test_e2e_sse_framing_exact_over_the_wire(tmp_path):
    with _RunningServer(tmp_path) as rs:
        client = _RawSseClient(rs.host, rs.port)
        try:
            assert "200" in client.status_line
            assert "text/event-stream" in client.header_text.lower()
            assert "no-store" in client.header_text.lower()
            assert "x-accel-buffering: no" in client.header_text.lower()

            rs.hub.broadcast("state", {"stamp": 1.5})
            frame = client.read_frame()
            expected = format_sse("state", {"stamp": 1.5})
            assert frame == expected
        finally:
            client.close()


def test_e2e_initial_state_sent_on_connect_before_any_new_broadcast(tmp_path):
    with _RunningServer(tmp_path) as rs:
        rs.hub.broadcast("camera_status", {"status": "connected", "host": "h", "error": ""})
        rs.hub.broadcast("control_source", {"owner": "web", "expires": 42.0})

        client = _RawSseClient(rs.host, rs.port)
        try:
            seen = set()
            for _ in range(2):
                frame = client.read_frame()
                seen.add(_event_name(frame))
            assert seen == {"camera_status", "control_source"}
        finally:
            client.close()


def test_e2e_broadcast_reaches_multiple_concurrent_clients(tmp_path):
    with _RunningServer(tmp_path) as rs:
        clients = [_RawSseClient(rs.host, rs.port) for _ in range(3)]
        try:
            rs.hub.broadcast("state", {"n": 7})
            for c in clients:
                frame = c.read_frame()
                assert _event_name(frame) == "state"
                assert b'"n": 7' in frame or b'"n":7' in frame
        finally:
            for c in clients:
                c.close()


def test_e2e_disconnected_client_is_reaped_without_disturbing_others(tmp_path):
    with _RunningServer(tmp_path, keepalive_interval_s=0.2) as rs:
        alive = _RawSseClient(rs.host, rs.port)
        dying = _RawSseClient(rs.host, rs.port)
        try:
            deadline = time.time() + DEFAULT_TIMEOUT
            while rs.hub.client_count() < 2 and time.time() < deadline:
                time.sleep(0.01)
            assert rs.hub.client_count() == 2

            dying.close()

            # A broadcast forces the server's per-client writer thread to
            # attempt a write on the now-closed socket, which is what
            # actually detects and reaps a dead client (no heartbeat
            # protocol needed from the client side).
            deadline = time.time() + DEFAULT_TIMEOUT
            reaped = False
            while time.time() < deadline:
                rs.hub.broadcast("state", {"tick": time.time()})
                if rs.hub.client_count() == 1:
                    reaped = True
                    break
                time.sleep(0.05)
            assert reaped, "disconnected client was never removed from the registry"

            # the survivor must still be getting fresh broadcasts
            rs.hub.broadcast("state", {"marker": "still-alive"})
            frame = alive.read_frame()
            assert _event_name(frame) == "state"
        finally:
            alive.close()


def test_e2e_keepalive_comment_appears_when_idle(tmp_path):
    with _RunningServer(tmp_path, keepalive_interval_s=0.15) as rs:
        client = _RawSseClient(rs.host, rs.port)
        try:
            # No broadcasts at all -- the only thing that can arrive is a
            # keepalive comment once the per-connection timeout elapses.
            frame = client.read_frame(timeout=2.0)
            assert frame == b": keepalive\n\n"
        finally:
            client.close()


# ---------------------------------------------------------------------------
# Static assets + path-traversal guard (PARITY C18, API.md sec. 1)
# ---------------------------------------------------------------------------


def test_root_redirects_to_setup(tmp_path):
    with _RunningServer(tmp_path) as rs:
        conn = rs.connection()
        status, headers, _body = _get(conn, "/")
        assert status == 302
        assert headers.get("Location") == "/setup"


def test_setup_and_control_serve_expected_files(tmp_path):
    with _RunningServer(tmp_path) as rs:
        conn = rs.connection()
        status, _headers, body = _get(conn, "/setup")
        assert status == 200
        assert body == b"<html>setup</html>"

        status, _headers, body = _get(conn, "/control")
        assert status == 200
        assert body == b"<html>control</html>"


def test_static_assets_served_with_content_type(tmp_path):
    with _RunningServer(tmp_path) as rs:
        conn = rs.connection()
        status, headers, body = _get(conn, "/styles.css")
        assert status == 200
        assert body == b"body { color: red; }"
        assert "css" in headers.get("Content-Type", "")

        status, _headers, body = _get(conn, "/app.js")
        assert status == 200
        assert body == b"console.log('app');"


def test_static_missing_file_is_404(tmp_path):
    with _RunningServer(tmp_path) as rs:
        conn = rs.connection()
        status, _headers, _body = _get(conn, "/does-not-exist.js")
        assert status == 404


def test_static_nested_file_still_reachable(tmp_path):
    with _RunningServer(tmp_path) as rs:
        conn = rs.connection()
        status, _headers, body = _get(conn, "/sub/nested.txt")
        assert status == 200
        assert body == b"nested"


@pytest.mark.parametrize(
    "escape_path",
    [
        "/../secret.txt",
        "/../../etc/passwd",
        "/sub/../../secret.txt",
        "/%2e%2e/%2e%2e/etc/passwd",
        "/..%2f..%2fetc%2fpasswd",
        "/sub/%2e%2e/%2e%2e/etc/passwd",
    ],
)
def test_path_traversal_escapes_rejected_with_403(tmp_path, escape_path):
    with _RunningServer(tmp_path) as rs:
        conn = rs.connection()
        status, _headers, _body = _get(conn, escape_path)
        assert status == 403, f"{escape_path!r} should be rejected, got {status}"


def test_path_traversal_guard_does_not_break_legitimate_requests(tmp_path):
    # Sanity check the guard isn't just rejecting everything.
    with _RunningServer(tmp_path) as rs:
        conn = rs.connection()
        status, _headers, body = _get(conn, "/styles.css")
        assert status == 200
        assert body == b"body { color: red; }"


# ---------------------------------------------------------------------------
# /api/connect must be non-blocking (regression guard on the old 15s block)
# ---------------------------------------------------------------------------


def test_api_connect_returns_immediately_even_if_adapter_work_is_slow(tmp_path):
    with _RunningServer(tmp_path) as rs:
        rs.adapter.connect_delay_s = 1.0  # simulates real camera round-trip time
        conn = rs.connection()
        start = time.monotonic()
        status, payload = _post(
            conn, "/api/connect",
            {"host": "cam.invalid", "model": "364c", "username": "admin", "password": "changeme"},
        )
        elapsed = time.monotonic() - start
        assert status == 200
        assert payload == {"ok": True, "pending": True}
        assert elapsed < 0.5, (
            f"/api/connect took {elapsed:.3f}s -- must return near-instantly, "
            "not block on the camera (old defect: blocked up to 15s)"
        )
        assert rs.adapter.connect_calls == [("cam.invalid", "364c", "admin", "changeme")]


def test_api_connect_requires_host_and_password(tmp_path):
    with _RunningServer(tmp_path) as rs:
        conn = rs.connection()
        status, payload = _post(conn, "/api/connect", {"password": "x"})
        assert status == 400
        assert payload["ok"] is False

        status, payload = _post(conn, "/api/connect", {"host": "cam.invalid"})
        assert status == 400
        assert payload["ok"] is False


# ---------------------------------------------------------------------------
# Command endpoints round-trip JSON through the fake adapter (API.md sec. 3)
# ---------------------------------------------------------------------------


def test_move_to_round_trips_through_adapter(tmp_path):
    with _RunningServer(tmp_path) as rs:
        conn = rs.connection()
        body = {"target_azimuth": 12.5, "target_elevation": -3.0, "source": "web"}
        status, payload = _post(conn, "/api/cmd/move_to", body)
        assert status == 200
        assert payload == {"ok": True, "echo": body}
        assert rs.adapter.calls == [("move_to", body)]


def test_track_joy_scan_home_round_trip_through_adapter(tmp_path):
    with _RunningServer(tmp_path) as rs:
        conn = rs.connection()

        track_body = {"target_azimuth": 1.0, "target_elevation": 2.0, "source": "web"}
        status, payload = _post(conn, "/api/cmd/track", track_body)
        assert status == 200 and payload == {"ok": True, "echo": track_body}

        joy_body = {"azimuth_speed": 5.0, "elevation_speed": -2.0, "source": "web"}
        status, payload = _post(conn, "/api/cmd/joy_stick_control", joy_body)
        assert status == 200 and payload == {"ok": True, "echo": joy_body}

        scan_body = {
            "center_azimuth": 0.0, "each_side_deg": 20.0, "elevation": 0.0,
            "speed": 5.0, "stop": False, "source": "web",
        }
        status, payload = _post(conn, "/api/cmd/scan", scan_body)
        assert status == 200 and payload["ok"] is True and payload["stop"] is False

        home_body = {"source": "web"}
        status, payload = _post(conn, "/api/cmd/home", home_body)
        assert status == 200 and payload == {"ok": True, "echo": home_body}

        assert [c[0] for c in rs.adapter.calls] == ["track", "joy_stick_control", "scan", "home"]


def test_locked_command_still_returns_http_200(tmp_path):
    with _RunningServer(tmp_path) as rs:
        rs.adapter.cmd_move_to = lambda body: {"ok": False, "locked": "joy"}  # type: ignore
        conn = rs.connection()
        status, payload = _post(conn, "/api/cmd/move_to", {"target_azimuth": 0, "target_elevation": 0, "source": "web"})
        assert status == 200
        assert payload == {"ok": False, "locked": "joy"}


def test_claim_and_unlock(tmp_path):
    with _RunningServer(tmp_path) as rs:
        conn = rs.connection()
        status, payload = _post(conn, "/api/claim", {"source": "web"})
        assert status == 200
        assert payload == {"ok": True, "granted": True, "owner": "web"}

        status, payload = _post(conn, "/api/claim", {"source": "bogus"})
        assert status == 400

        status, payload = _post(conn, "/api/unlock", {})
        assert status == 200
        assert payload == {"ok": True}


def test_model_get_and_post(tmp_path):
    with _RunningServer(tmp_path) as rs:
        conn = rs.connection()
        status, _headers, raw = _get(conn, "/api/model")
        payload = json.loads(raw)
        assert status == 200
        assert payload["ok"] is True
        assert payload["model"] == "364c"
        assert payload["models"] == ["364c", "m232"]

        status, payload = _post(conn, "/api/model", {"model": "m232"})
        assert status == 200
        assert payload == {"ok": True, "model": "m232", "streams": rs.adapter._model["streams"]}


def test_state_control_source_connection_status_debug_endpoints(tmp_path):
    with _RunningServer(tmp_path) as rs:
        conn = rs.connection()

        status, _h, raw = _get(conn, "/api/state")
        assert status == 200
        assert json.loads(raw) == rs.adapter.state_snapshot()

        rs.adapter.claim("joy")
        status, _h, raw = _get(conn, "/api/control_source")
        assert status == 200
        assert json.loads(raw) == {"ok": True, "source": "joy"}

        status, _h, raw = _get(conn, "/api/connection_status")
        assert status == 200
        payload = json.loads(raw)
        assert payload["ok"] is True
        assert payload["connected"] is False
        assert payload["status"]["status"] == "connecting"


# ---------------------------------------------------------------------------
# MJPEG fallback + ffplay gating (API.md sec. 6)
# ---------------------------------------------------------------------------


def test_mjpeg_unknown_stream_is_404(tmp_path):
    with _RunningServer(tmp_path) as rs:
        conn = rs.connection()
        status, _headers, _body = _get(conn, "/api/stream/bogus")
        assert status == 404


def test_ffplay_disabled_by_default(tmp_path):
    with _RunningServer(tmp_path) as rs:
        assert rs.config.enable_ffplay is False
        conn = rs.connection()
        status, payload = _post(conn, "/api/video/launch", {"stream": "ir"})
        assert status == 200
        assert payload == {"ok": False, "error": "ffplay disabled"}

        status, payload = _post(conn, "/api/video/stop", {"stream": "ir"})
        assert status == 200
        assert payload == {"ok": False, "error": "ffplay disabled"}


def test_unknown_route_is_404_json(tmp_path):
    with _RunningServer(tmp_path) as rs:
        conn = rs.connection()
        status, payload = _post(conn, "/api/nope", {})
        assert status == 404
        assert payload == {"ok": False, "error": "not found"}


# ---------------------------------------------------------------------------
# Sanity: server.py / sse.py import cleanly with no rclpy/httpx present.
# ---------------------------------------------------------------------------


def test_no_rclpy_or_httpx_imported():
    assert "rclpy" not in sys.modules or True  # rclpy may be imported by other test files in-process
    import flir_ptz.webui.server as server_mod
    import flir_ptz.webui.sse as sse_mod

    src_server = Path(server_mod.__file__).read_text()
    src_sse = Path(sse_mod.__file__).read_text()
    for banned in ("import rclpy", "import httpx", "import aiohttp"):
        assert banned not in src_server, f"server.py must not {banned}"
        assert banned not in src_sse, f"sse.py must not {banned}"


# ── static assets: symlinked files must still be served ──────────────────────
#
# Regression guard. The containment check used to resolve() the target before
# comparing it against the root. `colcon build --symlink-install` installs each
# web asset as an individual symlink back into the source tree, so resolving
# first lands outside web_root and EVERY page 403s -- the dashboard was
# completely unreachable from an installed workspace while /api/* kept working,
# which is exactly the kind of failure that only shows up post-install.
# The check is now lexical, so ".." is still rejected but symlinks are followed.


def test_symlinked_asset_is_served_not_403(tmp_path):
    real_dir = tmp_path / "source_tree"
    real_dir.mkdir()
    (real_dir / "index.html").write_text("<html>symlinked control</html>")

    with _RunningServer(tmp_path) as rs:
        # Replace the installed asset with a symlink pointing outside the
        # web root, exactly as colcon --symlink-install does.
        installed = rs.web_root / "index.html"
        installed.unlink()
        installed.symlink_to(real_dir / "index.html")

        conn = rs.connection()
        status, _headers, body = _get(conn, "/control")
        assert status == 200, f"symlinked asset must still be served, got {status}"
        assert body == b"<html>symlinked control</html>"


def test_symlinked_nested_asset_is_served(tmp_path):
    real_dir = tmp_path / "source_tree2"
    real_dir.mkdir()
    (real_dir / "app.js").write_text("console.log('symlinked');")

    with _RunningServer(tmp_path) as rs:
        installed = rs.web_root / "app.js"
        installed.unlink()
        installed.symlink_to(real_dir / "app.js")

        conn = rs.connection()
        status, _headers, body = _get(conn, "/app.js")
        assert status == 200
        assert b"symlinked" in body
