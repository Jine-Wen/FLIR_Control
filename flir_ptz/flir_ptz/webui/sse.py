#!/usr/bin/env python3
"""Server-Sent Events framing and a thread-safe client registry
(spec ARCHITECTURE.md sec. 6, API.md sec. 2).

This replaces the old dashboard's two polling timers (``pollState`` every
250ms, ``pollCameraConnection`` every 2s -- ``flir_ptz_web.py`` had every
open browser tab hit ``/api/state``/``/api/connection_status`` on its own
schedule) with a single push channel: the server calls :meth:`SSEHub.broadcast`
whenever new data arrives, and one background thread per connected client
(owned by ``webui/server.py``, not by this module) drains that client's
queue onto its socket.

No I/O here at all -- :class:`SSEHub` only touches in-memory queues and a
dict. Socket reads/writes and the per-connection HTTP handling live entirely
in ``webui/server.py``, which is what makes the "clean removal of
disconnected clients" requirement trivial: a broken pipe can only ever
happen in the handler thread's ``wfile.write()``, never inside
:meth:`SSEHub.broadcast`, so a stalled/dead browser tab can never raise out
of the publisher's call stack.

Wire format (https://html.spec.whatwg.org/multipage/server-sent-events.html)::

    event: <name>\\n
    data: <json-line-1>\\n
    data: <json-line-2>\\n
    \\n

``json.dumps`` never emits a literal newline inside a JSON string (control
characters are escaped as ``\\n``), so in practice a single event is always
one ``data:`` line -- but :func:`sse_frame` splits on ``\\n`` unconditionally
so a payload that *did* span multiple lines (e.g. someone switching to
``indent=`` output someday) still frames correctly instead of corrupting the
stream with an embedded blank line.

Backpressure policy (why it matters: this is meant to run unattended for a
long time, and a browser tab that was closed without the TCP connection
noticing yet -- sleeping laptop, flaky wifi -- must not turn into a slowly
growing in-memory leak behind a dead socket):

* Every client gets a bounded queue (``maxsize`` frames, default
  :data:`DEFAULT_QUEUE_MAXSIZE`). The publisher (``broadcast``) never blocks
  on a slow reader -- ``queue`` operations here are all non-blocking puts.
* ``state`` events are high-rate (~10Hz) and each one supersedes the last,
  so they are safe to drop: if a client's queue is full, a new ``state``
  frame is simply not enqueued (the *newest* published event still wins,
  because the client's queue only ever needs draining -- new callers keep
  calling ``broadcast`` with fresher data).
* ``control_source`` and ``camera_status`` are low-rate, latched-style facts
  (control ownership, camera connectivity) where losing an update leaves the
  UI wrong indefinitely rather than just one frame stale. Those are never
  dropped: on overflow, the oldest *droppable* queued frame is evicted to
  make room, so an undroppable frame always fits without the queue growing
  past ``maxsize``.
"""

from __future__ import annotations

import json
import threading
from collections import deque
from typing import Optional

# Per API.md sec. 2 / ARCHITECTURE.md sec. 6: every 15s so idle connections
# survive intermediaries (nginx, corporate proxies) that kill silent
# keep-alives. Configurable per-server for tests -- see
# ``webui/server.ServerConfig.keepalive_interval_s``.
KEEPALIVE_INTERVAL_S = 15.0

# Event names that must never be dropped under backpressure -- see the
# "Backpressure policy" note above.
UNDROPPABLE_EVENTS = frozenset({"control_source", "camera_status"})

DEFAULT_QUEUE_MAXSIZE = 64


def sse_frame(event: str, data: str) -> bytes:
    """Frame one already-serialized ``data`` string as a raw SSE event.

    Exposed separately from :func:`format_sse` so the exact wire framing
    (including the multi-line ``data:`` split) can be tested without going
    through JSON encoding at all.
    """
    lines = data.split("\n")
    out = [f"event: {event}\n"]
    out.extend(f"data: {line}\n" for line in lines)
    out.append("\n")
    return "".join(out).encode("utf-8")


def format_sse(event: str, payload: object) -> bytes:
    """JSON-encode ``payload`` and frame it as an SSE ``event`` message."""
    return sse_frame(event, json.dumps(payload))


def format_keepalive() -> bytes:
    """A bare SSE comment line. Any line starting with ``:`` is a comment
    per the SSE spec -- browsers ignore it, but it resets intermediary idle
    timers and lets a handler thread detect a dead socket (via the
    ``wfile.write`` it triggers) even during a quiet period."""
    return b": keepalive\n\n"


class _ClientQueue:
    """Bounded, thread-safe frame queue for one SSE client.

    Not ``queue.Queue`` because the backpressure policy needs to evict a
    specific *kind* of queued item (an old droppable frame) rather than
    just the head -- see the module docstring.
    """

    def __init__(self, maxsize: int) -> None:
        self._maxsize = maxsize
        self._items: deque[tuple[Optional[str], bytes]] = deque()
        self._cv = threading.Condition()

    def put(self, event: Optional[str], frame: bytes) -> None:
        """Enqueue ``frame`` (tagged with its event name, or ``None`` for a
        keepalive). Never blocks."""
        with self._cv:
            if len(self._items) >= self._maxsize:
                if event in UNDROPPABLE_EVENTS:
                    if not self._evict_one_droppable():
                        # Every queued item is itself undroppable (should
                        # not happen in practice -- control_source/
                        # camera_status are low-rate -- but fall back to
                        # dropping the oldest item rather than growing
                        # unbounded).
                        self._items.popleft()
                else:
                    # Droppable event and the queue is full: drop this NEW
                    # frame. The publisher never blocks and the queue never
                    # grows past maxsize.
                    return
            self._items.append((event, frame))
            self._cv.notify()

    def _evict_one_droppable(self) -> bool:
        for i, (ev, _frame) in enumerate(self._items):
            if ev not in UNDROPPABLE_EVENTS:
                del self._items[i]
                return True
        return False

    def get(self, timeout: float) -> Optional[bytes]:
        """Wait up to ``timeout`` seconds for the next frame. Returns
        ``None`` on timeout (the caller should treat that as "send a
        keepalive")."""
        with self._cv:
            if not self._items:
                self._cv.wait(timeout=timeout)
            if not self._items:
                return None
            _event, frame = self._items.popleft()
            return frame

    def __len__(self) -> int:
        with self._cv:
            return len(self._items)


class SSEHub:
    """Thread-safe SSE client registry and fan-out broadcaster.

    Usage (``webui/server.py`` owns one instance; ``nodes/web_node.py`` calls
    ``broadcast`` whenever a ROS subscription callback gets fresh data)::

        hub = SSEHub()
        ...
        hub.broadcast("state", {...})            # on every PtzState message
        hub.broadcast("control_source", {...})   # on every ControlSource message
        hub.broadcast("camera_status", {...})    # on every camera_status message

        # in the /api/events GET handler, one thread per connection:
        client_id, client_q = hub.register()
        try:
            while True:
                frame = client_q.get(timeout=15.0)
                self.wfile.write(frame if frame is not None else format_keepalive())
        finally:
            hub.unregister(client_id)
    """

    def __init__(self, maxsize: int = DEFAULT_QUEUE_MAXSIZE) -> None:
        self._maxsize = maxsize
        self._clients_lock = threading.Lock()
        self._clients: dict[int, _ClientQueue] = {}
        self._next_id = 0
        self._last_lock = threading.Lock()
        self._last: dict[str, object] = {}

    def register(self) -> tuple[int, _ClientQueue]:
        """Register a new client and return ``(client_id, queue)``. The
        current value of every event type broadcast so far is enqueued
        immediately, so a newly-opened tab renders instantly instead of
        waiting for the next change (API.md sec. 2)."""
        q = _ClientQueue(self._maxsize)
        with self._clients_lock:
            client_id = self._next_id
            self._next_id += 1
            self._clients[client_id] = q
        with self._last_lock:
            snapshot = list(self._last.items())
        for event, payload in snapshot:
            q.put(event, format_sse(event, payload))
        return client_id, q

    def unregister(self, client_id: int) -> None:
        """Remove a client. Safe to call more than once / with an unknown
        id (no-op) -- the handler's ``finally`` block always calls this
        regardless of why the connection ended."""
        with self._clients_lock:
            self._clients.pop(client_id, None)

    def broadcast(self, event: str, payload: object) -> None:
        """Fan ``payload`` out to every connected client and remember it as
        the "current value" of ``event`` for future ``register()`` calls.

        Never blocks and never raises due to a client's state (socket
        errors are impossible here -- this method touches only in-memory
        queues, never a socket)."""
        with self._last_lock:
            self._last[event] = payload
        frame = format_sse(event, payload)
        with self._clients_lock:
            queues = list(self._clients.values())
        for q in queues:
            q.put(event, frame)

    def client_count(self) -> int:
        with self._clients_lock:
            return len(self._clients)

    def last_value(self, event: str) -> Optional[object]:
        """Test/debug helper: the last payload broadcast for ``event``, or
        ``None`` if it has never been broadcast."""
        with self._last_lock:
            return self._last.get(event)
