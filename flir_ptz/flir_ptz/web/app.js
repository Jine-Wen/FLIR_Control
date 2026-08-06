const $ = (id) => document.getElementById(id);

const MAX_SPEED = 40.0;

// mediamtx WebRTC port (WHEP endpoint: http://<host>:8889/<stream>/whep)
const WEBRTC_PORT = 8889;

const ui = {
  connDot:         $("connDot"),
  connectionBadge: $("connectionBadge"),
  movingBadge:     $("movingBadge"),
  scanBadge:       $("scanBadge"),
  modeBadge:       $("modeBadge"),
  ctrlBadge:       $("ctrlBadge"),
  joy:             $("joystick"),
  stick:           $("stick"),
  joyVector:       $("joyVector"),
  joyGain:         $("joyGain"),
  joyGainValue:    $("joyGainValue"),
  absAz:  $("absAz"),  absEl:  $("absEl"),
  geoAz:  $("geoAz"),  geoEl:  $("geoEl"),
  speedX: $("speedX"), speedY: $("speedY"),
  barX:   $("barX"),   barY:   $("barY"),
  irUrl:  $("irUrl"),  eoUrl:  $("eoUrl"),
  irState:$("irState"),eoState:$("eoState"),
  irVideo:$("irVideo"),eoVideo:$("eoVideo"),
  irReticle:$("irReticle"), eoReticle:$("eoReticle"),
  irMjpeg:$("irMjpeg"),eoMjpeg:$("eoMjpeg"),
  irTransport:$("irTransport"), eoTransport:$("eoTransport"),
  // Retry buttons use data-retry attributes rather than static ids.
  irRetry: document.querySelector('[data-retry="ir"]'),
  eoRetry: document.querySelector('[data-retry="eo"]'),
  eoPaneWrap: $("eoPaneWrap"),
  videoGrid:  $("videoGrid"),
  camDisconnectWarn:     $("camDisconnectWarn"),
  camDisconnectWarnText: $("camDisconnectWarnText"),
  eoZoomIn:   $("eoZoomIn"),
  eoZoomOut:  $("eoZoomOut"),
  eoZoomPctg: $("eoZoomPctg"),
  irZoomIn:   $("irZoomIn"),
  irZoomOut:  $("irZoomOut"),
  irZoomPctg: $("irZoomPctg"),
};

// Buttons that must be disabled when locked
const CTRL_BUTTONS = ["moveButton", "centerButton", "homeButton", "scanStart", "scanStop", "eoZoomIn", "eoZoomOut", "irZoomIn", "irZoomOut"];
const CTRL_PANELS  = ["movePanel", "scanPanel"];

let joy = { active: false, x: 0, y: 0 };
let joyUnlocked = false;
let _lastCtrlSrc = null;

/* ── Circle-unlock gesture ──────────────────────────────────────────
   Twin of control/gestures.py::CircleDetector — the same gesture must unlock
   the console whether drawn with a mouse or a physical stick, so keep the two
   in step.

   Accumulates the unwrapped angle swept, incrementally. The previous version
   summed deltas across a sliding 120-sample window and refused to report
   anything before 20 samples, which left only a narrow band of drawing speeds
   that worked at all: draw it quickly and it was rejected for too few samples;
   draw it slowly and the earliest deltas fell out of the window faster than
   new ones arrived, so the total could never reach 2*pi however many turns
   were made. A careful first attempt lands squarely in that second case.
*/
const CIRCLE_MIN_R   = 0.4;   // below this the angle is too noisy to measure
const CIRCLE_RESET_R = 0.18;  // below this the stick has genuinely returned to rest

let _circlePrevAngle = null;
let _circleTotal     = 0;

function circleReset() {
  _circlePrevAngle = null;
  _circleTotal     = 0;
}

/** Fraction of a full turn swept so far, 0..1 — drives the on-screen hint. */
function circleProgress() {
  return Math.min(1, Math.abs(_circleTotal) / (2 * Math.PI));
}

function detectCircle(x, y) {
  const r = Math.hypot(x, y);

  if (r < CIRCLE_RESET_R) { circleReset(); return false; }
  // Between reset and min: hold what has been swept but stop measuring, so
  // brushing past the middle mid-gesture costs nothing.
  if (r < CIRCLE_MIN_R) { _circlePrevAngle = null; return false; }

  const angle = Math.atan2(y, x);
  if (_circlePrevAngle !== null) {
    let d = angle - _circlePrevAngle;
    if (d >  Math.PI) d -= 2 * Math.PI;
    else if (d < -Math.PI) d += 2 * Math.PI;
    _circleTotal += d;
  }
  _circlePrevAngle = angle;

  return Math.abs(_circleTotal) >= 2 * Math.PI;
}

const JOY_HINT_LOCKED = "Draw one full circle to unlock";

/** Show how far round the unlock gesture has got, so a partial sweep is
 *  visible feedback rather than silence. */
function updateUnlockHint() {
  const el = document.querySelector(".joy-hint");
  if (!el) return;
  const pct = Math.round(circleProgress() * 100);
  el.textContent = pct > 0 ? `Keep going… ${pct}%` : JOY_HINT_LOCKED;
}

function setJoyLock(locked, reason = "self") {
  joyUnlocked = !locked;
  const joyEl = ui.joy;
  joyEl.classList.remove("lock-self", "lock-remote", "lock-open");

  if (locked) {
    circleReset();
    const hint = document.querySelector(".joy-hint");
    if (hint) hint.textContent = JOY_HINT_LOCKED;
    joyEl.classList.add(reason === "remote" ? "lock-remote" : "lock-self");
    CTRL_BUTTONS.forEach(id => { const el = $(id); if (el) el.disabled = true; });
    CTRL_PANELS.forEach(id  => { const el = $(id); if (el) el.classList.add("ctrl-locked"); });
    $("joyPanel").classList.add("ctrl-locked");
  } else {
    joyEl.classList.add("lock-open");
    const hint = document.querySelector(".joy-hint");
    if (hint) hint.textContent = "Unlocked — drag to steer";
    CTRL_BUTTONS.forEach(id => { const el = $(id); if (el) el.disabled = false; });
    CTRL_PANELS.forEach(id  => { const el = $(id); if (el) el.classList.remove("ctrl-locked"); });
    $("joyPanel").classList.remove("ctrl-locked");
    post("/api/claim", { source: "web" });
  }
}

/* ── Differential DOM helpers ──────────────────────────────────────
   The SSE `state` event arrives at ~10Hz. Only touch the DOM when a
   field's rendered value actually changed.
*/
const _cache = new Map();

function diffText(el, key, value) {
  if (!el) return;
  if (_cache.get(key) !== value) {
    _cache.set(key, value);
    el.textContent = value;
  }
}

function diffClass(el, key, className) {
  if (!el) return;
  if (_cache.get(key) !== className) {
    _cache.set(key, className);
    el.className = className;
  }
}

function diffStyle(el, key, prop, value) {
  if (!el) return;
  const cacheKey = `${key}:${prop}`;
  if (_cache.get(cacheKey) !== value) {
    _cache.set(cacheKey, value);
    el.style[prop] = value;
  }
}

/* ── Formatting ─────────────────────────────────────────────── */
function fmt(value, dec = 2) {
  return Number.isFinite(value) ? value.toFixed(dec) : "--";
}

// Zoom percentage: rounded to the nearest whole percent rather than one
// decimal place. The camera's own reading has enough noise that a
// stationary lens can report e.g. 31.24 on one poll and 31.26 on the next
// -- fine at whole-percent resolution, but toFixed(1) turns that same noise
// into a visible flicker between "31.2" and "31.3". IR is quantised to
// 6.25% steps anyway, so whole-percent is no loss of real precision there
// either.
function fmtPct(value) {
  return Number.isFinite(value) ? String(Math.round(value)) : "--";
}

/* ── API ────────────────────────────────────────────────────── */
async function post(url, payload) {
  const res = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ ...payload, source: "web" }),
  });
  return res.json();
}

/* ── WebRTC / WHEP + MJPEG fallback ────────────────────────────
   WebRTC is the primary transport. If the WHEP request fails, or the
   peer connection ever moves to failed/disconnected, we automatically
   fall back to the MJPEG <img> stream and expose a manual "retry
   WebRTC" affordance.
*/
const peerConnections = {};
// Per-stream active transport: 'idle' | 'webrtc' | 'mjpeg'
const streamTransport = { ir: "idle", eo: "idle" };

// How long a transient "disconnected" may last before we give up on WebRTC.
const DISCONNECT_GRACE_MS = 6000;
// How long to wait for the first media track before falling back.
const MEDIA_TIMEOUT_MS = 10000;

const disconnectTimers = { ir: null, eo: null };
const mediaWatchdogs   = { ir: null, eo: null };

function clearDisconnectGrace(stream) {
  if (disconnectTimers[stream]) {
    clearTimeout(disconnectTimers[stream]);
    disconnectTimers[stream] = null;
  }
}

function clearMediaWatchdog(stream) {
  if (mediaWatchdogs[stream]) {
    clearTimeout(mediaWatchdogs[stream]);
    mediaWatchdogs[stream] = null;
  }
}

function setTransportLabel(stream, label, colorVar) {
  diffText(ui[`${stream}Transport`], `${stream}Transport`, label);
  diffStyle(ui[`${stream}Transport`], `${stream}TransportColor`, "color", colorVar);
}

function toggleRetryButton(stream, show) {
  const btn = ui[`${stream}Retry`];
  if (btn) btn.hidden = !show;
}

function setStreamState(stream, text, colorVar) {
  const el = ui[`${stream}State`];
  if (el) { el.textContent = text; el.style.color = colorVar; }
}

async function startWebRTC(stream) {
  stopMjpeg(stream);
  stopWebRTC(stream, { silent: true });

  const videoEl = ui[`${stream}Video`];
  const reticle = ui[`${stream}Reticle`];

  streamTransport[stream] = "webrtc";
  setStreamState(stream, "⟳ Connecting…", "var(--amber)");
  setTransportLabel(stream, "WebRTC", "var(--muted)");
  toggleRetryButton(stream, false);

  const pc = new RTCPeerConnection({
    iceServers: [],          // local network — no STUN/TURN needed
    bundlePolicy: "max-bundle",
  });
  peerConnections[stream] = pc;

  // We only want to receive video
  pc.addTransceiver("video", { direction: "recvonly" });

  pc.ontrack = (evt) => {
    videoEl.srcObject = evt.streams[0];
    videoEl.style.display = "block";
    if (reticle) reticle.style.display = "none";
    setStreamState(stream, "● Streaming", "var(--green)");
    setTransportLabel(stream, "WebRTC", "var(--green)");
    clearDisconnectGrace(stream);
  };

  // Falling back to MJPEG tears down a working peer connection, so only do it
  // when WebRTC is genuinely dead.
  //
  // "disconnected" is NOT dead. Per the WebRTC spec it is a transient state
  // that routinely recovers on its own -- an ICE consent-freshness hiccup or a
  // moment of packet loss is enough to enter it. Treating it as failure kills a
  // perfectly good stream mid-playback and swaps in MJPEG, which then needs
  // ffmpeg on the server; without ffmpeg the pane just goes black. Only
  // "failed" is terminal, so give "disconnected" a grace period to recover.
  pc.onconnectionstatechange = () => {
    if (peerConnections[stream] !== pc) return;
    const state = pc.connectionState;

    if (state === "connected") {
      clearDisconnectGrace(stream);
      setStreamState(stream, "● Streaming", "var(--green)");
      return;
    }

    if (state === "failed") {
      clearDisconnectGrace(stream);
      if (streamTransport[stream] === "webrtc") startMjpeg(stream);
      return;
    }

    if (state === "disconnected" && streamTransport[stream] === "webrtc") {
      if (disconnectTimers[stream]) return;   // already counting down
      setStreamState(stream, "⟳ Reconnecting…", "var(--amber)");
      disconnectTimers[stream] = setTimeout(() => {
        disconnectTimers[stream] = null;
        if (peerConnections[stream] !== pc) return;
        if (pc.connectionState === "disconnected" && streamTransport[stream] === "webrtc") {
          startMjpeg(stream);
        }
      }, DISCONNECT_GRACE_MS);
    }
  };

  // If signalling succeeds but no media ever arrives, nothing above fires and
  // the pane would sit black forever. Fall back once, after a grace period.
  clearMediaWatchdog(stream);
  mediaWatchdogs[stream] = setTimeout(() => {
    mediaWatchdogs[stream] = null;
    if (peerConnections[stream] !== pc) return;
    if (streamTransport[stream] === "webrtc" && !videoEl.srcObject) {
      setStreamState(stream, "✕ no media from WebRTC", "var(--red)");
      startMjpeg(stream);
    }
  }, MEDIA_TIMEOUT_MS);

  // Create offer
  const offer = await pc.createOffer();
  await pc.setLocalDescription(offer);

  // Wait for ICE gathering (or 2 s timeout)
  await new Promise((resolve) => {
    if (pc.iceGatheringState === "complete") { resolve(); return; }
    const timeout = setTimeout(resolve, 2000);
    pc.addEventListener("icegatheringstatechange", () => {
      if (pc.iceGatheringState === "complete") { clearTimeout(timeout); resolve(); }
    });
  });

  // Bail out if the stream was switched/stopped while we were gathering ICE.
  if (peerConnections[stream] !== pc) return;

  const whepUrl = `http://${location.hostname}:${WEBRTC_PORT}/${stream}/whep`;
  let resp;
  try {
    resp = await fetch(whepUrl, {
      method:  "POST",
      headers: { "Content-Type": "application/sdp" },
      body:    pc.localDescription.sdp,
    });
  } catch (err) {
    console.error("WHEP fetch failed:", err);
    if (peerConnections[stream] === pc) startMjpeg(stream);
    return;
  }

  if (peerConnections[stream] !== pc) return;

  if (!resp.ok) {
    console.error("WHEP error:", await resp.text());
    startMjpeg(stream);
    return;
  }

  const answerSdp = await resp.text();
  if (peerConnections[stream] !== pc) return;
  await pc.setRemoteDescription({ type: "answer", sdp: answerSdp });
}

function stopWebRTC(stream, opts = {}) {
  const { silent = false } = opts;
  const pc = peerConnections[stream];
  if (pc) {
    pc.onconnectionstatechange = null;
    pc.close();
    delete peerConnections[stream];
  }
  const videoEl = ui[`${stream}Video`];
  if (videoEl) {
    videoEl.srcObject = null;
    videoEl.style.display = "none";
  }
  if (!silent) {
    const reticle = ui[`${stream}Reticle`];
    if (reticle) reticle.style.display = "";
    setStreamState(stream, "Not streaming", "var(--muted)");
  }
}

function startMjpeg(stream) {
  stopWebRTC(stream, { silent: true });

  const imgEl   = ui[`${stream}Mjpeg`];
  const reticle = ui[`${stream}Reticle`];
  if (!imgEl) return;

  streamTransport[stream] = "mjpeg";
  imgEl.onerror = () => {
    if (streamTransport[stream] !== "mjpeg") return;
    setStreamState(stream, "✕ MJPEG stream failed", "var(--red)");
  };
  imgEl.onload = () => {
    if (streamTransport[stream] !== "mjpeg") return;
    setStreamState(stream, "● Streaming (MJPEG)", "var(--amber)");
  };
  imgEl.src = `/api/stream/${stream}?t=${Date.now()}`;
  imgEl.style.display = "block";
  if (reticle) reticle.style.display = "none";
  setStreamState(stream, "⟳ Falling back to MJPEG…", "var(--amber)");
  setTransportLabel(stream, "MJPEG (fallback)", "var(--amber)");
  toggleRetryButton(stream, true);
}

function stopMjpeg(stream) {
  const imgEl = ui[`${stream}Mjpeg`];
  if (imgEl) {
    imgEl.onerror = null;
    imgEl.onload = null;
    imgEl.removeAttribute("src");
    imgEl.style.display = "none";
  }
}

// Full manual stop — user clicked "■ Stop". Resets both transports.
function stopStream(stream) {
  stopWebRTC(stream);
  stopMjpeg(stream);
  streamTransport[stream] = "idle";
  setTransportLabel(stream, "—", "var(--muted)");
  toggleRetryButton(stream, false);
}

/* ── Render (differential) ─────────────────────────────────────── */
let _lastModel = null;

function applyModel(model, streams) {
  if (model && model !== _lastModel) {
    _lastModel = model;
    const isM232 = model === "m232";
    if (isM232) {
      ui.eoPaneWrap.setAttribute("hidden", "");
      ui.videoGrid.classList.add("single-pane");
      stopStream("eo");
    } else {
      ui.eoPaneWrap.removeAttribute("hidden");
      ui.videoGrid.classList.remove("single-pane");
    }
  }
  if (streams) {
    diffText(ui.irUrl, "irUrl", streams.ir ? `→ :${WEBRTC_PORT}/ir` : "—");
    diffText(ui.eoUrl, "eoUrl", streams.eo ? `→ :${WEBRTC_PORT}/eo` : (_lastModel === "m232" ? "N/A" : "—"));
  }
}

function applyControlSource(ownerRaw) {
  const src = ownerRaw || "none";
  const ctrlLabel = { web: "Web", joy: "Remote", none: "—" };
  diffText(ui.ctrlBadge, "ctrlBadge", ctrlLabel[src] || "—");
  diffStyle(ui.ctrlBadge, "ctrlBadgeColor", "color",
    src === "web" ? "var(--cyan)" : src === "joy" ? "var(--amber)" : "var(--muted)");

  // Remote (joy) just claimed control — lock web regardless of current state.
  if (src === "joy" && _lastCtrlSrc !== "joy") {
    setJoyLock(true, "remote");
  }
  _lastCtrlSrc = src;
}

function renderState(payload) {
  const connected = !!payload.connected;
  diffClass(ui.connDot, "connDot", "status-dot" + (connected ? " online" : ""));
  diffText(ui.connectionBadge, "connectionBadge", connected ? "Online" : "No State");
  diffClass(ui.connectionBadge, "connectionBadgeClass", "status-label " + (connected ? "online" : "offline"));

  const s = payload.state || {};

  diffClass(ui.movingBadge, "movingBadge", "state-pill" + (s.is_moving   ? " active" : ""));
  diffClass(ui.scanBadge,   "scanBadge",   "state-pill" + (s.is_scanning ? " active" : ""));

  diffText(ui.modeBadge, "modeBadge", s.mode != null ? String(s.mode) : "--");

  diffText(ui.geoAz,  "geoAz",  fmt(s.geo_azimuth));
  diffText(ui.geoEl,  "geoEl",  fmt(s.geo_elevation));
  diffText(ui.absAz,  "absAz",  fmt(s.abs_azimuth));
  diffText(ui.absEl,  "absEl",  fmt(s.abs_elevation));
  diffText(ui.speedX, "speedX", fmt(s.speed_x, 1));
  diffText(ui.speedY, "speedY", fmt(s.speed_y, 1));

  const px = (Math.min(100, Math.abs(s.speed_x || 0) / MAX_SPEED * 100)).toFixed(1) + "%";
  const py = (Math.min(100, Math.abs(s.speed_y || 0) / MAX_SPEED * 100)).toFixed(1) + "%";
  diffStyle(ui.barX, "barX", "width", px);
  diffStyle(ui.barY, "barY", "width", py);

  diffText(ui.eoZoomPctg, "eoZoomPctg", fmtPct(s.zoom_pctg));
  diffText(ui.irZoomPctg, "irZoomPctg", fmtPct(s.ir_zoom_pctg));

  applyModel(payload.model, payload.streams);
  applyControlSource(payload.control_source);
}

function applyCameraStatus(payload) {
  const status = payload && payload.status;
  if (status === "connected") {
    diffClass(ui.camDisconnectWarn, "camWarn", "cam-disconnect-warn");
  } else {
    const msg = status === "error"
      ? `⚠ Camera disconnected${payload.error ? ` (${payload.error})` : ""}.`
      : "⚠ Camera connecting…";
    diffText(ui.camDisconnectWarnText, "camWarnText", msg);
    diffClass(ui.camDisconnectWarn, "camWarn", "cam-disconnect-warn show");
  }
}

/* ── SSE event stream (replaces all polling) ───────────────────── */
function connectEvents() {
  const es = new EventSource("/api/events");

  es.addEventListener("state", (evt) => {
    let payload;
    try { payload = JSON.parse(evt.data); } catch { return; }
    renderState(payload);
  });

  es.addEventListener("control_source", (evt) => {
    let payload;
    try { payload = JSON.parse(evt.data); } catch { return; }
    applyControlSource(payload.owner);
  });

  es.addEventListener("camera_status", (evt) => {
    let payload;
    try { payload = JSON.parse(evt.data); } catch { return; }
    applyCameraStatus(payload);
  });

  // EventSource auto-reconnects on its own — do NOT close() here, just
  // reflect the disconnection in the UI. The next successful `state`
  // event will overwrite this once the stream is back.
  es.onerror = () => {
    diffText(ui.connectionBadge, "connectionBadge", "Bridge Offline");
    diffClass(ui.connectionBadge, "connectionBadgeClass", "status-label offline");
    diffClass(ui.connDot, "connDot", "status-dot");
  };

  return es;
}

/* ── Joystick ───────────────────────────────────────────────── */
function joystickPoint(evt) {
  const rect   = ui.joy.getBoundingClientRect();
  const cx     = rect.left + rect.width / 2;
  const cy     = rect.top  + rect.height / 2;
  const dx     = evt.clientX - cx;
  const dy     = evt.clientY - cy;
  const radius = rect.width * 0.38;
  const mag    = Math.min(radius, Math.hypot(dx, dy));
  const angle  = Math.atan2(dy, dx);
  return { x: Math.cos(angle) * mag / radius, y: Math.sin(angle) * mag / radius };
}

function setStick(x, y) {
  const rect   = ui.joy.getBoundingClientRect();
  const radius = rect.width * 0.38;
  ui.stick.style.transform = `translate(${x * radius}px, ${y * radius}px)`;
  // Label the axes explicitly: an unlabelled "0.00 / 0.00" gives no clue
  // which axis is which when one of them misbehaves.
  const f = (v) => (v >= 0 ? "+" : "") + v.toFixed(2);
  ui.joyVector.textContent = `AZ ${f(x)}  EL ${f(-y)}`;
}

function resetJoystick() {
  joy = { active: false, x: 0, y: 0 };
  setStick(0, 0);
  // Only send stop command if web is unlocked (avoid claiming control on pointer release).
  // Zero-speed joy_stick_control is always allowed through arbitration (API.md §3).
  if (joyUnlocked) {
    post("/api/cmd/joy_stick_control", { azimuth_speed: 0, elevation_speed: 0 });
  }
}

function wireJoystick() {
  ui.joy.addEventListener("pointerdown", evt => {
    ui.joy.setPointerCapture(evt.pointerId);
    joy.active = true;
    const p = joystickPoint(evt);
    joy.x = p.x; joy.y = p.y;
    setStick(joy.x, joy.y);
  });
  ui.joy.addEventListener("pointermove", evt => {
    if (!joy.active) return;
    const p = joystickPoint(evt);
    joy.x = p.x; joy.y = p.y;
    setStick(joy.x, joy.y);
    // Circle detection for unlock
    if (!joyUnlocked) {
      if (detectCircle(joy.x, joy.y)) {
        circleReset();
        setJoyLock(false);
      } else {
        updateUnlockHint();
      }
    }
  });
  ui.joy.addEventListener("pointerup",     resetJoystick);
  ui.joy.addEventListener("pointercancel", resetJoystick);
}

async function publishJoyStickControl() {
  if (!joy.active) return;
  if (!joyUnlocked) return;   // blocked until circle unlock
  const maxSpeed = Number(ui.joyGain.value);
  const azSpeed = joy.x * maxSpeed;
  const elSpeed = -joy.y * maxSpeed;
  await post("/api/cmd/joy_stick_control", {
    azimuth_speed: azSpeed,
    elevation_speed: elSpeed,
  });
}

/* ── Controls ───────────────────────────────────────────────── */
function wireControls() {
  ui.joyGain.addEventListener("input", () => {
    ui.joyGainValue.textContent = `${ui.joyGain.value} °/s`;
  });

  $("moveButton").addEventListener("click", () => post("/api/cmd/move_to", {
    target_azimuth:   Number($("moveAz").value),
    target_elevation: Number($("moveEl").value),
  }));

  $("homeButton").addEventListener("click", () => {
    if (!window.confirm("Move camera to AZ 0.0 / EL −90.0?")) return;
    post("/api/cmd/home", {});
  });

  $("centerButton").addEventListener("click", () => {
    post("/api/cmd/move_to", { target_azimuth: 0.0, target_elevation: 0.0 });
  });

  $("scanStart").addEventListener("click", () => post("/api/cmd/scan", {
    center_azimuth: Number($("scanCenter").value),
    each_side_deg:  Number($("scanSide").value),
    elevation:      Number($("scanEl").value),
    speed:          Number($("scanSpeed").value),
    stop: false,
  }));

  $("scanStop").addEventListener("click", () => post("/api/cmd/scan", { stop: true }));

  document.querySelectorAll("[data-launch]").forEach(btn => {
    btn.addEventListener("click", () => startWebRTC(btn.dataset.launch));
  });

  document.querySelectorAll("[data-retry]").forEach(btn => {
    btn.addEventListener("click", () => startWebRTC(btn.dataset.retry));
  });

  document.querySelectorAll("[data-stop]").forEach(btn => {
    btn.addEventListener("click", () => stopStream(btn.dataset.stop));
  });
}

/* ── EO/IR zoom: press-and-hold ────────────────────────────────────────
   Both lenses are CONTINUOUS -- one "in"/"out" keeps it moving until a
   "stop" arrives (control/controller.py runs a matching dead-man timer
   server-side, independently per device, as a last-resort safety net, but
   the browser must still behave: resend the held direction periodically so
   that timer never has a reason to fire, and send "stop" the instant the
   hold ends for any reason — pointerup, pointercancel, pointerleave, the
   tab going hidden, or the page being torn down entirely.

   EO and IR hold state is tracked independently (keyed by device) so
   holding one pane's button can never be confused with, or cancel, a hold
   on the other pane's button. Every command always names its device
   explicitly -- there is no "unspecified device" case here.
*/
const ZOOM_RESEND_MS = 400;
const zoomHold = {
  eo: { timer: null, direction: null },
  ir: { timer: null, direction: null },
};

function sendZoom(device, direction) {
  post("/api/cmd/zoom", { direction, device });
}

function stopZoomHold(device) {
  const state = zoomHold[device];
  if (!state) return;
  if (state.timer) { clearInterval(state.timer); state.timer = null; }
  if (state.direction) {
    state.direction = null;
    sendZoom(device, "stop");
  }
}

function stopAllZoomHolds() {
  stopZoomHold("eo");
  stopZoomHold("ir");
}

function startZoomHold(device, direction) {
  stopZoomHold(device);
  const state = zoomHold[device];
  state.direction = direction;
  sendZoom(device, direction);
  // Keep renewing while held so the server-side dead-man timer never fires
  // for a genuinely-held button.
  state.timer = setInterval(() => sendZoom(device, direction), ZOOM_RESEND_MS);
}

function wireZoomControls() {
  const buttons = [
    [ui.eoZoomIn, "eo", "in"], [ui.eoZoomOut, "eo", "out"],
    [ui.irZoomIn, "ir", "in"], [ui.irZoomOut, "ir", "out"],
  ];
  for (const [btn, device, direction] of buttons) {
    if (!btn) continue;
    btn.addEventListener("pointerdown", (evt) => {
      if (btn.disabled) return;
      btn.setPointerCapture(evt.pointerId);
      startZoomHold(device, direction);
    });
    btn.addEventListener("pointerup",     () => stopZoomHold(device));
    btn.addEventListener("pointercancel", () => stopZoomHold(device));
    btn.addEventListener("pointerleave",  () => stopZoomHold(device));
  }

  // Closing/backgrounding the tab, or losing focus entirely, must not
  // leave either lens running with nobody watching.
  document.addEventListener("visibilitychange", () => {
    if (document.visibilityState === "hidden") stopAllZoomHolds();
  });
  window.addEventListener("pagehide", stopAllZoomHolds);
}

/* ── Boot ───────────────────────────────────────────────────────── */
wireJoystick();
wireControls();
wireZoomControls();
connectEvents();
setInterval(publishJoyStickControl, 100);  // ~10Hz joystick command send loop while dragging
setJoyLock(true);  // start locked
