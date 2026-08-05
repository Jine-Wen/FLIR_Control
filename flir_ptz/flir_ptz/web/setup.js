(() => {
  const form      = document.getElementById("connect-form");
  const btn       = document.getElementById("connect-btn");
  const statusBox = document.getElementById("status-box");
  const params    = new URLSearchParams(location.search);
  const urlError  = params.get("error");

  // Sensible client-side timeout for the non-blocking /api/connect flow —
  // the real result arrives asynchronously via the `camera_status` SSE
  // event, so we bound how long we wait for it.
  const CONNECT_TIMEOUT_MS = 20000;
  // Old behaviour: give up watching for an already-connected camera on
  // fresh page load after ~8s and just show the empty form.
  const INITIAL_CHECK_TIMEOUT_MS = 8000;

  let formSubmitted   = false;
  let redirected      = false;
  let initialCheckDone = false;
  let connectTimeoutId = null;
  let initialTimeoutId = null;

  function setStatus(type, msg) {
    statusBox.textContent = msg;
    statusBox.className   = `status-box show ${type}`;
  }

  function hideStatus() {
    statusBox.className = "status-box";
  }

  function setLoading(on) {
    btn.disabled = on;
    btn.classList.toggle("loading", on);
    btn.querySelector(".btn-text").textContent = "Connect";
  }

  function clearConnectTimeout() {
    if (connectTimeoutId !== null) { clearTimeout(connectTimeoutId); connectTimeoutId = null; }
  }

  function clearInitialTimeout() {
    if (initialTimeoutId !== null) { clearTimeout(initialTimeoutId); initialTimeoutId = null; }
  }

  // Show error if redirected here due to disconnect
  if (urlError) {
    setStatus("error", "⚠ Camera disconnected: " + urlError);
  } else {
    setStatus("info", "Checking camera connection…");
    initialTimeoutId = setTimeout(() => {
      if (!initialCheckDone && !formSubmitted) hideStatus();
    }, INITIAL_CHECK_TIMEOUT_MS);
  }

  // ── Push-driven camera status (replaces the old up-to-8s poll loop) ────
  // The server sends the current camera_status immediately on connect, so
  // the very first event tells us whether the node is already connected.
  const es = new EventSource("/api/events");

  es.addEventListener("camera_status", (evt) => {
    let payload;
    try { payload = JSON.parse(evt.data); } catch { return; }
    handleCameraStatus(payload);
  });

  function handleCameraStatus(payload) {
    initialCheckDone = true;
    clearInitialTimeout();

    const status = payload && payload.status;

    if (status === "connected") {
      // Don't auto-bounce away from an explicit disconnect-error page the
      // user was just sent to unless they submitted new credentials here.
      if (!formSubmitted && urlError) return;
      if (redirected) return;
      redirected = true;
      clearConnectTimeout();
      setStatus("success", "Camera connected. Redirecting…");
      setLoading(false);
      setTimeout(() => { location.href = "/control"; }, 500);
      return;
    }

    if (status === "error") {
      if (formSubmitted) {
        clearConnectTimeout();
        setStatus("error", payload.error || "Connection failed.");
        setLoading(false);
        formSubmitted = false;
      } else if (!urlError) {
        // No prior configuration / camera never connected — just show the form.
        hideStatus();
      }
      return;
    }

    if (status === "connecting") {
      if (formSubmitted) {
        setStatus("info", "Connecting to camera…");
      } else if (!urlError) {
        setStatus("info", "Checking camera connection…");
      }
    }
  }

  es.onerror = () => {
    // Reflect nothing beyond letting the current status message stand —
    // EventSource reconnects automatically, no manual retry loop needed.
  };

  // ── Form submit ───────────────────────────────────────────────────────
  form.addEventListener("submit", async (e) => {
    e.preventDefault();
    hideStatus();

    const host     = document.getElementById("host").value.trim();
    const model    = document.getElementById("model").value;
    const username = document.getElementById("username").value.trim();
    const password = document.getElementById("password").value;

    if (!host)     { setStatus("error", "Camera IP address is required."); return; }
    if (!password) { setStatus("error", "Password is required."); return; }

    formSubmitted = true;
    redirected    = false;
    setLoading(true);
    setStatus("info", "Connecting to camera…");

    try {
      const resp = await fetch("/api/connect", {
        method:  "POST",
        headers: { "Content-Type": "application/json" },
        body:    JSON.stringify({ host, model, username, password }),
      });

      const data = await resp.json();

      if (!data.ok) {
        setStatus("error", data.error || "Connection failed.");
        setLoading(false);
        formSubmitted = false;
        return;
      }

      // /api/connect is non-blocking: {"ok":true,"pending":true}.
      // The real outcome arrives via the `camera_status` SSE event above.
      clearConnectTimeout();
      connectTimeoutId = setTimeout(() => {
        if (formSubmitted) {
          setStatus("error", "Connection timed out. Check camera IP/credentials and try again.");
          setLoading(false);
          formSubmitted = false;
        }
      }, CONNECT_TIMEOUT_MS);
    } catch (err) {
      setStatus("error", `Request failed: ${err.message}`);
      setLoading(false);
      formSubmitted = false;
    }
  });
})();
