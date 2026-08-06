# flir_ptz

ROS 2 control stack for FLIR PTZ cameras (**364C** and **M232**), speaking the
Nexus CGI HTTP API. Includes a push-based web dashboard, WebRTC low-latency
video via mediamtx, and a joystick bridge.

Targets **ROS 2 Jazzy** (Python 3.12); avoids Jazzy-only APIs so it also builds
on Humble.

---

## Repository layout

```
FLIR_Control/                    # repo root — not a package
├── flir_ptz_msgs/               # ament_cmake — interfaces only
│   ├── msg/  PtzState  MoveToCmd  JoyStickControlCmd  ScanCmd
│   │         ControlSource  ZoomCmd
│   └── srv/  ClaimControl
├── flir_ptz/                    # ament_python — all the code
│   ├── flir_ptz/
│   │   ├── nexus/    protocol.py  token.py  session.py
│   │   ├── control/  profiles.py  config.py  fsm.py  arbitration.py
│   │   │             gestures.py  zoom_optics.py  controller.py
│   │   ├── nodes/    ptz_node.py  web_node.py  joy_bridge.py
│   │   ├── webui/    server.py  sse.py
│   │   └── web/      index.html  app.js  styles.css  setup.html  setup.js
│   ├── launch/flir_ptz.launch.py
│   ├── config/params.example.yaml
│   └── test/                    # pytest — no ROS, no camera required
├── deploy/  startup_flir_ptz.sh  ros_env.sh  nginx-flir.conf
│           manage_auth.sh  mediamtx.yml  mediamtx-transcode.yml
└── README.md
```

The split into two packages is deliberate: `rosidl_generate_interfaces` claims
the package name for generated message modules, so a single package cannot also
install a hand-written Python package of the same name cleanly. Splitting frees
`flir_ptz` to be a normal `ament_python` package, which means
`colcon build --symlink-install` works properly and editing `.py` / `.html` /
`.js` / `.css` takes effect without a rebuild.

---

## Architecture

```
                    ┌─────────────────────────────────────┐
  /Nexus.cgi (HTTP) │            flir_ptz_node            │
  ◄───────────────► │  ONE tick loop: read + control      │
                    │  MotionFSM  ·  Arbiter  ·  Token    │
                    └───┬──────────────┬──────────────┬───┘
       /flir/ptz/state  │              │ /flir/cmd/*  │ flir/claim_control
                        │              │              │ /flir/control_source
              ┌─────────▼─────┐   ┌────▼──────────────▼───┐
              │ flir_ptz_web  │   │   flir_joy_bridge     │
              │ HTTP :8080    │   │   Joy → PTZ commands  │
              │ SSE push      │   │   circle-to-unlock    │
              └───────┬───────┘   └───────────────────────┘
         browser      │ SSE (telemetry down) + POST (commands up)
                      │
              ┌───────▼───────┐
              │   mediamtx    │  RTSP pull ◄── camera :8554
              │ :8889 WebRTC  │
              │ :9997 API     │
              └───────────────┘
```

### One tick loop

The controller reads the camera and issues control commands **on the same
tick**:

```
while running:
    intent = take_pending_intent()      # latest-wins, set by ROS callbacks
    sample = await session.read_nmea()  # always fresh
    publish_snapshot(sample, ...)
    result = fsm.step(sample, now)      # PURE — no I/O, no clock, no await
    for action in result.actions:
        await session.execute(action)
    await wait_for(wakeup_event, timeout=result.tick_period)
```

The camera's CGI endpoint is a single serial resource. An earlier design ran a
background telemetry poller *and* per-command tasks contending on one
semaphore, so reads and commands stole slots from each other and the
closed-loop tracking acted on stale positions. Reading and controlling on one
tick halves camera traffic and guarantees the control law sees a position taken
microseconds earlier.

Preemption is structural: a ROS callback replaces the pending intent and nudges
a wakeup event. There is nothing to cancel — the control plane never calls the
asyncio task cancellation API.

### Testability

Everything with decision content is a pure function of `(state, input, now)`.
Pure modules never import `rclpy` or `httpx` and never read the clock — `now`
is always injected. That is why the whole test suite runs with no ROS sourced,
no camera, and no third-party packages installed.

---

## Build

```bash
sudo apt install python3-httpx
source /opt/ros/jazzy/setup.bash
cd ~/FLIR_Control
colcon build --symlink-install
source install/setup.bash
```

## Test

```bash
cd flir_ptz && python3 -m pytest test/ -q
```

513 tests, no ROS, no camera and no third-party packages required. They are not
installed by `colcon build` and cost nothing at runtime.

Most of them exist because a specific failure already happened once and gave no
signal when it did — a token deadlock that froze the camera, an arbitration
lease that killed the joystick after 60 s idle, error responses read as
successes, a login made impossible by one query parameter, an unlock gesture
that only worked at one drawing speed, a CSS rule whose absence disabled the
entire control sidebar, and a lens that would run to its mechanical limit
unattended. None of those raised an error. Keep them.

---

## Credentials

Never committed. Resolved with precedence **YAML file < environment < explicit
override** (launch argument or the web setup page):

```bash
# environment
export FLIR_HOST=192.168.1.50 FLIR_USERNAME=admin FLIR_PASSWORD=... FLIR_MODEL=364c

# or a local YAML overlay (config/*.local.yaml is gitignored)
cp flir_ptz/config/params.example.yaml flir_ptz/config/camera.local.yaml
ros2 launch flir_ptz flir_ptz.launch.py \
    camera_config_yaml:=flir_ptz/config/camera.local.yaml
```

Every credential default in source, launch files and documentation is empty.
Verify before committing:

```bash
git check-ignore -v flir_ptz/config/camera.local.yaml
```

---

## Middleware: check this first if the dashboard "cannot connect"

The three nodes talk over ordinary ROS topics and services. On some hosts —
**WSL2 in particular** — the default Fast DDS middleware fails to discover
participants *across processes*, even though multicast, shared memory and
hostname resolution all test fine and pub/sub inside a single process works.
`ros2 node list` reports nothing at all.

The symptom in this project is misleading: the dashboard loads and its own HTTP
endpoints answer normally, but the setup page reports

> Connection timed out. Check camera IP/credentials and try again.

with a perfectly good camera IP and password, and never auto-redirects to
`/control` even though the PTZ node is connected. Nothing is wrong with the
credentials — the web node simply never receives the PTZ node's
`camera_status` message.

Check it in 30 seconds:

```bash
ros2 run demo_nodes_cpp talker &
ros2 topic echo /chatter --once
```

If no message arrives, switch to Zenoh:

```bash
sudo apt install ros-$ROS_DISTRO-rmw-zenoh-cpp   # if needed
source deploy/ros_env.sh                         # sets RMW + starts the router
```

Source it in **every** terminal that touches the ROS graph — a `ros2 topic
echo` or `joy_node` started without it will silently see an empty graph.

---

## Launch

```bash
# 364C — HTTP Basic Auth
ros2 launch flir_ptz flir_ptz.launch.py host:=192.168.1.50 login_mode:=basic

# M232 — POST login, no EO stream
ros2 launch flir_ptz flir_ptz.launch.py host:=192.168.1.60 login_mode:=post model:=m232

# Controller only
ros2 launch flir_ptz flir_ptz.launch.py launch_web:=false launch_joy:=false
```

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `host` `username` `password` | *(empty)* | Camera credentials — never defaulted |
| `login_mode` | `basic` | `basic` = 364C, `post` = M232 |
| `model` | `364c` | `364c` or `m232` |
| `camera_config_yaml` | *(empty)* | Local YAML credential overlay |
| `poll_hz` | `10.0` | IDLE-mode state publish rate |
| `poll_ms` | `60` | Active-mode tick period |
| `scan_poll_ms` | `150` | SCAN_ACTIVE tick period |
| `hold_token` | `false` | Hold the control token permanently (see below) |
| `home_on_shutdown` | `auto` | `auto` / `always` / `never` (see below) |
| `goto_feedback_frame` | `abs` | `abs` or `geo` (see below) |
| `lease_s` | `60.0` | Control-source lease duration |
| `launch_web` `web_host` `web_port` | `true` `0.0.0.0` `8080` | Web console |
| `mediamtx_api_port` | `9997` | mediamtx REST API port |
| `enable_ffplay` | `false` | Server-side ffplay endpoints (needs a display) |
| `launch_joy` `joy_topic` `joy_frame_id` | `true` `/joy` `ps5` | Joystick bridge |
| `verbose` `namespace` | `true` *(empty)* | |

---

## Topics, services, messages

| Name | Type | Direction |
|---|---|---|
| `/flir/ptz/state` | `PtzState` | published at `poll_hz` |
| `/flir/camera_status` | `std_msgs/String` (JSON) | published, latched |
| `/flir/control_source` | `ControlSource` | published, latched |
| `/flir/cmd/move_to` | `MoveToCmd` | subscribed — single-shot jump |
| `/flir/cmd/track` | `MoveToCmd` | subscribed — closed-loop tracking |
| `/flir/cmd/joy_stick_control` | `JoyStickControlCmd` | subscribed — direct speed |
| `/flir/cmd/scan` | `ScanCmd` | subscribed — auto-scan start/stop |
| `/flir/cmd/zoom` | `ZoomCmd` | subscribed — lens zoom, EO or IR |
| `/flir/camera_config` | `std_msgs/String` (JSON) | subscribed — live reconfigure |
| `/flir/claim_control` | `ClaimControl` | service |

All command messages carry a `source` field used for arbitration.

```bash
ros2 topic pub --once /flir/cmd/move_to flir_ptz_msgs/msg/MoveToCmd \
  '{target_azimuth: 45.0, target_elevation: 0.0, source: "cli"}'

ros2 topic pub --once /flir/cmd/scan flir_ptz_msgs/msg/ScanCmd \
  '{center_azimuth: 0.0, each_side_deg: 20.0, elevation: 5.0, speed: 5.0, stop: false, source: "cli"}'
```

---

## Control-source arbitration

The web UI and the joystick must not fight each other. Authority lives in
`flir_ptz_node` — the only process with an exclusive channel to the camera, and
therefore the only one that can make a rejection stick. (Previously the lock
lived in the web node's memory, so `ros2 topic pub` bypassed it entirely.)

- A client claims control via the `flir/claim_control` service. Last claim wins;
  there is no priority hierarchy.
- The current owner is broadcast on the latched `flir/control_source` topic, so
  late joiners and restarted nodes immediately learn the truth without a
  heartbeat protocol.
- A claim is a **lease** (`lease_s`, default 60 s), implicitly renewed by any
  accepted command. Closed browser tabs and dead bridges therefore release
  automatically.
- **Stop commands are always accepted from anyone**, regardless of ownership.
  A stop must never be blocked by arbitration.
- Both the web UI and the joystick require a full circular stick gesture to
  unlock — a deliberate guard against accidental camera movement. The two
  implementations share identical constants.

---

## Control token and the physical JCU

The camera grants one control token at a time. A physical JCU joystick competes
for it.

The previous implementation force-claimed the token every 10 seconds
unconditionally, even while completely idle — which took control away from a
human JCU operator every 10 seconds. The token is now kept alive only while a
motion mode is active or within a grace window after the last command, using the
non-destructive keepalive the camera's own web UI uses, and released afterwards.

- `hold_token:=true` restores permanent holding, for installations with no JCU.
- `home_on_shutdown` controls parking on exit:
  - `auto` (default) — park only if the token is ours or free. If another
    session holds it, an operator is driving: leave the camera alone.
  - `always` — force-claim and park regardless.
  - `never` — never park on exit.

---

## Zoom

Both lenses zoom. Verified on a 364C:

| | Command | Widest | Tele | Range |
|---|---|---|---|---|
| **VIS** (EO) | `DLTVZoomCountsIncrement` / `Decrement` / `Stop` | 63.7° | 2.12° | 30.0× optical |
| **eZoom** (IR) | `IRZoomIn` / `IRZoomOut` / `IRZoomStop` | 18.0° | 8.62° | 2.09× electronic |

Note the naming differs per device — the IR actions are not the DLTV ones with
the prefix swapped — and so does the state field: DLTV reports `Zoom_pctg`,
IR reports `Zoom_Pctg`. Reusing one parser for both silently reads 0.0.

Both are **continuous**: one command runs the lens until a stop arrives.

### The dead-man timer

Because a zoom keeps going, the browser re-sends the direction every 400 ms
while a button is held and the controller stops the lens by itself if no
renewal arrives within a second. A closed tab, a network drop or a lost
`pointerup` would otherwise drive the lens to its mechanical limit unattended.
The frontend also stops on `visibilitychange` and `pagehide`.

A renewal for a direction already running only refreshes that deadline — it
does not re-issue the command. Telling a lens to start zooming while it is
already zooming earns `RC=11 Device busy`, so re-issuing filled the log with
failures and wasted the camera's single serial channel.

`stop` is a stop command for arbitration: always accepted from any source,
regardless of who owns the lease. Any unrecognised direction is coerced to
`stop` rather than ignored — for a continuous zoom, "do nothing" is the
dangerous default.

EO and IR are tracked independently: separate pending slots, deadlines and
active flags, so zooming one never disturbs the other.

### Magnification

The camera does not report a zoom factor. Probing for capability, limits, lens,
info and magnification endpoints on both devices finds nothing — only
`SERVERVersionGet`, `DLTVLastNMEAGet` and `IRLastNMEAGet` answer at all.

It does report live field of view and zoom position as a percentage, and 0% is
the wide end by definition, so magnification is `widest_fov / current_fov` and
the reference is *learned from the camera* rather than configured. A reading at
the wide end is authoritative and is never revised by a narrower one. Until the
operator zooms out once, `eo_wide_fov_deg` / `ir_wide_fov_deg` stand in. The
provenance is logged, so it is visible whether a figure is calibrated or still
resting on a default.

---

## Coordinate frames

`PTGeoAzimuthElevationSet` commands the camera in the **geo** frame, while
arrival detection historically read the **abs** (mechanical gimbal) frame.

On a 364C with no platform heading offset these are the same angle expressed in
different ranges — `abs` in `(-180, 180]`, `geo` in `[0, 360)`; a measured
sample reads `Abs_Azimuth: -51.41` / `Geo_Azimuth: 308.59`. Azimuth error
normalisation makes them equivalent, so the default is safe.

They genuinely diverge once the platform has a non-zero heading offset (a moving
vehicle, or gyro stabilisation enabled). Then a `move_to` commands one frame and
waits for arrival in another, never converges, and the stall detector reports
failure even though the camera moved correctly.

The default `goto_feedback_frame:=abs` preserves historical behaviour. The node
monitors the two frames and logs a one-time warning if they diverge
persistently; switch to `geo` if you see it.

---

## Web dashboard

Open `http://<host>:8080` — the setup page asks for camera credentials, then
redirects to the control panel.

Telemetry is **pushed** over Server-Sent Events (`/api/events`), replacing the
previous 250 ms and 2 s polling loops. `EventSource` reconnects on its own. The
DOM is updated differentially — only fields whose formatted value actually
changed are written.

| Feature | Notes |
|---|---|
| Joystick | Drag to drive; circle gesture to unlock |
| Speed slider | 1–40 °/s |
| Move To / Center / Home | Home asks for confirmation |
| Scan | Center, half-width, elevation, speed |
| Telemetry bar | Geo & Abs AZ/EL, speed bars, mode |
| Status | Connection, Moving, Scanning, control source |
| Video | WebRTC (WHEP) primary, MJPEG fallback |

`connected` requires **both** an authenticated camera session *and* fresh
telemetry. Either signal alone is misleading: the status topic is latched, so a
dead PTZ node would otherwise leave a green light on forever; and staleness
alone cannot distinguish "unreachable" from "still starting up".

### Video

| Model | IR | EO |
|---|---|---|
| 364C | `rtsp://<ip>:8554/ir.0` | `rtsp://<ip>:8554/vis.0` |
| M232 | `rtsp://<ip>:8554/ir` | not available |

Switching model in the UI updates mediamtx's sources through its REST API — no
restart needed. Video does not pass through the dashboard's HTTP server; the
browser connects to mediamtx on `:8889` directly. If that port is unreachable
(for example over a VPN that only forwards `:8080`), the UI falls back to the
MJPEG stream, which does go through the proxy.

---

## Joystick bridge

Subscribes to `sensor_msgs/Joy`, processing only messages whose `frame_id`
matches `joy_frame_id` (default `ps5`).

| Input | Action |
|---|---|
| `axes[2]` | Azimuth speed (stick right = positive) |
| `axes[3]` | Elevation speed |
| `buttons[8]` tap | Center (AZ 0, EL 0) |
| `buttons[8]` held ≥ 3 s | Home (AZ 0, EL −90) |
| Full circular stick motion | Claim control / unlock |

Deadzone 0.05, maximum 40 °/s. Commands are published only while moving or on
the moving→stopped transition, so the topic is not flooded with zeros while
idle. A zero-speed command is emitted even when the bridge is locked out — if
the web UI takes control mid-motion, the camera must still be told to stop.

The bridge talks to the PTZ node purely over ROS. (It previously polled the web
node over localhost HTTP twice a second.)

---

## Deployment

```bash
# Video only — starts mediamtx, no nginx, no sudo. This is all you need for
# the dashboard's WebRTC video.
bash deploy/startup_flir_ptz.sh --video-only <CAMERA_IP> [364c|m232]

# Full: also installs and configures nginx + Basic Auth for exposing the
# dashboard beyond this machine.
bash deploy/startup_flir_ptz.sh <CAMERA_IP> [364c|m232]

bash deploy/startup_flir_ptz.sh stop
```

Run it, do not `source` it. The camera IP is required.

Streams are pulled **on demand**: `mediamtx` reports `ready=false` until a
viewer actually opens one, then connects to the camera. That is expected, not a
fault — check with:

```bash
curl -s http://127.0.0.1:9997/v3/paths/list
```

`ffmpeg` is **not** needed for WebRTC video; it is only used by the MJPEG
fallback that the UI switches to when WebRTC cannot be reached.

Downloads mediamtx for the host architecture, configures nginx with Basic Auth,
and starts the stream server. The camera IP is required — there is no default.
Credentials may also come from `FLIR_HOST`, `FLIR_AUTH_USER`, `FLIR_AUTH_PASS`.

Passwordless sudo is **not** configured unless you pass `--install-sudoers`, and
the generated rule is validated with `visudo -c` before being kept.

`deploy/nginx-flir.conf` disables proxy buffering for `/api/events` and
`/api/stream/` — without that, nginx holds SSE frames back and the dashboard
receives telemetry in delayed bursts or not at all.

---

## Nexus CGI reference

```
GET /Nexus.cgi?action=<CMD>&<params>&session=<ID>&DeviceID=0&_=<timestamp_ms>
```

| Command | Key parameters |
|---|---|
| `SERVERWhoAmI` | — (returns numeric session `Id`) |
| `SERVERLastNMEAGet` | `tokenoverride` (keepalive) — returns `Token_ID` |
| `SERVERRemoteControlRequestAsync` | `Forced` |
| `SERVERRemoteControlRelease` | — |
| `PTLastNMEAGet` | — (position, speed, mode) |
| `PTGeoAzimuthElevationSet` | `Geo_Azimuth`, `Geo_Elevation` |
| `PTSpeedModeSet` | `Azimuth_Speed`, `Elevation_Speed` |
| `PTAutoScanSpeedSet` | `Speed` |
| `PTAutoScanLimitsSet` | `Left_Azimuth`, `Right_Azimuth` |
| `PTAutoScanModeOn` / `PTAutoScanModeOff` | — |

Two wire details that are easy to get wrong, both verified against a 364C:

**Failures are keyed by `error`, not by the command name.**

```json
{"PTLastNMEAGet": {"Return Code": 0, "Abs_Azimuth": -51.41, ...}}
{"error":         {"Return Code": 21, "Return String": "Network client not authorized"}}
```

Looking the payload up by command name alone turns every failure into a silent
success — and since missing fields parse as `0.0`, an unauthorised read then
looks like a camera parked at AZ 0 / EL 0.

**Omit `session` entirely when you do not have one.** `session=0` is rejected:

```
?action=SERVERWhoAmI&DeviceID=0            → {"SERVERWhoAmI": {"Return Code": 0, "Id": 103, ...}}
?action=SERVERWhoAmI&session=0&DeviceID=0  → {"error": {"Return Code": 21, ...}}
```

Since `SERVERWhoAmI` is how the session id is obtained, sending `session=0`
there makes login impossible.
