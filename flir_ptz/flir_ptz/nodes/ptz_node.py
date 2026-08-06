#!/usr/bin/env python3
"""FLIR PTZ ROS 2 node -- the thin rclpy shell around the control plane
(spec ARCHITECTURE.md sec. 5). All decision logic lives in
``control/fsm.py`` (pure) and ``control/controller.py`` (the single tick
loop); this module's job is exactly three things:

  1. Own the background asyncio thread the tick loop runs on, and the
     outer "connect / reconnect-on-reconfigure" supervisor loop around
     it (PARITY A23/A28) -- ``control/controller.py`` deliberately does
     *not* own this: it assumes an already-connected ``CameraSession``
     and is unit-testable standalone against it (see
     ``test/test_controller.py``).
  2. Translate ROS messages <-> ``Intent`` / ``PtzState`` / status JSON.
  3. **Enforce control-source arbitration** (spec sec. 3.5/5): this is
     the one authority in the whole system that can actually make
     rejection stick, because it is the only process with an exclusive
     channel to the camera. A command whose ``source`` does not match
     the current ``Arbiter`` owner is dropped here -- except stop
     commands (zero-speed joystick, scan ``stop=true``), which are
     *always* accepted regardless of ownership (non-negotiable safety
     rule, spec sec. 3.5).

Publishes: flir/ptz/state      (PtzState, depth 10)
           flir/camera_status  (String JSON, latched)   -- PARITY A2/A28
           flir/control_source (ControlSource, latched) -- arbitration broadcast
Subscribes: flir/cmd/move_to, flir/cmd/track, flir/cmd/joy_stick_control,
            flir/cmd/scan, flir/camera_config
Service:   flir/claim_control (ClaimControl)

Every credential parameter defaults to the empty string (spec sec. 8
rule 1) -- real values only ever come from a gitignored local YAML file,
environment variables, or explicit launch/web overrides, resolved via
``control/config.py``'s ``load_camera_config`` (YAML < env < overrides).
"""

from __future__ import annotations

import asyncio
import json
import os
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String

from flir_ptz_msgs.msg import ControlSource, JoyStickControlCmd, MoveToCmd, PtzState, ScanCmd
from flir_ptz_msgs.srv import ClaimControl

from flir_ptz.control.arbitration import Arbiter
from flir_ptz.control.config import CameraConfig, ControlConfig, load_camera_config
from flir_ptz.control.controller import TickController
from flir_ptz.control.fsm import Intent, Mode, MotionFSM, StepResult
from flir_ptz.nexus.protocol import PtSample
from flir_ptz.nexus.session import CameraSession
from flir_ptz.nexus.token import TokenPolicy

# Latched QoS for camera_status / control_source (spec sec. 5): late
# subscribers (a freshly (re)loaded web page, a joy bridge that just
# started) must see the last published value immediately, with no
# heartbeat/poll protocol required.
_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
)


def _normalize_host(host: str) -> str:
    """A bare ``CameraConfig.host`` (just an IP address or hostname, no
    scheme) needs an ``http://`` prefix before it can be used as an
    ``httpx``/CameraSession base URL -- old node behaviour
    (``flir_ptz_node.py``'s ``_apply_pending_config``), preserved
    verbatim."""
    if not host:
        return ""
    if host.startswith("http://") or host.startswith("https://"):
        return host
    return f"http://{host}"


def _default_session_cache_path() -> Path:
    # PARITY A24 (reuse session_id across restarts) without PARITY-doc's
    # noted regression (old code wrote it world-readable to /tmp, spec
    # "刻意改变行为的项目" table): a per-user cache dir, file written 0600.
    return Path.home() / ".cache" / "flir_ptz" / "session_id"


class FlirPtzNode(Node):
    def __init__(self) -> None:
        super().__init__("flir_ptz")

        self._clock_fn = time.monotonic  # for Arbiter / TickController -- never ROS time

        self._declare_parameters()
        self._read_parameters()

        # -- arbitration (authoritative here -- spec sec. 3.5/5) -----------
        self._arbiter = Arbiter(lease_s=self._lease_s)
        self._last_published_owner: Optional[str] = None

        # -- shared mutable state between the ROS executor thread and the
        # background asyncio thread. Only ``_controller`` / ``_fsm`` /
        # ``_last_sample`` are ever touched from both sides; a plain lock
        # is enough (no long-held critical sections on either side).
        self._state_lock = threading.Lock()
        self._controller: Optional[TickController] = None
        self._fsm: Optional[MotionFSM] = None
        self._last_sample: Optional[PtSample] = None

        # -- publishers ------------------------------------------------
        self._state_pub = self.create_publisher(PtzState, "flir/ptz/state", 10)
        self._cam_status_pub = self.create_publisher(String, "flir/camera_status", _LATCHED_QOS)
        self._control_source_pub = self.create_publisher(
            ControlSource, "flir/control_source", _LATCHED_QOS
        )

        # Broadcast owner="" immediately on startup (spec sec. 5 "重启语意":
        # both web and joy bridge must see this and re-lock themselves).
        self._publish_control_source(force=True)

        # -- subscriptions -----------------------------------------------
        self.create_subscription(MoveToCmd, "flir/cmd/move_to", self._on_move_to, 5)
        self.create_subscription(MoveToCmd, "flir/cmd/track", self._on_track, 5)
        self.create_subscription(
            JoyStickControlCmd, "flir/cmd/joy_stick_control", self._on_joy_stick_control, 5
        )
        self.create_subscription(ScanCmd, "flir/cmd/scan", self._on_scan, 5)
        self.create_subscription(String, "flir/camera_config", self._on_config, 5)

        # -- service (PARITY C17) -----------------------------------------
        self.create_service(ClaimControl, "flir/claim_control", self._on_claim_control)

        # -- periodic arbitration-expiry sweep ------------------------------
        # claim()/release() already publish synchronously; this is only for
        # the case nobody actively releases -- the lease simply expires.
        self.create_timer(1.0, self._check_arbiter_expiry)

        # -- background asyncio thread: connect/reconnect supervisor + the
        # single tick loop (control/controller.py) ------------------------
        self._alive = True
        self._loop = asyncio.new_event_loop()
        self._reconfig_event: Optional[asyncio.Event] = None
        self._thread = threading.Thread(target=self._run_loop, daemon=True, name="flir_ptz_loop")
        self._thread.start()

        self.get_logger().info(
            f"FlirPtzNode started -> host={_normalize_host(self._camera_cfg.host) or '(unset)'}, "
            f"login_mode={self._camera_cfg.login_mode}, goto_feedback_frame={self._goto_feedback_frame}"
        )

    # -- parameters ---------------------------------------------------------

    def _declare_parameters(self) -> None:
        g = self.declare_parameter
        # -- camera connection (spec sec. 8 rule 1: credentials default "") --
        g("host", "")
        g("username", "")
        g("password", "")
        g("login_mode", "basic")  # "basic"=364C | "post"=M232
        g("model", "364c")
        g("camera_config_yaml", "")  # optional gitignored local YAML overlay
        g("session_cache_path", str(_default_session_cache_path()))

        # -- ported verbatim from the old node (flir_ptz_node.py:92-109) --
        g("poll_hz", 10.0)
        g("poll_ms", 60)
        g("scan_poll_ms", 150)
        g("az_tol", 0.5)
        g("el_tol", 0.5)
        g("az_hold_tol", 0.7)
        g("el_hold_tol", 0.7)
        g("settle_samples", 4)
        g("timeout_ms", 45000)
        g("interrupt_scan_wait_ms", 150)
        g("stop_scan_timeout_ms", 5000)
        g("scan_save_wait_ms", 150)
        g("scan_start_wait_ms", 300)
        g("verbose", True)

        # -- new in this rewrite --
        g("hold_token", False)  # spec sec. 3.3 bug fix: default no longer
        # force-claims the token unconditionally every 10s from a JCU.
        g(
            "goto_feedback_frame",
            "abs",
            # See worker brief "COORDINATE-FRAME ISSUE": PTGeoAzimuthElevationSet
            # commands the camera in the GEO frame, but goto/home arrival
            # detection historically reads the ABS (mechanical gimbal) frame.
            # They coincide only when the platform's heading offset is zero.
            # "abs" (default) is bit-for-bit the old, currently-working
            # behaviour -- zero regression risk. Set to "geo" if this
            # installation's platform has a non-zero heading offset (e.g. a
            # moving vehicle): goto/home will otherwise appear to "stall"
            # even though the camera moved correctly. A one-time log
            # warning fires automatically if the two frames are observed to
            # diverge persistently while running with the default "abs".
        )
        g("lease_s", 60.0)  # control-source arbitration lease duration
        g(
            "home_on_shutdown",
            "auto",
            # "auto"   -- park the camera on exit only if the control token is
            #             ours or free (nobody is driving). Safe default.
            # "always" -- force-claim the token and park regardless.
            # "never"  -- never park on exit.
            # Needed because dropping the old unconditional 10s force-claim
            # (which stole control from a physical JCU) also meant an idle
            # node no longer held the token at shutdown, so the camera
            # silently stopped being parked.
        )

    def _read_parameters(self) -> None:
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        overrides = {
            "host": _normalize_host(str(g("host") or "")),
            "username": str(g("username") or ""),
            "password": str(g("password") or ""),
            "login_mode": str(g("login_mode") or "basic"),
            "model": str(g("model") or "364c"),
        }
        yaml_str = str(g("camera_config_yaml") or "")
        self._camera_config_yaml: Optional[Path] = Path(yaml_str) if yaml_str else None
        self._camera_cfg: CameraConfig = load_camera_config(
            env=os.environ, yaml_path=self._camera_config_yaml, overrides=overrides
        )

        self._control_cfg = ControlConfig(
            poll_hz=float(g("poll_hz")),
            poll_ms=int(g("poll_ms")),
            scan_poll_ms=int(g("scan_poll_ms")),
            az_tol=float(g("az_tol")),
            el_tol=float(g("el_tol")),
            az_hold_tol=float(g("az_hold_tol")),
            el_hold_tol=float(g("el_hold_tol")),
            settle_samples=int(g("settle_samples")),
            timeout_s=float(g("timeout_ms")) / 1000.0,
            interrupt_scan_wait_s=float(g("interrupt_scan_wait_ms")) / 1000.0,
            stop_scan_timeout_s=float(g("stop_scan_timeout_ms")) / 1000.0,
            scan_save_wait_s=float(g("scan_save_wait_ms")) / 1000.0,
            scan_start_wait_s=float(g("scan_start_wait_ms")) / 1000.0,
            verbose=bool(g("verbose")),
        )

        self._hold_token = bool(g("hold_token"))
        self._goto_feedback_frame = str(g("goto_feedback_frame") or "abs")
        self._home_on_shutdown = str(g("home_on_shutdown") or "auto").lower()
        self._lease_s = float(g("lease_s"))

        cache_str = str(g("session_cache_path") or "")
        self._session_cache_path: Optional[Path] = Path(cache_str) if cache_str else None

        self._verbose = bool(g("verbose"))

    # -- background loop / supervisor ----------------------------------------

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._supervisor())
        except Exception as exc:  # pragma: no cover - defensive top-level guard
            self.get_logger().error(f"[flir_ptz] supervisor loop crashed: {exc}")

    def _token_policy(self) -> TokenPolicy:
        return TokenPolicy(hold_token=self._hold_token)

    async def _supervisor(self) -> None:
        """Connect, run the tick loop, and reconnect on demand (PARITY
        A23/A28) -- everything ``control/controller.py`` itself does not
        own (see module docstring)."""
        self._reconfig_event = asyncio.Event()

        while self._alive:
            camera_cfg = self._camera_cfg
            session = CameraSession(
                camera_cfg,
                session_id=self._load_saved_session_id(),
                token_policy=self._token_policy(),
            )

            connected = await self._connect_with_retry(session, camera_cfg)
            if not self._alive:
                try:
                    await session.close()
                except Exception:
                    pass
                break
            if not connected:
                # _connect_with_retry only returns False if a reconfigure
                # arrived while we were retrying -- loop back around and
                # rebuild with the new config. Close whatever half-open
                # transport the failed connect attempt(s) may have left
                # behind first.
                try:
                    await session.close()
                except Exception:
                    pass
                continue

            self._save_session_id(session.session_id)
            self._publish_cam_status("connected", camera_cfg.host, "")

            fsm = MotionFSM(self._control_cfg)
            controller = TickController(
                session,
                fsm,
                self._control_cfg,
                goto_feedback_frame=self._goto_feedback_frame,
                home_on_shutdown=self._home_on_shutdown,
                token_policy=self._token_policy(),
                on_snapshot=self._on_snapshot,
                logger=self.get_logger(),
            )
            with self._state_lock:
                self._controller = controller
                self._fsm = fsm

            self._reconfig_event.clear()
            run_task = asyncio.create_task(controller.run())

            # Poll (never cancel -- see controller.py sec. 8 rule 2, kept
            # consistent here too) until either the tick loop ends on its
            # own (shutdown -- see destroy_node) or a reconfigure request
            # comes in, in which case we ask it to stop and wait for it.
            while not run_task.done():
                if self._reconfig_event.is_set():
                    controller.request_stop()
                    break
                await asyncio.sleep(0.2)
            await run_task

            with self._state_lock:
                if self._controller is controller:
                    self._controller = None
                    self._fsm = None

            if not self._alive:
                break

            self.get_logger().info("[flir_ptz] reconfiguration requested -- reconnecting...")
            try:
                await session.close()
            except Exception:
                pass
            # loop back around: self._camera_cfg was updated by _on_config

    async def _connect_with_retry(self, session: CameraSession, camera_cfg: CameraConfig) -> bool:
        """Old node's ``_main`` connect phase (node:224-241, PARITY A28):
        publish status, retry every 3s, and respond immediately to a
        reconfigure request instead of waiting out the retry delay."""
        self._publish_cam_status("connecting", camera_cfg.host, "")
        while self._alive:
            try:
                await session.connect()
                return True
            except Exception as exc:
                self.get_logger().warn(f"[flir_ptz] connection failed: {exc} -- retrying")
                self._publish_cam_status("error", camera_cfg.host, str(exc))
                self._reconfig_event.clear()
                try:
                    await asyncio.wait_for(self._reconfig_event.wait(), timeout=3.0)
                except asyncio.TimeoutError:
                    pass
                if self._reconfig_event.is_set():
                    return False  # caller re-reads self._camera_cfg and rebuilds
        return False

    # -- snapshot publishing (called from the background asyncio thread) ----

    def _on_snapshot(self, sample: Optional[PtSample], result: StepResult) -> None:
        if sample is not None:
            with self._state_lock:
                self._last_sample = sample
        with self._state_lock:
            effective = sample if sample is not None else self._last_sample
        if effective is None:
            return  # never had a successful read yet -- nothing to publish

        msg = PtzState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "flir_ptz"
        msg.abs_azimuth = effective.abs_az
        msg.abs_elevation = effective.abs_el
        msg.geo_azimuth = effective.geo_az
        msg.geo_elevation = effective.geo_el
        msg.speed_x = effective.speed_x
        msg.speed_y = effective.speed_y
        msg.mode = effective.mode
        msg.is_moving = result.is_moving
        msg.is_scanning = result.is_scanning
        msg.active_move_seq = result.seq
        msg.active_scan_seq = result.seq
        self._publish_best_effort(self._state_pub, msg)

    @staticmethod
    def _publish_best_effort(publisher, msg) -> None:
        """Publish, but never let a publish failure escape into the control loop.

        This is called from the asyncio tick thread. On SIGTERM, rclpy's signal
        handler invalidates the context *before* ``destroy_node`` runs, so any
        publish issued during the shutdown sequence raises "publisher's context
        is invalid". Letting that propagate aborts ``controller.run()``, which
        crashes the supervisor, which stops the event loop -- orphaning the
        shutdown homing coroutine and the logout that were the entire point of
        the shutdown sequence. Telemetry is best-effort; parking the camera is
        not.
        """
        try:
            publisher.publish(msg)
        except Exception:
            pass

    def _publish_cam_status(self, status: str, host: str, error: str) -> None:
        msg = String()
        msg.data = json.dumps({"status": status, "host": host, "error": error})
        self._publish_best_effort(self._cam_status_pub, msg)

    # -- control-source arbitration (authoritative here) ---------------------

    def _ros_time_after(self, seconds_from_now: float):
        return (self.get_clock().now() + Duration(seconds=max(0.0, seconds_from_now))).to_msg()

    def _publish_control_source(self, force: bool = False) -> None:
        now = self._clock_fn()
        owner = self._arbiter.owner(now)
        if not force and owner == self._last_published_owner:
            return
        self._last_published_owner = owner
        msg = ControlSource()
        msg.owner = owner
        msg.claimed_at = self.get_clock().now().to_msg()
        if owner:
            # Best-effort: lease_s from now (renew()/claim() both reset the
            # expiry to lease_s out, so "now + lease_s" is exact right after
            # a claim and a very close upper bound otherwise).
            msg.expires = self._ros_time_after(self._lease_s)
        else:
            msg.expires = self.get_clock().now().to_msg()
        self._control_source_pub.publish(msg)

    def _check_arbiter_expiry(self) -> None:
        self._publish_control_source(force=False)

    def _on_claim_control(self, request: ClaimControl.Request, response: ClaimControl.Response):
        now = self._clock_fn()
        if request.release:
            result = self._arbiter.release(request.source, now)
        else:
            result = self._arbiter.claim(request.source, now)
        response.granted = result.granted
        response.owner = result.owner
        response.expires = (
            self._ros_time_after(result.expires_at - now)
            if result.owner
            else self.get_clock().now().to_msg()
        )
        self._publish_control_source(force=True)
        return response

    # -- command subscriptions: arbitration enforced here --------------------

    def _controller_ref(self) -> Optional[TickController]:
        with self._state_lock:
            return self._controller

    def _submit_if_allowed(self, source: str, is_stop: bool, intent: Intent, label: str) -> None:
        now = self._clock_fn()
        if not self._arbiter.allows(source, is_stop, now):
            self.get_logger().debug(
                f"[flir_ptz] dropping {label} from source={source!r} "
                f"(current owner={self._arbiter.owner(now)!r})"
            )
            return

        # An accepted command from a source that does not currently own the
        # lease means the lease was unowned (Arbiter.allows only lets those
        # two cases through) -- so take it, rather than merely renewing a
        # lease nobody holds. Without this the command would be executed but
        # ownership would stay empty, the next command would be judged
        # against an empty owner again, and the control_source topic would
        # never reflect who is actually driving.
        if self._arbiter.owner(now) != source and not is_stop:
            self._arbiter.claim(source, now)
        else:
            self._arbiter.renew(source, now)
        self._publish_control_source(force=False)

        controller = self._controller_ref()
        if controller is None:
            self.get_logger().warn(f"[flir_ptz] dropping {label}: camera not connected yet")
            return
        controller.submit_threadsafe(intent)

    def _on_move_to(self, msg: MoveToCmd) -> None:
        self._submit_if_allowed(
            msg.source, False, Intent.goto(msg.target_azimuth, msg.target_elevation, msg.source), "move_to"
        )

    def _on_track(self, msg: MoveToCmd) -> None:
        self._submit_if_allowed(
            msg.source, False, Intent.track(msg.target_azimuth, msg.target_elevation, msg.source), "track"
        )

    def _on_joy_stick_control(self, msg: JoyStickControlCmd) -> None:
        # PARITY safety rule (spec sec. 3.5/5): a zero-speed joystick
        # command is a stop command and must ALWAYS be accepted.
        is_stop = msg.azimuth_speed == 0.0 and msg.elevation_speed == 0.0
        self._submit_if_allowed(
            msg.source,
            is_stop,
            Intent.velocity(msg.azimuth_speed, msg.elevation_speed, msg.source),
            "joy_stick_control",
        )

    def _on_scan(self, msg: ScanCmd) -> None:
        if msg.stop:
            self._submit_if_allowed(msg.source, True, Intent.scan_stop(msg.source), "scan(stop)")
        else:
            self._submit_if_allowed(
                msg.source,
                False,
                Intent.scan_start(msg.center_azimuth, msg.each_side_deg, msg.elevation, msg.speed, msg.source),
                "scan(start)",
            )

    # -- dynamic reconfiguration (PARITY A23) ---------------------------------

    def _on_config(self, msg: String) -> None:
        try:
            cfg = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().error(f"[flir_ptz] [config] invalid JSON: {exc}")
            return

        overrides = {
            "host": _normalize_host(str(cfg.get("host", "") or "")),
            "username": str(cfg.get("username", "") or ""),
            "password": str(cfg.get("password", "") or ""),
            "model": str(cfg.get("model", "") or ""),
        }
        if "login_mode" in cfg and cfg.get("login_mode"):
            overrides["login_mode"] = str(cfg["login_mode"])
        elif overrides["model"]:
            overrides["login_mode"] = "post" if overrides["model"].lower() == "m232" else "basic"

        # Partial updates fall back to the currently-live config for any
        # field the web payload omitted, rather than blowing it away --
        # an improvement on the old node's _apply_pending_config, which
        # silently dropped any field not present in the incoming JSON.
        current = self._camera_cfg
        merged_overrides = {
            "host": overrides["host"] or current.host,
            "username": overrides["username"] or current.username,
            "password": overrides["password"] or current.password,
            "login_mode": overrides.get("login_mode", current.login_mode),
            "model": overrides["model"] or current.model,
        }

        self._camera_cfg = load_camera_config(
            env=os.environ, yaml_path=self._camera_config_yaml, overrides=merged_overrides
        )
        self.get_logger().info(
            f"[flir_ptz] [config] new camera config received: host={self._camera_cfg.host} "
            f"login_mode={self._camera_cfg.login_mode}"
        )
        if self._loop is not None and self._reconfig_event is not None:
            self._loop.call_soon_threadsafe(self._reconfig_event.set)

    # -- session_id persistence (PARITY A24 -- kept out of world-readable /tmp) --

    def _load_saved_session_id(self) -> Optional[int]:
        if self._session_cache_path is None:
            return None
        try:
            if self._session_cache_path.exists():
                text = self._session_cache_path.read_text(encoding="utf-8").strip()
                if text:
                    sid = int(text)
                    self.get_logger().info(f"[flir_ptz] reusing cached session_id={sid}")
                    return sid
        except Exception as exc:
            self.get_logger().warn(f"[flir_ptz] could not read cached session_id: {exc}")
        return None

    def _save_session_id(self, session_id: Optional[int]) -> None:
        if session_id is None or self._session_cache_path is None:
            return
        try:
            self._session_cache_path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
            fd = os.open(
                str(self._session_cache_path), os.O_CREAT | os.O_WRONLY | os.O_TRUNC, 0o600
            )
            try:
                os.write(fd, str(session_id).encode("utf-8"))
            finally:
                os.close(fd)
        except Exception as exc:
            self.get_logger().warn(f"[flir_ptz] could not cache session_id: {exc}")

    # -- shutdown (PARITY A26/A27) --------------------------------------------

    def destroy_node(self) -> None:
        self._alive = False

        controller = self._controller_ref()
        if controller is not None:
            try:
                home_future = controller.request_home(timeout=25.0)
                homed = home_future.result(timeout=27.0)
                self.get_logger().info(
                    "[shutdown] homing complete" if homed else "[shutdown] homing skipped/did not settle"
                )
            except Exception as exc:
                self.get_logger().warn(f"[shutdown] homing failed/timed out: {exc}")

            try:
                controller.request_logout().result(timeout=10.0)
            except Exception as exc:
                self.get_logger().warn(f"[shutdown] logout failed: {exc}")

            controller.request_stop()

        if self._reconfig_event is not None and self._loop is not None:
            # Wake the supervisor's connect-retry wait too, in case it is
            # currently blocked there rather than inside controller.run().
            try:
                self._loop.call_soon_threadsafe(self._reconfig_event.set)
            except Exception:
                pass

        self._thread.join(timeout=6.0)
        try:
            self._loop.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlirPtzNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as exc:  # pragma: no cover - top-level guard
        print(f"[main] unexpected error: {exc}")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
