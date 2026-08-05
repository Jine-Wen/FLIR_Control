"""flir_ptz.launch.py — bring up the FLIR PTZ controller, web console and joy bridge.

Credentials are NEVER defaulted here. Supply them by one of, in increasing
precedence:

  1. a gitignored local YAML overlay
       ros2 launch flir_ptz flir_ptz.launch.py \
           camera_config_yaml:=/path/to/camera.local.yaml
  2. environment variables
       FLIR_HOST=10.0.0.50 FLIR_USERNAME=admin FLIR_PASSWORD=... \
           ros2 launch flir_ptz flir_ptz.launch.py
  3. explicit launch arguments (shown below), or the web setup page at
     http://<host>:8080/setup which reconfigures a running node live.

Examples
--------
    # 364C over HTTP Basic Auth
    ros2 launch flir_ptz flir_ptz.launch.py host:=10.0.0.50 login_mode:=basic

    # M232 over POST login
    ros2 launch flir_ptz flir_ptz.launch.py host:=10.0.0.60 login_mode:=post model:=m232

    # Controller only, no web console and no joystick
    ros2 launch flir_ptz flir_ptz.launch.py launch_web:=false launch_joy:=false

    # No physical JCU on this rig: hold the control token permanently
    ros2 launch flir_ptz flir_ptz.launch.py hold_token:=true

    # Platform with a non-zero heading offset (e.g. a moving vehicle)
    ros2 launch flir_ptz flir_ptz.launch.py goto_feedback_frame:=geo
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _arg(name: str, default: str, description: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default, description=description)


def generate_launch_description() -> LaunchDescription:
    args = [
        # ── Camera connection ────────────────────────────────────────────────
        # All credential defaults are deliberately EMPTY. Never commit a real
        # IP or password into this file.
        _arg("host", "", "Camera IP address (or set FLIR_HOST)"),
        _arg("username", "", "Camera login username (or set FLIR_USERNAME)"),
        _arg("password", "", "Camera login password (or set FLIR_PASSWORD)"),
        _arg("login_mode", "basic", "basic = 364C (HTTP Basic Auth), post = M232"),
        _arg("model", "364c", "Camera model: 364c or m232"),
        _arg("camera_config_yaml", "", "Path to a local YAML credential overlay"),

        # ── Control loop ─────────────────────────────────────────────────────
        _arg("poll_hz", "10.0", "IDLE-mode state publish rate (Hz)"),
        _arg("poll_ms", "60", "Active-mode tick period (ms)"),
        _arg("scan_poll_ms", "150", "SCAN_ACTIVE tick period (ms)"),
        _arg("verbose", "true", "Verbose logging"),
        _arg("namespace", "", "ROS namespace for all three nodes"),

        # ── Control token policy ─────────────────────────────────────────────
        _arg(
            "hold_token", "false",
            "Hold the camera control token permanently. Default false releases "
            "it after an idle grace period so a physical JCU operator is not "
            "fought for control. Set true only on rigs with no JCU.",
        ),

        # ── Coordinate frame ─────────────────────────────────────────────────
        _arg(
            "goto_feedback_frame", "abs",
            "Frame used for goto/home arrival detection: abs (default, matches "
            "historical behaviour) or geo. PTGeoAzimuthElevationSet commands in "
            "the GEO frame; the two frames coincide only when the platform "
            "heading offset is zero. On a moving platform, leave this at abs and "
            "watch for the automatic divergence warning, then switch to geo.",
        ),

        # ── Arbitration ──────────────────────────────────────────────────────
        _arg("lease_s", "60.0", "Control-source lease duration (s)"),

        # ── Web console ──────────────────────────────────────────────────────
        _arg("launch_web", "true", "Start the web console"),
        _arg("web_host", "0.0.0.0", "Web console bind address"),
        _arg("web_port", "8080", "Web console port"),
        _arg("mediamtx_api_port", "9997", "mediamtx REST API port"),
        _arg(
            "enable_ffplay", "false",
            "Enable the server-side ffplay launch endpoints. Requires a display "
            "on the server host; off by default.",
        ),

        # ── Joystick bridge ──────────────────────────────────────────────────
        _arg("launch_joy", "true", "Start the joystick bridge"),
        _arg("joy_topic", "/joy", "Joy topic to subscribe (remap for your rig)"),
        _arg("joy_frame_id", "ps5", "Only Joy messages with this frame_id are used"),
    ]

    ns = LaunchConfiguration("namespace")

    ptz_node = Node(
        package="flir_ptz",
        executable="flir_ptz_node",
        name="flir_ptz",
        output="screen",
        namespace=ns,
        # Shutdown homes the camera, which takes up to home_timeout_s. Give the
        # node room to finish so SIGTERM is never needed mid-motion.
        sigterm_timeout="30",
        sigkill_timeout="35",
        parameters=[{
            "host": LaunchConfiguration("host"),
            "username": LaunchConfiguration("username"),
            "password": LaunchConfiguration("password"),
            "login_mode": LaunchConfiguration("login_mode"),
            "model": LaunchConfiguration("model"),
            "camera_config_yaml": LaunchConfiguration("camera_config_yaml"),
            "poll_hz": LaunchConfiguration("poll_hz"),
            "poll_ms": LaunchConfiguration("poll_ms"),
            "scan_poll_ms": LaunchConfiguration("scan_poll_ms"),
            "verbose": LaunchConfiguration("verbose"),
            "hold_token": LaunchConfiguration("hold_token"),
            "goto_feedback_frame": LaunchConfiguration("goto_feedback_frame"),
            "lease_s": LaunchConfiguration("lease_s"),
        }],
    )

    web_node = Node(
        package="flir_ptz",
        executable="flir_ptz_web",
        name="flir_ptz_web",
        output="screen",
        namespace=ns,
        condition=IfCondition(LaunchConfiguration("launch_web")),
        parameters=[{
            "camera_host": LaunchConfiguration("host"),
            "bind_host": LaunchConfiguration("web_host"),
            "port": LaunchConfiguration("web_port"),
            "login_mode": LaunchConfiguration("login_mode"),
            "mediamtx_api_port": LaunchConfiguration("mediamtx_api_port"),
            "enable_ffplay": LaunchConfiguration("enable_ffplay"),
        }],
    )

    joy_node = Node(
        package="flir_ptz",
        executable="flir_joy_bridge",
        name="flir_joy_bridge",
        output="screen",
        namespace=ns,
        condition=IfCondition(LaunchConfiguration("launch_joy")),
        parameters=[{
            "joy_topic": LaunchConfiguration("joy_topic"),
            "frame_id": LaunchConfiguration("joy_frame_id"),
        }],
    )

    return LaunchDescription(args + [ptz_node, web_node, joy_node])
