#!/usr/bin/env bash
# Source this before launching flir_ptz:
#
#     source deploy/ros_env.sh
#     ros2 launch flir_ptz flir_ptz.launch.py host:=... username:=... password:=...
#
# Why this exists
# ---------------
# flir_ptz runs three nodes that talk over ordinary ROS topics and services.
# On some hosts -- WSL2 in particular -- the default Fast DDS middleware fails
# to discover participants ACROSS PROCESSES, even though multicast, shared
# memory and hostname resolution all test fine, and even though pub/sub inside
# a single process works. `ros2 topic list` shows nothing and `ros2 node list`
# reports no nodes at all.
#
# The visible symptom in this project is confusing, because the web dashboard
# still loads and its own HTTP endpoints answer normally: the setup page just
# reports "Connection timed out. Check camera IP/credentials and try again."
# even with a perfectly good camera IP and password, and it never
# auto-redirects to /control despite the PTZ node being connected. Nothing is
# wrong with the credentials or the camera -- the web node simply never
# receives the PTZ node's camera_status message.
#
# Switching the middleware to Zenoh fixes it. Zenoh needs a router process
# (rmw_zenohd) running alongside the nodes; without it, discovery fails in
# exactly the same way. This script starts one if it is not already up.
#
# On a native Ubuntu machine the default Fast DDS normally works fine and you
# do not need this -- verify in 30 seconds with:
#     ros2 run demo_nodes_cpp talker &
#     ros2 topic echo /chatter --once
# If a message arrives, skip this script entirely.

if [ -z "${ROS_DISTRO:-}" ]; then
    echo "ros_env.sh: source /opt/ros/<distro>/setup.bash first" >&2
    return 1 2>/dev/null || exit 1
fi

export RMW_IMPLEMENTATION=rmw_zenoh_cpp

_zenohd="/opt/ros/${ROS_DISTRO}/lib/rmw_zenoh_cpp/rmw_zenohd"

if [ ! -x "$_zenohd" ]; then
    echo "ros_env.sh: rmw_zenoh_cpp not found — install it with:" >&2
    echo "    sudo apt install ros-${ROS_DISTRO}-rmw-zenoh-cpp" >&2
    return 1 2>/dev/null || exit 1
fi

if pgrep -x rmw_zenohd > /dev/null 2>&1; then
    echo "ros_env.sh: RMW=rmw_zenoh_cpp, router already running"
else
    nohup "$_zenohd" > /tmp/rmw_zenohd.log 2>&1 &
    sleep 2
    if pgrep -x rmw_zenohd > /dev/null 2>&1; then
        echo "ros_env.sh: RMW=rmw_zenoh_cpp, router started (log: /tmp/rmw_zenohd.log)"
    else
        echo "ros_env.sh: router failed to start — see /tmp/rmw_zenohd.log" >&2
    fi
fi

# Every process in the ROS graph must agree on the middleware. If you open a
# new terminal to run `ros2 topic echo` or `joy_node`, source this file there
# too, or those commands will silently see an empty graph.
