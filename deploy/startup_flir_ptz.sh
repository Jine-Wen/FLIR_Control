#!/bin/bash
# startup_flir_ptz.sh — FLIR PTZ system startup script
#
# Functions:
#   1. Check and install dependencies (nginx, apache2-utils)
#   2. Configure nginx + Basic Auth (if not already set up)
#   3. Download mediamtx (if not present)
#   4. Start mediamtx in the background
#
# Usage:
#   bash deploy/startup_flir_ptz.sh [stop]
#   bash deploy/startup_flir_ptz.sh <CAMERA_IP> [MODEL] [AUTH_USER] [AUTH_PASS]
#
# Flags:
#   --transcode        Re-encode the camera's H264 to Baseline with ffmpeg so
#                      browsers can decode it over WebRTC. Needed because the
#                      364C emits High profile 4.0, which Chrome/Edge cannot
#                      receive -- the symptom is a black video pane despite a
#                      successful handshake. Requires ffmpeg.
#   --video-only       Only download and start mediamtx. Skips nginx, Basic
#                      Auth and every sudo step. This is all you need for the
#                      dashboard's WebRTC video; nginx is only for exposing
#                      the dashboard outside the machine.
#   --install-sudoers  Grant passwordless sudo for the nginx/htpasswd steps.
#
#   CAMERA_IP  : Camera IP address (required, no default)
#   MODEL      : 364c (default) or m232
#   AUTH_USER  : nginx Basic Auth username (default: flir)
#   AUTH_PASS  : nginx Basic Auth password (required, no default)
#
# Examples:
#   bash deploy/startup_flir_ptz.sh 192.168.1.50 364c
#   bash deploy/startup_flir_ptz.sh stop

set -e

# Running this with `source`/`.` would make every `exit` below kill the calling
# shell. Detect it and bail out politely instead.
if [ "${BASH_SOURCE[0]}" != "${0}" ]; then
    echo "Run this script, do not source it:" >&2
    echo "    bash deploy/startup_flir_ptz.sh <CAMERA_IP> [MODEL]" >&2
    return 1
fi

# ── Parameters ───────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

# ── Flags (must be parsed before positional arguments) ───────────────────────
INSTALL_SUDOERS=0
VIDEO_ONLY=0
TRANSCODE=0
_args=()
for _a in "$@"; do
    case "$_a" in
        --install-sudoers) INSTALL_SUDOERS=1 ;;
        --video-only)      VIDEO_ONLY=1 ;;
        --transcode)       TRANSCODE=1 ;;
        *)                 _args+=("$_a") ;;
    esac
done
set -- "${_args[@]}"

# ── Stop command ─────────────────────────────────────────────────────────────
if [ "$1" = "stop" ]; then
    if pgrep -x mediamtx &>/dev/null; then
        pkill -x mediamtx && echo "✓ mediamtx stopped"
    else
        echo "! mediamtx is not running"
    fi
    exit 0
fi

# Credentials may also come from the environment, which is the preferred path
# for unattended / systemd deployments.
CAMERA_IP="${1:-${FLIR_HOST:-}}"
MODEL="${2:-${FLIR_MODEL:-364c}}"
AUTH_USER="${3:-${FLIR_AUTH_USER:-}}"
AUTH_PASS="${4:-${FLIR_AUTH_PASS:-}}"

if [ -z "$CAMERA_IP" ]; then
    echo "Error: camera IP is required (there is deliberately no default)." >&2
    echo "  usage: bash deploy/startup_flir_ptz.sh <CAMERA_IP> [MODEL] [AUTH_USER] [AUTH_PASS]" >&2
    echo "  or:    FLIR_HOST=<ip> bash deploy/startup_flir_ptz.sh" >&2
    exit 1
fi

MEDIAMTX_VERSION="v1.9.1"
MEDIAMTX_BIN="$SCRIPT_DIR/mediamtx"
if [ "${TRANSCODE:-0}" = "1" ]; then
    MEDIAMTX_CONFIG="$SCRIPT_DIR/mediamtx-transcode.yml"
else
    MEDIAMTX_CONFIG="$SCRIPT_DIR/mediamtx.yml"
fi
NGINX_CONF_SRC="$SCRIPT_DIR/nginx-flir.conf"
NGINX_CONF_DST="/etc/nginx/sites-available/flir-ptz"
NGINX_ENABLED="/etc/nginx/sites-enabled/flir-ptz"
HTPASSWD="/etc/nginx/.htpasswd"
NGINX_PORT=8443

BOLD="\033[1m"
GREEN="\033[0;32m"
YELLOW="\033[0;33m"
RED="\033[0;31m"
NC="\033[0m"

log()  { echo -e "${BOLD}==> $*${NC}"; }
ok()   { echo -e "${GREEN}    ✓ $*${NC}"; }
warn() { echo -e "${YELLOW}    ! $*${NC}"; }
err()  { echo -e "${RED}    ✗ $*${NC}"; exit 1; }

echo ""
echo -e "${BOLD}════════════════════════════════════════${NC}"
echo -e "${BOLD}  FLIR PTZ Startup Script${NC}"
echo -e "${BOLD}════════════════════════════════════════${NC}"

# ── Interactive credential prompt (only when htpasswd is not yet created) ────
# --video-only never touches nginx, so it must never ask for its password.
if [ "$VIDEO_ONLY" = "1" ]; then
    AUTH_USER="(skipped)"
elif [ ! -f "$HTPASSWD" ]; then
    # First-time setup: prompt for credentials
    if [ -z "$AUTH_USER" ]; then
        read -rp "  Enter web login username [default: flir]: " AUTH_USER
        AUTH_USER="${AUTH_USER:-flir}"
    fi
    if [ -z "$AUTH_PASS" ]; then
        read -rsp "  Enter web login password (required, no default): " AUTH_PASS
        echo ""
        AUTH_PASS="${AUTH_PASS:-}"
    fi
else
    # Already configured; use defaults without overwriting existing credentials
    AUTH_USER="${AUTH_USER:-flir}"
    AUTH_PASS="${AUTH_PASS:-}"
fi

echo "  Camera IP  : $CAMERA_IP"
echo "  Model      : $MODEL"
if [ "$VIDEO_ONLY" = "1" ]; then
    echo "  Mode       : video only (mediamtx; no nginx, no sudo)"
else
    echo "  Auth User  : $AUTH_USER"
    echo "  Nginx Port : $NGINX_PORT"
fi
echo ""

# ─────────────────────────────────────────────────────────────────────────────
# STEP 1: Install dependencies
# ─────────────────────────────────────────────────────────────────────────────
if [ "$VIDEO_ONLY" = "1" ]; then
    log "Steps 1-2/4 — skipped (--video-only)"
else
log "Step 1/4 — Checking dependencies"

PKGS_NEEDED=()
command -v nginx    &>/dev/null || PKGS_NEEDED+=(nginx)
command -v htpasswd &>/dev/null || PKGS_NEEDED+=(apache2-utils)

if [ ${#PKGS_NEEDED[@]} -gt 0 ]; then
    warn "Missing packages: ${PKGS_NEEDED[*]}, installing..."
    sudo apt-get update -qq
    sudo apt-get install -y -qq "${PKGS_NEEDED[@]}"
    ok "Packages installed"
else
    ok "All packages already installed"
fi

# ─────────────────────────────────────────────────────────────────────────────
# STEP 2: Configure nginx + Basic Auth
# ─────────────────────────────────────────────────────────────────────────────
log "Step 2/4 — Configuring Nginx Basic Auth"

# ── Passwordless sudo: EXPLICIT OPT-IN ONLY ─────────────────────────────────
# The previous version of this script silently wrote a NOPASSWD sudoers rule
# on first run. Granting passwordless root for any command is a decision the
# operator must make knowingly, so it now requires --install-sudoers.
#
# Without it the script still works; sudo simply prompts for a password as
# usual, which is the correct default for an interactive run. The rule is only
# worth installing for genuinely unattended deployments.
SUDOERS_FILE="/etc/sudoers.d/flir-auth"
if [ "${INSTALL_SUDOERS:-0}" = "1" ]; then
    if [ ! -f "$SUDOERS_FILE" ]; then
        USER_NAME="$(whoami)"
        warn "Installing NOPASSWD sudoers rule for '$USER_NAME' (requested via --install-sudoers)"
        warn "  scope: htpasswd, systemctl {reload,start,enable} nginx"
        echo "$USER_NAME ALL=(ALL) NOPASSWD: /usr/bin/htpasswd, /bin/systemctl reload nginx, /bin/systemctl start nginx, /bin/systemctl enable nginx" \
            | sudo tee "$SUDOERS_FILE" > /dev/null
        sudo chmod 440 "$SUDOERS_FILE"
        # A malformed sudoers file can lock the machine out of sudo entirely.
        if ! sudo visudo -cf "$SUDOERS_FILE" >/dev/null 2>&1; then
            sudo rm -f "$SUDOERS_FILE"
            err "Generated sudoers rule failed validation and was removed"
        fi
        ok "Sudoers rule created and validated"
    else
        ok "Sudoers rule already present"
    fi
elif [ ! -f "$SUDOERS_FILE" ]; then
    warn "sudo will prompt for a password (pass --install-sudoers to grant NOPASSWD)"
fi

# Create / update htpasswd
if [ ! -f "$HTPASSWD" ]; then
    echo "$AUTH_PASS" | sudo htpasswd -c -i "$HTPASSWD" "$AUTH_USER"
    ok "htpasswd created (user=$AUTH_USER)"
else
    # Add user only if not already present (avoid overwriting existing password)
    if ! sudo grep -q "^${AUTH_USER}:" "$HTPASSWD" 2>/dev/null; then
        echo "$AUTH_PASS" | sudo htpasswd -i "$HTPASSWD" "$AUTH_USER"
        ok "User '$AUTH_USER' added"
    else
        ok "User '$AUTH_USER' already exists, skipping"
    fi
fi

# Install nginx config
if [ ! -f "$NGINX_CONF_SRC" ]; then
    err "nginx config not found: $NGINX_CONF_SRC"
fi
sudo cp "$NGINX_CONF_SRC" "$NGINX_CONF_DST"
sudo ln -sf "$NGINX_CONF_DST" "$NGINX_ENABLED"
sudo rm -f /etc/nginx/sites-enabled/default

# Test and reload nginx
if sudo nginx -t 2>/dev/null; then
    sudo systemctl enable nginx --quiet
    sudo systemctl restart nginx
    ok "Nginx started (port $NGINX_PORT)"
else
    err "Nginx config error, please check $NGINX_CONF_DST"
fi

fi   # end of the nginx/Basic-Auth block skipped by --video-only

# ─────────────────────────────────────────────────────────────────────────────
# STEP 3: Download mediamtx
# ─────────────────────────────────────────────────────────────────────────────
log "Step 3/4 — Checking mediamtx"

if [ ! -f "$MEDIAMTX_BIN" ]; then
    warn "mediamtx not found, downloading $MEDIAMTX_VERSION ..."
    ARCH=$(uname -m)
    case $ARCH in
        x86_64)  ARCH_TAG="linux_amd64" ;;
        aarch64) ARCH_TAG="linux_arm64v8" ;;
        armv7l)  ARCH_TAG="linux_armv7" ;;
        *)       err "Unsupported architecture: $ARCH" ;;
    esac
    URL="https://github.com/bluenviron/mediamtx/releases/download/${MEDIAMTX_VERSION}/mediamtx_${MEDIAMTX_VERSION}_${ARCH_TAG}.tar.gz"
    TMP=$(mktemp -d)
    curl -sL "$URL" -o "$TMP/mediamtx.tar.gz"
    tar -xzf "$TMP/mediamtx.tar.gz" -C "$TMP"
    mv "$TMP/mediamtx" "$MEDIAMTX_BIN"
    chmod +x "$MEDIAMTX_BIN"
    rm -rf "$TMP"
    ok "mediamtx downloaded"
else
    ok "mediamtx already exists"
fi

# ─────────────────────────────────────────────────────────────────────────────
# STEP 4: Start mediamtx
# ─────────────────────────────────────────────────────────────────────────────
log "Step 4/4 — Starting mediamtx"

# Generate a temporary config with the correct camera IP
TMP_CONFIG=$(mktemp /tmp/mediamtx_XXXX.yml)
sed "s|CAMERA_IP|$CAMERA_IP|g" "$MEDIAMTX_CONFIG" > "$TMP_CONFIG"

if [ "$MODEL" = "m232" ]; then
    sed -i "s|rtsp://${CAMERA_IP}:8554/ir\.0|rtsp://${CAMERA_IP}:8554/ir|g" "$TMP_CONFIG"
    sed -i "s|source: rtsp://${CAMERA_IP}:8554/vis\.0|source: disable|g" "$TMP_CONFIG"
    ok "M232 mode: IR=rtsp://${CAMERA_IP}:8554/ir  EO=disabled"
else
    ok "364C mode: IR=rtsp://${CAMERA_IP}:8554/ir.0  EO=rtsp://${CAMERA_IP}:8554/vis.0"
fi

# Stop any existing mediamtx instance before starting a new one
if pgrep -x mediamtx &>/dev/null; then
    warn "Existing mediamtx detected, stopping..."
    pkill -x mediamtx || true
    sleep 1
fi

HOST_IP=$(hostname -I | awk '{print $1}')

echo ""
echo ""
if [ "$VIDEO_ONLY" = "1" ]; then
    echo -e "${BOLD}  Video endpoints (browser connects to these directly):${NC}"
    echo    "    WebRTC IR   http://${HOST_IP}:8889/ir"
    [ "$MODEL" != "m232" ] && \
    echo    "    WebRTC EO   http://${HOST_IP}:8889/eo"
    echo    "    mediamtx API http://127.0.0.1:9997/v3/paths/list"
    echo ""
    echo -e "${YELLOW}    Streams are pulled on demand: they read 'ready=false'${NC}"
    echo -e "${YELLOW}    until a viewer actually opens them. That is normal.${NC}"
else
    echo -e "${BOLD}  Service URLs:${NC}"
    echo    "    Dashboard (nginx)  http://${HOST_IP}:${NGINX_PORT}/"
    echo    "    Dashboard (direct) http://${HOST_IP}:8080/"
    echo    "    WebRTC IR          http://${HOST_IP}:8889/ir"
    [ "$MODEL" != "m232" ] && \
    echo    "    WebRTC EO          http://${HOST_IP}:8889/eo"
    echo    "    Auth user          ${AUTH_USER}"
fi
echo ""

ok "Starting mediamtx in background..."
nohup "$MEDIAMTX_BIN" "$TMP_CONFIG" > /tmp/mediamtx.log 2>&1 &
MEDIAMTX_PID=$!
echo "$MEDIAMTX_PID" > /tmp/mediamtx.pid

sleep 1
if kill -0 "$MEDIAMTX_PID" 2>/dev/null; then
    ok "mediamtx started (PID=$MEDIAMTX_PID, log=/tmp/mediamtx.log)"
else
    err "mediamtx failed to start, check /tmp/mediamtx.log"
fi

echo ""
echo -e "${GREEN}${BOLD}  ✓ FLIR PTZ system started successfully!${NC}"
echo ""
