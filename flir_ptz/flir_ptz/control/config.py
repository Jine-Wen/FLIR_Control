#!/usr/bin/env python3
"""Pure configuration dataclasses and camera-credential precedence resolution.

No I/O side effects beyond an optional, lazily-imported YAML file read. No
rclpy, no httpx. Must import cleanly even when PyYAML is not installed.

Hard rule (spec sec. 8): no default value anywhere in this module may be a
real IP address or password. All ``CameraConfig`` credential fields default
to the empty string.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional


@dataclass(frozen=True)
class ControlConfig:
    """All tolerances, timeouts, and tick periods for the motion control plane."""

    poll_hz: float = 10.0
    poll_ms: int = 60
    scan_poll_ms: int = 150

    az_tol: float = 0.5
    el_tol: float = 0.5
    az_hold_tol: float = 0.7
    el_hold_tol: float = 0.7
    settle_samples: int = 4

    track_idle_s: float = 1.5
    velocity_idle_s: float = 1.5

    goto_grace_s: float = 1.5
    stall_timeout_s: float = 4.0
    stall_progress_deg: float = 0.05

    home_az: float = 0.0
    home_el: float = -90.0
    home_timeout_s: float = 20.0

    # -- carried over from the old launch parameters --
    timeout_s: float = 45.0
    interrupt_scan_wait_s: float = 0.15
    stop_scan_timeout_s: float = 5.0
    scan_save_wait_s: float = 0.15
    scan_start_wait_s: float = 0.3
    verbose: bool = True


@dataclass(frozen=True)
class CameraConfig:
    """Camera connection credentials. Every default is an empty string —
    never a real IP address or password (spec sec. 8, rule 1)."""

    host: str = ""
    username: str = ""
    password: str = ""
    login_mode: str = "basic"  # "basic" = 364C (HTTP Basic Auth) | "post" = M232
    model: str = "364c"


_CAMERA_FIELDS = ("host", "username", "password", "login_mode", "model")

_ENV_KEYS = {
    "host": "FLIR_HOST",
    "username": "FLIR_USERNAME",
    "password": "FLIR_PASSWORD",
    "model": "FLIR_MODEL",
}


def _read_yaml_dict(path: Path) -> Optional[dict]:
    """Best-effort YAML read. Returns None if the file is missing, unreadable,
    not a mapping, or PyYAML is not installed. Import of ``yaml`` is lazy so
    this module remains importable without PyYAML present."""
    try:
        import yaml  # type: ignore
    except ImportError:
        return None

    try:
        if not path.exists():
            return None
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except Exception:
        return None

    return data if isinstance(data, dict) else None


def load_camera_config(
    env: Mapping[str, str],
    yaml_path: Optional[Path],
    overrides: dict,
) -> CameraConfig:
    """Resolve camera credentials with precedence (low to high):

        YAML file  <  environment variables  <  explicit overrides

    At each layer, a field is only overwritten by a non-empty value — an
    unset/empty entry at a higher-precedence layer leaves the
    lower-precedence value in place, which is what makes "partial override"
    at any single layer behave sensibly.
    """
    data: dict[str, Any] = {}

    # -- layer 1: YAML file (lowest precedence) --
    if yaml_path is not None:
        yaml_data = _read_yaml_dict(Path(yaml_path))
        if yaml_data:
            for key in _CAMERA_FIELDS:
                value = yaml_data.get(key)
                if value is not None and str(value) != "":
                    data[key] = str(value)

    # -- layer 2: environment variables --
    if env:
        for key, env_key in _ENV_KEYS.items():
            value = env.get(env_key)
            if value:
                data[key] = value

    # -- layer 3: explicit overrides (highest precedence) --
    if overrides:
        for key in _CAMERA_FIELDS:
            if key not in overrides:
                continue
            value = overrides[key]
            if value is not None and value != "":
                data[key] = value

    return CameraConfig(
        host=data.get("host", ""),
        username=data.get("username", ""),
        password=data.get("password", ""),
        login_mode=data.get("login_mode", "basic"),
        model=data.get("model", "364c"),
    )
