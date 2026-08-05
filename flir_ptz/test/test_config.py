#!/usr/bin/env python3
"""Tests for flir_ptz.control.config.

Must run offline with nothing but the Python 3.12 standard library plus
(optionally) PyYAML, which is expected to be installed in this environment
but must remain optional at import time.

IMPORTANT: no real IP address or password may appear anywhere in this file
(spec sec. 8, rule 1). All hosts/users/passwords below are obviously fake
placeholders.
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from flir_ptz.control.config import CameraConfig, ControlConfig, load_camera_config


# ---------------------------------------------------------------------------
# defaults — must never be a real credential
# ---------------------------------------------------------------------------

def test_camera_config_defaults_are_empty_strings():
    cfg = CameraConfig()
    assert cfg.host == ""
    assert cfg.username == ""
    assert cfg.password == ""
    assert cfg.login_mode == "basic"
    assert cfg.model == "364c"


def test_camera_config_is_frozen():
    cfg = CameraConfig()
    try:
        cfg.host = "example.invalid"  # type: ignore[misc]
        assert False, "CameraConfig should be frozen"
    except Exception:
        pass


def test_load_camera_config_all_empty_by_default():
    cfg = load_camera_config(env={}, yaml_path=None, overrides={})
    assert cfg.host == ""
    assert cfg.username == ""
    assert cfg.password == ""
    assert cfg.login_mode == "basic"
    assert cfg.model == "364c"


def test_control_config_defaults_sane():
    cfg = ControlConfig()
    assert cfg.poll_hz == 10.0
    assert cfg.poll_ms == 60
    assert cfg.scan_poll_ms == 150
    assert cfg.az_tol == 0.5
    assert cfg.el_tol == 0.5
    assert cfg.az_hold_tol == 0.7
    assert cfg.el_hold_tol == 0.7
    assert cfg.settle_samples == 4
    assert cfg.home_az == 0.0
    assert cfg.home_el == -90.0


# ---------------------------------------------------------------------------
# precedence: YAML < env < overrides
# ---------------------------------------------------------------------------

def test_env_overrides_yaml(tmp_path):
    yaml_path = tmp_path / "camera.yaml"
    yaml_path.write_text(
        "host: yaml-host.example.test\n"
        "username: yaml-user\n"
        "password: yaml-pass-PLACEHOLDER\n"
    )
    env = {"FLIR_HOST": "env-host.example.test"}

    cfg = load_camera_config(env=env, yaml_path=yaml_path, overrides={})

    assert cfg.host == "env-host.example.test"       # env wins over yaml
    assert cfg.username == "yaml-user"                # untouched by env -> falls through
    assert cfg.password == "yaml-pass-PLACEHOLDER"    # untouched by env -> falls through


def test_overrides_beat_env_and_yaml(tmp_path):
    yaml_path = tmp_path / "camera.yaml"
    yaml_path.write_text("host: yaml-host.example.test\nusername: yaml-user\n")
    env = {"FLIR_HOST": "env-host.example.test", "FLIR_USERNAME": "env-user"}
    overrides = {"host": "override-host.example.test"}

    cfg = load_camera_config(env=env, yaml_path=yaml_path, overrides=overrides)

    assert cfg.host == "override-host.example.test"  # override wins
    assert cfg.username == "env-user"                  # env wins (no override given)


def test_full_precedence_chain(tmp_path):
    yaml_path = tmp_path / "camera.yaml"
    yaml_path.write_text(
        "host: yaml-host.example.test\n"
        "username: yaml-user\n"
        "password: yaml-pass-PLACEHOLDER\n"
        "model: 364c\n"
    )
    env = {
        "FLIR_HOST": "env-host.example.test",
        "FLIR_PASSWORD": "env-pass-PLACEHOLDER",
    }
    overrides = {"username": "override-user"}

    cfg = load_camera_config(env=env, yaml_path=yaml_path, overrides=overrides)

    assert cfg.host == "env-host.example.test"          # yaml < env
    assert cfg.username == "override-user"                # yaml < override
    assert cfg.password == "env-pass-PLACEHOLDER"          # yaml < env
    assert cfg.model == "364c"                             # only in yaml


def test_yaml_only_partial_fields():
    # No yaml_path at all -> only env/overrides apply, rest stay default empty.
    cfg = load_camera_config(env={}, yaml_path=None, overrides={"host": "override-only.example.test"})
    assert cfg.host == "override-only.example.test"
    assert cfg.username == ""
    assert cfg.password == ""


def test_missing_yaml_file_is_ignored(tmp_path):
    missing = tmp_path / "does-not-exist.yaml"
    cfg = load_camera_config(env={"FLIR_HOST": "env-host.example.test"}, yaml_path=missing, overrides={})
    assert cfg.host == "env-host.example.test"


def test_empty_env_value_does_not_override_yaml(tmp_path):
    yaml_path = tmp_path / "camera.yaml"
    yaml_path.write_text("host: yaml-host.example.test\n")
    env = {"FLIR_HOST": ""}  # present but empty -> should not clobber yaml value

    cfg = load_camera_config(env=env, yaml_path=yaml_path, overrides={})
    assert cfg.host == "yaml-host.example.test"


def test_empty_override_value_does_not_clobber_lower_layers():
    env = {"FLIR_HOST": "env-host.example.test"}
    overrides = {"host": ""}

    cfg = load_camera_config(env=env, yaml_path=None, overrides=overrides)
    assert cfg.host == "env-host.example.test"


def test_login_mode_and_model_only_settable_via_yaml_or_overrides():
    # Spec only defines env vars for host/username/password/model — not
    # login_mode. login_mode can still be set via yaml or explicit overrides.
    overrides = {"login_mode": "post", "model": "m232"}
    cfg = load_camera_config(env={}, yaml_path=None, overrides=overrides)
    assert cfg.login_mode == "post"
    assert cfg.model == "m232"


def test_model_env_var():
    cfg = load_camera_config(env={"FLIR_MODEL": "m232"}, yaml_path=None, overrides={})
    assert cfg.model == "m232"


def test_non_mapping_yaml_content_is_ignored(tmp_path):
    yaml_path = tmp_path / "camera.yaml"
    yaml_path.write_text("- just\n- a\n- list\n")
    cfg = load_camera_config(env={}, yaml_path=yaml_path, overrides={})
    assert cfg.host == ""


# ---------------------------------------------------------------------------
# no real credentials anywhere in this module's defaults
# ---------------------------------------------------------------------------

def test_defaults_carry_no_credential_material():
    # Every credential-bearing field must default to the empty string —
    # never a baked-in host/IP/password (spec sec. 8, rule 1).
    cfg = CameraConfig()
    assert cfg.host == ""
    assert cfg.username == ""
    assert cfg.password == ""
    assert all(len(getattr(cfg, f)) == 0 for f in ("host", "username", "password"))
    assert cfg.host == ""
