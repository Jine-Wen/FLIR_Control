#!/usr/bin/env python3
"""Guards for the contract the web markup/stylesheet owes app.js.

These are not style opinions. Each rule below is load-bearing for behaviour,
and each has already broken once by simply going missing during a redesign --
silently, because CSS has no errors: the page renders, nothing logs, and a
feature just stops working.

Pure stdlib; no ROS, no browser.
"""

from __future__ import annotations

import os
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.dirname(_HERE)
_WEB = os.path.join(_PKG_ROOT, "flir_ptz", "web")

# Must be self-sufficient. Without this the file only passed when some other
# test module happened to be imported first and had already put the package
# root on sys.path -- so running this file alone failed, and its guards were
# silently order-dependent.
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

import pytest  # noqa: E402


def _read(name: str) -> str:
    with open(os.path.join(_WEB, name), encoding="utf-8") as fh:
        return fh.read()


CSS = _read("styles.css")
INDEX = _read("index.html")
SETUP = _read("setup.html")
APP_JS = _read("app.js")
SETUP_JS = _read("setup.js")


#: Stylesheet with /* comments */ stripped. Assertions must inspect declarations
#: only -- a rule's own explanatory comment mentioning a property would
#: otherwise satisfy a check for that property.
CSS_CODE = re.sub(r"/\*.*?\*/", "", CSS, flags=re.S)


def _rule_body(selector: str) -> str:
    """Declarations of the first rule whose selector matches exactly."""
    pattern = re.compile(
        r"(?:^|\})\s*" + re.escape(selector) + r"\s*\{([^}]*)\}", re.M
    )
    m = pattern.search(CSS_CODE)
    assert m, f"stylesheet has no rule for `{selector}`"
    return m.group(1)


# ── every id/hook app.js resolves must exist in the markup ───────────────────
#
# app.js builds its `ui` map once at load. A missing id yields null, and the
# feature that uses it silently no-ops -- which is how the camera-disconnected
# banner was nearly shipped broken.

def test_every_id_app_js_resolves_exists_in_the_markup():
    html = INDEX + SETUP
    js = APP_JS + SETUP_JS
    defined = set(re.findall(r'id="([A-Za-z0-9_-]+)"', html))

    wanted = set(re.findall(r'\$\("([A-Za-z0-9_-]+)"\)', js))
    wanted |= set(re.findall(r'getElementById\("([A-Za-z0-9_-]+)"\)', js))
    for suffix in re.findall(r"\$\{stream\}([A-Za-z]+)", js):
        wanted |= {"ir" + suffix, "eo" + suffix}

    # Resolved by attribute selector, and diffStyle cache keys that are not
    # element ids at all.
    wanted -= {"irRetry", "eoRetry", "irTransportColor", "eoTransportColor"}

    missing = sorted(w for w in wanted if w not in defined)
    assert not missing, f"markup is missing ids app.js resolves: {missing}"


def test_attribute_hooks_app_js_queries_exist():
    for sel in ('data-launch="ir"', 'data-launch="eo"',
                'data-stop="ir"', 'data-stop="eo"',
                'data-retry="ir"', 'data-retry="eo"'):
        assert sel in INDEX, f"markup lost the {sel} hook"
    assert "btn-text" in SETUP, "setup.js queries .btn-text"


def test_css_defines_every_custom_property_app_js_writes():
    used = set(re.findall(r"var\((--[a-z-]+)\)", APP_JS))
    for var in used:
        assert f"{var}:" in CSS, f"app.js writes {var} but the stylesheet never defines it"


#: Classes the scripts add/remove/toggle at runtime. Listed explicitly rather
#: than scraped: `classList.add(reason === "remote" ? "lock-remote" : ...)`
#: contains string literals that are comparison operands, not class names, and
#: a scraper cannot tell them apart.
TOGGLED_CLASSES = [
    "ctrl-locked",      # panels greyed out while another source drives
    "lock-self",        # joystick locked, awaiting the unlock gesture
    "lock-remote",      # joystick locked because the pad took control
    "lock-open",        # joystick unlocked
    "single-pane",      # EO hidden (M232), IR spans the grid
    "loading",          # setup page connect button spinner
    "status-box",       # setup page result banner
    "show",             # ...and its visible state
    "error", "success", "info",
]


@pytest.mark.parametrize("name", TOGGLED_CLASSES)
def test_class_app_js_toggles_is_styled(name):
    assert f".{name}" in CSS_CODE, f"the scripts toggle .{name} but it has no styling"


def test_toggled_class_list_is_still_complete():
    """Catches a newly-toggled class that nobody added to TOGGLED_CLASSES."""
    scraped = set()
    for group in re.findall(r'classList\.(?:add|remove|toggle)\(([^)]*)\)', APP_JS + SETUP_JS):
        scraped.update(re.findall(r'"([a-zA-Z0-9_-]+)"', group))
    # Ternary operands that are values, not class names.
    scraped -= {"remote", "self"}
    unknown = sorted(scraped - set(TOGGLED_CLASSES))
    assert not unknown, f"new runtime classes not covered by these guards: {unknown}"


# ── geometry the joystick's drag maths depends on ────────────────────────────
#
# app.js computes the drag radius from getBoundingClientRect().width
# (`rect.width * 0.38`) and applies it to BOTH axes. A control that is not
# square desynchronises pointer input from what is drawn.

def test_joystick_is_sized_square():
    body = _rule_body(".joystick")
    assert "aspect-ratio: 1" in body, "joystick must be square"
    assert "max-height" not in body, (
        "max-height caps one axis only, so the element stops being square "
        "while app.js still derives the radius from its width"
    )
    assert re.search(r"width:\s*min\(", body), (
        "joystick width must be bounded directly; a percentage width lets the "
        "sidebar width dictate the drag radius"
    )


def test_joystick_and_stick_suppress_text_selection():
    """Without this a drag starts a text selection, which fights the pointer
    capture and makes the control unusable."""
    for selector in (".joystick", ".stick"):
        assert "user-select: none" in _rule_body(selector), (
            f"{selector} must set user-select: none"
        )


def test_joystick_accepts_touch_drags():
    assert "touch-action: none" in _rule_body(".joystick")


# ── the EO pane must lay out identically to IR ───────────────────────────────

def test_eo_wrapper_is_display_contents():
    """#eoPaneWrap exists only to give app.js something to toggle `hidden` on.

    It must vanish from layout, or IT becomes the grid item -- stretching to
    the row height while the <article> inside keeps its content height, so the
    EO pane renders visibly shorter than the unwrapped IR pane.
    """
    assert "display: contents" in _rule_body("#eoPaneWrap")


def test_eo_wrapper_still_hides_when_hidden():
    """`display: contents` overrides the browser default for [hidden], so the
    hidden state needs restating -- otherwise switching to an M232 (no EO
    stream) leaves the empty pane on screen."""
    assert "display: none" in _rule_body("#eoPaneWrap[hidden]")


def test_hidden_buttons_actually_hide():
    """The base `button { display: inline-flex }` rule outranks the UA's
    `[hidden] { display: none }` at equal specificity, so the WebRTC retry
    button would render even while marked hidden."""
    assert "display: none" in _rule_body("button[hidden]")


# ── palette legibility ───────────────────────────────────────────────────────

def _luminance(hex_colour: str) -> float:
    h = hex_colour.lstrip("#")
    channels = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    channels = [c / 12.92 if c <= 0.03928 else ((c + 0.055) / 1.055) ** 2.4 for c in channels]
    return 0.2126 * channels[0] + 0.7152 * channels[1] + 0.0722 * channels[2]


def _contrast(a: str, b: str) -> float:
    la, lb = _luminance(a), _luminance(b)
    hi, lo = max(la, lb), min(la, lb)
    return (hi + 0.05) / (lo + 0.05)


@pytest.mark.parametrize("token", ["--muted", "--cyan", "--green", "--red", "--amber"])
def test_semantic_colours_meet_wcag_aa_on_both_surfaces(token):
    """app.js writes these straight into inline text colour, so they must be
    legible on every surface the console uses. They were originally chosen for
    a near-black background and did not survive the move to a light one."""
    values = dict(re.findall(r"(--[a-z0-9-]+):\s*(#[0-9A-Fa-f]{6})", CSS))
    colour = values[token]
    for surface in ("--panel", "--bg"):
        ratio = _contrast(colour, values[surface])
        assert ratio >= 4.5, (
            f"{token} ({colour}) on {surface} ({values[surface]}) "
            f"is {ratio:.2f}:1, below the 4.5:1 AA threshold"
        )


# ── the unlock gesture must remain reachable while locked ────────────────────
#
# The worst regression of the redesign, and the least visible: setJoyLock(true)
# marks EVERY panel ctrl-locked at boot, including #joyPanel — yet the only way
# to unlock the console is to draw a circle ON the joystick. Letting #joyPanel
# inherit `pointer-events: none` makes that gesture impossible, so nothing ever
# unlocks and the whole control sidebar is dead for the session. No error, no
# log, no visual cue that anything is wrong.


def test_locked_joystick_panel_still_accepts_pointer_events():
    body = _rule_body("#joyPanel.ctrl-locked")
    assert "pointer-events: auto" in body, (
        "#joyPanel must stay interactive while locked, or the circle-to-unlock "
        "gesture can never be performed and the console deadlocks"
    )


def test_generic_panel_lock_does_block_pointer_events():
    """The exception above only means anything if the general rule still bites."""
    assert "pointer-events: none" in _rule_body(".panel.ctrl-locked")


def test_locked_joystick_panel_still_disables_its_inputs():
    """Interactive ring, inert controls: the speed slider must not be
    adjustable while the console is locked."""
    assert "pointer-events: none" in _rule_body("#joyPanel.ctrl-locked .field")


def test_joystick_panel_is_not_dimmed_while_locked():
    """It is the affordance the operator must find and use; dimming it to .5
    alongside the dead panels hides the one thing that still works."""
    assert "opacity: 1" in _rule_body("#joyPanel.ctrl-locked")


def test_every_panel_setjoylock_touches_is_locked_by_the_same_class():
    """setJoyLock adds .ctrl-locked to CTRL_PANELS plus #joyPanel explicitly.
    If a new panel is added to that list, it needs styling too."""
    m = re.search(r"const CTRL_PANELS\s*=\s*\[([^\]]*)\]", APP_JS)
    assert m, "app.js no longer declares CTRL_PANELS"
    panels = re.findall(r'"([A-Za-z0-9_-]+)"', m.group(1))
    assert panels, "CTRL_PANELS is empty"
    for pid in panels:
        assert f'id="{pid}"' in INDEX, f"CTRL_PANELS names #{pid}, absent from the markup"
    assert 'id="joyPanel"' in INDEX


# ── the two circle detectors must stay in step ───────────────────────────────
#
# The same gesture has to unlock the console whether it is drawn with a mouse
# (app.js) or a physical stick (control/gestures.py). Nothing but a comment
# enforced that, and the two were free to drift apart silently.

def _js_number(name: str) -> float:
    m = re.search(rf"const\s+{name}\s*=\s*([0-9.]+)", APP_JS)
    assert m, f"app.js no longer defines {name}"
    return float(m.group(1))


def test_circle_thresholds_match_the_python_twin():
    from flir_ptz.control import gestures

    assert _js_number("CIRCLE_MIN_R") == gestures.CIRCLE_MIN_R
    assert _js_number("CIRCLE_RESET_R") == gestures.CIRCLE_RESET_R


def test_neither_detector_uses_a_sliding_window_or_sample_floor():
    """The defect both shared: deltas summed over a fixed-size window, plus a
    minimum sample count. Together they left only a narrow band of drawing
    speeds that worked -- too fast was rejected outright, too slow could never
    accumulate a full turn because early samples were discarded faster than new
    ones arrived."""
    import inspect
    from flir_ptz.control import gestures

    py = inspect.getsource(gestures.CircleDetector)
    assert "_history" not in py, "python detector went back to a sample window"
    assert "< 20" not in py, "python detector went back to a sample floor"

    js = re.search(r"function detectCircle\([^)]*\)\s*\{.*?\n\}", APP_JS, re.S)
    assert js, "app.js no longer defines detectCircle"
    body = js.group(0)
    assert "_circleHistory" not in body, "js detector went back to a sample window"
    assert "length < 20" not in body, "js detector went back to a sample floor"


def test_panels_cannot_be_squashed_by_the_flex_sidebar():
    """app.js derives the joystick's drag radius from its rendered width. If a
    height-constrained flex column shrinks the panel, rendered size stops
    matching what the stylesheet asked for."""
    assert "flex-shrink: 0" in _rule_body(".panel")
