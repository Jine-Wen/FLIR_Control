#!/usr/bin/env python3
"""Pure zoom-magnification math. No I/O, no rclpy, no camera access -- just
turning a horizontal-field-of-view reading into the "Nx" number an operator
actually thinks in.

MEASURED ON A REAL FLIR 364C (see the worker brief): both the EO/VIS
``DLTVLastNMEAGet`` ``Zoom`` field and the IR ``IRLastNMEAGet`` ``FOV`` field
report the lens's current HORIZONTAL FIELD OF VIEW IN DEGREES, *not* a focal
length -- despite ``Zoom`` sounding like one, and despite ``PtzState.zoom_mm``
(carrying that same field) being named as if it were millimetres. FOV shrinks
as the lens zooms in, so magnification is simply the ratio of the widest FOV
the lens can show to whatever FOV it is showing right now::

    EO / VIS:  widest  Zoom = 63.70 deg (Zoom_pctg   0)
               tele    Zoom =  2.12 deg (Zoom_pctg 100)   -> ~30.0x range
    IR eZoom:  widest  FOV  = 18.00 deg (Zoom_Pctg   0, Electronic_zoom 0)
               tele    FOV  =  8.62 deg (Zoom_Pctg 53.12, Electronic_zoom 2)
                                                            -> ~2.09x range
"""

from __future__ import annotations

from typing import Optional

# Widest (zoomed all the way out) horizontal FOV, in degrees, measured on a
# real FLIR 364C. A different camera model (e.g. an M232) will almost
# certainly measure differently -- that is exactly why both are exposed as
# ROS parameters on the PTZ node (`eo_wide_fov_deg` / `ir_wide_fov_deg`)
# rather than being hardcoded as the only option; these are just the
# defaults for the hardware this was verified against.
EO_WIDE_FOV_DEG = 63.7
IR_WIDE_FOV_DEG = 18.0


def magnification(wide_fov: float, current_fov: Optional[float]) -> float:
    """Zoom factor relative to the widest FOV, floored at 1.0.

    ``1.0`` (never less -- "no zoom" is the floor, not a fraction) is
    returned for every input that isn't a genuine, in-range FOV reading:
    ``current_fov`` missing (``None``), zero, negative, or larger than
    ``wide_fov`` (a bad reading or a wide_fov that doesn't actually match
    this lens). This is deliberately conservative -- a divide-by-zero
    exception or a nonsensical "0.4x" reading reaching the UI is worse than
    just reporting no magnification at all.
    """
    if current_fov is None or current_fov <= 0 or current_fov > wide_fov or wide_fov <= 0:
        return 1.0
    return wide_fov / current_fov


class WideFovCalibrator:
    """Learns a lens's widest FOV from the camera instead of assuming it.

    The camera reports its live field of view and its zoom position as a
    percentage, and 0% *is* the wide end by definition. So the reference this
    module needs is not really a constant to be configured -- it is a reading
    the camera will hand over the first time the operator zooms out, on any
    model, with no measurement or setup.

    Until that happens there is nothing to learn from, so the configured
    default stands in. The running maximum is used in between: it can only
    ever be an underestimate of the true wide FOV (magnification would read
    slightly low), never an overestimate, so it is safe to prefer over a
    default that might belong to a different camera entirely.

    Once a genuine wide-end reading arrives it wins outright and is not
    revised by anything narrower, which is what stops a mid-zoom sample from
    quietly redefining the reference.
    """

    #: A zoom percentage at or below this counts as "fully wide". Not exactly
    #: zero: the reading is a float from real hardware and lands on values
    #: like 0.0 or 3.96 depending on where the lens actually stopped.
    WIDE_PCTG_THRESHOLD = 1.0

    def __init__(self, fallback_deg: float, wide_pctg_threshold: Optional[float] = None) -> None:
        self._fallback = float(fallback_deg)
        self._threshold = (
            self.WIDE_PCTG_THRESHOLD if wide_pctg_threshold is None else float(wide_pctg_threshold)
        )
        self._learned: Optional[float] = None
        self._max_seen: Optional[float] = None

    def observe(self, fov: Optional[float], zoom_pctg: Optional[float]) -> None:
        """Feed one live reading. Cheap enough to call on every sample."""
        if fov is None or fov <= 0:
            return
        if self._max_seen is None or fov > self._max_seen:
            self._max_seen = fov
        if zoom_pctg is not None and zoom_pctg <= self._threshold:
            self._learned = fov

    @property
    def wide_fov(self) -> float:
        if self._learned is not None:
            return self._learned
        if self._max_seen is not None and self._max_seen > self._fallback:
            return self._max_seen
        return self._fallback

    @property
    def source(self) -> str:
        """Where the current reference came from, for logging: the operator
        should be able to tell a measured value from a fallback."""
        if self._learned is not None:
            return "camera"
        if self._max_seen is not None and self._max_seen > self._fallback:
            return "observed"
        return "default"

    def magnification(self, current_fov: Optional[float]) -> float:
        return magnification(self.wide_fov, current_fov)
