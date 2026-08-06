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
