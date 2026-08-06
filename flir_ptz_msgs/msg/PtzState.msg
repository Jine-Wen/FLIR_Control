# PTZ camera state polled from PTLastNMEAGet
std_msgs/Header header

float64 abs_azimuth      # Absolute azimuth (degrees)
float64 abs_elevation    # Absolute elevation (degrees)
float64 geo_azimuth      # Geographic azimuth (degrees)
float64 geo_elevation    # Geographic elevation (degrees)
float64 speed_x          # Current azimuth speed (deg/s)
float64 speed_y          # Current elevation speed (deg/s)
int32   mode             # 0=stopped, 1=auto-scan, other=moving

bool    is_moving        # True while move_to closed-loop is active
bool    is_scanning      # True while auto-scan mode is active
int32   active_move_seq  # Sequence number of current move command
int32   active_scan_seq  # Sequence number of current scan command

float64 zoom_pctg        # EO lens zoom, percent of travel (DLTVLastNMEAGet Zoom_pctg); 0 if unknown/IR-only
float64 zoom_mm           # EO lens field of view, degrees, despite the field's name
                          # (DLTVLastNMEAGet "Zoom" -- measured on a real FLIR 364C to be
                          # the horizontal FOV in degrees, not a focal length in mm);
                          # 0 if unknown/IR-only. Kept for existing consumers; see
                          # zoom_mag for the operator-facing magnification derived from it.

float64 ir_zoom_pctg     # IR lens zoom, percent of travel, quantised to 6.25 steps
                          # (IRLastNMEAGet Zoom_Pctg -- note capital P, differs from
                          # DLTV's Zoom_pctg); 0 if unknown/no reading yet
float64 ir_fov            # IR lens field of view, degrees (IRLastNMEAGet FOV); 0 if unknown

float64 zoom_mag          # EO/VIS zoom magnification relative to the lens's widest FOV
                          # (e.g. 12.4 == "12.4x"), computed from zoom_mm / eo_wide_fov_deg
                          # -- see flir_ptz.control.zoom_optics.magnification(); floored at
                          # 1.0; 0 if no live EO reading yet.
float64 ir_zoom_mag       # IR eZoom magnification relative to ir_wide_fov_deg, same rule
                          # as zoom_mag; 0 if no live IR reading yet.
