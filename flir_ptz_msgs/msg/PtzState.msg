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
float64 zoom_mm           # EO lens focal length, mm (DLTVLastNMEAGet Zoom); 0 if unknown/IR-only

float64 ir_zoom_pctg     # IR lens zoom, percent of travel, quantised to 6.25 steps
                          # (IRLastNMEAGet Zoom_Pctg -- note capital P, differs from
                          # DLTV's Zoom_pctg); 0 if unknown/no reading yet
float64 ir_fov            # IR lens field of view, degrees (IRLastNMEAGet FOV); 0 if unknown
