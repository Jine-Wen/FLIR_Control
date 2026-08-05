# Auto-scan command
float64 center_azimuth   # Scan center azimuth (degrees)
float64 each_side_deg    # Sweep range per side (degrees, e.g. 15 = ±15°)
float64 elevation        # Elevation to hold during scan (degrees)
float64 speed            # Scan speed (deg/s)
bool    stop             # true=stop scan, false=start scan
string  source             # Control-source id ("" | "web" | "joy"); dropped if source
                            # does not match the current owner, except stop commands
                            # which are always accepted.
