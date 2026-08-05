# Absolute pan/tilt move command
float64 target_azimuth    # Target azimuth (degrees, -180 ~ +180, wraps)
float64 target_elevation  # Target elevation (degrees, camera-dependent tilt range)
string  source             # Control-source id ("" | "web" | "joy"); dropped if source
                            # does not match the current owner, except zero-speed/stop
                            # commands which are always accepted.
