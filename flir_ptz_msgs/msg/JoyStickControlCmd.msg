# Joystick direct control command
float64 azimuth_speed    # Azimuth speed command (deg/s)
float64 elevation_speed  # Elevation speed command (deg/s)
string  source             # Control-source id ("" | "web" | "joy"); dropped if source
                            # does not match the current owner, except zero-speed/stop
                            # commands which are always accepted.
