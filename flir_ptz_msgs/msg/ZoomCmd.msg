# EO (daylight) or IR (thermal) camera zoom command.
#
# Both lenses are CONTINUOUS, not stepped: one "in"/"out" keeps the lens
# moving until an explicit "stop" arrives (or the controller's dead-man
# timer stops it for you -- see control/controller.py). EO and IR are
# zoomed and stopped completely independently of each other.
string direction   # "in" | "out" | "stop"
string device       # "eo" | "ir" -- which lens to zoom. Absent or unrecognised
                     # (including empty, for existing publishers that predate
                     # this field) defaults to "eo".
string source       # Control-source id ("" | "web" | "joy"); dropped if source
                     # does not match the current owner, except "stop" which is
                     # always accepted regardless of ownership (non-negotiable
                     # safety rule -- same class as zero-speed joystick/scan-stop),
                     # for both devices.
