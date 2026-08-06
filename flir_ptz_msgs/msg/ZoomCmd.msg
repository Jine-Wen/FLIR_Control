# EO (daylight) camera zoom command.
#
# The lens is CONTINUOUS, not stepped: one "in"/"out" keeps the lens moving
# until an explicit "stop" arrives (or the controller's dead-man timer stops
# it for you -- see control/controller.py). IR has no equivalent zoom.
string direction   # "in" | "out" | "stop"
string source       # Control-source id ("" | "web" | "joy"); dropped if source
                     # does not match the current owner, except "stop" which is
                     # always accepted regardless of ownership (non-negotiable
                     # safety rule -- same class as zero-speed joystick/scan-stop).
