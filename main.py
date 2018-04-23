import holdposition
from GPSWrapper import thread as gps
import DroneServerCom
import enum
import time


class Mode(enum.Enum):
    LANDED = 1
    TAKEOFF = 2
    CRUISING = 3
    LANDING = 4

mode = Mode.LANDED

while True:
    if DroneServerCom.has_waypoint():
        if mode == Mode.LANDED:
            # TODO: Takeoff
            pass
        elif mode == Mode.CRUISING:
            lat, lon, alt = DroneServerCom.get_waypoint()
            holdposition.set_target(lat, lon)
            # TODO: Check if close to target, and land if so
    elif mode != Mode.LANDED:
        # TODO: Handle this error
        pass
