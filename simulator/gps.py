import time
from simulator import simulation


class _Fix:
    mode = 1
    latitude = 0
    longitude = 0
    utc = 0
    altitude = 0
    speed = 0
    track = 0
    climb = 0


class __GPS:

    fix = _Fix()
    utc = 0

    def __init__(self, mode=None):
        self.utc = time.time()
        self.fix.mode = 2

    def next(self):
        time.sleep(1)
        self.fix.latitude = simulation.thread.current_coords[0]
        self.fix.longitude = simulation.thread.current_coords[1]
        self.fix.altitude = simulation.thread.current_altitude
        self.utc = time.time()


_gps = __GPS()


def GPS(mode=None):
    return _gps

WATCH_ENABLE = 1
