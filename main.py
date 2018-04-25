import holdposition
from GPSWrapper import thread as gps
import DroneServerCom
import enum
import time
import config
import threading
if config.SIMULATION:
    import simulator.uavcontrol as uavcontrol
else:
    import uavcontrol


class Mode(enum.Enum):
    LANDED = 1
    TAKEOFF = 2
    CRUISING = 3
    LANDING = 4


def main():
    mode = Mode.LANDED

    ALTITUDE = 100
    lat, lon, alt = 0, 0, 0
    while True:

        if DroneServerCom.has_waypoint():
            if mode == Mode.LANDED:
                DroneServerCom.set_status('transit')
                print("Drone is taking off!")
                lat, lon, alt = DroneServerCom.get_waypoint()
                holdposition.set_target(gps.latitude, gps.longitude, ALTITUDE)
                holdposition.set_enabled(True)
                mode = Mode.TAKEOFF
            elif mode == Mode.TAKEOFF:
                if gps.altitude >= (ALTITUDE - 5):
                    time.sleep(5)
                    print("Reached altitude! Transitioning to cruising.")
                    mode = Mode.CRUISING
                    lat, lon, alt = DroneServerCom.get_waypoint()
                    holdposition.set_target(lat, lon, ALTITUDE)
            elif mode == Mode.CRUISING:
                prevlat, prevlon, prevalt = lat, lon, alt
                lat, lon, alt = DroneServerCom.get_waypoint()
                if lat != prevlat or lon != prevlon or alt != prevalt:
                    print("Waypoint changed from ({:10.6f}, {:10.6f}) to ({:10.6f}, {:10.6f}), Altitude: {:5.2f}"
                          .format(prevlat, prevlon, lat, lon, alt))
                    holdposition.set_target(lat, lon, ALTITUDE)
                if gps.findDistanceBetweenLatLon(lat, lon) < 5:
                    time.sleep(5)
                    print("Reached destination! Landing...")
                    holdposition.set_target(holdposition.get_target()[0], holdposition.get_target()[1], -0.5)
                    mode = Mode.LANDING
            elif mode == Mode.LANDING:
                if gps.altitude < 1:
                    time.sleep(5)
                    holdposition.set_enabled(False)
                    uavcontrol.set_throttle(0)
                    mode = Mode.LANDED
                    DroneServerCom.set_status('delivered')
                    time.sleep(5)
                    DroneServerCom.set_status('idle')

        elif mode != Mode.LANDED:
            print("Error: Server returned no waypoint, even though drone is not landed!")
            holdposition.set_target(gps.latitude, gps.longitude, 0)
        time.sleep(0.5)

if __name__ == "__main__":
    main()

# from GPSWrapper import thread as gps; import holdposition as h; import simulator.uavcontrol as uav; import simulator.simulation as sim; import threading; import main; thread = threading.Thread(None, main.main)
