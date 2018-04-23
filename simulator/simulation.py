import config
import threading
import logging
import time
import math

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


# Function is called to calculate the final destination when provided an initial lat and lon in degrees
# distance to be traveled in meters, and a bearing value
# Returns the final destination GPS coordinate
def findDestination(initCoords, bearing, distance):
    R = 6371000  # average radius of earth in meters
    initRadLat = math.radians(initCoords[0])
    initRadLong = math.radians(initCoords[1])
    bearRad = math.radians(bearing)
    angularDist = distance / R

    destLatitude = math.asin(
        math.sin(initRadLat) * math.cos(angularDist) + math.cos(initRadLat) * math.sin(angularDist) * math.cos(bearRad))
    destLongitude = initRadLong + math.atan2(math.sin(bearRad) * math.sin(angularDist) * math.cos(initRadLat),
                                             math.cos(angularDist) - math.sin(initRadLat) * math.sin(destLatitude))

    return math.degrees(destLatitude), math.degrees(destLongitude)


def rotate(front, right, theta):
    theta = math.radians(theta)
    newfront = math.cos(theta) * front - math.sin(theta) * right
    newright = math.sin(theta) * front + math.cos(theta) * right
    return newfront, newright


class __Simulator(threading.Thread):

    # Current GPS coordinates of the drone [lat, lon]
    current_coords = tuple(config.START_COORDS)
    # Altitude of the drone above sea level, in meters.
    current_altitude = 0
    # Velocity of the drone, in meters/second, forward
    current_velocity_north = 0.0
    # Velocity of the drone, in meters/second, to the right
    current_velocity_east = 0.0
    # Current orientation of the drone, in degrees.
    current_compass_bearing = 0.0
    # Reading of the leddar sensor, in meters
    leddar_sensor = 0.0
    # Readings of the sonar sensors, in meters
    sonar_sensors = [0.0] * config.NUM_SONAR_SENSORS

    roll = 0
    pitch = 0
    yaw = 0
    throttle = 0

    def __init__(self):
        super().__init__()

    def run(self):
        while True:

            delta_yaw = -self.yaw * config.MAX_YAW_RATE * config.SIMULATOR_PERIOD
            self.current_compass_bearing += delta_yaw

            roll_degrees = self.roll * config.MAX_ROLL
            pitch_degrees = self.pitch * config.MAX_PITCH
            force_right = self.throttle * math.sin(math.radians(roll_degrees))
            force_forward = self.throttle * math.sin(math.radians(pitch_degrees))
            accel_right = force_right / config.DRONE_MASS
            accel_forward = force_forward / config.DRONE_MASS
            accel_forward, accel_right = rotate(accel_forward, accel_right, self.current_compass_bearing)
            self.current_velocity_north += accel_forward
            self.current_velocity_east += accel_right

            total_velocity = math.sqrt(self.current_velocity_north ** 2 + self.current_velocity_east ** 2)
            if total_velocity > config.MAX_VELOCITY:
                self.current_velocity_north = (self.current_velocity_north / total_velocity) * config.MAX_VELOCITY
                self.current_velocity_east = (self.current_velocity_east / total_velocity) * config.MAX_VELOCITY
            self.current_coords = findDestination(self.current_coords, 0,
                                                  self.current_velocity_north * config.SIMULATOR_PERIOD)
            self.current_coords = findDestination(self.current_coords, 90,
                                                  self.current_velocity_east * config.SIMULATOR_PERIOD)
            time.sleep(config.SIMULATOR_PERIOD)


thread = __Simulator()
thread.start()


# import simulator.uavcontrol as uav; import simulator.simulation as sim; uav.set_throttle(1); uav.set_pitch(1); from GPSWrapper import thread as gps; uav.set_pitch(0);