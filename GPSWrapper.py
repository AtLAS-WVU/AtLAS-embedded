import os
import time
import math
import threading
import time
#from gps import *
from gpspy3.gps import *


class __GPSWrapper(threading.Thread):
    latitude = None
    longitude = None
    time = None
    altitude = None
    speed = None
    track = None  # Degrees from true north (should be checked with bearing)
    climb = None  # Climb speed in m/s
    mode = 1      # Can be 1(fixing), 2(2D fix), 3(3D fix)
    ready = False

    def __init__(self):
        super().__init__()
        self.gpsd = GPS(mode=WATCH_ENABLE)
        self.current_value = None
        self.running = True

    def run(self):
        self.waitForFix()
        self.ready = True
        while self.running:
            self.updateGPSValues()
            # print("Updated gps. Lat: {}, Lon: {}".format(self.latitude, self.longitude))

    # Function is called to allow drone to acquire and hold a fix on location data before taking off
    # Returns 1 if GPS info has been acquired and held for a period of time, results in a failure otherwise
    def waitForFix(self):
        timeElapsed = 0
        self.gpsd.next()
        while self.gpsd.fix.mode != 2 and self.gpsd.fix.mode != 3:
            # os.system('clear')
            # print 'Waiting for fix (', timeElapsed, ' sec)'
            time.sleep(1)
            timeElapsed += 1
            self.gpsd.next()
        if self.gpsd.fix.mode == 2 or self.gpsd.fix.mode == 3:
            time.sleep(3)
            if self.gpsd.fix.mode == 2 or self.gpsd.fix.mode == 3:
                return 1
            else:
                return 0

    # Function is called to retrieve the newest values from the GPS
    # Returns 1 if read was successful, 0 if it failed
    def updateGPSValues(self):
        try:
            self.gpsd.next()  # grab next set of values
        except StopIteration:
            print("Failed, try again later")
            return 0
        self.latitude = self.gpsd.fix.latitude
        self.longitude = self.gpsd.fix.longitude
        self.time = self.gpsd.utc
        self.altitude = self.gpsd.fix.altitude
        self.speed = self.gpsd.fix.speed
        self.mode = self.gpsd.fix.mode
        self.track = self.gpsd.fix.track
        self.climb = self.gpsd.fix.climb
        return 1	 # Success

    # Function is called to calculate the distance between the current drone location and the destination
    # Returns the calculated distance
    def findDistanceBetweenLatLon(self, serverLat, serverLong):
        R = 6371000  # average radius of earth in meters
        # self.updateGPSValues()
        phi1 = math.radians(self.latitude)
        phi2 = math.radians(serverLat)
        deltaPhi = math.radians(serverLat - self.latitude)
        deltaLambda = math.radians(serverLong - self.longitude)

        # Uses Haversine formula to find distance between two points
        a = math.sin(deltaPhi/2) * math.sin(deltaPhi/2) + (math.cos(phi1) * math.cos(phi2)) * \
                                                          (math.sin(deltaLambda/2) * math.sin(deltaLambda/2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = R * c
        return distance

    # Function is called to calculate the bearing (position of drone relative to destination)
    # Returns the calculated bearing
    def findCurrentBearing(self, destLat, destLong):
        # self.updateGPSValues()
        curRadLat = math.radians(self.latitude)
        curRadLong = math.radians(self.longitude)
        destRadLat = math.radians(destLat)
        destRadLong = math.radians(destLong)

        val1 = math.sin(destRadLong - curRadLong) * math.cos(destRadLat)
        val2 = math.cos(curRadLat) * math.sin(destRadLat) - math.sin(curRadLat) * math.cos(destRadLat) * \
                                                            math.cos(destRadLong - curRadLong)
        bearing = math.degrees(math.atan2(val1, val2))
        return bearing


thread = __GPSWrapper()

while not thread.ready:
     time.sleep(0.5)


def get_distance_to(lat, lon):
    """
    Calculates the distance in meters from the drone's current GPS location to the given latitude and longitude
    :param lat: Destination latitude, in degrees
    :param lon: Destination longitude, in degrees
    :return: Distance to the given coordinates, in meters
    """
    return thread.findDistanceBetweenLatLon(lat, lon)


def get_bearing_to(lat, lon):
    """
    Calculates the direction the drone needs to travel to get to the given destination.
    :param lat: Destination latitude, in degrees
    :param lon: Destination longitude, in degrees
    :return: Bearing to the destination coordinates, in degrees
    """
    return thread.findCurrentBearing(lat, lon)