import os
import time
import threading
from gps import *

class GPSWrapper(threading.Thread):
        latitude = None
        longitude = None
        time = None
        altitude = None
        speed = None
	track = None	#Degrees from true north (should be checked with bearing)
	climb = None	#Climb speed in m/s
        mode = 1        #Can be 1(fixing), 2(2D fix), 3(3D fix)

        gpsd = None

        def __init__(self):
		threading.Thread.__init__(self)
                global gpsd
                gpsd = gps(mode=WATCH_ENABLE)
		self.current_value = None
		self.running = True

	def run(self):
		global gpsd
		while self.running:
			self.updateGPSValues()

        #Function is called to allow drone to acquire and hold a fix on location data before taking off
        #Returns 1 if GPS info has been acquired and held for a period of time, results in a failure otherwise
        def waitForFix(self):
                global gpsd
                timeElapsed = 0
                gpsd.next()
                while gpsd.fix.mode != 2 and gpsd.fix.mode != 3:
                        #os.system('clear')
                        #print 'Waiting for fix (', timeElapsed, ' sec)'
                        time.sleep(1)
                        timeElapsed +=1
                        gpsd.next()
                if gpsd.fix.mode == 2 or gpsd.fix.mode == 3:
                        time.sleep(3)
                        if gpsd.fix.mode == 2 or gpsd.fix.mode == 3:
                                return 1
                        else:
                                return 0

        #Function is called to retrieve the newest values from the GPS
        #Returns 1 if read was successful, 0 if it failed
        def updateGPSValues(self):
                global gpsd
                self.latitude = gpsd.fix.latitude
                self.longitude = gpsd.fix.longitude
                self.time = gpsd.utc
                self.altitude = gpsd.fix.altitude
                self.speed = gpsd.fix.speed
                self.mode = gpsd.fix.mode
		self.track = gpsd.fix.track
		self.climb = gpsd.fix.climb
                gpsd.next()	#grab next set of values
                return 1	#Success


	#Function is called to calculate the distance between the current drone location and the destination
	#Returns the calculated distance
        def findDistanceBetweenLatLon(self, serverLat, serverLong):
            R = 6371000 #average radius of earth in meters
            updateGPSValues()
            phi1 = math.radians(self.latitude)
            phi2 = math.radians(serverLat)
            deltaPhi = math.radians(serverLat - self.latitude)
            deltaLambda = math.radians(serverLong - self.longitude)

            #Uses Haversine formula to find distance between two points
            a = math.sin(deltaPhi/2) * math.sin(deltaPhi/2) + (math.cos(phi1) * math.cos(phi2)) * (sin(deltaLambda/2) * sin(deltaLambda/2))
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
            distance = R * c
            return distance

	#Function is called to calculate the bearing (position of drone relative to destination)
	#Returns the calculated bearing
        def findCurrentBearing(self, destLat, destLong):
            updateGPSValues()
            curRadLat = math.radians(self.latitude)
            curRadLong = math.radians(self.longitude)
            destRadLat = math.radians(destLat)
            destRadLong = math.radians(destLong)

            val1 = math.sin(destRadLong - curRadLong) * math.cos(destRadLat)
            val2 = math.cos(curRadLat) * math.sin(destRadLat) - math.sin(curRadLat) * math.cos(destRadLat) * math.cos(destRadLong - curRadLong)
            bearing = math.degrees(math.atan2(val1, val2))
            return bearing
