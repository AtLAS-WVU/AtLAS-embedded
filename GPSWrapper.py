import os
import time
from gps import *

class GPSWrapper():
        latitude = None
        longitude = None
        time = None
        altitude = None
        speed = None
	track = None	#Degrees from true north
	climb = None	#Climb speed in m/s
        mode = 1        #Can be 1(fixing), 2(2D fix), 3(3D fix)

        gpsd = None 

        def __init__(self):
                global gpsd
                gpsd = gps(mode=WATCH_ENABLE)
		self.latitude = 0.0
		self.longitude = 0.0
		self.time = 0
		self.altitude = 0.0
		self.speed = 0
		self.track = 0
		self.climb = 0.0

        #Function is called to allow drone to acquire and hold a fix on location data before taking off
        #Returns 1 if GPS info has been acquired and held for a period of time, results in a failure otherwise
        def waitForFix(self):
                global gpsd
                timeElapsed = 0
                gpsd.next()
                while gpsd.fix.mode == 1 and self.latitude == 0:
                        #os.system('clear')
                        print 'Waiting for fix (', timeElapsed, ' sec)'
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
                #if gpsd.fix.mode == 1:
                #       return 0     #Failure
                #else:
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

	#Untested
	def closeGPSChannel(self):
		global gpsd
		gpsd = gps(mode=WATCH_DISABLE)

