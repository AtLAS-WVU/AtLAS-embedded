import requests
import time
import os
import GPSWrapper
#Specifies where to send data to
API_DESTINATION = "https://deliverwithatlas.com/UpdateDroneStatus.php"

#Creates a gps object that holds all values to be sent to server
#Communicates via threads to constantly send new coordinates every second
gps = GPSWrapper.thread

#Set default values for aircraft being used
droneID = 55560935
dronePrivateKey = 1234567890
currentBatteryLife = 100
currentStageOfDelivery = 'idle'	#Needs to be updated
try:
	gps.start()
	while True:
		latitude = gps.latitude
		longitude = gps.longitude
		altitude = gps.altitude
		speed = gps.speed
		droneStatus = {"drone_id":droneID, "drone_private_key":dronePrivateKey, "current_battery_life":currentBatteryLife,
        	      	 "current_stage_of_delivery":currentStageOfDelivery, "latitude":latitude, "longitude":longitude,
              		 "altitude":altitude, "speed":speed}

		response = requests.post(url = API_DESTINATION, data = droneStatus)

		#For testing
		print("The result from sending data to server was: %s", response.text)
		time.sleep(1)
except (KeyboardInterrupt, SystemExit): 
	print("\nKilling Thread...")
	gps.running = False
	gps.join()
print("Done.")
