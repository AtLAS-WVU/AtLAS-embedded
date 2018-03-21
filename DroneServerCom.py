import requests
from GPSWrapper import *
#Specifies where to send data to
API_DESTINATION = "https://deliverwithatlas.com/UpdateDroneStatus.php"

#Creates a gps object that holds all values to be sent to server
gps = GPSWrapper()

#Gather data values to be sent
droneID = 55560935
dronePrivateKey = 1234567890
currentBatteryLife = 100    #(optional field until we can confirm this is possible)
currentStageOfDelivery = 'idle'     #(cruising, takeoff, landing, idle)
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
