import requests
import time
import os
from GPSWrapper import thread as gps
import json
import threading
from config import API_DESTINATION_FROM_SERVER, API_DESTINATION_TO_SERVER, DRONE_ID, DRONE_PRIVATE_KEY, \
    SERVER_POLLING_PERIOD


class __ServerThread(threading.Thread):

    # Set default values for aircraft being used
    currentBatteryLife = 100
    currentStageOfDelivery = 'idle'  # Needs to be updated
    isBlocked = False  # Needs to be updated constantly based on sensor readings

    waypoint_latitude = 0
    waypoint_longitude = 0
    waypoint_altitude = 0
    has_waypoint = False

    def __init__(self):
        super().__init__()

    def run(self):

        while True:
            latitude = gps.latitude
            longitude = gps.longitude
            altitude = gps.altitude
            speed = gps.speed
            droneStatus = {"drone_id": DRONE_ID, "drone_private_key": DRONE_PRIVATE_KEY,
                           "current_battery_life": self.currentBatteryLife,
                           "current_stage_of_delivery": self.currentStageOfDelivery, "latitude": latitude,
                           "longitude": longitude,
                           "altitude": altitude, "speed": speed}

            response = requests.post(url=API_DESTINATION_TO_SERVER, data=droneStatus)

            # For testing
            # print("The result from sending data to server was: ", response.text)

            obstacleStatus = {"drone_id": DRONE_ID, "drone_private_key": DRONE_PRIVATE_KEY,
                              "is_blocked": self.isBlocked}

            response = requests.post(url=API_DESTINATION_FROM_SERVER, data=obstacleStatus)
            response = response.json()
            if response["success"]:
                self.has_waypoint = True
                self.waypoint_longitude = response["waypoint"]["longitude"]
                self.waypoint_latitude = response["waypoint"]["latitude"]
                self.waypoint_altitude = response["waypoint"]["altitude"]
            else:
                self.has_waypoint = False

            # For testing
            # print("Result from getting new waypoint from server: ", response.text)
            time.sleep(SERVER_POLLING_PERIOD)


__thread = __ServerThread()
__thread.start()


def has_waypoint():
    return __thread.has_waypoint


def get_waypoint():
    """
    :return: longitude, latitude, altitude, or None if there is no waypoint
    """
    if __thread.has_waypoint:
        return __thread.waypoint_longitude, __thread.waypoint_latitude, __thread.waypoint_altitude
    else:
        return None
