# Use this file to store configuration values that might change in the future.

# Resolution to capture from the camera, as a tuple of (width, height)
CAMERA_RESOLUTION = (1920//4, 1080//4)  # TODO: Adjust to a good resolution

####################
# Simulator Values #
####################

# Switch to False to run the software on the actual drone
SIMULATION = True

ON_DRONE = False

# Drone mass in Kg
DRONE_MASS = 2.35

# Drag, in no particularly scientific unit
DRONE_DRAG = 0.01

# Max thrust. This is a blatant guess.
MAX_THRUST = 4 * DRONE_MASS * 9.81

# Limits of how far the drone can rotate, in degrees
MAX_PITCH = 30
MIN_PITCH = -30
MAX_ROLL = 30
MIN_ROLL = -30

# How fast the drone can yaw, in degrees / sec
MAX_YAW_RATE = 90

# Starting GPS coordinates [lat, lon]
START_COORDS = (39.637836, -79.955235)

# Length of simulator tick, in seconds
SIMULATOR_PERIOD = 0.1

# Maximum velocity, in meters/second
MAX_VELOCITY = 10

############################
# Web Communication Config #
############################

# Specifies where to send data to
API_DESTINATION_TO_SERVER = "https://deliverwithatlas.com/UpdateDroneStatus.php"
API_DESTINATION_FROM_SERVER = "https://deliverwithatlas.com/UpdateWaypoint.php"

DRONE_ID = 1234
DRONE_PRIVATE_KEY = 1234567890

SERVER_POLLING_PERIOD = 1

#####################
# Autonomy Settings #
#####################

# How often should control signals be sent to the flight controller, in seconds
UAV_CONTROL_UPDATE_PERIOD = 0.01

# Number of elements to take a rolling average of for the compass sensor
COMPASS_SENSOR_ROLLING_AVERAGE = 1.0 / UAV_CONTROL_UPDATE_PERIOD

# If the drone is further than this many meters from its target, it will constantly update its
# yaw to face toward the target. If this number is too small, it can lead to some crazy oscillations.
ORIENTATION_ERROR_MARGIN = 20

# Serial port for the arduino. Find out on Linux or Mac by running the command 'ls /dev/tty*'
SERIAL_PORT = "/dev/ttyS0"

########################################################
# MAKE SURE THESE MATCH THE VALUES IN THE ARDUINO CODE #
########################################################
# Baud rate for communicating with the arduino.
SERIAL_BAUD_RATE = 115200
# Number of PPM channels for controlling the drone.
PPM_CHANNELS = 4
THROTTLE_CHANNEL = 2
ROLL_CHANNEL = 0
PITCH_CHANNEL = 1
YAW_CHANNEL = 3
AUX_CHANNEL = 4
MANUAL_CONTROL_CH = 5

NUM_SENSORS = 13
NUM_SONAR_SENSORS = 2
LEDDAR_SENSOR_NUM = 6
COMPASS_SENSOR_NUM = 9
SONAR_SENSOR_NUMS = [7, 8]
YAW_SENSOR_NUM = 10
PITCH_SENSOR_NUM = 11
ROLL_SENSOR_NUM = 12
