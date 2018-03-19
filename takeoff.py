from uavcontrol import send_control_signal
from cruising import calculate_intensity, DEFAULT_DIR


# throttle upwards from rest
# TODO: implement more graceful takeoff behavior
def takeoff(speed):
    send_control_signal(calculate_intensity(speed), DEFAULT_DIR, DEFAULT_DIR, DEFAULT_DIR)


# default movement is taking off at 50% speed
if __name__ == '__main__':
    takeoff(0.5)
