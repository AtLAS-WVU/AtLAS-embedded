from uavcontrol import send_control_signal
from cruising import calculate_intensity, DEFAULT_DIR


# throttle downwards from rest
# TODO: implement more graceful landing behavior
def landing(speed):
    send_control_signal(calculate_intensity(speed), DEFAULT_DIR, DEFAULT_DIR, DEFAULT_DIR)


# default movement is landing at 50% speed
if __name__ == '__main__':
    landing(0.5)
