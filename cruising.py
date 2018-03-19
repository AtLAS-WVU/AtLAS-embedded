from uavcontrol import send_control_signal, calculate_intensity, DEFAULT_DIR


# throttle upwards (+) or downwards (-)
def throttle_cruise(speed):
    send_control_signal(calculate_intensity(speed), DEFAULT_DIR, DEFAULT_DIR, DEFAULT_DIR)


# pitch forwards (+) or backwards (-)
def pitch_cruise(speed):
    send_control_signal(DEFAULT_DIR, calculate_intensity(speed), DEFAULT_DIR, DEFAULT_DIR)


# roll right (+) or left (-)
def roll_cruise(speed):
    send_control_signal(DEFAULT_DIR, DEFAULT_DIR, DEFAULT_DIR, calculate_intensity(speed))


# yaw right (+) or left (-)
def yaw_cruise(speed):
    send_control_signal(DEFAULT_DIR, DEFAULT_DIR, calculate_intensity(speed), DEFAULT_DIR)


# default movement is pitching forward at 50% speed
if __name__ == '__main__':
    pitch_cruise(0.5)
