from uavcontrol import send_control_signal


MIN_DIR = 1000
MAX_DIR = 2000
DEFAULT_DIR = (MAX_DIR-MIN_DIR)/2 + MIN_DIR


# calculates the intensity of the motors spinning from 1 to -1
# 1 is the maximum speed in the positive direction
# -1 is the maximum speed in the negative direction
def calculate_intensity(intensity):
    if intensity > 1 or intensity < -1:
        print("Error: Only takes values between -1 and 1")
    return (MAX_DIR - DEFAULT_DIR) * intensity + DEFAULT_DIR


# throttle upwards (+) or downwards (-)
def elevate_cruise(speed):
    send_control_signal(calculate_intensity(speed), DEFAULT_DIR, DEFAULT_DIR, DEFAULT_DIR)


# pitch forwards (+) or backwards (-)
def pitch_cruise(speed):
    send_control_signal(DEFAULT_DIR, calculate_intensity(speed), DEFAULT_DIR, DEFAULT_DIR)


# roll right (+) or left (-)
def roll_cruise(speed):
    send_control_signal(DEFAULT_DIR, DEFAULT_DIR, DEFAULT_DIR, calculate_intensity(speed))


# rotate right (+) or left (-)
def rotate(speed):
    send_control_signal(DEFAULT_DIR, DEFAULT_DIR, calculate_intensity(speed), DEFAULT_DIR)


# default movement is pitching forward at 50% speed
if __name__ == '__main__':
    pitch_cruise(0.5)
