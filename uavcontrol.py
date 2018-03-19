import serial
from config import SERIAL_BAUD_RATE, SERIAL_PORT, THROTTLE_CHANNEL, PITCH_CHANNEL, YAW_CHANNEL, ROLL_CHANNEL

serial_port = serial.Serial(SERIAL_PORT, baudrate=SERIAL_BAUD_RATE, write_timeout=0.01)
MIN_DIR = 1000
MAX_DIR = 2000
DEFAULT_DIR = (MAX_DIR-MIN_DIR)/2 + MIN_DIR


def send_control_signal(throttle, pitch, yaw, roll):
    # This snippet orders the signals so they match the order the arduino expects
    signals = [x for _, x in sorted(zip(
        [THROTTLE_CHANNEL, PITCH_CHANNEL, ROLL_CHANNEL, YAW_CHANNEL],
        [throttle, pitch, roll, yaw]
    ))]
    signal_bytes = []
    for sig in signals:
        signal_bytes.append((sig >> 8) & 255)
        signal_bytes.append(sig & 255)
    serial_port.write(bytes(signal_bytes))


# calculates the intensity of the motors spinning from 1 to -1
# 1 is the maximum speed in the positive direction
# -1 is the maximum speed in the negative direction
def calculate_intensity(intensity):
    if intensity > 1 or intensity < -1:
        print("Error: Only takes values between -1 and 1")
        return
    return (MAX_DIR - DEFAULT_DIR) * intensity + DEFAULT_DIR
