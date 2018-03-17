import serial
from config import SERIAL_BAUD_RATE, SERIAL_PORT, THROTTLE_CHANNEL, PITCH_CHANNEL, YAW_CHANNEL, ROLL_CHANNEL

serial_port = serial.Serial(SERIAL_PORT, baudrate=SERIAL_BAUD_RATE, write_timeout=0.01)


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

