import serial
import threading
import time
from config import SERIAL_BAUD_RATE, SERIAL_PORT, THROTTLE_CHANNEL, PITCH_CHANNEL, YAW_CHANNEL, ROLL_CHANNEL, \
    AUX_CHANNEL, MANUAL_CONTROL_CH, UAV_CONTROL_UPDATE_PERIOD, NUM_SENSORS, LEDDAR_SENSOR_NUM, SONAR_SENSOR_NUMS, \
    COMPASS_SENSOR_NUM, YAW_SENSOR_NUM, PITCH_SENSOR_NUM, ROLL_SENSOR_NUM


MIN_DIR = 1000
MAX_DIR = 2000
DEFAULT_DIR = (MAX_DIR-MIN_DIR)/2 + MIN_DIR


class __UavControlThread(threading.Thread):

    def __init__(self):
        super().__init__()
        self.throttle = 1000
        self.pitch = 1500
        self.yaw = 1500
        self.roll = 1500
        self.serial_port = serial.Serial(SERIAL_PORT, baudrate=SERIAL_BAUD_RATE, timeout=0.05, write_timeout=0.1)
        self.running = True
        self.sensor_buffer = None

    def run(self):
        while self.running:
            self.__read_sensors()
            self.__send_control_signal()
            time.sleep(UAV_CONTROL_UPDATE_PERIOD)

    def __send_control_signal(self):
        # This snippet orders the signals so they match the order the arduino expects
        signals = [x for _, x in sorted(zip(
            [THROTTLE_CHANNEL, PITCH_CHANNEL, ROLL_CHANNEL, YAW_CHANNEL],
            [self.throttle, self.pitch, self.roll, self.yaw]
        ))]
        signal_bytes = []
        for sig in signals:
            signal_bytes.append((sig >> 8) & 255)
            signal_bytes.append(sig & 255)
        self.serial_port.write(bytes(signal_bytes))

    def __read_sensors(self):
        # print("Reading sensors...")
        # print("Num bytes available: {}".format(self.serial_port.in_waiting))
        self.sensor_buffer = self.serial_port.read(NUM_SENSORS * 2)


__thread = __UavControlThread()
__thread.start()

# Wait for the thread to make initial connection with the arduino.
while __thread.sensor_buffer is None:
    time.sleep(0.01)


def convert_bytes_to_int(bytes_):
    """
    Converts a byte string to a single unsigned integer
    :param bytes_: A little-endian byte string
    :return: A positive integer
    """
    result = 0
    for byte in reversed(bytes_):
        result <<= 8
        result += byte
    return result


def set_throttle(throttle):
    """
    Sets the throttle of the drone.
    :param throttle: A number between 0 and 1, where 0 is off, and 1 is full throttle
    :return: None
    :raise: ValueError is throttle is not between 0 and 1
    """
    if throttle < 0 or throttle > 1:
        raise ValueError("Throttle must be between 0 and 1, inclusive")
    __thread.throttle = int(throttle * (MAX_DIR - MIN_DIR) + MIN_DIR)


def set_pitch(pitch):
    """
    Sets the pitch of the drone.
    :param pitch: A number between -1 and 1, where 0 is neutral, 1 is full forward, and -1 is full backward
    :return: None
    :raise: ValueError if pitch is not between -1 and 1
    """
    if pitch < -1 or pitch > 1:
        raise ValueError("Pitch must be between -1 and 1, inclusive")
    __thread.pitch = int((pitch / 2 + 0.5) * (MAX_DIR - MIN_DIR) + MIN_DIR)


def set_yaw_signal(yaw):
    """
    Sets the yaw signal sent to the drone.
    :param yaw: A number between -1 and 1, where 0 is neutral, 1 is full right, and -1 is full left
    :return: None
    :raise: ValueError if yaw is not between -1 and 1
    """
    if yaw < -1 or yaw > 1:
        raise ValueError("yaw must be between -1 and 1, inclusive")
    __thread.yaw = int((yaw / 2 + 0.5) * (MAX_DIR - MIN_DIR) + MIN_DIR)


def set_roll(roll):
    """
    Sets the roll of the drone.
    :param roll: A number between -1 and 1, where 0 is neutral, 1 is full right, and -1 is full left
    :return: None
    :raise: ValueError if roll is not between -1 and 1
    """
    if roll < -1 or roll > 1:
        raise ValueError("roll must be between -1 and 1, inclusive")
    __thread.roll = int((roll / 2 + 0.5) * (MAX_DIR - MIN_DIR) + MIN_DIR)


def get_leddar_sensor():
    """
    Gets the most recent reading from the LEDdar sensor
    :return: Distance measured by the sensor, in meters
    """
    # TODO: Convert to meters
    return convert_bytes_to_int(__thread.sensor_buffer[LEDDAR_SENSOR_NUM * 2:LEDDAR_SENSOR_NUM * 2 + 2])


def get_sonar_sensors():
    """
    Get the most recent readings from all of the sonar sensors.
    :return: A list of distances read by the sensors, in meters
    """
    # TODO: Convert to meters
    return [convert_bytes_to_int(__thread.sensor_buffer[sensor_num * 2:sensor_num * 2 + 2])
            for sensor_num in SONAR_SENSOR_NUMS]


def get_compass_sensor():
    """
    Get the most recent reading from the compass
    :return: Compass bearing, in degrees
    """
    bearing = convert_bytes_to_int(__thread.sensor_buffer[COMPASS_SENSOR_NUM * 2:COMPASS_SENSOR_NUM * 2 + 2])
    bearing /= 10
    return bearing


def get_yaw_sensor():
    """
    Get the most recent reading from the compass
    :return: Compass bearing, in degrees
    """
    yaw = convert_bytes_to_int(__thread.sensor_buffer[YAW_SENSOR_NUM * 2:YAW_SENSOR_NUM * 2 + 2])
    yaw /= 10
    if yaw > 180:
        yaw -= 360
    return yaw


def get_pitch_sensor():
    """
    Get the most recent reading from the compass
    :return: Compass bearing, in degrees
    """
    pitch = convert_bytes_to_int(__thread.sensor_buffer[PITCH_SENSOR_NUM * 2:PITCH_SENSOR_NUM * 2 + 2])
    pitch /= 10
    if pitch > 180:
        pitch -= 360
    return pitch


def get_roll_sensor():
    """
    Get the most recent reading from the compass
    :return: Compass bearing, in degrees
    """
    roll = convert_bytes_to_int(__thread.sensor_buffer[ROLL_SENSOR_NUM * 2:ROLL_SENSOR_NUM * 2 + 2])
    roll /= 10
    if roll > 180:
        roll -= 360
    return roll


def __constrain(val, min_=-1, max_=1):
    if val < min_:
        return min_
    elif val > max_:
        return max_
    else:
        return val


def get_throttle_input():
    """
    :return: The value of the remote control throttle input, in range [0, 1]
    """
    throttle = convert_bytes_to_int(__thread.sensor_buffer[THROTTLE_CHANNEL * 2:THROTTLE_CHANNEL * 2 + 2])
    return __constrain((throttle - 1000) / 1000, min_=0)


def get_yaw_input():
    """
    :return: The value of the remote control yaw input, in range [-1, 1]
    """
    yaw = convert_bytes_to_int(__thread.sensor_buffer[YAW_CHANNEL * 2:YAW_CHANNEL * 2 + 2])
    return __constrain((yaw - 1500) / 500)


def get_pitch_input():
    """
    :return: The value of the remote control pitch input, in range [-1, 1]
    """
    pitch = convert_bytes_to_int(__thread.sensor_buffer[PITCH_CHANNEL * 2:PITCH_CHANNEL * 2 + 2])
    return __constrain((pitch - 1500) / 500)


def get_roll_input():
    """
    :return: The value of the remote control roll input, in range [-1, 1]
    """
    roll = convert_bytes_to_int(__thread.sensor_buffer[ROLL_CHANNEL * 2:ROLL_CHANNEL * 2 + 2])
    return __constrain((roll - 1500) / 500)


def get_aux_input():
    """
    :return: A boolean. True if the auxiliary switch (the left-most switch on the remote control) is on (flipped down)
    """
    return convert_bytes_to_int(__thread.sensor_buffer[AUX_CHANNEL * 2:AUX_CHANNEL * 2 + 2]) > 1500


def is_manual_mode():
    """
    :return: A boolean. True if the remote control is set to manually control the drone.
    """
    return convert_bytes_to_int(__thread.sensor_buffer[MANUAL_CONTROL_CH * 2:MANUAL_CONTROL_CH * 2 + 2]) > 1500


def _test():
    while True:
        print("Yaw: {:.1f}, Pitch: {:.1f}, Roll: {:.1f}, Compass: {:.1f}".format(get_yaw_sensor(), get_pitch_sensor(),
                                                                                 get_roll_sensor(),
                                                                                 get_compass_sensor()))
        time.sleep(0.1)
