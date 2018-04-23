from simulator import simulation


def set_throttle(throttle):
    if throttle < 0 or throttle > 1:
        raise ValueError("Throttle must be between 0 and 1, inclusive")
    simulation.thread.throttle = throttle


def set_pitch(pitch):
    if pitch < -1 or pitch > 1:
        raise ValueError("Pitch must be between -1 and 1, inclusive")
    simulation.thread.pitch = pitch


def set_yaw_signal(yaw):
    if yaw < -1 or yaw > 1:
        raise ValueError("yaw must be between -1 and 1, inclusive")
    simulation.thread.yaw = yaw


def set_roll(roll):
    if roll < -1 or roll > 1:
        raise ValueError("roll must be between -1 and 1, inclusive")
    simulation.thread.roll = roll


def get_leddar_sensor():
    return simulation.thread.leddar_sensor


def get_sonar_sensors():
    return simulation.thread.sonar_sensors


def get_compass_sensor(average=True, continuous=False):
    bearing = simulation.thread.current_compass_bearing
    if not continuous:
        bearing %= 360.0
    return bearing


# def get_yaw_sensor():
#     """
#     Get the most recent reading from the compass
#     :return: Compass bearing, in degrees
#     """
#     yaw = convert_bytes_to_int(__thread.sensor_buffer[YAW_SENSOR_NUM * 2:YAW_SENSOR_NUM * 2 + 2])
#     yaw /= 10
#     if yaw > 180:
#         yaw -= 360
#     return yaw
#
#
# def get_pitch_sensor():
#     """
#     Get the most recent reading from the compass
#     :return: Compass bearing, in degrees
#     """
#     pitch = convert_bytes_to_int(__thread.sensor_buffer[PITCH_SENSOR_NUM * 2:PITCH_SENSOR_NUM * 2 + 2])
#     pitch /= 10
#     if pitch > 180:
#         pitch -= 360
#     return pitch
#
#
# def get_roll_sensor():
#     """
#     Get the most recent reading from the compass
#     :return: Compass bearing, in degrees
#     """
#     roll = convert_bytes_to_int(__thread.sensor_buffer[ROLL_SENSOR_NUM * 2:ROLL_SENSOR_NUM * 2 + 2])
#     roll /= 10
#     if roll > 180:
#         roll -= 360
#     return roll

throttle_input = 0.8
pitch_input = 0
yaw_input = 0
roll_input = 0


def get_throttle_input():
    return throttle_input


def get_yaw_input():
    return yaw_input


def get_pitch_input():
    return pitch_input


def get_roll_input():
    return roll_input


def get_aux_input():
    return 0


def is_manual_mode():
    return False
