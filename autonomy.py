import threading
import uavcontrol
import time
import math
from pid import Pid
from GPSWrapper import thread as gps

from config import UAV_CONTROL_UPDATE_PERIOD


def constrain(val, min_=-1, max_=1):
    return max(min([val, max_]), min_)


class __AutonomyThread(threading.Thread):

    def __init__(self):
        super().__init__()
        self.running = True
        self.target_lat = 39
        self.target_lon = 70
        self.pid_left = Pid(1, 0.5, 0.5, max_integral=1)
        self.pid_forward = Pid(1, 1, 1, max_integral=1)
        self.aux_switch_was_flipped = False
        self.last_debug_print = time.time()

    def calc_error(self):
        """
        Calculates the distance the drone is from the current target latitude and longitude,
        in meters, in each direction.
        :return: (left_error, forward_error): left_error is how far the drone is, to the left, from the target.
                 forward_error is how far the drone is, in the forward direction, from the target.
                 Either error can be negative or positive.
        """
        # TODO: Uncomment the real stuff and comment out the fake values
        error_magnitude = gps.findDistanceBetweenLatLon(self.target_lat, self.target_lon)
        # error_magnitude = 4
        error_angle = gps.findCurrentBearing(self.target_lat, self.target_lon)
        # error_angle = 0
        bearing = uavcontrol.get_compass_sensor()
        # bearing = 0

        left_error = error_magnitude * math.sin(math.radians(bearing - error_angle))
        forward_error = error_magnitude * math.cos(math.radians(bearing - error_angle))

        return left_error, forward_error

    def reset_target(self, lat, lon):
        self.target_lat = lat
        self.target_lon = lon
        self.pid_forward.reset()
        self.pid_left.reset()

    def run(self):
        self.reset_target(gps.latitude, gps.longitude)
        while self.running:
            if uavcontrol.get_aux_input() and not self.aux_switch_was_flipped:
                self.aux_switch_was_flipped = True
                print("AUX SWITCH FLIPPED")
                self.reset_target(gps.latitude, gps.longitude)
            elif not uavcontrol.get_aux_input():
                self.aux_switch_was_flipped = False
            # Set throttle manually from remote control
            throttle = uavcontrol.get_throttle_input()
            uavcontrol.set_throttle(throttle)

            left_error, forward_error = self.calc_error()

            left_correction = self.pid_left.update(left_error)
            forward_correction = self.pid_forward.update(forward_error)
            if time.time() - self.last_debug_print > 0.5:
                self.last_debug_print = time.time()
                print("Err: (L: {:7.4f}, F: {:7.4f}), PID: (L: {:7.4f}, F: {:7.4f})".format(
                    left_error, forward_error, left_correction, forward_correction)
                )

            time.sleep(UAV_CONTROL_UPDATE_PERIOD)


__thread = __AutonomyThread()
# __thread.start()
thread = __thread
