import threading
import config
if config.SIMULATION:
    from simulator import uavcontrol
else:
    import uavcontrol
import time
import math
from pid import Pid
from GPSWrapper import thread as gps

from config import UAV_CONTROL_UPDATE_PERIOD


def constrain(val, min_=-1.0, max_=1.0):
    return max(min(val, max_), min_)


class __AutonomyThread(threading.Thread):

    def __init__(self):
        super().__init__()
        self.running = True
        self.target_lat = 39
        self.target_lon = 70
        self.target_yaw = 0
        self.pid_left = Pid(0.1, 0, 0.3, max_integral=1)
        self.pid_forward = Pid(0.1, 0, 0.3, max_integral=1)
        self.pid_yaw = Pid(0.01, 0.005, 0, max_integral=10)
        self.prev_yaw_error = 0
        self.aux_switch_was_flipped = False
        self.last_debug_print = time.time()
        self.last_gps_update = 0

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
        error_angle = gps.findCurrentBearing(self.target_lat, self.target_lon)
        bearing = uavcontrol.get_compass_sensor()

        left_error = error_magnitude * math.sin(math.radians(bearing - error_angle))
        forward_error = error_magnitude * math.cos(math.radians(bearing - error_angle))

        return -left_error, -forward_error

    def reset_target(self, lat, lon, yaw):
        self.target_lat = lat
        self.target_lon = lon
        self.target_yaw = yaw
        self.pid_forward.reset()
        self.pid_left.reset()
        self.pid_yaw.reset()

    def run(self):
        time.sleep(3)
        self.reset_target(gps.latitude, gps.longitude, uavcontrol.get_compass_sensor(average=True, continuous=True))

        while self.running:
            if uavcontrol.get_aux_input() and not self.aux_switch_was_flipped:
                self.aux_switch_was_flipped = True
                print("AUX SWITCH FLIPPED")
                self.reset_target(gps.latitude, gps.longitude,
                                  uavcontrol.get_compass_sensor(average=True, continuous=True))
            elif not uavcontrol.get_aux_input():
                self.aux_switch_was_flipped = False
            # Set throttle manually from remote control
            throttle = uavcontrol.get_throttle_input()
            uavcontrol.set_throttle(throttle)
            # print("set throttle to {}".format(throttle))

            yaw_error = uavcontrol.get_compass_sensor(average=True, continuous=True) - self.target_yaw
            yaw_correction = self.pid_yaw.update(yaw_error)
            yaw_correction = constrain(yaw_correction, min_=-0.8, max_=0.8)
            # Positive yaw is right, so negate it to make it left
            uavcontrol.set_yaw_signal(-yaw_correction)
            # uavcontrol.set_yaw_signal(uavcontrol.get_yaw_input())

            if time.time() - self.last_gps_update >= 1:
                self.last_gps_update = time.time()
                gps.updated = False
                left_error, forward_error = self.calc_error()
                left_correction = self.pid_left.update(left_error)
                forward_correction = self.pid_forward.update(forward_error)

                left_correction = constrain(left_correction, min_=-0.8, max_=0.8)
                forward_correction = constrain(forward_correction, min_=-0.8, max_=0.8)

                # Positive pitch is forward, so this is all good
                uavcontrol.set_pitch(forward_correction)
                # Positive roll is right, so negate it to make it left
                uavcontrol.set_roll(-left_correction)

                print("{:.1f} Err: (L: {:7.3f}, F: {:7.3f}, Y: {:7.3f}), "
                      "PID: (L: {:7.3f}, F: {:7.3f}, Y: {:7.3f})".format(
                          time.time(), left_error, forward_error, yaw_error, left_correction,
                          forward_correction, yaw_correction)
                      )

            # uavcontrol.set_pitch(uavcontrol.get_pitch_input())
            # uavcontrol.set_roll(uavcontrol.get_roll_input())

            time.sleep(UAV_CONTROL_UPDATE_PERIOD)


__thread = __AutonomyThread()
__thread.start()
if __name__ == "__main__":
    while True:
        time.sleep(1)

# thread = __thread
