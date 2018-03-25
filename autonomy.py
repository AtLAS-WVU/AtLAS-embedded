import threading
import uavcontrol
import time

from config import UAV_CONTROL_UPDATE_PERIOD


class __AutonomyThread(threading.Thread):

    def __init__(self):
        super().__init__()
        self.running = True

    def run(self):
        while self.running:
            try:
                throttle = uavcontrol.get_throttle_input()
                uavcontrol.set_throttle(throttle)
            except TypeError:
                print("Type error...")
            time.sleep(UAV_CONTROL_UPDATE_PERIOD)


__thread = __AutonomyThread()
__thread.start()
