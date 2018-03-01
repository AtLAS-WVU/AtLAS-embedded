
from enum import Enum
import threading
import logging

logger = logging.getLogger(__name__)


class Mode(Enum):
    IDLE = 1
    TAKEOFF = 2
    CRUISE = 3
    LANDING = 4

lock = threading.Lock()
currentmode = Mode.IDLE


def get_mode():
    """
    Returns the current mode of the drone. Is thread-safe.
    :return: The current mode of the drone. One of IDLE, TAKEOFF, CRUISE, or LANDING
    """
    lock.acquire()
    mode = currentmode
    lock.release()
    logger.debug("Set mode to %s", mode)
    return mode


def set_mode(mode):
    """
    Sets the mode of the drone. Is thread-safe.
    :param mode: One of IDLE, TAKEOFF, CRUISE, or LANDING
    :return: None
    """
    global currentmode
    lock.acquire()
    currentmode = mode
    lock.release()
