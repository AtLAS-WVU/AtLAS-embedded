import time


class Pid:

    def __init__(self, p, i, d, initial_val=0):
        """

        :param p: P constant
        :param i: I constant
        :param d: D constant
        :param initial_val: Initial error, for use in integral and derivative calculation
        """
        self.p = p
        self.i = i
        self.d = d
        self.prev_error = initial_val
        self.integral = 0
        self.prev_time = time.time()

    def update(self, error, delta_time=None, derivative=None):
        """
        Perform an iteration of the PID loop.
        :param error: The current error
        :param delta_time: How much time has passed since the last time this loop was called. If None, then
            the actual delta time in seconds is measured.
        :param derivative: Derivative of the error. If none, then the derivative is calculated from the last time
            this loop was called.
        :return: Correction value
        """
        if delta_time is None:
            delta_time = time.time() - self.prev_time
        if derivative is None:
            derivative = (error - self.prev_error) / delta_time

        self.prev_time = time.time()
        self.prev_error = error
        self.integral += (error * delta_time)

        return (self.p * error) + (self.i * self.integral) + (self.d * derivative)

    def reset(self):
        """
        Resets the PID loop like it's never been updated before. (Zeros out the integral and prior error)
        :return: None
        """
        self.prev_time = time.time()
        self.integral = 0
        self.prev_error = 0
