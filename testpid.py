from pid import Pid
from matplotlib import pyplot
import random
import math

"""
This is just a dumb test for my PID loop.
It in no way properly reflects the actual drone.
"""


def test_pid(mass=3, velocity=1, start_point=2, iterations=2000, time_step=0.01, wind_force=0.1, max_force=5):
    pid = Pid(1.5, 0.5, 0.5)
    current_position = start_point
    times = []
    positions = []
    winds = []

    wind_f = [(random.random(), random.random()) for _ in range(0, 10)]

    def wind(time_step):
        result = 0
        for x, y in wind_f:
            result += wind_force * (x - 0.5) * math.sin(y * 4 * time_step)
        return result

    for i in range(0, iterations):
        force = min(max(pid.update(current_position, delta_time=time_step), -max_force), max_force)
        force += wind(i * time_step)
        acceleration = force / mass
        velocity += acceleration
        current_position += velocity * time_step
        times.append(i * time_step)
        positions.append(current_position)
        winds.append(wind(i * time_step))

    pyplot.plot(times, positions)
    pyplot.plot(times, [0] * len(times), linestyle="--")
    pyplot.plot(times, winds, linestyle=":")
    pyplot.show()


if __name__ == "__main__":
    test_pid()