
import random
import threading
import time


# The (threading.Thread) is the super class of the Demo class.
class Demo(threading.Thread):

    # This is the constructor for the Demo class. Every method in Python needs the "self" parameter first,
    # for reasons
    def __init__(self):
        super().__init__()
        self.running = True
        self.number = random.randint(0, 100)
        # Remember semaphores from CS 350? This is one of those
        self.lock = threading.Lock()

    def stop(self):
        self.running = False

    # The run method is executed in a separate thread
    def run(self):
        print("Starting!")
        while self.running:
            # Gotta acquire a lock because the number variable is accesed by multiple threads.
            self.lock.acquire()
            self.number = random.randint(0, 100)
            print("Changed number to {}.".format(self.number))
            # Always remember to release the lock ASAP
            self.lock.release()
            # Release the lock BEFORE sleeping, or else it'll hold onto the lock for the entire 5 seconds
            # while it's sleeping
            time.sleep(5)

    def get_number(self, unused_parameter=True):
        """
        This is how functions are documented in python.
        :param unused_parameter: I just put this here to demonstrate documenting parameters
        :return: The number, duh
        """
        self.lock.acquire()
        # Here I make a copy of the number so that I can release the lock before returning it
        number = self.number
        self.lock.release()
        return number

# This is the equivalent of Java's "new" keyword. It constructs an object.
demo = Demo()
demo.start()
