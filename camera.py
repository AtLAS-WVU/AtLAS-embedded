from cv2 import cv2
import numpy as np
from config import CAMERA_RESOLUTION
import threading
import rwlock
import time


"""
Module for reading images from the camera.
The difficulty in reading the camera values is that this is a blocking IO operation, and the camera operates
at a maximum of 30FPS. This means that every time I want to read from the camera, I need to wait an average
of 1/30 seconds. Therefore, I have split this functionality into this separate thread, which can alert other
threads when the next frame is ready.

The only public members of this package are the functions add_event, remove_event, and copy_last_frame, at the
end of this file.
"""


__event_lock = threading.Lock()
__events = set()


class __CameraReader(threading.Thread):

    def __init__(self, event_lock, events):
        super().__init__()
        # TODO: Swap this out for picam
        self.cap = cv2.VideoCapture(0)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_RESOLUTION[1])
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_RESOLUTION[0])

        # Preallocate two buffers, so that I don't have to allocate a new array for every frame.
        # last_frame holds the frame that was most recently read from the camera.
        self.last_frame = np.zeros(CAMERA_RESOLUTION, dtype=np.uint8)
        # next_frame holds the frame which I am currently waiting to get from the camera.
        self.next_frame = np.zeros(CAMERA_RESOLUTION, dtype=np.uint8)
        # Because of multithreading, we need some locks. A RWLock (Read Write Lock) allows any number of threads to
        # acquire a read lock simultaneously, but only one thread can have a write lock at a time.
        self.frame_lock = rwlock.RWLock()
        # To stop the thread, set self.running to False
        self.running = True
        self.event_lock = event_lock
        self.events = events

        # Timing for debugging
        self.__start_time = time.time()
        self.__num_iterations = 0
        # This event is set once the first frame has been read. That way, processes can't try to read from the
        # camera before it is initialized.
        self.ready_event = threading.Event()

    def __capture_frame(self, array):
        """
        Captures an image frame from the camera and stores it in array.
        This is a temporary function which uses the laptop webcam and OpenCV to capture the image.
        :param array: A numpy array with shape (width, height) or (width, height, 1) for a grayscale image,
                      or (width, height, 3) for color. The captured image is stored within this array.
                      The image is automatically resized to fit within the array.
        :return: None.
        """
        # TODO: Replace this function with one which uses the raspberry pi camera
        # TODO: Add an error code return value, based on what picam returns
        color = len(array.shape) == 3 and array.shape[2] == 3
        success, frame = self.cap.read()
        if not color:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.resize(frame, array.shape[0:2])
        array[...] = np.resize(frame.transpose((1, 0, 2) if color else (1, 0)), array.shape)

    def run(self):
        """
        This function is called once in a new thread
        :return: None
        """
        while self.running:
            self.__num_iterations += 1
            # I do not need a lock here, because this is the only place that next_frame is accessed
            self.__capture_frame(self.next_frame)
            # I only need the lock HERE. This is after the frame has been read, so I copy it into last_frame
            self.frame_lock.acquire_write()
            self.last_frame, self.next_frame = self.next_frame, self.last_frame
            self.frame_lock.release()
            # Set the flag for this event because we have now read at least one entire frame
            if not self.ready_event.is_set():
                self.ready_event.set()

            # This is how other threads are notified that a new frame is ready
            with self.event_lock:
                for event in self.events:
                    event.set()
            end_time = time.time()
            # print("Camera average FPS: {:>4.1f}".format(self.__num_iterations / (end_time - self.__start_time)))


# Double underscores make these values private
__camera = __CameraReader(__event_lock, __events)
__camera.start()


def add_event(event: threading.Event):
    """
    An event is a threading object, like a lock. When a thread calls event.wait(), it will block, until another
    thread calls event.set(). This is useful if another thread wants to wait until the camera has a new frame
    available. This module will call event.set() on every event that has been added to it as soon as each new
    frame is available.
    :param event: An event, which will be set as soon as a new frame is available
    :return: None
    """
    with __event_lock:
        __events.add(event)


def remove_event(event: threading.Event):
    """
    Removes an event, so this module will no longer set it when new frames are available.
    :param event: Event to be removed
    :return: None
    """
    with __event_lock:
        __events.remove(event)


def copy_last_frame(array: np.ndarray):
    """
    Copy the last-read frame into the provided array.
    This function is non-blocking*, so it will not wait until a new frame is available.
    *: Technically, it blocks for a very short time if it happens to be called at the exact moment that a new
    frame has just been read.
    :param array: A numpy array into which the last frame should be copied
    :return: None
    """
    __camera.ready_event.wait()
    __camera.frame_lock.acquire_read()
    np.copyto(array, __camera.last_frame)
    __camera.frame_lock.release()
