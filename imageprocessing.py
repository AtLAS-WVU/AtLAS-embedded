from cv2 import cv2

import numpy as np
import time
from config import CAMERA_RESOLUTION
import threading
import camera


if __name__ == "__main__":

    event = threading.Event()
    camera.add_event(event)

    # Parameters for extracting features to track.
    # See https://docs.opencv.org/3.0-beta/modules/imgproc/doc/feature_detection.html#goodfeaturestotrack
    feature_params = {  # TODO: Tweak these values for performance and accuracy
        'maxCorners': 50,
        'qualityLevel': 0.05,
        'minDistance': 3,
        'blockSize': 64
    }

    # Parameters for calculating optical flow.
    # See https://docs.opencv.org/3.0-beta/modules/video/doc/motion_analysis_and_object_tracking.html
    lk_params = {  # TODO: Tweak these values for performance and accuracy
        'winSize': (64, 64),
        'maxLevel': 2,
        'criteria': (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
    }

    # Pre-allocate two frames of the image, so I don't have to allocate an entire new array for every frame
    new_frame = np.zeros(CAMERA_RESOLUTION, dtype=np.uint8)
    old_frame = np.zeros_like(new_frame)

    camera.copy_last_frame(new_frame)
    new_points = cv2.goodFeaturesToTrack(new_frame.transpose(), mask=None, **feature_params)

    # Represents the total motion in the frame since the program started running.
    # Only useful for debugging, since in real use, we'll use the relative motion over the past few frames
    total_diff = np.zeros([2])

    total_time = 0
    num_iterations = 0

    while True:
        event.wait()
        event.clear()
        num_iterations += 1

        # Used only for timing, to see how processor intensive this stuff is.
        start_time = time.time()

        # Swap the new and old frames.
        # This would accomplish the same thing:
        #   old_frame = new_frame
        #   new_frame = np.zeros_like(old_frame)
        # But with the downside of allocating an entire new numpy array every frame.
        old_frame, new_frame = new_frame, old_frame

        # This function finds good "tracking points" in the image, such as sharp corners. Returns a numpy array of
        # shape (n, 2), where n is the number of points found.
        old_points = cv2.goodFeaturesToTrack(old_frame.transpose(), mask=None, **feature_params)
        camera.copy_last_frame(new_frame)

        # This function looks at the tracking points in the previous frame, and tries to find the same points in the
        # new frame. The output new_points is a numpy array of shape (n, 2), where n is the same n in old_points.
        # status is a numpy array of shape (n), where each entry is 1 if the corresponding point was successfully found
        # in the new frame, or 0 if the tracking point was lost.
        # I don't fully understand what error is, but I don't think I have to.
        # TODO: Check to make sure there are more than 0 points in old_points, or else bad error happens.
        new_points, status, error = cv2.calcOpticalFlowPyrLK(old_frame.transpose(), new_frame.transpose(), old_points,
                                                             None, **lk_params)

        # This filters out the points so that we are left with only points that were found in both the old and new frame
        old_points = old_points[status == 1]
        new_points = new_points[status == 1]

        # This code draws the frames with tracking points overlaid.
        frame_drawing = np.copy(new_frame)
        for old_point, new_point in zip(old_points, new_points):
            frame_drawing[int(old_point[0]), int(old_point[1])] = 0
            frame_drawing[int(new_point[0]), int(new_point[1])] = 255
        cv2.imshow('frame', frame_drawing.transpose())
        if (cv2.waitKey(2) & 0xFF) == 27:
            break

        # Calculate the movement of the points between the old and new frames
        diff_points = new_points - old_points

        average_diff = np.mean(diff_points, axis=0)
        if not np.isnan(average_diff).any():
            total_diff += average_diff

        # Just for seeing how slow this runs
        end_time = time.time()
        total_time += (end_time - start_time)

        print("Num points: ({0[0]:>3d}, {0[1]:>1d}), Diff this frame: ({1[0]:>7.2f}, {1[1]:>7.2f}), "
              "Total diff: ({2[0]:>7.2f}, {2[1]:>7.2f}), Avg time: {3:>8.6f}"
              .format(new_points.shape, average_diff, total_diff, total_time / num_iterations))
