#import cv2
from cv2 import cv2

import numpy as np
import time


cap = cv2.VideoCapture(0)

# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.1,
                       minDistance = 3,
                       blockSize = 64 )
# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (128,128),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
# Create some random colors
color = np.random.randint(0,255,(100,3))
# Take first frame and find corners in it
ret, old_frame = cap.read()
old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
# Create a mask image for drawing purposes
mask = np.zeros_like(old_frame)
totaldiff = np.zeros([2])
while True:
    ret,frame = cap.read()
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # calculate optical flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
    # Select good points
    good_new = p1[st==1]
    good_old = p0[st==1]


    #print("Old: {}, new: {}".format(good_old.shape, good_new.shape))

    diff = good_new - good_old
    mean = np.mean(diff, axis=0)
    totaldiff += mean
    print("Mean: {}, total: {}".format(mean, totaldiff))
    img = cv2.add(frame,mask)
    #cv2.imshow('frame',img)
    #k = cv2.waitKey(30) & 0xff
    #if k == 27:
    #    break
    # Now update the previous frame and previous points
    old_gray = frame_gray.copy()
    p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)
cv2.destroyAllWindows()
cap.release()
