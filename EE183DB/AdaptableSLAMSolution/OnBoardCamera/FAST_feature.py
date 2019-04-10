import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
 
# we used 4.0.1 opencv
print(cv.__version__)
# initialize the camera
camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 32
# grab a reference to the raw camera capture
rawCapture = PiRGBArray(camera,size=(640,480))

 # allow the camera to warmup
time.sleep(0.1)


for frame in camera.capture_continuous(rawCapture, format ="bgr", use_video_port = True):
    # get a frame
    img = frame.array

    # init fast detector
    fast = cv.FastFeatureDetector_create()

    # pixel intensity difference threshold. if more than this threshold, it is picked up as a feature
    fast.setThreshold(10)

    # find and draw the keypoints
    kp = fast.detect(img,None)
    img2 = cv.drawKeypoints(img, kp, None, color=(255,0,0))

    # show the video
    cv.imshow('frame',img2)

    # q to quit
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
    # Clear the stream in preparation for the next frame
    rawCapture.truncate(0)

