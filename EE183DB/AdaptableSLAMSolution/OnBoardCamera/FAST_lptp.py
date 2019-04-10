import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
# img = cv.imread('simple2.jpg',0)
import cv2.aruco as aruco
import timer
 
# Uses FAST and BRIEF
def image_detect_and_compute(detector, descriptor, img):
    """Detect and compute interest points and their descriptors."""

    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    kp = detector.detect(img, None)
    kp, des = descriptor.compute(img,kp)
    #print(des)
    return kp, des


def draw_image_matches(detector, descriptor, img1, img2, nmatches=10):
    """Draw ORB feature matches of the given two images."""
    kp1, des1 = image_detect_and_compute(detector, descriptor, img1)
    kp2, des2 = image_detect_and_compute(detector, descriptor, img2)
    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)
    matches = sorted(matches, key = lambda x: x.distance) # Sort matches by distance.  Best come first.
    
    img_matches = cv.drawMatches(img1, kp1, img2, kp2, matches[:nmatches], img2, flags=2) # Show top 10 matches
    #plt.figure(figsize=(16, 16))
    #plt.title(type(detector))
    cv.imshow('frames',img_matches); #plt.show()
    
cap = cv.VideoCapture(0)
ret, img = cap.read()
img_prev = np.copy(img)
while(True):
    # Capture frame-by-frame
    ret, img = cap.read()
    # Initiate FAST object with default values
    fast = cv.FastFeatureDetector_create()
    brief = cv.xfeatures2d.BriefDescriptorExtractor_create()

    # Try to change threshold:
    fast.setThreshold(10)

    # Try to get name?
    # print( fast.getDefaultName())

    # find and draw the keypoints
    # img2 = cv.drawKeypoints(img, kp, None, color=(255,0,0))

    draw_image_matches(fast, brief, img, img_prev)
    
    img_prev = np.copy(img)

    #cv.imshow('frame',img2)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
 
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()