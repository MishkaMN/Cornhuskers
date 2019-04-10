import numpy as np
import cv2 as cv2
import matplotlib.pyplot as plt
import time

cap = cv2.VideoCapture(1)

while(True):
    start = time.time()
    _, img1 = cap.read()
    _, img2 = cap.read()

    # Initiate ORB detector
    orb = cv2.ORB_create()
    # find the keypoints and descriptors with ORB
    kp1, des1 = orb.detectAndCompute(img1,None)
    kp2, des2 = orb.detectAndCompute(img2,None)

    # create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    # Match descriptors.
    matches = bf.match(des1,des2)
    # Sort them in the order of their distance.
    matches = sorted(matches, key = lambda x:x.distance)
    # Draw first 10 matches.
    img3 = cv2.drawMatches(img1,kp1,img2,kp2,matches[:10],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    cv2.imshow('frame', img3)
    print(time.time() - start)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
