import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
img = cv.imread('simple2.jpg',0)
import cv2.aruco as aruco
 
 
cap = cv.VideoCapture(0)
 
while(True):
	# Capture frame-by-frame
	ret, img = cap.read()

	# Initiate FAST object with default values
	fast = cv.FastFeatureDetector_create()

	# Try to change threshold:
	fast.setThreshold(10)

	# Try to get name?
	print( fast.getDefaultName())

	# find and draw the keypoints
	kp = fast.detect(img,None)

	img2 = cv.drawKeypoints(img, kp, None, color=(255,0,0))

	# Print all default params
	# print( "Threshold: {}".format(fast.getThreshold()) )
	# print( "nonmaxSuppression:{}".format(fast.getNonmaxSuppression()) )
	# print( "neighborhood: {}".format(fast.getType()) )
	# print( "Total Keypoints with nonmaxSuppression: {}".format(len(kp)) )
	# cv.imwrite('fast_true2.png',img2)

	# # Disable nonmaxSuppression
	# fast.setNonmaxSuppression(0)
	# kp = fast.detect(img,None)
	# print( "Total Keypoints without nonmaxSuppression: {}".format(len(kp)) )
	# img3 = cv.drawKeypoints(img, kp, None, color=(255,0,0))
	# cv.imwrite('fast_false2.png',img3)

	cv.imshow('frame',img2)
	if cv.waitKey(1) & 0xFF == ord('q'):
		break
 
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()