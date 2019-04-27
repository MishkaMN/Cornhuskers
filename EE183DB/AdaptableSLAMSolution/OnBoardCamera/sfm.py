import numpy as np
import cv2 as cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import time
import math
from picamera import PiCamera

# cx = 640 / 2 # half horizontal resolution
# cy = 480 / 2 # half vertical resolution
# fov_x = 62.2 * math.pi / 180 # horizontal fov
# fov_y = 48.8 * math.pi / 180 # vertical fov
# s = 0 # skew, usually 0
# fx = cx / math.tan(fov_x/2) #horizontal focal len
# fy = cy / math.tan(fov_y/2) #vertical focal len
# #pi camera matrix
# K = np.array([[fx,s,cx],[0,fy,cy],[0,0,1]])

K = np.loadtxt("cameraMatrix.txt")

# initialize the camera
camera = PiCamera()
rawCapture = PiRGBArray(camera)

camera.start_preview(fullscreen=False, window=(0,0,640,360))
print("Taking first image in 5 seconds")
sleep(5)
camera.capture(rawCapture, format="bgr")
img1 = rawCapture.array
img1=cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
print("Taking second image in 5 seconds")
sleep(5)
camera.capture(rawCapture, format="bgr")
img2 = rawCapture.array
img2=cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
print("Images taken")
camera.stop_preview()

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
kp1 = np.float32([kp1[mat.queryIdx].pt for mat in matches])
kp2 = np.float32([kp2[mat.trainIdx].pt for mat in matches])


E, mask = cv2.findEssentialMat(kp1, kp2, K,  method=cv2.RANSAC)

points, R, t, mask = cv2.recoverPose(E, kp1, kp2)

#https://stackoverflow.com/questions/33906111/how-do-i-estimate-positions-of-two-cameras-in-opencv

M_r = np.hstack((R, t))
M_l = np.hstack((np.eye(3, 3), np.zeros((3, 1))))

P_l = np.dot(K,  M_l)
P_r = np.dot(K,  M_r)
point_4d_hom = cv2.triangulatePoints(P_l, P_r, np.expand_dims(kp1, axis=1), np.expand_dims(kp2, axis=1))
point_4d = point_4d_hom / np.tile(point_4d_hom[-1, :], (4, 1))
point_3d = point_4d[:3, :].T

print(point_3d)

fig = plt.figure()
ax = Axes3D(fig)
ax.scatter(point_3d[:,0], point_3d[:,1], point_3d[:,2])
plt.show()

if cv2.waitKey(1) & 0xFF == ord('q'):
    break
