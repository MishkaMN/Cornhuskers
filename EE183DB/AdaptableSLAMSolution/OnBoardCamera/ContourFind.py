# from picamera.array import PiRGBArray
# from picamera import PiCamera
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
from time import sleep
objHeight = 50 #mm
f_app = 1783.166667
f = 3.04 # 4.5 # mm
sens_h = 2.76 #6.828 # mm
# d = 107.95 #millimeters

def locateObstacle(img):
    #_, threshold = cv2.threshold(img, 150, 255, cv2.THRESH_BINARY)
    #imgray = cv2.cvtColor(threshold, cv2.COLOR_BGR2GRAY);
    #contours, hierarchy = cv2.findContours(imgray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    lower_red2 = np.array([120, 60, 75])
    upper_red2 = np.array([179, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    
    mask = cv2.addWeighted(mask1, 1.0, mask2, 1.0, 0.0);
    
    isolated = cv2.bitwise_and(img,img, mask= mask)
    #cv2.imshow("mask", isolated)
    _, threshold = cv2.threshold(isolated, 80, 255, cv2.THRESH_BINARY)

    imgray = cv2.cvtColor(threshold, cv2.COLOR_BGR2GRAY);
    contours, hierarchy = cv2.findContours(imgray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Find the index of the largest contour
    areas = np.array([cv2.contourArea(c) for c in contours])
    cnts = [contours[i] for i in np.where(areas > 1000)[0]]

    locations = []
    for cnt in cnts:
        x,y,w,h = cv2.boundingRect(cnt)
        
        # DISTANCE CALC
        d2 = f_app*objHeight/h
        #d2 = f * objHeight / (sens_h * h/img.shape[0])
        #f_app = d * h / objHeight

        #ANGLE CALC
        px_from_center = (x+w/2) - img.shape[1]/2
        angle = np.arctan(px_from_center/f*sens_h/(img.shape[1]))
        locations.append((d2,angle,x,y,w,h))
    return locations

def pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi    

def motion_model(st, u, DT):
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * np.cos(st[2]), 0],
                  [DT * np.sin(st[2]), 0],
                  [0.0, DT]])

    v = np.zeros((2,1))
    
    if u[0,0] == 90:
        v[0,0] = 0.0
    elif (u[0,0] == 180): #FW
        v[0,0] = 228.1183511
    elif (u[0,0] == 83): #BW
        v[0,0] = -230.366008

    if u[1,0] == 90:
        v[1,0] = 0.0
    elif (u[1,0] == 85): #FW
        v[1,0] = 235.3996466
    elif (u[1,0] == 101): #BW
        v[1,0] = -208.6064064

    v = np.array([[(v[0,0] + v[1,0])/2], [1/123 * (v[1,0] - v[0,0])]])
    print(st)
    st = F @ st + np.transpose(B @ v)

    st[0,2] = pi_2_pi(st[0,2])
    return st    

def main():
    input("Press Enter to Start")

    # camera = PiCamera()
    # camera.vflip = True
    # rawCap = PiRGBArray(camera)
    # time.sleep(1)
    cap = cv2.VideoCapture("wk8.mp4")
    flag=True
    objPoseX = []
    objPoseY = []
    st = np.array([[0,0,np.pi/2]])
    u = np.array([[180], [85]])
    dt = 1.0/30.0
    plt.figure()
    frame = 0
    while(cap.isOpened()):
        # camera.capture(rawCap, format="bgr")
        # img = rawCap.array
        print(frame,flush=True)
        input
        if frame > 35 and frame < 50:
            st_dr = motion_model(st[-1,:], u, dt)
            st = np.vstack((st,st_dr));
        frame += 1

        ret, img = cap.read()
        if ret:
            locations = locateObstacle(img)

            
            
            #print("Frame:")
            for loc in locations:
                d2 = loc[0]
                angle = loc[1]
                #print(d2, angle*180/np.pi)
                x = loc[2]
                y = loc[3]  
                w = loc[4]
                h = loc[5]
                cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                cv2.putText(img, "angle"+str(angle*180/np.pi), (x, y), cv2.FONT_HERSHEY_TRIPLEX, 2, (0,255,0), lineType=cv2.LINE_AA) 
                cv2.putText(img, "Dist"+str(d2), (x, y-100), cv2.FONT_HERSHEY_TRIPLEX, 2, (0,255,0), lineType=cv2.LINE_AA)
                
                objPoseX.append(d2*np.sin(angle) + st[-1,0])
                objPoseY.append(d2*np.cos(angle) + st[-1,1])
        else:
            break
        cv2.imshow('Shapes', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
           break
        plt.cla()
        plt.scatter(objPoseX,objPoseY)
        plt.scatter(st[:,0],st[:,1])
        plt.pause(0.001)
        

    
    cap.release()
    cv2.destroyAllWindows()
        #rawCap.truncate(0)

if __name__ == "__main__":
    main()
