from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
objHeight = 54 #mm
f_app = 2277.68014059754 
f = 3.04 # 4.5 # mm
sens_h = 2.76 #6.828 # mm
# d = 107.95 #millimeters

def locateObstacle(img):
    #_, threshold = cv2.threshold(img, 150, 255, cv2.THRESH_BINARY)
    #imgray = cv2.cvtColor(threshold, cv2.COLOR_BGR2GRAY);
    #contours, hierarchy = cv2.findContours(imgray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([90,50,50])
    upper_blue = np.array([110,255,255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    isolated_blue = cv2.bitwise_and(img,img, mask= mask)
    _, threshold = cv2.threshold(isolated_blue, 80, 255, cv2.THRESH_BINARY)
    imgray = cv2.cvtColor(threshold, cv2.COLOR_BGR2GRAY);
    contours, hierarchy = cv2.findContours(imgray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Find the index of the largest contour
    areas = np.array([cv2.contourArea(c) for c in contours])
    cnts = [contours[i] for i in np.where(areas > 10000)[0]]

    locations = []
    for cnt in cnts:
        x,y,w,h = cv2.boundingRect(cnt)
        
        # DISTANCE CALC
        #d = f_app*objHeight/h
        d2 = f * objHeight / (sens_h * h/img.shape[0])
        #f_app = d * h / objHeight
        #print(d, d2, f_app)

        #ANGLE CALC
        px_from_center = (x+w/2) - img.shape[1]/2
        angle = np.arctan(px_from_center/f*sens_h/(img.shape[1]))
        locations.append((d2,angle,x,y,w,h))
    return locations

    
def main():
    input("Press Enter to Start")

    #cap = cv2.VideoCapture('LMarkVideo.mp4')
    camera = PiCamera()
    camera.vflip = True
    rawCap = PiRGBArray(camera)
    time.sleep(1)
    flag=True
    while(1):
        startTime = time.time()
        #ret, img = cap.read()
        camera.capture(rawCap, format="bgr")
        img = rawCap.array
        locations = locateObstacle(img)

        objPoseX = []
        objPoseY = []
        print("Frame:")
        for loc in locations:
            #print(angle*180/np.pi)
            d2 = loc[0]
            angle = loc[1]
            print(d2,angle)
            x = loc[2]
            y = loc[3]
            w = loc[4]
            h = loc[5]
            cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
            cv2.putText(img, "angle"+str(angle*180/np.pi), (x, y), cv2.FONT_HERSHEY_TRIPLEX, 2, (0,255,0), lineType=cv2.LINE_AA) 
            cv2.putText(img, "Dist"+str(d2), (x, y-100), cv2.FONT_HERSHEY_TRIPLEX, 2, (0,255,0), lineType=cv2.LINE_AA)
            objPoseX.append(d2*np.sin(angle))
            objPoseY.append(d2*np.cos(angle))

        #plt.cla()
        #plt.scatter(objPoseX, objPoseY)
        bottom, top = plt.ylim()
        fovDistX = np.max(plt.xlim())
        #plt.xlim(-1.05*fovDistX,1.05*fovDistX)
        #plt.ylim(0-top*.05,top*1.05)
        #plt.scatter([0],[0], color="red")

        #for i,_ in enumerate(objPoseX):
            #plt.plot(np.array((0, objPoseX[i])),np.array((0, objPoseY[i])), '--r')
        #cv2.imshow("shapes", img)
        cv2.imwrite("tmp.jpg", img)
        
        #plt.pause(0.000001)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #    break
        if flag:
            input("Press Enter to Continue")
            flag=False
            exit()
        rawCap.truncate(0)
#        print(time.time()-startTime, "Processing Time")


    #cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
