import numpy as np
import cv2
import cv2.aruco as aruco
import math

#helper function: find center of aruco tag from corners
def getCenter(corners, i, j):
    center = (int((corners[i][j][0][0] + corners[i][j][1][0] + corners[i][j][2][0] + corners[i][j][3][0])/4), int((corners[i][j][0][1] + corners[i][j][1][1] + corners[i][j][2][1] + corners[i][j][3][1])/4))
    return center

#helper function: find center and direction vector from aruco tag
def getPose(corners, pMatrix):
    center = np.array(getCenter(corners, 4, 0), dtype='float32')
    topCenter = np.array( ((corners[4][0][0][0] + corners[4][0][1][0])/2, (corners[4][0][0][1] + corners[4][0][1][1])/2) , dtype='float32')
    pts = np.array([np.array([center, topCenter], dtype='float32')])
    newPts = cv2.perspectiveTransform(pts, pMatrix)
    center = newPts[0][0]
    topCenter = newPts[0][1]
    vec = topCenter - center
    return center,topCenter,vec

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    envLength = 1219
    envWidth = 914
    print("Starting...")
    flag = False
    while(True):

        ret, frame = cap.read()

        #detect aruco tags and find corners
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if (not ids is None) and (len(ids) == 5):
            flag = True
            sortedCorners =  [x for _,x in sorted(zip(ids,corners))]

            #find centers of environment tags
            topLeft = getCenter(sortedCorners, 0, 0)
            topRight = getCenter(sortedCorners, 1, 0)
            bottomLeft = getCenter(sortedCorners, 2, 0)
            bottomRight = getCenter(sortedCorners, 3, 0)

            #Perspective transform to correct for angle of camera
            pts1 = np.float32([topLeft, topRight, bottomLeft, bottomRight])
            pts2 = np.float32([[0,0],[envWidth,0],[0,envLength],[envWidth,envLength]])
            M = cv2.getPerspectiveTransform(pts1,pts2)

            #perform pose estimates
            center, topCenter, vec = getPose(sortedCorners,M)
            center = (center[0], envLength - center[1])
            angle = np.arctan2(-1*vec[1], vec[0])
            print(center, angle*180/np.pi)

            cx, cy = center
            angle_to_center = math.degrees(math.atan2(530 - cy, 530 - cx))
            print(angle_to_center)

            #warp frames
            frame = cv2.warpPerspective(frame,M,(envWidth,envLength))

            # Identify blue obstacles
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_blue = np.array([90,50,50])
            upper_blue = np.array([110,255,255])
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            isolated_blue = cv2.bitwise_and(frame,frame, mask= mask)
            _, threshold = cv2.threshold(isolated_blue, 80, 255, cv2.THRESH_BINARY)
            imgray = cv2.cvtColor(threshold, cv2.COLOR_BGR2GRAY);
            contours, hierarchy = cv2.findContours(imgray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # Find the index of the largest contour
            areas = np.array([cv2.contourArea(c) for c in contours])
            cnts = [contours[i] for i in np.where(areas > 10)[0]]
            for cnt in cnts:
                x,y,w,h = cv2.boundingRect(cnt)
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                print( (x+w/2, envLength - (y+h/2) ))

            cv2.line(frame, (int(center[0]), int(envLength - center[1])), (int(topCenter[0]), int(topCenter[1])), (0,255,0), 3)
            cv2.imshow('frame',frame)

        else:
            flag = False

        if not flag:
            cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
