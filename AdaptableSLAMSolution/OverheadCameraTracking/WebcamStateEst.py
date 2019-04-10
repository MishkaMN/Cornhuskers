import numpy as np
import cv2
import cv2.aruco as aruco
 
cap = cv2.VideoCapture(1)
envSize = 300

while(True):

    ret, frame = cap.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
 

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    def getCenter(corners, i, j):
        center = (int((corners[i][j][0][0] + corners[i][j][1][0] + corners[i][j][2][0] + corners[i][j][3][0])/4), int((corners[i][j][0][1] + corners[i][j][1][1] + corners[i][j][2][1] + corners[i][j][3][1])/4))
        return center

    def getPose(corners, pMatrix):        
        center = np.array(getCenter(corners, 4, 0), dtype='float32')
        topCenter = np.array( ((corners[4][0][0][0] + corners[4][0][1][0])/2, (corners[4][0][0][1] + corners[4][0][1][1])/2) , dtype='float32')
        pts = np.array([np.array([center, topCenter], dtype='float32')])
        newPts = cv2.perspectiveTransform(pts, pMatrix)
        center = newPts[0][0]
        topCenter = newPts[0][1]
        vec = topCenter - center
        return center,topCenter,vec


    if (not ids is None) and (len(ids) == 5):
        sortedCorners =  [x for _,x in sorted(zip(ids,corners))]
        
        topLeft = getCenter(sortedCorners, 0, 0)
        topRight = getCenter(sortedCorners, 1, 0)
        bottomLeft = getCenter(sortedCorners, 2, 0)
        bottomRight = getCenter(sortedCorners, 3, 0)

        pts1 = np.float32([topLeft, topRight, bottomLeft, bottomRight])
        pts2 = np.float32([[0,0],[envSize,0],[0,envSize],[envSize,envSize]])
        M = cv2.getPerspectiveTransform(pts1,pts2)
        center, topCenter, vec = getPose(sortedCorners,M)
        center = (center[0], envSize - center[1])
        angle = np.arctan2(-1*vec[1], vec[0])
        print(center, angle*180/np.pi)
        frame = cv2.warpPerspective(frame,M,(envSize,envSize))
        cv2.line(frame, (int(center[0]), int(center[1])), (int(topCenter[0]), int(topCenter[1])), (0,255,0), 3)

        cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()