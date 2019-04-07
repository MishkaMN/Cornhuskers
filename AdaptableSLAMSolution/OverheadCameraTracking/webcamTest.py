import numpy as np
import cv2
import cv2.aruco as aruco
 
 
cap = cv2.VideoCapture(1)
 
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    #print(frame.shape) #480x640
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
 
    #print(parameters)
 
    '''    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
        '''
        #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(ids)
    #It's working.
    # my problem was that the cellphone put black all around it. The alrogithm
    # depends very much upon finding rectangular black blobs
 
    #gray = aruco.drawDetectedMarkers(gray, corners)
    
    if not ids is None:
        center = (int((corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0])/4), int((corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1])/4))
        topCenter = (int((corners[0][0][0][0] + corners[0][0][1][0])/2), int( (corners[0][0][0][1] + corners[0][0][1][1])/2))
        
        cv2.line(frame, center, topCenter, (0,255,0), 3)

        pts1 = np.float32([corners[0][0][0], corners[0][0][1], corners[0][0][3], corners[0][0][2]])
        pts2 = np.float32([[0,0],[300,0],[0,300],[300,300]])
        M = cv2.getPerspectiveTransform(pts1,pts2)
        dst = cv2.warpPerspective(frame,M,(300,300))
        frame = dst

    #print(rejectedImgPoints)
    # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
 
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()