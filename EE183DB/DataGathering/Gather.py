from ws4py.client.threadedclient import WebSocketClient
import time, requests
import numpy as np
import math
import numpy as np
import cv2
import cv2.aruco as aruco
import time
import datetime
import csv

current_milli_time = lambda: int(round(time.time() * 1000))

#esp8266host = "ws://192.168.0.104:81/"
esp8266host = "ws://192.168.50.133:81/"

command = ""

class DummyClient(WebSocketClient):
    def __init__(self, host):
        super(DummyClient, self).__init__(host)
        #self.est_state = np.array([0, Kfilter.W/2, Kfilter.L/2]);
        #self.P = np.eye(3)
        #self.z_init = np.array([0,0,0])
        #self.command = np.array([0,0,0])
        #self.z_final = np.array([0,0,0])
    def opened(self):
        print("Socket Opened")
    def closed(self, code, reason=None):
        print("Socket Closed", code, reason);
    def received_message(self, msg):
        parts = str(msg).split(",")
        if(parts[0] == "Last"):
            frontSense = (float(parts[1])-62.4)/.937
            sideSense = (float(parts[2])-41.7)/.972
            theta = float(parts[3])

            self.z_final = [theta, frontSense, sideSense]
            #print(self.z_final)
            """ START FILTERING """
            #print("State:")
            #print(self.est_state[0]*180.0/math.pi, self.est_state[1], self.est_state[2])
            pwmL, pwmR, dt = command.split(" ")
            self.est_state, self.P = Kfilter.aPrioriUpdate(self.est_state, float(dt)/1000.0, self.P, float(pwmR), float(pwmL))
            #print(self.est_state)
            self.est_state, self.P = Kfilter.aPosterioriUpdate(self.P, self.z_final, self.est_state, float(dt)/1000.0)
            print("Filtered State")
            print(self.est_state[0]*180.0/math.pi, self.est_state[1], self.est_state[2])
            #print(self.P)

def dir_to_cmd(command):
    tmp = command.split()
    dirs = ['f', 'r', 'b', 'l','a', 'w', 's', 'd' ]
    if tmp[0] in dirs:
        if tmp[0] == 'f' or tmp[0] == 'w':
            cmd = '180 0 ' + tmp[1]
        elif tmp[0] == 'b' or tmp[0] == 's':
            cmd = '0 180 ' + tmp[1]
        elif tmp[0] == 'r' or tmp[0] == 'd':
            cmd = '180 180 '+ tmp[1]
        elif tmp[0] == 'l' or tmp[0] == 'a':
            cmd = '0 0 '+ tmp[1]
    else:
        cmd = command
    return cmd

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

    data = []

    try:
        ws = DummyClient(esp8266host)
        ws.connect()

        cap = cv2.VideoCapture(1)

        envLength = 1219
        envWidth = 914

        l_inputs = [83, 90, 180]
        r_inputs = [101, 90, 85]

        l_stop, r_stop = [90, 90]

        command_times = range(100, 1500 + 100, 100)

        commands = []
        l_len = len(l_inputs)
        r_len = len(r_inputs)
        for l_idx in range(0, l_len):
            for r_idx in range(0, r_len):
                for t in command_times:
                    commands.append((l_inputs[l_idx], r_inputs[r_idx], t, True))
                    commands.append((l_stop, r_stop, 500, False))
                    commands.append((l_inputs[l_len - l_idx - 1], r_inputs[r_len - r_idx - 1], t, False))
                    commands.append((l_stop, r_stop, 500, False))

        print("Starting...")


        flag = False
        x = envWidth / 2
        y = envLength / 2
        theta = 0
        l_input = r_input = 90
        record = False

        command_stop_time = 0
        start_time = current_milli_time()

        trial_num = 0
        filename = time.strftime("%Y%m%d_%H%M%S") + '.csv'
        with open(filename, 'w') as csvfile:
            writer = csv.writer(csvfile)
            while(commands):
                ret, frame = cap.read()

                #detect aruco tags and find corners
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
                parameters =  aruco.DetectorParameters_create()
                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

                if (not ids is None) and (len(ids) == 5):
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

                    x,y = center
                    theta = angle*180/np.pi
                    print(x, y, theta)

                    #warp frames
                    frame = cv2.warpPerspective(frame,M,(envWidth,envLength))

                    # Identify red obstacles
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    lower_red1 = np.array([0, 200, 100])
                    upper_red1 = np.array([10, 255, 255])
                    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
                    lower_red2 = np.array([160, 100, 100])
                    upper_red2 = np.array([179, 255, 255])
                    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
                    mask = cv2.addWeighted(mask1, 1.0, mask2, 1.0, 0.0);
                    isolated = cv2.bitwise_and(frame, frame, mask= mask)
                    #cv2.imshow("mask", isolated)
                    _, threshold = cv2.threshold(isolated, 80, 255, cv2.THRESH_BINARY)
                    imgray = cv2.cvtColor(threshold, cv2.COLOR_BGR2GRAY);
                    contours, hierarchy = cv2.findContours(imgray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                    # Find the index of the largest contour
                    areas = np.array([cv2.contourArea(c) for c in contours])
                    cnts = [contours[i] for i in np.where(areas > 10)[0]]
                    for cnt in cnts:
                        bx,by,bw,bh = cv2.boundingRect(cnt)
                        cv2.rectangle(frame,(bx,by),(bx+bw,by+bh),(0,255,0),2)
                        print( (bx+bw/2, envLength - (by+bh/2) ))

                    cv2.line(frame, (int(center[0]), int(envLength - center[1])), (int(topCenter[0]), int(topCenter[1])), (0,255,0), 3)

                    cv2.imshow('frame',frame)
                current_time = (current_milli_time() - start_time) / 1000 # Time in seconds
                current_data = np.array([trial_num, current_time, np.array([l_input, r_input]), np.array([x, y, theta])])
                #data.append(current_data)
                if record:
                    writer.writerow([trial_num, current_time, l_input, r_input, x, y, theta])

                if current_milli_time() >= command_stop_time:
                    # Get next command
                    l_input, r_input, duration, record = commands.pop(0)

                    trial_num += 1

                    # Drive motors
                    command = str(l_input) + ' ' + str(r_input)
                    ws.send(command)

                    command_stop_time = current_milli_time() + duration

                    print('Input ' + str(l_input) + ' ' + str(r_input) + ' for ' + str(duration) + ' ms')

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            print('All trials complete')
            print('Total inputs: ', trial_num + 1)

    except KeyboardInterrupt:
        ws.send('90 90')

    finally:
        ws.send('90 90')
        ws.close()
        cap.release()
        cv2.destroyAllWindows()
        # filename = time.strftime("%Y%m%d_%H%M%S")
        #data = np.array(data)
        #np.save(filename, data)
