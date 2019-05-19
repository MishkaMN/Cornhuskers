from ws4py.client.threadedclient import WebSocketClient
import time, requests
import numpy as np
import math
import numpy as np
import cv2
import cv2.aruco as aruco
import random
import time
import csv
import datetime

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

def inBounds(envWidth, envLength, x, y):
    padding = 200
    return (x >= padding and x <= envWidth - padding and
            y >= padding and y <= envLength - padding)

if __name__ == '__main__':
    random.seed()
    try:
        ws = DummyClient(esp8266host)
        ws.connect()

        cap = cv2.VideoCapture(0)
        #envSize = 1060
        envLength = 1219
        envWidth = 914

        print("Starting...")

        filename = time.strftime("%Y-%m-%d %H:%M:%S") + '.csv'
        with open(filename, 'w') as csvfile:
            writer = csv.writer(csvfile)

            flag = False
            x = envWidth / 2
            y = envLength / 2
            theta = 0
            moving_to_center = False
            upper_angle = lower_angle = 0
            angle_padding = 5
            left_pwm = right_pwm = 90

            # PWMs
            motor_min = 82
            motor_max = 98

            # Millisecond times
            command_stop_time = 0
            command_time_min = 100
            command_time_max = 3000

            start_time = current_milli_time()
            total_duration = 3600 * 1000 # in MS
            end_time = start_time + total_duration

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

                    x,y = center
                    theta = angle*180/np.pi
                    print(x, y, theta)

                    #warp frames
                    frame = cv2.warpPerspective(frame,M,(envWidth,envLength))

                    # Identify blue obstacles
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    lower_blue = np.array([100,50,50])
                    upper_blue = np.array([140,255,255])
                    mask = cv2.inRange(hsv, lower_blue, upper_blue)
                    isolated_blue = cv2.bitwise_and(frame,frame, mask= mask)
                    _, threshold = cv2.threshold(isolated_blue, 80, 255, cv2.THRESH_BINARY)
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

                else:
                    flag = False

                if moving_to_center:
                    current_angle = theta

                    # Put angle between 0 and 360 degrees
                    if current_angle < 0:
                        current_angle += 360

                    # Check if robot is facing towards center (within a threshold)
                    if (((upper_angle < lower_angle) and (current_angle > upper_angle and current_angle < lower_angle)) or
                        (current_angle > lower_angle and current_angle < upper_angle)):

                        # Stop
                        ws.send('90 90')
                        moving_to_center = False

                        # Move forward
                        ws.send('98 82')
                        command_stop_time = current_milli_time() + 1000

                elif not inBounds(envWidth, envLength, x, y):
                    ws.send('90 90')
                    command_stop_time = 0

                    # Get angle from point of robot to center point
                    angle_to_center = math.degrees(math.atan2(envWidth - x / envLength - y))
                    current_angle = theta

                    # Put angles between 0 and 360 degrees
                    if angle_to_center < -180:
                        angle_to_center += 360
                    if current_angle < 0:
                        current_angle += 360

                    # Determine optimal rotation direction
                    diff = angle_to_center - current_angle
                    if diff < 0:
                        diff += 360
                    if diff > 180:
                        # turn CCW
                        command = '83 85'
                    else:
                        # turn CW
                        command = '180 101'

                    ws.send(command)

                    # Minimum threshold to consider robot to be "facing the center"
                    lower_angle = angle_to_center - angle_padding
                    if lower_angle < 0:
                        lower_angle += 360
                    upper_angle = angle_to_center + angle_padding
                    if upper_angle >= 360:
                        upper_angle -= 360

                elif current_milli_time() >= command_stop_time:
                    # Stop
                    ws.send('90 90')

                    # duration = randint(command_time_min, command_time_max)
                    duration = 10000

                    # Random inputs
                    left_pwm = random.choice([180, 83, 90])
                    right_pwm = random.choice([85, 101, 90])

                    # Drive motors
                    # command = str(left_pwm) + ' ' + str(right_pwm)
                    command = '180 85'
                    ws.send(command)

                    command_stop_time = current_milli_time() + duration

                    # print('Input ' + str(left_pwm) + ' ' + str(right_pwm) + ' for ' + str(duration) + ' ms')

                if not flag:
                    cv2.imshow('frame',frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                # Record data in CSV file
                current_time = (current_milli_time() - start_time) / 1000 # Time in seconds
                writer.writerow([current_time, left_pwm, right_pwm, x, y, theta])

                if current_time >= end_time:
                    break

        cap.release()
        cv2.destroyAllWindows()

    except KeyboardInterrupt:
        ws.close()
