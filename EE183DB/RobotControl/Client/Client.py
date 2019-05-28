from ws4py.client.threadedclient import WebSocketClient
import time, requests
#import Kfilter
import numpy as np
import math

#esp8266host = "ws://192.168.0.104:81/"
import time
esp8266host = "ws://192.168.0.104:81/"
#esp8266host = "ws://192.168.50.133:81/"

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

if __name__ == '__main__':
    try:
        ws = DummyClient(esp8266host)
        ws.connect()
        print("Ready")

        print("Command Format: LeftPWM RightPWM")

        while(1):
            print("Send Command")
            command = input()
            sleeptime = int(command.split(' ')[2])
            command = dir_to_cmd(command)

            if command:
                ws.send(command)
                time.sleep(float(sleeptime)/1000.0)
                ws.send("90 90 100")

        ws.received_message()
        exit()

    except KeyboardInterrupt:
        ws.close()
