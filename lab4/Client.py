from ws4py.client.threadedclient import WebSocketClient
import time, requests
import Kfilter
import numpy as np
import math

# Yikenet- 192.168.50.133
# 8HZ-Wan-IP : 192.168.1.36
esp8266host = "ws://192.168.50.133:81/"

command = ""

class RobotClient(WebSocketClient):
    def __init__(self, host):
        super(RobotClient, self).__init__(host)
        self.est_state = np.array([0, Kfilter.W/2, Kfilter.L/2]);
        self.P = np.eye(3)
        self.z_init = np.array([0,0,0])
        self.command = ""
        self.z_final = np.array([0,0,0])
    def opened(self):
        print("Socket Opened")
    def closed(self, code, reason=None):
        print("Socket Closed", code, reason);
    def received_message(self, msg):
        """
        parts = str(msg).split(",")
        if(parts[0] == "Command"):
            self.command = str(parts[1]) + " " + str(parts[2]) + " " + str(parts[3])
        if(parts[0] == "Last"):
            frontSense = (float(parts[1])-62.4)/.937
            sideSense = (float(parts[2])-41.7)/.972
            theta = float(parts[3])
            
            self.z_final = [theta, frontSense, sideSense]
            #print(self.z_final)
            """ """START FILTERING
            #print("State:")
            #print(self.est_state[0]*180.0/math.pi, self.est_state[1], self.est_state[2])
            pwmL, pwmR, dt = self.command.split(" ")
            self.est_state, self.P = Kfilter.aPrioriUpdate(self.est_state, float(dt)/1000.0, self.P, float(pwmR), float(pwmL))
            #print(self.est_state)
            self.est_state, self.P = Kfilter.aPosterioriUpdate(self.P, self.z_final, self.est_state, float(dt)/1000.0)
            #print("Filtered State")
            #print(self.est_state[0]*180.0/math.pi, self.est_state[1], self.est_state[2])
            #print(self.P)
        """

        

if __name__ == '__main__':
    try:
        ws = RobotClient(esp8266host)
        ws.connect()
        print("Ready")

        while(1):
            print("Send Command")
            command = input();
            ws.send(command)

        ws.received_message()
        ws.close()
        exit()

    except KeyboardInterrupt:
        ws.close()