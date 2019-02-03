from ws4py.client.threadedclient import WebSocketClient
import time, requests
import Kfilter
import numpy as np
import math

esp8266host = "ws://192.168.50.133:81/"

command = ""

class DummyClient(WebSocketClient):
    def __init__(self, host):
        super(DummyClient, self).__init__(host)
        self.est_state = np.array([0, Kfilter.W/2, Kfilter.L/2]);
        self.P = np.eye(3)
        self.z_init = np.array([0,0,0])
        self.command = np.array([0,0,0])
        self.z_final = np.array([0,0,0])
    def opened(self):
        print("Socket Opened")
    def closed(self, code, reason=None):
        print("Socket Closed", code, reason);
    def received_message(self, msg):
        parts = str(msg).split(",")
        if(parts[0] == "Last"):
            print(parts[1])
            theta = (-1*float(parts[1]) + 290.0) * math.pi/180.0;
            print(theta)
            self.z_final = [theta, float(parts[2]), float(parts[3])]
            #print(self.z_final)
            """ START FILTERING """
            #print("State:")
            #print(self.est_state[0]*180.0/math.pi, self.est_state[1], self.est_state[2])
            pwmL, pwmR, dt = command.split(" ")
            self.est_state, self.P = Kfilter.aPrioriUpdate(self.est_state, float(dt)/1000.0, self.P, float(pwmR), float(pwmL))
            #print(self.est_state)
            self.est_state, self.P = Kfilter.aPosterioriUpdate(self.P, self.z_final, self.est_state, self.command[2])
            #print("Filtered State")
            print(self.est_state[0]*180.0/math.pi, self.est_state[1], self.est_state[2])
            #print(self.P)
            

        

if __name__ == '__main__':
    try:
        ws = DummyClient(esp8266host)
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