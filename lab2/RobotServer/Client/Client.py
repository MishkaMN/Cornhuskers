from ws4py.client.threadedclient import WebSocketClient
import time, requests

esp8266host = "ws://192.168.43.240:81/"

class DummyClient(WebSocketClient):
    def opened(self):
        print("Socket Opened")
    def closed(self, code, reason=None):
        print("Socket Closed", code, reason);
    def received_message(self, msg):
        print(msg)

if __name__ == '__main__':
    try:
        ws = DummyClient(esp8266host)
        ws.connect()
        print("Ready !")
        
        i = 0
        while(1):
            ws.send("state")

        ws.send("led0:0")
        ws.received_message()
        ws.close()
        exit()

    except KeyboardInterrupt:
        ws.send("led0:0")
        ws.close()