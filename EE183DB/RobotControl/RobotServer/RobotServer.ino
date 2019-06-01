#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <Servo.h>
#include "Drivetrain.h"

Servo servoLeft, servoRight;

//const char* ssid     = "Kelton 211 (2G)";
//const char* password = "interesting";

const char* ssid     = "YikeNet_2G";
const char* password = "luckytrain022";

WebSocketsServer webSocket = WebSocketsServer(81);

void parseCommand(String command, int* commandParts)
{
  char * strtokIndx;
  char * comm_cstr = (char *)command.c_str();
  strtokIndx = strtok(comm_cstr,", ");
  commandParts[0] = atoi(strtokIndx);
  strtokIndx = strtok(NULL,", ");
  commandParts[1] = atoi(strtokIndx);
  strtokIndx = strtok(NULL,", ");
  commandParts[2] = atoi(strtokIndx);
  Serial.print(commandParts[2]);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t len) {
    switch(type) {
        case WStype_DISCONNECTED:
            break;
        case WStype_CONNECTED:
            {
              IPAddress ip = webSocket.remoteIP(num);
              Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\r\n", num, ip[0], ip[1], ip[2], ip[3], payload);
            }
            break;
        case WStype_TEXT:
            {
              String _payload = String((char *) &payload[0]);
              int commandArgs[2];
              char data[64];

              parseCommand(_payload, commandArgs);
              drive(commandArgs[0], commandArgs[1], servoLeft, servoRight);
              delay(commandArgs[2]);
              drive(90,90,servoLeft,servoRight);
              sprintf(data, "Command,%d,%d,%d", commandArgs[0], commandArgs[1]);
              webSocket.sendTXT(num, data);
            }
            break;
        case WStype_BIN:
            {
              hexdump(payload, len);
            }
            // echo data back to browser
            webSocket.sendBIN(num, payload, len);
            break;

    }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED) {
     Serial.print(".");
     delay(200);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  delay(500);

  Serial.println("Start Websocket Server");
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  //Setup Servos:
  servoLeft.attach(SERVO_LEFT);
  servoRight.attach(SERVO_RIGHT);

  delay(3000);

}

void loop() {
  webSocket.loop();
  
}
