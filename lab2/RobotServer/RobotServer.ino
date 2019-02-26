#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <Servo.h>
#include "Drivetrain.h"
#include "Sensors.h"

VL53L0X fSensor, sSensor;
Servo servoLeft, servoRight;

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
              int commandArgs[3];

              //Initial sensor readings
              float f,s, gz, head;
              ReadDistSensors(f,s,fSensor,sSensor);
              ReadIMU(gz,head);
              Serial.print(head); Serial.print("\n");
              char data[64];
              sprintf(data, "First,%f,%f,%f", f, s, head);
              webSocket.sendTXT(num, data);
              
              parseCommand(_payload, commandArgs);
              Serial.print(commandArgs);
              drive(commandArgs[0], commandArgs[1], commandArgs[2], servoLeft, servoRight);
              sprintf(data, "Command,%d,%d,%d", commandArgs[0], commandArgs[1], commandArgs[2]);
              webSocket.sendTXT(num, data);
              
              //final sensor readings
              ReadDistSensors(f,s,fSensor,sSensor);
              ReadIMU(gz,head);
              sprintf(data, "Last,%f,%f,%f", f, s, (-1*head+ 290.0)*DEG_TO_RAD);
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


  //Setup Distance Sensors
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);

  delay(500);
  Wire.begin(SDA_PORT, SCL_PORT);

  digitalWrite(D3, HIGH);
  delay(150);
  Serial.println("00");

  fSensor.init(true);
  Serial.println("01");
  delay(100);
  fSensor.setAddress((uint8_t)22);

  digitalWrite(D4, HIGH);
  delay(150);
  sSensor.init(true);
  Serial.println("03");
  delay(100);
  sSensor.setAddress((uint8_t)25);
  Serial.println("04");

  Serial.println("addresses set");

  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

  for (byte i = 1; i < 120; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);

  delay(3000);

  setupIMU();
  
}

void loop() {
  webSocket.loop();
  float f,s, gz, head;
  ReadIMU(gz,head);
  Serial.print((-1*head+ 290.0)*DEG_TO_RAD); Serial.print("\n");
}
