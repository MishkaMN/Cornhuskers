How to use:

1. Open RobotServer.ino in Arduino
2. Check that the wifi name and password values are correct
3. Upload it to the board
    - If you cannot upload with Arduino saying it does not detect the ESP board.
    - Check to see if there is no previous version of ESP8266 in ...\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266
4. In Client.py, change the IP address of ESP (from router webpage or what the ino file told you when uploaded)
5. Open a terminal window and run Client.py
6. Send commands with the following format:
    LeftPWM RightPWM TimeInMS
