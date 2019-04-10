# Lab 2
The videos in this directory contain the results of the open loop and closed loop state estimator and the comparison of the EKF state estimate to the actual state.

The Client folder contains python code to be run on a computer that connects to the robot, generates a state estimate based on received data and sends commands to the robot.

The RobotServer folder contains code to be run on the ESP8266 that opens a websocket to send data to a connected computer as well as to control the sensors and motors of the robot.
