# RRT Planner

The RRT planner can be run using env.py. A trajectory through the discretized space is generated in the main loop and inputs are generated and sent to the robot.
A visualization of the trajectory is presented after the inputs are sent.

rrt.py contains helper functions that facilitate the RRT generation.

Client.py and KFilter.py are slightly modified versions of the files from lab 2 which allow inputs to be sent to the robot over a WebSocket.
