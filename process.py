import numpy as np
import matplotlib.pyplot as plt
import math

landmark_positions = []
robot_states = []

with open('data.txt', 'r') as fp:
    line = fp.readline()
    while line:
        if line[0] == '(':
            line = line[1:-2]
            pos = line.split(', ')
            x = float(pos[0])
            y = float(pos[1])
            landmark_positions.append([x,y])
        else:
            state = line.split(' ')
            x = float(state[0])
            y = float(state[1])
            theta = float(state[2])
            robot_states.append([x,y,theta])

        line = fp.readline()

landmark_x = np.array([x for x,y in landmark_positions])
landmark_y = np.array([y for x,y in landmark_positions])

robot_x = np.array([x for x,y,theta in robot_states])
robot_y = np.array([y for x,y,theta in robot_states])
robot_theta = np.array([theta for x,y,theta in robot_states])

plt.plot(landmark_x, landmark_y, 'ro', label='landmarks')

r = 1
robot_u = [r * math.cos(math.radians(theta)) for theta in robot_theta]
robot_v = [r * math.sin(math.radians(theta)) for theta in robot_theta]
plt.quiver(robot_x, robot_y, robot_u, robot_v, pivot='mid', color='b', label='robot trajectory')
plt.legend(loc='lower right')
plt.xlabel('x (mm)')
plt.ylabel('y (mm)')
plt.title('Robot and Landmark State Estimation')
plt.show()
