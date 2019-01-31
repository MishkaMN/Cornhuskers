import numpy as np
import math

L = 750.0
W = 500.0
# corners are ordered clkwise starting from origin
corners = [[W, L], [W, 0], [0, 0], [0, L]]
DEGREE_TO_RAD = ,math.pi/180
HALF_PI = math.pi/2
PI = math.pi
NORTH_WALL = 0
EAST_WALL = 1
SOUTH_WALL = 2
WEST_WALL = 3


sigmaR = 0.115 # mm/s
sigmaL = 0.0538 # mm/s
sumVar = sigmaR * sigmaR + sigmaL * sigmaL
diffVar = sigmaR * sigmaR - sigmaL * sigmaL
sigmaAngle = 15.3*DEGREE_TO_RAD
sigmaFLaser = 3.91 # mm
sigmaSLaser = 5.68 # mm
b = 94 # mm

# state, output vectors
q = np.array([0, W/2, L/2])
# initialized with initial position
z = None
# w is process noise and v is observation noise


def det_wall(state, sensorType):
    # NOTE: the center of the vehicle is the intersection
    # of the lines extended from front and right sensors
    
    # theta is wrt true North
    theta, x, y = state
    
    if sensorType == 1:
        theta += math.pi/2
    if theta > 2*math.pi:
        theta -= 2*math.pi
    
    dist_to_corners = [math.sqrt((corner_x - x)**2 + math.sqrt((corner_y - y)**2)) for (corner_x, corner_y) in corners]

    thetas = []
    # using seg to top right corner
    thetas.append(np.arccos((L-y)/dist_to_corners[0]))
    thetas.append(np.arccos((W-x)/dist_to_corners[0]))
    # using seg to bottom right corner
    thetas.append(np.arccos((W-x)/dist_to_corners[1]))
    thetas.append(np.arccos(y/dist_to_corners[1]))
    # using seg to bottom left corner
    thetas.append(np.arccos(y/dist_to_corners[2]))
    thetas.append(np.arccos(x/dist_to_corners[2]))
    # using seg to top left corner
    thetas.append(np.arccos(x/dist_to_corners[3]))
    thetas.append(np.arccos((L-y)/dist_to_corners[3]))

    cum_thetas = np.cumsum(thetas)
    for (idx, cum_theta) in enumerate(cum_thetas):
        if i == 0:
            if theta < cum_theta:
                return NORTH_WALL
            continue
        if (theta < cum_theta):
            w = (i+1)/2
            return NORTH_WALL if w == 4 else w

FRONT = 0
SIDE = 1
def HJacobian_at(q, case):
    # argument: state vector at each time step
    theta, x, y = q
    wall_f = det_wall(FRONT)
    wall_s = det_wall(SIDE)
    # Assume handling only
    if wall_f == 0 and wall_s == 0:
        # case 0, 0
        return np.array([[1, 0, 0],
            [(L-y)*sin(theta), 0, -1/cos(theta)],
            [-(-L+y)*cos(theta)/(2*sin(theta)), 0, 1/sin(theta)]])
    elif wall_f == 0 and wall_s == 1:
        # case 0, 1
        return np.array([[1, 0, 0],
            [(L-y)*sin(theta)/(2*cos(theta)), 0, -1/cos(theta)], 
            [(W-x)*sin(theta)/(2*cos(theta)), -1/cos(theta), 0]])
    elif wall_f == 0 and wall_s == 2:
        # case 0, 2
        return np.array([[1, 0, 0],
            [(L-y)*sin(theta)/(2*cos(theta), 0, -1/cos(theta)],
            [-y*cos(theta)/(2*sin(theta)), 0, 1/sin(theta)]])
    elif wall_f == 1 and wall_s == 1:
        # case 1, 1
        return np.array([[1, 0, 0],
            [-(W-x)*cos(theta)/(2*sin(theta)), -1/sin(theta), 0],
            [(W-x)*sin(theta)/(2*cos(theta)), -1/cos(theta), 0]])
    elif wall_f == 1 and wall_s == 2:
        # case 1, 2
        return np.array([[1, 0, 0],
            [-(W-x)*cos(theta)/(2*sin(theta)), -1/sin(theta), 0], 
            [-y*cos(theta)/(2*sin(theta)), 0, 1/sin(theta)]])
    elif wall_f == 1 and wall_s == 3:
        # case 1, 3
        return np.array([[1, 0, 0], 
            [-(W-x)*cos(theta)/(2*sin(theta)), -1/sin(theta), 0], 
            [-x*sin(theta)/(2*cos(theta)), -1/cos(theta), 0]])
    elif wall_f == 2 and wall_s == 0:
        # case 2, 0
        return np.array([[1, 0, 0],
            [-y*sin(theta)/(2*cos(theta)), 0, -1/cos(theta)], 
            [-(-L+y)*cos(theta)/(2*sin(theta)), 0, 1/sin(theta)]]
    elif wall_f == 2 and wall_s == 2:
        # case 2, 2
        return np.array([[1, 0, 0], 
            [-y*sin(theta)/(2*cos(theta)), 0, -1/cos(theta)], 
            [-y*cos(theta)/(2*sin(theta)), 0, 1/sin(theta)]])
    elif wall_f == 2 and wall_s == 3:
        # case 2, 3
        return np.array([[1, 0, 0], 
            [-y*sin(theta)/(2*cos(theta)), 0, -1/cos(theta)], 
            [-x*sin(theta)/(2*cos(theta)), -1/cos(theta), 0]])
    elif wall_f == 3 and wall_s == 0:
        # case 3, 0
        return np.array([[1, 0, 0],
            [x*cos(theta)/(2*sin(theta)), -1/sin(theta), 0], 
            [-(-L+y)*cos(theta)/(2*sin(theta)), 0, 1/sin(theta)]])
    elif wall_f == 3 and wall_s == 1:
        # case 3, 1
        return np.array([[1, 0, 0], 
            [x*sin(theta)/(2*sin(theta)), -1/sin(theta), 0], 
            [(W-x)*sin(theta)/(2*cos(theta)), -1/cos(theta), 0]])
    elif wall_f == 3 and wall_s == 3:
        # case 3, 3
        return np.array([[1, 0, 0],
            [x*cos(theta)/(2*sin(theta)), -1/sin(theta), 0], 
            [-x*sin(theta)/(2*cos(theta)), -1/cos(theta), 0]]

class Robot:
    pass
