import numpy as np
import math
from math import *

L = 610.0
W = 406.0

# corners are ordered clkwise starting from origin
corners = [[W, L], [W, 0], [0, 0], [0, L]]
DEGREE_TO_RAD = math.pi/180
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
R = np.array([[sigmaAngle**2, 0, 0],[0, sigmaFLaser**2, 0],[0, 0, sigmaSLaser**2]])
b = 94 # mm
FRONT = 0
SIDE = 1

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
    if(theta == 0.0):
        theta += .0001
    if sensorType == 1:
        theta += math.pi/2
    if theta > 2*math.pi:
        theta -= 2*math.pi

    dist_to_corners = [math.sqrt((corner_x - x)**2 + (corner_y - y)**2) for (corner_x, corner_y) in corners]

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
    for (idx, cum_theta) in enumerate(cum_thetas.tolist()):
        if idx == 0:
            if theta < cum_theta:
                return NORTH_WALL
            continue
        if (theta < cum_theta):
            w = math.floor((idx+1)/2)
            return NORTH_WALL if w == 4 else w

    print("NO WALL")


FRONT = 0
SIDE = 1
def HJacobian_at(state):
    # argument: state vector at each time step
    theta, x, y = state
    if theta == 0:
        theta += .1

    wall_f = det_wall(state, FRONT)
    wall_s = det_wall(state, SIDE)
    # Assume handling only
    if wall_f == NORTH_WALL:
        if wall_s == NORTH_WALL:
            # case 0, 0
            return np.array([[1, 0, 0],
                [(L-y)*sin(theta)**2, 0, -1/cos(theta)],
                [-(-L+y)*cos(theta)/(sin(theta)**2), 0, 1/sin(theta)]])
        elif wall_s == EAST_WALL:
        # case 0, 1
            return np.array([[1, 0, 0],
                [(L-y)*sin(theta)/(cos(theta)**2), 0, -1/cos(theta)],
                [(W-x)*sin(theta)/(cos(theta)**2), -1/cos(theta), 0]])
        elif wall_s == SOUTH_WALL:
            # case 0, 2
            return np.array([[1, 0, 0],
                [(L-y)*sin(theta)/(cos(theta)**2), 0, -1/cos(theta)],
                [-y*cos(theta)/(sin(theta)**2), 0, 1/sin(theta)]])

    elif wall_f == EAST_WALL:
        if wall_s == EAST_WALL:
            # case 1, 1
            return np.array([[1, 0, 0],
                [-(W-x)*cos(theta)/(sin(theta)**2), -1/sin(theta), 0],
                [(W-x)*sin(theta)/(cos(theta)**2), -1/cos(theta), 0]])
        elif wall_s == SOUTH_WALL:
        # case 1, 2
            return np.array([[1, 0, 0],
                [-(W-x)*cos(theta)/(sin(theta)**2), -1/sin(theta), 0],
                [-y*cos(theta)/(sin(theta)**2), 0, 1/sin(theta)]])
        elif wall_s == WEST_WALL:
            # case 1, 3
            return np.array([[1, 0, 0],
                [-(W-x)*cos(theta)/(sin(theta)**2), -1/sin(theta), 0],
                [-x*sin(theta)/(cos(theta)**2), -1/cos(theta), 0]])

    if wall_f == SOUTH_WALL:
        if wall_s == NORTH_WALL:
            # case 2, 0
            return np.array([[1, 0, 0],
                [-y*sin(theta)/(cos(theta)**2), 0, -1/cos(theta)],
                [-(-L+y)*cos(theta)/(sin(theta)**2), 0, 1/sin(theta)]])
        elif wall_s == SOUTH_WALL:
            # case 2, 2
            return np.array([[1, 0, 0],
                [-y*sin(theta)/(cos(theta)**2), 0, -1/cos(theta)],
                [-y*cos(theta)/(sin(theta)**2), 0, 1/sin(theta)]])
        elif wall_s == WEST_WALL:
            # case 2, 3
            return np.array([[1, 0, 0],
                [-y*sin(theta)/(cos(theta)**2), 0, -1/cos(theta)],
                [-x*sin(theta)/(cos(theta)**2), -1/cos(theta), 0]])
    
    if wall_f == WEST_WALL:
        if wall_s == NORTH_WALL:
            # case 3, 0
            return np.array([[1, 0, 0],
                [x*cos(theta)/(sin(theta)**2), -1/sin(theta), 0],
                [-(-L+y)*cos(theta)/(sin(theta)**2), 0, 1/sin(theta)]])
        elif wall_s == EAST_WALL:
            # case 3, 1
            return np.array([[1, 0, 0],
                [x*sin(theta)/(sin(theta)**2), -1/sin(theta), 0],
                [(W-x)*sin(theta)/(cos(theta)**2), -1/cos(theta), 0]])
        elif wall_s == WEST_WALL:
            # case 3, 3
            return np.array([[1, 0, 0],
                [x*cos(theta)/(sin(theta)**2), -1/sin(theta), 0],
                [-x*sin(theta)/(cos(theta)**2), -1/cos(theta), 0]])
    
    print("INVALID STATE")

def outputEstimate(q_est):
    wall_f = det_wall(q_est, FRONT)
    wall_s = det_wall(q_est, SIDE)

    theta = q_est.item(0)
    x = q_est.item(1)
    y = q_est.item(2)
    z_est = 0
    if(wall_f == NORTH_WALL and wall_s == SOUTH_WALL):
        z_est = np.array([q_est.item(0),
            (L - y) / math.cos(theta),
            y / math.sin(theta)])
    elif(wall_f == NORTH_WALL and wall_s == EAST_WALL):
        z_est = np.array([q_est.item(0),
            (L - y) / math.cos(theta),
            (W - x) / math.cos(theta)])
    elif(wall_f == NORTH_WALL and wall_s == NORTH_WALL):
        z_est = np.array([q_est.item(0),
            (L - y) / math.cos(theta),
            -(L - y) / math.sin(theta)])
    elif(wall_f == EAST_WALL and wall_s == WEST_WALL):
        z_est = np.array([q_est.item(0),
            (W - x) / math.cos(theta - math.pi/2),
            x / math.sin(theta - math.pi/2)])
    elif(wall_f == EAST_WALL and wall_s == SOUTH_WALL):
        z_est = np.array([q_est.item(0),
            (W - x) / math.cos(theta - math.pi/2),
            y / math.cos(theta - math.pi/2)])
    elif(wall_f == EAST_WALL and wall_s == 1):
        z_est = np.array([q_est.item(0),
            (W - x) / math.cos(theta - math.pi/2),
            -(W - x) / math.sin(theta - math.pi/2)])
    elif(wall_f == SOUTH_WALL and wall_s == NORTH_WALL):
        z_est = np.array([q_est.item(0),
            y / math.cos(theta - math.pi),
            (L - y) / math.sin(theta - math.pi)])
    elif(wall_f == SOUTH_WALL and wall_s == WEST_WALL):
        z_est = np.array([q_est.item(0),
            y / math.cos(theta - math.pi),
            x / math.cos(theta - math.pi)])
    elif(wall_f == SOUTH_WALL and wall_s == SOUTH_WALL):
        z_est = np.array([q_est.item(0),
            y / math.cos(theta - math.pi),
            -y / math.sin(theta - math.pi)])
    elif(wall_f == WEST_WALL and wall_s == EAST_WALL):
        z_est = np.array([q_est.item(0),
            x / math.cos(theta - 3 * math.pi / 2),
            (W - x) / math.sin(theta - 3 * math.pi / 2)])
    elif(wall_f == WEST_WALL and wall_s == NORTH_WALL):
        z_est = np.array([q_est.item(0),
            x / math.cos(theta - 3 * math.pi / 2),
            (L - y) / math.cos(theta - 3 * math.pi / 2)])
    elif(wall_f == WEST_WALL and wall_s == WEST_WALL):
        z_est = np.array([q_est.item(0),
            x / math.cos(theta - 3 * math.pi / 2),
            -x / math.sin(theta - 3 * math.pi / 2)])
    offset = np.array([0, 25, 30])
    z_est -= offset
    return z_est

def getVelocities(pwmR, pwmL):
    vR = -140*math.tanh(-0.048*(pwmR - 91.8))
    vL = 139*math.tanh(-0.047*(pwmL - 92.6))
    vT = .5*(vL+vR)
    wAng = 1/b*(vL-vR)
    return (vR, vL, vT, wAng)

def update_F(state, dt, pwmR, pwmL):
    theta, x, y = state
    _, _, vT, _ = getVelocities(pwmR, pwmL)
    return np.array([[1, 0, 0], 
        [-vT*math.sin(theta)*dt, 1, 0], 
        [vT*math.cos(theta)*dt, 0, 1]])

def update_Q(state, dt):
    theta, x, y = state
    return np.array([[((dt/b)**2)*(sumVar),
        ((dt**2)*math.cos(theta))/(2*b)*diffVar,
        ((dt**2)*math.sin(theta))/(2*b)*diffVar], 
        [((dt**2)*math.cos(theta))/(2*b)*diffVar,
            (cos(theta)**2*(dt**2))/4*sumVar,
            cos(theta)*sin(theta)*(dt**2)/4*sumVar],
        [(dt**2)*sin(theta)/(2*b)*diffVar,
            cos(theta)*sin(theta)*(dt**2)/4*sumVar,
            (sin(theta)**2)*(dt**2)/4*sumVar]])

def aPrioriUpdate(est_state, dt, P, pwmR, pwmL):
    vR, vL, vT, wAng = getVelocities(pwmR, pwmL)
    #print("THETA:   FIRST")
    #print(est_state[0] * 180.0/math.pi)
    est_state[1] += vT*math.sin(est_state[0])*dt
    est_state[2] += vT*math.cos(est_state[0])*dt 
    est_state[0] = est_state[0] + (wAng*dt)
    F = update_F(est_state, dt, pwmR, pwmL)
    #print("F:")
    #print(F)
    Q = update_Q(est_state, dt)
    #print("Q:")
    #print(Q)
    P = ((F*P)*np.transpose(F))+Q
    """
    print("P:")
    print(P)
    """
    #print("THETA:   AFTER")
    #print(est_state[0] * 180.0/math.pi)
    
    return (est_state, P)

def aPosterioriUpdate(P, z, q_est, dt):
    z_est = outputEstimate(q_est)
    H = HJacobian_at(q_est)
    #print("H:")
    #print(H)
    #print("ZEES")
    #print(z)
    #print(z_est)
    innovation = z - z_est
    #print("S:")
    S = ((H*P)*np.transpose(H))+R
    #print(S)
    K = np.matmul(np.matmul(P,np.transpose(H)),np.linalg.inv(S))
    """
    print("K:")
    print(K)
    """
    q_est += (np.matmul(.01*K,innovation))
    """
    print("q_est")
    print(q_est)
    print("P_post")
    """
    P = (np.eye(3) - (K*H))*P
    #print(P)

    return (q_est, P)
