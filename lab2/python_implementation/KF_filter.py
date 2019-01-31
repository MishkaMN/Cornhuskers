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

# test input
pwmL = 90
pwmR = 90

sigmaR = 0.115 # mm/s
sigmaL = 0.0538 # mm/s
sumVar = sigmaR * sigmaR + sigmaL * sigmaL
diffVar = sigmaR * sigmaR - sigmaL * sigmaL
sigmaAngle = 15.3*DEGREE_TO_RAD
sigmaFLaser = 3.91 # mm
sigmaSLaser = 5.68 # mm
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
def HJacobian_at(state):
    # argument: state vector at each time step
    theta, x, y = state
    if theta == 0:
        theta += .1

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

def outputEstimate(q_est):
    wall_f = det_wall(FRONT)
    wall_s = det_wall(SIDE)
    print(wall_f + " " + wall_s + "\n")

    theta = q_est.item(0)
    x = q_est.item(1)
    y = q_est.item(2)

    if(wall_f == 0 and wall_s == 2):
        z_est = np.array([q_est.item(0),
            (L - y) / math.cos(theta),
            y / math.sin(theta)])
    elif(wall_f == 0 and wall_s == 1):
        z_est = np.array([q_est.item(0),
            (L - y) / math.cos(theta),
            (W - x) / math.cos(theta)])
    elif(wall_f == 0 and wall_s == 0):
        z_est = np.array([q_est.item(0),
            (L - y) / math.cos(theta),
            -(L - y) / math.sin(theta)])
    elif(wall_f == 0 and wall_s == 0):
        z_est = np.array([q_est.item(0),
            (L - y) / math.cos(theta),
            -(L - y) / math.sin(theta)])
    elif(wall_f == 1 and wall_s == 3):
        z_est = np.array([q_est.item(0),
            (W - x) / math.cos(theta - math.pi/2),
            x / math.sin(theta - math.pi/2)])
    elif(wall_f == 1 and wall_s == 2):
        z_est = np.array([q_est.item(0),
            (W - x) / math.cos(theta - math.pi/2),
            y / math.cos(theta - math.pi/2)])
    elif(wall_f == 1 and wall_s == 1):
        z_est = np.array([q_est.item(0),
            (W - x) / math.cos(theta - math.pi/2),
            -(W - x) / math.sin(theta - math.pi/2)])
    elif(wall_f == 2 and wall_s == 0):
        z_est = np.array([q_est.item(0),
            y / math.cos(theta - math.pi),
            (L - y) / math.sin(theta - math.pi)])
    elif(wall_f == 2 and wall_s == 3):
        z_est = np.array([q_est.item(0),
            y / math.cos(theta - math.pi),
            x / math.cos(theta - math.pi)])
    elif(wall_f == 2 and wall_s == 2):
        z_est = np.array([q_est.item(0),
            y / math.cos(theta - math.pi),
            -y / math.sin(theta - math.pi)])
    elif(wall_f == 3 and wall_s == 1):
        z_est = np.array([q_est.item(0),
            x / math.cos(theta - 3 * math.pi / 2),
            (W - x) / math.sin(theta - 3 * math.pi / 2)])
    elif(wall_f == 3 and wall_s == 0):
        z_est = np.array([q_est.item(0),
            x / math.cos(theta - 3 * math.pi / 2),
            (L - y) / math.cos(theta - 3 * math.pi / 2)])
    elif(wall_f == 3 and wall_s == 3):
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
    wAng = 1/b*(vR-vL)
    return (vR, vL, vT, wAng)

def update_F(state, dt):
    theta, x, y = state
    _, _, vT, _ = getVelocities(pwmR, pwmL)
    return np.array([[1, 0, 0], 
        [-vT*math.sin(theta)*dt, 1, 0], 
        [vT*math.cos(theta)*dt, 1, 0]])

def update_Q(state, dt):
    theta, x, y = state
    return np.array([[((dt/b)**2)*(sumVar),
        dt**2*math.cos(theta)/(2*b)*diffVar,
        dt**2*math.sin(theta)/(2*b)*diffVar], 
        [dt**2*math.cos(theta)/(2*b)*diffVar,
            cos(theta)**2*dt**2/4*sumVar,
            cos(theta)*sin(theta)*dt**2/4*sumVar],
        [dt**2*sin(theta)/(2*b)*diffVar,
            cos(theta)*sin(theta)*dt**2/4*sumVar,
            sin(theta)**2*dt**2/4*sumVar]])

def aPrioriUpdate(state, est_state, dt, P):
    vR, vL, vT, wAng = getVelocities(state)
    est_state[1] += vT * math.cos(est_state[0]*dt)
    est_state[2] += vT * math.sin(est_state[0]*dt) 
    est_state[0] = est_state[0] + (wAng*dt)

    F = update_F(state, dt)
    Q = update_Q(state, dt)
    P = ((F*P)*np.transpose(F))+Q

    return (est_state, P)

def aPosterioriUpdate(P, R, q_est, dt):
    z_est = outputEstimate(q_est)
    H = update_H(q_est)
    # TODO: get input stream
    z = np.array([0, 0, 0])
    innovation = z - z_est
    S = ((H*P)*np.transpose(H))+R
    K = (P*np.transpose(H))*np.inverse(S)
    q_est += (K*innovation)
    P = (np.eye(3) - (K*H))*P

    return (q_est, P)

class Robot:
    pass
