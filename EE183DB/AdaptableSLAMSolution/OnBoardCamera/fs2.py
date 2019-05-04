import matplotlib.pyplot as plt
import numpy as np
import math
# Fast SLAM covariance
R = np.diag([3.0, np.deg2rad(10.0)])**2
Q = np.diag([1.0, np.deg2rad(20.0)])**2

#  Simulation parameter
Rsim = np.diag([0.3, np.deg2rad(2.0)])**2
Qsim = np.diag([0.5, 0.5])**2
OFFSET_YAWRATE_NOISE = 0.01

DT = 0.1  # time tick [s]
SIM_LENGTH = 50.0  # simulation time [s]
MAX_RANGE = 200.0  # maximum observation range
M_DIST_TH = 2.0  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM srate size [x,y]
N_PARTICLE = 100  # number of particle
NTH = N_PARTICLE / 1.5  # Number of particle for re-sampling
N_LM = 10 # upper limit on number of landmarks
ENV_SIZE = 700 # 70cm environment

class Particle:

    def __init__(self, N_LM):
        self.w = 1.0 / N_PARTICLE
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.P = np.eye(3)
        # landmark x-y positions
        self.lm = np.zeros((N_LM, LM_SIZE))
        # landmark position covariance
        self.lmP = np.zeros((N_LM * LM_SIZE, LM_SIZE))

def gen_input(t):
    if t < 3.0:
        v_l = 0
        v_r = 0
    else:
        v_l = 1.0
        v_r = 2.0
    
    return np.array([[v_l], [v_r]])

def pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def motion_model(st, u):
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * np.cos(st[2, 0]), 0],
                  [DT * np.sin(st[2, 0]), 0],
                  [0.0, DT]])

    u_prime = np.array([[u[0,0] + u[1,0]/2], [1/70 * (u[1,0] - u[0,0])]])
    print(u_prime)
    st = F @ st + B @ u_prime
    st[2, 0] = pi_2_pi(st[2, 0])
    return st

def make_obs(st_true, st_dr, u, env_lm):
    st_true = motion_model(st_true, u)
    
    #noisy observation
    z = np.zeros((3, 0))
    for i in range(len(env_lm[:, 0])):
        dx = env_lm[i, 0] - st_true[0, 0]
        dy = env_lm[i, 1] - st_true[1, 0]
        d = np.sqrt(dx**2+dy**2)
        angle = pi_2_pi(math.atan2(dy, dx) - st_true[2, 0])
        #check if lm in front
        if d <= MAX_RANGE and angle > -1*np.pi/2 and angle < np.pi/2:
            d_n = d + np.random.randn() * Rsim[0, 0]  # add noise
            angle_n = angle + np.random.randn() * Rsim[1, 1]  # add noise
            z_i = np.array([[d_n], [pi_2_pi(angle_n)], [i]])
            z = np.hstack((z, z_i))

    #noisy input
    ud1 = u[0, 0] + np.random.randn() * Qsim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * Qsim[1, 1]
    ud = np.array([[ud1], [ud2]])
    st_dr = motion_model(st_dr,ud)

    return st_true, st_dr, z, ud

def normalize_weight(particles):
    sumw = sum([p.w for p in particles])

    try:
        for i in range(N_PARTICLE):
            particles[i].w /= sumw
    except ZeroDivisionError:
        for i in range(N_PARTICLE):
            particles[i].w = 1.0 / N_PARTICLE

        return particles

    return particles

def calc_final_state(particles):
    st_est = np.zeros((3,1))
    particles = normalize_weight(particles)

    for i in range(N_PARTICLE):
        st_est[0, 0] += particles[i].w * particles[i].x
        st_est[1, 0] += particles[i].w * particles[i].y
        st_est[2, 0] += particles[i].w * particles[i].yaw

    st_est[2, 0] = pi_2_pi(st_est[2, 0])
    return st_est

def main():
    print("Starting Simulation...")
    sim_time = 0.0

    #initialize environment
    env_lm = np.array([[40,90],[600, 500],[300, 300], [100,400], [400,100], [290, 100], [500,200], [300,550], [600,200]])
    
    #States
    st_est = np.zeros((3,1))
    st_true = np.zeros((3,1))
    st_dr = np.zeros((3,1))

    #state histories
    hist_est = st_est
    hist_true = st_true
    hist_dr = st_dr

    particles = [Particle(N_LM) for i in range(N_PARTICLE)]

    while(SIM_LENGTH >= sim_time):
        sim_time += DT
        
        u = gen_input(sim_time)

        st_true, st_dr, z, ud = make_obs(st_true, st_dr, u, env_lm)

        ### FAST SLAM HERE ###

        st_est = calc_final_state(particles)


if __name__ == '__main__':
    main()