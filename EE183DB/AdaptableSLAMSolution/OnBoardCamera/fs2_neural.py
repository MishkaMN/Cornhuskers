import matplotlib.pyplot as plt
import numpy as np
import math
import time
import ContourFind
import cv2
import matplotlib.animation as animation
from keras.models import Sequential, model_from_json
from keras.initializers import Identity, RandomNormal
from keras.layers import Dense, Dropout, LSTM, SimpleRNN, Dropout, GRU, BatchNormalization, Activation
from keras.optimizers import Adam
from scipy import stats
import os

from picamera.array import PiRGBArray
from picamera import PiCamera
import Client
import IMU 

from scipy.stats import chi2
import matplotlib.mlab as mlab
from random import *
from matplotlib.patches import Ellipse
from numpy.linalg import cholesky

# animation or debug purpose
NUM_ITER = 1
show_animation = False
PRINT_DEBUG = False
numz = 0
change = 0
INITIAL_ANGLE = 0
width = 123

# Fast SLAM covariance
# What we model
# standard error for dist was found to be 3.2872 after linear fit
# approximate with 4
R = np.diag([4.0, np.deg2rad(10.0)])**2
Q = np.diag([23.57, np.deg2rad(1/width * 2*18.4)])**2


D_ASSOC_RADIUS = 5
DT = 0.2  # time tick [s]
SIM_LENGTH = 65  # simulation time [s]
MAX_RANGE = 300.0  # maximum observation range

STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]
N_PARTICLE = 100  # number of particle
NTH = N_PARTICLE / 1.5  # Number of particle for re-sampling
N_LM = 100 # upper limit on number of landmarks

INIT_X = 287.58
INIT_Y = 328.35
INIT_YAW =  88.5788/180*np.pi

def load_model(dirpath, model_fname, lr=1e-3):
    # load theta RNN model
    json_file = open(os.path.join(dirpath, '{}.json'.format(model_fname)), 'r')
    loaded_model_json = json_file.read()
    json_file.close()
    model = model_from_json(loaded_model_json)
    # load weights into new model
    model.load_weights(os.path.join(dirpath, "{}.h5".format(model_fname)))

    optimizer = Adam(lr=lr)
    model.compile(loss='mean_squared_error', optimizer=optimizer)

    print("Loaded {} from disk".format(model_fname))
    return model

class Particle:

    def __init__(self, N_LM):
        self.w = 1.0 / N_PARTICLE
        self.x = INIT_X
        self.y = INIT_Y
        self.yaw = INIT_YAW
        self.P = np.eye(3)
        # landmark x-y positions
        self.lm = np.zeros((N_LM, LM_SIZE))
        # landmark position covariance
        self.lmP = np.zeros((LM_SIZE, LM_SIZE))
        #self.lmP = R
        self.lmP = np.copy(np.tile(self.lmP, (N_LM,1)))
        
        self.seen = 0

def gen_input(t, num, ws):
    if (num % 2 == 0):
        v_l = 90
        v_r = 90
        ws.send("90 90 100")
    else:
        while (True):
            cmd_raw = input("Control with F/B/L/R or W/S/A/D or q: \n")
            #Exit
            if cmd_raw == 'q':
                return 'q'
            inputs = Client.dir_to_cmd(cmd_raw)
            if cmd_raw == 'invalid':
                print("Invalid direction... please try again")
                continue
            else:
                break
        
        print("SLAMING! Please wait...")
        ws.send(inputs)
        inputs = np.fromstring(inputs, dtype=int, sep=' ')
        """
        inputs  = np.array([[160,15, 200],
            [160,15, 200],
            [160,15, 200],
            [83,85, 200],
            [160,15, 200],
            [160,15, 200],
            [170,101, 100],
            [160,15, 200],
            [160,15, 200],
            [83,85, 200],
            [160,15, 100],
            [170,101, 200],
            [160,15, 100],
            [170,101, 200],
            [160,15, 200],
            [83,85, 200],
            [160,15, 200],
            [170,101, 200],
            [160,15, 200],
            [83,85, 100],
            [160,15, 100],
            [170,101, 200],
            ])
        """
        v_l = inputs[0]
        v_r = inputs[1]
        #tm = inputs[2]
        #print(cmd)
        #cmd = str(v_l) + " " + str(v_r)
    return np.array([[v_l], [v_r]])


def fast_slam2(particles, u, st_est, camera, rawCap):

    particles = predict_particles(particles, u)

    st_est, locations = make_obs(particles, st_est, u, camera, rawCap)

    z = data_assoc(particles, locations)

    particles = update_with_observation(particles, z)

    particles = resampling(particles)

    return particles, st_est, z

def predict_particles(particles, u):

    for i in range(N_PARTICLE):
        px = np.zeros((STATE_SIZE, 1))
        px[0, 0] = particles[i].x
        px[1, 0] = particles[i].y
        px[2, 0] = particles[i].yaw
        #print("px_before\n", px)
        #ud = u + (np.random.randn(1, 2) @ Q).T  # add noise
        px = motion_model(px, u)
        #print("px_after\n", px)
        particles[i].x = px[0, 0]
        particles[i].y = px[1, 0]
        particles[i].yaw = px[2, 0]

    return particles

def add_new_lm_mod(particle, z, R):
    
    r = z[0,0]
    b = z[1,0]
    lm_id = -1
    lst = [list(x) for x in particle.lm]
    try:
        lm_id = lst.index([0,0])
    
    except:
        return -1
    # plus should be changed to minus when using contour

    s = math.sin(pi_2_pi(particle.yaw + b))
    c = math.cos(pi_2_pi(particle.yaw + b))

    particle.lm[lm_id, 0] = particle.x + r * c
    particle.lm[lm_id, 1] = particle.y + r * s

    # covariance
    Gz = np.array([[c, -r * s],
                   [s, r * c]])
    
    particle.lmP[2 * lm_id:2 * lm_id + 2] = Gz @ R @ Gz.T
    particle.seen += 1

    return lm_id

def dist_from_obs_to_stored(particle, z, stored_lm_state):
    # was used for min dist data assoc
    r_obs = z[0]
    b_obs = z[1]

    x_std = stored_lm_state[0]
    y_std = stored_lm_state[1]

    # plus should be changed to minus when using contour
    y_obs = particle.y + r_obs * math.sin(pi_2_pi(particle.yaw + b_obs))
    x_obs = particle.x + r_obs * math.cos(pi_2_pi(particle.yaw + b_obs))

    dist = math.sqrt((x_obs - x_std)**2 + (y_obs - y_std)**2)

    return dist

def get_prob(particle, z, stored_lm_state, sigma):


    dx = stored_lm_state[0] - particle.x
    dy = stored_lm_state[1] - particle.y

    dist =  math.sqrt(dx**2 + dy**2)
    theta = pi_2_pi(math.atan2(dy, dx) - particle.yaw)

    mu = np.array([dist,theta]).reshape(2,1)
   
    # Cholesky Update
    Sf = (sigma + sigma.T) * 0.5
    # R of cholesky
    SChol = np.linalg.cholesky(Sf)
    # Rinv
    SCholInv = np.linalg.inv(SChol)
    dz = SCholInv @ (z-mu)
    dz = dz.T @ dz

    p = (1/(2*np.pi)/math.sqrt(np.linalg.det(Sf)))*math.exp(-1/2*dz)

    return p
    #return 1-stats.chi2.cdf(m_dist_x, 2)

def compute_jacobians(particle, xf, Pf, Q):
    dx = xf[0, 0] - particle.x
    dy = xf[1, 0] - particle.y
    d2 = dx**2 + dy**2
    d = math.sqrt(d2)

    zp = np.array(
        [d, pi_2_pi(math.atan2(dy, dx) - particle.yaw)]).reshape(2, 1)

    Hv = np.array([[-dx / d, -dy / d, 0.0],
                   [dy / d2, -dx / d2, -1.0]])

    Hf = np.array([[dx / d, dy / d],
                   [-dy / d2, dx / d2]])

    Sf = Hf @ Pf @ Hf.T + Q

    return zp, Hv, Hf, Sf


def update_KF_with_cholesky(xf, Pf, v, Q, Hf):
    PHt = Pf @ Hf.T
    S = Hf @ PHt + Q

    S = (S + S.T) * 0.5
    SChol = np.linalg.cholesky(S).T
    SCholInv = np.linalg.inv(SChol)
    W1 = PHt @ SCholInv
    W = W1 @ SCholInv.T

    x = xf + W @ v
    P = Pf - W1 @ W1.T

    return x, P


def update_landmark(particle, z, Q):

    lm_id = int(z[2])
    xf = np.array(particle.lm[lm_id, :]).reshape(2, 1)
    Pf = np.array(particle.lmP[2 * lm_id:2 * lm_id + 2])

    zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q)

    dz = z[0:2].reshape(2, 1) - zp
    dz[1, 0] = pi_2_pi(dz[1, 0])

    xf, Pf = update_KF_with_cholesky(xf, Pf, dz, Q, Hf)
    
    particle.lm[lm_id, :] = xf.T

    particle.lmP[2 * lm_id:2 * lm_id + 2, :] = Pf

    return


def compute_weight(particle, z, Q):

    lm_id = int(z[2])
    xf = np.array(particle.lm[lm_id, :]).reshape(2, 1)
    Pf = np.array(particle.lmP[2 * lm_id:2 * lm_id + 2])
    zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q)

    dz = z[0:2].reshape(2, 1) - zp
    dz[1, 0] = pi_2_pi(dz[1, 0])

    try:
        invS = np.linalg.inv(Sf)
    except np.linalg.linalg.LinAlgError:
        return 1.0

    num = math.exp(-0.5 * dz.T @ invS @ dz)
    den = 2.0 * math.pi * math.sqrt(np.linalg.det(Sf))

    w = num / den
    return w


def proposal_sampling(particle, z, Q):

    lm_id = int(z[2])
    xf = particle.lm[lm_id, :].reshape(2, 1)
    Pf = particle.lmP[2 * lm_id:2 * lm_id + 2]
    # State
    x = np.array([particle.x, particle.y, particle.yaw]).reshape(3, 1)
    P = np.copy(particle.P)
    zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q)

    Sfi = np.linalg.inv(Sf)
    dz = z[0:2].reshape(2, 1) - zp
    dz[1] = pi_2_pi(dz[1])

    Pi = np.linalg.inv(P)

    particle.P = np.linalg.inv(Hv.T @ Sfi @ Hv + Pi)  # proposal covariance
    x += particle.P @ Hv.T @ Sfi @ dz  # proposal mean

    particle.x = x[0, 0]
    particle.y = x[1, 0]
    particle.yaw = x[2, 0]

    return


def pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def motion_model(st, u):
    #print(u[0])
    #print(u[1])
    #print(np.cos(current_theta))
    _X = np.array([DT, u[0], u[1], np.cos(st[2]), np.sin(st[2])]).reshape(1, 1, -1)
    
    predictions = nn_model.predict(_X).ravel()

    st[0] = st[0] + predictions[0]
    st[1] = st[1] + predictions[1]
    #st[2] = pi_2_pi(IMU.readAngle() - INITIAL_ANGLE + INIT_YAW)
    st[2] = pi_2_pi(st[2] + predictions[2])
    #st[2] = arctan2(predictions[3], predictions[2])
    return st

def make_obs(particles, st_est, u, camera, rawCap):
    # dead reckoning
    st_est = motion_model(st_est, u)

    # Camera setup
    camera.capture(rawCap, format="bgr")
    img = rawCap.array

    # noisy observation
    locations = np.zeros((2,0))
    global numz
    locations = ContourFind.locateObstacle(img)
    numz += 1
    rawCap.truncate(0)

    return st_est, locations

def data_assoc(particles, locations):

    z = np.zeros((3,0))
    if (locations.shape[1] != 0):
        for k, loc in enumerate(np.hsplit(locations, locations.shape[1])):
            dist = loc[0]
            angle = loc[1]
            lm_ids = np.zeros((1,N_LM))
            lm_id = 0
            for ip, particle in enumerate(particles):
                #vote for the shortest distance one
                max_ip = -1
                skip = False
                for il in range(particles[ip].seen):
                    # Sample here
                    if (not skip and dist_from_obs_to_stored(particles[ip], loc, particles[ip].lm[il]) <= D_ASSOC_RADIUS):
                        skip = True
                        
                    zp, Hv, Hf, Sf = compute_jacobians(particles[ip], particles[ip].lm[il].reshape(2,1), particles[ip].lmP[2*il : 2*il + 2], R)

                    curr_ip = get_prob(particles[ip], loc, particles[ip].lm[il], Sf)

                    if (max_ip < curr_ip and curr_ip > 0.1):
                        lm_id = il
                        max_ip = curr_ip
                
                if max_ip == -1:
                    if skip:
                        continue
                    # otherwise add the new lm
                    lm_id = add_new_lm_mod(particles[ip], loc, R)
                    if lm_id == -1:
                        continue

                lm_ids[0,lm_id] += 1

            lm_id = np.argmax(lm_ids)
            z_i = np.array([[dist[0]], [pi_2_pi(angle[0])], [lm_id]])
            z = np.hstack((z, z_i))
    return z


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

def update_with_observation(particles, z):

    for iz in range(len(z[0, :])):
        lmid = int(z[2, iz])

        for ip in range(N_PARTICLE):
            # new landmark
            w = compute_weight(particles[ip], z[:, iz], R)
            particles[ip].w *= w
            update_landmark(particles[ip], z[:, iz], R)
            proposal_sampling(particles[ip], z[:, iz], R)
    return particles


def resampling(particles):
    """
    low variance re-sampling
    """

    particles = normalize_weight(particles)

    pw = []
    for i in range(N_PARTICLE):
        pw.append(particles[i].w)

    pw = np.array(pw)

    Neff = 1.0 / (pw @ pw.T)  # Effective particle number

    if Neff < NTH:  # resampling
        wcum = np.cumsum(pw)
        base = np.cumsum(pw * 0.0 + 1 / N_PARTICLE) - 1 / N_PARTICLE
        resampleid = base + np.random.rand(base.shape[0]) / N_PARTICLE

        inds = []
        ind = 0
        for ip in range(N_PARTICLE):
            while ((ind < wcum.shape[0] - 1) and (resampleid[ip] > wcum[ind])):
                ind += 1
            inds.append(ind)

        tparticles = list(particles[:])
        
        for i in range(len(inds)):
            particles[i].x = tparticles[inds[i]].x
            particles[i].y = tparticles[inds[i]].y
            particles[i].yaw = tparticles[inds[i]].yaw
            particles[i].lm = np.copy(tparticles[inds[i]].lm[:, :])
            particles[i].lmP = np.copy(tparticles[inds[i]].lmP[:, :])
            particles[i].w = 1.0 / N_PARTICLE

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

def plot_cov_ellipse(cov, pos, nstd=2, ax=None, **kwargs):
    """
    Plots an `nstd` sigma error ellipse based on the specified covariance
    matrix (`cov`). Additional keyword arguments are passed on to the 
    ellipse patch artist.
    Parameters
    ----------
        cov : The 2x2 covariance matrix to base the ellipse on
        pos : The location of the center of the ellipse. Expects a 2-element
            sequence of [x0, y0].
        nstd : The radius of the ellipse in numbers of standard deviations.
            Defaults to 2 standard deviations.
        ax : The axis that the ellipse will be plotted on. Defaults to the 
            current axis.
        Additional keyword arguments are pass on to the ellipse patch.
    Returns
    -------
        A matplotlib ellipse artist
    """
    def eigsorted(cov):
        vals, vecs = np.linalg.eigh(cov)
        order = vals.argsort()[::-1]
        return vals[order], vecs[:,order]

    if ax is None:
        ax = plt.gca()

    vals, vecs = eigsorted(cov)
    theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))

    # Width and height are "full" widths, not radius
    width, height = 2 * nstd * np.sqrt(vals)
    ellip = Ellipse(xy=pos, width=width, height=height, angle=theta, **kwargs)

    ax.add_artist(ellip)
    return ellip

def main(num_particle = 100, dt = 0.2):

    global nn_model
    nn_model = load_model('.', 'fnn_model')
    try:
        ws = Client.DummyClient(Client.esp8266host)
        ws.connect()
        print("Ready")
        print("Starting...")

        global N_PARTICLE 
        N_PARTICLE = num_particle

        global INITIAL_ANGLE    
        INITIAL_ANGLE = IMU.readAngle()
        global DT
        DT = dt


        camera = PiCamera()
        camera.vflip = True
        #camera.hflip = True
        rawCap = PiRGBArray(camera)
        time.sleep(1)
        rawCap.truncate(0)
        
        
        # iterating for NUM_ITER time to get better average results
        # set to just 1 to run once
        #total_time = []
        dist_err = [] 
        angle_err = []
        for k in range(NUM_ITER):
            print("Starting SLAM! Press q to quit when it prompts")
             
            # For evaluation purpose
            """
            env_lm = np.array([
                           [315.0,659.5],
                           [71.0, 721.5],
                           [50.5, 731.0], 
                           [90.0, 743.0],
                           [351.0, 841.0],
                           [316.5, 868.0]
                           ])
            """
            #initialize states
            st_est = np.array([[INIT_X,INIT_Y,INIT_YAW]]).T
            #st_true = np.array([[INIT_X,INIT_Y,INIT_YAW]]).T
            st_dr = np.array([[INIT_X,INIT_Y,INIT_YAW]]).T

            #state histories
            hist_est = st_est
            #hist_true = st_true
            hist_dr = st_dr

            particles = [Particle(N_LM) for i in range(N_PARTICLE)]
        
            ## Initialize the error holder
            hist_err = np.zeros((STATE_SIZE, 1)) # Error in state

            ## Simulation utilities
            sim_time = 0.0
            sim_dt = 0.0
            sim_num = 0
            start_time = time.time()
            fig = plt.figure()
            ax1 = fig.add_subplot(1,1,1)
            while(True):
                startTime = time.time()
                #print("%.2f%%: %d Particles, dt = %.2f" % ((100*sim_time/SIM_LENGTH), num_particle, sim_dt), flush=True)
                
                sim_time += sim_dt
                u = gen_input(sim_time,sim_num, ws)
                if u == 'q':
                    break
                particles, st_est, z = fast_slam2(particles, u, st_est, camera, rawCap)

                st_est = calc_final_state(particles)

                hist_est = np.hstack((hist_est, st_est))
                hist_dr = np.hstack((hist_dr, st_dr))

                sim_num += 1
                if show_animation:
                    
                    ax1.cla()
                    #plt.plot(env_lm[:, 0], env_lm[:, 1], "*k")
                    if(len(z[0,:]) > 0):
                        for iz in range(len(z[0,:])):
                            lmid = int(z[2,iz])
                    
                    for i in range(N_PARTICLE):
                        ax1.plot(particles[i].x, particles[i].y, ".r")
                        ax1.plot(particles[i].lm[:particles[i].seen, 0], particles[i].lm[:particles[i].seen, 1], "xb")
                        # gaussian contour
                        for j in range(particles[i].seen):
                            plot_cov_ellipse(particles[i].lmP[2*j:2*j + 2],particles[i].lm[j], 2, ax1)

                    ax1.plot(hist_dr[0, :], hist_dr[1, :], "-k")
                    ax1.plot(hist_est[0, :], hist_est[1, :], "-r")
                    ax1.plot(st_est[0], st_est[1], "xk")
                    ax1.axis("equal")
                    ax1.grid(True)
                    #plt.pause(0.0001)
                    plt.savefig("evolution%d,%d.png" %(num_particle, sim_num))
                    
                sim_dt = time.time() - startTime
            

            total_time = abs(start_time - time.time())
            print("=================================================")
            print("FastSLAM ended in %.2fs" % (total_time))

            if show_animation:
                # if we had an animation 
                plt.savefig("Sim with %d.png" %(num_particle))
            else:
                plt.cla()
                # if we didnt, just plot the final version
                # and save
                
                #plt.plot(env_lm[:, 0], env_lm[:, 1], "*k")

                if(len(z[0,:]) > 0):
                    for iz in range(len(z[0,:])):
                        ## CHECK z[iz,2] exists
                        lmid = int(z[2,iz])
                for i in range(N_PARTICLE):
                    plt.plot(particles[i].x, particles[i].y, ".r")
                    plt.plot(particles[i].lm[:, 0], particles[i].lm[:, 1], "xb")
                    for j in range(particles[i].seen):
                        plot_cov_ellipse(particles[i].lmP[2*j:2*j + 2],particles[i].lm[j], 2, ax1)


                
                #plt.plot(hist_true[0, :], hist_true[1, :], "-b")
                plt.plot(hist_dr[0, :], hist_dr[1, :], "-k")
                plt.plot(hist_est[0, :], hist_est[1, :], "-r")
                plt.plot(st_est[0], st_est[1], "xk")
                plt.axis("equal")
                plt.grid(True)
                plt.savefig("NeuralSLAM%d.png" %(num_particle))

            return
    except KeyboardInterrupt:
        rawCap.truncate(0)
        camera.close()

    #return sum(dist_err)/NUM_ITER, sum(angle_err)/NUM_ITER, sum(total_time)/NUM_ITER

def run(num_particle):
    return main(num_particle)


if __name__ == '__main__':
    print("Number of Particles: 10")
    i = 10#input()
    if i == '':
        main()
    else:
        main(int(i))
