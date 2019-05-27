import matplotlib.pyplot as plt
import numpy as np
import math
import time
import ContourFind
import cv2
import matplotlib.animation as animation
from scipy import stats

from picamera.array import PiRGBArray
from picamera import PiCamera
import Client

from scipy.stats import chi2
import matplotlib.mlab as mlab
from random import *
from matplotlib.patches import Ellipse
from numpy.linalg import cholesky

PRINT_DEBUG = False
numz = 0
change = 0

width = 70

# Fast SLAM covariance
# What we model
R = np.diag([10.0, np.deg2rad(10.0)])**2
Q = np.diag([18.4, np.deg2rad(1/width * 2*18.4)])**2


#  Simulation parameter
#Rsim = np.diag([1.0, np.deg2rad(2.0)])**2
#Qsim = np.diag([0.5, 0.5])**2
#OFFSET_YAWRATE_NOISE = 1.0

DT = 0.1  # time tick [s]
SIM_LENGTH = 30.0  # simulation time [s]
MAX_RANGE = 300.0  # maximum observation range
M_DIST_TH = 2.0  # Threshold off Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]
N_PARTICLE = 100  # number of particle
NTH = N_PARTICLE / 1.5  # Number of particle for re-sampling
N_LM = 10 # upper limit on number of landmarks
ENV_SIZE = 700 # 70cm environment

INIT_X = 300.0
INIT_Y = 200.0
INIT_YAW = 0.0

NUM_ITER = 1
show_animation = False

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

completedInput = False
def gen_input(t, ws):
    # if t > 1.1 and t < 1.67:
    #     v_l = 180
    #     v_r = 85
    # else:
    global completedInput    
    v_l = 90
    v_r = 90
    ws.send("90 90 10")
    if not completedInput:
        completedInput = True
        ws.send("180 85 1")
        return np.array([[180],[85]])
    return np.array([[v_l], [v_r]])

def fast_slam2(particles, u, st_dr, camera, rawCap):

    particles = predict_particles(particles, u)

    st_dr, locations = make_obs(particles, st_dr, u, camera, rawCap)

    z = data_assoc(particles, locations)

    particles = update_with_observation(particles, z)

    particles = resampling(particles)

    return particles, st_dr, z

def predict_particles(particles, u):

    for i in range(N_PARTICLE):
        px = np.zeros((STATE_SIZE, 1))
        px[0, 0] = particles[i].x
        px[1, 0] = particles[i].y
        px[2, 0] = particles[i].yaw
        ud = u + (np.random.randn(1, 2) @ R).T  # add noise
        px = motion_model(px, ud)
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
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * np.cos(st[2, 0]), 0],
                  [DT * np.sin(st[2, 0]), 0],
                  [0.0, DT]])

    v = np.zeros((2,1))
    
    if u[0,0] == 90:
        v[0,0] = 0.0
    elif (u[0,0] == 180): #FW
        v[0,0] = 228.1183511
    elif (u[0,0] == 83): #BW
        v[0,0] = -230.366008

    if u[1,0] == 90:
        v[1,0] = 0.0
    elif (u[1,0] == 85): #FW
        v[1,0] = 235.3996466
    elif (u[1,0] == 101): #BW
        v[1,0] = -208.6064064

    v = np.array([[(v[0,0] + v[1,0])/2], [1/width * (v[1,0] - v[0,0])]])
    st = F @ st + B @ v
    st[2, 0] = pi_2_pi(st[2, 0])
    return st

def make_obs(particles, st_dr, u, camera, rawCap):
    # dead reckoning
    st_dr = motion_model(st_dr, u)

    # Camera setup
    camera.capture(rawCap, format="bgr")
    img = rawCap.array

    # noisy observation
    locations = np.zeros((2,0))
    locations = ContourFind.locateObstacle(img)

    rawCap.truncate(0)
    """
    for i in range(len(env_lm[:, 0])):

        dx = env_lm[i, 0] - st_true[0, 0]
        dy = env_lm[i, 1] - st_true[1, 0]
        d = math.sqrt(dx**2+dy**2)
        angle = pi_2_pi(math.atan2(dy, dx) - st_true[2, 0])

        #check if lm in front
        if d <= MAX_RANGE and angle > -1*np.pi/2 and angle < np.pi/2:
            d_n = d + np.random.randn() * Rsim[0, 0]  # add noise
            angle_n = angle + np.random.randn() * Rsim[1, 1]  # add noise
            loc = np.array([[d_n], [pi_2_pi(angle_n)]])
            locations = np.hstack((locations, loc))
    """
    return st_dr, locations

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
                    if (not skip and dist_from_obs_to_stored(particles[ip], loc, particles[ip].lm[il]) <= 20):
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

def main(num_particle = 100, dt = 0.1):

    try:
        global DT
        DT = dt
        ws = Client.DummyClient(Client.esp8266host)
        ws.connect()
        print("Ready")
        print("Starting...")

        global N_PARTICLE 
        N_PARTICLE = num_particle
        
        camera = PiCamera()
        camera.vflip = True
        rawCap = PiRGBArray(camera)
        time.sleep(1)
        rawCap.truncate(0)
    

        # iterating for NUM_ITER time to get better average results
        # set to just 1 to run once
        #total_time = []
        dist_err = [] 
        angle_err = []
        for k in range(NUM_ITER):
            print("Starting Simulation %d..." % (k))
        
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
            sim_num = 0
            start_time = time.time()
            fig = plt.figure()
            ax1 = fig.add_subplot(1,1,1)
        
            while(SIM_LENGTH >= sim_time):
                startTime = time.time()
                print("%.2f%%: %d Particles, dt = %.2f" % ((100*sim_time/SIM_LENGTH), num_particle, DT), flush=True)
            
                sim_time += DT
                
                u = gen_input(sim_time, ws)

                particles, st_dr, z = fast_slam2(particles, u, st_dr, camera, rawCap)

                st_est = calc_final_state(particles)

                # store data history
                hist_est = np.hstack((hist_est, st_est))
                hist_dr = np.hstack((hist_dr, st_dr))
                #hist_true = np.hstack((hist_true, st_true))

                #st_err = abs(st_est - st_true)

                #hist_err += st_err
                sim_num += 1
                if show_animation:
                    ax1.cla()
                    
                    
                    if(len(z[0,:]) > 0):
                        for iz in range(len(z[0,:])):
                            ## CHECK z[iz,2] exists
                            lmid = int(z[2,iz])
                            #ax1.plot([st_est[0], env_lm[lmid, 0]], [
                            #        st_est[1], env_lm[lmid, 1]], "-k")
                            #ax1.plot([st_est[0], env_lm[lmid, 0]], [
                            #        st_est[1], env_lm[lmid, 1]], "-k")
                    
                    for i in range(N_PARTICLE):
                        ax1.plot(particles[i].x, particles[i].y, ".r")
                        ax1.plot(particles[i].lm[:particles[i].seen, 0], particles[i].lm[:particles[i].seen, 1], "xb")
                        # gaussian contour
                        for j in range(particles[i].seen):
                            plot_cov_ellipse(particles[i].lmP[2*j:2*j + 2],particles[i].lm[j], 2, ax1)

                    #ax1.plot(hist_true[0, :], hist_true[1, :], "-b")

                    ax1.plot(hist_dr[0, :], hist_dr[1, :], "-k")
                    ax1.plot(hist_est[0, :], hist_est[1, :], "-r")
                    ax1.plot(st_est[0], st_est[1], "xk")
                    ax1.axis("equal")
                    ax1.grid(True)
                    plt.pause(0.0001)
                DT = time.time() - startTime
            
            # Report Error
            
            #hist_err = hist_err / sim_num
            total_time = abs(start_time - time.time())
            #dist_err += [math.sqrt(hist_err[0]**2 + hist_err[1]**2)]
            #angle_err += list(np.rad2deg(hist_err[2]))
            print("=================================================")
            print("FastSLAM ended in %.2fs" % (total_time))
            #print("FastSLAM k = %d ended in %.2fs with Distance Error: %.2fmm, Angle Error: %.2fdeg" % (k, total_time[k], dist_err[k], angle_err[k]))
            #print("total difference", change)
            if show_animation: 
                plt.savefig("Sim with %d.png" %(num_particle))
            else:
                plt.cla()
                
                #plt.plot(env_lm[:, 0], env_lm[:, 1], "*k")

                if(len(z[0,:]) > 0):
                    for iz in range(len(z[0,:])):
                        ## CHECK z[iz,2] exists
                        lmid = int(z[2,iz])
                        ##  need another function that gets correct id
                        #plt.plot([st_est[0], particles[0].lm[lmid, 0]], [
                        #        st_est[1], particles[0].lm[lmid, 1]], "-k")
                        #plt.plot([st_est[0], particles[0].lm[lmid, 0]], [
                        #        st_est[1], particles[0].lm[lmid, 1]], "-k")

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
                plt.savefig("CompletedSLAM%d.png" %(num_particle))

            return
    except KeyboardInterrupt:
        rawCap.truncate(0)
        camera.close()

    #return sum(dist_err)/NUM_ITER, sum(angle_err)/NUM_ITER, sum(total_time)/NUM_ITER

def run(num_particle, dt):
    return main(num_particle, dt)


if __name__ == '__main__':
    print("Number of Particles?")
    i = input()
    if i == '':
        main()
    else:
        main(int(i), 1.3)
