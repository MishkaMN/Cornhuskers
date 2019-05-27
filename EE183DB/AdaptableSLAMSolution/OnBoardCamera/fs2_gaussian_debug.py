import matplotlib.pyplot as plt
import numpy as np
import math
import time
import matplotlib.animation as animation
from scipy import stats
    
import sklearn
from scipy.stats import chi2
import matplotlib.mlab as mlab
from random import *
from matplotlib.patches import Ellipse
from numpy.linalg import cholesky

PRINT_DEBUG = True
numz = 0
change = 0
# Fast SLAM covariance
# What we model
R = np.diag([2.5, np.deg2rad(10.0)])**2
Q = np.diag([5.0, np.deg2rad(20.0)])**2

#  Simulation parameter
Rsim = np.diag([1.0, np.deg2rad(2.0)])**2
Qsim = np.diag([0.5, 0.5])**2
OFFSET_YAWRATE_NOISE = 1.0

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
show_animation = True

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

def gen_input(t):
    if t < 3.0:
        v_l = 0
        v_r = 0
    else:
        v_l = 10.0
        v_r = 20.0
    
    return np.array([[v_l], [v_r]])

def fast_slam2(particles, u, st_true, st_dr, env_lm):

    particles = predict_particles(particles, u)

    st_true, st_dr, locations, debug_coords = make_obs(particles, st_true, st_dr, u, 0, env_lm)

    z = data_assoc(particles, locations, debug_coords)

    particles = update_with_observation(particles, z)

    particles = resampling(particles)

    return particles, st_true, st_dr

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

def add_new_lm_mod(particle, z, R, i, debug_coords, prob, particles):

    if (PRINT_DEBUG):
        print ("adding new for particle", i)
        print(particle.lm)
    
    r = z[0,0]
    b = z[1,0]
    lm_id = -1
    lst = [list(x) for x in particle.lm]
    try:
        lm_id = lst.index([0,0])
    
    except:
        #print("Could not find 0,0; lm list full")
        #print("leaving add new")
        
        return -1
    # plus should be changed to minus when using contour
    print(lm_id)
    s = math.sin(pi_2_pi(particle.yaw + b))
    c = math.cos(pi_2_pi(particle.yaw + b))

    if (i+ 1 != 10 and PRINT_DEBUG):
        print("before assignment", numz)
        print(particle.lm)
        print(particles[i + 1].lm)
        

    particle.lm[lm_id, 0] = particle.x + r * c
    particle.lm[lm_id, 1] = particle.y + r * s

    #for debugging, tracking the first particle
    if (PRINT_DEBUG):
        print("=============")
        print("adding z", z)
        print("at x,y", particle.lm[lm_id, 0], particle.lm[lm_id, 1])
        print("truth:", debug_coords)
        print("lm_id", lm_id)
        print("new seen", particle.seen +1)
        print("=============")
        print(i)
        if i+ 1 != 10:
            if (particle.lm[lm_id,0] == particles[i+1].lm[lm_id,0]):
                print("Why these are similar", numz)
                print(particle.lm)
                print(particles[i + 1].lm)
                print(particles[i + 2].lm)
                input()
        #input()
    # covariance
    Gz = np.array([[c, -r * s],
                   [s, r * c]])
    
    particle.lmP[2 * lm_id:2 * lm_id + 2] = Gz @ R @ Gz.T
    particle.seen += 1

    #print(lm_id)
    #input()
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
    """
    if dist >= 20:
        print("(x,y) stored", x_std, y_std)
        print("(x,y) obs", x_obs, y_obs)
        input()
    """

    return dist

def get_prob(particle, z, stored_lm_state, sigma, i):

    #if (PRINT_DEBUG):
        #print("===========")
        #print("Getting prob for particle", i)

    dx = stored_lm_state[0] - particle.x
    dy = stored_lm_state[1] - particle.y

    dist =  math.sqrt(dx**2 + dy**2)
    theta = pi_2_pi(math.atan2(dy, dx) - particle.yaw)

    mu = np.array([dist,theta]).reshape(2,1)
    """
    if (i == 0 and PRINT_DEBUG):
        print("===========")
        print("particle 0, prob calc between:")
        print("z", z)
        print("mu", mu)
    """
    # Cholesky Update
    Sf = (sigma + sigma.T) * 0.5
    # R of cholesky
    SChol = np.linalg.cholesky(Sf)
    # Rinv
    SCholInv = np.linalg.inv(SChol)
    dz = SCholInv @ (z-mu)
    dz = dz.T @ dz

    p2 = (1/(2*np.pi)/math.sqrt(np.linalg.det(Sf)))*math.exp(-1/2*dz)
    """
    if (i == 0 and PRINT_DEBUG):
        print("particle 0, assoc prob", p2)
        print("===============")
        #input()
        #input()
    """
    #p1 = 1-stats.chi2.cdf(m_dist_x, 2)

    return p2
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


def update_landmark(particle, z, Q, i):

    #tlm = np.copy(particle.lm)
    #tlmP = np.copy(particle.lmP)

    lm_id = int(z[2])
    xf = np.array(particle.lm[lm_id, :]).reshape(2, 1)
    Pf = np.array(particle.lmP[2 * lm_id:2 * lm_id + 2])

    zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q)

    dz = z[0:2].reshape(2, 1) - zp
    dz[1, 0] = pi_2_pi(dz[1, 0])

    xf, Pf = update_KF_with_cholesky(xf, Pf, dz, Q, Hf)
    
    particle.lm[lm_id, :] = xf.T
    #print("particle %d updated landmark" % (i))
    #print(tlm)
    """
    if (i == 0 and PRINT_DEBUG):
        print("===")
        print("zp", zp)
        print("z", z)
        print("particle 0, lm_id: %d updated (x,y):" %(lm_id))
        print(particle.lm[lm_id, :])
        if lm_id == 5:
            print("update_landmark is called with id 5")
            input()
    """
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


def proposal_sampling(particle, z, Q, i):

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

    return particle


def pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def motion_model(st, u):
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * np.cos(st[2, 0]), 0],
                  [DT * np.sin(st[2, 0]), 0],
                  [0.0, DT]])

    #why 1/70?
    u_prime = np.array([[(u[0,0] + u[1,0])/2], [1/70 * (u[1,0] - u[0,0])]])
    #print(u_prime)
    st = F @ st + B @ u_prime
    st[2, 0] = pi_2_pi(st[2, 0])
    return st

def make_obs(particles, st_true, st_dr, u, img, env_lm):
    # dead reckoning
    st_dr = motion_model(st_dr, u)
    
    
    # noisy input
    ud1 = u[0, 0] + np.random.randn() * Qsim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * Qsim[1, 1]
    ud = np.array([[ud1], [ud2]])

    # ground truth
    st_true = motion_model(st_true,ud)

    # noisy observation
    locations = np.zeros((2,0))
    #locations = ContourFind.locateObstacle(img)
    debug_coords = np.zeros((2,0))
    for i in range(len(env_lm[:, 0])):
        #print(i)
        dx = env_lm[i, 0] - st_true[0, 0]
        dy = env_lm[i, 1] - st_true[1, 0]
        d = math.sqrt(dx**2+dy**2)
        angle = pi_2_pi(math.atan2(dy, dx) - st_true[2, 0])
        #check if lm in front
        if d <= MAX_RANGE and angle > -1*np.pi/2 and angle < np.pi/2:
            #print(env_lm
            #print(env_lm[i,0], env_lm[i,1])
            #print("Observing in Original Way\n")
            d_n = d + np.random.randn() * Rsim[0, 0]  # add noise
            angle_n = angle + np.random.randn() * Rsim[1, 1]  # add noise
            loc = np.array([[d_n], [pi_2_pi(angle_n)]])
            locations = np.hstack((locations, loc))
            #print(env_lm[i,:2].)
            debug_coords = np.hstack((debug_coords, env_lm[i,:2].reshape(2,1)))
    
    return st_true, st_dr, locations, debug_coords

def data_assoc(particles, locations, debug_coords):

    z = np.zeros((3,0))
    if (locations.shape[1] != 0):
        debug_coords = np.hsplit(debug_coords, debug_coords.shape[1])
        for k, loc in enumerate(np.hsplit(locations, locations.shape[1])):
            #print("==============================================")
            #print("Processing loc:\n", loc)
            dist = loc[0]
            angle = loc[1]
            #lm_probs = np.zeros((N_LM,N_PARTICLE))
            lm_ids = np.zeros((1,N_LM))
            lm_id = 0
            for ip, particle in enumerate(particles):
                print("start of ip: %d and %d" % (ip, k))
                print(particles[ip].lm)
                #vote for the shortest distance one
                max_ip = -1
                skip = False
                for il in range(particles[ip].seen):
                    # Sample here
                    if (dist_from_obs_to_stored(particles[ip], loc, particles[ip].lm[il]) <= 20):
                        skip = True
                        
                    zp, Hv, Hf, Sf = compute_jacobians(particles[ip], particles[ip].lm[il].reshape(2,1), particles[ip].lmP[2*il : 2*il + 2], R)

                    curr_ip = get_prob(particles[ip], loc, particles[ip].lm[il], Sf, ip)

                    if (max_ip < curr_ip and curr_ip > 0.1):
                        lm_id = il
                        max_ip = curr_ip
                
                if max_ip == -1:
                    #print("No suitable landmarks matched")
                    # already know, it was mistake
                    if skip:
                        continue
                    # otherwise add the new lm

                    lm_id = add_new_lm_mod(particles[ip], loc, R, ip, debug_coords[k], max_ip,particles)
                    print("end of add_new lm, particle and k", ip, k)
                    print(particles[ip].lm)
                    if lm_id == -1:
                        #print("No space or low prob")
                        #print("ip", ip)
                        #input()
                        continue
                    #print(vote_id)
                print("end of for ip check lm, particle and k", ip, k)
                print(particles[ip].lm)
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
    #print("observed:", len(z[0, :]))
    for iz in range(len(z[0, :])):
        lmid = int(z[2, iz])

        for ip in range(N_PARTICLE):
            # new landmark
            w = compute_weight(particles[ip], z[:, iz], R)
            particles[ip].w *= w
            update_landmark(particles[ip], z[:, iz], R, ip)
            proposal_sampling(particles[ip], z[:, iz], R, ip)

    for ip in range(N_PARTICLE):
        for ipk in range(N_PARTICLE):
            # new landmark
            if (ip != ipk and particles[ip] == particles[ipk]):
                print("what")
                input()
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
    #  print(Neff)

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

    
    global DT
    DT = dt

    global N_PARTICLE 
    N_PARTICLE = num_particle

    # iterating for NUM_ITER time to get better average results
    total_time = []
    dist_err = [] 
    angle_err = []
    for k in range(NUM_ITER):
        print("Starting Simulation %d..." % (k))
    
        #initialize environment
    
        env_lm = np.array([[40,90],
                           [600, 500],
                           [300, 300], 
                           [100,400], 
                           [400,100], 
                           [290, 100], 
                           [500,200], 
                           [300,550], 
                           [600,200]])
        """
        
        env_lm = np.array([[300,300],
                           [400,100],
                           [290, 100],
                           [100, 400],
                           [40,90]])
        """
        #N_LM = env_lm.shape[0]

        #initialize states
        st_est = np.array([[INIT_X,INIT_Y,INIT_YAW]]).T
        st_true = np.array([[INIT_X,INIT_Y,INIT_YAW]]).T
        st_dr = np.array([[INIT_X,INIT_Y,INIT_YAW]]).T

        #state histories
        hist_est = st_est
        hist_true = st_true
        hist_dr = st_dr

        particles = []
        for i in range(N_PARTICLE):
            particles.append(Particle(N_LM))
        #particles = [Particle(N_LM) for i in range(N_PARTICLE)]

        ## Initialize the error holder
        hist_err = np.zeros((STATE_SIZE, 1)) # Error in state

        ## Simulation utilities
        sim_time = 0.0
        sim_num = 0
        start_time = time.time()
        fig = plt.figure()
        ax1 = fig.add_subplot(1,1,1)
    
        while(SIM_LENGTH >= sim_time):
            #print("SimN:%d,%.2f%%: %d Particles, dt = %.2f" % (sim_num+1,(100*sim_time/SIM_LENGTH), num_particle, dt), flush=True)
            sim_time += DT
            
            u = gen_input(sim_time)

            #moved into fastslam
            #st_true, st_dr, z, ud = make_obs(particles, st_true, st_dr, u, 0, env_lm)

            particles, st_true, st_dr = fast_slam2(particles, u, st_true, st_dr, env_lm)

            st_est = calc_final_state(particles)

            # store data history
            hist_est = np.hstack((hist_est, st_est))
            hist_dr = np.hstack((hist_dr, st_dr))
            hist_true = np.hstack((hist_true, st_true))

            st_err = abs(st_est - st_true)

            #print("Current Distance Error: %.2f, Angle Error: %.2f" % (math.sqrt(st_err[0]**2 + st_err[1]**2), np.rad2deg(abs(st_err[2]))))
            hist_err += st_err
            sim_num += 1
            if show_animation:
                ax1.cla()
                ax1.plot(env_lm[:, 0], env_lm[:, 1], "*k")
                """
                if(len(z[0,:]) > 0):
                    for iz in range(len(z[0,:])):
                        ## CHECK z[iz,2] exists
                        lmid = int(z[2,iz])
                        ax1.plot([st_est[0], env_lm[lmid, 0]], [
                                st_est[1], env_lm[lmid, 1]], "-k")
                        ax1.plot([st_est[0], env_lm[lmid, 0]], [
                                st_est[1], env_lm[lmid, 1]], "-k")
                """
                for i in range(N_PARTICLE):
                    ax1.plot(particles[i].x, particles[i].y, ".r")
                    ax1.plot(particles[i].lm[:particles[i].seen, 0], particles[i].lm[:particles[i].seen, 1], "xb")
                    print("particle %d has seen %d at:" % (i, particles[i].seen))
                    print(particles[i].lm)
                    for j in range(particles[i].seen):
                        #print(particles[i].lm[j].shape)
                        #print(particles[i].lmP[2*j:2*j + 2].shape)
                        plot_cov_ellipse(particles[i].lmP[2*j:2*j + 2],particles[i].lm[j], 2, ax1)

                ax1.plot(hist_true[0, :], hist_true[1, :], "-b")

                ax1.plot(hist_dr[0, :], hist_dr[1, :], "-k")
                ax1.plot(hist_est[0, :], hist_est[1, :], "-r")
                ax1.plot(st_est[0], st_est[1], "xk")
                ax1.axis("equal")
                ax1.grid(True)
                plt.pause(0.0001)
        # Report Error
        
        hist_err = hist_err / sim_num
        total_time += [abs(start_time - time.time())]
        dist_err += [math.sqrt(hist_err[0]**2 + hist_err[1]**2)]
        angle_err += list(np.rad2deg(hist_err[2]))
        print("=================================================")
        print("FastSLAM k = %d ended in %.2fs with Distance Error: %.2fmm, Angle Error: %.2fdeg" % (k, total_time[k], dist_err[k], angle_err[k]))
        print(numz)
        print("total difference", change)
        if show_animation: 
            plt.savefig("Sim with %d.png" %(num_particle))
        #input()
        
        #plt.pause(0.000001)

    return sum(dist_err)/NUM_ITER, sum(angle_err)/NUM_ITER, sum(total_time)/NUM_ITER

def run(num_particle, dt):
    return main(num_particle, dt)


if __name__ == '__main__':
    print("Number of Particles?")
    i = input()
    if i == '':
        main()
    else:
        main(int(i))
