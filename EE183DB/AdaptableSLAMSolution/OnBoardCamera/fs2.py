import matplotlib.pyplot as plt
import numpy as np
import math
import time
import ContourFind
from scipy.stats import multivariate_normal


width = 123
"""
# Fast SLAM covariance
# What we modeled:
#R = np.diag([10.0, np.deg2rad(10.0)])**2
#Q = np.diag([18.4, np.deg2rad(1/width * 2*18.4)])**2


#  Simulation parameter
Rsim = np.diag([1.0, np.deg2rad(2.0)])**2
Qsim = np.diag([0.5, 0.5])**2
OFFSET_YAWRATE_NOISE = 1.0
"""
# Original
R = np.diag([2.5, np.deg2rad(10.0)])**2
Q = np.diag([5.0, np.deg2rad(20.0)])**2

#  Simulation parameter
Rsim = np.diag([1.0, np.deg2rad(2.0)])**2
Qsim = np.diag([0.5, 0.5])**2
OFFSET_YAWRATE_NOISE = 1.0

#DT = 1  # time tick [s]
SIM_LENGTH = 50.0  # simulation time [s]
MAX_RANGE = 300.0  # maximum observation range
M_DIST_TH = 2.0  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM srate size [x,y]
N_PARTICLE = 100  # number of particle
NTH = N_PARTICLE / 1.5  # Number of particle for re-sampling
N_LM = 10 # upper limit on number of landmarks
ENV_SIZE = 700 # 70cm environment

INIT_X = 100.0
INIT_Y = 100.0
INIT_YAW = 0.0

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
        self.lmP = np.zeros((N_LM * LM_SIZE, LM_SIZE))
        #self.lmP = (self.lmP + self.lmP.T)/2
        self.seen = 0

def gen_input(t):
    if t < 3.0:
        v_l = 0
        v_r = 0
    else:
        v_l = 10.0
        v_r = 20.0
    
    return np.array([[v_l], [v_r]])

def gen_input_ours(t):
    if t < 4.0:
        v_l = 180
        v_r = 85
    else:
        v_l = 90
        v_r = 90
    
    return np.array([[v_l], [v_r]])

def fast_slam2(particles, u, z):

    particles = predict_particles(particles, u)

    particles = update_with_observation(particles, z)

    particles = resampling(particles)

    return particles

def predict_particles(particles, u):

    for i in range(N_PARTICLE):
        px = np.zeros((STATE_SIZE, 1))
        px[0, 0] = particles[i].x
        px[1, 0] = particles[i].y
        px[2, 0] = particles[i].yaw
        ud = u #+ (np.random.randn(1, 2) @ R).T  # add noise
        px = motion_model(px, ud)
        particles[i].x = px[0, 0]
        particles[i].y = px[1, 0]
        particles[i].yaw = px[2, 0]

    return particles

def add_new_lm_mod(particle, z, R):

    r = z[0,0]
    b = z[1,0]

    lst = [list(x) for x in particle.lm]
    #try:
    lm_id = lst.index([0,0])
    #except ValueError:
    #    print("Could not find 0,0")
    # it should definitely be different number than 0
    #    return particle, 0
    # plus should be changed to minus when using contour
    s = math.sin(pi_2_pi(particle.yaw + b))
    c = math.cos(pi_2_pi(particle.yaw + b))

    particle.lm[lm_id, 0] = particle.x + r * c
    particle.lm[lm_id, 1] = particle.y + r * s

    # covariance
    Gz = np.array([[c, -r * s],
                   [s, r * c]])
    print("Matrice Gz\n", Gz)
    #print("Gz:", Gz)
    particle.lmP[2 * lm_id:2 * lm_id + 2] = Gz @ R @ Gz.T
    particle.seen += 1

    return particle, lm_id

def dist_from_obs_to_stored(particle, z, stored_lm_state):

    r_obs = z[0]
    b_obs = z[1]

    x_std = stored_lm_state[0]
    y_std = stored_lm_state[1]

    # plus should be changed to minus when using contour
    y_obs = math.sin(pi_2_pi(particle.yaw + b_obs))
    x_obs = math.cos(pi_2_pi(particle.yaw + b_obs))

    dist = np.sqrt((x_obs - x_std)**2 + (y_obs - y_std)**2)

    return dist

def add_new_lm(particle, z, R):

    r = z[0]
    b = z[1]
    lm_id = int(z[2])

    s = math.sin(pi_2_pi(particle.yaw + b))
    c = math.cos(pi_2_pi(particle.yaw + b))

    particle.lm[lm_id, 0] = particle.x + r * c
    particle.lm[lm_id, 1] = particle.y + r * s

    # covariance
    Gz = np.array([[c, -r * s],
                   [s, r * c]])

    particle.lmP[2 * lm_id:2 * lm_id + 2] = Gz @ R @ Gz.T

    return particle


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

    return particle


def compute_weight(particle, z, Q):

    lm_id = int(z[2])
    print("lm_id", lm_id)
    xf = np.array(particle.lm[lm_id, :]).reshape(2, 1)
    Pf = np.array(particle.lmP[2 * lm_id:2 * lm_id + 2])
    zp, Hv, Hf, Sf = compute_jacobians(particle, xf, Pf, Q)

    dz = z[0:2].reshape(2, 1) - zp
    dz[1, 0] = pi_2_pi(dz[1, 0])

    try:
        invS = np.linalg.inv(Sf)
    except np.linalg.linalg.LinAlgError:
        print("Error")
        return 1.0
    print("dz",dz)
    print("invS", invS)
    num = math.exp(-0.5 * dz.T @ invS @ dz)
    print("num", num)

    den = 2.0 * math.pi * math.sqrt(np.linalg.det(Sf))
    print("den", den)
    w = num / den
    print("weight in compute_weight", w)

    return w


def proposal_sampling(particle, z, Q):

    lm_id = int(z[2])
    xf = particle.lm[lm_id, :].reshape(2, 1)
    Pf = particle.lmP[2 * lm_id:2 * lm_id + 2]
    # State
    x = np.array([particle.x, particle.y, particle.yaw]).reshape(3, 1)
    P = particle.P
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

def motion_model_ours(st, u):
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
# would not need env_lm in the future
def make_obs(particles, st_true, st_dr, u, img, env_lm):
    # dead reckoning
    st_dr = motion_model(st_dr, u)
    
    # noisy observation
    
    z = np.zeros((3, 0))
    """
    locations = ContourFind.locateObstacle()
    not connected right now, so using the old observation method to create dist, angle
    but once it is connected, it can be completely taken out
    """
    locations = np.zeros((2, 0))
    for i in range(len(env_lm[:, 0])):
        dx = env_lm[i, 0] - st_true[0, 0]
        dy = env_lm[i, 1] - st_true[1, 0]
        d = np.sqrt(dx**2+dy**2)
        angle = pi_2_pi(math.atan2(dy, dx) - st_true[2, 0])
        #check if lm in front
        if d <= MAX_RANGE and angle > -1*np.pi/2 and angle < np.pi/2:
            print("Observing in Original Way\n")
            d_n = d + np.random.randn() * Rsim[0, 0]  # add noise
            angle_n = angle + np.random.randn() * Rsim[1, 1]  # add noise
            loc = np.array([[d_n], [pi_2_pi(angle_n)]])
            locations = np.hstack((locations, loc))
    print("Original type of observed locations:\n ", locations)

    # Until this part can be taken out
    if (locations.shape[1] != 0):
        for loc in np.hsplit(locations, locations.shape[1]):
            print("Processing loc:\n", loc)
            dist = loc[0]
            angle = loc[1]
            lm_probs = np.zeros((N_LM,N_PARTICLE))
            lm_ids = np.zeros((1,N_LM))
            lm_id = 0
            for ip in range(N_PARTICLE):
                """
                Problems right now:
                1. How would the particle know to add 
                a new landmark if it has seen a few before, but not this one.
                Because right now, it is associating what it is seeing to a previously seen one
                that seems the most similar(closest).
                Setting boundary of 20 mm
                """
                if particles[ip].seen == 0:
                    print("Particle has never seen before\n")
                    particles[ip], lm_id = add_new_lm_mod(particles[ip], loc, R)
                    lm_ids[0,lm_id] += 1
                else:
                    #vote for the shortest distance one
                    vote_id = 0
                    min_dist= 10000000
                    for il in range(particles[ip].seen):
                        curr_dist = dist_from_obs_to_stored(particles[ip],loc,particles[ip].lm[il])
                        print("Dist from obs to stored", curr_dist)
                        if (min_dist > curr_dist and curr_dist < 50.0):
                            print("Associated with distance", curr_dist)
                            vote_id = il
                            min_dist = curr_dist

                    if min_dist == 10000000:
                        print("No suitable landmarks matched")
                        particles[ip], lm_id = add_new_lm_mod(particles[ip], loc, R)
                        vote_id = lm_id
                    #print(vote_id)
                    lm_ids[0,vote_id] += 1



            """
            has some bug in multivariate normal
            as well as lm_ids vector, which is being treated as a matrix
                lm_ids[lm_id,ip] = multivariate_normal.pdf(particles[ip].lm[lm_id], particles[ip].lmP[2 * lm_id:2 * lm_id + 2])
            else:
                for il in range(particles[ip].seen):
                    lm_probs[il,ip] = multivariate_normal.pdf(particles[ip].lm[il], particles[ip].lmP[2 * il:2 * il + 2])
                    _best_prob = np.argmax(lm_probs[:,ip])
                # if the lm is sufficiently probable
                if (_best_prob > 0.5):
                    lm_ids[0,ip] = _best_prob
                else: #if we basically did not find any suitable particle
                    print("None was good!")
                    particle[ip], lm_id = add_new_lm_mod(particle[ip], loc, R)
                    lm_ids[lm_id,ip] = multivariate_normal.pdf(particles[ip].lm[lm_id], particles[ip].lmP[2 * lm_id:2 * lm_id + 2])
                    input()

            """
            lm_id = np.argmax(lm_ids)
            
            z_i = np.array([[dist[0]], [pi_2_pi(angle[0])], [lm_id]])
            z = np.hstack((z, z_i))
    print("Printing z in make_obs", z)
    #st_true = st_dr
    return st_true, st_dr, z, u

        


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
            if abs(particles[ip].lm[lmid, 0]) <= 0.01:
                particles[ip] = add_new_lm(particles[ip], z[:, iz], R)
            # known landmark
            else:
                w = compute_weight(particles[ip], z[:, iz], R)
                particles[ip].w *= w
                print("w:", particles[ip].w)
                particles[ip] = update_landmark(particles[ip], z[:, iz], R)
                particles[ip] = proposal_sampling(particles[ip], z[:, iz], R)

    return particles


def resampling(particles):
    """
    low variance re-sampling
    """

    particles = normalize_weight(particles)

    pw = []
    for i in range(N_PARTICLE):
        print("Particle weight:", particles[i].w)
        pw.append(particles[i].w)

    pw = np.array(pw)
    print("pw\n", pw)
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

        tparticles = particles[:]
        for i in range(len(inds)):
            particles[i].x = tparticles[inds[i]].x
            particles[i].y = tparticles[inds[i]].y
            particles[i].yaw = tparticles[inds[i]].yaw
            particles[i].lm = tparticles[inds[i]].lm[:, :]
            particles[i].lmP = tparticles[inds[i]].lmP[:, :]
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

def main(num_particle = 100, dt = 0.1):
    global DT
    DT = dt
    print("Starting Simulation...")

    global N_PARTICLE 
    N_PARTICLE = num_particle

    #camera = PiCamera()
    #camera.vflip = True
    #rawCap = PiRGBArray(camera)
    time.sleep(1)

    #initialize environment
    """
    env_lm = np.array([[0,0],
                       [0, 1000],
                       [300, 300], 
                       [1000,0], 
                       [400,100], 
                       [290, 100], 
                       [500,200], 
                       [1000,1000]])
    """
    env_lm = np.array([[40,90],
                           [600, 500],
                           [300, 300], 
                           [100,400], 
                           [400,100], 
                           [290, 100], 
                           [500,200], 
                           [300,550], 
                           [600,200]])
    N_LM = env_lm.shape[0]

    #initialize states
    st_est = np.array([[INIT_X,INIT_Y,INIT_YAW]]).T
    st_true = np.array([[INIT_X,INIT_Y,INIT_YAW]]).T
    st_dr = np.array([[INIT_X,INIT_Y,INIT_YAW]]).T

    #state histories
    hist_est = st_est
    hist_true = st_true
    hist_dr = st_dr

    particles = [Particle(N_LM) for i in range(N_PARTICLE)]

    ## Initialize the error holder
    hist_err = np.zeros((STATE_SIZE, 1)) # Error in state

    ## Simulation utilities
    sim_time = 0.0
    sim_num = 0
    start_time = time.time()

    fig_err_t = plt.figure()

    while(SIM_LENGTH >= sim_time):
        print("%.2f%%: %d Particles, dt = %.2f" % ((100*sim_time/SIM_LENGTH), num_particle, dt), flush=True)
        sim_time += DT

        #camera.capture(rawCap, format="bgr")
        #img = rawCap.array
        u = gen_input(sim_time)

        st_true, st_dr, z, ud = make_obs(particles, st_true, st_dr, u, 0, env_lm)

        particles = fast_slam2(particles, ud, z)

        st_est = calc_final_state(particles)

        # store data history
        hist_est = np.hstack((hist_est, st_est))
        hist_dr = np.hstack((hist_dr, st_dr))
        hist_true = np.hstack((hist_true, st_true))

        st_err = abs(st_est - st_true)

        #print("Current Distance Error: %.2f, Angle Error: %.2f" % (math.sqrt(st_err[0]**2 + st_err[1]**2), np.rad2deg(abs(st_err[2]))))
        hist_err += st_err
        sim_num += 1

        if show_animation:  # pragma: no cover
            plt.cla()
            plt.plot(env_lm[:, 0], env_lm[:, 1], "*k")

            if(len(z[0,:]) > 0):
                for iz in range(len(z[0,:])):
                    ## CHECK z[iz,2] exists
                    lmid = int(z[2,iz])
                    plt.plot([st_est[0], particles[0].lm[lmid, 0]], [
                            st_est[1], particles[0].lm[lmid, 1]], "-k")
                    plt.plot([st_est[0], particles[0].lm[lmid, 0]], [
                            st_est[1], particles[0].lm[lmid, 1]], "-k")

            for i in range(N_PARTICLE):
                plt.plot(particles[i].x, particles[i].y, ".r")
                plt.plot(particles[i].lm[:, 0], particles[i].lm[:, 1], "xb")

            
            plt.plot(hist_true[0, :], hist_true[1, :], "-b")
            plt.plot(hist_dr[0, :], hist_dr[1, :], "-k")
            plt.plot(hist_est[0, :], hist_est[1, :], "-r")
            plt.plot(st_est[0], st_est[1], "xk")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.000001)
            #plt.show()

    # Report Error
    
    hist_err = hist_err / sim_num
    total_time = abs(start_time - time.time())
    dist_err = math.sqrt(hist_err[0]**2 + hist_err[1]**2)
    angle_err = np.rad2deg(hist_err[2])
    print("=================================================")
    print("FastSLAM ended in %.2fs with Distance Error: %.2fmm, Angle Error: %.2fdeg" % (total_time, dist_err, angle_err))
    if show_animation:
        plt.savefig("Sim with %d.png" %(num_particle))
    return dist_err, angle_err

def run(num_particle, dt):
    return main(num_particle, dt)


if __name__ == '__main__':
    print("Number of Particles?")
    i = input()
    if i == '':
        main()
    else:
        main(int(i),1.0)
