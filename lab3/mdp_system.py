import numpy as np
import time
import pickle
from utils import *
from env import *
from robot import *
from visualizer import Visualizer
import pdb

def calc_reward(env, inputs):
    # inputs are states that are traversed
    net_reward = 0
    for state in inputs:
        net_reward += state.reward
    return net_reward

if __name__ == '__main__':
    # Problem 2.1
    # rewards = range(W*L)
    # Problem 2.2

    goal_state = State(4, 4, 6, 0)
    robot_init_state = State(1, 4, 6, 0)

    rewards_2d = np.array([[-100, -100, -100, -100, -100, -100],
    [-100, 0, 0, -10, 0, -100],
    [-100, 0, 0, -10, 0, -100],
    [-100, 0, 0, 0, 0, -100],
    [-100, 0, 0, 0, 0, -100],
    [-100, -100, -100, -100, -100, -100]])
    rewards_3d = np.empty((len(headings),)+rewards_2d.shape)

    # expand rewards to 3D and assign reward of 1 only
    # when the heading = 6 at the goal state
    for idx in range(len(headings)):
        rewards_3d[idx,:,:] = rewards_2d
        if idx == goal_state.heading:
            rewards_3d[goal_state.heading, goal_state.y, goal_state.x] = 1

    gamma = float(input("Discount Factor:"))
    error_prob = float(input("Error Probability:"))

    robot = Robot(robot_init_state.x, robot_init_state.y,
        robot_init_state.heading, error_prob)
    env = Environment(W, L, rewards_3d, robot)
    
    policy = env.get_init_policy()

    global opt_policy
    global opt_values
    
    start = time.time()
    
    print((time.time()-start), "Seconds")

    route = [(robot.x, robot.y, robot.heading)]
    # Serlializer:
    iterType = input("Iteration type (policy, value):")
    loadFile = input("Load from file? (y/n):")
    if(loadFile == "y"):
        opt_policy = pickle.load("{}Iter_{}_policy.p".format(iterType,env.robot.p_e))
        opt_values = pickle.load("{}Iter_{}_values.p".format(iterType,env.robot.p_e))
    elif(loadFile == "n"):
        start = time.time()
        if(iterType == "policy"):
            opt_policy, opt_values = env.policy_iteration(policy, gamma)
        elif(iterType == "value"):
            opt_policy, opt_values = env.value_iteration(State(robot.x, robot.y, robot.heading, 0), policy)
            exit()
        else:
            raise ValueError("Unspecified iteration method selected")
        print((time.time()-start), "Seconds")
    else:
        raise ValueError("Invalid input")

    if(loadFile == "n"):
        pickle.dump(opt_policy, open( "{}Iter_{}_policy.p".format(iterType,env.robot.p_e), "wb" )) 
        pickle.dump(opt_values, open( "{}Iter_{}_values.p".format(iterType,env.robot.p_e), "wb" ))

    i = 0
    #input("Start the Animation?")
    while i < 100:

        tmp = opt_p[env.robot.heading][env.robot.y][env.robot.x]
        action = actions[tmp.astype(int)]
        next_state = env.get_next_state(action)
        #print('next_state: ', next_state)
        env.robot.move(next_state.x, next_state.y, next_state.heading)

        #print('robot pos: ', env.robot.x, env.robot.y, env.robot.heading)
        route.append(( env.robot.x, env.robot.y, env.robot.heading))
        #if env.robot.x == 4 and env.robot.y == 4:
        i = i+ 1
    print(route)
    vis = Visualizer(route)
    vis.show()
    

