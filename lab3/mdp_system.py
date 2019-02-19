import numpy as np
from utils import *
from env import *
from robot import *
from visualizer import Visualizer
import time
import pickle


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
    rewards = [-100, -100, -100, -100, -100, -100,
    -100, 0, 0, -10, 1, -100,
    -100, 0, 0, -10, 0, -100,
    -100, 0, 0, 0, 0, -100,
    -100, 0, 0, 0, 0, -100,
    -100, -100, -100, -100, -100, -100
    ]

    gamma = float(input("Discount Factor:"))
    error_prob = float(input("Error Probability:"))

    robot = Robot(1, 4, 6, error_prob)
    env = Environment(W, L, rewards, robot)
    
    policy = env.get_init_policy()

    global opt_policy
    global opt_values

    iterType = input("Iteration type (policy, value):")
    loadFile = input("Load from file? (y/n):")
    if(loadFile == "y"):
        with open('{}Iter_{}_policy.p'.format(iterType,env.robot.p_e), 'rb') as pickle_file:
            opt_policy = pickle.load(pickle_file)
        with open('{}Iter_{}_values.p'.format(iterType,env.robot.p_e), 'rb') as pickle_file:
            opt_values = pickle.load(pickle_file)
    elif(loadFile == "n"):
        start = time.time()
        if(iterType == "policy"):
            opt_policy, opt_values = env.find_opt_policy(policy, gamma)
        elif(iterType == "value"):
            opt_policy, opt_values = env.value_iteration(State(robot.x, robot.y, robot.heading, 0), gamma)
        else:
            raise ValueError("Unspecified iteration method selected")
        print((time.time()-start), "Seconds. Whole MDP.")
        route = [(robot.x, robot.y, robot.heading)]
    else:
        raise ValueError("Invalid input")
    
    if(loadFile == "n"):
         pickle.dump(opt_policy, open( "{}Iter_{}_policy.p".format(iterType,env.robot.p_e), "wb" )) 
         pickle.dump(opt_values, open( "{}Iter_{}_values.p".format(iterType,env.robot.p_e), "wb" ))
    # Serlializer:
    # iterType = input("Iteration type (policy, value):")
    # loadFile = input("Load from file? (y/n):")
    # if(loadFile == "y"):
    #     opt_policy = pickle.load("{}Iter_{}_policy.p".format(iterType,env.robot.p_e))
    #     opt_values = pickle.load("{}Iter_{}_values.p".format(iterType,env.robot.p_e))
    # elif(loadFile == "n"):
    #     start = time.time()
    #     if(iterType == "policy"):
    #         opt_policy, opt_values = env.find_opt_policy(policy, gamma)
    #     elif(iterType == "value"):
    #         ###Value iteration here
    #         exit()
    #     else:
    #         raise ValueError("Unspecified iteration method selected")
    #     print((time.time()-start), "Seconds")
    #     route = [(robot.x, robot.y, robot.heading)]
    # else:
    #     raise ValueError("Invalid input")

    seq = [(robot.x, robot.y, robot.heading)]
    i = 0
    #input("Start the Animation?")
    while i < 100:

        action = opt_policy[env.robot.heading][env.robot.y][env.robot.x]
        print('action:', action)
        next_state = env.get_next_state(action)
        #print('next_state: ', next_state)
        env.robot.move(next_state.x, next_state.y, next_state.heading)
        #print('robot pos: ', env.robot.x, env.robot.y, env.robot.heading)
        seq.append(( env.robot.x, env.robot.y, env.robot.heading))
        if env.robot.x == 4 and env.robot.y == 4:
            i = i+ 1
    print(seq)
    vis = Visualizer(seq)
    vis.show()
    

