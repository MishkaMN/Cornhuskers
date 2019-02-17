import numpy as np
from utils import *
from env import *
from robot import *

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
    gamma = 0.9

    robot = Robot(1, 4, 6, .1)
    env = Environment(W, L, rewards, robot)
    
    policy = env.get_init_policy()

    route = []
    # print(policy.shape)
    # for i in range(10):
        # action = policy[env.robot.heading][env.robot.y][env.robot.x]
        # print('action:', action)
        # next_state = env.get_next_state(action)
        # print('next_state: ', next_state)
        # env.robot.move(next_state.x, next_state.y, next_state.heading)
        # print('robot pos: ', env.robot.x, env.robot.y, env.robot.heading)
        # route.append((env.robot.x, env.robot.y, env.robot.heading))
    print(env.policy_eval(State(robot.x, robot.y, robot.heading, 0),
        policy, gamma))
