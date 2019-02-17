import numpy as np
from utils import *
from env import Environment
from robot import Action

def transition_prob(p_e, s, s_prime, a):
    # p_e: numpy 3D matrix containing state transition probabilities
    # layout is using grid-view indices (i.e. state.iden)
    return p_e[s.iden][s_prime.iden][a]

def get_next_state(p_e, s, a):
    possible_next_states = p_e[s.iden, :, a]
    # pick next state based on probabilities
    return np.random.choice(range(len(possible_next_states)),
            possible_next_states)

def calc_reward(env, inputs):
    # inputs are states that are traversed
    net_reward = 0
    for state in inputs:
        net_reward += state.reward
    return net_reward

if __name__ == '__main__':
    # Problem 2.1
    # rewards = range(W*L)
    # env = Environment(W, L, rewards)
    # flattenStates = env.flattenStates()

    # STM = np.array(
    #     [[[(idx_i, idx_j, idx_k)  for idx_k in range(len(Action.actions))] 
    #         for idx_j in range(len(flattenStates))] 
    #         for idx_i in range(len(flattenStates))]
    # )

    # Problem 2.2
    rewards = [-100, -100, -100, -100, -100, -100,
    -100, 0, 0, -10, 1, -100,
    -100, 0, 0, -10, 0, -100,
    -100, 0, 0, 0, 0, -100,
    -100, 0, 0, 0, 0, -100,
    -100, -100, -100, -100, -100, -100
    ]

    env = Environment(W, L, rewards)
    print(env.get_next_state(Action.FORWARD_CLK))
    
    # flattenStates = env.flattenStates()

    # Problem 2.3
    # assume initial policy theta_0 of taking the action ta
