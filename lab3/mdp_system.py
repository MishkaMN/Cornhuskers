from state import *
from action import *
from env_params import *
import numpy as np

class Environment:
    def __init__(self, W, L, rewards, states, robot):
        self.W = W
        self.L = L
        self.robot = robot
        # states in the environment in array-view
        # 2.1(a) State space, N_s = 6*6*12
        self.states = states

    def printEnv(self):
        tmp = self.states[self.robot.heading][:][:]
        for y in reversed(range(self.W)):
            line = ''
            for x in range(self.L):
                state = tmp[y][x]
                line += str(state)
                if state.x == self.robot.x and state.y == self.robot.y:
                    line += '(R)'
                else:
                    line += '   '
            print(line)

    # TODO: implement policy iteration/value iteration
    def run(self):
        pass

    def flattenStates(self):
        flatten = []
        for i in self.states:
            for j in i:
                flatten += j
        return flatten

    def stateAt(self, x, y, h):
        # return the state at x and y (in grid view)
        return self.states[h][y][x] 

    # 2.1(c) return p(s,a,s')
    def p_sa(self, a, s_prime):
        return self.robot.get_p(a, s_prime)

    # 2.1(d) finds next state using state transition probabilities
    def next_state(self, a):
        return self.robot.next_state_help(a, self.flattenStates());

    # 2.2(a) gets reward for given state
    def get_reward_at(self, x, y, h):
        return self.stateAt(x,y,h).reward
    
    # 2.2(a) gets reward for given state
    def get_reward_at(self, s_prime):
        return self.stateAt(s_prime.x,s_prime.y,s_prime.h).reward

class Robot:
    def __init__(self, x, y, heading, p_e):
        # p_e: prerotate probability
        
        # 11, 0, 1 --> north
        # 2, 3, 4 --> east
        self.heading = heading
        self.x = x
        self.y = y
        self.p_e = p_e

    def move(self, x, y, h):
        if x < 0 or x >=L or y < 0 or y>=W or h < 0 or h > 11:
            raise ValueError('Robot cannot move this far')
        self.x = x
        self.y = y
        self.heading = h

    def get_p(self, a, s_new):
        #100 percent chance to stay in current spot
        if a == Action.STAY:
            if s_new.x == self.x and s_new.y == self.y and s_new.h and self.heading:
                return 1
            else:
                return 0

        #If robot moves
        else:
            #prerotation chances
            chance = np.array([self.p_e, 1.0 - 2.0*self.p_e, self.p_e])
            head = np.array([(self.heading - 1) % 12, self.heading, (self.heading + 1) % 12])

            #get candidate states by x,y
            s_primes = np.zeros([3,3])
            for idx,h in enumerate(head):
                if h in UP:
                    if a in fw_actions and self.y <= W-1:
                        s_primes[idx][:] = np.array([self.x, self.y+1, h])
                    elif a in bw_actions and self.y > 0:
                        s_primes[idx][:] = np.array([self.x, self.y-1, h])
                    else:
                        s_primes[idx][:] = np.array([self.x, self.y, h])
                elif h in LEFT:
                    if a in fw_actions and self.x > 0:
                        s_primes[idx][:] = np.array([self.x-1, self.y, h])
                    elif a in bw_actions and self.x < L-1:
                        s_primes[idx][:] = np.array([self.x+1, self.y, h])
                    else:
                        s_primes[idx][:] = np.array([self.x, self.y, h])
                elif h in RIGHT:
                    if a in fw_actions and self.x <= L-1:
                        s_primes[idx][:] = np.array([self.x+1, self.y, h])
                    elif a in bw_actions and self.x > 0:
                        s_primes[idx][:] = np.array([self.x-1, self.y, h])
                    else:
                        s_primes[idx][:] = np.array([self.x, self.y, h])
                elif h in DOWN:
                    if a in fw_actions and self.y > 0:
                        s_primes[idx][:] = np.array([self.x, self.y-1, h])
                    elif a in bw_actions and self.y <= W - 1:
                        s_primes[idx][:] = np.array([self.x, self.y+1, h])
                    else:
                        s_primes[idx][:] = np.array([self.x, self.y, h])

            #update candidate states with new heading if necessary     
            if(a in clk_actions):
                s_primes[:,2] += 1
            elif(a in cclk_actions):
                s_primes[:,2] += -1
            s_primes[:,2] %= 12

            #return nonzero chance if any candidates match the argument
            for idx,row in enumerate(s_primes):
                if s_new.x == row[0] and s_new.y == row[1] and s_new.h == row[2]:
                    return chance[idx]
                else:
                    continue
        return 0

    def next_state_help(self, a, all_states):
        pmf = []
        pos = []
        for st in all_states:
            p = self.get_p(a, st)

            if p > 0:
                pmf.append(p)
                pos.append(st)

        idx = np.random.choice(len(pmf), p=pmf);
        return pos[idx]
                
if __name__ == '__main__':
    states = [[[State(x, y, h, rewards[y][x]) for x in range(L)] for y in range(W)] for h in headings]
    robot = Robot(1, 4, 6, 0)
    env = Environment(W, L, rewards, states, robot)
    print(env.next_state(Action.FORWARD_CLK))

