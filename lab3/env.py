from utils import *
import numpy as np
from robot import *

class State:
    def __init__(self, x, y, heading, reward):
        # x: x coordinate in the grid-view
        # y: y coordinate in the grid-view
        self.x = x
        self.y = y
        self.heading = heading
        self.reward = reward

        self.iden = self.x+self.y*L+heading*(W*L)

    def facing_a_wall(self):
        # return true the state is facing a wall

        # facing top wall
        if (self.y == 0 or self.y == W-1) and \
            (self.heading in UP or self.heading in DOWN):
            return True
        if (self.x == 0 or self.x == L-1) and \
            (self.heading in LEFT or self.heading in RIGHT):
            return True
        return False

    def __eq__(self, other):
        print('equating')
        # only compare the x and y coordinates of state
        return self.x == other.x and self.y == other.y

    def __repr__(self):
        return "State(grid_x: {}, grid_y: {}, heading: {}, reward: {}, \
            id: {})".format(self.x, self.y, self.heading, self.reward, self.iden)

    def __str__(self):
        return "({}, {}, {}, {})".format(self.x, self.y, self.heading, self.reward)

class Environment:
    def __init__(self, W, L, rewards, robot):
        self.W = W
        self.L = L
        
        # states in the environment in array-view
        self.states = []
        self.goal_state = None
        
        for h in headings:
            y_states = []
            for y in range(self.W):
                x_states = []
                for x in range(self.L):
                    # find goal state
                    state = State(x, self.W-1-y, h, rewards[y*self.W+x])
                    x_states.append(state)
                    if rewards[y*self.W+x] == 1:
                        self.goal_state = state
                y_states.append(x_states)
            self.states.append(y_states)

        if not self.goal_state:
            raise ValueError('There is no goal state in rewards')

        self.robot = robot

    def printEnv(self, heading=None):
        for h in headings:
            if h is not None and h != heading:
                continue
            print('heading: %d' % h)
            for y in range(self.W):
                line = ''
                for x in range(self.L):
                    state = self.states[h][y][x]
                    line += str(state)
                    if state.x == self.robot.x \
                        and state.y == self.robot.y \
                        and state.heading == self.robot.heading:
                        line += '(R)'
                    else:
                        line += '   '
                
                print(line)
                print('\n')
            print('\n')

    def step(self, translate, rotate):
        # translate: either move forwards or backwards
        if translate == STAY:
            return
        self.robot.prerotate()
        new_x, new_y = self.robot.attempt_move(translate)
        if self.checkLoc(new_x, new_y):
            self.robot.move(new_x, new_y)
        self.robot.rotate(rotate)

    # TODO: implement policy iteration/value iteration
    def run(self):
        pass

    def flattenStates(self):
        flatten = []
        for h in headings:
            for col in reversed(self.states[h]):
                flatten += col
        return flatten

    def stateAt(self, x, y, heading):
        # return the state at x and y (in grid view)
        return self.states[heading][self.W-1-y][x] 

    def action_to_take(self, state, goal_state):
        if state.x != goal_state.x or state.y != goal_state.y:
            goal_x = goal_state.x
            goal_y = goal_state.y
            # actions = [STAY, FORWARD_NOROT, FORWARD_CLK, FORWARD_CCLK,
            # BACKWARD_NOROT, BACKWARD_CLK, BAC_CCLK]
            if state.heading in UP:
                if state.x < goal_x and state.y < goal_y:
                    return Action.FORWARD_CLK
                elif state.x == goal_x and state.y < goal_y:
                    return Action.FORWARD_NOROT
                elif state.x > goal_x and state.y < goal_y:
                    return Action.FORWARD_CCLK
                elif state.x < goal_x and state.y == goal_y:
                    return Action.FORWARD_CLK
                elif state.x > goal_x and state.y == goal_y:
                    return Action.FORWARD_CCLK
                elif state.x < goal_x and state.y > goal_y:
                    return Action.BACKWARD_CCLK
                elif state.x == goal_x and state.y > goal_y:
                    return Action.BACKWARD_NOROT
                elif state.x > goal_x and state.y > goal_y:
                    return Action.BACKWARD_CLK            
            elif state.heading in LEFT:
                if state.x < goal_x and state.y < goal_y:
                    return Action.BACKWARD_CCLK
                elif state.x == goal_x and state.y < goal_y:
                    return Action.FORWARD_CLK
                elif state.x > goal_x and state.y < goal_y:
                    return Action.FORWARD_CLK
                elif state.x < goal_x and state.y == goal_y:
                    return Action.BACKWARD_NOROT
                elif state.x > goal_x and state.y == goal_y:
                    return Action.FORWARD_NOROT
                elif state.x < goal_x and state.y > goal_y:
                    return Action.BACKWARD_CLK
                elif state.x == goal_x and state.y > goal_y:
                    return Action.FORWARD_CCLK
                elif state.x > goal_x and state.y > goal_y:
                    return Action.FORWARD_CCLK
            elif state.heading in RIGHT:
                if state.x < goal_x and state.y < goal_y:
                    return Action.FORWARD_CCLK
                elif state.x == goal_x and state.y < goal_y:
                    return Action.FORWARD_CCLK
                elif state.x > goal_x and state.y < goal_y:
                    return Action.BACKWARD_CLK
                elif state.x < goal_x and state.y == goal_y:
                    return Action.FORWARD_NOROT
                elif state.x > goal_x and state.y == goal_y:
                    return Action.BACKWARD_NOROT
                elif state.x < goal_x and state.y > goal_y:
                    return Action.FORWARD_CLK
                elif state.x == goal_x and state.y > goal_y:
                    return Action.FORWARD_CLK
                elif state.x > goal_x and state.y > goal_y:
                    return Action.BACKWARD_CCLK
            elif state.heading in DOWN:
                if state.x < goal_x and state.y < goal_y:
                    return Action.BACKWARD_CCLK
                elif state.x == goal_x and state.y < goal_y:
                    return Action.BACKWARD_NOROT
                elif state.x > goal_x and state.y < goal_y:
                    return Action.BACKWARD_CLK
                elif state.x < goal_x and state.y == goal_y:
                    return Action.FORWARD_CCLK
                elif state.x > goal_x and state.y == goal_y:
                    return Action.FORWARD_CLK
                elif state.x < goal_x and state.y > goal_y:
                    return Action.FORWARD_CCLK
                elif state.x == goal_x and state.y > goal_y:
                    return Action.FORWARD_NOROT
                elif state.x > goal_x and state.y > goal_y:
                    return Action.FORWARD_CLK 
        else:
            return Action.STAY;  

    def get_init_policy(self):
        print('goal: ', self.goal_state)
        # populate an init policy that moves closer to goal state
        init_policy = [ [ [ self.action_to_take(self.stateAt(x, y, h), self.goal_state)
            for x in range(L) ]for y in range(W)] for h in headings]
        
        return init_policy

    def get_p(self, a, s_new):
        #100 percent chance to stay in current spot
        if a == Action.STAY:
            if s_new.x == self.robot.x and s_new.y == self.robot.y and s_new.heading == self.robot.heading:
                return 1
            else:
                return 0

        #If robot moves
        else:
            #prerotation chances
            chance = np.array([self.robot.p_e, 1.0 - 2.0*self.robot.p_e, self.robot.p_e])
            head = np.array([(self.robot.heading - 1) % 12, self.robot.heading, (self.robot.heading + 1) % 12])

            #get candidate states by x,y
            s_primes = np.zeros([3,3])
            for idx,h in enumerate(head):
                if h in UP:
                    if a in fw_actions and self.robot.y <= W-1:
                        s_primes[idx][:] = np.array([self.robot.x, self.robot.y+1, h])
                    elif a in bw_actions and self.robot.y > 0:
                        s_primes[idx][:] = np.array([self.robot.x, self.robot.y-1, h])
                    else:
                        s_primes[idx][:] = np.array([self.robot.x, self.robot.y, h])
                elif h in LEFT:
                    if a in fw_actions and self.robot.x > 0:
                        s_primes[idx][:] = np.array([self.robot.x-1, self.robot.y, h])
                    elif a in bw_actions and self.robot.x < L-1:
                        s_primes[idx][:] = np.array([self.robot.x+1, self.robot.y, h])
                    else:
                        s_primes[idx][:] = np.array([self.robot.x, self.robot.y, h])
                elif h in RIGHT:
                    if a in fw_actions and self.robot.x <= L-1:
                        s_primes[idx][:] = np.array([self.robot.x+1, self.robot.y, h])
                    elif a in bw_actions and self.robot.x > 0:
                        s_primes[idx][:] = np.array([self.robot.x-1, self.robot.y, h])
                    else:
                        s_primes[idx][:] = np.array([self.robot.x, self.robot.y, h])
                elif h in DOWN:
                    if a in fw_actions and self.robot.y > 0:
                        s_primes[idx][:] = np.array([self.robot.x, self.robot.y-1, h])
                    elif a in bw_actions and self.robot.y <= W - 1:
                        s_primes[idx][:] = np.array([self.robot.x, self.robot.y+1, h])
                    else:
                        s_primes[idx][:] = np.array([self.robot.x, self.robot.y, h])

            #update candidate states with new heading if necessary     
            if(a in clk_actions):
                s_primes[:,2] += 1
            elif(a in cclk_actions):
                s_primes[:,2] += -1
            s_primes[:,2] %= 12

            #return nonzero chance if any candidates match the argument
            for idx,row in enumerate(s_primes):
                if s_new.x == row[0] and s_new.y == row[1] and s_new.heading == row[2]:
                    return chance[idx]
                else:
                    continue
        return 0

    def get_next_state(self, a):
        pmf = []
        pos = []
        for st in self.flattenStates():
            p = self.get_p(a, st)

            if p > 0:
                pmf.append(p)
                pos.append(st)

        idx = np.random.choice(len(pmf), p=pmf);
        return pos[idx]

    def get_reward_at(self, x, y, h):
        return self.stateAt(x,y,h).reward
    
    def get_reward_at(self, s_prime):
        return self.stateAt(s_prime.x,s_prime.y,s_prime.h).reward