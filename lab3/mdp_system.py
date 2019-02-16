import numpy as np

headings = range(12)
UP = headings[-1:] + headings[:2]
RIGHT = headings[2:5]
DOWN = headings[5:8]
LEFT = headings[8:11]

# Test
W = 6
L = 6
rewards = np.flipud(np.array([
    [-100, -100, -100, -100, -100, -100], 
    [-100,    0,    0,  -10,    1, -100],
    [-100,    0,    0,  -10,    0, -100],
    [-100,    0,    0,    0,    0, -100],
    [-100,    0,    0,    0,    0, -100],
    [-100, -100, -100, -100, -100, -100]
]));

class State:
    def __init__(self, x, y, h, reward):
        # x: x coordinate in the grid-view
        # y: y coordinate in the grid-view
        self.x = x
        self.y = y
        self.h = h
        self.reward = reward
        #
        self.iden = self.x+self.y*W

        # arrayIdx
    def __repr__(self):
        return "State(grid_x: {}, grid_y: {}, grid_head: {} reward: {}, id: {})".format(self.x, self.y, self.h, self.reward, self.iden)

    def __str__(self):
        return "({}, {}, {})".format(self.x, self.y, self.h)

class Action:
    STAY = 0
    FORWARD_NOROT = 1
    FORWARD_CLK = 2
    FORWARD_CCLK = 3
    BACKWARD_NOROT = 4
    BACKWARD_CLK = 5
    BACKWARD_CCLK = 6
    fw_actions = [FORWARD_NOROT, FORWARD_CLK, FORWARD_CCLK]
    bw_actions = [BACKWARD_NOROT, BACKWARD_CLK, BACKWARD_CCLK]
    clk_actions = [FORWARD_CLK, BACKWARD_CLK];
    cclk_actions = [FORWARD_CCLK, BACKWARD_CCLK];
    actions = [STAY, FORWARD_NOROT, FORWARD_CLK, FORWARD_CCLK, BACKWARD_NOROT, BACKWARD_CLK, BACKWARD_CCLK]

class Environment:
    def __init__(self, W, L, rewards):
        self.W = W
        self.L = L
        self.robot = Robot(0, 0, 0, 0.1)
        # states in the environment in array-view
        self.states = [[[State(x, y, h, rewards[y][x]) 
             for x in range(self.L)] for y in range(self.W)] for h in headings]

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

    def checkLoc(x, y):
        # check whether x and y (in grid view) are traversable
        if (x < 0 or x >= L or y < 0 or y >= W):
            return False
        else:
            return True

    def flattenStates(self):
        flatten = []
        for i in self.states:
            for j in i:
                flatten += j
        return flatten

    def stateAt(self, x, y, h):
        # return the state at x and y (in grid view)
        return self.states[h][y][x] 
    
    def next_state(self, a):
        return self.robot.next_state_help(a, self.flattenStates());

    def get_reward_at(self, x, y, h):
        return self.stateAt(x,y,h).reward
    
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

    def prerotate(self):
        # prerotate can only possibly happen when the robot
        # moves
        dice = np.random.choice(3, 1, 
                p=[self.p_e, self.p_e, 1 - 2*self.p_e])[0]

        if dice == 0:
            self.heading += 1
            if self.heading > 11:
                self.heading = 0
        elif dice == 1:
            self.heading -= 1
            if self.heading < 0:
                self.heading = 11
        
    def attempt_move(self, translate):
        if translate == STAY:
            return (self.x, self.y)
         
        if self.heading in UP:
            if translate == FORWARD:
                return (self.x, self.y+1)
            return (self.x, self.y-1)
        elif self.heading in RIGHT:
            if translate == FORWARD:
                return (self.x+1, self.y)
            return (self.x-1, self.y)
        elif self.heading in DOWN:
            if translate == FORWARD:
                return (self.x, self.y-1)
            return (self.x, self.y+1)
        elif self.heading in LEFT:
            if translate == FORWARD:
                return (self.x-1, self.y)
            return (self.x+1, self.y)
        else:
            raise ValueError('Invalid translation')

    def rotate(self, rotate):
        if rotate == CLOCKWISE:
            self.heading += 1
            self.heading = self.heading % 12
        elif rotate == CCLOCKWISE:
            self.heading -= 1
            if self.heading == -1:
                self.heading = 11
        else:
            raise ValueError('Invalid rotation')

    def move(self, x, y):
        if x < 0 or x >=L or y < 0 or y>=W:
            raise ValueError('Robot cannot move this far')
        self.x = x
        self.y = y

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
                    if a in Action.fw_actions and self.y <= W-1:
                        s_primes[idx][:] = np.array([self.x, self.y+1, h])
                    elif a in Action.bw_actions and self.y > 0:
                        s_primes[idx][:] = np.array([self.x, self.y-1, h])
                    else:
                        s_primes[idx][:] = np.array([self.x, self.y, h])
                elif h in LEFT:
                    if a in Action.fw_actions and self.x > 0:
                        s_primes[idx][:] = np.array([self.x-1, self.y, h])
                    elif a in Action.bw_actions and self.x < L-1:
                        s_primes[idx][:] = np.array([self.x+1, self.y, h])
                    else:
                        s_primes[idx][:] = np.array([self.x, self.y, h])
                elif h in RIGHT:
                    if a in Action.fw_actions and self.x <= L-1:
                        s_primes[idx][:] = np.array([self.x+1, self.y, h])
                    elif a in Action.bw_actions and self.x > 0:
                        s_primes[idx][:] = np.array([self.x-1, self.y, h])
                    else:
                        s_primes[idx][:] = np.array([self.x, self.y, h])
                elif h in DOWN:
                    if a in Action.fw_actions and self.y > 0:
                        s_primes[idx][:] = np.array([self.x, self.y-1, h])
                    elif a in Action.bw_actions and self.y <= W - 1:
                        s_primes[idx][:] = np.array([self.x, self.y+1, h])
                    else:
                        s_primes[idx][:] = np.array([self.x, self.y, h])

            #update candidate states with new heading if necessary     
            if(a in Action.clk_actions):
                s_primes[:,2] += 1
            elif(a in Action.cclk_actions):
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
    env = Environment(W, L, rewards)
    print(env.next_state(Action.FORWARD_CLK))

