import numpy as np

headings = range(12)
UP = headings[-1:] + headings[:2]
RIGHT = headings[2:5]
DOWN = headings[5:8]
LEFT = headings[8:11]

STAY = 0
FORWARD = 1
BACKWARD = 2

CLOCKWISE = -1
CCLOCKWISE = -2

# Test
W = 5
L = 6
rewards = [1]*W*L

class State:
    def __init__(self, x, y, row, col, reward):
        # x: x coordinate in the grid-view
        # y: y coordinate in the grid-view
        self.x = x
        self.y = y
        self.reward = reward
        #
        self.iden = self.x+self.y*L

        # arrayIdx
    def __repr__(self):
        return "State(grid_x: {}, grid_y: {}, reward: {}, id: {})".format(self.x, self.y, self.reward, self.iden)

    def __str__(self):
        return "({}, {})".format(self.x, self.y)

class Action:
    STAY = 0
    UP = 1
    RIGHT = 2
    DOWN = 3
    LEFT = 4
    actions = [STAY, UP, RIGHT, DOWN, LEFT]

class StateTransitionBlock:
    def __init__(self, state, next_state, action):
        self.state = state
        self.next_state = next_state
        self.action = action

class Environment:
    def __init__(self, W, L, rewards):
        self.W = W
        self.L = L
        self.robot = Robot(0, 0, 0, 0.1)
        # states in the environment in array-view
        self.states = [[State(x, W-1-y, y, x, rewards[y*self.W+x]) 
            for x in range(self.L)] for y in range(self.W)]

    def printEnv(self):
        for y in range(self.W):
            line = ''
            for x in range(self.L):
                state = self.states[y][x]
                line += str(state)
                if state.x == self.robot.x and state.y == self.robot.y:
                    line += '(R)'
                else:
                    line += '   '
                
            print(line)
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

    def checkLoc(x, y):
        # check whether x and y (in grid view) are traversable
        pass

    def flattenStates(self):
        flatten = []
        for col in reversed(self.states):
            flatten += col
        return flatten

    def stateAt(self, x, y):
        # return the state at x and y (in grid view)
        return self.states[W-1-y][x] 

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
                p=[self.p_e, self.p_e, 1 - 2*self.p_2])[0]

        if dice == 0:
            self.heading += 1
        elif dice == 1:
            self.heading -= 1
        
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
        if abs(self.x-x) + abs(self.y-y) > 1:
            raise ValueError('Robot cannot move this far')
        self.x = x
        self.y = y
 
def transition_prob(p_e, s, s_prime, a):
    # p_e: numpy 3D matrix containing state transition probabilities
    # layout is using grid-view indices (i.e. state.iden)
    return p_e[s.iden][s_prime.iden][a]

def get_next_state(p_e, s, a):
    possible_next_states = p_e[s.iden, :, a]
    
    # pick next state based on probabilities
    return np.random.choice(possible_next_states)

if __name__ == '__main__':
    env = Environment(W, L, rewards)
    flattenStates = env.flattenStates()
    print(flattenStates[10:20])
    STM = np.array(
        [[[(idx_k, idx_j, idx_k)  for (idx_k, a) in enumerate(Action.actions)] 
            for (idx_j, s_prime) in enumerate(flattenStates)] 
            for (idx_k, s) in enumerate(flattenStates)]
    )
    
    env.printEnv()
    # print('%r' % env.stateAt(1, 2))
    print(transition_prob(STM, env.stateAt(1, 2), env.stateAt(2, 3), Action.UP))
