from utils import *

from enum import Enum

class Action(Enum):
    STAY = 0
    FORWARD_NOROT = 1
    FORWARD_CLK = 2
    FORWARD_CCLK = 3
    BACKWARD_NOROT = 4
    BACKWARD_CLK = 5
    BACKWARD_CCLK = 6
    
fw_actions = [Action.FORWARD_NOROT, Action.FORWARD_CLK, Action.FORWARD_CCLK]
bw_actions = [Action.BACKWARD_NOROT, Action.BACKWARD_CLK, Action.BACKWARD_CCLK]
clk_actions = [Action.FORWARD_CLK, Action.BACKWARD_CLK];
cclk_actions = [Action.FORWARD_CCLK, Action.BACKWARD_CCLK];
#  2.1(b) Action space, N_a = 7;
actions = [Action.STAY, Action.FORWARD_NOROT, Action.FORWARD_CLK, Action.FORWARD_CCLK, Action.BACKWARD_NOROT, Action.BACKWARD_CLK, Action.BACKWARD_CCLK]

class Robot:
    def __init__(self, state, p_e):
        # p_e: prerotate probability
        
        # 11, 0, 1 --> north
        # 2, 3, 4 --> east 
        self.state = state
        self.x = self.state.x
        self.y = self.state.y
        self.heading = self.state.heading
        self.p_e = p_e

    def prerotate(self):
        # prerotate can only possibly happen when the robot
        # moves
        dice = np.random.choice(3, 1, 
                p=[self.p_e, self.p_e, 1 - 2*self.p_e])[0]

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