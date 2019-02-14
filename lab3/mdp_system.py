import numpy as np

headings = range(12)
UP = headings[-1] + headings[:2]
RIGHT = headings[2:5]
DOWN = headings[5:8]
LEFT = headings[8:11]

STAY = 0
FORWARD = 1
BACKWARD = 2

CLOCKWISE = -1
CCLOCKWISE = -2

class Environment:
    def __init__(self, W, L):
        self.W = W
        self.L = L
        self.robot = Robot(0, 0, 0, 0.1)

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

    def move(self, x, y)
        if abs(self.x-x) + abs(self.y-y) > 1:
            raise ValueError('Robot cannot move this far')
        self.x = x
        self.y = y
 

