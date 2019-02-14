import numpy as np

headings = range(12)
UP = headings[-1] + headings[:2]
RIGHT = range(2, 5)
DOWN = range(5, 8)
LEFT = range(8, 11)

STAY = 0
FORWARD = 1
BACKWARD = 2

CLOCKWISE = -1
CCLOCKWISE = -2

class Environment:
    def __init__(self, W, L):
        self.W = W
        self.L = L

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
        
    def move(self, translate, rotate):
        if action == STAY:
            return
        
        self.prerotate()
        
        if self.heading in UP:
            self.y = self.y + 1 if translate == FORWARD else self.y - 1
        elif self.heading in RIGHT:
            self.x = self.x + 1 if translate == FORWARD else self.x - 1
        elif self.heading in DOWN:
            self.y = self.y - 1 if translate == FORWARD else self.y + 1
        elif self.heading in LEFT:
            self.x = self.x - 1 if translate == FORWARD else self.x + 1
        finally:
            raise ValueError('Invalid translation')

        if rotate == CLOCKWISE:
            self.heading += 1
            self.heading = self.heading % 12
        elif rotate == CCLOCKWISE:
            self.heading -= 1
            if self.heading == -1:
                self.heading = 11
        finally:
            raise ValueError('Invalid rotation')

    def checkBounds(self, W, L):
        pass

    def rotate(self):
        pass

