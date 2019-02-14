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
        self.robot = Robot(0, 0, 0, 0.1)

    # Returns true iff the robot can currently perform the given translation
    def robotCanTranslate(self, translate):
        if translate == STAY:
            return True;

        x = self.robot.x
        y = self.robot.y
        heading = self.robot.heading

        if heading in UP:
            return y + 1 < W if translate == FORWARD else return y - 1 >= 0
        elif heading in RIGHT:
            return x + 1 < L if translate == FORWARD else return x - 1 >= 0
        elif heading in DOWN:
            return y - 1 >= 0 if translate == FORWARD else return y + 1 < W
        elif heaind in LEFT:
            return x - 1 >= 0 if translate == FORWARD else return x + 1 < L

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
        if translate == STAY:
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
        else:
            raise ValueError('Invalid translation')

        if rotate == CLOCKWISE:
            self.heading += 1
            self.heading = self.heading % 12
        elif rotate == CCLOCKWISE:
            self.heading -= 1
            self.heading = self.heading % 12
        else:
            raise ValueError('Invalid rotation')

    def checkBounds(self, translate, W, L):
        pass

    def rotate(self):
        pass
