import numpy as np
import matplotlib.pyplot as plt

W = 10
L = 10

class Environment:
    def __init__(self):
        x = np.linspace(0,39,num=40)
        y = np.linspace(0,59,num=60)
        theta = np.linspace(0,348,num=30)
        self.C = np.zeros([40,60,30])
        for xx in range(len(x)):
            for yy in range(len(y)):
                for tt in range(len(theta)):
                    self.C[xx][yy][tt] = xx+yy+tt

class State:
    def __init__(self, x, y, heading, reward):
        # x: x coordinate in the grid-view
        # y: y coordinate in the grid-view
        self.x = x
        self.y = y
        self.heading = heading
        self.reward = reward

        self.iden = self.x+self.y*L+heading*(W*L)

    def __eq__(self, other):
        print('equating')
        # only compare the x and y coordinates of state
        return self.x == other.x and self.y == other.y

    def __repr__(self):
        return "State(grid_x: {}, grid_y: {}, heading: {}, reward: {}, \
            id: {})".format(self.x, self.y, self.heading, self.reward, self.iden)

    def __str__(self):
        return "({}, {}, {}, {})".format(self.x, self.y, self.heading, self.reward)

if __name__ == "__main__":
        env = Environment()
