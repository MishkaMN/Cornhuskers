from env_params import *

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
