import numpy as np
import matplotlib.pyplot as plt
from robot import *

# class Environment:
#     def __init__(self):
#         x = np.linspace(0,39,num=40)
#         y = np.linspace(0,59,num=60)
#         theta = np.linspace(0,348,num=30)
#         self.C = np.zeros([40,60,30])
#         for xx in range(len(x)):
#             for yy in range(len(y)):
#                 for tt in range(len(theta)):
#                     self.C[xx][yy][tt] = xx+yy+tt

class State:
    def __init__(self, x, y, heading=0):
        # x: x coordinate in the grid-view
        # y: y coordinate in the grid-view
        self.x = x
        self.y = y
        self.heading = heading

        self.iden = self.x+self.y*L+heading*(W*L)

    def __repr__(self):
        return "State(grid_x: {}, grid_y: {}, heading: {}, reward: {}, \
            id: {})".format(self.x, self.y, self.heading, self.reward, self.iden)

class Environment:
    def __init__(self, W, L, numHeading, robot, goal_state):
        self.W = W
        self.L = L
        self.goal_state = goal_state
        self.robot = robot

        #discretize state space
        x_vals = np.linspace(0,W,num=ceil(W))
        y_vals = np.linspace(0,L,num=ceil(L))
        heading_vals = np.linspace(0,360,num=numHeading)

        # states in the environment in array-view
        self.states = []
        
        for h in heading_vals:
            y_states = []
            for y in y_vals:
                x_states = []
                for x in x_vals:
                    # find goal state
                    # print(rewards[y*self.W+x],self.W-1-y,x)
                    state = State(x, self.W-y, heading=h)
                    x_states.append(state)
                y_states.append(x_states)
            self.states.append(y_states)

    def visualize(self):
        plt.plot(list(map(lambda s: s.x, self.states)),
            list(map(lambda x: s.y, self.states)))
        plt.show()



if __name__ == "__main__":
        env = Environment()