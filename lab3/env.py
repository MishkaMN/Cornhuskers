from robot import Robot, Action
from utils import *

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

    def __repr__(self):
        return "State(grid_x: {}, grid_y: {}, heading: {}, reward: {}, \
            id: {})".format(self.x, self.y, self.heading, self.reward, self.iden)

    def __str__(self):
        return "({}, {}, {}, {})".format(self.x, self.y, self.heading, self.reward)

class Environment:
    def __init__(self, W, L, rewards):
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
                    if rewards[y*self.W+x] == 1:
                        self.goal_state = (x, self.W-1-y)
                    x_states.append(State(x, self.W-1-y, h, rewards[y*self.W+x]))
                y_states.append(x_states)
            self.states.append(y_states)

        if not self.goal_state:
            raise ValueError('There is no goal state in rewards')

        self.robot = Robot(self.stateAt(0, 0), 0.1)

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

    def stateAt(self, x, y, heading=0):
        # return the state at x and y (in grid view)
        return self.states[heading][self.W-1-y][x] 

    def action_to_take(self, x, y, heading, goal_x, goal_y):

        # if self.state.facing_a_wall():
            # reorient the robot to face the direction that
            # would require longer to reach the goal state
            # so that there is more time to adjust the orientation
            # of the robot
        pass

    def populate_init_policy(self):
        # populate an init policy that moves closer to goal state
        pass