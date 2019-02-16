from state import State
from action import Action
from env_params import *

goal_x = 4
goal_y = 4
goal_h = headings

class Policy:
    def __init__(self, states):
        self.states = states
        self.policy = [ [ [ [None] for _ in headings] for _ in range(W)] for _ in range(L)]
        return

    def stateAt(self, x, y, h):
        return self.states[h][y][x] 

    def flattenStates(self):
        flatten = []
        for i in self.states:
            for j in i:
                flatten += j
        return flatten

    def gen_initial(self):
        goal = []
        for h in goal_h:
            goal.append(self.stateAt(goal_x, goal_y, h))

        for state in self.flattenStates():
            if not state in goal:
                if state.h in UP:
                    pass
                elif state.h in LEFT:
                    pass
                elif state.h in RIGHT:
                    pass
                elif state.h in DOWN:
                    pass
            else:
                self.policy[state.x][state.y][state.h] = Action.STAY;

        return goal

# Unit testing only
if __name__ == '__main__':
    states = [[[State(x, y, h, rewards[y][x]) for x in range(L)] for y in range(W)] for h in headings]
    pol = Policy(states)
    pol.gen_initial()
    for i in headings:
        print(4,4,i,pol.policy[4][4][i])