import numpy as np
from matplotlib.ticker import AutoMinorLocator
from rrt import *
import matplotlib.pyplot as plt
import random
from treelib import Node, Tree
from math import *

delta = 3 # Distance the robot can run for 1sec.

class CState:
    def __init__(self, x, y, clear=1):
        self.x = x
        self.y = y
        self.clear = clear

    def __str__(self):
        return str(self.x)+" "+str(self.y)+" "+str(self.clear)

class Obstacle:
    def __init__(self,x,y,w,l):
        self.x = x
        self.y = y
        self.w = w
        self.l = l

class Robot:
    def __init__(self,x,y,theta, radius = 5):
        self.x = x
        self.y = y
        self.theta = theta
        self.radius = radius

class Environment:
    def __init__(self, Nx, Ny, robot, goal = (0,0), obstacles=None):
        """
        Obstacles format:
        rectangular: x,y,w,l
        """
        x = np.linspace(0,Nx-1,num=Nx)
        y = np.linspace(0,Ny-1,num=Ny)
        self.Nx = Nx
        self.Ny = Ny
        self.robot = robot
        self.C = set()
        self.goal = goal

        openV = []
        closedV = []
        for xx in range(len(x)):
            for yy in range(len(y)):
                if obstacles is None:
                    openV.append((xx,yy,1))
                else:
                    for o in obstacles:
                        if (xx < o.x or xx > (o.x+o.w)) or (yy < o.y or yy > (o.y+o.l)):
                            if((xx,yy,1) not in openV):
                                openV.append((xx,yy,1))
                        else:
                            if((xx,yy,0) not in closedV):
                                closedV.append((xx,yy,0))
        for st in (openV+closedV):
            self.C.add(CState(st[0],st[1],st[2]))
        
        self.V = set()
        self.V.add(self.stateAt(robot.x, robot.y))

    def show(self, route = None):
        pts = []
        for v in self.C:
            if (v.clear == 0) and ((v.x,v.y) not in pts):
                pts.append((v.x,v.y))
        arena = np.ones((self.Ny,self.Nx))
        for pt in pts:
            arena[pt[1],pt[0]] = 0
        for st in self.V:
            arena[st.y,st.x] = 2
        fig = plt.figure();
        plt.xlim((0,self.Nx))
        plt.ylim((0,self.Ny))
        ax = plt.gca();
        ax.set_xticks(np.arange(0.5, self.Nx+.5, 1));
        ax.set_yticks(np.arange(0.5, self.Ny+.5, 1));
        ax.set_xticklabels(np.arange(0, self.Nx, 1));
        ax.set_yticklabels(np.arange(0, self.Ny, 1));
        ax.pcolor(arena, edgecolors='k', linestyle= 'dashed', linewidths=0.2, cmap='RdYlGn', vmin=0.0, vmax=3.0)
        minor_locator = AutoMinorLocator(2)
        plt.gca().xaxis.set_minor_locator(minor_locator)
        plt.gca().yaxis.set_minor_locator(minor_locator)
        plt.grid(which='minor')
        plt.scatter(self.robot.x+.5, self.robot.y+.5, c='black')
        plt.scatter(self.goal[0]+.5, self.goal[1]+.5, c='green')

        if route is not None:
            for line in route:
                if line:
                    plt.plot([line[0].x+0.5, line[1].x+0.5], [line[0].y+0.5, line[1].y+0.5], c='blue')

        plt.show()

    def step_from_to(self, from_state, to_state):
        """
        2.2c
        This func makes the robot step 1 sec to target state from initial state
        Args:
        p1: initial state
        p2: target state (with any heading because only when physically turning we use theta)
        Returns:
        p: actual state ended with
        """
        if dist(from_state,to_state) < delta:
            return to_state
        theta = atan2(to_state.y-from_state.y,to_state.x-from_state.x)
        next_state = CState(from_state.x + delta*cos(theta), 0, 0)
        # check for available closest available state
        closest_next_states = nearestNeighbors(self.C-self.V, next_state)

        return np.random.choice(closest_next_states)

    def sampleState(self):
        def get_random_idx(lim):
            return ceil(lim*random_sample())

        rand_state = random.sample(self.C - self.V, 1)[0]
        return rand_state

    def expandTree(self):
        rand_state = self.sampleState()
        
        NNs = nearestNeighbors(self.V, rand_state)
        rand_nn = np.random.choice(NNs)
        next_step = self.step_from_to(rand_nn, rand_state)
        if next_step.clear:
            self.V.add(next_step)
            return (rand_nn, next_step)

        return None

    def stateAt(self,x,y):
        for st in self.C:
            if st.x == x and st.y == y:
                return st     

def route2tree(route):
    trees = Tree()
    for idx,line in enumerate(route):
        if not line is None:
            parent = line[0]
            child = line[1]
            if(idx == 0):
                trees.create_node("root", parent)
            trees.create_node(str(child), child, parent=parent)
    return trees
        
def find_path(tree, startState, goalState):
    candidates = tree.paths_to_leaves()
    goalpath = None
    startpath = None
    for path in candidates:
        if goalState in path:
            goalpath = path[::-1]
        if startState in path:
            startpath = path
        if (not startpath is None) and (not goalpath is None):
            break
    if (startpath is None) or (goalpath is None):
        return None

    print("MERGING PATHS")

    longpath = startpath + goalpath
    print(longpath)
    path = []
    for st in longpath:
        if st not in path:
            path.append(st)

    for start in range(len(path)):
        if path[start] == startState:
            path = path[start:-1]
    print(path)
    path = path[::-1]
    for goal in range(len(path)):
        if path[goal] == startState:
            path = path[goal:-1]

    path = path[::-1]
    return path

if __name__ == "__main__":
    can = Obstacle(10,10,5,5)
    final = (5,5)
    obs = [can]
    robot = Robot(20,20,10)
    env = Environment(40,60, robot, goal=final, obstacles=obs)
    goalState = env.stateAt(final[0], final[1])
    startState = env.stateAt(23, 31)

    route = []
    thing = 0
    while goalState not in env.V or startState not in env.V:
        print("Expanding Tree " + str(thing))
        route.append(env.expandTree())
        thing = thing + 1

    routeTree = route2tree(route)
    path = find_path(routeTree, startState, goalState)
    
    for st in path:
        print(st)
    env.show(route=route)
    
