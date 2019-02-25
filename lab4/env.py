import numpy as np
from matplotlib.ticker import AutoMinorLocator
from rrt import *
import matplotlib.pyplot as plt
import random
from treelib import Node, Tree
import math
from robot import *

delta = 10      # Distance the robot can run for 1sec.

def getVelocities(pwmR, pwmL):
    b = 9.1         # distance between the wheels in cm

    # units in centimeters
    vR = abs(14*math.tanh(-0.048*(pwmR - 91.8)))
    vL = abs(13.9*math.tanh(-0.047*(pwmL - 92.6)))
    if pwmL < 90 and pwmR > 90:
        pass
    elif pwmL < 90 and pwmR < 90:
        vL *= -1
    elif pwmL > 90 and pwmR < 90:
        vR *= -1
        vL *= -1
    elif pwmL > 90 and pwmR > 90:
        vR *= -1

    vT = .5*(vL+vR)
    wAng = 1/b*(vL-vR)
    return (vR, vL, vT, wAng)

#  (leftpwm, rightpwm)
# forward: 180, 0
# right: 180, 180
# left: 0, 0
forward_pwms = (180, 0)
right_pwms = (180, 180)
left_pwms = (0, 0)

forward_velocities = getVelocities(*forward_pwms)
right_velocities = getVelocities(*right_pwms)
left_velocities = getVelocities(*left_pwms)

class CState:
    def __init__(self, x, y, heading, clear=1):
        self.x = x
        self.y = y
        self.heading = heading
        self.clear = clear

    def __str__(self):
        return str(self.x)+" "+str(self.y)+" "+str(self.heading)+" "+str(self.clear)

class Obstacle:
    def __init__(self,x,y,w,l, robot_rad=0):
        # x, y is the bottom left corner of obstacle
        self.x = x
        self.y = y
        self.w = w
        self.l = l

        self.edges = [[(self.x-robot_rad, self.y-robot_rad), (self.x+self.w+robot_rad, self.y-robot_rad)],
                [(self.x+self.w+robot_rad, self.y-robot_rad), (self.x+self.w+robot_rad, self.y+self.l+robot_rad)],
                [(self.x+self.w+robot_rad, self.y+self.l+robot_rad), (self.x-robot_rad, self.y+self.l+robot_rad)],
                [(self.x-robot_rad, self.y+self.l+robot_rad), (self.x-robot_rad, self.y-robot_rad)]]

class Environment:
    def __init__(self, Nx, Ny, Nt, robot, goal=(0,0), obstacles=None):
        """
        Obstacles format:
        rectangular: x,y,w,l
        """
        x = np.linspace(0,Nx-1,num=Nx)
        y = np.linspace(0,Ny-1,num=Ny)
        heading = np.linspace(0,(360-360/Nt),num=Nt)
        self.Nx = Nx
        self.Ny = Ny
        self.Nt = Nt
        self.robot = robot
        self.C = set()
        self.goal = goal
        self.obstacles = obstacles

        openV = []
        closedV = []
        for xx in range(len(x)):
            for yy in range(len(y)):
                for tt in range(len(heading)):
                    if obstacles is None:
                        openV.append((xx,yy,tt,1))
                    else:
                        for o in obstacles:
                            if (xx < o.x or xx > (o.x+o.w)) or (yy < o.y or yy > (o.y+o.l)):
                                if((xx,yy,tt,1) not in openV):
                                    openV.append((xx,yy,tt,1))
                            else:
                                if((xx,yy,tt,0) not in closedV):
                                    closedV.append((xx,yy,tt,0))
        for st in (openV+closedV):
            self.C.add(CState(st[0],st[1],st[2],st[3]))
        
        self.V = set()
        self.V.add(self.stateAt(robot.x, robot.y, robot.heading))

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
        p2: target state (with any heading because only when physically turning we use heading)
        Returns:
        p: actual state ended with
        """
        if dist(from_state,to_state) < delta:
            return to_state
        heading = math.atan2(to_state.y-from_state.y,to_state.x-from_state.x)
        next_state = CState(from_state.x + delta*math.cos(heading), from_state.y + delta*math.sin(heading), 0, 0)
        # check for available closest available state
        closest_next_states = nearestNeighbors(self.C-self.V, next_state)

        return np.random.choice(closest_next_states)

    def checkTraj(self, traj):
        # check whether traj intersects with obstacle
        def line_intersects(e1, e2):
            p0_x, p0_y = e1[0]
            p1_x, p1_y = e1[1]
            p2_x, p2_y = e2[0]
            p3_x, p3_y = e2[1]

            # Returns 1 if the lines intersect, otherwise 0. In addition, if the lines 
            # intersect the intersection point may be stored in the floats i_x and i_y.
            s1_x = p1_x - p0_x
            s1_y = p1_y - p0_y
            s2_x = p3_x - p2_x
            s2_y = p3_y - p2_y

            denom = (-s2_x * s1_y + s1_x * s2_y)
            if (denom == 0):
                return 0

            s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / denom
            t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / denom

            if s >= 0 and s <= 1 and t >= 0 and t <= 1:
                return 1
            return 0

        for ob in self.obstacles:
            for e in ob.edges:
                print('edge: ', e)
                # check collision
                if line_intersects(e, traj):
                    return False
        return True

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
        if next_step.clear and self.checkTraj(((rand_nn.x, rand_nn.y),(next_step.x, next_step.y))):
            self.V.add(next_step)
            return (rand_nn, next_step)

        return None

    def stateAt(self,x,y,heading):
        for st in self.C:
            if st.x == x and st.y == y and st.heading == heading:
                return st

    def generateInputs(route):
        # produce a series of inputs for the robot to move to goal
        prevState = route[0]
        inputs = []

        for state in route[1:]:
            # heading that robot should position itself
            heading = math.atan2((state.y - prevState.y)/(state.x - prevState.x))
            
            # magnitude to travel
            mag = dist(state, prevState)
            prevState = state

            # map heading to robot action
            if heading < self.robot.heading:
                # turn left
                action = 'L'
                seconds = (self.robot.heading-heading)/left_velocities[3]
            else:
                # turn left
                action = 'R'
                seconds = (heading-self.robot.heading)/right_velocities[3]
            inputs.append((action, seconds))

            # map translation to robot action
            inputs.append(('F', mag/forward_velocities[2]))

        return inputs

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
        
def find_path(tree, goalState):
    candidates = tree.paths_to_leaves()
    for path in candidates:
        if path[-1] == goalState:
            return path
    return None

if __name__ == "__main__":
    robot_rad = 8
    can = Obstacle(10,10,5,5, robot_rad=robot_rad)
    final = (5,5)
    obs = [can]
    robot = Robot(20,20,10)
    env = Environment(40,60,12, robot, goal=final, obstacles=obs)
    goalState = env.stateAt(final[0], final[1], 0)

    route = []

    counta = 0
    while goalState not in env.V:
        newState = env.expandTree()
        if not newState:
            print("failed to expand tree (no trajectory to reach sampled state")
            continue
        print("Expanding Tree " + str(counta))

        route.append(newState)
        counta = counta + 1

    # inputs = generateInputs(route)

    routeTree = route2tree(route)
    paths = []
    path = find_path(routeTree, goalState)
    
    for st in path:
        print(st)
    env.show(route=route)
    
