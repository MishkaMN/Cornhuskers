import numpy as np
from matplotlib.ticker import AutoMinorLocator
from rrt import *
import matplotlib.pyplot as plt
import random
from treelib import Node, Tree
import math
from robot import *
from Client import *
import time

delta = 3      # Distance the robot can run for 1sec.

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
    def __init__(self, x, y, clear=1):
        self.x = x
        self.y = y
        self.clear = clear

    def __str__(self):
        return str(self.x)+" "+str(self.y)+" "+str(self.clear)

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
    def __init__(self, Nx, Ny, robot,goal=(0,0), obstacles=None):
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
        self.obstacles = obstacles

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

    def show(self, route = None, path = None):
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
                    plt.plot([line[0].x+0.5, line[1].x+0.5], [line[0].y+0.5, line[1].y+0.5], c="black")

        if path is not None:
            pathX = []
            pathY = []
            for st in path:
                pathX.append(st.x+.5)
                pathY.append(st.y+.5)
            plt.plot(pathX,pathY, c="blue")
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
        next_state = CState(from_state.x + delta*math.cos(heading), from_state.y + delta*math.sin(heading), 0)

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
                # check collision
                if line_intersects(e, traj):
                    return False
        return True

    def sampleState(self):
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

    def stateAt(self,x,y):
        for st in self.C:
            if st.x == x and st.y == y:
                return st

    def generateInputs(self, path):
        # produce a series of inputs for the robot to move to goal
        prevState = path[0]
        inputs = []

        test_heading = 0
        for state in path[1:]:
            raw_heading = math.atan2((state.y-prevState.y), (state.x-prevState.x))
            # heading that robot should position itself
            if raw_heading < 0:
                heading = abs(raw_heading)+math.pi/2
            else:
                if raw_heading > math.pi/2:
                    heading = math.pi-abs(raw_heading) + 3*math.pi/2
                else:
                    heading = math.pi/2 - raw_heading

            # magnitude to travel
            mag = dist(state, prevState)

            prevState = state

            heading_diff = test_heading - heading
            # map heading to robot action
            if (heading_diff > 0 and heading_diff > math.pi) \
                or (heading_diff <= 0 and heading_diff > -math.pi):
                # turn right
                action = 'R'
                seconds = abs(heading_diff/right_velocities[3])
            else:
                # turn left
                action = 'L'
                seconds = abs(heading_diff/left_velocities[3])
                
            # map translation to robot action
            inputs.append([(action, seconds), ('F', mag/forward_velocities[2])])

            # update robot heading
            test_heading = heading
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
        
def find_path(tree, startState, goalState):
    candidates = tree.paths_to_leaves()
    goalpath = None
    for path in candidates:
        if goalState in path:
            goalpath = path

    #path = startpath + goalpath 

    return goalpath

if __name__ == "__main__":
    robot_rad = 8
    can = Obstacle(39,59,1,1, robot_rad=robot_rad)
    final = (5,5)
    obs = [can]
    robot = Robot(20,30,0)
    env = Environment(40,60, robot, goal=final, obstacles=obs)

    goalState = env.stateAt(final[0],final[1])
    startState = env.stateAt(30,30)

    route = []

    print("Expanding Tree ")
    while goalState not in env.V or startState not in env.V:
        next_state = env.expandTree()
        if next_state:
            route.append(next_state)

    routeTree = route2tree(route)
    path = find_path(routeTree, startState, goalState)
    inputs = env.generateInputs(path)


    try:
        ws = RobotClient(esp8266host)
        ws.connect()
        print("Ready")

        for turn,fw in inputs:
            print(turn,fw)
            turnDir, turnLen = turn
            fwLen = fw[1]

            #turn
            if(turnDir == 'L'):
                command = "0 0 " + str(int(1000*turnLen))
                ws.send(command)
                
            else:
                command = "180 180 " + str(int(1000*turnLen))
                ws.send(command)
            time.sleep(turnLen)

            #Forward
            command = "180 0 "+ str(int(1000*fwLen))
            ws.send(command)
            time.sleep(fwLen)

        ws.close()
        env.show(route=route, path=path)

    except KeyboardInterrupt:
        ws.close()

    
    
    