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
    vR = abs(13.7*math.tanh(0.0474*(pwmR - 87.646)))
    vL = abs(11.2*math.tanh(-0.0577*(pwmL - 89.714)))
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
    if(pwmR == 180 and pwmL == 180):
        wAng = 0.155  * math.pi / 180.0 * 1000
    if(pwmR == 0 and pwmL == 0):
        wAng = 0.153 * math.pi / 180.0 * 1000
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

def rightTurnTime(angle):
    return 6.52*angle+18.38

def leftTurnTime(angle):
    return 6.43*angle+56.29

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
        self.rad=robot_rad
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
        self.Cobs=set()
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
                        if (xx < (o.x - o.rad) or xx > (o.x+o.w+o.rad)) or (yy < (o.y-o.rad) or yy > (o.y+o.l+o.rad)):
                            if((xx,yy,1) not in openV):
                                openV.append((xx,yy,1))
                        else:
                            if((xx,yy,0) not in closedV):
                                closedV.append((xx,yy,0))
        for st in (openV+closedV):
            self.C.add(CState(st[0],st[1],st[2]))
        for st in closedV:
            self.Cobs.add(CState(st[0],st[1],st[2]))
        
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
        plt.axes().set_aspect('equal')
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
        closest_next_states = nearestNeighbors(self.C-self.Cobs-self.V, next_state)

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
    goalpath = None
    for path in candidates:
        if goalState in path:
            goalpath = path

    #path = startpath + goalpath 

    return goalpath

if __name__ == "__main__":
    robot_rad = 8
    can = Obstacle(12,22,1,1, robot_rad=robot_rad)
    final = (2,7)
    """
    rightWall = [Obstacle(39,59,1,1, robot_rad=robot_rad),Obstacle(39,51,1,1, robot_rad=robot_rad),Obstacle(39,43,1,1, robot_rad=robot_rad),Obstacle(39,35,1,1, robot_rad=robot_rad),Obstacle(39,23,1,1, robot_rad=robot_rad),Obstacle(39,15,1,1, robot_rad=robot_rad),Obstacle(39,7,1,1, robot_rad=robot_rad)]
    leftWall = [Obstacle(0,59,1,1, robot_rad=robot_rad),Obstacle(0,51,1,1, robot_rad=robot_rad),Obstacle(0,43,1,1, robot_rad=robot_rad),Obstacle(0,35,1,1, robot_rad=robot_rad),Obstacle(0,23,1,1, robot_rad=robot_rad),Obstacle(0,15,1,1, robot_rad=robot_rad),Obstacle(0,7,1,1, robot_rad=robot_rad)]
    topWall = [Obstacle(1,59,1,1, robot_rad=robot_rad),Obstacle(9,59,1,1, robot_rad=robot_rad),Obstacle(17,59,1,1, robot_rad=robot_rad),Obstacle(25,51,1,1, robot_rad=robot_rad),Obstacle(33,59,1,1, robot_rad=robot_rad)]
    bottomWall = [Obstacle(1,0,1,1, robot_rad=robot_rad),Obstacle(9,0,1,1, robot_rad=robot_rad),Obstacle(17,0,1,1, robot_rad=robot_rad),Obstacle(25,0,1,1, robot_rad=robot_rad),Obstacle(33,0,1,1, robot_rad=robot_rad)]
    """
    obs = [can]#+leftWall+rightWall+topWall+bottomWall
    robot = Robot(22,37,0)
    env = Environment(24,44, robot, goal=final, obstacles=obs)

    goalState = env.stateAt(final[0],final[1])

    route = []

    print("Expanding Tree ")
    while goalState not in env.V:
        next_state = env.expandTree()
        if next_state:
            route.append(next_state)

    routeTree = route2tree(route)
    path = find_path(routeTree, goalState)
    inputs = env.generateInputs(path)
    print("Tree Complete")

    env.show(route=route, path=path)
    exit()

    try:
        ws = RobotClient(esp8266host)
        ws.connect()
        print("Connected to robot")

        ws.send("90 90 5000")  

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

            #Forward
            command = "180 0 "+ str(int(1000*fwLen))
            ws.send(command)

        ws.close()
        env.show(route=route, path=path)

    except KeyboardInterrupt:
        ws.close()

    
    
    