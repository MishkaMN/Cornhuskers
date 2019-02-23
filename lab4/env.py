import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator

class CState:
    def __init__(self, x, y, theta, clear=1):
        self.x = x
        self.y = y
        self.theta = theta
        self.clear = clear

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
    def __init__(self, Nx, Ny, Nt, robot, obstacles=None):
        """
        Obstacles format:
        rectangular: x,y,w,l
        """
        x = np.linspace(0,Nx-1,num=Nx)
        y = np.linspace(0,Ny-1,num=Ny)
        theta = np.linspace(0,(360-360/Nt),num=Nt)
        self.Nx = Nx
        self.Ny = Ny
        self.Nt = Nt
        self.robot = robot
        self.C = []
        openV = []
        closedV = []
        for xx in range(len(x)):
            for yy in range(len(y)):
                for tt in range(len(theta)):
                    if obstacles is None:
                        self.C.append(CState(xx,yy,tt))
                        return
                    else:
                        for o in obstacles:
                            if (xx < o.x or xx > (o.x+o.w)) or (yy < o.y or yy > (o.y+o.l)):
                                if((xx,yy,tt,1) not in openV):
                                    openV.append((xx,yy,tt,1))
                            else:
                                if((xx,yy,tt,0) not in closedV):
                                    closedV.append((xx,yy,tt,0))
        for st in (openV+closedV):
            self.C.append(CState(st[0],st[1],st[2],st[3]))

    def show(self):
        pts = []
        for V in self.C:
            if (V.clear == 0) and ((V.x,V.y) not in pts):
                pts.append((V.x,V.y))
        arena = np.ones((self.Ny,self.Nx))
        for pt in pts:
            arena[pt[1],pt[0]] = 0
        
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
        plt.scatter(self.robot.x+.5, self.robot.y+.5)
        plt.show()

    def stateAt(self,x,y,theta):
        for st in self.C:
            if st.x == x and st.y == y and st.theta == theta:
                return st     
        



if __name__ == "__main__":
    can = Obstacle(10,10,5,5)
    obs = [can]
    robot = Robot(20,20,10)
    env = Environment(40,60,12, robot, obstacles=obs)
    
    env.show()