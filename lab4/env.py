import numpy as np
import matplotlib.pyplot as plt

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

class Environment:
    def __init__(self, Nx, Ny, Nt, obstacles=None):
        """
        Obstacles format:
        rectangular: x,y,w,l
        """
        x = np.linspace(0,Nx-1,num=Nx)
        y = np.linspace(0,Ny-1,num=Ny)
        theta = np.linspace(0,(360-360/Nt),num=Nt)
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
                    



if __name__ == "__main__":
    can = Obstacle(10,10,10,10)
    obs = [can]
    env = Environment(40,60,12,obstacles=obs)
    for V in env.C:
        print("V: ", V.x, V.y, V.theta, V.clear)