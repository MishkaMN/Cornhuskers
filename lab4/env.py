import numpy as np
import matplotlib.pyplot as plt
class Environment:
    def __init__(self):
        x = np.linspace(0,39,num=40)
        y = np.linspace(0,59,num=60)
        theta = np.linspace(0,348,num=30)
        self.C = np.zeros([40,60,30])
        for xx in range(len(x)):
            for yy in range(len(y)):
                for tt in range(len(theta)):
                    self.C[xx][yy][tt] = xx+yy+tt

if __name__ == "__main__":
        env = Environment()