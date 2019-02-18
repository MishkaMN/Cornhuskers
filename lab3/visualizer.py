import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import AutoMinorLocator
import matplotlib.animation as animation
from robot import *
import time 
X = 0
O = 2
G = 3
P = 1

class Visualizer:
    def __init__(self, sequence):
        self.sequence = sequence
    

    def make_arena(self):
        arena = np.array([[X,X,X,X,X,X,X,X,X],    
                     [X,O,O,O,O,O,P,O,X],    
                     [X,O,O,O,O,O,O,O,X],    
                     [X,O,O,O,O,O,O,O,X],    
                     [X,O,O,O,O,O,O,O,X],
                     [X,O,O,O,O,P,O,O,X],
                     [X,O,O,O,O,P,O,G,X],
                     [X,O,O,O,O,P,O,O,X],
                     [X,X,X,X,X,X,X,X,X]])
        arena = np.flip(arena,0)
        return arena

    def show(self):

        x = []
        y = []
        h = []
        U = []
        V = []
        #Adjust ticks and grid lines
        
        fig = plt.figure();
        plt.xlim((0,9))
        plt.ylim((0,9))
        ax = plt.gca();
        ax.set_xticks(np.arange(0.5, 9.5, 1));
        ax.set_yticks(np.arange(0.5, 9.5, 1));
        ax.set_xticklabels(np.arange(0, 9, 1));
        ax.set_yticklabels(np.arange(0, 9, 1));
        arena = self.make_arena()
        ax.pcolor(arena, edgecolors='k', linestyle= 'dashed', linewidths=0.2, cmap='RdYlGn', vmin=0.0, vmax=3.0)
        minor_locator = AutoMinorLocator(2)
        plt.gca().xaxis.set_minor_locator(minor_locator)
        plt.gca().yaxis.set_minor_locator(minor_locator)
        plt.grid(which='minor')
        line, = ax.plot([], [], lw=2)

        time.sleep(5)
        def animate(st):
            x.append(st[0]+.5)
            y.append(st[1]+.5)
            h.append(st[2])
            U.append(np.cos(-st[2]*2*np.pi/12+np.pi/2))
            V.append(np.sin(-st[2]*2*np.pi/12+np.pi/2))
            line.set_data(x,y)
            ax.quiver(x,y, U, V)


        ani = animation.FuncAnimation(fig, animate, frames=self.sequence, interval=200)
        plt.show()

    def showPolicy(self,policy):

        fig = plt.figure();
        plt.xlim((0,9))
        plt.ylim((0,9))
        ax = plt.gca();
        ax.set_xticks(np.arange(0.5, 9.5, 1));
        ax.set_yticks(np.arange(0.5, 9.5, 1));
        ax.set_xticklabels(np.arange(0, 6, 1));
        ax.set_yticklabels(np.arange(0, 6, 1));
        arena = self.make_arena()
        ax.pcolor(arena, edgecolors='k', linestyle= 'dashed', linewidths=0.2, cmap='RdYlGn', vmin=0.0, vmax=3.0)
        minor_locator = AutoMinorLocator(2)
        plt.gca().xaxis.set_minor_locator(minor_locator)
        plt.gca().yaxis.set_minor_locator(minor_locator)
        plt.grid(which='minor')
        
        X = []
        Y = []
        U = []
        V = []
        for h_idx,h in enumerate(policy):
            for y_idx,y_layer in enumerate(h):
                for x_idx,a in enumerate(y_layer):
                    X.append(x_idx + .5)
                    Y.append(y_idx + .5)
                    tmp = np.array([0,0])
                    
                    if a == Action.STAY:
                        U.append(tmp[0])
                        V.append(tmp[1])
                    else:
                        if a in fw_actions:
                            tmp[0] = 1
                        elif a in bw_actions:
                            tmp[0] = -1
                        if a in clk_actions:
                            correction = np.pi*2.0/12.0;
                        elif a in cclk_actions:
                            correction = -np.pi*2.0/12.0;
                        else:
                            correction = 0;
                        h_u = np.cos(-1*(h_idx*2*np.pi/12 + correction)+np.pi/2)
                        h_v = np.sin(-1*(h_idx*2*np.pi/12 + correction)+np.pi/2)
                        h_uv = np.array([h_u, h_v])
                        vec = tmp[0] * h_uv
                        U.append(vec[0])
                        V.append(vec[1])
            
        ax.quiver(X,Y,U,V)
        plt.show()