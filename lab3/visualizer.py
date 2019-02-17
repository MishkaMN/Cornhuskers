import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import AutoMinorLocator
import matplotlib.animation as animation
X = 0
O = 2
G = 3
P = 1

class Visualizer:
    def __init__(self, sequence):
        self.sequence = sequence
    

    def make_arena(self):
        arena = np.array([[X,X,X,X,X,X],    
                     [X,O,O,P,G,X],    
                     [X,O,O,P,O,X],    
                     [X,O,O,O,O,X],    
                     [X,O,O,O,O,X],
                     [X,X,X,X,X,X]])
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
        plt.xlim((0,6))
        plt.ylim((0,6))
        ax = plt.gca();
        ax.set_xticks(np.arange(0.5, 6.5, 1));
        ax.set_yticks(np.arange(0.5, 6.5, 1));
        ax.set_xticklabels(np.arange(0, 6, 1));
        ax.set_yticklabels(np.arange(0, 6, 1));
        arena = self.make_arena()
        ax.pcolor(arena, edgecolors='k', linestyle= 'dashed', linewidths=0.2, cmap='RdYlGn', vmin=0.0, vmax=3.0)
        minor_locator = AutoMinorLocator(2)
        plt.gca().xaxis.set_minor_locator(minor_locator)
        plt.gca().yaxis.set_minor_locator(minor_locator)
        plt.grid(which='minor')
        line, = ax.plot([], [], lw=2)


        def animate(st):
            x.append(st[0]+.5)
            y.append(st[1]+.5)
            h.append(st[2])
            U.append(np.cos(-st[2]*2*np.pi/12+np.pi/2))
            V.append(np.sin(-st[2]*2*np.pi/12+np.pi/2))
            line.set_data(x,y)
            ax.quiver(x,y, U, V)


        ani = animation.FuncAnimation(fig, animate, frames=self.sequence, interval=500)
        plt.show()