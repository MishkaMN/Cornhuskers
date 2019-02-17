import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import AutoMinorLocator
import matplotlib.animation as animation

class Visualizer:
    def __init__(self, sequence):
        self.sequence = sequence
    


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