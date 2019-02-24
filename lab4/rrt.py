
import numpy as np
import time
import pickle
from utils import *
from env import *
from robot import *
from visualizer import Visualizer
import pdb
import math

delta = 10 # Distance the robot can run for 1sec.



def get_control_input(s1,s2):
    #2.2c
    return




# Return Euclidean distance between two States
def dist(s0, s1):
    return math.sqrt((s0.x - s1.x)**2 + (s0.y - s1.y)**2)

# Given a set of points V in C-space and single other target point,
# determine which points in V are closest to target
def nearestNeighbors(V, target):
    res = []
    min_distance = None

    # Each point is a State object
    for point in V:
        curr_distance = distance(point, target)
        if min_distance is None:
            res.append(point)
            min_distance = curr_distance
        elif curr_distance < min_distance:
            res = [point]
            min_distance = curr_distance
        # Can return multiple points if necessary
        elif curr_distance == min_distance:
            res.append(point)

    return res

