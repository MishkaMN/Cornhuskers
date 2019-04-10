import numpy as np
from env import *
import math

def collides(s):    #check if state/point collides with the obstacle
    for rect in rectObs:
        if rect.collidepoint(p) == True:
            return True
    return False

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
        curr_distance = dist(point, target)
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

