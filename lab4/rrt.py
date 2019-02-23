
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

def sim_step_from_to(s1, s2):
	"""
    2.2c
	This func makes the robot step 1 sec to target state from initial state
	Args:
		p1: initial state
		p2: target state (with any heading because only when physically turning we use theta)
	Returns:
		p: actual state ended with
	"""
	if dist(s1,s2) < delta:
        return s2
    else:
        theta = atan2(s2.y-s1.y,s2.x-s1.x)
        return s1.x + delta*cos(theta), s1.y + delta*sin(theta)

def collides(s):    #check if state/point collides with the obstacle
    for rect in rectObs:
        if rect.collidepoint(p) == True:
            return True
    return False



# Return Euclidean distance between two States
def distance(s0, s1):
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

