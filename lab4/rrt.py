import numpy as np
import time
import pickle
from utils import *
from env import *
from robot import *
from visualizer import Visualizer
import pdb

delta = 10 # Distance the robot can run for 1sec.


def dist(s1,s2):
	"""
	Returns the distance between 2 states
	Args:
		p1: initial state
		p2: target state
	Returns:
		d: distance between two states
	"""

	return sqrt(pow((s1.x-s2.x),2)+pow((s1.y-s2.y),2))


def get_control_input(s1,s2):

def sim_step_from_to(s1, s2):
	"""

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