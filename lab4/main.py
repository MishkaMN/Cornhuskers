from robot import *
from env import *

L = 610.0
W = 406.0
numHeading = 12

goal_state = State(400, 400, 0)
robot = Robot(200, 200, 0)
env = Environment(W, L, numHeading, robot, goal_state)
