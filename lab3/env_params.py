import numpy as np

# Environment, System Parameters

headings = range(12)
UP = headings[-1:] + headings[:2]
RIGHT = headings[2:5]
DOWN = headings[5:8]
LEFT = headings[8:11]

W = 6
L = 6
rewards = np.flipud(np.array([
    [-100, -100, -100, -100, -100, -100], 
    [-100,    0,    0,  -10,    1, -100],
    [-100,    0,    0,  -10,    0, -100],
    [-100,    0,    0,    0,    0, -100],
    [-100,    0,    0,    0,    0, -100],
    [-100, -100, -100, -100, -100, -100]
]));

goal_x = 4
goal_y = 4
goal_h = headings