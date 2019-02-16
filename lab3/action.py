from enum import Enum
from env_params import *

class Action(Enum):
    STAY = 0
    FORWARD_NOROT = 1
    FORWARD_CLK = 2
    FORWARD_CCLK = 3
    BACKWARD_NOROT = 4
    BACKWARD_CLK = 5
    BACKWARD_CCLK = 6
    
fw_actions = [Action.FORWARD_NOROT, Action.FORWARD_CLK, Action.FORWARD_CCLK]
bw_actions = [Action.BACKWARD_NOROT, Action.BACKWARD_CLK, Action.BACKWARD_CCLK]
clk_actions = [Action.FORWARD_CLK, Action.BACKWARD_CLK];
cclk_actions = [Action.FORWARD_CCLK, Action.BACKWARD_CCLK];
#  2.1(b) Action space, N_a = 7;
actions = [Action.STAY, Action.FORWARD_NOROT, Action.FORWARD_CLK, Action.FORWARD_CCLK, Action.BACKWARD_NOROT, Action.BACKWARD_CLK, Action.BACKWARD_CCLK]
