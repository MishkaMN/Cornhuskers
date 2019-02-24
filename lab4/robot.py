class Robot:
    def __init__(self, x, y, heading):
        # p_e: prerotate probability
        
        # 11, 0, 1 --> north
        # 2, 3, 4 --> east 
        self.x = x
        self.y = y
        self.heading = heading

    def rotate(self, rotate):
        pass

    def move(self, x, y, heading):
        pass