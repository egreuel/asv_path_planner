"""
Module for target ships (TS)/obstacles consisting of one class

"""

class TS:
    """
    Class that stores information about TS/obstacles:
    
    - Position
    - Length
    - Width
    - Speed
    - Orientation

    and other that are stored during the collision avoidance algorithm.

    """
    def __init__(self,length):
        
        self.colreg_init = []
        self.os_init = []
        
        self.pos = []
        self.length = []
        self.width = []
        self.speed = []
        self.ang = []
        self.vo = []
        self.vert_hull = []
        self.tang_points = []
        self.points_colreg_line = []
        self.coll_check = []
        self.coll_check_des = []
        self.colreg_rule = None
        self.colreg_con = []
        self.coll_point = []
        self.coll_dist = []
        self.coll_time = []
