class TS:

    def __init__(self, len_ts, wid_ts, pos_ts, speed_ts, ang_ts):
        self.colreg_init = []
        self.os_init = []
        
        self.pos = pos_ts
        self.length = len_ts
        self.width = wid_ts
        self.speed = speed_ts
        self.ang = ang_ts
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
