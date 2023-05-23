"""
This is a minimal example on how to use the collision avoidance algorithm

"""

#!/usr/bin/env python3
from velobst_class import VO
from ts_class import TS

# Properties of the own ship (OS)
vel_OS = [3.0, 90]
vel_des = [3.0, 90]
os = [vel_OS, vel_des]

# Declare an object of the target ship (TS) class to initalize an obstacle and add properties
# to the obstacle (e.g. obtained by sensors or AIS)
ts = TS()
ts.pos = [100,0]
ts.length = 6.0
ts.width = 3.0
ts.speed = 10.0
ts.ang = 270

# Add all obstacles to a list
all_ts = [ts]

# Declare an object of the velocity obstacle class with inital parameters needed by the
# algorithm
vo = VO(3.0,1.5,5.0,120, 60, 5, 0.15, 1.5, 0.25, 3)

# Use the collision avoidance algorithm and calculate new velocity for the OS
new_vel = vo.calc_vel_final(all_ts, os, False)

print("New velocity is: ", new_vel)