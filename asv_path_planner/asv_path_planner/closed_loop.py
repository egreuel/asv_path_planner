#!/usr/bin/env python3
from asv_path_planner import velobst
import atexit
import numpy as np
import math
import rclpy
from tf_transformations import euler_from_quaternion
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Empty
from std_msgs.msg import Float32MultiArray
from functools import partial
from time import perf_counter_ns
from matplotlib import pyplot as plt
from scipy.spatial import ConvexHull
from geopy import distance

class ClosedLoopNode(Node):

    def __init__(self):
        super().__init__("closed_loop")
        # atexit.register(self.exit_handler)
        self.os_pos = []
        self.ts_pos = []
        self.ts1_pos = []
        self.start_time = perf_counter_ns()
        self.wid_os = 1.75
        self.len_os = 3.0
        self.len_ts_1 = 1.75
        self.wid_ts_1 = 3.0
        self.len_ts_2 = 1.75
        self.wid_ts_2 = 3.0
        self.last_gps = None
        self.last_gps_time = None
        self.last_gps_1 = None
        self.last_gps_time_1 = None
        self.last_gps_2 = None
        self.last_gps_time_2 = None
        self.max_speed_os = 3.0
        # self.len_os = 0.5
        # self.wid_os = 0.2
        # self.max_ttc = 5
        # self.safe_fact = 3
        # self.max_speed_os = 2
        # self.unc_speed = 0.05
        # self.unc_ang = 5
        # self.w_1 = 1
        # self.w_2 = 2
        # self.w_3 = 0.5
        # self.w_4 = 0.1
        # self.n = 0.1
        # self.m = 5
        # self.threshold = 3
        # self.plotting = False

        self.gps_sub_tp = self.create_subscription(
            NavSatFix, "/target_point", self.gps_callback_tp, 10)

        self.gps_sub_ts_1 = self.create_subscription(
            NavSatFix, "/marus_boat_2/gps", self.gps_callback_ts_1, 10)
        self.thruster_pub_os_ts_1 = self.create_publisher(
            Float32MultiArray, "/marus_boat_2/pwm_out", 10)

        self.gps_sub_ts_2 = self.create_subscription(
            NavSatFix, "/marus_boat_3/gps", self.gps_callback_ts_2, 10)
        self.thruster_pub_ts_2 = self.create_publisher(
            Float32MultiArray, "/marus_boat_3/pwm_out", 10)

        self.gps_sub_os = self.create_subscription(
            NavSatFix, "/marus_boat/gps", self.gps_callback_os, 10)
        self.thruster_pub_os = self.create_publisher(
            Float32MultiArray, "/marus_boat/pwm_out", 10)

        self.get_logger().info("Closed Loop started!")

    def gps_callback_ts_1(self, pose: NavSatFix):
        gps_time_sec = pose.header.stamp.sec
        gps_time_nanosec =  pose.header.stamp.nanosec
        gps_time = gps_time_sec + gps_time_nanosec*(10**-9)
        gps_lat = pose.latitude
        gps_lon = pose.longitude
        self.gps_1 = np.array([gps_lat, gps_lon])
        if self.last_gps_1 is None:
            self.last_gps_1 = self.gps_1
        if self.last_gps_time_1 is None:
            self.last_gps_time_1 = gps_time

        gps_time_diff = gps_time - self.last_gps_time_1
                
        if gps_time_diff > 0.0:
            dist_gps = distance.great_circle(self.gps_1, self.last_gps_1)
            self.speed_gps_1 = dist_gps.meters / gps_time_diff
            ang_x = math.cos(self.gps_1[0]) * math.sin(self.gps_1[1]-self.last_gps_1[1])
            ang_y = math.cos(self.last_gps_1[0]) * math.sin(self.gps_1[0]) - math.sin(self.last_gps_1[0]) * math.cos(self.gps_1[0]) * math.cos(self.gps_1[1]-self.last_gps_1[1])
            self.ang_gps_1 = np.rad2deg(np.arctan2(ang_x, ang_y))
            self.ang_gps_1 = (self.ang_gps_1+360) % 360
            #  print("Speed ", self.speed_gps_1, "Angle ", self.ang_gps_1)
        else:
            self.speed_gps_1 = 0
            self.ang_gps_1 = 0
            
        self.last_gps_time_1 = gps_time
        self.last_gps_1 = self.gps_1

        msg = Float32MultiArray()
        msg.data = [1.0, 1.0, 0.0]
        self.thruster_pub_os_ts_1.publish(msg)

    def gps_callback_ts_2(self, pose: NavSatFix):
        gps_time_sec = pose.header.stamp.sec
        gps_time_nanosec =  pose.header.stamp.nanosec
        gps_time = gps_time_sec + gps_time_nanosec*(10**-9)
        gps_lat = pose.latitude
        gps_lon = pose.longitude
        self.gps_2 = np.array([gps_lat, gps_lon])
        if self.last_gps_2 is None:
            self.last_gps_2 = self.gps_2
        if self.last_gps_time_2 is None:
            self.last_gps_time_2 = gps_time

        gps_time_diff = gps_time - self.last_gps_time_2
                
        if gps_time_diff > 0.0:
            dist_gps = distance.great_circle(self.gps_2, self.last_gps_2)
            self.speed_gps_2 = dist_gps.meters / gps_time_diff
            ang_x = math.cos(self.gps_2[0]) * math.sin(self.gps_2[1]-self.last_gps_2[1])
            ang_y = math.cos(self.last_gps_2[0]) * math.sin(self.gps_2[0]) - math.sin(self.last_gps_2[0]) * math.cos(self.gps_2[0]) * math.cos(self.gps_2[1]-self.last_gps_2[1])
            self.ang_gps_2 = np.rad2deg(np.arctan2(ang_x, ang_y))
            self.ang_gps_2 = (self.ang_gps_2+360) % 360
            # print("Speed ", self.speed_gps_2, "Angle ", self.ang_gps_2)
        else:
            self.speed_gps_2 = 0
            self.ang_gps_2 = 0
            
        self.last_gps_time_2 = gps_time
        self.last_gps_2 = self.gps_2

        # msg = Float32MultiArray()
        # msg.data = [10.0, 10.0, 0.0]
        # self.thruster_pub_os_ts_1.publish(msg)


    def gps_callback_tp(self, pose: NavSatFix):
        gps_lat = pose.latitude
        gps_lon = pose.longitude
        # self.gps_tp = np.array([gps_lat, gps_lon])


    def gps_callback_os(self, pose: PoseWithCovarianceStamped):
        gps_time_sec = pose.header.stamp.sec
        gps_time_nanosec =  pose.header.stamp.nanosec
        gps_time = gps_time_sec + gps_time_nanosec*(10**-9)
        gps_lat = pose.latitude
        gps_lon = pose.longitude
        self.gps = np.array([gps_lat, gps_lon])
        if self.last_gps is None:
            self.last_gps = self.gps
        if self.last_gps_time is None:
            self.last_gps_time = gps_time

        gps_time_diff = gps_time - self.last_gps_time
                
        if gps_time_diff > 0.0:
            dist_gps = distance.great_circle(self.gps, self.last_gps)
            self.speed_gps = dist_gps.meters / gps_time_diff
            ang_x = math.cos(self.gps[0]) * math.sin(self.gps[1]-self.last_gps[1])
            ang_y = math.cos(self.last_gps[0]) * math.sin(self.gps[0]) - math.sin(self.last_gps[0]) * math.cos(self.gps[0]) * math.cos(self.gps[1]-self.last_gps[1])
            self.ang_gps = np.rad2deg(np.arctan2(ang_x, ang_y))
            self.ang_gps = (self.ang_gps+360) % 360
            # print("Speed ", self.speed_gps, "Angle ", self.ang_gps)
        else:
            self.speed_gps = 0
            self.ang_gps = 0
            
        self.last_gps_time = gps_time
        self.last_gps = self.gps

        # Calculate relative position of TS to OS
        self.pos_ts_rel_1 = velobst.calc_coord_gps_to_xy(self.gps, self.gps_1)
        # self.pos_ts_rel_2 = velobst.calc_coord_gps_to_xy(self.gps, self.gps_2)

        # Calculate the relative position of the target point to OS
        self.gps_tp = np.array([45.0017996649828, 14.999999999999998])
        pos_TP_rel = velobst.calc_coord_gps_to_xy(self.gps, self.gps_tp)
        ang_TP = math.degrees(np.arctan2(pos_TP_rel[1]+2, pos_TP_rel[0]+2))
        ang_TP = (ang_TP+360) % 360
        ang_TP = velobst.calc_ang_n_to_e(ang_TP)

        vel_OS = np.array([self.speed_gps, self.ang_gps])
        vel_OSxy = velobst.vect_to_xy(vel_OS)
        ang_OS_rad = np.deg2rad(self.ang_gps)
        vel_des = np.array([3.0, ang_TP])
        
        self.TS_1 = np.array([[self.pos_ts_rel_1,self.len_ts_1,self.wid_ts_1, self.speed_gps_1, self.ang_gps_1]],dtype=object)
        # self.TS_2 = np.array([[self.pos_ts_rel_2,self.len_ts_2,self.wid_ts_2, self.speed_gps_2, self.ang_gps_2]],dtype=object)
        # self.TS_all = np.vstack((self.TS_1, self.TS_2))
        self.OS = np.array([vel_OS, vel_OSxy,ang_OS_rad,vel_des], dtype=object)


        self.new_vel = velobst.calc_vel_final(self.TS_1, self.OS, False, np.array([0,0]))
        self.thetime = round(((perf_counter_ns()-self.start_time)/1000000000), 3)
        print("New vel: ", self.new_vel, "Time: ",(perf_counter_ns()-self.start_time)/1000000)
        if velobst.check_between_angles(self.new_vel[1], vel_OS[1], (vel_OS[1]+180) % 360):
            rot = True # Clockwise
        else:
            rot = False # Counter-Clockwise
        if rot and velobst.ang_betw_vect(self.new_vel, vel_OS) > 5:
            msg = Float32MultiArray()
            msg.data = [self.new_vel[0]/self.max_speed_os,self.new_vel[0]/(self.max_speed_os*2), 0.0]
        elif not rot and velobst.ang_betw_vect(self.new_vel, vel_OS) > 5:
            msg = Float32MultiArray()
            msg.data = [self.new_vel[0]/(self.max_speed_os*2), self.new_vel[0]/self.max_speed_os, 0.0]
        else:
            msg = Float32MultiArray()
            msg.data = [vel_des[0]/self.max_speed_os, vel_des[0]/self.max_speed_os, 0.0]
        self.thruster_pub_os.publish(msg)
    
    # def exit_handler(self):
    #     os_position = np.array(self.os_pos)
    #     ts_position = np.array(self.ts_pos)
    #     #ts1_position = np.array(self.ts1_pos)
    #     new_vel_xy = velobst.vect_to_xy(self.new_vel) 
          
    #     plt.plot(os_position[:,0], os_position[:,1], c="teal",zorder=0.5, linestyle="--", label="OS path")
    #     plt.plot(ts_position[:,0], ts_position[:,1], c="red",zorder=0.05, linestyle="-.", label="TS 1 path")
    #     #plt.plot(ts1_position[:,0], ts1_position[:,1], c="red",zorder=0.05, linestyle=":", label="TS 1 path")
    #     plt.plot((0,self.pos_TP[0]+5),(0,self.pos_TP[1]+5),c="gray",zorder=0.02, alpha=1.0, label="global path")
    #     plt.scatter(13.0,13.0,c="dimgray", marker="+", label="OS goal")
    #     plt.quiver(os_position[-1,0], os_position[-1,1], new_vel_xy[0], new_vel_xy[1],scale=1,
    #                 scale_units='xy', angles='xy', color='blue', zorder=6,width=0.005, hatch="----", edgecolor="black", linewidth=0.5, label="v\u20D7 new")
        
    #     velobst.calc_vel_final(self.TS, self.OS, True, self.os_pos[-1])
    #     plt.axis('scaled')
    #     plt.axis([-0.5,14,-0.5,14])
    #     plt.title("time = " + str(self.thetime) + " s")
    #     plt.xlabel('x-coordinate [m]')
    #     plt.ylabel('y-coordinate [m]')
    #     # plt.legend(["OS path", "TS 1 path", "TS 2 path", "desired path", "target position", "new vel", "desired vel","OS","safety domain","TS"])
        
    #     #get handles and labels
    #     handles, labels = plt.gca().get_legend_handles_labels()

    #     #specify order of items in legend
    #     # order = [9,7,12,8,0,1,2,3,4,10,5,6,11]
    #     order = [8,6,7,0,1,2,3,9,4,5,10]

    #     #add legend to plot
    #     #plt.legend([handles[idx] for idx in order],[labels[idx] for idx in order]) 
        
    #     plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
