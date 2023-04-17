#!/usr/bin/env python3
from asv_path_planner import velobst
from .velobst_class import VO
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
        atexit.register(self.exit_handler)
        self.os_pos = []
        self.os_speed = []
        self.os_ang = []
        self.ts_pos_1 = []
        self.ts_pos_2 = []
        self.dist_os_ts_1 = []
        self.dist_os_ts_2 = []
        self.simu_time = []
        self.start_time = perf_counter_ns()
        self.len_os = 3.0
        self.wid_os = 1.75
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
        self.os_max_speed = 6.0
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

        self.ts_pos_1.append(self.gps_1)

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

        self.ts_pos_2.append(self.gps_2)

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

        self.os_pos.append(self.gps)

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

        self.test_vo = VO(3.0, 1.75, 6.0, 15, 7.5, 3, 0.5, 5, 0.25, 3)

        # Calculate relative position of TS to OS
        self.pos_ts_rel_1 = self.test_vo.calc_coord_gps_to_xy(self.gps, self.gps_1)
        # self.pos_ts_rel_2 = velobst.calc_coord_gps_to_xy(self.gps, self.gps_2)

        self.thetime = round(((perf_counter_ns()-self.start_time)/1000000000), 3)
        self.simu_time.append(self.thetime)

        # Calculate distance between OS and TS and save it in an list
        self.dist_os_ts_1.append(distance.great_circle(self.gps, self.gps_1).meters) 
        # self.dist_os_ts_2.append(distance.great_circle(self.gps, self.gps_2))
         
        # Save speed and orientation of the OS in a list
        self.os_speed.append(self.speed_gps)
        self.os_ang.append(self.ang_gps)  

        # Calculate the relative position of the target point to OS
        self.gps_tp = np.array([45.0017996649828, 14.999999999999998])
        pos_TP_rel = self.test_vo.calc_coord_gps_to_xy(self.gps, self.gps_tp)
        ang_TP = math.degrees(np.arctan2(pos_TP_rel[1]+2, pos_TP_rel[0]+2))
        ang_TP = (ang_TP+360) % 360
        ang_TP = self.test_vo.calc_ang_n_to_e(ang_TP)

        vel_OS = np.array([self.speed_gps, self.ang_gps])
        vel_OSxy = self.test_vo.vect_to_xy(vel_OS)
        ang_OS_rad = np.deg2rad(self.ang_gps)
        vel_des = np.array([3.0, ang_TP])
        
        self.TS_1 = np.array([[self.pos_ts_rel_1,self.len_ts_1,self.wid_ts_1, self.speed_gps_1, self.ang_gps_1]],dtype=object)
        # self.TS_2 = np.array([[self.pos_ts_rel_2,self.len_ts_2,self.wid_ts_2, self.speed_gps_2, self.ang_gps_2]],dtype=object)
        # self.TS_all = np.vstack((self.TS_1, self.TS_2))
        self.OS = np.array([vel_OS, vel_OSxy,ang_OS_rad,vel_des], dtype=object)


        self.new_vel = self.test_vo.calc_vel_final(self.TS_1, self.OS, False, np.array([0,0]))
        
        print("New vel: ", self.new_vel, "Time: ",(perf_counter_ns()-self.start_time)/1000000)
        if self.test_vo.check_between_angles(self.new_vel[1], vel_OS[1], (vel_OS[1]+180) % 360):
            rot = True # Clockwise
        else:
            rot = False # Counter-Clockwise
        if rot and self.test_vo.ang_betw_vect(self.new_vel, vel_OS) > 5:
            msg = Float32MultiArray()
            msg.data = [self.new_vel[0]/self.os_max_speed,self.new_vel[0]/(self.os_max_speed*2), 0.0]
        elif not rot and self.test_vo.ang_betw_vect(self.new_vel, vel_OS) > 5:
            msg = Float32MultiArray()
            msg.data = [self.new_vel[0]/(self.os_max_speed*2), self.new_vel[0]/self.os_max_speed, 0.0]
        else:
            msg = Float32MultiArray()
            msg.data = [vel_des[0]/self.os_max_speed, vel_des[0]/self.os_max_speed, 0.0]
        self.thruster_pub_os.publish(msg)

        # # Calculate relative position of TS to OS
        # self.pos_ts_rel_1 = velobst.calc_coord_gps_to_xy(self.gps, self.gps_1)
        # # self.pos_ts_rel_2 = velobst.calc_coord_gps_to_xy(self.gps, self.gps_2)

        # self.thetime = round(((perf_counter_ns()-self.start_time)/1000000000), 3)
        # self.simu_time.append(self.thetime)

        # # Calculate distance between OS and TS and save it in an list
        # self.dist_os_ts_1.append(distance.great_circle(self.gps, self.gps_1).meters) 
        # # self.dist_os_ts_2.append(distance.great_circle(self.gps, self.gps_2))
         
        # # Save speed and orientation of the OS in a list
        # self.os_speed.append(self.speed_gps)
        # self.os_ang.append(self.ang_gps)  

        # # Calculate the relative position of the target point to OS
        # self.gps_tp = np.array([45.0017996649828, 14.999999999999998])
        # pos_TP_rel = velobst.calc_coord_gps_to_xy(self.gps, self.gps_tp)
        # ang_TP = math.degrees(np.arctan2(pos_TP_rel[1]+2, pos_TP_rel[0]+2))
        # ang_TP = (ang_TP+360) % 360
        # ang_TP = velobst.calc_ang_n_to_e(ang_TP)

        # vel_OS = np.array([self.speed_gps, self.ang_gps])
        # vel_OSxy = velobst.vect_to_xy(vel_OS)
        # ang_OS_rad = np.deg2rad(self.ang_gps)
        # vel_des = np.array([3.0, ang_TP])
        
        # self.TS_1 = np.array([[self.pos_ts_rel_1,self.len_ts_1,self.wid_ts_1, self.speed_gps_1, self.ang_gps_1]],dtype=object)
        # # self.TS_2 = np.array([[self.pos_ts_rel_2,self.len_ts_2,self.wid_ts_2, self.speed_gps_2, self.ang_gps_2]],dtype=object)
        # # self.TS_all = np.vstack((self.TS_1, self.TS_2))
        # self.OS = np.array([vel_OS, vel_OSxy,ang_OS_rad,vel_des], dtype=object)


        # self.new_vel = velobst.calc_vel_final(self.TS_1, self.OS, False, np.array([0,0]))
        
        # print("New vel: ", self.new_vel, "Time: ",(perf_counter_ns()-self.start_time)/1000000)
        # if velobst.check_between_angles(self.new_vel[1], vel_OS[1], (vel_OS[1]+180) % 360):
        #     rot = True # Clockwise
        # else:
        #     rot = False # Counter-Clockwise
        # if rot and velobst.ang_betw_vect(self.new_vel, vel_OS) > 5:
        #     msg = Float32MultiArray()
        #     msg.data = [self.new_vel[0]/self.os_max_speed,self.new_vel[0]/(self.os_max_speed*2), 0.0]
        # elif not rot and velobst.ang_betw_vect(self.new_vel, vel_OS) > 5:
        #     msg = Float32MultiArray()
        #     msg.data = [self.new_vel[0]/(self.os_max_speed*2), self.new_vel[0]/self.os_max_speed, 0.0]
        # else:
        #     msg = Float32MultiArray()
        #     msg.data = [vel_des[0]/self.os_max_speed, vel_des[0]/self.os_max_speed, 0.0]
        # self.thruster_pub_os.publish(msg)
    
    def exit_handler(self):
        os_position = np.array(self.os_pos)
        ts_position = np.array(self.ts_pos_1)
        #ts1_position = np.array(self.ts_pos_2)
        os_speed = np.array(self.os_speed)
        os_ang = np.array(self.os_ang)
        dist_os_ts_1 = np.array(self.dist_os_ts_1)
        dist_os_ts_2 = np.array(self.dist_os_ts_2)
        simu_time = np.array(self.simu_time)
        new_vel_xy = self.test_vo.vect_to_xy(self.new_vel) 

        # plt.plot(simu_time, os_speed)
        # plt.plot(simu_time, os_ang)
        plt.plot(simu_time, dist_os_ts_1)

        # plt.plot(os_position[:,1], os_position[:,0], c="teal",zorder=0.5, linestyle="--", label="OS path")
        # plt.plot(ts_position[:,1], ts_position[:,0], c="red",zorder=0.05, linestyle="-.", label="TS 1 path")
        # #plt.plot(ts1_position[:,0], ts1_position[:,1], c="red",zorder=0.05, linestyle=":", label="TS 1 path")
        # plt.plot((os_position[0,1],self.gps_tp[1]),(os_position[0,0],self.gps_tp[0]),c="gray",zorder=0.02, alpha=1.0, label="global path")
        # plt.scatter(self.gps_tp[1],self.gps_tp[0],c="dimgray", marker="+", label="OS goal")
        # plt.quiver(os_position[-1,1], os_position[-1,0], new_vel_xy[0]*(10**-4), new_vel_xy[1]*(10**-4),scale=1,
        #             scale_units='xy', angles='xy', color='blue', zorder=6,width=0.005, hatch="----", edgecolor="black", linewidth=0.5, label="v\u20D7 new")
        
        # velobst.calc_vel_final(self.TS_1, self.OS, True, self.os_pos[-1])
        plt.axis('scaled')
        # plt.axis([14.996956254877997,15.008243476322948,44.997525162211005,45.00584886852189])

        # plt.title("time = " + str(self.thetime) + " s")
        # plt.xlabel('Longitude [°]')
        # plt.ylabel('Latitude [°]')
        # # plt.legend(["OS path", "TS 1 path", "TS 2 path", "desired path", "target position", "new vel", "desired vel","OS","safety domain","TS"])
        
        # #get handles and labels
        # handles, labels = plt.gca().get_legend_handles_labels()

        # #specify order of items in legend
        # # order = [9,7,12,8,0,1,2,3,4,10,5,6,11]
        # order = [8,6,7,0,1,2,3,9,4,5,10]

        #add legend to plot
        #plt.legend([handles[idx] for idx in order],[labels[idx] for idx in order]) 
        # current_values_x = plt.gca().get_xticks()
        # current_values_y = plt.gca().get_yticks()
        # plt.gca().set_xticklabels(['{:.4f}'.format(x) for x in current_values_x])
        # plt.gca().set_yticklabels(['{:.4f}'.format(x) for x in current_values_y])

        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
