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
from std_srvs.srv import Empty
from std_msgs.msg import Float32MultiArray
from functools import partial
from time import perf_counter_ns
from matplotlib import pyplot as plt
from scipy.spatial import ConvexHull

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
        self.len_ts = 1.75
        self.wid_ts = 3.0
        self.len_ts_1 = 1.75
        self.wid_ts_1 = 3.0
        self.last_pos = None
        self.last_time = None
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
               
        self.pose_sub_os = self.create_subscription(
            PoseWithCovarianceStamped, "/marus_boat/pose", self.pose_callback_os, 10)
        self.thruster_pub_os = self.create_publisher(
            Float32MultiArray, "/marus_boat/pwm_out", 10)

        self.pose_sub_ts_1 = self.create_subscription(
            PoseWithCovarianceStamped, "/marus_boat_2/pose", self.pose_callback_ts_1, 10)
        self.thruster_pub_os_ts_1 = self.create_publisher(
            Float32MultiArray, "/marus_boat_2/pwm_out", 10)

        self.pose_sub_ts_2 = self.create_subscription(
            PoseWithCovarianceStamped, "/marus_boat_3/pose", self.pose_callback_ts_2, 10)
        self.thruster_pub_ts_2 = self.create_publisher(
            Float32MultiArray, "/marus_boat_3/pwm_out", 10)
       
        self.get_logger().info("Closed Loop started!")
        

    def pose_callback_ts_1(self, pose: PoseWithCovarianceStamped):
        self.time_sec_ts_1 = pose.header.stamp.sec
        self.time_nanosec_ts_1 = pose.header.stamp.nanosec
        self.time = self.time_sec_ts_1 + self.time_nanosec_ts_1*(10**-9)
        self.pos_x_ts_1 = pose.pose.pose.position.x
        self.pos_y_ts_1 = pose.pose.pose.position.y
        self.pos_ts_1 = np.array([self.pos_x_ts_1,self.pos_y_ts_1])
        # print("sec ", self.time_sec_ts_1, " nanosec ", self.time_nanosec_ts_1, " posX ", self.pos_x_ts_1, " posY ", self.pos_y_ts_1 ) 

        if self.last_pos is None:
            self.last_pos = self.pos_ts_1
        if self.last_time is None:
            self.last_time = self.time

        self.time_diff = self.time - self.last_time
        self.pos_diff = self.pos_ts_1 - self.last_pos

        
        if self.time_diff > 0.0:
            self.dist = np.sqrt(self.pos_diff[0] ** 2 + self.pos_diff[1] ** 2)
            self.speed = self.dist / self.time_diff
            self.ang = math.degrees(np.arctan2(self.pos_diff[1], self.pos_diff[0]))
            self.ang = (self.ang+360) % 360
            self.ang = velobst.calc_ang_n_to_e(self.ang)
            self.get_logger().info("Speed " + str(self.speed) + " Angle " + str(self.ang) )
            #print("Sec ", self.time_sec_ts_1, "Nanosec ", self.time_nanosec_ts_1)

        self.last_time = self.time
        self.last_pos = self.pos_ts_1
        
        msg = Float32MultiArray()
        msg.data = [10.0, 10.0, 0.0]
        self.thruster_pub_os_ts_1.publish(msg)

    def pose_callback_ts_2(self, pose: PoseWithCovarianceStamped):
        self.time_sec_ts_2 = pose.header.stamp.sec
        self.time_nanosec_ts_2 = pose.header.stamp.nanosec
        self.pos_x_ts_2 = pose.pose.pose.position.x
        self.pos_y_ts_2 = pose.pose.pose.position.y
        
        # self.get_logger().info("Dies Das" + str(self.pos_x_ts_2) + "Das" + str(self.pos_y_ts_2) ) 
        
        # msg = Float32MultiArray()
        # msg.data = [10.0, 10.0, 0.0]
        # self.thruster_pub_ts_2.publish(msg)   


    def pose_callback_os(self, pose: PoseWithCovarianceStamped):
        self.time_sec = pose.header.stamp.sec
        self.time_nanosec = pose.header.stamp.nanosec
        self.pos_x_os = pose.pose.pose.position.x
        self.pos_y_os = pose.pose.pose.position.y
        self.quat_x_os = pose.pose.pose.orientation.x
        self.quat_y_os = pose.pose.pose.orientation.y
        self.quat_z_os = pose.pose.pose.orientation.z
        self.quat_w_os = pose.pose.pose.orientation.w  
        quaternion = [
            self.quat_x_os,
            self.quat_y_os,
            self.quat_z_os,
            self.quat_w_os]
        euler = euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]    
        yaw = euler[2]
        # self.get_logger().info("x " + str(roll) + " y " + str(pitch) + " z " + str(yaw) )

        # msg = Float32MultiArray()
        # msg.data = [10.0, 10.0, 0.0]
        # self.thruster_pub_os.publish(msg)

    # def pose_callback(self, pose: Pose):
    #     self.pos_x_os = pose.x
    #     self.pos_y_os = pose.y
    #     self.ang_os = velobst.calc_ang_n_to_e(math.degrees(pose.theta))
    #     self.speed_os = pose.linear_velocity
    #     # self.get_logger().info("Vel OS " + str(self.speed_os) + " " + str(self.ang_os))
    #     self.os_pos.append((pose.x,pose.y))
    #     self.pos_x_ts_rel = self.pos_x_ts - self.pos_x_os
    #     self.pos_y_ts_rel = self.pos_y_ts - self.pos_y_os
    #     self.pos_rel = (self.pos_x_ts_rel, self.pos_y_ts_rel)
    #     self.pos_ts_rel = np.array(self.pos_rel)

    #     self.pos_x_ts_rel_1 = self.pos_x_ts_1 - self.pos_x_os
    #     self.pos_y_ts_rel_1 = self.pos_y_ts_1 - self.pos_y_os
    #     self.pos_rel_1 = (self.pos_x_ts_rel_1, self.pos_y_ts_rel_1)
    #     self.pos_ts_rel_1 = np.array(self.pos_rel_1)

    #     self.pos_TP = np.array([11.0, 11.0])
    #     pos_TP_rel = (self.pos_TP[0]-self.pos_x_os, self.pos_TP[1]-self.pos_y_os)
    #     ang_TP = math.degrees(np.arctan2(pos_TP_rel[1]+2, pos_TP_rel[0]+2))
    #     ang_TP = (ang_TP+360) % 360
    #     ang_TP = velobst.calc_ang_n_to_e(ang_TP)
    #     vel_OS = np.array([self.speed_os, self.ang_os])
        
    #     vel_OSxy = velobst.vect_to_xy(vel_OS)
    #     ang_OS_rad = np.deg2rad(self.ang_os)
    #     vel_des = np.array([0.5, ang_TP])
    #     self.TS = np.array([[self.pos_ts_rel,self.len_ts,self.wid_ts, self.speed_ts, self.ang_ts]],dtype=object)
    #     self.TS_1 = np.array([[self.pos_ts_rel_1,self.len_ts_1,self.wid_ts_1, self.speed_ts_1, self.ang_ts_1]],dtype=object)
    #     self.TS_all = np.vstack((self.TS, self.TS_1))
    #     self.OS = np.array([vel_OS, vel_OSxy,ang_OS_rad,vel_des], dtype=object)
        
    #     self.new_vel = velobst.calc_vel_final(self.TS, self.OS, False, np.array([0,0]))
    #     self.thetime = round(((perf_counter_ns()-self.start_time)/1000000000), 3)
    #     print("New vel: ", self.new_vel, "Time: ",(perf_counter_ns()-self.start_time)/1000000)
    #     if velobst.check_between_angles(self.new_vel[1], vel_OS[1], (vel_OS[1]+180) % 360):
    #         rot = True # Clockwise
    #     else:
    #         rot = False # Counter-Clockwise
    #     if rot and velobst.ang_betw_vect(self.new_vel, vel_OS) > 5:
    #         cmd = Twist()
    #         cmd.linear.x = self.new_vel[0]
    #         cmd.angular.z = -0.75
    #     elif not rot and velobst.ang_betw_vect(self.new_vel, vel_OS) > 5:
    #         cmd = Twist()
    #         cmd.linear.x = self.new_vel[0]
    #         cmd.angular.z = 0.75
    #     else:
    #         cmd = Twist()
    #         cmd.linear.x = vel_des[0]
    #         cmd.angular.z = 0.0
    #     self.cmd_vel_pub_.publish(cmd)

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
