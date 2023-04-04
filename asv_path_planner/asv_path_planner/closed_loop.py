#!/usr/bin/env python3
from asv_path_planner import velobst
import atexit
import numpy as np
import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, TeleportAbsolute, Spawn
from std_srvs.srv import Empty
from functools import partial
from time import perf_counter_ns
from matplotlib import pyplot as plt
from scipy.spatial import ConvexHull

class ClosedLoopNode(Node):

    def __init__(self):
        super().__init__("closed_loop")
        atexit.register(self.exit_handler)
        self.os_pos = []
        self.ts_pos = []
        self.ts1_pos = []
        self.start_time = perf_counter_ns()
        self.wid_os = 0.2
        self.len_os = 0.5
        self.len_ts = 0.75
        self.wid_ts = 0.2
        self.len_ts_1 = 1.0
        self.wid_ts_1 = 0.4
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
        self.set_pos(2.0,2.0,math.radians(45))
        #self.spawn_turtle(11.0,11.0, math.radians(-135),"turtle2")
        self.spawn_turtle(11.0, 0.0, math.radians(135),"turtle3")
        #self.set_pos_sec(11.0,11.0, math.radians(-135))
        self.set_pos_thi(11.0, 0.0, math.radians(135))
        self.spawn_turtle(0.0,0.0, math.radians(45),"turtle2")
        # self.spawn_turtle(5.0, 5.0, math.radians(45),"turtle3")
        self.set_pos_sec(0.0,0.0, math.radians(45))
        # self.set_pos_thi(5.0, 5.0, math.radians(45))
        self.call_set_pen_serv(255,0,0,3,0)
        self.call_set_pen_serv_2(255,120,0,3,0)
        
        

        self.cmd_vel_pub_sec_ = self.create_publisher(
            Twist, "/turtle2/cmd_vel", 10)
        self.pose_sub_sec_ = self.create_subscription(
            Pose, "/turtle2/pose", self.pose_callback_sec, 10)
        
        self.cmd_vel_pub_thi_ = self.create_publisher(
            Twist, "/turtle3/cmd_vel", 10)
        self.pose_sub_thi_ = self.create_subscription(
            Pose, "/turtle3/pose", self.pose_callback_thi, 10)
        
        self.cmd_vel_pub_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)
        self.pose_sub_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10)
        
        self.get_logger().info("Closed Loop started!")
        self.clear_path()
        
    def pose_callback_sec(self, pose: Pose):
        self.pos_x_ts = pose.x
        self.pos_y_ts = pose.y
        self.ang_ts = velobst.calc_ang_n_to_e(math.degrees(pose.theta))
        self.speed_ts = pose.linear_velocity
        # self.get_logger().info("Winkel TS " + str(self.ang_ts))
        self.ts_pos.append((pose.x,pose.y))      
        msg = Twist()
        msg.linear.x = 0.75
        msg.angular.z = 0.0
        self.cmd_vel_pub_sec_.publish(msg)
        

    def pose_callback_thi(self, pose: Pose):
        self.pos_x_ts_1 = pose.x
        self.pos_y_ts_1 = pose.y
        self.ang_ts_1 = velobst.calc_ang_n_to_e(math.degrees(pose.theta))
        self.speed_ts_1 = pose.linear_velocity
        # self.get_logger().info("Winkel TS " + str(self.ang_ts_1))
        self.ts1_pos.append((pose.x,pose.y))      
        msg = Twist()
        msg.linear.x = 0.6
        msg.angular.z = 0.0
        self.cmd_vel_pub_thi_.publish(msg)

    def pose_callback(self, pose: Pose):
        self.pos_x_os = pose.x
        self.pos_y_os = pose.y
        self.ang_os = velobst.calc_ang_n_to_e(math.degrees(pose.theta))
        self.speed_os = pose.linear_velocity
        # self.get_logger().info("Vel OS " + str(self.speed_os) + " " + str(self.ang_os))
        self.os_pos.append((pose.x,pose.y))
        self.pos_x_ts_rel = self.pos_x_ts - self.pos_x_os
        self.pos_y_ts_rel = self.pos_y_ts - self.pos_y_os
        self.pos_rel = (self.pos_x_ts_rel, self.pos_y_ts_rel)
        self.pos_ts_rel = np.array(self.pos_rel)

        self.pos_x_ts_rel_1 = self.pos_x_ts_1 - self.pos_x_os
        self.pos_y_ts_rel_1 = self.pos_y_ts_1 - self.pos_y_os
        self.pos_rel_1 = (self.pos_x_ts_rel_1, self.pos_y_ts_rel_1)
        self.pos_ts_rel_1 = np.array(self.pos_rel_1)

        self.pos_TP = np.array([11.0, 11.0])
        pos_TP_rel = (self.pos_TP[0]-self.pos_x_os, self.pos_TP[1]-self.pos_y_os)
        ang_TP = math.degrees(np.arctan2(pos_TP_rel[1]+2, pos_TP_rel[0]+2))
        ang_TP = (ang_TP+360) % 360
        ang_TP = velobst.calc_ang_n_to_e(ang_TP)
        vel_OS = np.array([self.speed_os, self.ang_os])
        
        vel_OSxy = velobst.vect_to_xy(vel_OS)
        ang_OS_rad = np.deg2rad(self.ang_os)
        vel_des = np.array([0.5, ang_TP])
        self.TS = np.array([[self.pos_ts_rel,self.len_ts,self.wid_ts, self.speed_ts, self.ang_ts]],dtype=object)
        self.TS_1 = np.array([[self.pos_ts_rel_1,self.len_ts_1,self.wid_ts_1, self.speed_ts_1, self.ang_ts_1]],dtype=object)
        self.TS_all = np.vstack((self.TS, self.TS_1))
        self.OS = np.array([vel_OS, vel_OSxy,ang_OS_rad,vel_des], dtype=object)
        
        self.new_vel = velobst.calc_vel_final(self.TS, self.OS, False, np.array([0,0]))
        self.thetime = round(((perf_counter_ns()-self.start_time)/1000000000), 3)
        print("New vel: ", self.new_vel, "Time: ",(perf_counter_ns()-self.start_time)/1000000)
        if velobst.check_between_angles(self.new_vel[1], vel_OS[1], (vel_OS[1]+180) % 360):
            rot = True # Clockwise
        else:
            rot = False # Counter-Clockwise
        if rot and velobst.ang_betw_vect(self.new_vel, vel_OS) > 5:
            cmd = Twist()
            cmd.linear.x = self.new_vel[0]
            cmd.angular.z = -0.75
        elif not rot and velobst.ang_betw_vect(self.new_vel, vel_OS) > 5:
            cmd = Twist()
            cmd.linear.x = self.new_vel[0]
            cmd.angular.z = 0.75
        else:
            cmd = Twist()
            cmd.linear.x = vel_des[0]
            cmd.angular.z = 0.0
        self.cmd_vel_pub_.publish(cmd)

    def call_set_pen_serv(self, r, g, b, width, off):
        client = self.create_client(SetPen, "/turtle2/set_pen")
        while not client.wait_for_service(1.0):
                    self.get_logger().warn("Waiting for service...")
        
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))

    def call_set_pen_serv_2(self, r, g, b, width, off):
        client = self.create_client(SetPen, "/turtle3/set_pen")
        while not client.wait_for_service(1.0):
                    self.get_logger().warn("Waiting for service...")
        
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen_2))
    
    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service called failed: %r" % (e,))

    def callback_set_pen_2(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service called failed: %r" % (e,))  
    
    def spawn_turtle(self, x, y, theta, name):
                client = self.create_client(Spawn, "/spawn")
                while not client.wait_for_service(1.0):
                    self.get_logger().warn("Waiting for service...")

                request = Spawn.Request()
                request.x = x
                request.y = y
                request.theta = theta
                request.name = name
                    
                future = client.call_async(request)
                future.add_done_callback(partial(self.callback_spawn_turtle))

    def callback_spawn_turtle(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service called failed: %r" % (e,))

    def clear_path(self):
            client = self.create_client(Empty, "/clear")
            while not client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for service...")

            request = Empty.Request()
                
            future = client.call_async(request)
            future.add_done_callback(partial(self.callback_clear_path))

    def callback_clear_path(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service called failed: %r" % (e,))

    def set_pos_sec(self, x, y, theta):
        client = self.create_client(TeleportAbsolute, "/turtle2/teleport_absolute")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")

        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
      
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pos_sec))

    def callback_set_pos_sec(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service called failed: %r" % (e,))

    def set_pos_thi(self, x, y, theta):
        client = self.create_client(TeleportAbsolute, "/turtle3/teleport_absolute")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")

        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
      
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pos_thi))

    def callback_set_pos_thi(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service called failed: %r" % (e,))

    def set_pos(self, x, y, theta):
        client = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")

        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
      
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pos))

    def callback_set_pos(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service called failed: %r" % (e,))

    def exit_handler(self):
        os_position = np.array(self.os_pos)
        ts_position = np.array(self.ts_pos)
        #ts1_position = np.array(self.ts1_pos)
        new_vel_xy = velobst.vect_to_xy(self.new_vel) 
          
        plt.plot(os_position[:,0], os_position[:,1], c="teal",zorder=0.5, linestyle="--", label="OS path")
        plt.plot(ts_position[:,0], ts_position[:,1], c="red",zorder=0.05, linestyle="-.", label="TS 1 path")
        #plt.plot(ts1_position[:,0], ts1_position[:,1], c="red",zorder=0.05, linestyle=":", label="TS 1 path")
        plt.plot((0,self.pos_TP[0]+5),(0,self.pos_TP[1]+5),c="gray",zorder=0.02, alpha=1.0, label="global path")
        plt.scatter(13.0,13.0,c="dimgray", marker="+", label="OS goal")
        plt.quiver(os_position[-1,0], os_position[-1,1], new_vel_xy[0], new_vel_xy[1],scale=1,
                    scale_units='xy', angles='xy', color='blue', zorder=6,width=0.005, hatch="----", edgecolor="black", linewidth=0.5, label="v\u20D7 new")
        
        velobst.calc_vel_final(self.TS, self.OS, True, self.os_pos[-1])
        plt.axis('scaled')
        plt.axis([-0.5,14,-0.5,14])
        plt.title("time = " + str(self.thetime) + " s")
        plt.xlabel('x-coordinate [m]')
        plt.ylabel('y-coordinate [m]')
        # plt.legend(["OS path", "TS 1 path", "TS 2 path", "desired path", "target position", "new vel", "desired vel","OS","safety domain","TS"])
        
        #get handles and labels
        handles, labels = plt.gca().get_legend_handles_labels()

        #specify order of items in legend
        # order = [9,7,12,8,0,1,2,3,4,10,5,6,11]
        order = [8,6,7,0,1,2,3,9,4,5,10]

        #add legend to plot
        #plt.legend([handles[idx] for idx in order],[labels[idx] for idx in order]) 
        
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
