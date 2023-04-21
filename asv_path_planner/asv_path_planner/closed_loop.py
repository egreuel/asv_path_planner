#!/usr/bin/env python3
from asv_path_planner import velobst
from .velobst_class import VO
import atexit
import numpy as np
import math
import rclpy
import csv
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
from scipy.interpolate import splrep, BSpline

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
        self.elapsed_time = []
        self.delta_ang = []
        self.speed_com = []
        self.ang_com = []
        self.start_time = perf_counter_ns()
        self.len_os = 3.0
        self.wid_os = 1.75
        self.len_ts_1 = 3.5
        self.wid_ts_1 = 6.0
        self.len_ts_2 = 1.75
        self.wid_ts_2 = 3.0
        self.last_gps = None
        self.last_gps_time = None
        self.last_gps_1 = None
        self.last_gps_time_1 = None
        self.last_gps_2 = None
        self.last_gps_time_2 = None
        self.os_max_speed = 6.0
        self.ref_point = [45.001799636812144, 15.002536642856318]
        # PID controll parameters
        # ku = 0.6
        # kp = 0.36
        # Tu = 1.6 s
        # ki = 0.45
        # kd = 0.072
        self.kp = 0.36
        self.ki = 0.15
        self.kd = 0.072
        self.error = 0
        self.der_error = 0
        self.int_error = 0
        self.last_error = 0
        self.output = 0
        self.error_1 = 0
        self.der_error_1 = 0
        self.int_error_1 = 0
        self.last_error_1 = 0
        self.output_1 = 0

        self.test_vo = VO(3.0, 1.75, 6.0, 15, 7.5, 5, 0.5, 5, 0.25, 3)

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

    def angle_diff(self, first_angle, second_angle):
        diff = second_angle - first_angle
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        return diff

    def compute_pid(self, setpoint, vel, time_step):
        self.error = setpoint - vel  
        self.int_error += self.error * time_step
        self.der_error = (self.error - self.last_error) / time_step
        self.last_error = self.error
        self.output = self.kp*self.error + self.ki*self.int_error + self.kd*self.der_error
        if self.output >= 1:
            self.output = 1.0
        elif self.output <= 0:
            self.output = 0.0
        return self.output
       
    def compute_pid_1(self, setpoint, vel, time_step):
        self.error_1 = setpoint - vel  
        self.int_error_1 += self.error_1 * time_step
        self.der_error_1 = (self.error_1 - self.last_error_1) / time_step
        self.last_error_1 = self.error_1
        self.output_1 = self.kp*self.error_1 + self.ki*self.int_error_1 + self.kd*self.der_error_1
        if self.output_1 >= 1:
            self.output_1 = 1.0
        elif self.output_1 <= 0:
            self.output_1 = 0.0
        return self.output_1
    
    def gps_callback_tp(self, pose: NavSatFix):
        gps_lat = pose.latitude
        gps_lon = pose.longitude
        # self.gps_tp = np.array([gps_lat, gps_lon])

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

        # self.ts_pos_1.append(self.gps_1)

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

        thrust = self.compute_pid_1(9.0, self.speed_gps_1, 0.020)
        msg = Float32MultiArray()
        msg.data = [thrust, thrust, 0.0]
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

        # self.ts_pos_2.append(self.gps_2)

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

        # self.os_pos.append(self.gps)

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
        self.pos_ts_rel_1 = self.test_vo.calc_coord_gps_to_xy(self.gps, self.gps_1)
        # self.pos_ts_rel_2 = velobst.calc_coord_gps_to_xy(self.gps, self.gps_2)

        # Calculate relative position of all vesselt to the reference point
        self.ref_os = self.test_vo.calc_coord_gps_to_xy(self.ref_point, self.gps)
        self.ref_ts_1 = self.test_vo.calc_coord_gps_to_xy(self.ref_point, self.gps_1)
        # self.ref_ts_2 = self.test_vo.calc_coord_gps_to_xy(self.ref_point, self.gps_2)
        self.os_pos.append(self.ref_os)
        self.ts_pos_1.append(self.ref_ts_1)
        # self.ts_pos_2.append(self.ref_ts_2)

        self.thetime = round(((perf_counter_ns()-self.start_time)/1000000000), 3)
        self.simu_time.append(self.thetime)

        # Calculate distance between OS and TS and save it in an list
        self.dist_os_ts_1.append(distance.great_circle(self.gps, self.gps_1).meters) 
        # self.dist_os_ts_2.append(distance.great_circle(self.gps, self.gps_2))
        
        # Save speed and orientation of the OS in a list
        self.os_speed.append(self.speed_gps)
        self.os_ang.append(self.ang_gps)  
        

        # Calculate the relative position of the target point to OS
        self.gps_tp = np.array([45.00179960159882, 15.003804964281368])
        pos_TP_rel = self.test_vo.calc_coord_gps_to_xy(self.gps, self.gps_tp)
        ang_TP = math.degrees(np.arctan2(pos_TP_rel[1]+2, pos_TP_rel[0]+2))
        ang_TP = (ang_TP+360) % 360
        ang_TP = self.test_vo.calc_ang_n_to_e(ang_TP)

        vel_OS = np.array([self.speed_gps, self.ang_gps])
        vel_OSxy = self.test_vo.vect_to_xy(vel_OS)
        ang_OS_rad = np.deg2rad(self.ang_gps)
        vel_des = np.array([3.0, ang_TP])
        ang_diff = (self.ang_gps - 0 + 180 + 360) % 360 - 180
        self.delta_ang.append(ang_diff)
        self.TS_1 = np.array([[self.pos_ts_rel_1,self.len_ts_1,self.wid_ts_1, self.speed_gps_1, self.ang_gps_1]],dtype=object)
        # self.TS_2 = np.array([[self.pos_ts_rel_2,self.len_ts_2,self.wid_ts_2, self.speed_gps_2, self.ang_gps_2]],dtype=object)
        # self.TS_all = np.vstack((self.TS_1, self.TS_2))
        self.OS = np.array([vel_OS, vel_OSxy,ang_OS_rad,vel_des], dtype=object)
       
        if self.thetime < 1.5:
            self.new_vel = vel_des
            thrust = self.compute_pid(self.new_vel[0], self.speed_gps, 0.020)
            msg = Float32MultiArray()
            msg.data = [thrust, thrust, 0.0]
        else:
            if distance.great_circle(self.gps, self.gps_1).meters < 200:
                starting_time = perf_counter_ns()
                self.new_vel = self.test_vo.calc_vel_final(self.TS_1, self.OS, False, np.array([0,0]))
                self.elapsed_time.append((perf_counter_ns()-starting_time)/1000000)
            else:
                self.elapsed_time.append(0)
                self.new_vel = vel_des
            print("New vel: ", self.new_vel, "OS vel:", vel_OS, "Time: ",(perf_counter_ns()-self.start_time)/1000000)
                    
            if self.angle_diff(vel_OS[1], self.new_vel[1]) > 5:
                thrust = self.compute_pid(self.new_vel[0], self.speed_gps, 0.020)   
                rot = (self.angle_diff(vel_OS[1], self.new_vel[1])/90)*1
                msg = Float32MultiArray()
                msg.data = [thrust+rot, thrust-rot, 0.0]
                # msg = Float32MultiArray()
                # msg.data = [self.new_vel[0]*1.5/self.os_max_speed,self.new_vel[0]/(self.os_max_speed*2), 0.0]
            elif self.angle_diff(vel_OS[1], self.new_vel[1]) < 5:
                thrust = self.compute_pid(self.new_vel[0], self.speed_gps, 0.020)
                rot = (self.angle_diff(vel_OS[1], self.new_vel[1])/90)*1
                msg = Float32MultiArray()
                msg.data = [thrust+rot, thrust-rot, 0.0]
                # msg = Float32MultiArray()
                # msg.data = [self.new_vel[0]/(self.os_max_speed*2), self.new_vel[0]*1.5/self.os_max_speed, 0.0]
            else:
                thrust = self.compute_pid(self.new_vel[0], self.speed_gps, 0.020)
                msg = Float32MultiArray()
                msg.data = [thrust, thrust, 0.0]

                 
        self.thruster_pub_os.publish(msg)
        self.speed_com.append(self.new_vel[0])
        self.ang_com.append(self.new_vel[1])
    
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

        fields = ["Sim Time", "Distance", "Speed Com", "Angle Com", "Speed OS", "Angle OS", "Delta Angle", "Run Time", "OS pos", "TS pos"]
        rows = [simu_time, dist_os_ts_1, self.speed_com, self.ang_com, os_speed, os_ang, self.delta_ang, self.elapsed_time, os_position, ts_position]
        filename = "simulation_results_.csv"
        # writing to csv file  
        with open(filename, 'w') as csvfile:  
            # creating a csv writer object  
            csvwriter = csv.writer(csvfile)  
                
            # writing the fields  
            csvwriter.writerow(fields)  
                
            # writing the data rows  
            csvwriter.writerows(rows)
             
        min_length = min(len(simu_time), len(dist_os_ts_1), len(self.speed_com), len(self.ang_com), len(os_speed), len(os_ang), len(self.delta_ang), len(self.elapsed_time))
        
        simu_time = simu_time[15:min_length]
        dist_os_ts_1 = dist_os_ts_1[15:min_length]
        self.speed_com = self.speed_com[15:min_length]
        self.ang_com = self.ang_com[15:min_length]
        os_speed = os_speed[15:min_length]
        os_ang = os_ang[15:min_length]
        self.delta_ang = self.delta_ang[15:min_length]
        self.elapsed_time = self.elapsed_time[15:min_length]

        fig1 = plt.figure()
        ax1 = fig1.add_subplot(111)
        plt.plot(simu_time, dist_os_ts_1, c="blue", label="Distance between OS and TS", linewidth=2.5)
        min_dist = min(dist_os_ts_1)
        min_dist = round(min_dist, 2)
        ind_min_dist = dist_os_ts_1.argmin()
        time_min_dist = simu_time[ind_min_dist]
        plt.scatter(time_min_dist, min_dist, marker="x", c="orange", zorder=2, label=f"min. distance = {min_dist} m", linewidth=2.5)
        # plt.title("Distance betweenn OS and TS")
        plt.xlabel("Time [s]")
        plt.ylabel("Distance [m]")
        plt.legend(loc="upper right", fontsize="10")
        ax1.set_aspect(1.0/ax1.get_data_ratio(), adjustable='box')
        
        
        fig2 = plt.figure()
        ax2 = fig2.add_subplot(111)
        plt.plot(simu_time, os_speed, c="green", label="Current speed (OS)", linewidth=2.5)
        plt.plot(simu_time, self.speed_com, c="red", label="Desired speed (OS)", linestyle="dashed", linewidth=2.5)
        # plt.title("Speed OS")
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [m/s]")
        plt.legend(loc="lower right", fontsize="10")
        ax2.set_aspect(1.0/ax2.get_data_ratio(), adjustable='box')
        
        fig3 = plt.figure()
        ax3 = fig3.add_subplot(111)
        plt.plot(simu_time, os_ang, c="teal", label="Current orientation (OS)", linewidth=2.5)
        plt.plot(simu_time, self.ang_com, c="red", label="Desired orientation (OS)", linestyle="dashed", linewidth=2.5)
        # plt.title("Orientation OS")
        plt.xlabel("Time [s]")
        plt.ylabel("Oritentation [°]")
        plt.legend(loc="lower left", fontsize="10")
        ax3.set_aspect(1.0/ax3.get_data_ratio(), adjustable='box')
        
        # Algorithm run time
        # fig4 = plt.figure()
        # ax4 = fig4.add_subplot(111)
        # plt.plot(simu_time, self.elapsed_time, c="red", label="Run time")
        # # plt.title("Algorithm run time")
        # plt.xlabel("Time [s]")
        # plt.ylabel("Run time [ms]")
        # # plt.legend(loc="upper left", fontsize="10")
        # ax4.set_aspect(1.0/ax4.get_data_ratio(), adjustable='box')
        
        
        
        ### For Subplots
        # fig, axs = plt.subplots(2, 2, constrained_layout = True)
        # fig.suptitle("Results Head-on encounter with one TS", fontsize=15)
        # axs[0, 0].plot(simu_time, dist_os_ts_1, c="blue", label="Distance between OS and TS")
        # axs[0, 0].set_title('Distance OS_TS')
        # axs[0, 0].set(xlabel="Time [s]", ylabel="Distance [m]")
        # min_dist = min(dist_os_ts_1)
        # min_dist = round(min_dist, 2)
        # ind_min_dist = dist_os_ts_1.argmin()
        # time_min_dist = simu_time[ind_min_dist]
        # axs[0, 0].scatter(time_min_dist, min_dist, marker="x", c="orange", zorder=2, label=f"min. distance {min_dist} m")
        # axs[0, 0].legend(loc="lower left", fontsize="7")
        # axs[0, 0].set_box_aspect(1)

        # axs[0, 1].plot(simu_time, os_speed, c="green", label="Current speed (OS)")
        # axs[0, 1].plot(simu_time, self.speed_com, c="red", label="Speed input command", linestyle="dashed")
        # axs[0, 1].set_title('Speed OS')
        # axs[0, 1].set(xlabel="Time [s]", ylabel="Speed [m/s]")
        # axs[0, 1].legend(loc="lower right", fontsize="7")
        # axs[0, 1].set_box_aspect(1)

        # axs[1, 0].plot(simu_time, os_ang, c="teal", label="Current orientation (OS)")
        # axs[1, 0].plot(simu_time, self.ang_com, c="red", label="Orientation input command", linestyle="dashed")
        # axs[1, 0].set_title('Orientation OS')
        # axs[1, 0].set(xlabel="Time [s]", ylabel="Orientation [°]")
        # axs[1, 0].legend(loc="upper left", fontsize="7")
        # axs[1, 0].set_box_aspect(1)
        
        # axs[1, 1].plot(simu_time, self.elapsed_time, c="red", label="Run time")
        # axs[1, 1].set_title('Algorithm run time')
        # axs[1, 1].set(xlabel="Time [s]", ylabel="Run time [ms]")
        # # axs[1, 1].legend(loc="upper left", fontsize="10")
        # axs[1, 1].set_box_aspect(1)
        # # fig.tight_layout()
        ###

        fig5 = plt.figure()
        ref_tp = self.test_vo.calc_coord_gps_to_xy(self.ref_point, self.gps_tp)
        plt.plot(os_position[:,0], os_position[:,1], c="teal",zorder=0.5, linestyle="--", label="OS path", linewidth=2.5)
        plt.plot(ts_position[:,0], ts_position[:,1], c="red",zorder=0.05, linestyle="-.", label="TS 1 path", linewidth=2.5)
        #plt.plot(ts1_position[:,0], ts1_position[:,1], c="red",zorder=0.05, linestyle=":", label="TS 1 path")
        plt.plot((os_position[0,0],ref_tp[0]),(os_position[0,1],ref_tp[1]),c="gray",zorder=0.02, alpha=1.0, label="global path", linewidth=1.5)
        plt.scatter(ref_tp[0],ref_tp[1],c="dimgray", marker="+", label="OS goal", linewidth=2.5)
        plt.quiver(os_position[-1,0], os_position[-1,1], new_vel_xy[0]*5, new_vel_xy[1],scale=1,
                    scale_units='xy', angles='xy', color='blue', zorder=6,width=0.01, hatch="----", edgecolor="black", linewidth=0.5, label="v\u20D7 new")
        plt.legend(loc="upper left", fontsize="10")
        
        # velobst.calc_vel_final(self.TS_1, self.OS, True, self.os_pos[-1])
        plt.axis('square')
        bot_left = self.test_vo.calc_coord_gps_to_xy(self.ref_point, np.array([45.000899825520364, 15.00126830157632]))
        bot_right = self.test_vo.calc_coord_gps_to_xy(self.ref_point, np.array([45.000899769180805, 15.003804904724001]))
        top_left = self.test_vo.calc_coord_gps_to_xy(self.ref_point, np.array([45.002699490216614, 15.001268341281854]))
        top_right = self.test_vo.calc_coord_gps_to_xy(self.ref_point, np.array([45.002699433873545, 15.003805023840597]))
        plt.axis([bot_left[0],bot_right[0]+5,bot_left[1]-2.5,top_left[1]+2.5])
        

        # plt.title("time = " + str(self.thetime) + " s")
        # plt.title('Vehicle trajectories')
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        # # plt.legend(["OS path", "TS 1 path", "TS 2 path", "desired path", "target position", "new vel", "desired vel","OS","safety domain","TS"])
        
        # #get handles and labels
        # handles, labels = plt.gca().get_legend_handles_labels()

        # #specify order of items in legend
        # # order = [9,7,12,8,0,1,2,3,4,10,5,6,11]
        # order = [8,6,7,0,1,2,3,9,4,5,10]

        # add legend to plot
        # plt.legend([handles[idx] for idx in order],[labels[idx] for idx in order]) 
        # current_values_x = plt.gca().get_xticks()
        # current_values_y = plt.gca().get_yticks()
        # plt.gca().set_xticklabels(['{:.4f}'.format(x) for x in current_values_x])
        # plt.gca().set_yticklabels(['{:.4f}'.format(x) for x in current_values_y])

        if self.test_vo.coll_safety:
            fig6 = plt.figure()
            

        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
