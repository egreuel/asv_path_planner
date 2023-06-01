"""
This is the ROS script to use the MARUS simulator and test the collision avoidance algorithm including some simple control algorithms for the OS in the simulation

"""

#!/usr/bin/env python3
import atexit
import numpy as np
import math
import rclpy
import csv
from .velobst_class import VO
from .ts_class import TS
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray
from time import perf_counter_ns
from matplotlib import pyplot as plt 
from geopy import distance

class RosScriptNode(Node):
    """
    This class defines the ROS node which is used to connect to the MARUS simulator
    """

    def __init__(self):
        super().__init__("ros_script")
        atexit.register(self.exit_handler)
        
        self.declare_parameter('vel_ts_1', 0.0)
        self.declare_parameter('vel_ts_2', 0.0)
        self.declare_parameter('vel_ts_3', 0.0)
        self.declare_parameter('os_max_speed', 0.0)
        self.declare_parameter('os_des_speed', 0.0)
        self.declare_parameter('detec_range', 50)
        self.declare_parameter('length_os', 3.0) 
        self.declare_parameter('width_os', 1.5)
        self.declare_parameter('length_ts_1', 6.0)
        self.declare_parameter('width_ts_1', 3.0)
        self.declare_parameter('length_ts_2', 6.0)
        self.declare_parameter('width_ts_2', 3.0)
        self.declare_parameter('length_ts_3', 6.0)
        self.declare_parameter('width_ts_3', 3.0)
        self.declare_parameter('vel_kp', 0.36)
        self.declare_parameter('vel_ki', 0.15)
        self.declare_parameter('vel_kd', 0.072)
        self.declare_parameter('head_kp', 0.014)
        self.declare_parameter('head_kd', 0.0082)
       
        self.vel_ts_1 = self.get_parameter('vel_ts_1').value # input for simulation to move the TS (slow: 1.5, fast: 3.0; Overtaking: slow: 0.3, fast: 0.75, Left crossing: slow: 2.0, fast: 2.0, Right crossing: slow: 2.0, fast: 3.0)
        self.vel_ts_2 = self.get_parameter('vel_ts_2').value # input for simulation to move the TS (slow: 3.0, fast: 6.0)
        self.vel_ts_3 = self.get_parameter('vel_ts_3').value
        self.os_max_speed = self.get_parameter('os_max_speed').value
        self.os_des_speed = self.get_parameter('os_des_speed').value
        self.detec_range = self.get_parameter('detec_range').value
        self.os = TS()
        self.os.length = self.get_parameter('length_os').value
        self.os.width = self.get_parameter('width_os').value
        self.ts_1 = TS()
        self.ts_1.length = self.get_parameter('length_ts_1').value
        self.ts_1.width = self.get_parameter('width_ts_1').value
        self.ts_2 = TS()
        self.ts_2.length = self.get_parameter('length_ts_2').value
        self.ts_2.width = self.get_parameter('width_ts_2').value
        self.ts_3 = TS()
        self.ts_3.length = self.get_parameter('length_ts_3').value
        self.ts_3.width = self.get_parameter('width_ts_3').value
        # PID speed control parameters (found with Ziegler-Nichols methode)
        self.kp = self.get_parameter('vel_kp').value
        self.ki = self.get_parameter('vel_ki').value
        self.kd = self.get_parameter('vel_kd').value
        # PD heading control parameters (found with Ziegler-Nichols methode)
        self.kp_ang = self.get_parameter('head_kp').value
        self.kd_ang = self.get_parameter('head_kd').value

        
        # self.detec_range = 50 # Range in which obstacles can be detected around the OS  

        self.os_pos = []
        self.os_speed = []
        self.os_ang = []
        self.ts_pos_1 = []
        self.ts_pos_2 = []
        self.ts_pos_3 = []
        self.dist_os_ts_1 = []
        self.dist_os_ts_2 = []
        self.dist_os_ts_3 = []
        self.simu_time = []
        self.elapsed_time = []
        self.delta_ang = []
        self.speed_com = []
        self.ang_com = []
        self.coll_check = []
        self.start_time = perf_counter_ns()

        self.ref_point = [45.001799636812144, 15.002536642856318] # Just for plotting
        self.last_gps = None
        self.last_gps_time = None
        self.last_gps_1 = None
        self.last_gps_time_1 = None
        self.last_gps_2 = None
        self.last_gps_time_2 = None
        self.last_gps_3 = None
        self.last_gps_time_3 = None
        self.flag = False
        self.wait_bool_ts_1 = False # Start the algorithm once position of the TSs are recieved
        self.wait_bool_ts_2 = False # Start the algorithm once position of the TSs are recieved
        self.wait_bool_ts_3 = False
        self.wait_bool_os = False
        self.wait_bool_tp = False # Start the algorithm once position of the TSs are recieved
       
        # Variables for the PID and PD control     
        self.last_error = 0
        self.int_error = 0
        self.last_error_ang = 0
        self.last_error_1 = 0
        self.int_error_1 = 0
        self.last_error_2 = 0
        self.int_error_2 = 0
        self.last_error_3 = 0
        self.int_error_3 = 0

        # VO(OS length, OS width, OS max speed, max TTC, threshhold, safety factor, speed unc, angle unc, speed res, angle res)
        self.vo = VO(self.os.length, self.os.width, self.os_max_speed, 15, 7.5, 6, 0.5, 5, 0.25, 3) # Initalize the VO algorithm

        # Define publisher and subscriber
        # Subscriber for the target/destination the OS should go to
        self.gps_sub_tp = self.create_subscription(
            NavSatFix, "/target_point", self.gps_callback_tp, 10)
        # Subscriber and publsher for target ship 1 (GPS coordinates, Thruster output)
        self.gps_sub_ts_1 = self.create_subscription(
            NavSatFix, "/marus_boat_1/gps", self.gps_callback_ts_1, 10)
        self.thruster_pub_ts_1 = self.create_publisher(
            Float32MultiArray, "/marus_boat_1/pwm_out", 10)
        # Subscriber and publsher for target ship 2 (GPS coordinates, Thruster output)
        self.gps_sub_ts_2 = self.create_subscription(
            NavSatFix, "/marus_boat_2/gps", self.gps_callback_ts_2, 10)
        self.thruster_pub_ts_2 = self.create_publisher(
            Float32MultiArray, "/marus_boat_2/pwm_out", 10)
        # Subscriber and publsher for target ship 3 (GPS coordinates, Thruster output)
        self.gps_sub_ts_3 = self.create_subscription(
            NavSatFix, "/marus_boat_3/gps", self.gps_callback_ts_3, 10)
        self.thruster_pub_ts_3 = self.create_publisher(
            Float32MultiArray, "/marus_boat_3/pwm_out", 10)
        
        # Subscriber and publsher for owm ship (GPS coordinates, Thruster output)
        self.gps_sub_os = self.create_subscription(
            NavSatFix, "/marus_boat/gps", self.gps_callback_os, 10)
        self.thruster_pub_os = self.create_publisher(
            Float32MultiArray, "/marus_boat/pwm_out", 10)

        # Callback function for parameter callback
        self.add_on_set_parameters_callback(self.param_callback)

        self.get_logger().info("Node started!")
    
    def param_callback(self, params: list[Parameter]):
        for param in params:
            if param.name == "vel_ts_1":
                self.vel_ts_1 = param.value
            if param.name == "vel_ts_2":
                self.vel_ts_2 = param.value
            if param.name == "vel_ts_3":
                self.vel_ts_3 = param.value
            if param.name == "os_max_speed":
                self.os_max_speed = param.value
            if param.name == "os_des_speed":
                self.os_des_speed = param.value
            if param.name == "detec_range":
                self.detec_range = param.value
            if param.name == "length_os":
                self.os.length = param.value
            if param.name == "width_os":
                self.os.width = param.value
            if param.name == "length_ts_1":
                self.ts_1.length = param.value
            if param.name == "width_ts_1":
                self.ts_1.width = param.value
            if param.name == "length_ts_2":
                self.ts_2.length = param.value
            if param.name == "width_ts_2":
                self.ts_2.width = param.value
            if param.name == "length_ts_3":
                self.ts_3.length = param.value
            if param.name == "width_ts_3":
                self.ts_3.width = param.value
            if param.name == "vel_kp":
                self.kp = param.value
            if param.name == "vel_ki":
                self.ki = param.value
            if param.name == "vel_kd":
                self.kd = param.value
            if param.name == "head_kp":
                self.kp_ang = param.value
            if param.name == "head_kd":
                self.kd_ang = param.value
        return SetParametersResult(successful=True)

    # Calculate the course difference between two courses (new velocity and current velocity)
    def angle_diff(self, ref_angle, angle):
        """Function that calculates the difference of an orientation angle in respect to another from -180 to 180°

        Args:
            ref_angle (float[°]): Reference angle to which the difference is calculated
            angle (float[°]): Deviating angle from the reference angle

        Returns:
            float: Angle difference in a range from -180 to 180°
        """
        diff = angle - ref_angle
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        return diff

    # PID speed controller for thruster output
    def compute_pid(self, setpoint, vel, time_step):
        """PID controller to maintain a given speed for OS

        Args:
            setpoint (float [m/s]): Desired speed
            vel (float [m/s]): Current speed
            time_step (float [m/s]): Time step between measurments

        Returns:
            float: Thruster input value from 0 to 1
        """
        error = setpoint - vel  
        self.int_error += error * time_step
        der_error = (error - self.last_error) / time_step
        self.last_error = error
        output = self.kp*error + self.ki*self.int_error + self.kd*der_error
        if output >= 1:
            output = 1.0
        elif output <= 0:
            output = 0.0
        return output

    # PID speed controller for thruster output   
    def compute_pid_1(self, setpoint, vel, time_step):
        """PID controller to maintain a given speed for TS

        Args:
            setpoint (float [m/s]): Desired speed
            vel (float [m/s]): Current speed
            time_step (float [m/s]): Time step between measurments

        Returns:
            float: Thruster input value from 0 to 1
        """
        error_1 = setpoint - vel  
        self.int_error_1 += error_1 * time_step
        der_error_1 = (error_1 - self.last_error_1) / time_step
        self.last_error_1 = error_1
        output_1 = self.kp*error_1 + self.ki*self.int_error_1 + self.kd*der_error_1
        if output_1 >= 1:
            output_1 = 1.0
        elif output_1 <= 0:
            output_1 = 0.0
        return output_1
    
     # PID speed controller for thruster output   
    def compute_pid_2(self, setpoint, vel, time_step):
        """PID controller to maintain a given speed for TS

        Args:
            setpoint (float [m/s]): Desired speed
            vel (float [m/s]): Current speed
            time_step (float [m/s]): Time step between measurments

        Returns:
            float: Thruster input value from 0 to 1
        """
        error_2 = setpoint - vel  
        self.int_error_2 += error_2 * time_step
        der_error_2 = (error_2 - self.last_error_2) / time_step
        self.last_error_2 = error_2
        output_2 = self.kp*error_2 + self.ki*self.int_error_2 + self.kd*der_error_2
        if output_2 >= 1:
            output_2 = 1.0
        elif output_2 <= 0:
            output_2 = 0.0
        return output_2
    
     # PID speed controller for thruster output   
    def compute_pid_3(self, setpoint, vel, time_step):
        """PID controller to maintain a given speed for TS

        Args:
            setpoint (float [m/s]): Desired speed
            vel (float [m/s]): Current speed
            time_step (float [m/s]): Time step between measurments

        Returns:
            float: Thruster input value from 0 to 1
        """
        error_3 = setpoint - vel  
        self.int_error_3 += error_3 * time_step
        der_error_3 = (error_3 - self.last_error_3) / time_step
        self.last_error_3 = error_3
        output_3 = self.kp*error_3 + self.ki*self.int_error_3 + self.kd*der_error_3
        if output_3 >= 1:
            output_3 = 1.0
        elif output_3 <= 0:
            output_3 = 0.0
        return output_3
    
    # PD heading control for thruster output
    def compute_pd(self, error, time_step):
        """PD controller for orientation control

        Args:
            setpoint (float [°]): Desired orientation
            vel (float [°]): Current Orientation
            time_step (float [m/s]): Time step between measurments

        Returns:
            float: Thruster input value from -0.5 to 0.5
        """
        error_ang = error
        der_error_ang = (error_ang - self.last_error_ang) / time_step
        self.last_error_ang = error_ang
        output_ang = self.kp_ang*error_ang + self.kd_ang*der_error_ang
        if output_ang >= 0.5:
            output_ang = 0.5
        elif output_ang <= -0.5:
            output_ang = -0.5
        return output_ang
    
    # Callback function to receive GPS coordinates of the target point/destination (TP)
    def gps_callback_tp(self, pose: NavSatFix):
        """Callback function to receive GPS data from the target point/destination inside the unity simulation

        Args:
            pose (NavSatFix): GPS coordinates
        """
        gps_lat = pose.latitude
        gps_lon = pose.longitude
        self.gps_tp = np.array([gps_lat, gps_lon])
        self.wait_bool_tp = True

    # Callback function to receive GPS coordinates of target ship 1 (TS 1) to calculate velocity and publish a velocity to the TS
    def gps_callback_ts_1(self, pose: NavSatFix):
        """Callback function to receive GPS data from the first target ship inside the unity simulation to calculate speed and
        orientation and also publishing thruster inputs

        Args:
            pose (NavSatFix): GPS coordinates
        """
        
        vel_ts_1 = self.vel_ts_1
        gps_time_sec = pose.header.stamp.sec
        gps_time_nanosec =  pose.header.stamp.nanosec
        gps_time = gps_time_sec + gps_time_nanosec*(10**-9)
        gps_lat = pose.latitude
        gps_lon = pose.longitude
        self.gps_1 = np.array([gps_lat, gps_lon])
        self.wait_bool_ts_1 = True
        if self.last_gps_1 is None:
            self.last_gps_1 = self.gps_1
        if self.last_gps_time_1 is None:
            self.last_gps_time_1 = gps_time

        # Time difference between two gps measurments
        gps_time_diff = gps_time - self.last_gps_time_1

        # Calculate the speed and orientation of the ship from the GPS coordinates        
        if gps_time_diff > 0.0:
            dist_gps = distance.great_circle(self.gps_1, self.last_gps_1)
            self.ts_1.speed = dist_gps.meters / gps_time_diff
            ang_x = math.cos(self.gps_1[0]) * math.sin(self.gps_1[1]-self.last_gps_1[1])
            ang_y = math.cos(self.last_gps_1[0]) * math.sin(self.gps_1[0]) - math.sin(self.last_gps_1[0]) * math.cos(self.gps_1[0]) * math.cos(self.gps_1[1]-self.last_gps_1[1])
            self.ang_gps_1 = np.rad2deg(np.arctan2(ang_x, ang_y))
            self.ts_1.ang = (self.ang_gps_1+360) % 360
        else:
            self.ts_1.speed = 0
            self.ts_1.ang = 0
            
        self.last_gps_time_1 = gps_time
        self.last_gps_1 = self.gps_1

        # Publish the values of the thrusters to move the ship
        if gps_time_diff > 0:
            thrust = self.compute_pid_1(vel_ts_1, self.ts_1.speed, gps_time_diff)
            msg = Float32MultiArray()
            msg.data = [thrust, thrust, 0.0]
            self.thruster_pub_ts_1.publish(msg)
            
    # Callback function to receive GPS coordinates of target ship 2 (TS 2) to calculate velocity and publish a velocity to the TS
    def gps_callback_ts_2(self, pose: NavSatFix):
        """Callback function to receive GPS data from the first target ship inside the unity simulation to calculate speed and
        orientation and also publishing thruster inputs

        Args:
            pose (NavSatFix): GPS coordinates
        """
        
        vel_ts_2 = self.vel_ts_2
        gps_time_sec = pose.header.stamp.sec
        gps_time_nanosec =  pose.header.stamp.nanosec
        gps_time = gps_time_sec + gps_time_nanosec*(10**-9)
        gps_lat = pose.latitude
        gps_lon = pose.longitude
        self.gps_2 = np.array([gps_lat, gps_lon])
        self.wait_bool_ts_2 = True
        if self.last_gps_2 is None:
            self.last_gps_2 = self.gps_2
        if self.last_gps_time_2 is None:
            self.last_gps_time_2 = gps_time

        # Time difference between two gps measurments
        gps_time_diff = gps_time - self.last_gps_time_2

        # Calculate the speed and orientation of the ship from the GPS coordinates    
        if gps_time_diff > 0.0:
            dist_gps = distance.great_circle(self.gps_2, self.last_gps_2)
            self.ts_2.speed = dist_gps.meters / gps_time_diff
            ang_x = math.cos(self.gps_2[0]) * math.sin(self.gps_2[1]-self.last_gps_2[1])
            ang_y = math.cos(self.last_gps_2[0]) * math.sin(self.gps_2[0]) - math.sin(self.last_gps_2[0]) * math.cos(self.gps_2[0]) * math.cos(self.gps_2[1]-self.last_gps_2[1])
            self.ang_gps_2 = np.rad2deg(np.arctan2(ang_x, ang_y))
            self.ts_2.ang = (self.ang_gps_2+360) % 360
        else:
            self.ts_2.speed = 0
            self.ts_2.ang = 0
            
        self.last_gps_time_2 = gps_time
        self.last_gps_2 = self.gps_2

       # Publish the values of the thrusters to move the ship
        if gps_time_diff > 0:
            thrust = self.compute_pid_2(vel_ts_2, self.ts_2.speed, gps_time_diff)
            msg = Float32MultiArray()
            msg.data = [thrust, thrust, 0.0]
            self.thruster_pub_ts_2.publish(msg)
            
               
    def gps_callback_ts_3(self, pose: NavSatFix):
        """Callback function to receive GPS data from the second target ship inside the unity simulation to calculate speed and
        orientation and also publishing thruster inputs

        Args:
            pose (NavSatFix): GPS coordinates
        """
       
        vel_ts_3 = self.vel_ts_3
        gps_time_sec = pose.header.stamp.sec
        gps_time_nanosec =  pose.header.stamp.nanosec
        gps_time = gps_time_sec + gps_time_nanosec*(10**-9)
        gps_lat = pose.latitude
        gps_lon = pose.longitude
        self.gps_3 = np.array([gps_lat, gps_lon])
        self.wait_bool_ts_3 = True
        if self.last_gps_3 is None:
            self.last_gps_3 = self.gps_3
        if self.last_gps_time_3 is None:
            self.last_gps_time_3 = gps_time

        # Time difference between two gps measurments
        gps_time_diff = gps_time - self.last_gps_time_3

        # Calculate the speed and orientation of the ship from the GPS coordinates    
        if gps_time_diff > 0.0:
            dist_gps = distance.great_circle(self.gps_3, self.last_gps_3)
            self.ts_3.speed = dist_gps.meters / gps_time_diff
            ang_x = math.cos(self.gps_3[0]) * math.sin(self.gps_3[1]-self.last_gps_3[1])
            ang_y = math.cos(self.last_gps_3[0]) * math.sin(self.gps_3[0]) - math.sin(self.last_gps_3[0]) * math.cos(self.gps_3[0]) * math.cos(self.gps_3[1]-self.last_gps_3[1])
            self.ang_gps_3 = np.rad2deg(np.arctan2(ang_x, ang_y))
            self.ts_3.ang = (self.ang_gps_3+360) % 360
        else:
            self.ts_3.speed = 0
            self.ts_3.ang = 0
            
        self.last_gps_time_3 = gps_time
        self.last_gps_3 = self.gps_3

        # Publish the values of the thrusters to move the ship
        if gps_time_diff > 0:
            thrust = self.compute_pid_3(vel_ts_3, self.ts_3.speed, gps_time_diff)
            msg = Float32MultiArray()
            msg.data = [thrust, thrust, 0.0]
            self.thruster_pub_ts_3.publish(msg)

    # Main callback function where the VO is calculated and OS control is done 
    def gps_callback_os(self, pose: NavSatFix):
        """Callback function to receive GPS data from the own ship inside the unity simulation to calculate speed and orientation and
        unsing the collision avoidance algorithm to find a collision free path through the simulation;it is also used to publish 
        commands to the thrusters of the OS

        Args:
            pose (NavSatFix): GPS coordinates
        # """
        
        gps_time_sec = pose.header.stamp.sec
        gps_time_nanosec =  pose.header.stamp.nanosec
        gps_time = gps_time_sec + gps_time_nanosec*(10**-9)
        gps_lat = pose.latitude
        gps_lon = pose.longitude
        self.gps = np.array([gps_lat, gps_lon])
        self.wait_bool_os = True
        if self.last_gps is None:
            self.last_gps = self.gps
        if self.last_gps_time is None:
            self.last_gps_time = gps_time

        # Calculate the OS course from the GPS data
        gps_time_diff = gps_time - self.last_gps_time

        # Calculate the speed and orientation of the ship from the GPS coordinates         
        if gps_time_diff > 0.0:
            dist_gps = distance.great_circle(self.gps, self.last_gps)
            self.os.speed = dist_gps.meters / gps_time_diff
            ang_x = math.cos(self.gps[0]) * math.sin(self.gps[1]-self.last_gps[1])
            ang_y = math.cos(self.last_gps[0]) * math.sin(self.gps[0]) - math.sin(self.last_gps[0]) * math.cos(self.gps[0]) * math.cos(self.gps[1]-self.last_gps[1])
            self.os.ang = np.rad2deg(np.arctan2(ang_x, ang_y))
            self.os.ang = (self.os.ang+360) % 360
        else:
            self.os.speed = 0
            self.os.ang = 0
            
        self.last_gps_time = gps_time
        self.last_gps = self.gps

        if self.wait_bool_ts_1 and self.wait_bool_ts_2 and self.wait_bool_ts_3 and self.wait_bool_tp: # Only start this callback, if every TS has received GPS data once
            
            # Calculate relative position of TS to OS
            self.ts_1.pos = self.vo.calc_coord_gps_to_xy(self.gps, self.gps_1)
            self.ts_2.pos = self.vo.calc_coord_gps_to_xy(self.gps, self.gps_2)
            self.ts_3.pos = self.vo.calc_coord_gps_to_xy(self.gps, self.gps_3)

            # Calculate relative position of all vessels to the reference point (for plotting)
            self.ref_os = self.vo.calc_coord_gps_to_xy(self.ref_point, self.gps)
            self.ref_ts_1 = self.vo.calc_coord_gps_to_xy(self.ref_point, self.gps_1)
            self.ref_ts_2 = self.vo.calc_coord_gps_to_xy(self.ref_point, self.gps_2)
            self.ref_ts_3 = self.vo.calc_coord_gps_to_xy(self.ref_point, self.gps_3)
            self.os_pos.append(self.ref_os)
            self.ts_pos_1.append(self.ref_ts_1)
            self.ts_pos_2.append(self.ref_ts_2)
            self.ts_pos_3.append(self.ref_ts_3)

            # Save the time for each iteration and store it in a list
            self.thetime = round(((perf_counter_ns()-self.start_time)/1000000000), 3)
            self.simu_time.append(self.thetime)

            # Calculate distance between OS and TS and store it in a list
            self.dist_os_ts_1.append(distance.great_circle(self.gps, self.gps_1).meters) 
            self.dist_os_ts_2.append(distance.great_circle(self.gps, self.gps_2).meters)
            self.dist_os_ts_3.append(distance.great_circle(self.gps, self.gps_3).meters)
            
            # Store speed and orientation of the OS in a list
            self.os_speed.append(self.os.speed)
            self.os_ang.append(self.os.ang)  
            
            # Calculate the relative position of the target point to OS 
            pos_TP_rel = self.vo.calc_coord_gps_to_xy(self.gps, self.gps_tp)
            # Calculate relative bearing of TP to OS for the desired velocity (Pure pursuit guidance)
            ang_TP = math.degrees(np.arctan2(pos_TP_rel[1]+2, pos_TP_rel[0]+2))
            ang_TP = (ang_TP+360) % 360
            ang_TP = self.vo.calc_ang_n_to_e(ang_TP)

            # OS velocity calculated from GPS data
            vel_OS = np.array([self.os.speed, self.os.ang])

            # Desired velocity with desired speed and angle to TP
            vel_des = np.array([self.os_des_speed, ang_TP])
            
            # Store the parameter that checks if OS is colliding with safety area around TS
            self.coll_check.append(self.vo.coll_safety)

            # Check wether any of the target ships is within a range of 50 meters and add them to TS_all
            all_ts = [self.ts_1, self.ts_2, self.ts_3]
            all_ts_gps = [self.gps_1, self.gps_2, self.gps_3]
            TS_all = []
            for ship, pos in zip(all_ts, all_ts_gps):
                if distance.great_circle(self.gps, pos).meters < self.detec_range:
                    TS_all.append(ship)
                else:
                    pass
            if TS_all == []:
                self.flag = False
            else:
                self.flag = True
            
            # Array of information of the OS needed by the VO algorithm
            info_OS = np.array([vel_OS, vel_des], dtype=object)

            # for the first 1.5 s do not calculate the VO because the calculated speed and angle is not stabil if the OS is not moving yet (Start up phase)
            if self.thetime < 1.5:
                self.new_vel = vel_des
                if gps_time_diff > 0:
                    thrust = self.compute_pid(self.new_vel[0], self.os.speed, gps_time_diff)
                    msg = Float32MultiArray()
                    msg.data = [thrust, thrust, 0.0]
            else:
                if self.flag:
                    print("häää")
                    starting_time = perf_counter_ns()
                    # Calculate the VOs and the new optimal velocity
                    self.new_vel = self.vo.calc_vel_final(TS_all, info_OS, False)
                    self.elapsed_time.append((perf_counter_ns()-starting_time)/1000000)
                else:
                    self.elapsed_time.append(0)
                    self.new_vel = vel_des

                print("New vel: ", self.new_vel, "OS vel:", vel_OS, "Time: ",(perf_counter_ns()-self.start_time)/1000000)
                                
                # Control output for changing the course angle based on the new velocity     
                if self.angle_diff(vel_OS[1], self.new_vel[1]) > 5:
                    thrust = self.compute_pid(self.new_vel[0], self.os.speed, gps_time_diff)   
                    rot = self.compute_pd(self.angle_diff(vel_OS[1], self.new_vel[1]), gps_time_diff)
                    msg = Float32MultiArray()
                    msg.data = [thrust+rot, thrust-rot, 0.0]
                elif self.angle_diff(vel_OS[1], self.new_vel[1]) < -5:
                    thrust = self.compute_pid(self.new_vel[0], self.os.speed, gps_time_diff)
                    rot = self.compute_pd(self.angle_diff(vel_OS[1], self.new_vel[1]), gps_time_diff)
                    msg = Float32MultiArray()
                    msg.data = [thrust+rot, thrust-rot, 0.0]
                else:
                    thrust = self.compute_pid(self.new_vel[0], self.os.speed, gps_time_diff)
                    msg = Float32MultiArray()
                    msg.data = [thrust, thrust, 0.0]
            
            # publish the new values for the thrusters to unity
            if gps_time_diff > 0:
                self.thruster_pub_os.publish(msg)
            # Store the velocity obtained by the VO algorithm in a list    
            self.speed_com.append(self.new_vel[0])
            self.ang_com.append(self.new_vel[1])
    
    def exit_handler(self): # Just for plotting when closing the node with ctrl+c
        """Function that is only used once when the ROS node is closed, to plot the results of the simulation and store the results in a .csv file
        """
        os_position = np.array(self.os_pos)
        ts_position = np.array(self.ts_pos_1)
        ts_position_2 = np.array(self.ts_pos_2)
        ts_position_3 = np.array(self.ts_pos_3)

        os_speed = np.array(self.os_speed)
        os_ang = np.array(self.os_ang)
        dist_os_ts_1 = np.array(self.dist_os_ts_1)
        dist_os_ts_2 = np.array(self.dist_os_ts_2)
        dist_os_ts_3 = np.array(self.dist_os_ts_3)
        simu_time = np.array(self.simu_time)
        
        # Setup for the csv file
        fields = ["Sim Time", "Distance to TS 1", "Distance to TS 2", "Distance to TS 3", "Speed Com", "Angle Com", "Speed OS", "Angle OS", "Run Time", "OS pos", "TS pos", "Coll check"]
        rows = [simu_time, dist_os_ts_1, dist_os_ts_2, dist_os_ts_3,self.speed_com, self.ang_com, os_speed, os_ang, self.elapsed_time, os_position, ts_position, self.coll_check]
        
        filename = "src/asv_path_planner/Simulation output/simulation_results_.csv"
        # writing to csv file  
        with open(filename, 'w') as csvfile:  
            # creating a csv writer object  
            csvwriter = csv.writer(csvfile)  
                
            # writing the fields  
            csvwriter.writerow(fields)  
                
            # writing the data rows  
            csvwriter.writerow(rows)

        # For plotting all data have to be of the same size     
        min_length = min(len(simu_time), len(dist_os_ts_1), len(dist_os_ts_2), len(self.speed_com), len(self.ang_com), len(os_speed), len(os_ang), len(self.elapsed_time), len(self.coll_check))
        # Remove the first 15 measurents because here the ships are still starting up and the speed and orientation calculation from the GPS are not good
        simu_time = simu_time[15:min_length]
        dist_os_ts_1 = dist_os_ts_1[15:min_length]
        dist_os_ts_2 = dist_os_ts_2[15:min_length]
        dist_os_ts_3 = dist_os_ts_3[15:min_length]
        self.speed_com = self.speed_com[15:min_length]
        self.ang_com = self.ang_com[15:min_length]
        os_speed = os_speed[15:min_length]
        os_ang = os_ang[15:min_length]
        self.elapsed_time = self.elapsed_time[15:min_length]
        self.coll_check = self.coll_check[15:min_length]

        # Plot the distance between TS and OS over time
        fig1 = plt.figure()
        ax1 = fig1.add_subplot(111)
             
        min_dist = min(dist_os_ts_1)
        min_dist = round(min_dist, 2)
        ind_min_dist = dist_os_ts_1.argmin()
        time_min_dist = simu_time[ind_min_dist]
        plt.plot(simu_time, dist_os_ts_1, c="blue", label="Distance between OS and TS 1", linewidth=2.5)
        plt.scatter(time_min_dist, min_dist, marker="x", c="orange", zorder=2, label=f"min. distance to TS 1 = {min_dist} m", linewidth=2.5)
        min_dist_2 = min(dist_os_ts_2)
        min_dist_2 = round(min_dist_2, 2)
        ind_min_dist_2 = dist_os_ts_2.argmin()
        time_min_dist_2 = simu_time[ind_min_dist_2]
        # plt.plot(simu_time, dist_os_ts_2, c="red", linestyle="dashed", label="Distance between OS and TS 2", linewidth=2.5)
        # plt.scatter(time_min_dist_2, min_dist_2, marker="+", c="green", zorder=2, label=f"min. distance to TS 2 = {min_dist_2} m", linewidth=2.5)
        min_dist_3 = min(dist_os_ts_3)
        min_dist_3 = round(min_dist_3, 2)
        ind_min_dist_3 = dist_os_ts_3.argmin()
        time_min_dist_3 = simu_time[ind_min_dist_3]
        # plt.plot(simu_time, dist_os_ts_3, c="orange", linestyle="dotted", label="Distance between OS and TS 3", linewidth=2.5)
        # plt.scatter(time_min_dist_3, min_dist_3, marker=".", c="blue", zorder=2, label=f"min. distance to TS 3 = {min_dist_3} m", linewidth=2.5)

        # plt.title("Distance betweenn OS and TS")
        plt.xlabel("Time [s]")
        plt.ylabel("Distance [m]")
        plt.legend(loc="best", fontsize="10")
        ax1.set_aspect(1.0/ax1.get_data_ratio(), adjustable='box')
        
        plt.savefig("src/asv_path_planner/Simulation output/Distance.pdf", format="pdf", bbox_inches="tight")

        # Plot the current speed and the desired speed (calculated by VO algorithm) of the OS
        fig2 = plt.figure()
        ax2 = fig2.add_subplot(111)
        plt.plot(simu_time, os_speed, c="teal", label="Current speed (OS)", linewidth=2.5)
        plt.plot(simu_time, self.speed_com, c="red", label="Desired speed (OS)", linestyle="dashed", linewidth=2.5)
        # plt.title("Speed OS")
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [m/s]")
        plt.legend(loc="best", fontsize="10")
        ax2.set_aspect(1.0/ax2.get_data_ratio(), adjustable='box')

        plt.savefig("src/asv_path_planner/Simulation output/Speed.pdf", format="pdf", bbox_inches="tight")

        # Plot the current orientation and the desired orientation (calculated by VO algorithm) of the OS
        fig3 = plt.figure()
        ax3 = fig3.add_subplot(111)
        plt.plot(simu_time, os_ang, c="teal", label="Current orientation (OS)", linewidth=2.5)
        plt.plot(simu_time, self.ang_com, c="red", label="Desired orientation (OS)", linestyle="dashed", linewidth=2.5)
        # plt.title("Orientation OS")
        plt.xlabel("Time [s]")
        plt.ylabel("Orientation [°]")
        plt.legend(loc="best", fontsize="10")
        ax3.set_aspect(1.0/ax3.get_data_ratio(), adjustable='box')
        
        plt.savefig("src/asv_path_planner/Simulation output/Orientation.pdf", format="pdf", bbox_inches="tight")

        # Plot of the VO Algorithm run time
        # fig4 = plt.figure()
        # ax4 = fig4.add_subplot(111)
        # plt.plot(simu_time, self.elapsed_time, c="red", label="Run time")
        # # plt.title("Algorithm run time")
        # plt.xlabel("Time [s]")
        # plt.ylabel("Run time [ms]")
        # # plt.legend(loc="upper left", fontsize="10")
        # ax4.set_aspect(1.0/ax4.get_data_ratio(), adjustable='box')

        # Plot the collision check of OS with the safety area (0=no collision, 1=collision)
        fig5 = plt.figure()
        ax5 = fig5.add_subplot(111)
        plt.plot(simu_time, self.coll_check, c="red", label="Collision check with safety domain")
        plt.xlabel("Time [s]")
        plt.ylabel("In/Out safety area")
        plt.legend(loc="best", fontsize="10")
        ax5.set_aspect(1.0/ax5.get_data_ratio(), adjustable='box')

        plt.savefig("src/asv_path_planner/Simulation output/Collision.pdf", format="pdf", bbox_inches="tight")

        # Plot the trajectory of the ships
        fig6 = plt.figure()
        # points_test = []
        os_timestamp = np.empty((0,2))
        ts_timestamp = np.empty((0,2))
        ts_timestamp_2 = np.empty((0,2))
        ts_timestamp_3 = np.empty((0,2))
        ref_tp = self.vo.calc_coord_gps_to_xy(self.ref_point, self.gps_tp)
        prev_timestamp = simu_time[0]
        for timestamp, os_pos, ts_pos, ts_pos2, ts_pos3 in zip(simu_time, os_position, ts_position, ts_position_2, ts_position_3):
            time_diff = timestamp - prev_timestamp
            if time_diff >= 7.0:
                os_timestamp = np.vstack((os_timestamp, os_pos))
                ts_timestamp = np.vstack((ts_timestamp, ts_pos))
                ts_timestamp_2 = np.vstack((ts_timestamp_2, ts_pos2))
                ts_timestamp_3 = np.vstack((ts_timestamp_3, ts_pos3))
                prev_timestamp = timestamp
        
        
        plt.scatter(os_timestamp[:,0], os_timestamp[:,1], c="black", marker="4", linewidth=1.5, s=50, label="time interval: 7 s")
        plt.plot(os_position[:,0], os_position[:,1], c="teal",zorder=0.5, linestyle="--", label="OS path", linewidth=1.5)
        plt.scatter(ts_timestamp[:,0], ts_timestamp[:,1], c="black", marker="3", linewidth=1.5, s=50)
        plt.plot(ts_position[:,0], ts_position[:,1], c="red",zorder=0.05, linestyle="-.", label="TS 1 path", linewidth=2.5)
        # plt.scatter(ts_timestamp_2[:,0], ts_timestamp_2[:,1], c="black", marker="2", linewidth=1.5, s=50)
        # plt.plot(ts_position_2[:,0], ts_position_2[:,1], c="red",zorder=0.05, linestyle=":", label="TS 2 path")
        # plt.scatter(ts_timestamp_3[:,0], ts_timestamp_3[:,1], c="black", marker="2", linewidth=1.5, s=50)
        # plt.plot(ts_position_3[:,0], ts_position_3[:,1], c="red",zorder=0.05, linestyle=(0, (1, 10)), label="TS 3 path")
        plt.plot((os_position[0,0],ref_tp[0]),(os_position[0,1],ref_tp[1]),c="gray",zorder=0.02, alpha=1.0, label="global path", linewidth=1.5)
        plt.scatter(ref_tp[0],ref_tp[1],c="dimgray", marker="+", label="OS goal", linewidth=2.5)

        # Calculate the vertices of the OS around it
        vert_OS = np.array([[-0.5*self.os.width, -0.5*self.os.length],
                            [0.5*self.os.width, -0.5*self.os.length],
                            [0.5*self.os.width, 0.5*self.os.length],
                            [0, self.os.length],
                            [-0.5*self.os.width, 0.5*self.os.length]])

        # Rotate the vertices in the direction the OS is facing
        rot_M_OS = np.array([[np.cos(np.deg2rad(self.os.ang)), -np.sin(np.deg2rad(self.os.ang))],
                            [np.sin(np.deg2rad(self.os.ang)), np.cos(np.deg2rad(self.os.ang))]])
    
        vert_OS = np.dot(vert_OS,rot_M_OS)

        # Add the Position of the OS
        vert_OS[:, 0] = vert_OS[:, 0]+os_position[-1,0]
        vert_OS[:, 1] = vert_OS[:, 1]+os_position[-1,1]
        plt.fill(vert_OS[:,0],vert_OS[:,1], "blue", hatch="----", edgecolor="black", linewidth=0.5, label="OS")

        # Calculate the vertices of the TS around it
        vert_TS = np.array([[-0.5*self.ts_1.width, -0.5*self.ts_1.length],
                            [0.5*self.ts_1.width, -0.5*self.ts_1.length],
                            [0.5*self.ts_1.width, 0.5*self.ts_1.length],
                            [0, self.ts_1.length],
                            [-0.5*self.ts_1.width, 0.5*self.ts_1.length]])

        # Rotate the vertices in the direction the TS is facing
        rot_M_TS = np.array([[np.cos(np.deg2rad(self.ts_1.ang)), -np.sin(np.deg2rad(self.ts_1.ang))],
                            [np.sin(np.deg2rad(self.ts_1.ang)), np.cos(np.deg2rad(self.ts_1.ang))]])

        vert_TS[:] = np.dot(vert_TS[:], rot_M_TS)
        
        # Add the Position of the TS
        vert_TS[:, 0] = vert_TS[:, 0]+ts_position[-1,0]
        vert_TS[:, 1] = vert_TS[:, 1]+ts_position[-1,1]
        plt.fill(vert_TS[:,0],vert_TS[:,1], "orange", hatch="||||", edgecolor="black", linewidth=0.5, label="TS 1")

        # Calculate the vertices of the TS around it
        vert_TS_2 = np.array([[-0.5*self.ts_2.width, -0.5*self.ts_2.length],
                            [0.5*self.ts_2.width, -0.5*self.ts_2.length],
                            [0.5*self.ts_2.width, 0.5*self.ts_2.length],
                            [0, self.ts_2.length],
                            [-0.5*self.ts_2.width, 0.5*self.ts_2.length]])

        # Rotate the vertices in the direction the TS is facing
        rot_M_TS_2 = np.array([[np.cos(np.deg2rad(self.ts_2.ang)), -np.sin(np.deg2rad(self.ts_2.ang))],
                            [np.sin(np.deg2rad(self.ts_2.ang)), np.cos(np.deg2rad(self.ts_2.ang))]])

        vert_TS_2[:] = np.dot(vert_TS_2[:], rot_M_TS_2)
        
        # Add the Position of the TS
        vert_TS_2[:, 0] = vert_TS_2[:, 0]+ts_position_2[-1,0]
        vert_TS_2[:, 1] = vert_TS_2[:, 1]+ts_position_2[-1,1]
        # plt.fill(vert_TS_2[:,0],vert_TS_2[:,1], "orange", hatch="////", edgecolor="black", linewidth=0.5, label="TS 2")

        # Calculate the vertices of the TS around it
        vert_TS_3 = np.array([[-0.5*self.ts_3.width, -0.5*self.ts_3.length],
                            [0.5*self.ts_3.width, -0.5*self.ts_3.length],
                            [0.5*self.ts_3.width, 0.5*self.ts_3.length],
                            [0, self.ts_3.length],
                            [-0.5*self.ts_3.width, 0.5*self.ts_3.length]])

        # Rotate the vertices in the direction the TS is facing
        rot_M_TS_3 = np.array([[np.cos(np.deg2rad(self.ts_3.ang)), -np.sin(np.deg2rad(self.ts_3.ang))],
                            [np.sin(np.deg2rad(self.ts_3.ang)), np.cos(np.deg2rad(self.ts_3.ang))]])

        vert_TS_3[:] = np.dot(vert_TS_3[:], rot_M_TS_3)
        
        # Add the Position of the TS
        vert_TS_3[:, 0] = vert_TS_3[:, 0]+ts_position_3[-1,0]
        vert_TS_3[:, 1] = vert_TS_3[:, 1]+ts_position_3[-1,1]
        # plt.fill(vert_TS_3[:,0],vert_TS_3[:,1], "orange", hatch="----", edgecolor="black", linewidth=0.5, label="TS 3")

        plt.legend(loc="best", fontsize="10")
        
        # velobst.calc_vel_final(self.TS_1, self.OS, True, self.os_pos[-1])
        plt.axis('square')
        bot_left = self.vo.calc_coord_gps_to_xy(self.ref_point, np.array([45.000899825520364, 15.00126830157632]))
        bot_right = self.vo.calc_coord_gps_to_xy(self.ref_point, np.array([45.000899769180805, 15.003804904724001]))
        top_left = self.vo.calc_coord_gps_to_xy(self.ref_point, np.array([45.002699490216614, 15.001268341281854]))
        top_right = self.vo.calc_coord_gps_to_xy(self.ref_point, np.array([45.002699433873545, 15.003805023840597]))
        plt.axis([bot_left[0],bot_right[0]+5,bot_left[1]-2.5,top_left[1]+2.5])
       
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')

        plt.savefig("src/asv_path_planner/Simulation output/Trajectory.pdf", format="pdf", bbox_inches="tight")
        plt.show()

def main(args=None):
    """Main function where the ROS node is assigned and it the node loops infinite
    """
    rclpy.init(args=args)
    node = RosScriptNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
