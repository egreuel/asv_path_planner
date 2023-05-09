from collections import Counter

import numpy as np
import matplotlib.pyplot as plt
import math

from shapely.geometry import Polygon, LineString, Point
from geopy import distance
from scipy.spatial import ConvexHull
from scipy.spatial.distance import cdist
from scipy.stats import mode

# input: vel_OS[speed(m/s), angle(° in NED frame)], len_OS[m], wid_OS[m], max_speed_OS [m/s], max_TTC[s], min_TTC[s], saftey_fact[],
#        speed_TS[m/s], speed_TS_unc[m/s], ang_TS[° in NED frame], ang_TS_unc[°], speed_res[m/s], ang_res[°], pos_TS_rel[x,y], len_TS[m], wid_TS[m]

# output: VO, absolute course

# Plotting
# plotting = False
class VO:

    # Weights for the cost-function to choose the new velocity
    w_1 = 1  # Angle deviation from desired velocity
    w_2 = 1.3  # Speed deviation from desried velocity
    w_3 = 1.5  # Angle deviation from 30°
    
    def __init__(self, leng_OS, width_OS, max_speedOS, max_time_col, treshhold, safe_domain_fact, speed_unc, ang_unc, speed_res, ang_res):
        
        # Properties of OS
        self.len_OS = leng_OS  # Length OS in m
        self.wid_OS = width_OS  # Width OS in m
        self.max_speed_OS = max_speedOS # Maximum speed of OS in m/s

        # Time thresholds
        self.max_TTC = max_time_col  # Time to collison in s
        self.threshold = treshhold  # time to collision threshold in s for standby actions

        # Safety domain factor
        self.safe_Fact = safe_domain_fact
        
        # Uncertainty handling
        self.unc_speed = speed_unc  # m/s
        self.unc_angle = ang_unc  # degrees
        
        # Resolution of the discretized velocity space
        self.res_speed = speed_res # m/s
        self.res_ang = ang_res # degrees
              
        # Create last state check array
        self.ts_vo_checks = np.empty((1, 15), dtype=object)

        # Fill the array with None values
        self.ts_vo_checks.fill(None)

        # Variable to store the inital state of OS once a collision check is true
        self.vel_OS_init = []
        self.flag = True

        # Variable to store if the saftey domain is violated
        self.flag_coll = True
        self.coll_safety = False

        # Variable to store the latest calculated new velocity
        self.latest_new_vel = np.empty((0,2))

    def calc_coord_gps_to_xy(self, coord_os, coord_ts):
        """ Function to calc GPS coordinates to xy-coordinates in meters to the OS;
        Returns the position of the target ship relative to the own ship in the xy-
        coordinate system
        """
        pos_TSy = np.array([coord_ts[0], coord_os[1]])
        pos_TSx = np.array([coord_os[0], coord_ts[1]])
        # Relative position and realtive velocity
        # Relative position:
        pos_Rel_gps = coord_ts - coord_os
        if pos_Rel_gps[0] < 0:
            pos_Rely = distance.great_circle(pos_TSy, coord_os) * -1
        else:
            pos_Rely = distance.great_circle(pos_TSy, coord_os)

        if pos_Rel_gps[1] < 0:
            pos_Relx = distance.great_circle(pos_TSx, coord_os) * -1
        else:
            pos_Relx = distance.great_circle(pos_TSx, coord_os)
        pos_Rel = (pos_Relx.meters, pos_Rely.meters)
        return np.array(pos_Rel)


    def calc_abs_vel(self, ref_vel, targ_vel):
        """ Function to calculate the absolute velocity of the TS if only the
        relative velocity of TS is given;
        Returns the absolute velocity of the target ship with speed and angle
        """
        # x and y componet of the vector from magnitude(speed) and direction(angle)
        ref_velxy = self.vect_to_xy(ref_vel)
        vel_Relxy = self.vect_to_xy(targ_vel)
        # Calculate the TS vector in component form
        vel_TSxy_calc = vel_Relxy + ref_velxy
        # Calculate the magnitude(speed) of the vector
        speed_TS_calc = np.sqrt(vel_TSxy_calc[0] ** 2 + vel_TSxy_calc[1] ** 2)
        # Calculate the dircetion(angle) of the vector
        ang_TS_calc = np.degrees(np.arctan2(vel_TSxy_calc[1], vel_TSxy_calc[0]))
        ang_TS_calc = (ang_TS_calc+360) % 360
        # convert the angle from east CCW to north CW
        ang_TS_calc = self.calc_ang_n_to_e(ang_TS_calc)
        vel_TS_calc = np.array([speed_TS_calc, ang_TS_calc])
        return np.array(vel_TS_calc)


    def calc_rel_vel(self, vel_OS, vel_TS):
        """ Function to calculate the relative velocity of OS and TS """
        # x and y componet of the vector from magnitude(speed) and direction(angle)
        vel_OSxy = self.vect_to_xy(vel_OS)
        vel_TSxy = self.vect_to_xy(vel_TS)
        # Calculate the relative vector in component form
        vel_Relxy = vel_OSxy - vel_TSxy
        # Calculate the magnitude(speed) of the vector
        speed_Rel = np.sqrt(vel_Relxy[0] ** 2 + vel_Relxy[1] ** 2)
        # Calculate the dircetion(angle) of the vector
        # Add special cases for rel Velocity speed = 0
        ang_Rel = np.degrees(np.arctan2(vel_Relxy[1], vel_Relxy[0]))
        ang_Rel = (ang_Rel+360) % 360
        # convert the angle from east CCW to north CW
        ang_Rel = self.calc_ang_n_to_e(ang_Rel)
        vel_Rel = np.array([speed_Rel, ang_Rel])
        return np.array(vel_Rel)

    def vect_to_ang_mag(self, vect_xy):
        """ Function that calculates the magnitude and direction of a vector given
        in x/y-form """
        speed = np.sqrt(vect_xy[0] ** 2 + vect_xy[1] ** 2)
        angle = math.degrees(np.arctan2(vect_xy[1], vect_xy[0]))
        angle = (angle+360) % 360
        angle = self.calc_ang_n_to_e(angle)
        vector = ([speed, angle])
        return np.array(vector)

    def vect_to_xy(self, vector):
        """ Function that calculates the x/y-form of a vector given in
        magnitude/direction-form """
        vect_xy = ([vector[0] * math.cos(math.radians(self.calc_ang_n_to_e(vector[1]))),
                            vector[0] * math.sin(math.radians(self.calc_ang_n_to_e(vector[1])))])
        
        # slower:
        # vect_xy = ([vector[:,0] * np.cos(np.radians(self.calc_ang_n_to_e(vector[:,1]))),
        #                     vector[:,0] * np.sin(np.radians(self.calc_ang_n_to_e(vector[:,1])))])
        return np.array(vect_xy)

    def calc_ang_n_to_e(self, angle):
        """ Function that converts the angle from the north clockwise system to the
        east counter clockwise system and vice versa """
        alpha = (-angle + 90) % 360
        return alpha

    def calc_minkowski_sum(self, vert_TS, vert_OS):
        """ Function of the minkoswski sum """
        result = []
        for x in vert_TS:
            for y in vert_OS:
                result.append((x[0] + y[0], x[1] + y[1]))
        return np.array(result)

    def orientation(self, v1, v2, p):
        """
        Calculation of all points that are right or left of a directed vector
        Parameters
        ----------
        v1 : Vertex 1 of the velocity obstacle.
        v2 : Vertex 2 of the velocity obstacle.
        p : Points that are tested

        Returns
        -------
        v_out :  all points left from the vector formed by v1, v2
        v_in : all points left from the vector formed by v1, v2
        """     
        val = (float(v2[1] - v1[1]) * (p[:,0] - v2[0])) - \
            (float(v2[0] - v1[0]) * (p[:,1] - v2[1]))
        
        v_out = []
        v_in = []
        for i in range(len(val)):
            
            # check if inside
            if val[i] <= 0:
                v_in.append(p[i, :])
            else:
            # check if outside
                v_out.append(p[i, :])
                
        return np.array(v_out), np.array(v_in)
        

    def inOutVO(self, v_test, vo):
        """
        Calculation of all point inside or outside the velocity obstacle
        Parameters
        ----------
        v_test : Velocity points to test
        vo : velocity obstacle vertices

        Returns
        -------
        v_outn : all points outside the VO 
        v_test : all points inside the VO

        """
        v_outn = np.empty((0,2))
        for i in range(len(vo)-1):
            if not np.any(v_test):
                break
            v_out, v_test  = (self.orientation(vo[i,:], vo[i+1,:], v_test))
            if np.any(v_out):
                v_outn = np.vstack((v_outn, v_out))
        return np.array(v_outn), np.array(v_test)

    def check_collision(self, vel_p, vel_obs):
        """ Function Check for Collision """
        vel_p_xy = self.vect_to_xy(vel_p)
        vel_p_xy = np.reshape(vel_p_xy, (1,2))
        outo, intu = self.inOutVO(vel_p_xy, vel_obs)
        
        if np.any(intu):
            collision = True
        else:
            collision = False
        return collision

    def check_collision_col(self, vel_point, pos_tip, col_cons):
        """ Function Check for Collision """
        bound_left = col_cons[0]
        bound_right = col_cons[1]
        
        # Form the vector from the tip of the vel_obs to the point that has to be checked
        vel_point_xy = self.vect_to_xy(vel_point)
        vect_check_xy = vel_point_xy - pos_tip
        vect_check = self.vect_to_ang_mag(vect_check_xy)
        
        if self.check_between_angles(vect_check[1],bound_left,bound_right):
            collision = True
        else:
            collision = False
        return collision

    def check_between_angles(self, angle, bound_left, bound_right):
        """ Function to check wether an angle is between two bound, given by two angles"""
        if bound_right > bound_left:
            ang_betw = bound_right - bound_left
        else:
            ang_betw = (bound_right + 360) - bound_left
        
        ang_diff_1 = angle - bound_left
        if ang_diff_1 < 0:
            ang_diff_1 += 360
            
        if ang_diff_1 < ang_betw:
            return True
            
        ang_diff_2 = bound_right - angle
        if ang_diff_2 < 0:
            ang_diff_2 += 360
            
        if ang_diff_2 < ang_betw:
            return True
            
        return False

    def calc_ang_vect(self, vel_OS, vel_Rel):
        """ Function to calulate the angle between the two velocity vectors """
        phi = ((vel_Rel[1]-vel_OS[1])+360) % 360
        return phi

    def check_colreg_rule_heading(self, vel_OS_2, vel_TS_2):
        """ Function to check COLREG rule (angle between vel_OS and
        vel_TS heading// course over ground) """
        vel = vel_TS_2.copy()
        vel[1] = vel_TS_2[1] - 180
        if vel[0] <= 0.1:  # Threshhold for static objects
            colreg_rule_2 = 'Static object'
        else:
            phi_2 = self.calc_ang_vect(vel_OS_2, vel)
            if 15 <= phi_2 <= 112.5:
                colreg_rule_2 = 'Right crossing (Rule 15)'
            elif 112.5 < phi_2 <= 247.5:
                if vel_OS_2[0] <= vel[0]:
                    colreg_rule_2 = 'Being Overtaken (Rule 13)'
                else:
                    colreg_rule_2 = 'Overtaking (Rule 13)'
            elif 247.5 < phi_2 <= 345:
                colreg_rule_2 = 'Left crossing (Rule 15)'
            else:
                colreg_rule_2 = 'Head-on (Rule 14)'
        return colreg_rule_2

    def check_side(self, vel_Rel, tang_coord):
        """ Function that checks wether the relative velocity is inside the CC or
        outside, if outside check which side """
        side = 'empty or inside'

        tang_points_vec_1 = self.vect_to_ang_mag(tang_coord[0, :])
        tang_points_vec_2 = self.vect_to_ang_mag(tang_coord[1, :])

        angles_betw = self.ang_betw_vect(vel_Rel, tang_points_vec_1, tang_points_vec_2)
        angles_betw_tang = self.ang_betw_vect(tang_points_vec_1, tang_points_vec_2)

        if angles_betw[0] > angles_betw_tang:
            side = 'right'
        elif angles_betw[1] > angles_betw_tang:
            side = 'left'

        return side

    def ang_betw_vect(self, ref_vect, *test_vect):
        """ Function to calculate the angle between two vectors """
        ref_vect_xy = self.vect_to_xy(ref_vect)
        test_vect_xy = np.apply_along_axis(self.vect_to_xy, 1, test_vect)
        test_vect = np.apply_along_axis(self.vect_to_ang_mag, 1, test_vect_xy)
        in_arcos = (np.dot(test_vect_xy, ref_vect_xy)) / \
            (test_vect[:, 0]*ref_vect[0])
        in_arcos = np.round_(in_arcos, decimals=10)
        angles_betw = np.rad2deg(np.arccos(in_arcos))

        return np.array(angles_betw)

    def calc_vel_obst(self, TS, ang_os_rad):
        """ Function to calculate the VO, gives back an array with the vertices of
        the VO """
        pos_TS_rel = TS[0]
        len_TS = TS[1]
        wid_TS = TS[2]
        speed_TS = TS[3]
        ang_TS = TS[4]
        vel_TS = np.array([speed_TS, ang_TS])
        # vel_Rel = calc_rel_vel(vel_OS, vel_TS)
        # vel_Relxy = self.vect_to_xy(vel_Rel) # (Just for plotting)
        # Geodesic Distance (ellipsoidal model of the earth) for GPS coordinates
        # dist_TSOS = distance.distance(pos_TS, pos_OS)
        # print('Distance =', dist_TSOS.meters, 'm')

        # Convert angle from degree to radians
        ang_TS_rad = math.radians(ang_TS)
        ang_OS_rad = ang_os_rad
        # Calculate the xy-form of the velocity vector
        vel_TSxy = self.vect_to_xy(vel_TS)

        # Calclate the relative position of the TS in meters
        # pos_TS_rel = calc_coord_gps_to_xy(pos_OS, pos_TS)

        # Calculate the vertices of the OS around it
        vert_OS = np.array([[-0.5*self.wid_OS, -0.5*self.len_OS],
                            [0.5*self.wid_OS, -0.5*self.len_OS],
                            [0.5*self.wid_OS, 0.5*self.len_OS],
                            [-0.5*self.wid_OS, 0.5*self.len_OS]])

        # Rotate the vertices in the direction the OS is facing
        rot_M_OS = np.array([[np.cos(ang_OS_rad), -np.sin(ang_OS_rad)],
                            [np.sin(ang_OS_rad), np.cos(ang_OS_rad)]])
    
        vert_OS = np.dot(vert_OS,rot_M_OS)
        
        # add safety factor for the OS
        vert_OS_safe = vert_OS*self.safe_Fact

        
        # Calculate the vertices of the TS around it
        vert_TS = np.array([[-0.5*wid_TS, -0.5*len_TS],
                            [0.5*wid_TS, -0.5*len_TS],
                            [0.5*wid_TS, 0.5*len_TS],
                            [-0.5*wid_TS, 0.5*len_TS]])

        # Rotate the vertices in the direction the TS is facing
        rot_M_TS = np.array([[np.cos(ang_TS_rad), -np.sin(ang_TS_rad)],
                            [np.sin(ang_TS_rad), np.cos(ang_TS_rad)]])

        vert_TS[:] = np.dot(vert_TS[:], rot_M_TS)
        
        # Add the relative Position of the TS
        vert_TS[:, 0] = vert_TS[:, 0]+pos_TS_rel[0]
        vert_TS[:, 1] = vert_TS[:, 1]+pos_TS_rel[1]

        exp_vert_TS = np.zeros([0, 2])  # (Just for plotting)
        exp_vert_TS_safe = np.zeros([0, 2])
        vert_hull = []

        # Expand the shape of the TS with the shape of OS and OS*safty factor
        exp_vert_TS = self.calc_minkowski_sum(vert_TS, -vert_OS)  # (Just for plotting)
        exp_vert_TS_safe = self.calc_minkowski_sum(vert_TS, -vert_OS_safe)

        # # plot the vertices of the OS
        # plt.scatter(vert_OS[:,0], vert_OS[:,1], marker='.', linewidths=(0.1))

        # Convex Hull function of scipy library to create a hull of the new points of the minkowski sum
        
        # Expanded TS shape by OS (just for plotting)
        hull_exp = ConvexHull(exp_vert_TS)
        hull_safe = ConvexHull(exp_vert_TS_safe)

        # Plot the Polygons formed by the hull as filled polygons
        if plotting:
            # Plot the shape of OS
            
            # plt.fill(vert_OS[:, 0]+vert_TS[0, 0], vert_OS[:,1]+vert_TS[0, 1], 'blue', alpha=0.3, zorder=2, edgecolor='black')
            # plt.fill(vert_OS[:, 0]+vert_TS[1, 0], vert_OS[:,1]+vert_TS[1, 1], 'blue', alpha=0.3, zorder=2, edgecolor='black')
            # plt.fill(vert_OS[:, 0]+vert_TS[2, 0], vert_OS[:,1]+vert_TS[2, 1], 'blue', alpha=0.3, zorder=2, edgecolor='black')
            # plt.fill(vert_OS[:, 0]+vert_TS[3, 0], vert_OS[:,1]+vert_TS[3, 1], 'blue', alpha=0.3, zorder=2, edgecolor='black')
            hullTS = ConvexHull(vert_TS)  # Shape of TS (just for plotting)
            
            # plt.fill(exp_vert_TS[hull_exp.vertices, 0]+pos_OS_xy[0],
            #           exp_vert_TS[hull_exp.vertices, 1]+pos_OS_xy[1], 'orange',
            #           alpha=1, zorder=0.1, edgecolor='black')
            if plotvar == 1:
                plt.fill(vert_TS[hullTS.vertices, 0]+pos_OS_xy[0], vert_TS[hullTS.vertices, 1]+pos_OS_xy[1],
                        'red', alpha=1, zorder=0.15, edgecolor='black', label="TS 1")
                plt.fill(exp_vert_TS_safe[hull_safe.vertices, 0]+pos_OS_xy[0],
                    exp_vert_TS_safe[hull_safe.vertices, 1]+pos_OS_xy[1], 'yellow',
                    alpha=0.25, zorder=0.1, edgecolor='black', label="safety area \naround TS")
                plt.fill(vert_OS[:, 0]+pos_OS_xy[0], vert_OS[:,1]+pos_OS_xy[1], 'blue', alpha=1, zorder=0, edgecolor='black',hatch="...", label="OS")
                
            if plotvar == 2:
                plt.fill(vert_TS[hullTS.vertices, 0]+pos_OS_xy[0], vert_TS[hullTS.vertices, 1]+pos_OS_xy[1],
                        'red', alpha=1, zorder=0.15, edgecolor='black', hatch='||', label="TS 2")
                plt.fill(exp_vert_TS_safe[hull_safe.vertices, 0]+pos_OS_xy[0],
                    exp_vert_TS_safe[hull_safe.vertices, 1]+pos_OS_xy[1], 'yellow',
                    alpha=0.25, zorder=0.1, edgecolor='black')

        # Plot the position of OS and TS / optional vertices of the poylgon
        if plotting:
            # plt.scatter(vert_hull[:,0], vert_hull[:,1], marker='.', linewidths=(0.1))
            
            # Plot the origin point of OS and TS
            plt.scatter(pos_OS_xy[0], pos_OS_xy[1], marker='.', s=(5), c="black")
            plt.scatter(pos_TS_rel[0]+pos_OS_xy[0], pos_TS_rel[1]+pos_OS_xy[1], marker='.', s=(5), c="black")

        # Extract the vertices of the polygon from the hull object
        for x in hull_safe.vertices:
            vert_hull.append(hull_safe.points[x])
        
        vert_hull = np.array(vert_hull)
        vert_hull = np.resize(vert_hull, (hull_safe.vertices.shape[0], 2))
        
        # ### just for testing 
        testo_poly = Polygon(vert_hull)
        if testo_poly.contains(Point(0,0)) and self.flag_coll:
            self.coll_safety = True
            self.flag_coll = False
        else:
            self.coll_safety = False

        # if inside the safety domain, use the expanded hull 


        # Find the tangent lines by checking which vertices has the greatest angle in between
        rel_angles = []
        # Calculate the angle between all vertices points
        for i in range(len(vert_hull)):
            for j in range(i+1, len(vert_hull)):
                if (abs(np.arctan2(vert_hull[i, 1], vert_hull[i, 0]) - np.arctan2(vert_hull[j, 1], vert_hull[j, 0]))) > math.radians(180):
                    rel_angles_raw = (np.arctan2(
                        vert_hull[i, 1], vert_hull[i, 0]) - np.arctan2(vert_hull[j, 1], vert_hull[j, 0]))
                    if rel_angles_raw > 0:
                        rel_angles.append(((np.arctan2(vert_hull[i, 1], vert_hull[i, 0]) - np.arctan2(
                            vert_hull[j, 1], vert_hull[j, 0])-math.radians(360)), i, j))
                    else:
                        rel_angles.append(((np.arctan2(vert_hull[i, 1], vert_hull[i, 0]) - np.arctan2(
                            vert_hull[j, 1], vert_hull[j, 0])+math.radians(360)), i, j))
                else:
                    rel_angles.append(((np.arctan2(
                        vert_hull[i, 1], vert_hull[i, 0]) - np.arctan2(vert_hull[j, 1], vert_hull[j, 0])), i, j))
        rel_angles = np.array(rel_angles)            
        rel_angles[:, 0] = np.rad2deg(rel_angles[:, 0])
        # Find the greatest angle in positive or negative direction
        abs_angles = np.abs(rel_angles)

        max_index = np.argmax(abs_angles[:, 0])

        tang_vert = abs_angles[max_index, :]
        if rel_angles[max_index, 0] < 0:
            sign = -1
        else:
            sign = 1

        # Take all vertices between the max angle vertix and min angle vertix
        if sign > 0:
            vert_hull_facing = vert_hull[np.int_(
                tang_vert[1]):np.int_(tang_vert[2])+1]
        elif sign < 0:
            vert_hull_facing = np.delete(vert_hull, np.s_[np.int_(
                tang_vert[1])+1:np.int_(tang_vert[2])], axis=0)
        # Change the order of the coordinates, so that the index of the max angle is on 0. So that the plot later on is correct
            vert_hull_facing = np.concatenate([vert_hull_facing[np.int_(
                tang_vert[1])+1:], vert_hull_facing[:np.int_(tang_vert[1])+1]])

        # extract the two tangent points so that they can be added to the VO later on
        tang_points = np.array([[vert_hull_facing[0, 0], vert_hull_facing[0, 1]],
                                [vert_hull_facing[len(vert_hull_facing)-1, 0],
                                vert_hull_facing[len(vert_hull_facing)-1, 1]]])
        if plotting:
            tang_points = np.array([[vert_hull_facing[0, 0], vert_hull_facing[0, 1]],
                                [vert_hull_facing[len(vert_hull_facing)-1, 0],
                                vert_hull_facing[len(vert_hull_facing)-1, 1]]])*1
        
        # Collision Cone (CC)
        coll_cone = np.vstack((tang_points[0, :], (0,0), tang_points[1, :])) # (Just for plotting)
        
        tang_points_shift = tang_points.copy()
        tang_points_shift[:, 0] = tang_points_shift[:, 0] + vel_TSxy[0]
        tang_points_shift[:, 1] = tang_points_shift[:, 1] + vel_TSxy[1]

        # Calulate the shifted vertices by TTC
        vert_hull_TTC = vert_hull_facing/self.max_TTC
        
        # Collision Cone with time contrains (CC)
        coll_cone_TTC = np.vstack((tang_points[0, :], vert_hull_TTC, tang_points[1, :])) # (Just for plotting)
        
        # Plot the minimum speed shifted with the velocity of TS
        vert_hull_TTC_shifted = vert_hull_TTC.copy()
        vert_hull_TTC_shifted[:, 0] = vert_hull_TTC_shifted[:, 0] + vel_TSxy[0]
        vert_hull_TTC_shifted[:, 1] = vert_hull_TTC_shifted[:, 1] + vel_TSxy[1]
        
        # VO without unc and without time contrains
        vert_VO_no_unc_ttc = np.vstack((tang_points_shift[0, :], [vel_TSxy[0],vel_TSxy[1]], tang_points_shift[1, :])) # (Just for plotting)
        
        # Calculate all points if the vertices TTC_shifted with the circle

        # Line equation for points between shifted tangent points and shifted origin
        # line of the side the COLREG contrains has to be on
        x_p1 = vel_TSxy[0]
        y_p1 = vel_TSxy[1]
        x_p2 = tang_points[0, 0] + vel_TSxy[0]  # left tangent point
        y_p2 = tang_points[0, 1] + vel_TSxy[1]  # left tangent point
        x_p3 = tang_points[1, 0] + vel_TSxy[0]  # right tangent point
        y_p3 = tang_points[1, 1] + vel_TSxy[1]  # right tangent point
        m_1 = (y_p2-y_p1)/(x_p2-x_p1)  # left line seen from OS
        m_2 = (y_p3-y_p1)/(x_p3-x_p1)  # right line seen from OS

        # add uncertainties
        vert_VO_testo = np.vstack((tang_points_shift[0, :],
                                vert_hull_TTC_shifted, tang_points_shift[1, :]))
        vert_VO_testo_unc = self.add_uncert(vert_VO_testo, pos_TS_rel, vel_TS)
        vert_VO_unc_no_TTC = self.add_uncert(vert_VO_no_unc_ttc, pos_TS_rel, vel_TS) # (Just for plotting)
        
        # find the line parallel to the VO of the VO_unc
        b_l = vert_VO_testo_unc[:, 1] - m_1 * vert_VO_testo_unc[:, 0]
        counts = Counter(np.round(b_l, 10))
        most_common = counts.most_common(1)
        b_1 = most_common[0][0]
        
        b_r = vert_VO_testo_unc[:, 1] - m_2 * vert_VO_testo_unc[:, 0]
        counts = Counter(np.round(b_r, 10))
        most_common = counts.most_common(1)
        b_2 = most_common[0][0]
        
        # # intersection of the two tangent lines added with unc
        # x_inters = (b_1-b_2)/(m_2-m_1)
        # y_inters = m_1*x_inters + b_1
        # point_tipp = np.array([x_inters, y_inters])
        
        # find index of the value b_1 in b_l
        # find index of the value b_2 in b_r
        index_l = np.where(np.round(b_l, 10) == b_1)[0]
        index_r = np.where(np.round(b_r, 10) == b_2)[0]
        
        # Calculate distance of the two each indices, take the one with the furthest distance
        distance_i_l = []
        distance_i_r = []
        for x in index_l:
            cache = np.sqrt((vel_TSxy[1]-vert_VO_testo_unc[x, 1])**2
                            + (vel_TSxy[0]-vert_VO_testo_unc[x, 0])**2)
            distance_i_l.append(cache)
        for x in index_r:
            cache = np.sqrt((vel_TSxy[1]-vert_VO_testo_unc[x, 1])**2
                            + (vel_TSxy[0]-vert_VO_testo_unc[x, 0])**2)
            distance_i_r.append(cache)

        distance_i_l = np.array(distance_i_l)
        distance_i_r = np.array(distance_i_r)
        index_p_l = index_l[np.argmin(distance_i_l)]
        index_p_r = index_r[np.argmin(distance_i_r)]
        
        if index_p_l > index_p_r:
            
            vert_hull_unc = np.vstack(
                (vert_VO_testo_unc[index_p_l-1:], vert_VO_testo_unc[:index_p_r+2]))
        else:
            if index_p_l == 0:
                vert_hull_unc = np.vstack((vert_VO_testo_unc[index_p_l-1],vert_VO_testo_unc[:index_p_r+2]))
            elif index_p_r == (len(vert_VO_testo_unc)-1):
                vert_hull_unc = np.vstack((vert_VO_testo_unc[index_p_l-1:],vert_VO_testo_unc[:1]))
            else:
                vert_hull_unc = vert_VO_testo_unc[index_p_l-1:index_p_r+2] ##### == shifted hull vert unc with the 2nd point on line if index_p_l-1 and index_p_r+2
        # Extract the unc vertices
        # take the two points of the b and calculate the distance to TS. Take the further point. Then extract all points between these two points. Depending on where they are in the array either delete or take all.
        points_colreg_line = np.vstack((vert_hull_unc[0],vert_hull_unc[1]))
        # VO with unc and no TTC
        vel_obst_TTC_unc = vert_hull_unc.copy()
        
        if plotting:
            # plt.quiver(0, 0, vel_Relxy[0], vel_Relxy[1], scale=1, scale_units='xy', angles='xy', color='purple', zorder=2, width=0.01)
            # plt.annotate('$V_{OS}-V_{TS}$', (vel_Relxy[0],vel_Relxy[1]), (vel_Relxy[0]+1.5,vel_Relxy[1]-2), c="purple")
            if plotvar == 1:
                plt.quiver(pos_OS_xy[0], pos_OS_xy[1], vel_OSxy[0], vel_OSxy[1], scale=1,
                            scale_units='xy', angles='xy', color='black', zorder=6, width=0.005, label="v\u20D7 ships")
            plt.quiver(pos_TS_rel[0]+pos_OS_xy[0], pos_TS_rel[1]+pos_OS_xy[1], vel_TSxy[0], vel_TSxy[1],
                        scale=1, scale_units='xy', angles='xy', color='black', zorder=2, width=0.005)
            # plt.quiver(0, 0, vel_TSxy[0], vel_TSxy[1],
            #             scale=1, scale_units='xy', angles='xy', color='black',
            #             zorder=2, width=0.0051)
            # plt.annotate('$V_{TS}$', (vel_TSxy[0],vel_TSxy[1]), ((vel_TSxy[0]-2.3),(vel_TSxy[1]-1.5)),c="black")
            # plt.quiver(tang_points[0,0],tang_points[0,1],vel_TSxy[0], vel_TSxy[1],
            #             scale=1, scale_units='xy', angles='xy', color='red',
            #             zorder=2, width=0.0051)
            # plt.annotate('$V_{TS}$', (tang_points[0,0]+vel_TSxy[0],tang_points[0,1]+vel_TSxy[1]), ((vel_TSxy[0]+tang_points[0,0]-2.3),(vel_TSxy[1]+tang_points[0,1]+1.2)))
            # plt.quiver(tang_points[1,0],tang_points[1,1],vel_TSxy[0], vel_TSxy[1],
            #             scale=1, scale_units='xy', angles='xy', color='red',
            #             zorder=2, width=0.0051)
            # plt.annotate('$V_{TS}$', (tang_points[1,0]+vel_TSxy[0],tang_points[1,1]+vel_TSxy[1]), ((vel_TSxy[0]+tang_points[1,0]-0.5),(vel_TSxy[1]+tang_points[1,1]-2)))
            # plt.plot(coll_cone[:,0], coll_cone[:,1], "lightgray", zorder=0)
            # plt.fill(coll_cone[:,0], coll_cone[:,1], "white", zorder=0)
            # plt.annotate("CC", (9,15),c="dimgray")
            # plt.plot(coll_cone_TTC[:,0], coll_cone_TTC[:,1], "gray", zorder=0.001)
            # plt.fill(coll_cone_TTC[:,0], coll_cone_TTC[:,1], "lightgray", zorder=0.001)
            # plt.annotate("CC$^*$", (9,15),c="dimgray")
            # plt.plot(vert_VO_no_unc_ttc[:,0], vert_VO_no_unc_ttc[:,1], "salmon",zorder=0)
            # plt.plot(vel_obst_unc[:,0], vel_obst_unc[:,1], "red",zorder=0)
            # plt.plot(vert_VO_testo[:,0], vert_VO_testo[:,1], "black",zorder=0.02)
            # plt.fill(vert_VO_testo[:,0], vert_VO_testo[:,1], "salmon", zorder=0.02)
            # plt.annotate("VO", (4,20),c="darkred")
            # plt.plot(vert_VO_testo_unc[:,0], vert_VO_testo_unc[:,1], "green")
            # plt.scatter(pos_TS_rel[0]+vel_TSxy[0],pos_TS_rel[1]+vel_TSxy[1], marker="x",c="black", s=10, linewidths=(1),zorder=1)
            # plt.plot(vel_obst_TTC_unc[:,0]+pos_OS_xy[0], vel_obst_TTC_unc[:,1]+pos_OS_xy[1], "black", zorder=0.01)
            if plotvar == 1:
                plt.fill(vel_obst_TTC_unc[:,0]+pos_OS_xy[0], vel_obst_TTC_unc[:,1]+pos_OS_xy[1], "red", zorder=0.01,alpha=0.5,label="VO")
            if plotvar == 2:
                plt.fill(vel_obst_TTC_unc[:,0]+pos_OS_xy[0], vel_obst_TTC_unc[:,1]+pos_OS_xy[1], "red", zorder=0.01,alpha=0.5)
            # plt.annotate('OS', xy=(0,0), xytext=(self.len_OS/2,-1.1))
            # plt.annotate('TS', xy=(pos_TS_rel[0],pos_TS_rel[1]), xytext=(pos_TS_rel[0]+0.5,pos_TS_rel[1]-0))
            # plt.annotate('$V_{OS}$', (vel_OSxy[0],vel_OSxy[1]), (vel_OSxy[0]+0.5,vel_OSxy[1]-2),zorder=6)
            # plt.annotate('$V_{TS}$', (vel_TSxy[0]+pos_TS_rel[0],vel_TSxy[1]+pos_TS_rel[1]), ((vel_TSxy[0]+pos_TS_rel[0]-4),(vel_TSxy[1]+pos_TS_rel[1]+2)))
            
            # plt.annotate('Safety area \naround TS \n$TS \oplus (n*\minus OS)$', xy=(18, 35), xycoords='data', xytext=(-102,-13), textcoords='offset points', arrowprops=dict(arrowstyle="->", connectionstyle="angle3,angleA=20,angleB=140"))
            # plt.annotate('variable safety \narea around TS', xy=(30 , 23), xycoords='data', xytext=(10, -70), textcoords='offset points', arrowprops=dict(arrowstyle="->", connectionstyle="angle3,angleA=0,angleB=90"))
            # plt.annotate('Uncertainty \nin $V_{TS}$ \n(Unc)', xy=(9.2 , 26), xycoords='data', xytext=(-89, 30), textcoords='offset points', arrowprops=dict(arrowstyle="->", connectionstyle="angle3,angleA=0,angleB=90"), bbox=dict(pad=-9, facecolor="none", edgecolor="none"))
            # plt.annotate('VO with \nuncertainty \nin $V_{TS}$ \n($VO \oplus Unc$)', xy=(11, 15), xycoords='data', xytext=(10, -70), textcoords='offset points', arrowprops=dict(arrowstyle="->", connectionstyle="angle3,angleA=90,angleB=0"))

        return np.array([vel_obst_TTC_unc, vert_hull, tang_points, points_colreg_line], dtype=object)

    def add_uncert(self, vel_obst, posTS_rel, vel_TS):
        """ Function to add the uncertainties in speed and angle of the TS to the
        VO """
        unc_vel = np.array([[vel_TS[0]+self.unc_speed, vel_TS[1]-self.unc_angle],
                        [vel_TS[0]+self.unc_speed, vel_TS[1]],
                        [vel_TS[0]+self.unc_speed, vel_TS[1]+self.unc_angle],
                        [vel_TS[0]-self.unc_speed, vel_TS[1]+self.unc_angle],
                        [vel_TS[0]-self.unc_speed, vel_TS[1]],
                        [vel_TS[0]-self.unc_speed, vel_TS[1]-self.unc_angle]])

        vel_TSxy = self.vect_to_xy(vel_TS)
        # Vectors to xy component
        unc_vert = np.apply_along_axis(self.vect_to_xy, 1, unc_vel)

        # Plot the uncertainty of vel_TS
        # plt.scatter(unc_vert[:,0]+posTS_rel[0], unc_vert[:,1]+posTS_rel[1], marker='.', linewidths=(0.1))
        # plt.fil   l(unc_vert[:,0]+posTS_rel[0], unc_vert[:,1]+posTS_rel[1], 'green', alpha=1 ,zorder=1, edgecolor='black', linewidth=1.5)

        unc_vert[:, 0] = unc_vert[:, 0] - vel_TSxy[0]

        unc_vert[:, 1] = unc_vert[:, 1] - vel_TSxy[1]

        unc_VO = self.calc_minkowski_sum(vel_obst, unc_vert)

        unc_VO_hull = ConvexHull(unc_VO)

        unc_VO_vert = []

        # Extract the vertices of the polygon from the hull object
        for x in unc_VO_hull.vertices:
            unc_VO_vert.append(unc_VO_hull.points[x])

        # print(hull_safe.vertices.shape[0])
        unc_VO_vert = np.array(unc_VO_vert)
        unc_VO_vert = np.resize(unc_VO_vert, (unc_VO_hull.vertices.shape[0], 2))

        # # Plot the vertices of the poylgon
        # if plotting:
        #     # plt.scatter(unc_VO_vert[:,0], unc_VO_vert[:,1], marker='.', linewidths=(0.1))
        #     plt.fill(unc_VO[unc_VO_hull.vertices,0], unc_VO[unc_VO_hull.vertices,1], 'r', alpha=1 ,zorder=0.5, edgecolor='black', linewidth=1.5)

        return np.array(unc_VO_vert)

    def check_coll_point(self, vel_Relxy, vel_OS, vel_OSxy, vel_TS, vel_TSxy, vert_hull, tang_points):
        """ Function to calculate Point of collision, Distance to collision,
        Time to collision """
        # Calculate point, where the vel_Rel hits the polygone of the TS
        # Calculate the relative Velocotiy with magnitude and direction
        vel_Rel_in = self.vect_to_ang_mag(vel_Relxy)
        # Line function of the realtive veloctiy
        m_Rel = vel_Relxy[1]/vel_Relxy[0]
        b_Rel = 0
        # For the intersection point we need a line between two points; take max. x-value of the tangent points to calculate the y-value
        x_line_Rel_1 = tang_points[0, 0]
        x_line_Rel_2 = tang_points[1, 0]
        if x_line_Rel_1 == x_line_Rel_2:
            y_line_Rel_1 = tang_points[0, 1]
            y_line_Rel_2 = tang_points[1, 1]
        else:
            y_line_Rel_1 = m_Rel * x_line_Rel_1 + b_Rel
            y_line_Rel_2 = m_Rel * x_line_Rel_2 + b_Rel
        
        ##### Add cases for m=0 and m=inf!
        # print("xlin",x_line_Rel_2)
        # print("ylin",y_line_Rel_2)
        if 0-0.001 < m_Rel < 0+0.001:
            line_Rel = LineString([(0, 0), (x_line_Rel_2, y_line_Rel_2)])
            
        else:
            line_Rel = LineString(
                [(x_line_Rel_1, y_line_Rel_1), (x_line_Rel_2, y_line_Rel_2)])
        # plt.plot(*line_Rel.xy, label='LineString')
        
        # Contruct a polygone of the hull points of the polygon
        points = vert_hull.tolist()
        hull_safe_2 = Polygon(points)

        # Check wether the relative velocity is inside the CC or outside, if outside check which side and choose the tangent point of this side
        if self.check_side(vel_Rel_in, tang_points) == 'right':
            # print('right')
            pos_TS_rel_coll = tang_points[1, :]
        elif self.check_side(vel_Rel_in, tang_points) == 'left':
            # print('left')
            pos_TS_rel_coll = tang_points[0, :]
        else:
            # print('inside')
            pos_TS_rel_coll = line_Rel.intersection(hull_safe_2)
            # Check which intersection is close to the OS and choose this one as collision point with the hull
            if np.sum(np.abs(pos_TS_rel_coll.bounds[:2])) < np.sum(np.abs(pos_TS_rel_coll.bounds[2:])):
                pos_TS_rel_coll = pos_TS_rel_coll.bounds[:2]
            else:
                pos_TS_rel_coll = pos_TS_rel_coll.bounds[2:]
        
        # Check wether the TS ans OS are in straight line or speed of the TS is zero
        if (vel_OS[1] % 180) == (vel_TS[1] % 180) and vel_OS[1] == vel_TS[1] or vel_TS[0] <= 0:
            time_coll = (np.sqrt(
                pos_TS_rel_coll[0] ** 2 + pos_TS_rel_coll[1] ** 2))/(np.abs(vel_OS[0]-vel_TS[0]))
            dist_coll = vel_OS[0] * time_coll
            coll_vect = np.array([dist_coll, vel_OS[1]])
            point_coll = self.vect_to_xy(coll_vect)
        elif (vel_OS[1] % 180) == (vel_TS[1] % 180) and vel_OS[1] != vel_TS[1]:
            time_coll = (np.sqrt(
                pos_TS_rel_coll[0] ** 2 + pos_TS_rel_coll[1] ** 2))/(vel_OS[0]+vel_TS[0])
            dist_coll = vel_OS[0] * time_coll
            coll_vect = np.array([dist_coll, vel_OS[1]])
            point_coll = self.vect_to_xy(coll_vect)
        else:
            # Calculate the line function of the velocity vector of OS and TS; Origin point of TS is the intersection point where the OS will collide in
            m_TS = vel_TSxy[1]/vel_TSxy[0]
            m_OS = vel_OSxy[1]/vel_OSxy[0]

            b_TS = pos_TS_rel_coll[1]-(m_TS*pos_TS_rel_coll[0])
            b_OS = 0

            # Intersection of the lines (point of collision)
            x = (b_TS-b_OS)/(m_OS-m_TS)
            y = m_OS * x + b_OS
            point_coll = x, y
            # print('Point of collision:',point_coll)

            # Distance to collision
            dist_coll = np.sqrt(point_coll[0] ** 2 + point_coll[1] ** 2)
            # print('Distance to collision:',dist_coll)

            # Time to collision
            time_coll = dist_coll / vel_OS[0]
            # print('Time to collision:',time_coll)
        # if plotting:
        #     plt.scatter(point_coll[0], point_coll[1],
        #                 marker='x', c='black', linewidths=(0.7))
        return np.array([point_coll, dist_coll, time_coll], dtype=object)

    def calc_colreg_con(self, vel_OS, vel_TS, vel_TS_xy, pos_TS_rel):
        """ Function to calculate the COLREG constrains"""
        
        colreg_con = []
        if (self.check_colreg_rule_heading(vel_OS, vel_TS) == 'Head-on (Rule 14)'
            or self.check_colreg_rule_heading(vel_OS, vel_TS) == 'Right crossing (Rule 15)'):
            vect_col_line = (pos_TS_rel + vel_TS_xy) - vel_TS_xy
            perp_line = np.array([-vect_col_line[1], vect_col_line[0]])
            bound_left = perp_line + vel_TS_xy
            # bound_left_2 = -perp_line + vel_TS_xy
            # Either take the left line of the VO and the perpendicular line or take the middle line of the VO and the perpendicular line
            colreg_con = np.array([bound_left, vel_TS_xy, (pos_TS_rel + vel_TS_xy)])
            # if plotting:
            #     colreg_con_2 = np.array([bound_left_2, vel_TS_xy, (pos_TS_rel + vel_TS_xy)])
            #     plt.plot(colreg_con_2[:,0], colreg_con_2[:,1], c='purple',zorder=0.05)
            #     plt.plot(colreg_con_2[:,0], colreg_con_2[:,1], c='purple',zorder=0.05,linestyle='dashed')
            #     # Add the patch to the Axes
            #     plt.gca().add_patch(Rectangle((vel_TS_xy[0],vel_TS_xy[1]),3,3,linewidth=1,edgecolor='purple',facecolor='none', angle=-32))
                # plt.plot(colreg_con[:,0], colreg_con[:,1], c='purple',zorder=0.)
                # plt.fill(colreg_con[:,0], colreg_con[:,1], c='purple',zorder=0, alpha=0.3,linewidth=0.5)
                # plt.annotate("COLREG \ncontrains", (-25,20),c="purple")
                # plt.annotate("$V_1$", (-20,20),c="purple")
                # plt.annotate("$V_2$", (15,0),c="purple")
                # plt.annotate("$V_3$", (-22,-15),c="purple")
        return np.array(colreg_con)

    def calc_free_vel_space(self, velobs, vel_space_free):
        """
        Calculate all velocities that are free of collision

        Parameters
        ----------
        velobs : velocity obstacle
        vel_space_free : all possible velocities of the OS

        Returns
        -------
        vel_space_free : free velocities
        in_all : all velocities inside the VO

        """
        
        in_all = np.empty((0,2))
        for vo in velobs:
            if np.any(vo):
                vel_space_free, into = self.inOutVO(vel_space_free, vo)
                if np.any(into):
                    in_all = np.vstack((in_all, into))
        
        return vel_space_free, in_all

    def calc_new_vel_colreg(self, vel_des, search_area, vel_OS):
        """ Function to calculate the new velocity for encounters, where COLREG
        constrains are applied """
        if np.any(search_area) and self.coll_safety == False:
            # Calculate new vel
            vel_space_free_xy = search_area
            vel_space_free_mag = np.sqrt(vel_space_free_xy[:,0] ** 2 + vel_space_free_xy[:,1] ** 2)
            
            # Cost function to choose new velocity
            in_arcos_des = (np.dot(vel_space_free_xy, self.vect_to_xy(vel_des))
                            )/(vel_space_free_mag*vel_des[0])
            in_arcos_des = np.round_(in_arcos_des, decimals=10)
            angles_des_free = np.rad2deg(np.arccos(in_arcos_des))
            
            speed_des_free = np.abs(vel_space_free_mag - vel_des[0])
            
            vel_30 = vel_OS.copy()
            vel_30[1] = (vel_30[1] + 30) % 360
            in_arcos_30 = (np.dot(vel_space_free_xy, self.vect_to_xy(vel_30))
                            )/(vel_space_free_mag*vel_30[0])
            in_arcos_30 = np.round_(in_arcos_30, decimals=10)
            angles_30_free = np.rad2deg(np.arccos(in_arcos_30))
                                        
            # Normalize the values between 0-1, with 0 = 0 and 1 = max value
            norm_ang_des_free = angles_des_free / np.max(angles_des_free)
            norm_speed_des_free = speed_des_free / np.max(speed_des_free)
            if np.max(angles_30_free) == 0:
                norm_ang_30_free = angles_30_free
            else:
                norm_ang_30_free = angles_30_free / np.max(angles_30_free)
        
        
            # Cost function
            J = (VO.w_1 * norm_ang_des_free + VO.w_2 * norm_speed_des_free +
                VO.w_3 * norm_ang_30_free)
        
            # Extract index and plot the new velocity
            index_j = np.argmin(J)
            new_vel_xy = ([vel_space_free_xy[index_j, 0],
                    vel_space_free_xy[index_j, 1]])
            
            new_vel = self.vect_to_ang_mag(new_vel_xy)
            
            # if plotting:
            #     plt.quiver(0, 0, new_vel_xy[0], new_vel_xy[1], scale=1,
            #                 scale_units='xy', angles='xy', color='blue', zorder=6)
            #     plt.annotate('$V_{new}$', (new_vel_xy[0],new_vel_xy[1]), (new_vel_xy[0]+0,new_vel_xy[1]-1), zorder=6, c="blue")
                
        else:
            print("Death!")
            new_vel = self.latest_new_vel
            # new_vel = []

        return np.array(new_vel)

    def calc_new_vel_overtaking(self, vel_des, search_area, vel_OS):
        """ Function to calculate the new velocity for encounters, where COLREG
        constrains are applied """
        if np.any(search_area) and self.coll_safety == False:
            # Calculate new vel
            vel_space_free_xy = search_area
            vel_space_free_mag = np.sqrt(vel_space_free_xy[:,0] ** 2 + vel_space_free_xy[:,1] ** 2)
            
            # Cost function to choose new velocity
            in_arcos_des = (np.dot(vel_space_free_xy, self.vect_to_xy(vel_des))
                            )/(vel_space_free_mag*vel_des[0])
            in_arcos_des = np.round_(in_arcos_des, decimals=10)
            angles_des_free = np.rad2deg(np.arccos(in_arcos_des))
            
            speed_des_free = np.abs(vel_space_free_mag - vel_des[0])
           
            vel_30 = vel_OS.copy()
            in_arcos_30 = (np.dot(vel_space_free_xy, self.vect_to_xy(vel_30))
                            )/(vel_space_free_mag*vel_30[0])
            in_arcos_30 = np.round_(in_arcos_30, decimals=10)
            angles_30_free = np.rad2deg(np.arccos(in_arcos_30))
                                    
            angles_30_free = (30-angles_30_free)*-1
            angles_30_free[angles_30_free>0] = 0
            angles_30_free = -angles_30_free
            angles_30_free = np.round_(angles_30_free, decimals=8)

            # Normalize the values between 0-1, with 0 = 0 and 1 = max value
            norm_ang_des_free = angles_des_free / np.max(angles_des_free)
            norm_speed_des_free = speed_des_free / np.max(speed_des_free)
            if np.max(angles_30_free) == 0:
                norm_ang_30_free = angles_30_free
            else:
                norm_ang_30_free = angles_30_free / np.max(angles_30_free)
        
        
            # Cost function
            J = (VO.w_1 * norm_ang_des_free + VO.w_2 * norm_speed_des_free +
                VO.w_3 * norm_ang_30_free)
        
            # Extract index and plot the new velocity
            index_j = np.argmin(J)
            new_vel_xy = ([vel_space_free_xy[index_j, 0],
                    vel_space_free_xy[index_j, 1]])
            
            new_vel = self.vect_to_ang_mag(new_vel_xy)
            
            # if plotting:
            #     plt.quiver(0, 0, new_vel_xy[0], new_vel_xy[1], scale=1,
            #                 scale_units='xy', angles='xy', color='blue', zorder=6)
            #     plt.annotate('$V_{new}$', (new_vel_xy[0],new_vel_xy[1]), (new_vel_xy[0]+0,new_vel_xy[1]-1), zorder=6, c="blue")
                
        else:
            print("Death!")
            new_vel = self.latest_new_vel
            # new_vel = []

        return np.array(new_vel)

    def calc_new_vel(self, vel_des, search_area, vel_OS):
        """ Function to calculate the new velocity for encounters, where no COLREG
        constrains are applied """
        if np.any(search_area) and self.coll_safety == False:
            vel_space_free_xy = search_area
            vel_space_free_mag = np.sqrt(vel_space_free_xy[:,0] ** 2 + vel_space_free_xy[:,1] ** 2)
        
            in_arcos_des = (np.dot(vel_space_free_xy, self.vect_to_xy(vel_des))
                            )/(vel_space_free_mag*vel_des[0])
            in_arcos_des = np.round_(in_arcos_des, decimals=10)
            angles_des_free = np.rad2deg(np.arccos(in_arcos_des))
        
            speed_des_free = np.abs(vel_space_free_mag - vel_des[0])

            vel_30 = vel_OS.copy()
            vel_30[1] = (vel_30[1] + 30) % 360
            in_arcos_30 = (np.dot(vel_space_free_xy, self.vect_to_xy(vel_30))
                            )/(vel_space_free_mag*vel_30[0])
            in_arcos_30 = np.round_(in_arcos_30, decimals=10)
            angles_30_free = np.rad2deg(np.arccos(in_arcos_30))

            # Normalize the values between 0-1, with 0 = 0 and 1 = max value
            norm_ang_des_free = angles_des_free / np.max(angles_des_free)
            norm_speed_des_free = speed_des_free / np.max(speed_des_free)
            if np.max(angles_30_free) == 0:
                norm_ang_30_free = angles_30_free
            else:
                norm_ang_30_free = angles_30_free / np.max(angles_30_free)
        
            # Cost function
            J = (VO.w_1 * norm_ang_des_free + VO.w_2 *
                norm_speed_des_free + VO.w_3 * norm_ang_30_free)
        
            # Extract index and plot the new velocity
            index_j = np.argmin(J)
            new_vel_xy = ([vel_space_free_xy[index_j, 0],
                    vel_space_free_xy[index_j, 1]])
            new_vel = self.vect_to_ang_mag(new_vel_xy)
            # if plotting:
            #     plt.quiver(0, 0, new_vel_xy[0], new_vel_xy[1], scale=1,
            #                scale_units='xy', angles='xy', color='blue', zorder=4, width=0.003)
        else:
            print("Death")
            new_vel = self.latest_new_vel
            # new_vel = []
        return np.array(new_vel)

    def extract_most_common_vel(self, arr, threshold):
        """ Function to calculate the most common velocity"""
        distances = cdist(arr, arr, 'euclidean')
        similar_rows = np.where(distances <= threshold)
        most_common_index = mode(similar_rows[0]).mode[0]
        return arr[most_common_index]

    def calc_vel_final(self, ts_info, os_info, this, pos_os):
        global plotting
        plotting = this
        global pos_OS_xy
        pos_OS_xy = pos_os

        # if plotting:
        #     # Create a circle around the OS with the maximum speed of the OS --> all possible velocities of OS
        #     # Parametric equation circle
        #     c_angle = np.linspace(0, np.deg2rad(360), 150)
        #     x = self.max_speed_OS * np.cos(c_angle)
        #     y = self.max_speed_OS * np.sin(c_angle)
        #     # List of all circle points = c_points
        #     c_points = np.column_stack((x, y))
        #     c_points[:,0] = c_points[:,0] + pos_OS_xy[0]
        #     c_points[:,1] = c_points[:,1] + pos_OS_xy[1]
        #     plt.plot(c_points[:, 0], c_points[:, 1], 'black',zorder=6)
        #     plt.fill(c_points[:, 0], c_points[:, 1], 'lightgray',
        #               alpha=0.5, zorder=0, edgecolor='black', linewidth=1.5)
        
        vel_OS = os_info[0]
        global vel_OSxy
        vel_OSxy = os_info[1]
        ang_OS_rad = os_info[2]
        vel_des = os_info[3]
        vel_des_xy = self.vect_to_xy(vel_des)
        
        if plotting:
            plt.quiver(pos_OS_xy[0], pos_OS_xy[1], vel_des_xy[0], vel_des_xy[1],scale=1,
                        scale_units='xy', angles='xy', color='gray', zorder=6,width=0.005, hatch="||||", edgecolor="black", linewidth=0.5, label="v\u20D7 desired")
        global plotvar
        plotvar = 1     
        # for each TS --> calc VO with uncertainties
        TS_VO = np.empty((0, 9))
        for TSo in ts_info:
            VOs = self.calc_vel_obst(TSo, ang_OS_rad)
            plotvar += 1
            TSo = np.append(TSo, VOs)
            TS_VO = np.vstack((TS_VO, TSo))
                
        # for each VO --> check collision
        TS_VO_check = np.empty((0, 15))
        for TSv in TS_VO:
            vel_TS_ang = np.array([TSv[3], TSv[4]])
            vel_TS_xy = self.vect_to_xy(vel_TS_ang)
            vel_rela = self.calc_rel_vel(vel_OS, vel_TS_ang)
            vel_rela_xy = self.vect_to_xy(vel_rela)
            VO_check = self.check_collision(vel_OS, TSv[5])
            TSv = np.append(TSv, VO_check)
            
            # if check collision = TRUE --> check rule, calculate COLREG con, calc collision point (assigned to TS)
            if VO_check:
                # Save the inital velocity of OS the first time a collision is detected
                if self.flag:
                    self.vel_OS_init = vel_OS
                    self.flag = False

                # Check COLREG rule only if the last time the VO_check was false --> So that the COLREG rule does not change during the avoidance manouver, and so alter the course which results in chattering
                if self.ts_vo_checks[0,10] == None:
                    VO_rule = self.check_colreg_rule_heading(vel_OS, vel_TS_ang)
                    TSv = np.append(TSv, VO_rule)
                else:
                    TSv = np.append(TSv, self.ts_vo_checks[0,10])
                Col_con = self.calc_colreg_con(vel_OS, vel_TS_ang, vel_TS_xy, TSv[0])
                
                TSv = np.append(TSv, [0])
                TSv[11] = Col_con
                Coll = self.check_coll_point(
                    vel_rela_xy, vel_OS, vel_OSxy, vel_TS_ang, vel_TS_xy,
                    TSv[6], TSv[7])
                TSv = np.append(TSv, Coll)
            else:
                TSv = np.append(TSv, [None, None, None, None, None])
                self.flag = True
            TS_VO_check = np.vstack((TS_VO_check, TSv))
            self.ts_vo_checks = TS_VO_check
        ### TS_VO_check = (pos_TS_rel, len_TS, wid_TS, speed_TS, ang_TS, VO_vert, hull_vert, tang_points, points_colreg_lines, point_tipp, Check_coll, col_rule, col_con, point_coll,dist_coll, time_coll)
        
        # if np.any(TS_VO_check[:,9]):
        if True:    
            # Create the possible velocities to search a new vel from
            th = np.arange(0, 2*np.pi, np.deg2rad(self.res_ang))
            vel = np.arange(0.000000000001, self.max_speed_OS+self.res_speed, self.res_speed)
            vv, thth = np.meshgrid(vel, th)
            vx_sample = (vv * np.cos(thth)).flatten()
            vy_sample = (vv * np.sin(thth)).flatten()
            v_sample = np.column_stack((vx_sample, vy_sample))
            ### only if any point of the circle of the velocity is inisde the VO, use this VO fpr calculating the free vel space
            
            # Calculate free vel space --> all velocities - VOs - COLREG cons
            free_vel, in_all = self.calc_free_vel_space(TS_VO_check[:,5], v_sample)
            free_vel, col_con = self.calc_free_vel_space(TS_VO_check[:,11], free_vel)
            # if np.any(free_vel) and plotting:
            #     plt.scatter(free_vel[:,0]+pos_OS_xy[0],free_vel[:,1]+pos_OS_xy[1], marker=".", s = 2, c="green", zorder=5)
            #     plt.scatter(in_all[:,0]+pos_OS_xy[0],in_all[:,1]+pos_OS_xy[1], marker=".", s = 2, c="red",zorder=5)
            #     plt.scatter(col_con[:,0]+pos_OS_xy[0],col_con[:,1]+pos_OS_xy[1], marker=".", s = 2, c="purple",zorder=5)
                # plt.annotate("Discretized velocity space", xy=(0, -15), xycoords='data', xytext=(-102,-25), textcoords='offset points', arrowprops=dict(arrowstyle="->", connectionstyle="angle3,angleA=0,angleB=90"),zorder=6)
                
                # ## polar plot for display
                # free_vel_1 = np.apply_along_axis(self.vect_to_ang_mag, 1, free_vel)
                # in_all_1 = np.apply_along_axis(self.vect_to_ang_mag, 1, in_all)
                # col_con_1 = np.apply_along_axis(self.vect_to_ang_mag, 1, col_con)
                # plt.rcParams["figure.figsize"] = [7.00, 3.50]
                # plt.rcParams["figure.autolayout"] = True
                # ax = plt.subplot(1, 1, 1, projection='polar')
                # ax.set_theta_direction(-1)
                # ax.set_theta_offset(np.pi / 2.0)
                # ax.scatter(np.deg2rad(free_vel_1[:,1]), free_vel_1[:,0], c="green", s=1, zorder=5)
                # ax.scatter(np.deg2rad(in_all_1[:,1]), in_all_1[:,0], c="red",s=1,zorder=5)
                # ax.scatter(np.deg2rad(col_con_1[:,1]), col_con_1[:,0], c="purple",s=1,zorder=5)
    
            # if all velocities would lead to a collision, expand the free vel space by the colreg constrains, so that a colreg non-compliant action is possible
            if not np.any(free_vel):
                free_vel = col_con        
        # Calculate new velocities for each VO the OS vel is colliding with
        new_vel_testo = np.empty((0, 2))
        for TS_vel in TS_VO_check:
            # for each TS with collision and COLREG rules where contrains have to be applied, calculate the new velocity in free velocity space
            if TS_vel[10] == 'Right crossing (Rule 15)' or TS_vel[10] == 'Head-on (Rule 14)' or TS_vel[10] == 'Overtaking (Rule 13)':

                # Calculate new velocity
                if TS_vel[10] == 'Overtaking (Rule 13)':
                    new_vel = self.calc_new_vel_overtaking(vel_des, free_vel, self.vel_OS_init)
                else:
                    new_vel = self.calc_new_vel(vel_des, free_vel, self.vel_OS_init)
                
                if np.any(new_vel):
                    new_vel_testo = np.vstack((new_vel_testo, new_vel, new_vel))

                else:
                    new_vel_testo = np.vstack((new_vel_testo, np.empty((0,2))))
            # for each TS with collision and no COLREG constrains have to be applied, calculate the new velocity in free velocity space
            elif TS_vel[10] == 'Left crossing (Rule 15)' or TS_vel[10] == 'Being Overtaken (Rule 13)':
                
                if TS_vel[14] <= self.threshold:
                    new_vel = self.calc_new_vel(vel_des, free_vel, self.vel_OS_init)
                    if np.any(new_vel):
                        new_vel_testo = np.vstack((new_vel_testo, new_vel))

                    else:
                        new_vel_testo = np.vstack((new_vel_testo, np.empty((0,2))))
                else:
                    new_vel_testo = np.vstack((new_vel_testo, np.empty((0,2))))
            elif TS_vel[10] == 'Static object':
                new_vel = self.calc_new_vel_overtaking(vel_des, free_vel, self.vel_OS_init)
                if np.any(new_vel):
                    new_vel_testo = np.vstack((new_vel_testo, new_vel))

                else:
                    new_vel_testo = np.vstack((new_vel_testo, np.empty((0,2))))

        # Extract the final new velocity
        if np.any(new_vel_testo):
            is_inside = False
            for vel_obs in TS_VO_check[:,5]:
                if self.check_collision(vel_des, vel_obs):
                    
                    is_inside = True
            if not is_inside:
                new_vel_final = vel_des.copy()
            else:
                new_vel_final = self.extract_most_common_vel(new_vel_testo, 0)
        #### what to do if no new_vel is there? Stop or drive on with the current speed?    
        else:
            is_inside = False
            for vel_obs in TS_VO_check[:,5]:
                if self.check_collision(vel_des, vel_obs):
                    
                    is_inside = True
            if not is_inside:
                new_vel_final = vel_des.copy()
            elif not np.any(self.latest_new_vel):
                new_vel_final = vel_des.copy()
            else:
                new_vel_final = self.latest_new_vel
                
        self.latest_new_vel = new_vel_final
        return new_vel_final
