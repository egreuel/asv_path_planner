from collections import Counter
from time import perf_counter_ns
from shapely.validation import make_valid

import numpy as np
import matplotlib.pyplot as plt
import math
import torch

from shapely.geometry import Point, MultiPoint, Polygon, MultiPolygon, LineString
# from shapely.ops import transform
from geopy import distance
from scipy.spatial import ConvexHull
from scipy.spatial.distance import cdist
from scipy.stats import mode

# input: speed_OS[m/s], ang_OS[°], pos_OS[Lat,Lon], speed_TS[m/s], ang_TS[°],
#        pos_TS[Lat,Lon], len_OS[m], wid_OS[m], len_TS[m], wid_TS[m]
# output: VO, absolute course

# Plotting
plotting = False
# Debug

# input values
# either as GPS coordinates or realtive to OS from sensor measurments
# pos_OS = np.array([43.934530, 15.441278])
# ang_OS = 340  # °
# speed_OS = 6  # m/s
len_OS = 0.5  # Length OS in m
wid_OS = 0.2  # Width OS in m

# ang_TS = 235  # °
# speed_TS = 10  # m/s
# len_TS = 10  # Length TS in m
# wid_TS = 2  # Width TS in m

max_TTC = 5  # Time to collison in s
safe_Fact = 3
max_speed_OS = 3
# Uncertainty handling
unc_speed = 0.2  # m/s
unc_angle = 3  # degrees
# Weights for the cost-function to choose the new velocity
w_1 = 1  # Angle deviation from desired velocity
w_2 = 2  # Speed deviation from desried velocity
w_3 = 1.5  # Angle deviation from 30°
w_4 = 0.1  # Angle deviation from vector to target position
# Resolution of the discretized velocity space
n = 0.25  # m/s
m = 5  # °

threshold = 3  # time to collision threshold in s for standby actions

# @profile
def calc_coord_gps_to_xy(coord_os, coord_ts):
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


def calc_abs_vel(ref_vel, targ_vel):
    """ Function to calculate the absolute velocity of the TS if only the
    relative velocity of TS is given;
    Returns the absolute velocity of the target ship with speed and angle
    """
    # x and y componet of the vector from magnitude(speed) and direction(angle)
    ref_velxy = vect_to_xy(ref_vel)
    vel_Relxy = vect_to_xy(targ_vel)
    # Calculate the TS vector in component form
    vel_TSxy_calc = vel_Relxy + ref_velxy
    # Calculate the magnitude(speed) of the vector
    speed_TS_calc = np.sqrt(vel_TSxy_calc[0] ** 2 + vel_TSxy_calc[1] ** 2)
    # Calculate the dircetion(angle) of the vector
    ang_TS_calc = np.degrees(np.arctan2(vel_TSxy_calc[1], vel_TSxy_calc[0]))
    ang_TS_calc = (ang_TS_calc+360) % 360
    # convert the angle from east CCW to north CW
    ang_TS_calc = calc_ang_n_to_e(ang_TS_calc)
    vel_TS_calc = np.array([speed_TS_calc, ang_TS_calc])
    return np.array(vel_TS_calc)


def calc_rel_vel(vel_OS, vel_TS):
    """ Function to calculate the relative velocity of OS and TS """
    # x and y componet of the vector from magnitude(speed) and direction(angle)
    vel_OSxy = vect_to_xy(vel_OS)
    vel_TSxy = vect_to_xy(vel_TS)
    # Calculate the relative vector in component form
    vel_Relxy = vel_OSxy - vel_TSxy
    # Calculate the magnitude(speed) of the vector
    speed_Rel = np.sqrt(vel_Relxy[0] ** 2 + vel_Relxy[1] ** 2)
    # Calculate the dircetion(angle) of the vector
    # Add special cases for rel Velocity speed = 0
    ang_Rel = np.degrees(np.arctan2(vel_Relxy[1], vel_Relxy[0]))
    ang_Rel = (ang_Rel+360) % 360
    # convert the angle from east CCW to north CW
    ang_Rel = calc_ang_n_to_e(ang_Rel)
    vel_Rel = np.array([speed_Rel, ang_Rel])
    return np.array(vel_Rel)


def vect_to_ang_mag(vect_xy):
    """ Function that calculates the magnitude and direction of a vector given
    in x/y-form """
    speed = np.sqrt(vect_xy[0] ** 2 + vect_xy[1] ** 2)
    angle = math.degrees(np.arctan2(vect_xy[1], vect_xy[0]))
    angle = (angle+360) % 360
    angle = calc_ang_n_to_e(angle)
    vector = ([speed, angle])
    return np.array(vector)

def vect_to_xy(vector):
    """ Function that calculates the x/y-form of a vector given in
    magnitude/direction-form """
    vect_xy = ([vector[0] * math.cos(math.radians(calc_ang_n_to_e(vector[1]))),
                        vector[0] * math.sin(math.radians(calc_ang_n_to_e(vector[1])))])
    
    # slower:
    # vect_xy = ([vector[:,0] * np.cos(np.radians(calc_ang_n_to_e(vector[:,1]))),
    #                     vector[:,0] * np.sin(np.radians(calc_ang_n_to_e(vector[:,1])))])
    return np.array(vect_xy)

def calc_ang_n_to_e(angle):
    """ Function that converts the angle from the north clockwise system to the
    east counter clockwise system and vice versa """
    alpha = (-angle + 90) % 360
    return alpha

def calc_minkowski_sum(vert_TS, vert_OS):
    """ Function of the minkoswski sum """
    result = []
    for x in vert_TS:
        for y in vert_OS:
            result.append((x[0] + y[0], x[1] + y[1]))
    return np.array(result)

def check_collision(vel_OS, vel_obs):
    """ Function Check for Collision (Shaply is used here) """
    if np.any(vel_obs):
        vel_OS_xy = vect_to_xy(vel_OS)
        point = Point(vel_OS_xy[0], vel_OS_xy[1])
        points = vel_obs.tolist()
        polygon = Polygon(points)
        # if Polygon.is_simple:
        #     print('simple Polygon')
        # else:
        #     print('complex Polygon')
        collision = point.within(polygon)
    else:
        collision = False
    return collision


def calc_ang_vect(vel_OS, vel_Rel):
    """ Function to calulate the angle between the two velocity vectors """
    phi = ((vel_Rel[1]-vel_OS[1])+360) % 360
    return phi


def check_colreg_rule(vel_OS, pos_TS_rel):
    """ Function Check COLREG rule (relative bearing) """
    phi = calc_ang_vect(vel_OS, vect_to_ang_mag(pos_TS_rel))
    if 15 <= phi <= 112.5:
        colreg_rule = 'Right crossing (Rule 15)'
    elif 112.5 < phi <= 247.5:
        colreg_rule = 'Being Overtaken (Rule 13)'
    elif 247.5 < phi <= 345:
        colreg_rule = 'Left crossing (Rule 15'
    else:
        colreg_rule = 'Head-on (Rule 14)'
    return colreg_rule


def check_colreg_rule_heading(vel_OS_2, vel_TS_2):
    """ Function to check COLREG rule (angle between vel_OS and
    vel_TS heading) """
    vel = vel_TS_2.copy()
    vel[1] = vel_TS_2[1] - 180
    if vel[0] <= 0:  # Threshhold for static objects
        colreg_rule_2 = 'Static Object, no COLREGS applied'
    else:
        phi_2 = calc_ang_vect(vel_OS_2, vel)
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


def check_side(vel_Rel, tang_coord):
    """ Function that checks wether the relative velocity is inside the CC or
    outside, if outside check which side """
    side = 'empty or inside'

    tang_points_vec_1 = vect_to_ang_mag(tang_coord[0, :])
    tang_points_vec_2 = vect_to_ang_mag(tang_coord[1, :])

    angles_betw = ang_betw_vect(vel_Rel, tang_points_vec_1, tang_points_vec_2)
    angles_betw_tang = ang_betw_vect(tang_points_vec_1, tang_points_vec_2)

    if angles_betw[0] > angles_betw_tang:
        side = 'right'
    elif angles_betw[1] > angles_betw_tang:
        side = 'left'

    return side


def ang_betw_vect(ref_vect, *test_vect):
    """ Function to calculate the angle between two vectors """
    ref_vect_xy = vect_to_xy(ref_vect)
    test_vect_xy = np.apply_along_axis(vect_to_xy, 1, test_vect)
    test_vect = np.apply_along_axis(vect_to_ang_mag, 1, test_vect_xy)
    in_arcos = (np.dot(test_vect_xy, ref_vect_xy)) / (test_vect[:, 0]*ref_vect[0])
    in_arcos = np.round_(in_arcos, decimals=10)
    angles_betw = np.rad2deg(np.arccos(in_arcos))

    return np.array(angles_betw)

# @profile
def calc_vel_obst(TS, ang_os_rad):
    """ Function to calculate the VO, gives back an array with the vertices of
    the VO """
    pos_TS_rel = TS[0]
    len_TS = TS[1]
    wid_TS = TS[2]
    speed_TS = TS[3]
    ang_TS = TS[4]
    vel_TS = np.array([speed_TS, ang_TS])
    # vel_Rel = calc_rel_vel(vel_OS, vel_TS)
    # vel_Relxy = vect_to_xy(vel_Rel)
    # Geodesic Distance (ellipsoidal model of the earth) for GPS coordinates
    # dist_TSOS = distance.distance(pos_TS, pos_OS)
    # print('Distance =', dist_TSOS.meters, 'm')

    # Convert angle from degree to radians
    ang_TS_rad = math.radians(ang_TS)
    ang_OS_rad = ang_os_rad
    # Calculate the xy-form of the velocity vector
    vel_TSxy = vect_to_xy(vel_TS)

    # Calclate the relative position of the TS in meters
    # pos_TS_rel = calc_coord_gps_to_xy(pos_OS, pos_TS)

    # Calculate the vertices of the OS around it
    vert_OS = np.array([[-0.5*wid_OS, -0.5*len_OS],
                        [0.5*wid_OS, -0.5*len_OS],
                        [0.5*wid_OS, 0.5*len_OS],
                        [-0.5*wid_OS, 0.5*len_OS]])

    # Rotate the vertices in the direction the OS is facing
    rot_M_OS = np.array([[np.cos(ang_OS_rad), -np.sin(ang_OS_rad)],
                         [np.sin(ang_OS_rad), np.cos(ang_OS_rad)]])
   
    # vert_OS = np.dot(vert_OS,rot_M_OS)
    vert_OS = torch.matmul(torch.from_numpy(vert_OS), torch.from_numpy(rot_M_OS))
    vert_OS = np.array(vert_OS)
    # add safety factor for the OS
    vert_OS_safe = vert_OS*safe_Fact

    # Create a convex hull obejct of the OS vertices
    hullOS = ConvexHull(vert_OS)
    # Plot the OS shape
    if plotting:
        plt.fill(vert_OS[hullOS.vertices, 0], vert_OS[hullOS.vertices,
                 1], 'blue', alpha=1, zorder=0, edgecolor='black')

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
    exp_vert_TS = calc_minkowski_sum(vert_TS, vert_OS)  # (Just for plotting)
    exp_vert_TS_safe = calc_minkowski_sum(vert_TS, vert_OS_safe)

    # # plot the vertices of the OS
    # plt.scatter(vert_OS[:,0], vert_OS[:,1], marker='.', linewidths=(0.1))

    # Convex Hull function of scipy library to create a hull of the new points of the minkowski sum
    
    # Expanded TS shape by OS (just for plotting)
    hull_exp = ConvexHull(exp_vert_TS)
    hull_safe = ConvexHull(exp_vert_TS_safe)

    # Plot the Polygons formed by the hull as filled polygons
    if plotting:
        hullTS = ConvexHull(vert_TS)  # Shape of TS (just for plotting)
        plt.fill(exp_vert_TS_safe[hull_safe.vertices, 0],
                 exp_vert_TS_safe[hull_safe.vertices, 1], 'yellow',
                 alpha=1, zorder=0, edgecolor='black')
        plt.fill(exp_vert_TS[hull_exp.vertices, 0],
                 exp_vert_TS[hull_exp.vertices, 1], 'orange',
                 alpha=1, zorder=0, edgecolor='black')
        plt.fill(vert_TS[hullTS.vertices, 0], vert_TS[hullTS.vertices, 1],
                 'red', alpha=1, zorder=0, edgecolor='black')

     # Plot the position of OS and TS / optional vertices of the poylgon
    if plotting:
        # plt.scatter(vert_hull[:,0], vert_hull[:,1], marker='.', linewidths=(0.1))
        # Plot the origin point of OS and TS
        plt.scatter(0, 0, marker='.', linewidths=(0.1))
        plt.scatter(pos_TS_rel[0], pos_TS_rel[1], marker='.', linewidths=(0.1))

    # Extract the vertices of the polygon from the hull object
    for x in hull_safe.vertices:
        vert_hull.append(hull_safe.points[x])
    
    vert_hull = np.array(vert_hull)
    vert_hull = np.resize(vert_hull, (hull_safe.vertices.shape[0], 2))
    
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
    tang_points_shift = tang_points.copy()
    tang_points_shift[:, 0] = tang_points_shift[:, 0] + vel_TSxy[0]
    tang_points_shift[:, 1] = tang_points_shift[:, 1] + vel_TSxy[1]

    # Calulate the shifted vertices by TTC
    vert_hull_TTC = vert_hull_facing/max_TTC

    # Plot the minimum speed shifted with the velocity of TS
    vert_hull_TTC_shifted = vert_hull_TTC.copy()
    vert_hull_TTC_shifted[:, 0] = vert_hull_TTC_shifted[:, 0] + vel_TSxy[0]
    vert_hull_TTC_shifted[:, 1] = vert_hull_TTC_shifted[:, 1] + vel_TSxy[1]

    # Calculate all points if the vertices TTC _shifted with the circle

    ##### Form the velocity obstacle out of the intersections with the circle and the lines formed by the shifted tangent vertices + all points of the circle inbetween #####
    # Line equation for points between shifted tangent points and shifted origin
    # line of the side the COLREG contrains has to be on
    x_p1 = vel_TSxy[0]
    y_p1 = vel_TSxy[1]
    x_p2 = tang_points[0, 0] + vel_TSxy[0]  # left tangent point
    y_p2 = tang_points[0, 1] + vel_TSxy[1]  # left tangent point
    x_p3 = tang_points[1, 0] + vel_TSxy[0]  # right tangent point
    y_p3 = tang_points[1, 1] + vel_TSxy[1]  # right tangent point
    m_1 = (y_p2-y_p1)/(x_p2-x_p1)  # left line seen from OS
    b_1_VO = y_p1 - m_1*x_p1
    m_2 = (y_p3-y_p1)/(x_p3-x_p1)  # right line seen from OS
    b_2_VO = y_p1 - m_2*x_p1

    # ### Here add uncertainties!!!
    vert_VO_testo = np.vstack((tang_points_shift[0, :],
                               vert_hull_TTC_shifted, tang_points_shift[1, :]))
    vert_VO_testo_unc = add_uncert(unc_speed, unc_angle,
                                   vert_VO_testo, pos_TS_rel, vel_TS)

    # 
    b_l = vert_VO_testo_unc[:, 1] - m_1 * vert_VO_testo_unc[:, 0]
    counts = Counter(np.round(b_l, 10))
    most_common = counts.most_common(1)
    b_1 = most_common[0][0]
    # find index of the value b_1 in b_l
    
    b_r = vert_VO_testo_unc[:, 1] - m_2 * vert_VO_testo_unc[:, 0]
    counts = Counter(np.round(b_r, 10))
    most_common = counts.most_common(1)
    b_2 = most_common[0][0]

    # find index of the value b_2 in b_r
    index_l = np.where(np.round(b_l, 10) == b_1)[0]
    index_r = np.where(np.round(b_r, 10) == b_2)[0]

    # Calculate distance of the two each indices, take the one with the furthest distance --> not the tangepoint with unc but the one closer to OS
    distance_i_l = []
    distance_i_r = []
    for x in index_l:
        cache = np.sqrt((pos_TS_rel[1]-vert_VO_testo_unc[x, 1])**2
                        + (pos_TS_rel[0]-vert_VO_testo_unc[x, 0])**2)
        distance_i_l.append(cache)
    for x in index_r:
        cache = np.sqrt((pos_TS_rel[1]-vert_VO_testo_unc[x, 1])**2
                        + (pos_TS_rel[0]-vert_VO_testo_unc[x, 0])**2)
        distance_i_r.append(cache)

    distance_i_l = np.array(distance_i_l)
    distance_i_r = np.array(distance_i_r)
    index_p_l = index_l[np.argmax(distance_i_l)]
    index_p_r = index_r[np.argmax(distance_i_r)]

    if index_p_l > index_p_r:
        vert_hull_unc = np.vstack(
            (vert_VO_testo_unc[index_p_l:], vert_VO_testo_unc[:index_p_r+1]))
    else:
        vert_hull_unc = vert_VO_testo_unc[index_p_l:index_p_r+1]

    # Extract the unc vertices
    # take the two points of the b and calculate the distance to TS. Take the further point. Then extract all points between these two points. Depending on where they are in the array either delete or take all.

    # Calculate the discriminent D for both lines with the circle
    # VO unc
    a_x = 1+(1/m_1**2)
    b_x = -2*(1/m_1)*(b_1/m_1)
    c_x = (b_1**2/m_1**2) - max_speed_OS**2
    d_1 = b_x**2 - (4*a_x*c_x)
    a_y = 1+(1/m_2**2)
    b_y = -2*(1/m_2)*(b_2/m_2)
    c_y = (b_2**2/m_2**2) - max_speed_OS**2
    d_2 = b_y**2 - (4*a_y*c_y)

    # VO without unc
    b_x_VO = -2*(1/m_1)*(b_1_VO/m_1)
    c_x_VO = (b_1_VO**2/m_1**2) - max_speed_OS**2
    d_1_VO = b_x_VO**2 - (4*a_x*c_x_VO)
    b_y_VO = -2*(1/m_2)*(b_2_VO/m_2)
    c_y_VO = (b_2_VO**2/m_2**2) - max_speed_OS**2
    d_2_VO = b_y_VO**2 - (4*a_y*c_y_VO)

    # Calculate the intersection points
    # Take only the two intersection points close to TS (point_left, point_right)
    if d_1 > 0:
        inter_y_1_l = (-b_x + np.sqrt(d_1))/(2*a_x)
        inter_y_2_l = (-b_x - np.sqrt(d_1))/(2*a_x)
        inter_x_1_l = (inter_y_1_l - b_1)/m_1
        inter_x_2_l = (inter_y_2_l - b_1)/m_1

        int_point_1_l = np.array([[inter_x_1_l, inter_y_1_l]])
        int_point_2_l = np.array([[inter_x_2_l, inter_y_2_l]])

        # Calculate distance of TS to the intersection points
        dist_p1_l = np.sqrt((pos_TS_rel[1]-int_point_1_l[0, 1])**2
                            + (pos_TS_rel[0]-int_point_1_l[0, 0])**2)
        dist_p2_l = np.sqrt((pos_TS_rel[1]-int_point_2_l[0, 1])**2
                            + (pos_TS_rel[0]-int_point_2_l[0, 0])**2)

        if dist_p1_l < dist_p2_l:
            point_left = int_point_1_l
        else:
            point_left = int_point_2_l

    if d_2 > 0:
        inter_y_1_r = (-b_y + np.sqrt(d_2))/(2*a_y)
        inter_y_2_r = (-b_y - np.sqrt(d_2))/(2*a_y)
        inter_x_1_r = (inter_y_1_r - b_2)/m_2
        inter_x_2_r = (inter_y_2_r - b_2)/m_2

        int_point_1_r = np.array([[inter_x_1_r, inter_y_1_r]])
        int_point_2_r = np.array([[inter_x_2_r, inter_y_2_r]])

        # Calculate distance of TS to the intersection points
        dist_p1_r = np.sqrt((pos_TS_rel[1]-int_point_1_r[0, 1])**2
                            + (pos_TS_rel[0]-int_point_1_r[0, 0])**2)
        dist_p2_r = np.sqrt((pos_TS_rel[1]-int_point_2_r[0, 1])**2
                            + (pos_TS_rel[0]-int_point_2_r[0, 0])**2)

        if dist_p1_r < dist_p2_r:
            point_right = int_point_1_r
        else:
            point_right = int_point_2_r

    # Calculate the intersection points
    # Take only the two intersection points close to TS (point_left, point_right)
    if d_1_VO > 0:
        inter_y_1_l_VO = (-b_x_VO + np.sqrt(d_1_VO))/(2*a_x)
        inter_y_2_l_VO = (-b_x_VO - np.sqrt(d_1_VO))/(2*a_x)
        inter_x_1_l_VO = (inter_y_1_l_VO - b_1_VO)/m_1
        inter_x_2_l_VO = (inter_y_2_l_VO - b_1_VO)/m_1

        int_point_1_l_VO = np.array([[inter_x_1_l_VO, inter_y_1_l_VO]])
        int_point_2_l_VO = np.array([[inter_x_2_l_VO, inter_y_2_l_VO]])

        # Calculate distance of TS to the intersection points
        dist_p1_l_VO = np.sqrt((pos_TS_rel[1]-int_point_1_l_VO[0, 1])**2
                               + (pos_TS_rel[0]-int_point_1_l_VO[0, 0])**2)
        dist_p2_l_VO = np.sqrt((pos_TS_rel[1]-int_point_2_l_VO[0, 1])**2
                               + (pos_TS_rel[0]-int_point_2_l_VO[0, 0])**2)

        if dist_p1_l_VO < dist_p2_l_VO:
            point_left_VO = int_point_1_l_VO
        else:
            point_left_VO = int_point_2_l_VO

    if d_2_VO > 0:
        inter_y_1_r_VO = (-b_y_VO + np.sqrt(d_2_VO))/(2*a_y)
        inter_y_2_r_VO = (-b_y_VO - np.sqrt(d_2_VO))/(2*a_y)
        inter_x_1_r_VO = (inter_y_1_r_VO - b_2_VO)/m_2
        inter_x_2_r_VO = (inter_y_2_r_VO - b_2_VO)/m_2

        int_point_1_r_VO = np.array([[inter_x_1_r_VO, inter_y_1_r_VO]])
        int_point_2_r_VO = np.array([[inter_x_2_r_VO, inter_y_2_r_VO]])

        # Calculate distance of TS to the intersection points
        dist_p1_r_VO = np.sqrt((pos_TS_rel[1]-int_point_1_r_VO[0, 1])**2
                               + (pos_TS_rel[0]-int_point_1_r_VO[0, 0])**2)
        dist_p2_r_VO = np.sqrt((pos_TS_rel[1]-int_point_2_r_VO[0, 1])**2
                               + (pos_TS_rel[0]-int_point_2_r_VO[0, 0])**2)

        if dist_p1_r_VO < dist_p2_r_VO:
            point_right_VO = int_point_1_r_VO
        else:
            point_right_VO = int_point_2_r_VO

    # Build polygon out of the circle points
    c_points_poly = c_points.tolist()
    poly_c = Polygon(c_points_poly)

    # Get all points inside the circle
    vert_TTC_shif = vert_hull_TTC_shifted.tolist()
    poly_vert_TTC_shif = LineString(vert_TTC_shif)
    int_c_poly_vert_shift = poly_c.intersection(poly_vert_TTC_shif)
    int_c_poly_vert_shift = np.array(int_c_poly_vert_shift.coords)

    if d_1_VO > 0 and d_2_VO > 0 and np.any(int_c_poly_vert_shift):
        vert_VO = np.vstack(
            (point_left_VO, (vel_TSxy[0], vel_TSxy[1]), point_right_VO))
        vert_VO_TTC = np.vstack(
            (point_left_VO, int_c_poly_vert_shift, point_right_VO))
    elif d_1_VO > 0 and d_2_VO <= 0 and np.any(int_c_poly_vert_shift):
        vert_VO = np.vstack((point_left_VO, (vel_TSxy[0], vel_TSxy[1])))
        vert_VO_TTC = np.vstack((point_left_VO, int_c_poly_vert_shift))
    elif d_1_VO <= 0 and d_2_VO > 0 and np.any(int_c_poly_vert_shift):
        vert_VO = np.vstack(((vel_TSxy[0], vel_TSxy[1]), point_right_VO))
        vert_VO_TTC = np.vstack((int_c_poly_vert_shift, point_right_VO))
    else:
        vert_VO = []
        vert_VO_TTC = []

    if d_1 > 0 and d_2 > 0:
        # vert_VO = np.vstack((point_left, (vel_TSxy[0],vel_TSxy[1]), point_right))
        vert_VO_TTC_unc = np.vstack((point_left, vert_hull_unc, point_right))
    elif d_1 > 0 and d_2 <= 0:
        # vert_VO = np.vstack((point_left, (vel_TSxy[0],vel_TSxy[1])))
        vert_VO_TTC_unc = np.vstack((point_left, vert_hull_unc))
    elif d_1 <= 0 and d_2 > 0:
        # vert_VO = np.vstack(((vel_TSxy[0],vel_TSxy[1]), point_right))
        vert_VO_TTC_unc = np.vstack((vert_hull_unc, point_right))
    else:
        # vert_VO = []
        vert_VO_TTC_unc = []

    # Make a shapely LineString out of it
    line_seg_VO = LineString(vert_VO_TTC_unc)
    # Intersecton of linestring and circle
    inters_VO_c = line_seg_VO.intersection(poly_c)
    # Extract the coordinates of the result
    points_inters = np.empty((0, 2))
    if inters_VO_c.is_empty is True:
        vel_obst_TTC_unc = []
        tang_unc_points = []
    else:
        if inters_VO_c.geom_type == "LineString":
            points_inters = np.vstack([points_inters, (inters_VO_c.coords)])

        if inters_VO_c.geom_type == "MultiLineString":
            for line in inters_VO_c.geoms:
                points_inters = np.vstack([points_inters, (line.coords)])

        # Add circle points to the points_intersection
        # Calculate the distance between the circle coordinates and the point coordinates
        dist_r = cdist(points_inters[-1].reshape(1, -1), c_points)
        dist_l = cdist(points_inters[0].reshape(1, -1), c_points)

        # find the index, where the distance is the smallest
        index_r_1st = np.argmin(dist_r)
        index_l_1st = np.argmin(dist_l)

        # Sort the distances and find the index of the second smallest distance
        sorted_indices_r = np.argsort(dist_r)
        index_r_2nd = sorted_indices_r[0, 1]

        # Sort the distances and find the index of the second smallest distance
        sorted_indices_l = np.argsort(dist_l)
        index_l_2nd = sorted_indices_l[0, 1]

        # take bigger index between 1st and 2nd of clos and smaller index of 1st and 2ns od furt
        if index_r_2nd > index_r_1st:
            index_r = index_r_2nd
        else:
            index_r = index_r_1st

        if index_l_2nd < index_l_1st:
            index_l = index_l_2nd
        else:
            index_l = index_l_1st

        # If Index closest < index furthest --> take all points inbetween
        # Else Index closest > index furthest --> take all execpt the ones inbetween
        if index_r < index_l:
            c_points_new = c_points[index_r:index_l+1]
        else:
            c_points_new = np.delete(
                c_points, np.s_[index_l+1:index_r], axis=0)
            # Change the order of the coordinates
            c_points_new = np.concatenate(
                [c_points_new[index_l+1:], c_points_new[:index_l+1]])

        # New velocity obstacle only inside circle
        vel_obst_TTC_unc = np.vstack(
            (points_inters, c_points_new))  # with time contrains

        if np.any(vert_VO):
            # without time contrains (just for plotting)
            vel_obst_new = np.vstack((vert_VO, c_points_new))
            # if plotting:
            #     # Plot VO inside circle without TTC
            #     plt.fill(vel_obst_new[:,0],vel_obst_new[:,1],'orange', alpha=0.25 ,zorder=1, edgecolor='black', linewidth=1.5 )
        else:
            vel_obst_new = []
        if np.any(vert_VO_TTC):
            vel_obst_TTC = np.vstack((vert_VO_TTC, c_points_new))
            if plotting:
                # Plot VO inside circle with TTC
                plt.fill(vel_obst_TTC[:, 0], vel_obst_TTC[:, 1], 'orange',
                         alpha=0.75, zorder=1, edgecolor='black',
                         linewidth=1.5)
        else:
            vel_obst_TTC = []

        tang_unc_points = np.vstack(
            (points_inters[0], vert_VO_testo_unc[index_p_l]))

        # Draw shifted lines
        if plotting:
            # Plot VO inside circle with TTC and uncertainties
            plt.fill(vel_obst_TTC_unc[:, 0], vel_obst_TTC_unc[:, 1], 'red',
                     alpha=1, zorder=0.5, edgecolor='black', linewidth=1.5)
            # plt.text(0.5*pos_TS_rel[0]+vel_TSxy[0],0.5*pos_TS_rel[1]+vel_TSxy[1],'VO', zorder=2,fontweight=550)

        # # Plot the lines that form the CC
        # if plotting:
        #     plt.plot([0,vert_hull[np.int_(tang_vert[1]),0]],[0,vert_hull[np.int_(tang_vert[1]),1]],'black',zorder=0)
        #     plt.plot([0,vert_hull[np.int_(tang_vert[2]),0]],[0,vert_hull[np.int_(tang_vert[2]),1]],'black',zorder=0)
        #     # Plot the area between the origin point and the two vertices the lines are connected to
        #     plt.fill([0,vert_hull[np.int_(tang_vert[1]),0],vert_hull[np.int_(tang_vert[2]),0]],[0,vert_hull[np.int_(tang_vert[1]),1],vert_hull[np.int_(tang_vert[2]),1]],"gray", alpha = 0.5,zorder=0)
        #     # plt.text(0.5*pos_TS_rel[0],0.5*pos_TS_rel[1],'CC', zorder=0,fontweight=550)

        # Plot the connecting lines between the coordinates of the shifted facing vertices due to time contrains
        # if plotting:
        #     plt.plot(vert_hull_TTC[:,0], vert_hull_TTC[:,1], 'black',zorder=0)

    # Plot the veloctiy vector
    # if plotting:
    #     # plt.quiver(0, 0, vel_Relxy[0], vel_Relxy[1], scale=1, scale_units='xy', angles='xy', color='purple', zorder=2)
    #     plt.quiver(0, 0, vel_OSxy[0], vel_OSxy[1], scale=1,
    #                scale_units='xy', angles='xy', color='black', zorder=2)
    #     plt.quiver(pos_TS_rel[0], pos_TS_rel[1], vel_TSxy[0], vel_TSxy[1],
    #                scale=1, scale_units='xy', angles='xy', color='black',
    #                zorder=2)
        # plt.annotate('OS', (0,0), (len_OS-0.5,-0.5))
        # plt.annotate('TS', (pos_TS_rel[0],pos_TS_rel[1]), (pos_TS_rel[0]+0.5,pos_TS_rel[1]-0.8))
        # plt.annotate('$V_{OS}$', (vel_OSxy[0],vel_OSxy[1]), (vel_OSxy[0],vel_OSxy[1]))
        # plt.annotate('$V_{OS}-V_{TS}$', (vel_Relxy[0],vel_Relxy[1]), (vel_Relxy[0],vel_Relxy[1]))
        # plt.annotate('$V_{TS}$', (vel_TSxy[0]+pos_TS_rel[0],vel_TSxy[1]+pos_TS_rel[1]), ((vel_TSxy[0]+pos_TS_rel[0]),(vel_TSxy[1]+pos_TS_rel[1]+1)))
        # plt.annotate('VO with \nuncertain- \nties in $V_{TS}$', xy=(-5, 18), xycoords='data', xytext=(-60, 40), textcoords='offset points', arrowprops=dict(arrowstyle="->", connectionstyle="angle3,angleA=0,angleB=-90"))
        # plt.annotate('TS expanded \nwith OS', xy=(30, 27), xycoords='data', xytext=(20,-5), textcoords='offset points', arrowprops=dict(arrowstyle="->", connectionstyle="angle3,angleA=0,angleB=140"))
        # plt.annotate('variable safety \narea around TS', xy=(30 , 23), xycoords='data', xytext=(10, -70), textcoords='offset points', arrowprops=dict(arrowstyle="->", connectionstyle="angle3,angleA=0,angleB=90"))
        # plt.annotate('Uncertainty in $V_{TS}$', xy=(10.5 , 28), xycoords='data', xytext=(40, 20), textcoords='offset points', arrowprops=dict(arrowstyle="->", connectionstyle="angle3,angleA=0,angleB=90"))
    return np.array([vel_obst_TTC_unc, vert_hull, tang_points, tang_unc_points], dtype=object)

# @profile
def add_uncert(unc_speed, unc_angle, vel_obst, posTS_rel, vel_TS):
    """ Function to add the uncertainties in speed and angle of the TS to the
    VO """
    unc_vel = np.array([[vel_TS[0]+unc_speed, vel_TS[1]-unc_angle],
                       [vel_TS[0]+unc_speed, vel_TS[1]],
                       [vel_TS[0]+unc_speed, vel_TS[1]+unc_angle],
                       [vel_TS[0]-unc_speed, vel_TS[1]+unc_angle],
                       [vel_TS[0]-unc_speed, vel_TS[1]],
                       [vel_TS[0]-unc_speed, vel_TS[1]-unc_angle]])

    vel_TSxy = vect_to_xy(vel_TS)
    # Vectors to xy component
    unc_vert = np.apply_along_axis(vect_to_xy, 1, unc_vel)

    # # Plot the uncertainty of vel_TS
    # plt.scatter(unc_vert[:,0]+posTS_rel[0], unc_vert[:,1]+posTS_rel[1], marker='.', linewidths=(0.1))
    # plt.fill(unc_vert[:,0]+posTS_rel[0], unc_vert[:,1]+posTS_rel[1], 'green', alpha=1 ,zorder=1, edgecolor='black', linewidth=1.5)

    unc_vert[:, 0] = unc_vert[:, 0] - vel_TSxy[0]

    unc_vert[:, 1] = unc_vert[:, 1] - vel_TSxy[1]

    unc_VO = calc_minkowski_sum(vel_obst, unc_vert)

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


def check_coll_point(vel_Relxy, vel_OS, vel_OSxy, vel_TS, vel_TSxy, vert_hull, tang_points):
    """ Function to calculate Point of collision, Distance to collision,
    Time to collision """
    # Calculate point, where the vel_Rel hits the polygone of the TS
    # Calculate the relative Velocotiy with magnitude and direction
    vel_Rel_in = vect_to_ang_mag(vel_Relxy)

    # Line function of the realtive veloctiy
    m_Rel = vel_Relxy[1]/vel_Relxy[0]
    b_Rel = 0

    # For the intersection point we need a line between two points; take max. x-value of the tangent points to calculate the y-value
    x_line_Rel_1 = tang_points[0, 0]
    x_line_Rel_2 = tang_points[1, 0]
    y_line_Rel_1 = m_Rel * x_line_Rel_1 + b_Rel
    y_line_Rel_2 = m_Rel * x_line_Rel_2 + b_Rel
    # print("xlin",x_line_Rel)
    # print("ylin",y_line_Rel)
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
    if check_side(vel_Rel_in, tang_points) == 'right':
        # print('right')
        pos_TS_rel_coll = tang_points[1, :]
    elif check_side(vel_Rel_in, tang_points) == 'left':
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
    if (vel_OS[1] % 180) == (vel_TS[1] % 180) and vel_OS[1] == vel_TS[1] or vel_TS[0] <= 0.0:
        time_coll = (np.sqrt(
            pos_TS_rel_coll[0] ** 2 + pos_TS_rel_coll[1] ** 2))/(np.abs(vel_OS[0]-vel_TS[0]))
        dist_coll = vel_OS[0] * time_coll
        coll_vect = np.array([dist_coll, vel_OS[1]])
        point_coll = vect_to_xy(coll_vect)
    elif (vel_OS[1] % 180) == (vel_TS[1] % 180) and vel_OS[1] != vel_TS[1]:
        time_coll = (np.sqrt(
            pos_TS_rel_coll[0] ** 2 + pos_TS_rel_coll[1] ** 2))/(vel_OS[0]+vel_TS[0])
        dist_coll = vel_OS[0] * time_coll
        coll_vect = np.array([dist_coll, vel_OS[1]])
        point_coll = vect_to_xy(coll_vect)
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

        if plotting:
            plt.scatter(point_coll[0], point_coll[1],
                        marker='x', c='red', linewidths=(0.7))

        # Distance to collision
        dist_coll = np.sqrt(point_coll[0] ** 2 + point_coll[1] ** 2)
        # print('Distance to collision:',dist_coll)

        # Time to collision
        time_coll = dist_coll / vel_OS[0]
        # print('Time to collision:',time_coll)
    return np.array([point_coll, dist_coll, time_coll], dtype=object)


def calc_colreg_con(vel_OS, vel_TS, tang_unc_points, max_speed_OS, pos_TS_rel):
    """ Function to calculate the COLREG constrains"""
    # c_points_new = np.empty([1,2])
    # pos_TS_rel = calc_coord_gps_to_xy(pos_OS, pos_TS)
    c_points_new = []
    if (check_colreg_rule_heading(vel_OS, vel_TS) == 'Head-on (Rule 14)'
        or check_colreg_rule_heading(vel_OS, vel_TS) == 'Right crossing (Rule 15)'):
        # line of the side the COLREG contrains has to be on
        x_p1 = tang_unc_points[1, 0]
        y_p1 = tang_unc_points[1, 1]
        x_p2 = tang_unc_points[0, 0]
        y_p2 = tang_unc_points[0, 1]
        m_1 = (y_p2-y_p1)/(x_p2-x_p1)  # left line seen from OS
        b_1 = y_p1 - m_1*x_p1
        # Dicriminent D
        a_x = 1+(1/m_1**2)
        b_x = -2*(1/m_1)*(b_1/m_1)
        c_x = (b_1**2/m_1**2) - max_speed_OS**2
        d_1 = b_x**2 - (4*a_x*c_x)
        if d_1 > 0:
            inter_y_1_l = (-b_x + np.sqrt(d_1))/(2*a_x)
            inter_y_2_l = (-b_x - np.sqrt(d_1))/(2*a_x)
            inter_x_1_l = (inter_y_1_l - b_1)/m_1
            inter_x_2_l = (inter_y_2_l - b_1)/m_1

            int_point_1 = np.array([[inter_x_1_l, inter_y_1_l]])
            int_point_2 = np.array([[inter_x_2_l, inter_y_2_l]])

            # Calculate distance of TS to the intersection points
            dist_p1 = np.sqrt((pos_TS_rel[1]-int_point_1[0, 1])**2
                              +(pos_TS_rel[0]-int_point_1[0, 0])**2)
            dist_p2 = np.sqrt((pos_TS_rel[1]-int_point_2[0, 1])**2
                              +(pos_TS_rel[0]-int_point_2[0, 0])**2)

            if dist_p1 < dist_p2:
                point_clos = int_point_1
                point_furt = int_point_2
            else:
                point_clos = int_point_2
                point_furt = int_point_1

            # Calculate the distance between the circle coordinates and the point coordinates
            distances_clos = cdist(point_clos.reshape(1, -1), c_points)
            distances_furt = cdist(point_furt.reshape(1, -1), c_points)

            # find the index, where the disance is the smallest
            index_clos_1st = np.argmin(distances_clos)
            index_furt_1st = np.argmin(distances_furt)

            # Sort the distances and find the index of the second smallest distance
            sorted_indices_clos = np.argsort(distances_clos)
            index_clos_2nd = sorted_indices_clos[0, 1]

            # Sort the distances and find the index of the second smallest distance
            sorted_indices_furt = np.argsort(distances_furt)
            index_furt_2nd = sorted_indices_furt[0, 1]

            # take bigger index between 1st and 2nd of clos and smaller index of 1st and 2ns od furt
            if index_clos_2nd > index_clos_1st:
                index_clos = index_clos_2nd
            else:
                index_clos = index_clos_1st

            if index_furt_2nd < index_furt_1st:
                index_furt = index_furt_2nd
            else:
                index_furt = index_furt_1st

            # If Index closest < index furthest --> take all points inbetween
            # Else Index closest > index furthest --> take all execpt the ones inbetween
            if index_clos < index_furt:
                c_points_new = c_points[index_clos-1:index_furt+2]
            else:
                c_points_new = np.delete(
                    c_points, np.s_[index_furt+2:index_clos-1], axis=0)
                # Change the order of the coordinates
                c_points_new = np.concatenate(
                    [c_points_new[index_furt+2:], c_points_new[:index_furt+2]])

            # Plot the points and fill between
            if plotting:
                plt.plot(c_points_new[:, 0], c_points_new[:, 1], 'black')
                plt.fill(c_points_new[:, 0], c_points_new[:, 1], 'purple',
                         alpha=0.5, zorder=0, edgecolor='black', linewidth=1.5)
                # plt.annotate('COLREG constrains', xy=(-8 , 8), xycoords='data', xytext=(-32, 80), textcoords='offset points', arrowprops=dict(arrowstyle="->", connectionstyle="angle3,angleA=0,angleB=90"))
    return np.array(c_points_new)


def merge_shapes(shape):
    """ Merge the velocity obstacles or COLRGE constrains to get one shape """
    added_poly = Polygon()
    for x in shape:
        if np.any(x):
            shape_vert = x.tolist()
            poly_shape = Polygon(shape_vert)
            # poly_shape = make_valid(poly_shape)
            added_poly = added_poly.union(poly_shape)

    # added_shape = added_poly.exterior.coords

    return added_poly

# @profile
def calc_free_vel_space(c_points, colreg_con, vel_obst_add):
    """ Function to calculate the free velocity space based on the maximum
    possible velocity of the OS """
    # vo_poly = vel_obst_add.tolist()
    # poly_vo = Polygon(vo_poly)
    
    # colreg_poly = colreg_con.tolist()
    # poly_colreg = Polygon(colreg_poly)
    poly_vo = vel_obst_add

    poly_colreg = colreg_con

    c_points_poly = c_points.tolist()
    poly_c = Polygon(c_points_poly)
    
    if poly_colreg.is_empty is False and poly_vo.is_empty is False:
        diff_c_colreg = poly_c.difference(poly_colreg)
        diff_c_col_vo = diff_c_colreg.difference(poly_vo)
    elif poly_colreg.is_empty is True and poly_vo.is_empty is False:
        diff_c_col_vo = poly_c.difference(poly_vo)
    else:
        diff_c_col_vo = poly_c
    
    if plotting:
        if diff_c_col_vo.geom_type == "MultiPolygon":
            for geom in diff_c_col_vo.geoms:
                plt.plot(*geom.exterior.xy, zorder=3, color='green')
        else:
            plt.plot(*diff_c_col_vo.exterior.xy, zorder=3, color='green')

    # discretize the velocity space; angle between 0-360° in n° steps, speed between 0-max. speed in n m/s steps
    vel_space = []
    vel_space_free = []
    
    for x in np.arange(n, max_speed_OS+n, n):
        for y in np.arange(0, 360, m):
            vel_space.append((x,y))
    
    vel_space = np.array(vel_space)
    vel_space_copy = vel_space.copy()
    # Vector to xy-components
    vel_space_ang = ((90 - vel_space_copy[:,1]) % 360)
    vel_space[:,0] = vel_space_copy[:,0] * np.cos(np.radians(vel_space_ang))
    vel_space[:,1] = vel_space_copy[:,0] * np.sin(np.radians(vel_space_ang))
   
    ### this here takes 12 ms (delete)
    # vel_space = np.apply_along_axis(vect_to_xy, 1, vel_space)
    
    ### this takes 18 ms
    vel_space = MultiPoint(vel_space)
   
    # validate if each point falls inside the free velocity space
    # points in the free space!!!
        
    vel_space_free = vel_space.intersection(diff_c_col_vo)

    if not vel_space_free.is_empty:      
        ### this takes 25 ms
        vel_space_free_xy = np.array([(point.x, point.y) for point in vel_space_free.geoms])
    
        if plotting:
            plt.scatter(vel_space_free_xy[:,0], vel_space_free_xy[:,1], 0.1, c='green', alpha=0.5)
        
        vel_space_free_vect = np.apply_along_axis(
        vect_to_ang_mag, 1, vel_space_free_xy)
    else:
        ### no velocity is free!!!
        vel_space_free_xy = []
        vel_space_free_vect = []

    return np.array(vel_space_free_xy), np.array(vel_space_free_vect)


def calc_new_vel_colreg(vel_des, search_area, pos_TS_rel, vel_OS):
    """ Function to calculate the new velocity for encounters, where COLREG
    constrains are applied """
    # Calculate new vel
    vel_space_free_xy = search_area[0]
    vel_space_free_vect = search_area[1]
    # pos_TS_rel = calc_coord_gps_to_xy(pos_OS, pos_TS)

    # Cost function to choose new velocity
    vect_pos_Rel = vect_to_ang_mag(pos_TS_rel)
    in_arcos_rel = (np.dot(vel_space_free_xy, vect_to_xy(
        vect_pos_Rel)))/(vel_space_free_vect[:, 0]*vect_pos_Rel[0])
    in_arcos_rel = np.round_(in_arcos_rel, decimals=10)
    angles_rel_free = np.rad2deg(np.arccos(in_arcos_rel))

    in_arcos_des = (np.dot(vel_space_free_xy, vect_to_xy(vel_des))
                    )/(vel_space_free_vect[:, 0]*vel_des[0])
    in_arcos_des = np.round_(in_arcos_des, decimals=10)
    angles_des_free = np.rad2deg(np.arccos(in_arcos_des))

    speed_des_free = np.abs(vel_space_free_vect[:, 0] - vel_des[0])

    vel_30 = vel_OS.copy()
    vel_30[1] = (vel_30[1] + 30) % 360
    in_arcos_30 = (np.dot(vel_space_free_xy, vect_to_xy(vel_30))
                    )/(vel_space_free_vect[:, 0]*vel_30[0])
    in_arcos_30 = np.round_(in_arcos_30, decimals=10)
    angles_30_free = np.rad2deg(np.arccos(in_arcos_30))
    # for angles in angles_30_free:
    #     if angles >= 30:
    #         angles = 30
    # angles_30_free = np.abs(angles_30_free - 30)
    
    # Normalize the values between 0-1, with 0 = 0 and 1 = max value
    norm_ang_des_free = angles_des_free / np.max(angles_des_free)
    norm_speed_des_free = speed_des_free / np.max(speed_des_free)
    norm_ang_30_free = angles_30_free / np.max(angles_30_free)
    norm_ang_rel_free = angles_rel_free / np.max(angles_rel_free)

    # Cost function
    J = (w_1 * norm_ang_des_free + w_2 * norm_speed_des_free +
         w_3 * norm_ang_30_free + w_4 * norm_ang_rel_free)

    # Extract index and plot the new velocity
    index_j = np.argmin(J)
    new_vel = ([vel_space_free_vect[index_j, 0],
               vel_space_free_vect[index_j, 1]])
    new_vel_xy = vect_to_xy(new_vel)
    if plotting:
        plt.quiver(0, 0, new_vel_xy[0], new_vel_xy[1], scale=1,
                   scale_units='xy', angles='xy', color='blue', zorder=4)

    return np.array(new_vel)


def calc_new_vel(vel_des, search_area, vel_OS):
    """ Function to calculate the new velocity for encounters, where no COLREG
    constrains are applied """
    vel_space_free_xy = search_area[0]
    vel_space_free_vect = search_area[1]

    in_arcos_des = (np.dot(vel_space_free_xy, vect_to_xy(vel_des))
                    )/(vel_space_free_vect[:, 0]*vel_des[0])
    in_arcos_des = np.round_(in_arcos_des, decimals=10)
    angles_des_free = np.rad2deg(np.arccos(in_arcos_des))

    speed_des_free = np.abs(vel_space_free_vect[:, 0] - vel_des[0])

    vel_30 = vel_OS.copy()
    vel_30[1] = (vel_30[1] + 30) % 360
    in_arcos_30 = (np.dot(vel_space_free_xy, vect_to_xy(vel_30))
                    )/(vel_space_free_vect[:, 0]*vel_30[0])
    in_arcos_30 = np.round_(in_arcos_30, decimals=10)
    angles_30_free = np.rad2deg(np.arccos(in_arcos_30))
    # for angles in angles_30_free:
    #     if angles >= 30:
    #         angles = 30
    # angles_30_free = np.abs(angles_30_free - 30)
    

    # Normalize the values between 0-1, with 0 = 0 and 1 = max value
    norm_ang_des_free = angles_des_free / np.max(angles_des_free)
    norm_speed_des_free = speed_des_free / np.max(speed_des_free)
    norm_ang_30_free = angles_30_free / np.max(angles_30_free)

    # Cost function
    J = (w_1 * norm_ang_des_free + w_2 *
         norm_speed_des_free + w_3 * norm_ang_30_free)

    # Extract index and plot the new velocity
    index_j = np.argmin(J)
    new_vel = ([vel_space_free_vect[index_j, 0],
               vel_space_free_vect[index_j, 1]])
    new_vel_xy = vect_to_xy(new_vel)
    if plotting:
        plt.quiver(0, 0, new_vel_xy[0], new_vel_xy[1], scale=1,
                   scale_units='xy', angles='xy', color='blue', zorder=4)

    return np.array(new_vel)


def extract_most_common_vel(arr, threshold):
    """ Function to calculate the most common velocity"""
    distances = cdist(arr, arr, 'euclidean')
    similar_rows = np.where(distances <= threshold)
    most_common_index = mode(similar_rows[0]).mode[0]
    return arr[most_common_index]

def calc_vel_final(ts_info, os_info):
    
    if plotting:
        plt.plot(c_points[:, 0], c_points[:, 1], 'black')
        plt.fill(c_points[:, 0], c_points[:, 1], 'gray',
                 alpha=0.5, zorder=0, edgecolor='black', linewidth=1.5)
    
    vel_OS = os_info[0]
    vel_OSxy = os_info[1]
    ang_OS_rad = os_info[2]
    vel_des = os_info[3]
    vel_des_xy = vect_to_xy(vel_des)
    # for each TS --> calc VO with uncertainties
    TS_VO = np.empty((0, 9))
    for TSo in ts_info:
        VOs = calc_vel_obst(TSo, ang_OS_rad)
        # TSo = TSo.tolist()
        # TSo.append(VOs)
        # TSo = np.array(TSo, dtype=object)
        TSo = np.append(TSo, VOs)
        TS_VO = np.vstack((TS_VO, TSo))
    print(TS_VO) 
    # for each VO --> check collision
    TS_VO_check = np.empty((0, 15))
    for TSv in TS_VO:
        vel_test = np.array([TSv[3], TSv[4]])
        vel_test_xy = vect_to_xy(vel_test)
        vel_Rel_test = calc_rel_vel(vel_OS, vel_test)
        vel_Rel_test_xy = vect_to_xy(vel_Rel_test)
        VO_check = check_collision(vel_OS, TSv[5])
        TSv = np.append(TSv, VO_check)
         
        # if check collision = TRUE --> check rule, calculate COLREG con, calc collision point (assigned to TS)
        if VO_check:
            VO_rule = check_colreg_rule_heading(vel_OS, vel_test)
            TSv = np.append(TSv, VO_rule)
            Col_con = calc_colreg_con(
                vel_OS, vel_test, TSv[8], max_speed_OS, TSv[0])
            TSv = np.append(TSv, [0])
            TSv[11] = Col_con
            Coll = check_coll_point(
                vel_Rel_test_xy, vel_OS, vel_OSxy, vel_test, vel_test_xy,
                TSv[6], TSv[7])
            TSv = np.append(TSv, Coll)
        else:
            TSv = np.append(TSv, [0, 0, 0, 0, 0])
        TS_VO_check = np.vstack((TS_VO_check, TSv))

    ## TS_VO_check = (pos_TS, len_TS, wid_TS, speed_TS, ang_TS, VO_vert, hull_vert, tang_points, tang_unc_points, Check_coll, col_rule, col_con, point_coll,dist_coll, time_coll)
    # Merge VOs and COLREG cons
    #print(TS_VO_check[:,5])
    VO_merged = merge_shapes(TS_VO_check[:, 5])
    COL_con_merged = merge_shapes(TS_VO_check[:, 11])
    
    new_vel_testo = np.empty((0, 2))
    if Point(vel_OSxy[0], vel_OSxy[1]).within(VO_merged):
        # Calculate free vel space --> Circle - VOs - COLREG cons
        free_vel = calc_free_vel_space(c_points, COL_con_merged, VO_merged)
    # Calculate new velocities for each VO the OS vel is colliding with
    for TS_vel in TS_VO_check:
        # for each TS with collision and COLREG rules where contrains have to be applied, calculate the new velocity in free velocity space
        if TS_vel[10] == 'Right crossing (Rule 15)' or TS_vel[10] == 'Head-on (Rule 14)' or TS_vel[10] == 'Overtaking (Rule 13)':

            # if free_vel is empty, choose a velocity in the COLREG_con

            # Calculate new velocity
            new_vel = calc_new_vel_colreg(vel_des, free_vel, TS_vel[0], vel_OS)
            if np.any(new_vel):
                new_vel_testo = np.vstack((new_vel_testo, new_vel))

            else:
                new_vel_testo = np.vstack((new_vel_testo, ["no", "no"]))
        # for each TS with collision and no COLREG constrains have to be applied, calculate the new velocity in free velocity space
        elif TS_vel[10] == 'Left crossing (Rule 15)' or TS_vel[10] == 'Being Overtaken (Rule 13)':
            if TS_vel[14] <= threshold:
                new_vel = calc_new_vel(vel_des, free_vel, vel_OS)
                if np.any(new_vel):
                    new_vel_testo = np.vstack((new_vel_testo, new_vel))
                    
                else:
                    new_vel_testo = np.vstack((new_vel_testo, ["no", "no"]))
    
    # Extract the final new velocity
    if np.any(new_vel_testo):
        if not Point(vel_des_xy[0], vel_des_xy[1]).within(VO_merged):
            new_vel_final = vel_des
        else:
            new_vel_final = extract_most_common_vel(new_vel_testo, 0)
        
    else:
        if not Point(vel_des_xy[0], vel_des_xy[1]).within(VO_merged):
            new_vel_final = vel_des
        else:
            new_vel_final = vel_OS
            
        
            
    return new_vel_final



# Create a circle around the OS with the maximum speed of the OS --> all possible velocities of OS
# Parametric equation circle
c_angle = np.linspace(0, np.radians(360), 150)
x = max_speed_OS * np.cos(c_angle)
y = max_speed_OS * np.sin(c_angle)
# List of all circle points = c_points
c_points = np.column_stack((x, y))

# ROS 2 implimentation, find Simulator,  Build a class with functions

# How to compare the performance of the code, time difference with more ships?

# Hysteresis function

# Dynamic Window (test with and without DW)
# Which velocity can be reached in a given time window --> max. acceleration and deceleretaion, angular velocity (turn ratio)
