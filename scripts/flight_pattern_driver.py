#!/usr/bin/env python

#package imports
import rospy
import numpy as np
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import os
#message imports

from geometry_msgs.msg import Twist, PoseStamped

#subscribed topics
T_pattern_params = "/uav/pattern_params"
T_pose = "/mavros/local_position/pose"

#published topics
T_set_pos = "/mavros/setpoint_position/local"

#global variables
PATTERN_PARAMS = PoseStamped()
UAV_POSE = PoseStamped()
SET_POSE = PoseStamped()
ZERO_POSE = PoseStamped()
#constants


FILEPATH = "/home/mavros_test/catkin_ws/src/uav_nav_ops/scripts"
FILENAME = "mocap_to_drone_world.txt"
XFORM_FILE = os.path.join(FILEPATH, FILENAME)



#functions
def calculate_distance(P1, P2):
    dx = P1.pose.position.x - P2.pose.position.x
    dy = P1.pose.position.y - P2.pose.position.y
    dz = P1.pose.position.z - P2.pose.position.z

    distance = np.sqrt(dx**2 + dy**2 + dz**2)
    return distance

def box_waypoints(pattern_params):
    bx = pattern_params.pose.position.x
    by = pattern_params.pose.position.y
    bz = pattern_params.pose.position.z
    R = pattern_params.pose.orientation.x
    waypoints = [(bx-R, by-R, bz),
                 (bx-R, by+R, bz),
                 (bx+R, by+R, bz),
                 (bx+R, by-R, bz)]
    return waypoints

def read_transform(path):
    f = open(path, "r")
    lines = f.readlines()
    #result = (float(lines[0]), float(lines[1]), float(lines[2]), float(lines[3]), float(lines[4]), float(lines[5]))
    result = (float(lines[0]), float(lines[1]), float(lines[2]), 0, 0, float(lines[5]))

    return result

def xform_HT(xform):

    tx = xform[0]
    ty = xform[1]
    tz = xform[2]
    rx = xform[3]
    ry = xform[4]
    rz = xform[5]
    R11 = np.cos(ry)*np.cos(rz)
    R12 = np.sin(rx)*np.sin(ry)*np.cos(rz) - np.cos(rx)*np.sin(rz)
    R13 = np.cos(rx)*np.sin(ry)*np.cos(rz) + np.sin(rx)*np.sin(rz)
    
    R21 = np.cos(ry)*np.sin(rz)
    R22 = np.sin(rx)*np.sin(ry)*np.sin(rz) + np.cos(rx)*np.cos(rz)
    R23 = np.cos(rx)*np.sin(ry)*np.sin(rz) - np.sin(rx)*np.cos(rz)
    
    R31 = -np.sin(ry)
    R32 = np.sin(rx)*np.sin(ry)
    R33 = np.cos(rx)*np.cos(ry)
    result = np.array([[R11, R12, R13, tx],
                       [R21, R22, R23, ty],
                       [R31, R32, R33, tz],
                       [0, 0, 0, 1]])
    return result
ENU_FLU = xform_HT((0,0,0,0,0,90*np.pi/180))
Hdw_0 = np.linalg.inv(xform_HT(read_transform(XFORM_FILE)))
xform = np.matmul(ENU_FLU, Hdw_0)

def convert_pos(xform, pos_in):
    pos_vect_h = np.array([[float(pos_in[0])],
                            [float(pos_in[1])],
                            [float(pos_in[2])],
                            [1]
                            ])
    pos_d = np.matmul(xform, pos_vect_h)
    pos_out = [float(pos_d[0]), float(pos_d[1]), float(pos_d[2])]
    return pos_out

def calculate_set_pose(pos_in, uav_pose):
    
    SET_POSE = PoseStamped()
    SET_POSE.pose.position.x = float(pos_in[0])
    SET_POSE.pose.position.y = float(pos_in[1])
    SET_POSE.pose.position.z = float(pos_in[2])

    error_x = float(pos_in[0]) - uav_pose.pose.position.x
    error_y = float(pos_in[1]) - uav_pose.pose.position.y
    target_angle = np.arctan2(error_y, error_x)
    orientation_euler = (0,0,target_angle)
    (SET_POSE.pose.orientation.x, SET_POSE.pose.orientation.y, SET_POSE.pose.orientation.z, SET_POSE.pose.orientation.w) = quaternion_from_euler(0,0,target_angle)
    return SET_POSE

#callbacks
def handle_pose(msg):
    global UAV_POSE
    UAV_POSE = msg

def handle_params(msg):
    global PATTERN_PARAMS
    PATTERN_PARAMS = msg


prev_param_seq = 0
def check_update_param(pattern_param):
    global prev_param_seq
    if pattern_param.header.seq != prev_param_seq:
        result = True
    else:
        result = False
    prev_param_seq = pattern_param.header.seq
    return result

#main loop

def main_node():
    #init
    
    global UAV_POSE
    global PATTERN_PARAMS
    
    rospy.init_node("flight_pattern_driver", anonymous = False)
    
    #subscribers
    rospy.Subscriber(T_pattern_params, PoseStamped, handle_params)
    rospy.Subscriber(T_pose, PoseStamped, handle_pose)

    #publishers/mavros/local_position/pose
    SET_POSE_pub = rospy.Publisher(T_set_pos, PoseStamped, queue_size =10)
    wp_idx = 0
    SET_POSE = ZERO_POSE
    while not rospy.is_shutdown():
        
        if check_update_param(PATTERN_PARAMS):
            
            while PATTERN_PARAMS.pose.orientation.w != 0:
                try:
                    waypoints = box_waypoints(PATTERN_PARAMS)
                    current_waypoint_w0 = waypoints[wp_idx]
                    current_waypoint_dw_raw = convert_pos(xform, current_waypoint_w0)
                    current_waypoint_pose = calculate_set_pose(current_waypoint_dw_raw, UAV_POSE)
                    SET_POSE = current_waypoint_pose
                    SET_POSE_pub.publish(SET_POSE)
                    distance = calculate_distance(current_waypoint_pose, UAV_POSE)
                    print(wp_idx)
                    print("x: "+str(SET_POSE.pose.position.x))
                    print("y: "+str(SET_POSE.pose.position.y))
                    print("z: "+str(SET_POSE.pose.position.z))
                    print("d: "+str(distance))
                    print(" ")
                    
                    if distance < 0.1:
                        wp_idx = wp_idx + 1
                except:
                    wp_idx = 0
                
            else:
                SET_POSE=UAV_POSE
                print("holding")
        
        
        SET_POSE_pub.publish(SET_POSE)
        #print(PATTERN_PARAMS.header.seq)
        
    rospy.spin()


if __name__ == '__main__':
    try:
        main_node()
    except rospy.ROSInterruptException:
        pass
    