#!/usr/bin/env python

#package imports
import numpy as np
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf.transformations import quaternion_conjugate as qc, quaternion_multiply as qm
import time
import roslib
import tf2_ros
import socket
import os
#message imports
from sensor_msgs.msg import LaserScan, CameraInfo
from geometry_msgs.msg import Pose2D, Vector3, Pose, PoseStamped
#subscribed topics
T_TAG_POS = "uav/tag_pos"
T_POSE = "/mavros/local_position/pose"

#published topics



#constants
# FILEPATH = "/home/uoroswork/catkin_ws/src/uav_nav_ops/scripts"
FILEPATH = "/home/mavros_test/catkin_ws/src/uav_nav_ops/scripts"
FILENAME = "mocap_to_drone_world.txt"
XFORM_FILE = os.path.join(FILEPATH, FILENAME)

CAMERA_MAT = np.array([[2.8054458697503384*10**2, 0, 3.4675256521746036*10**2],
                        [2.8054458697503384*10**2,  2.1038132491104975*10**2, 0],
                        [0,0,1]])
CAMERA_CENTER_X = 3.4675256521746036*10**2
CAMERA_CENTER_Y = 2.1038132491104975*10**2
FOCAL_LEN = 2.8054458697503384*10**2
WP_DIST_TH = 1
#globals
tag_pos = Vector3()
dw_pose = PoseStamped()
all_waypoints =[]

#functions
def handle_uav_pose(msg):
    global dw_pose
    pose_quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    (dw_pose.pose.orientation.x, dw_pose.pose.orientation.y, dw_pose.pose.orientation.z)=euler_from_quaternion(pose_quat)
    dw_pose.pose.position.x = msg.pose.position.x
    dw_pose.pose.position.y = msg.pose.position.y
    dw_pose.pose.position.z = msg.pose.position.z

def handle_tag_pos(msg):
    global tag_pos
    tag_pos = msg

def ROT_mat(rx, ry, rz):
    R11 = np.cos(ry)*np.cos(rz)
    R12 = np.sin(rx)*np.sin(ry)*np.cos(rz) - np.cos(rx)*np.sin(rz)
    R13 = np.cos(rx)*np.sin(ry)*np.cos(rz) + np.sin(rx)*np.sin(rz)
    
    R21 = np.cos(ry)*np.sin(rz)
    R22 = np.sin(rx)*np.sin(ry)*np.sin(rz) + np.cos(rx)*np.cos(rz)
    R23 = np.cos(rx)*np.sin(ry)*np.sin(rz) - np.sin(rx)*np.cos(rz)
    
    R31 = -np.sin(ry)
    R32 = np.sin(rx)*np.sin(ry)
    R33 = np.cos(rx)*np.cos(ry)
    result = np.array([[R11, R12, R13],
                       [R21, R22, R23],
                       [R31, R32, R33]])
    return result

def HT_mat(tx, ty, tz, rx, ry, rz):
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
                       [0,0,0,1]])
    return result

def read_transform(path):
    f = open(path, "r")
    lines = f.readlines()
    #result = (float(lines[0]), float(lines[1]), float(lines[2]), float(lines[3]), float(lines[4]), float(lines[5]))
    result = (float(lines[0]), float(lines[1]), float(lines[2]), 0, 0, float(lines[5]))

    return result

def uv_cam(px, py, cx, cy, f):
    vx = px - cx
    vy = py - cy
    vz = f
    
    magnitude = np.sqrt(vx**2+vy**2+vz**2)
    
    result = (1/magnitude)*np.array([[vx],
                                    [vy],
                                    [vz]])
    return result

def tag_pos_world(camera_pos, uv_world):
    tx = float(camera_pos[0])
    ty = float(camera_pos[1])
    tz = float(camera_pos[2])

    k_z0 = -tz/uv_world[2]
    tag_position = (tx + float(k_z0*uv_world[0]), ty + float(k_z0*uv_world[1]))
    return tag_position

def calculate_distance(P1,P2):
    distance = np.sqrt((P1[0]-P2[0])**2+(P1[1]-P2[1])**2)
    return distance

def new_wp_check(waypoint_list, new_wp, TH):
    if len(waypoint_list) == 0:
        return True
    else:
        distances = []
        for wp in waypoint_list:
            distance = calculate_distance(new_wp, wp)
            distances.append(distance)
            
        if min(distances) > TH:
            result = True
        else:
            result = False
        return result

pose_offset = read_transform(XFORM_FILE)
print(pose_offset)
H0_dw = HT_mat(pose_offset[0], pose_offset[1], pose_offset[2], pose_offset[3], pose_offset[4], pose_offset[5])
#Hdw_d from pose topic
Hd_imu = HT_mat(0.027, 0.01,-0.019, 0, 0, 0)
Himu_cl = HT_mat(0.06, -0.01, 0.018, 0, 45*np.pi/180, 0)
Hcl_cf = HT_mat(0, 0, 0, -90*np.pi/180,0,-90*np.pi/180)


R0_dw = ROT_mat(pose_offset[3], pose_offset[4], pose_offset[5])
#Rdw_d from pose topic
Rd_imu = ROT_mat(0, 0, 0)
Rimu_cl = ROT_mat(0, 45*np.pi/180, 0)
Rcl_cf = ROT_mat(-90*np.pi/180,0,-90*np.pi/180)

R_ENU_FLU = ROT_mat(0,0,-90*np.pi/180)
H_ENU_FLU = HT_mat(0,0,0,0,0,-90*np.pi/180)

def calculate_tag_pos(camera_xform, uv_world):
    tx = float(camera_xform[0][3])
    ty = float(camera_xform[1][3])
    tz = float(camera_xform[2][3])

    ux = float(uv_world[0])
    uy = float(uv_world[1])
    uz = float(uv_world[2])

    k_z0 = -tz/uz

    tag_pos_w0 = (tx + k_z0*ux, ty + k_z0*uy)

    return tag_pos_w0
def main_loop():
    #init
    global tag_pos
    global all_waypoints
    global dw_pose
    rospy.init_node("apriltag_xform", anonymous = False)
  
    #f.write("test")
    
    #subscribers
    rospy.Subscriber(T_POSE, PoseStamped, handle_uav_pose)
    rospy.Subscriber(T_TAG_POS, Vector3, handle_tag_pos)
    
    #publishers
    
    while not rospy.is_shutdown():

        Rdw_d_ENU = ROT_mat(dw_pose.pose.orientation.x, dw_pose.pose.orientation.y, dw_pose.pose.orientation.z)
        Rdw_d = np.matmul(R_ENU_FLU, Rdw_d_ENU)


        Hdw_d_ENU = HT_mat(dw_pose.pose.position.x, dw_pose.pose.position.y, dw_pose.pose.position.z, dw_pose.pose.orientation.x, dw_pose.pose.orientation.y, dw_pose.pose.orientation.z)
        Hdw_d = np.matmul(H_ENU_FLU, Hdw_d_ENU)
        H0_d = np.matmul(H0_dw, Hdw_d)
        H0_cf = np.matmul(np.matmul(np.matmul(np.matmul(H0_dw, Hdw_d),Hd_imu),Himu_cl),Hcl_cf)
   
        if tag_pos.x != 0:
            try:

                uv = uv_cam(tag_pos.x, tag_pos.y, CAMERA_CENTER_X, CAMERA_CENTER_Y, FOCAL_LEN)

                uv_world = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(R0_dw, Rdw_d),Rd_imu),Rimu_cl),Rcl_cf),uv)
                
                tag_pos_w0 = calculate_tag_pos(H0_cf, uv_world)
                print("tag_pos_w0")
                print(tag_pos_w0)

            except:
                pass
        time.sleep(0.25)
    
if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass