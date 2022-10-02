#!/usr/bin/env python

#package imports
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import os
#message imports

from geometry_msgs.msg import Twist, PoseStamped

#subscribed topics
T_pose = "/mavros/local_position/pose"
#published topics
T_set_pos = "/mavros/setpoint_position/local"
#global variables

#constants
UAV_POSE = PoseStamped()
SET_POSE = PoseStamped()
FILEPATH = "/home/uoroswork/catkin_ws/src/uav_nav_ops/scripts"
FILENAME = "mocap_to_drone_world.txt"
XFORM_FILE = os.path.join(FILEPATH, FILENAME)

#callbacks
def subpub_SET_POSE(msg):
    global SET_POSE
    rate = rospy.Rate(60)
    pub = rospy.Publisher(T_set_pos, PoseStamped, queue_size=10)
    pub.publish(SET_POSE)
    rate.sleep()

def handle_pose(msg):
    global UAV_POSE
    UAV_POSE = msg

#functions

def read_transform(path):
    f = open(path, "r")
    lines = f.readlines()
    result = (float(lines[0]), float(lines[1]), float(lines[2]), float(lines[3]), float(lines[4]), float(lines[5]))
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

xform = xform_HT(read_transform(XFORM_FILE))

def convert_pos(xform, pos_in):
    pos_vect_h = np.array([[float(pos_in[0])],
                            [float(pos_in[1])],
                            [float(pos_in[2])],
                            [1]
                            ]
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

   
#main loop
def main_node():
    #init
    global SET_POSE
    rospy.init_node("position_publisher", anonymous = False)
    
    #subscribers
    rospy.Subscriber(T_set_pos, PoseStamped, subpub_SET_POSE)
    rospy.Subscriber(T_pose, PoseStamped, handle_pose)
    #publishers
    SET_POSE_pub = rospy.Publisher(T_set_pos, PoseStamped, queue_size =10)

    while not rospy.is_shutdown():
        rospy.Subscriber(T_set_pos, Twist, subpub_SET_POSE)
        user_input = raw_input("enter pos [x, y, z]: ")
        
        try:
            pos_in_raw  = user_input.split(",")
            pos_in_d = mocap_to_drone(pos_in_raw)
            SET_POSE = calculate_set_pose(pos_in, UAV_POSE)
            print("publishing pos = "+ str(SET_POSE.pose.position.x)+", "+str(SET_POSE.pose.position.y)+", "+str(SET_POSE.pose.position.z))

        except:
            SET_POSE = UAV_POSE
            print("invalid input, holding position")
        SET_POSE_pub.publish(SET_POSE)
        # print("publish")
        # except:
        #     print("invalid input")
        # print("")
        
    rospy.spin()


if __name__ == '__main__':
    try:
        main_node()
    except rospy.ROSInterruptException:
        pass
    