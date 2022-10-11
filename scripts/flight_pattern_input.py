#!/usr/bin/env python

#package imports
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import os
#message imports

from geometry_msgs.msg import Twist, PoseStamped

#subscribed topics

#published topics
T_pattern_params = "/uav/pattern_params"

#global variables

#constants
UAV_POSE = PoseStamped()
PATTERN_PARAMS = PoseStamped()
HOLD_POS_PARAM = PoseStamped()
HOLD_POS_PARAM.pose.orientation.w = 0
FILEPATH = "/home/mavros_test/catkin_ws/src/uav_nav_ops/scripts"
FILENAME = "mocap_to_drone_world.txt"
XFORM_FILE = os.path.join(FILEPATH, FILENAME)

#callbacks
def subpub_PATTERN_PARAMS(msg):
    global PATTERN_PARAMS
    rate = rospy.Rate(60)
    #pub = rospy.Publisher(T_pattern_params, PoseStamped, queue_size=10)
    #pub.publish(PATTERN_PARAMS)
    rate.sleep()

#functions

def read_transform(path):
    f = open(path, "r")
    lines = f.readlines()
    #result = (float(lines[0]), float(lines[1]), float(lines[2]), float(lines[3]), float(lines[4]), float(lines[5]))
    result = (float(lines[0]), float(lines[1]), float(lines[2]), 0, 0, float(lines[5]))

    return result

def to_params(pattern_input):
    
    PATTERN_PARAMS = PoseStamped()
    PATTERN_PARAMS.pose.position.x = float(pattern_input[0])
    PATTERN_PARAMS.pose.position.y = float(pattern_input[1])
    PATTERN_PARAMS.pose.position.z = float(pattern_input[2])
    PATTERN_PARAMS.pose.orientation.x = float(pattern_input[3])
    PATTERN_PARAMS.pose.orientation.w = 1
    return PATTERN_PARAMS

   
#main loop
def main_node():
    #init
    global PATTERN_PARAMS
    rospy.init_node("flight_pattern_input", anonymous = False)
    
    #subscribers
    rospy.Subscriber(T_pattern_params, PoseStamped, subpub_PATTERN_PARAMS)
   
    #publishers
    PATTERN_PARAMS_pub = rospy.Publisher(T_pattern_params, PoseStamped, queue_size =10)

    while not rospy.is_shutdown():
        rospy.Subscriber(T_pattern_params, Twist, subpub_PATTERN_PARAMS)
        user_input = raw_input("enter pattern params [x, y, z, r]: ")
        
        try:
            pattern_input = user_input.split(",")
            PATTERN_PARAMS = to_params(pattern_input)
        except:
            PATTERN_PARAMS = HOLD_POS_PARAM
            print("invalid input, holding position")
        PATTERN_PARAMS_pub.publish(PATTERN_PARAMS)
        

        
    rospy.spin()


if __name__ == '__main__':
    try:
        main_node()
    except rospy.ROSInterruptException:
        pass
    