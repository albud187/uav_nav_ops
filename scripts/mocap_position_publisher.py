#!/usr/bin/env python

#package imports
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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
            pos_in  = user_input.split(",")
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
    