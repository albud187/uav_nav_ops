#!/usr/bin/env python

#package imports
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#message imports
#from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist, PoseStamped
#subscribed topics

#published topics
T_pose = "/mavros/local_position/pose"
T_set_vel = "/mavros/setpoint_velocity/cmd_vel_unstamped"
#global variables
SET_VEL = Twist()
UAV_POSE = PoseStamped()
#constants

#SET_VEL_Stamped = TwistStamped()

#callbacks
def subpub_SET_VEL(msg):
    global SET_VEL
    rate = rospy.Rate(60)
    pub = rospy.Publisher(T_set_vel, Twist, queue_size=10)
    pub.publish(SET_VEL)
    
    rate.sleep()

def handle_pose(msg):
    global UAV_POSE
    UAV_POSE = msg

#functions

#calculates the rotation matrix from the world frame to the object's body frame
#input pose is the drone's pose directly from PoseStampted topic
def ROT_mat(input_pose):
    orientation_quat = (input_pose.pose.orientation.x, input_pose.pose.orientation.y, input_pose.pose.orientation.z, input_pose.pose.orientation.w)
    rx, ry, rz = euler_from_quaternion(orientation_quat)
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

#converts a velocity command expressed in the body frame to a velocity command in the drone world frame
#input pose is the drone's pose directly from PoseStampted topic
def body_to_world_vel(bvel, input_pose):
    result_vel = Twist()
    v_linear = np.array([[bvel.linear.x],
                            [bvel.linear.y],
                            [bvel.linear.z]])
    v_angular = np.array([[bvel.angular.x],
                            [bvel.angular.y],
                            [bvel.angular.z]])
    ROT = ROT_mat(input_pose)

    world_linear = np.matmul(ROT, v_linear)
    world_angular = np.matmul(ROT, v_angular)

    result_vel.linear.x = float(world_linear[0])
    result_vel.linear.y = float(world_linear[1])
    result_vel.linear.z = bvel.linear.z

    result_vel.angular.x = 0
    result_vel.angular.y = 0
    result_vel.angular.z = float(world_angular[2])

    return result_vel

def orbit_vel(radius):
    result_vel = Twist()
    orbit_speed = 0.2
    result_vel.linear.x = orbit_speed
    result_vel.angular.z = orbit_speed/radius
    return result_vel

def xy_mov(x_vel, y_vel):
    result_vel = Twist()
    orbit_speed = 0.2
    result_vel.linear.x = x_vel
    result_vel.linear.y = y_vel
    return result_vel

orbit_radius = 0.3
#positive x = forwards
#negative x = backwards
#positve y = left
#positve y = right
x_vel = -0.2
y_vel = 0
bvel = orbit_vel(orbit_radius)
#bvel = xy_mov(x_vel, y_vel)

#main loop
def main_node():
    #init
    global SET_VEL
    global UAV_POSE
    rospy.init_node("velocity_publisher", anonymous = False)
    
    #subscribers
    rospy.Subscriber(T_set_vel, Twist, subpub_SET_VEL)
    rospy.Subscriber(T_pose, PoseStamped, handle_pose)
    #publishers
    SET_VEL_pub = rospy.Publisher(T_set_vel, Twist, queue_size =10)

    while not rospy.is_shutdown():
        rospy.Subscriber(T_set_vel, Twist, subpub_SET_VEL)
        SET_VEL = body_to_world_vel(bvel, UAV_POSE)
        print(SET_VEL)
        SET_VEL_pub.publish(SET_VEL)
    rospy.spin()


if __name__ == '__main__':
    try:
        main_node()
    except rospy.ROSInterruptException:
        pass
    