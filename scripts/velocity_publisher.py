#!/usr/bin/env python

#package imports
import rospy

#message imports
#from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
#subscribed topics

#published topics
T_set_vel = "/mavros/setpoint_velocity/cmd_vel_unstamped"
#global variables

#constants
SET_VEL = Twist()
#SET_VEL_Stamped = TwistStamped()

#callbacks
def subpub_SET_VEL(msg):
    global SET_VEL
    rate = rospy.Rate(60)
    pub = rospy.Publisher(T_set_vel, Twist, queue_size=10)
    pub.publish(SET_VEL)
    
    rate.sleep()
#functions

#main loop
def main_node():
    #init
    global SET_VEL
    rospy.init_node("velocity_publisher", anonymous = False)
    
    #subscribers
    rospy.Subscriber(T_set_vel, Twist, subpub_SET_VEL)

    #publishers
    SET_VEL_pub = rospy.Publisher(T_set_vel, Twist, queue_size =10)

    while not rospy.is_shutdown():
        rospy.Subscriber(T_set_vel, Twist, subpub_SET_VEL)
        user_input = raw_input("enter velocity [vx, vy, vz, omega]: ")
        vel_in  = user_input.split(",")
        try:
            SET_VEL.linear.x = float(vel_in [0])
            SET_VEL.linear.y = float(vel_in [1])
            SET_VEL.linear.z = float(vel_in [2])
            SET_VEL.angular.z = float(vel_in [3])
            
            SET_VEL_pub.publish(SET_VEL)
            print("publish")
        except:
            SET_VEL.linear.x = 0
            SET_VEL.linear.y = 0
            SET_VEL.linear.z = 0
            SET_VEL.angular.z = 0
            SET_VEL_pub.publish(SET_VEL)
        print("")
        SET_VEL_pub.publish(SET_VEL)
    rospy.spin()


if __name__ == '__main__':
    try:
        main_node()
    except rospy.ROSInterruptException:
        pass
    