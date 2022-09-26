#!/usr/bin/env python3

#package imports
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#message imports
from geometry_msgs.msg import Vector3

#subscribed topics

#published topics
TP_QC_WP = "/hc_WP"

#global variables
WP = Vector3()


#callback functions
def subpub_WP(msg):
    global WP
    rate = rospy.Rate(60)
    pub = rospy.Publisher(TP_QC_WP, Vector3, queue_size=10)
    pub.publish(WP)
    rate.sleep()

#main loop
def WP_publisher():
    #globals
    global WP
    #init node
    rospy.init_node('velocity_publisher', anonymous=False)

    #subscribers
    rospy.Subscriber(TP_QC_WP , Vector3, subpub_WP)

    #publishers
    WP_pub = rospy.Publisher(TP_QC_WP, Vector3, queue_size = 10)

    while not rospy.is_shutdown():
        rospy.Subscriber(TP_QC_WP, Vector3, subpub_WP)
        user_input = input("enter waypoint [x, y, z]: ")
        WP_in  = user_input.split(",")
        try:
            WP.x = float(WP_in[0])
            WP.y = float(WP_in[1])
            WP.z = float(WP_in[2])
            
            WP_pub.publish(WP)

        except:
            print("invalid input")
        print("")
        WP_pub.publish(WP)
    rospy.spin()

if __name__ == '__main__':
    try:
        WP_publisher()
    except rospy.ROSInterruptException:
        pass