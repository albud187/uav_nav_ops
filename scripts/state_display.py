#!/usr/bin/env python

#package imports
import rospy
#message imports
from nav_msgs.msg import Odometry
#subscribed topics

#published topics

#global variables

#constants

#callbacks

#functions

#main loop
def main_node():
    #init
    rospy.init_node("position_publisher", anonymous = False)
    #rate = rospy.Rate(2000)
    global sensor_data
    #subscribers


    #publishers

    #loop
    #rospy.spin()
    #print(sensor_data)
    while not rospy.is_shutdown():


if __name__ == '__main__':
    try:
        main_node()
    except rospy.ROSInterruptException:
        pass
    