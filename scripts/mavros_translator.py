#!/usr/bin/env python
#run on docker image on voxl flight board

#package imports

#message imports

#subscribed topics

#published topics

#global variables

#constants

#callbacks

#functions

#main loop
def main_node():
    #init
    rospy.init_node("mavros_translator", anonymous = False)
    #rate = rospy.Rate(2000)
    
    #subscribers


    #publishers

    #loop
    #rospy.spin()
    
    while not rospy.is_shutdown():


if __name__ == '__main__':
    try:
        main_node()
    except rospy.ROSInterruptException:
        pass
    