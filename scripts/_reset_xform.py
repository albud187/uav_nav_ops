#!/usr/bin/env python
#_*_ coding: utf8 _*_
import socket
import rospy
import ast
import re
import json
from _thread import *
import os
#message imports

#subscribed topics

#published topics

#global variables

#functions

FILEPATH = "/home/uoroswork/catkin_ws/src/uav_nav_ops/scripts"
FILENAME = "mocap_to_drone_world.txt"
TARGET_FILE = os.path.join(FILEPATH, FILENAME)

def Main():
   

    f = open(TARGET_FILE, "w")
  

    RB5_transform = (0,0,0,0,0,0)
    print(RB5_transform)
    print(" ")

    
    for coord in RB5_transform:
        print(coord)
        f.write(str(coord)+"\n")

    f.close()



# if __name__ == '__main__':
#     Main()
Main()
