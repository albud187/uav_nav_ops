#!/usr/bin/env python
#_*_ coding: utf8 _*_
import socket
import rospy
import ast
import re
import json

import os
import time
#message imports

#subscribed topics

#published topics

#global variables

#functions
def handle_pose(key, pose_data):
    
    pose_data_str = str(pose_data)
    pose_data_strs = re.findall('\{.*?\}',pose_data_str)
    pose_dict = ast.literal_eval(pose_data_strs[0])
    
    tx = pose_dict[key][0]
    ty = pose_dict[key][1]
    tz = pose_dict[key][2]

    rx = 0
    ry = 0
    rz = pose_dict[key][5]
    result = [tx,ty,tz,rx,ry,rz]

    return(result)

def parse_data(msg_data, RB5_key):
    
    msg_input = str(msg_data).split("--")
    result = handle_pose(WB0_key, msg_input[1:])
    return(result)


host = raw_input("enter IP address: ")
#host = "192.168.82.194"
port = 1236
ThreadCount = 0

RB5_key = 9
#host is always server (reciever IP)

FILEPATH = "/home/uoroswork/catkin_ws/src/uav_nav_ops/scripts"
FILENAME = "mocap_to_drone_world.txt"
TARGET_FILE = "mocap_to_drone_world.txt"

def Main():
   
    print(socket.gethostname())

    mySocket = socket.socket()
    mySocket.bind((host,port))

    mySocket.listen(1)
    conn, addr = mySocket.accept()
    print ("Connection from: " + str(addr))
    f = open(TARGET_FILE, "w")
    while True:
            data = conn.recv(1024).decode()
            if not data:
                    break
            try:
                msg = data.split("--")
                msg_input = str(msg[1:])

                pose_data_strs = re.findall('\{.*?\}',msg_input)
                pose_dict = ast.literal_eval(pose_data_strs[0])
                RB5_transform = pose_dict[RB5_key]
                print(RB5_transform)
                print(" ")

                
                for coord in RB5_transform:
                    print(coord)
                    f.write(str(coord)+"\n")

                f.close()

            except:
                print("no data")
            
    conn.close()
    exit()

# if __name__ == '__main__':
#     Main()
Main()
