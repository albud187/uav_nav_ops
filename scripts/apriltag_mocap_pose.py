#!/usr/bin/env python3
#_*_ coding: utf8 _*_
import socket
import rospy
import ast
import re
import json
from _thread import *
#message imports
from geometry_msgs.msg import Vector3, Pose

#subscribed topics

#published topics
T_MOCAP_pose = "/uav/pose"

#global variables
MOCAP_pose = Pose()

#functions
def handle_pose_data(data):
    
    pose_data_strs = re.findall('\{.*?\}',data)
    result_dict = ast.literal_eval(pose_data_strs[0])
    pose_data = result_dict
    return pose_data

def handle_pose(key, pose_data):
    pose = Pose()
    pose_data_str = str(pose_data)
    pose_data_strs = re.findall('\{.*?\}',pose_data_str)
    pose_dict = ast.literal_eval(pose_data_strs[0])
    
    pose.position.x = pose_dict[key][0]
    pose.position.y = pose_dict[key][1]
    pose.position.z = pose_dict[key][2]

    pose.orientation.x = pose_dict[key][3]
    pose.orientation.y = pose_dict[key][4]
    pose.orientation.z = pose_dict[key][5]
    
    return(pose)

def test_parse_data(msg_data):
    print(msg_data)

def parse_data(msg_data):
    drone_key = 9

    msg_input = str(msg_data).split("--")

    #publishers
    MOCAP_pose_pub = rospy.Publisher(T_MOCAP_pose, Pose, queue_size = 10)
    if msg_input[0] == "b'MCP":
        try:
           
            MOCAP_pose = handle_pose(drone_key, msg_input[1:])
            
            MOCAP_pose_pub.publish(MOCAP_pose)
        except:
            pass

    else:
        print("undef msg")
        print(msg_input)
        print(" ")

def multi_threaded_client(connection):
    connection.send(str.encode('Server is working:'))
    while True:
        data = connection.recv(2048)
        parse_data(data)
        #response = 'Server message: ' + data.decode('utf-8')
        if not data:
            break
        #connection.sendall(str.encode(response))
    connection.close()

#init socket
ServerSideSocket = socket.socket()
host = "192.168.82.194"
port = 1236
ThreadCount = 0

try:
    ServerSideSocket.bind((host, port))
except socket.error as e:
    print(str(e))
print('Socket is listening..')
ServerSideSocket.listen(5)

#main loop
def multi_object_server():
    
    global MOCAP_pose
    global tag_pos
    global ThreadCount
    rospy.init_node("main_server", anonymous = False)
    #subscribers

    #publishers
    
    while not rospy.is_shutdown():
        
        Client, address = ServerSideSocket.accept()
        print('Connected to: ' + address[0] + ':' + str(address[1]))
        start_new_thread(multi_threaded_client, (Client, ))
        ThreadCount += 1
        print('Thread Number: ' + str(ThreadCount))

    ServerSideSocket.close()

if __name__ == '__main__':
    try:
        multi_object_server()
    except rospy.ROSInterruptException:
        pass