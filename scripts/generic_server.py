#!/usr/bin/env python
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
T_WB1_POSE = "/WB1_pose"
T_HC_POSE = "/HC_pose"
T_TAG_POS = "/tag_pos"

#global variables
HC_pose = Pose()
WB1_pose = Pose()
tag_pos = Vector3()

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

def handle_visual_servo(msg_data):
    tag_pos = Vector3()
    
    
    tag_pos_tuple = re.findall('\(.*?\)',msg_data[1])[0]
    
    try:
        tag_pos_tuple_num = ast.literal_eval(tag_pos_tuple)
        tag_pos.x = tag_pos_tuple_num[0]
        tag_pos.y = tag_pos_tuple_num[1]
        tag_pos.z = tag_pos_tuple_num[2]
        
    except:
        print("cannot convert")
    # tag_pos = Vector3()TAP
    # tag_pos.x = msg_data[0]
    # tag_pos.y = msg_data[1]
    # tag_pos.z = msg_data[2]

    return(tag_pos)

def test_parse_data(msg_data):
    print(msg_data)

def parse_data(msg_data):

    
    HC_key = 0
    WB1_key = 1
    msg_input = str(msg_data).split("--")

    #publishers
    HC_pose_pub = rospy.Publisher(T_HC_POSE, Pose, queue_size = 10)
    WB1_pose_pub = rospy.Publisher(T_WB1_POSE, Pose, queue_size = 10)
    tag_pos_pub = rospy.Publisher(T_TAG_POS, Vector3, queue_size = 10)
    if msg_input[0] == "b'MCP":
        try:
           
            HC_pose = handle_pose(HC_key, msg_input[1:])
            WB1_pose = handle_pose(WB1_key, msg_input[1:])
            
            HC_pose_pub.publish(HC_pose)
            WB1_pose_pub.publish(WB1_pose)
        except:
            pass
    if msg_input[0] == "b'tag":
        try:
            tag_pos = handle_visual_servo(msg_input)
            tag_pos_pub.publish(tag_pos)
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
host = "192.168.82.129"
port = 1236
ThreadCount = 0

try:
    ServerSideSocket.bind((host, port))
except socket.error as e:
    print(str(e))
print('Socket is listening..')
ServerSideSocket.listen(5)

#main loop
def uav_server():
    global HC_pose
    global WB1_pose
    global tag_pos
    global ThreadCount
    rospy.init_node("uav_server", anonymous = False)
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
        uav_server()
    except rospy.ROSInterruptException:
        pass