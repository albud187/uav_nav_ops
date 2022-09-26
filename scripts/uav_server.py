#!/usr/bin/env python
#_*_ coding: utf8 _*_
import socket
import rospy
import ast
import re
#message imports
from geometry_msgs.msg import Vector3, Pose

#subscribed topics

#published topics
T_WB1_POSE = "/WB1_pose"
T_HC_POSE = "/HC_pose"
TAG_POS = "/tag_pos"
#!/usr/bin/env python
#_*_ coding: utf8 _*_
import socket
import rospy
import ast
import re
#message imports
from geometry_msgs.msg import Vector3, Pose

#subscribed topics

#published topics
T_WB1_POSE = "/WB1_pose"
T_HC_POSE = "/HC_pose"
TAG_POS = "/tag_pos"

#global variables
HC_pose = Pose()
WB1_pose = Pose()
tag_pos = Vector3()
#constants
HOST = "192.168.82.129"
PORT = 1236

#functions

def handle_pose_data(data):
    
    pose_data_strs = re.findall('\{.*?\}',data)
    result_dict = ast.literal_eval(pose_data_strs[0])
    pose_data = result_dict
    return pose_data

def handle_pose(key, pose_data):
    pose = Pose()
    
    pose.position.x = pose_data[key][0]
    pose.position.y = pose_data[key][1]
    pose.position.z = pose_data[key][2]
    
    pose.orientation.x = pose_data[key][3]
    pose.orientation.y = pose_data[key][4]
    pose.orientation.z = pose_data[key][5]
    
    return(pose)

def handle_visual_servo(msg_data):
    tag_pos = Vector3()
    tag_pos.x = msg_data[0]
    tag_pos.y = msg_data[1]
    tag_pos.z = msg_data[2]

    return(tag_pos)

def test_parse_data(msg_data):
    print(msg_data)

def parse_data(msg_data):
    global HC_pose
    global WB1_pose
    global tag_pos
    HC_key = 0
    WB1_key = 1
    msg_input = msg_data.split("--"):
    if msg_input[0] == "MCP":
        try:
            HC_pose = handle_pose(HC_key, msg_input)
            WB1_pose = handle_pose(WB1_key, msg_input)
        except:
            pass
    if msg_input[0] == "TAP":
        try:
            tag_pos = handle_visual_servo(msg_input)
        except:
            pass

def multi_object_server():
    
    print(socket.gethostname())
    mySocket = socket.socket()
    mySocket.bind((HOST,PORT))
    mySocket.listen(1)
    conn, addr = mySocket.accept()
    print("connection from: " + str(addr))

    rospy.init_node("uav_server", anonymous = False)

    #subscribers

    #publishers
    HC_pose_pub = rospy.Publisher(T_HC_POSE, Pose, queue_size = 10)
    WB1_pose_pub = rospy.Publisher(T_WB1_POSE, Pose, queue_size = 10)
    
    while not rospy.is_shutdown():
        
        data = conn.recv(1024).decode()

        try:
          
            pose_data = handle_pose_data(data)
            HC_key = 0
            WB1_key = 1

            HC_pose = handle_pose(HC_key, pose_data)
            WB1_pose = handle_pose(WB1_key, pose_data)

            HC_pose_pub.publish(HC_pose)
            
            WB1_pose_pub.publish(WB1_pose)
                
        except:
           
            print("error")

if __name__ == '__main__':
    try:
        multi_object_server()
    except rospy.ROSInterruptException:
        pass
tag_pos = Vector3()
#constants
HOST = "192.168.82.129"
PORT = 1236

#functions

def handle_pose_data(data):
    
    pose_data_strs = re.findall('\{.*?\}',data)
    result_dict = ast.literal_eval(pose_data_strs[0])
    pose_data = result_dict
    return pose_data

def handle_pose(key, pose_data):
    pose = Pose()
    
    pose.position.x = pose_data[key][0]
    pose.position.y = pose_data[key][1]
    pose.position.z = pose_data[key][2]
    
    pose.orientation.x = pose_data[key][3]
    pose.orientation.y = pose_data[key][4]
    pose.orientation.z = pose_data[key][5]
    
    return(pose)

def handle_visual_servo(msg_data):
    tag_pos = Vector3()
    tag_pos.x = msg_data[0]
    tag_pos.y = msg_data[1]
    tag_pos.z = msg_data[2]

    return(tag_pos)

def test_parse_data(msg_data):
    print(msg_data)

def parse_data(msg_data):
    global HC_pose
    global WB1_pose
    global tag_pos
    HC_key = 0
    WB1_key = 1
    msg_input = msg_data.split("--"):
    if msg_input[0] == "MCP":
        try:
            HC_pose = handle_pose(HC_key, msg_input)
            WB1_pose = handle_pose(WB1_key, msg_input)
        except:
            pass
    if msg_input[0] == "TAP":
        try:
            tag_pos = handle_visual_servo(msg_input)
        except:
            pass

def multi_object_server():
    
    print(socket.gethostname())
    mySocket = socket.socket()
    mySocket.bind((HOST,PORT))
    mySocket.listen(1)
    conn, addr = mySocket.accept()
    print("connection from: " + str(addr))

    rospy.init_node("uav_server", anonymous = False)

    #subscribers

    #publishers
    HC_pose_pub = rospy.Publisher(T_HC_POSE, Pose, queue_size = 10)
    WB1_pose_pub = rospy.Publisher(T_WB1_POSE, Pose, queue_size = 10)
    
    while not rospy.is_shutdown():
        
        data = conn.recv(1024).decode()

        try:
          
            pose_data = handle_pose_data(data)
            HC_key = 0
            WB1_key = 1

            HC_pose = handle_pose(HC_key, pose_data)
            WB1_pose = handle_pose(WB1_key, pose_data)

            HC_pose_pub.publish(HC_pose)
            
            WB1_pose_pub.publish(WB1_pose)
                
        except:
           
            print("error")

if __name__ == '__main__':
    try:
        multi_object_server()
    except rospy.ROSInterruptException:
        pass