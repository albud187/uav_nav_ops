#package imports
import numpy as np
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf.transformations import quaternion_conjugate as qc, quaternion_multiply as qm
import time
import image_geometry as ig
import roslib
import tf2_ros
import socket
#message imports
from sensor_msgs.msg import LaserScan, CameraInfo
from geometry_msgs.msg import Pose2D, Vector3, Pose
#subscribed topics
T_TAG_POS = "uav/tag_pos"
T_UAV_POSE = "/uav/pose"
T_UAV_CAM_INFO = "/uav/down_cam/camera/camera_info"
#published topics

CAMERA_MAT = np.array([[2.8054458697503384*10**2, 0, 3.4675256521746036*10**2],
                        [2.8054458697503384*10**2,  2.1038132491104975*10**2, 0],
                        [0,0,1]])

#constants
CAMERA_CENTER_X = 3.4675256521746036*10**2
CAMERA_CENTER_Y = 2.1038132491104975*10**2
FOCAL_LEN = 2.8054458697503384*10**2
WP_DIST_TH = 1
#globals
tag_pos = Vector3()
uav_pose = Pose()
uav_pcm = ig.PinholeCameraModel()
all_waypoints =[]

#functions
def handle_uav_pose(msg):
    global uav_pose
    uav_pose = msg

def handle_tag_pos(msg):
    global tag_pos
    tag_pos = msg

def ROT_mat(rx, ry, rz):
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

def HT_mat(tx, ty, tz, rx, ry, rz):
    R11 = np.cos(ry)*np.cos(rz)
    R12 = np.sin(rx)*np.sin(ry)*np.cos(rz) - np.cos(rx)*np.sin(rz)
    R13 = np.cos(rx)*np.sin(ry)*np.cos(rz) + np.sin(rx)*np.sin(rz)
    
    R21 = np.cos(ry)*np.sin(rz)
    R22 = np.sin(rx)*np.sin(ry)*np.sin(rz) + np.cos(rx)*np.cos(rz)
    R23 = np.cos(rx)*np.sin(ry)*np.sin(rz) - np.sin(rx)*np.cos(rz)
    
    R31 = -np.sin(ry)
    R32 = np.sin(rx)*np.sin(ry)
    R33 = np.cos(rx)*np.cos(ry)
    result = np.array([[R11, R12, R13, tx],
                       [R21, R22, R23, ty],
                       [R31, R32, R33, tz]
                       [0,0,0,1]])

def uv_cam(px, py, cx, cy, f):
    vx = px - cx
    vy = py - cy
    vz = f
    
    magnitude = np.sqrt(vx**2+vy**2+vz**2)
    
    result = (1/magnitude)*np.array([[vx],
                                    [vy],
                                    [vz]])
    return result

def find_xform(tf_buffer, frame1, frame2):
    xform = tf_buffer.lookup_transform(frame1, frame2, rospy.Time(0), rospy.Duration(10))
    tx = xform.transform.translation.x
    ty = xform.transform.translation.y
    tz = xform.transform.translation.z

    orientation_quat = (xform.transform.rotation.x, xform.transform.rotation.y, xform.transform.rotation.z, xform.transform.rotation.w )
    rx, ry, rz = euler_from_quaternion(orientation_quat)
    result = (tx,ty,tz,rx,ry,rz)

    return(result)

def calc_tag_pos_world(transform, tag_pos):
    tx = transform[0]
    ty = transform[1]
    tz = transform[2]
    rx = transform[3]
    ry = transform[4]
    rz = transform[5]
    px = tag_pos.x
    py = tag_pos.y
    cx = CAMERA_CENTER_X
    cy = CAMERA_CENTER_Y
    f = FOCAL_LEN
    ROT_world_cam = ROT_mat(rx, ry, rz)
    uv = uv_cam(px, py, cx, cy, f)
    uv_world = ROT_world_cam @ uv
    k_z0 = -tz/uv_world[2]
  
    tag_position = (tx + float(k_z0*uv_world[0]), ty + float(k_z0*uv_world[1]))
    return tag_position

def calculate_distance(P1,P2):
    distance = np.sqrt((P1[0]-P2[0])**2+(P1[1]-P2[1])**2)
    return distance

def new_wp_check(waypoint_list, new_wp, TH):
    if len(waypoint_list) == 0:
        return True
    else:
        distances = []
        for wp in waypoint_list:
            distance = calculate_distance(new_wp, wp)
            distances.append(distance)
            
        if min(distances) > TH:
            result = True
        else:
            result = False
        return result

f = open("wp-list_master.txt", "w")
f.close()
def main_loop():
    #init
    global tag_pos
    global uav_pcm
    global all_waypoints
    rospy.init_node("uav_waypoint_img_calculator", anonymous = False)
    tf_listener = tf.TransformListener()
    rate = rospy.Rate(20)
    
    tf_buffer = tf2_ros.Buffer()
    tf2_listener = tf2_ros.TransformListener(tf_buffer)


    
    #f.write("test")
    
    #subscribers
    rospy.Subscriber(T_UAV_POSE, Pose, handle_uav_pose)
    rospy.Subscriber(T_TAG_POS, Vector3, handle_tag_pos)
    rospy.Subscriber(T_UAV_CAM_INFO, CameraInfo, handle_pcm)
    #publishers
    
    while not rospy.is_shutdown():
        

        #xform = tf_buffer.lookup_transform(WORLD_FRAME, UAV_CAMERA_FRAME, rospy.Time(0), rospy.Duration(10))
        if tag_pos.x != 0:
            try:
                xform = find_xform(tf_buffer, WORLD_FRAME, UAV_CAMERA_FRAME)
                tag_xy = calc_tag_pos_world(xform, tag_pos)
                if new_wp_check(all_waypoints, tag_xy, WP_DIST_TH) == True:
                    all_waypoints.append(tag_xy)
                    f = open("wp-list_master.txt", "a")
                    f.write(str(tag_xy[0])+","+str(tag_xy[1])+","+"\n")
                    f.close()
                    print("writing")
                    print(all_waypoints)
            except:
                pass
    
if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass