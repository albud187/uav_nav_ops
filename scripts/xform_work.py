import os
import numpy as np

FILEPATH = "/home/uoroswork/catkin_ws/src/uav_nav_ops/scripts"
FILENAME = "mocap_to_drone_world.txt"
TARGET_FILE = os.path.join(FILEPATH, FILENAME)
# xform = (1.11, 2.22, 3.33, 4.44, 5.55, 6.66)



# f = open(TARGET_FILE, "w")

# for coord in xform:
#     print(coord)
#     f.write(str(coord)+"\n")

# f.close()
# print(os.getcwd())

def read_transform(path):
    f = open(path, "r")
    lines = f.readlines()
    result = (float(lines[0]), float(lines[1]), float(lines[2]), float(lines[3]), float(lines[4]), float(lines[5]))
    return result
def xform_HT(xform):

    tx = xform[0]
    ty = xform[1]
    tz = xform[2]
    rx = xform[3]
    ry = xform[4]
    rz = xform[5]
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
                       [R31, R32, R33, tz],
                       [0, 0, 0, 1]])
    return result

xform = read_transform(TARGET_FILE)
HT = xform_HT(xform)
print(HT)