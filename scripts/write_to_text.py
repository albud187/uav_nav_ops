import os


FILEPATH = "/home/uoroswork/catkin_ws/src/uav_nav_ops/scripts"
FILENAME = "mocap_to_drone_world.txt"
TARGET_FILE = os.path.join(FILEPATH, FILENAME)
xform = (1.11, 2.22, 3.33, 4.44, 5.55, 6.66)



f = open(TARGET_FILE, "w")

for coord in xform:
    print(coord)
    f.write(str(coord)+"\n")

f.close()
print(os.getcwd())