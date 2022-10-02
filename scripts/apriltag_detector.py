#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
import apriltag
#https://pyimagesearch.com/2020/11/02/apriltag-with-python/
#https://pyimagesearch.com/2016/03/28/measuring-size-of-objects-in-an-image-with-opencv/

#message imports
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, Vector3

#subscribed topics
T_CAM_IMG = "/tracking"

#published topics
T_TAG_POS = "uav/tag_pos"

#constants
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)

#needs 20 pixels to detect april tag

#node
class image_converter:
    
    def __init__(self):
        #important inits
        rospy.init_node('apriltag_detector', anonymous=True)
        self.bridge = CvBridge()

        #subscribers
        self.image_sub = rospy.Subscriber(T_CAM_IMG,Image,self.callback)

        #publishers
        self.tag_pos_pub = rospy.Publisher(T_TAG_POS, Vector3, queue_size=10)

    
    def callback(self,data):
        tag_pos = Vector3()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image = cv_image
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            results = detector.detect(gray)
            print("test")
            for r in results:
              # extract the bounding box (x, y)-coordinates for the AprilTag
              # and convert each of the (x, y)-coordinate pairs to integers
              (ptA, ptB, ptC, ptD) = r.corners
              ptB = (int(ptB[0]), int(ptB[1]))
              ptC = (int(ptC[0]), int(ptC[1]))
              ptD = (int(ptD[0]), int(ptD[1]))
              ptA = (int(ptA[0]), int(ptA[1]))
              # draw the bounding box of the AprilTag detection
              cv2.line(image, ptA, ptB, (0, 255, 0), 2)
              cv2.line(image, ptB, ptC, (0, 255, 0), 2)
              cv2.line(image, ptC, ptD, (0, 255, 0), 2)
              cv2.line(image, ptD, ptA, (0, 255, 0), 2)
              # draw the center (x, y)-coordinates of the AprilTag
              (cX, cY) = (int(r.center[0]), int(r.center[1]))
              cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
              # draw the tag family on the image
              tagFamily = r.tag_family.decode("utf-8")
              cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
              print("[INFO] tag family: {}".format(tagFamily))
              
              tag_pos.x = cX
              tag_pos.y = cY
              tag_pos.z = np.sqrt((int(ptB[0])-int(ptC[0]))**2 + (int(ptB[1])-int(ptC[1]))**2)

            self.tag_pos_pub.publish(tag_pos)
            print(tag_pos)
            print(" ")

            #cv2.imshow("Image window", cv_image)
            #cv2.waitKey(3)
        except CvBridgeError as e:
            print("test")
            pass

        except CvBridgeError as e:
          print(e)

def main(args):
  ic = image_converter()
  #rospy.init_node('image_process_attachment', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)