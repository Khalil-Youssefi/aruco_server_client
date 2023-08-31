#!/usr/bin/env python

import rospy
import cv2
from cv2 import aruco
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import yaml
import socket
import rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


rospack = rospkg.RosPack()
CALIB = rospack.get_path('aruco_marker_localization') + '/src/scripts/calib/'
IMAGEID = 0
def compressed_image_callback(msg):
    global CALIB, IMAGEID
    try:
        # Convert compressed image to numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        cv2.imwrite(CALIB + "img_"+str(IMAGEID) + '.jpg', cv2_img)
        print(f'saved image {IMAGEID}')
        IMAGEID = IMAGEID + 1
        
    except Exception as e:
        rospy.logerr("Error while converting compressed image: %s", str(e))

def main():
    rospy.init_node("virtualcamera_calibration", anonymous=True)
    rospy.Subscriber("/virtual_aruco_camera/compressed", CompressedImage, compressed_image_callback)
    rospy.spin()

if __name__ == "__main__":
    main()