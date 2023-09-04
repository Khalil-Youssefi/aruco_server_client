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

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def publish_transform():

    rospy.init_node('cameraclient')
    image_publisher = rospy.Publisher('/cameraclient/image', Image, queue_size=10)

    rate = rospy.Rate(30)

    server_ip = "192.168.1.104"
    server_port = 54323

    bridge = CvBridge()
    while not rospy.is_shutdown():
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((server_ip, server_port))
            client_socket.sendall("send".encode())

            image_bytes = b""
            while True:
                data = client_socket.recv(1024)
                if not data:
                    break
                image_bytes += data
                
            nparr = np.frombuffer(image_bytes, np.uint8)
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            ros_img_msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
            image_publisher.publish(ros_img_msg)

            client_socket.close()
        
        except Exception as e:
            rospy.logerr(e)
            
if __name__ == "__main__":
    try:
        publish_transform()
    except rospy.ROSInterruptException:
        pass
