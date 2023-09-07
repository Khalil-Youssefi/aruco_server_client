#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import socket
import cv2
from cv_bridge import CvBridge
from time import time
import threading

# Server settings
server_ip = "192.168.4.126"
server_port = 54323

# Create a TCP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to a specific IP and port
server_socket.bind((server_ip, server_port))

# Listen for incoming connections
server_socket.listen(1)
print("Image server is listening on {}:{}".format(server_ip, server_port))

image = None

def image_callback(msg):
    global image
    image = msg

def camera_server():
    # Initialize the ROS node
    rospy.init_node('fast_camera_server', anonymous=True)
    rospy.Subscriber('/camera/image', CompressedImage, image_callback)

    bridge = CvBridge()
    while not rospy.is_shutdown():
        # Accept a connection from a client
        if image is None:
            continue

        client_socket, client_address = server_socket.accept()
        rospy.logdebug("Connection established with {}:{}".format(client_address[0], client_address[1]))

        # Receive data from the client
        data = client_socket.recv(1024).decode()
        if data == "send":
            try:
                cv2_image = bridge.compressed_imgmsg_to_cv2(image)
                _, image_data = cv2.imencode(".jpg", cv2_image)
                image_bytes = image_data.tobytes()
                client_socket.sendall(image_bytes)
            except Exception as e:
                rospy.logerr("Error occurred while sending the image:", str(e))
                no_camera_detected = True
                break

        # Close the connection
        client_socket.close()

if __name__ == '__main__':
    try:
        camera_server()
    except rospy.ROSInterruptException as e:
        print(str(e))
