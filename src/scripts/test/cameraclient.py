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

    broadcaster = tf.TransformBroadcaster()

    # Set the parent and child frame IDs
    parent_frame_id = "world_frame"
    child_frame_id = "camera_frame"

    rate = rospy.Rate(10)

    server_ip = "192.168.43.186"
    # server_ip = "localhost"
    server_port = 54323

    rospack = rospkg.RosPack()
    calibration_matrix_file = rospack.get_path('aruco_marker_localization') + '/src/scripts/calibration_matrix.yaml'
    with open(calibration_matrix_file, 'r') as stream:
        data_loaded = yaml.safe_load(stream)
    cameraMatrix = np.array(data_loaded['camera_matrix'])
    distcoeff = np.array(data_loaded['dist_coeff'])

    marker_size = 0.52

    detectorParams = aruco.DetectorParameters()
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
    detector = aruco.ArucoDetector(dictionary, detectorParams)


    objPoints = np.zeros((4, 1, 3), dtype=np.float32)
    objPoints[0] = np.array([-marker_size/2.0, marker_size/2.0, 0])
    objPoints[1] = np.array([marker_size/2.0, marker_size/2.0, 0])
    objPoints[2] = np.array([marker_size/2.0, -marker_size/2.0, 0])
    objPoints[3] = np.array([-marker_size/2.0, -marker_size/2.0, 0])

    bridge = CvBridge()
    image_publisher = rospy.Publisher('/cameraclient/image', Image, queue_size=10)
    while not rospy.is_shutdown():

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

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray)

        if len(markerCorners) > 0:
            # print('publishing ...')
            retval, rvec, tvec = cv2.solvePnP(objPoints, markerCorners[0],cameraMatrix,distcoeff)


            quaternion = tf.transformations.quaternion_from_euler(rvec[0], rvec[1], rvec[2])

            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = parent_frame_id
            pose = PoseStamped(header=header)
            pose.pose.position.x = tvec[0]
            pose.pose.position.y = tvec[1]
            pose.pose.position.z = tvec[2]
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            broadcaster.sendTransform((tvec[0], tvec[1], tvec[2]),
                                    quaternion,
                                    header.stamp,
                                    child_frame_id,
                                    parent_frame_id)

            rate.sleep()

if __name__ == "__main__":
    try:
        publish_transform()
    except rospy.ROSInterruptException:
        pass
