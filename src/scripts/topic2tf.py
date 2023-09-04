#!/usr/bin/env python

import rospy
import cv2
from cv2 import aruco
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import yaml
import rospkg
import subprocess
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def get_reprojection_error(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec):
    projected_points, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)

    total_error = 0.0
    for i in range(len(object_points)):
        image_points = image_points.reshape((4,2))
        projected_points = projected_points.reshape((4,2))
        error = np.linalg.norm(np.array(image_points[i]) - np.array(projected_points[i]))
        total_error += error * error

    rerror = total_error / len(object_points)
    return rerror

def image_callback(image_msg):
    global broadcaster,cameraMatrix,distcoeff,detector,objPoints,image_publisher,bridge

    image = bridge.imgmsg_to_cv2(image_msg)
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image = gray

    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray)

    if len(markerCorners) > 0:
        # print('publishing ...')
        retval, rvec, tvec = cv2.solvePnP(objPoints, markerCorners[0],cameraMatrix,distcoeff)

        estimation_error = get_reprojection_error(objPoints,markerCorners[0],cameraMatrix,distcoeff,rvec,tvec)
        corners = np.array(markerCorners).reshape((4,2))
        object_error = (estimation_error / np.linalg.norm(corners[0]-corners[2])) * (np.linalg.norm(tvec) / marker_size)

        if (object_error > 0.2):
            print(f'ignoring detection with big error of {object_error}')
        else:
            print(f'accepting detection with big error of {object_error}')


            cv2.aruco.drawDetectedMarkers(image, markerCorners, markerIds)

            axis_length = marker_size/2
            axis_points = np.array([[0, 0, 0],               # Origin
                                    [axis_length, 0, 0],     # X-axis
                                    [0, axis_length, 0],     # Y-axis
                                    [0, 0, axis_length]],    # Z-axis
                                    dtype=np.float32)
            img_points, _ = cv2.projectPoints(axis_points, rvec, tvec, cameraMatrix, distcoeff)
            img_points = img_points.astype(int)
            origin = tuple(img_points[0].ravel())
            x_axis = tuple(img_points[1].ravel())
            y_axis = tuple(img_points[2].ravel())
            z_axis = tuple(img_points[3].ravel())
            cv2.line(image, origin, x_axis, (0, 0, 255), 5)
            cv2.line(image, origin, y_axis, (0, 255, 0), 5)
            cv2.line(image, origin, z_axis, (255, 0, 0), 5)

            ros_img_msg = bridge.cv2_to_imgmsg(image)   
            image_publisher.publish(ros_img_msg)

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

    ros_img_msg = bridge.cv2_to_imgmsg(image)   
    image_publisher.publish(ros_img_msg)
    
if __name__ == "__main__":
    try:
        rospy.init_node('topic2tf', anonymous=True)

        parent_frame_id = "world_frame"
        child_frame_id = "camera_frame"
        broadcaster = tf.TransformBroadcaster()

        rospack = rospkg.RosPack()
        calibration_matrix_file = rospack.get_path('aruco_marker_localization') + '/src/scripts/calibration_matrix.yaml'
        with open(calibration_matrix_file, 'r') as stream:
            data_loaded = yaml.safe_load(stream)
        cameraMatrix = np.array(data_loaded['camera_matrix'])
        distcoeff = np.array(data_loaded['dist_coeff'])

        # marker_size = 10.2
        marker_size = 0.19
        
        detectorParams = aruco.DetectorParameters()
        detectorParams.minMarkerPerimeterRate = 0.03
        detectorParams.maxMarkerPerimeterRate = 4.0
        detectorParams.polygonalApproxAccuracyRate = 0.005 #0.05
        detectorParams.minCornerDistanceRate = 0.005 #0.05
        detectorParams.minOtsuStdDev = 5 # 5
        detectorParams.perspectiveRemovePixelPerCell = 1 # 4
        detectorParams.perspectiveRemoveIgnoredMarginPerCell = 0.1 #0.13
        detectorParams.maxErroneousBitsInBorderRate = 0.35 #0.35
        detectorParams.errorCorrectionRate = 0.5 # 0.6
        detectorParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX # CORNER_REFINE_NONE | CORNER_REFINE_SUBPIX | CORNER_REFINE_CONTOUR
        # for subpix
        detectorParams.cornerRefinementWinSize = 5 # 5
        detectorParams.cornerRefinementMaxIterations = 1000 # 30
        detectorParams.cornerRefinementMinAccuracy = 0.01 # 0.1
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        detector = aruco.ArucoDetector(dictionary, detectorParams)

        objPoints = np.zeros((4, 1, 3), dtype=np.float32)
        objPoints[0] = np.array([-marker_size/2, -marker_size/2, 0])
        objPoints[1] = np.array([marker_size/2, -marker_size/2, 0])
        objPoints[2] = np.array([marker_size/2, marker_size/2, 0])
        objPoints[3] = np.array([-marker_size/2, marker_size/2, 0])

        bridge = CvBridge()

        image_publisher = rospy.Publisher('/topic2tf/image', Image, queue_size=10)

        rospy.Subscriber("/cameraclient/image", Image, image_callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
