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

from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


rospack = rospkg.RosPack()
calibration_matrix_file = rospack.get_path('aruco_marker_localization') + '/src/scripts/calibration_matrix.yaml'
with open(calibration_matrix_file, 'r') as stream:
    data_loaded = yaml.safe_load(stream)
cameraMatrix = np.array(data_loaded['camera_matrix'])
distcoeff = np.array(data_loaded['dist_coeff'])
detectorParams = aruco.DetectorParameters()
dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
detector = aruco.ArucoDetector(dictionary, detectorParams)
objPoints = np.zeros((4, 1, 3), dtype=np.float32)
marker_size = 0.10
objPoints[0] = np.array([-marker_size/2, -marker_size/2, 0])
objPoints[1] = np.array([marker_size/2, -marker_size/2, 0])
objPoints[2] = np.array([marker_size/2, marker_size/2, 0])
objPoints[3] = np.array([-marker_size/2, marker_size/2, 0])
parent_frame_id = "world_frame"
child_frame_id = "camera_frame"
broadcaster = broadcaster = tf.TransformBroadcaster()
detection_publisher = rospy.Publisher("/detected_marker", Image)
cvbridge = CvBridge()
def compressed_image_callback(msg):
    global cameraMatrix, distcoeff, dictionary
    global detector,objPoints,marker_size,parent_frame_id,child_frame_id,broadcaster,detection_publisher
    try:
        # Convert compressed image to numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray)

        if len(markerCorners) > 0:
            # print('publishing ...')
            retval, rvec, tvec = cv2.solvePnP(objPoints, markerCorners[0],cameraMatrix,distcoeff)
            print('corners:',markerCorners[0])
            label = ['tl','tr','br','bl']
            for i in range(4):
                xy = (int(markerCorners[0][0][i][0]),int(markerCorners[0][0][i][1]))
                cv2.circle(image,xy,5,(0,0,255),3)
                cv2.putText(image, label[i], xy, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

            cv2.circle(image,(0,0),5,(255,0,255),3)
            cv2.putText(image, '(0,0)', (0,0), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
            print('tvec:',tvec)
            # print(tvec)

            # # Convert rotation vector to rotation matrix
            # R, _ = cv2.Rodrigues(rvec)

            # # Calculate the inverse rotation matrix and inverse translation vector
            # R_inv = np.linalg.inv(R)
            # tvec_inv = np.dot(-R_inv, tvec)

            # # Convert the inverse rotation matrix to a rotation vector
            # rvec_inv, _ = cv2.Rodrigues(R_inv)

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
            outputImage = image + 0
            outputImage = cv2.drawFrameAxes(outputImage, cameraMatrix, None, rvec, tvec, 0.01)
            detection_publisher.publish(cvbridge.cv2_to_imgmsg(outputImage))
    except Exception as e:
        rospy.logerr("Error while converting compressed image: %s", str(e))

def main():
    rospy.init_node("virtualcamera", anonymous=True)
    rospy.Subscriber("/virtual_aruco_camera/compressed", CompressedImage, compressed_image_callback)
    rospy.spin()

if __name__ == "__main__":
    main()