#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospkg

def publish_video_frames():
    # Initialize the ROS node
    rospy.init_node('video2topic', anonymous=True)
    
    rospack = rospkg.RosPack()
    video_path = rospack.get_path('aruco_marker_localization') + '/src/scripts/recordings/sample_video.mp4'
    topic_name = '/cameraclient/image'

    # Create a publisher for publishing image frames
    image_pub = rospy.Publisher(topic_name, Image, queue_size=10)
    
    # Initialize the CvBridge
    bridge = CvBridge()

    while not rospy.is_shutdown():
        # Open the video file at the beginning of each loop iteration
        cap = cv2.VideoCapture(video_path)

        if not cap.isOpened():
            rospy.logerr("Error: Unable to open video file.")
            return

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                rospy.loginfo("End of video reached. Restarting...")
                break
            
            # Convert the frame to a ROS Image message
            image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

            # Publish the frame to the ROS topic
            image_pub.publish(image_msg)

            rate.sleep()

        # Release the video capture object
        cap.release()

    rospy.loginfo("Video publishing node has finished.")
    rospy.signal_shutdown("Video publishing node finished.")

if __name__ == '__main__':   
    try:
        publish_video_frames()
    except rospy.ROSInterruptException:
        pass
