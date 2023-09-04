#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospkg
from pynput import keyboard
import threading
import time

def publish_video_frames():
    global current_frame
    global block,rblock
    # Initialize the ROS node
    rospy.init_node('video2topic', anonymous=True)
    
    rospack = rospkg.RosPack()
    video_path = rospack.get_path('aruco_marker_localization') + '/src/scripts/recordings/sample_video.mp4'
    topic_name = '/cameraclient/image'

    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        rospy.logerr("Error: Unable to open video file.")
        return

    frames = []
    while True:
        ret, frame = cap.read()
        if not ret:
            rospy.loginfo("Video is loaded.")
            break
        frames.append(frame)
    cap.release()

    # Create a publisher for publishing image frames
    image_pub = rospy.Publisher(topic_name, Image, queue_size=10)
    
    # Initialize the CvBridge
    bridge = CvBridge()

    rate = rospy.Rate(10)
    current_frame = 0
    framecount = len(frames)
    while not rospy.is_shutdown():
            with keyboard.Events() as events:
                event = events.get(rate.sleep_dur.to_sec())
                if event is not None:
                    if event.key == keyboard.Key.page_down:
                        current_frame -= 1
                    elif event.key == keyboard.Key.page_up:
                        current_frame += 1
                    print(f'showing frame {current_frame}/{framecount}')
            frame = frames[current_frame]
            
            # Convert the frame to a ROS Image message
            image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

            # Publish the frame to the ROS topic
            image_pub.publish(image_msg)
            rate.sleep()

    rospy.loginfo("Video publishing node has finished.")
    rospy.signal_shutdown("Video publishing node finished.")

if __name__ == '__main__':   
    try:
        publish_video_frames()
    except rospy.ROSInterruptException:
        pass
