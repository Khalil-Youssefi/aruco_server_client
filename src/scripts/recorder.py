#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import cv2
import os
import rospkg

def generate_video_name():
    return f"video_{time.strftime('%Y%m%d%H%M%S', time.gmtime())}.mp4"

def image_callback(image_msg):
    global bridge, out, video_name, recording

    try:
        frame = bridge.imgmsg_to_cv2(image_msg)
    
        if out is None:
            out = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'mp4v'), 15, (frame.shape[1],frame.shape[0]))
        else:
            if recording:
                out.write(frame)
                print('Writing ..')
    except  Exception as e:
        rospy.logerr(e)
    
if __name__ == "__main__":
    try:
        rospy.init_node('recorder', anonymous=True)
        
        bridge = CvBridge()
        
        video_name = generate_video_name()
        print(f'Video will be saved to {video_name}')
        rospack = rospkg.RosPack()
        os.makedirs(rospack.get_path('aruco_marker_localization') + '/src/scripts/recordings') if not os.path.exists(rospack.get_path('aruco_marker_localization') + '/src/scripts/recordings') else None
        video_name = rospack.get_path('aruco_marker_localization') + '/src/scripts/recordings/' + video_name

        out = None

        rospy.Subscriber("/cameraclient/image", Image, image_callback)

        recording = False
        changed = True
        while True:
            if not recording:
                print('Not recording. Press Enter key to start recording.')
                if input()=='q':
                    break
                recording = True
            if recording:
                print('Recording... Press Enter key to start recording.')
                if input()=='q':
                    break
                recording = False
        if out is not None:
            out.release()
        print('Saving video and exiting.')

    except Exception as e:
        rospy.logerr(e)