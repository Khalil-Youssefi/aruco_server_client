#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import CompressedImage,Image
from cv_bridge import CvBridge
import threading
import numpy as np

def generate_random_image(width: int, height: int):
    '''
    Produce an image from random data.

    # Parameters
    - width: Width of the image in pixel.
    - height: Height of the image in pixel.

    # Returns
    A sensor_msgs/CompressedImage object.
    '''
    # generate random pixel values
    pixels = np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)
    txtt = str(np.random.randint(0, 100))
    text_size, _ = cv2.getTextSize(txtt, cv2.FONT_HERSHEY_SIMPLEX,1,2)
    text_x = (width - text_size[0]) // 2
    text_y = (height + text_size[1]) // 2

    # write the random number on the array
    cv2.putText(pixels, txtt, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 5, (255), 10)

    # convert the pixel values to a bytes-like object
    pixels_bytes = pixels.tobytes()

    # create a new random image
    image = Image()
    image.header.stamp = rospy.Time.now()
    image.width = width
    image.height = height
    image.encoding = "rgb8"
    image.step = 3 * width
    image.data = pixels_bytes

    # convert to open cv format
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")

    # Compress the image using jpeg encoding
    _, compressed_data = cv2.imencode(".jpg", cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])

    # Create a new CompressedImage message
    compressed_image = CompressedImage()
    compressed_image.header = image.header
    compressed_image.format = "jpeg"
    compressed_image.data = np.array(compressed_data).tobytes()

    return compressed_image

# for fast capturing
# A thread that keeps 'image' up to date
capturer = cv2.VideoCapture(0)
width = 1280
height = 720
capturer.set(cv2.CAP_PROP_FRAME_WIDTH, width)
capturer.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
image = None
block_r = False
block_w = False
flag_new = False
no_camera_detected = False
def image_updater():
    global capturer, image, block_r, block_w, flag_new, no_camera_detected
    bridge = CvBridge()
    while True:
        try:
            ret, image_c = capturer.read()
            if not ret:
                raise Exception('Camera not connected! Exiting capturer frame ...')
            if not block_r:
                block_w = True
                image = bridge.cv2_to_compressed_imgmsg(image_c)
                flag_new = True
                block_w = False
        except Exception as e:
            rospy.logerr(e)
            no_camera_detected = True
            break
stream_thread = threading.Thread(target=image_updater)
stream_thread.start()

def camera_publisher():
    global capturer, image, block_r, block_w, flag_new, no_camera_detected
    # Initialize the ROS node
    rospy.init_node('fast_camera_publisher', anonymous=True)

    # Create a publisher for the image topic
    image_pub = rospy.Publisher('/camera/image', CompressedImage, queue_size=2)

    rate = rospy.Rate(30)  # Publish at 10 Hz
    
    while not rospy.is_shutdown():
        if no_camera_detected:
            rospy.logdebug('Camera not connected, synthesize random image data!')
            image = generate_random_image(720,720)
            image_pub.publish(image)
        else:
            if not flag_new:
                continue
            flag_new = False
            block_r = True
            while(block_w):
                continue
            image_pub.publish(image)
            block_r = False

        rate.sleep()

    capturer.release()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
