# List alls devices
import v4l2ctl
import cv2
import numpy as np
from v4l2ctl import Frame
from v4l2ctl import V4l2
import threading
from time import sleep
# Capture Code
capturer = Frame('/dev/video0')
image = None
block_r = False
block_w = False
flag_new = False
def image_updater():
    global capturer, image, block_r, block_w, flag_new
    while True:
        frame_data = capturer.get_frame()

        np_array = np.frombuffer(frame_data, dtype=np.uint8)

        image_c = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

        if not block_r:
            block_w = True
            image = image_c
            flag_new = True
            block_w = False

stream_thread = threading.Thread(target=image_updater)
stream_thread.start()

while(True):
    if not flag_new:
        continue
    flag_new = False
    block_r = True
    while(block_w):
        continue
    cv2.imshow('JPEG Image', image)
    if cv2.waitKey(700) & 0xFF == ord('q'):
        break
    block_r = False

