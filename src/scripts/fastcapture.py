#!/usr/bin/env python

import socket
import cv2
import subprocess
from time import time
import shlex
import numpy as np

st = time()
command_str = (
    "v4l2-ctl --device=/dev/video0 "
    "--set-fmt-video=width=1280,height=720,pixelformat=MJPG "
    "--set-ctrl=exposure_time_absolute=1 --stream-mmap "
    "--stream-count=1 --stream-to=-"
)
command = shlex.split(command_str)
process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
image_bytes = process.stdout.read()
image_bytes = np.frombuffer(image_bytes, dtype=np.uint8).reshape((720, 1280, 3))
print(image_bytes)