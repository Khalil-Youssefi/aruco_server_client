#!/usr/bin/env python

import socket
import cv2
import subprocess
from time import time
import shlex
import numpy as np
# Server settings
server_ip = "192.168.1.19"
# server_ip = "192.168.43.186"
# server_ip = "localhost"
server_port = 54321
image_path = "temp.jpg"

# Create a TCP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to a specific IP and port
server_socket.bind((server_ip, server_port))

# Listen for incoming connections
server_socket.listen(1)
print("Server is listening on {}:{}".format(server_ip, server_port))

while True:
    # Accept a connection from a client
    client_socket, client_address = server_socket.accept()
    print("Connection established with {}:{}".format(client_address[0], client_address[1]))

    # Receive data from the client
    data = client_socket.recv(1024).decode()
    
    if data == "send":
        try:
            # st = time()
            # cmd = f"v4l2-ctl --device=/dev/video0 --set-fmt-video=width={1280},height={720},pixelformat=MJPG --set-ctrl=exposure_time_absolute=1 --stream-mmap --stream-count=1 --stream-to=temp.jpg"
            # subprocess.run(cmd, shell=True)
            # # Load the image
            # image = cv2.imread(image_path)

            # # Encode the image as bytes
            # _, image_data = cv2.imencode(".jpg", image)
            # image_bytes = image_data.tobytes()
            # print('Capturing took:',time()-st)

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
            print('Capturing took:',time()-st)

            # Send the image back to the client
            client_socket.sendall(image_bytes)
            print("Image sent to the client.")

        except Exception as e:
            print("Error occurred while sending the image:", str(e))

    # Close the connection
    client_socket.close()