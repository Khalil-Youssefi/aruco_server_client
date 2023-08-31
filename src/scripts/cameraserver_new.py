#!/usr/bin/env python

import socket
import cv2
from time import time
import threading
# Server settings
server_ip = "192.168.1.19"
# server_ip = "192.168.43.186"
# server_ip = "localhost"
server_port = 54322
image_path = "temp.jpg"

# Create a TCP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to a specific IP and port
server_socket.bind((server_ip, server_port))

# Listen for incoming connections
server_socket.listen(1)
print("Server is listening on {}:{}".format(server_ip, server_port))

# Capture Code
capturer = cv2.VideoCapture(0)
image = None
block_r = False
block_w = False
flag_new = False
def image_updater():
    global capturer, image, block_r, block_w, flag_new
    while True:
        _, image_c = capturer.read()
        if not block_r:
            block_w = True
            image = image_c
            flag_new = True
            block_w = False
stream_thread = threading.Thread(target=image_updater)
stream_thread.start()

while True:
    # Accept a connection from a client
    client_socket, client_address = server_socket.accept()
    print("Connection established with {}:{}".format(client_address[0], client_address[1]))

    # Receive data from the client
    data = client_socket.recv(1024).decode()
    
    if data == "send":
        try:
            if not flag_new:
                continue
            flag_new = False
            block_r = True
            while(block_w):
                continue
            _, image_data = cv2.imencode(".jpg", image)
            image_bytes = image_data.tobytes()
            client_socket.sendall(image_bytes)
            block_r = False
            print("Image sent to the client.")

        except Exception as e:
            print("Error occurred while sending the image:", str(e))

    # Close the connection
    client_socket.close()