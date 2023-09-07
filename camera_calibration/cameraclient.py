import socket
import cv2
import numpy as np
from time import time
import datetime
import os

curdir = os.path.dirname(os.path.abspath(__file__))
folder_name = curdir+ "/calib_drone_purple"
if not os.path.exists(folder_name):
    os.makedirs(folder_name)

def save_image_with_timestamp(image):
    try:
        now = datetime.datetime.now()
        date_time_str = now.strftime("%Y%m%d_%H%M%S")
        new_filename = f"img_{date_time_str}.jpg"
        cv2.imwrite(folder_name + "/" + new_filename, image)
        print(f"Image saved as {new_filename}")
    except Exception as e:
        print(f"Error: {e}")

# Server settings
server_ip = "192.168.1.16"
server_port = 54323

stime = time()
cv2.namedWindow("Received Image", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Received Image", 720, 1280)
while True:
    # Create a new TCP socket for each request
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Connect to the server
    client_socket.connect((server_ip, server_port))

    # Send the request to the server
    client_socket.sendall("send".encode())

    # Receive the image from the server
    image_bytes = b""
    while True:
        data = client_socket.recv(1024)
        if not data:
            break
        image_bytes += data

    # Convert the received bytes to an image
    nparr = np.frombuffer(image_bytes, np.uint8)
    image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

    # Close the connection
    client_socket.close()
    # Display the image
    cv2.imshow("Received Image", image)
       # Wait for key press
    key = cv2.waitKey(0)
    stime = time()
    # Check if the key is 'q' to exit
    if key == ord('q'):
        break
    if key == ord('s'):
        save_image_with_timestamp(image)

# Close any remaining connections
cv2.destroyAllWindows()