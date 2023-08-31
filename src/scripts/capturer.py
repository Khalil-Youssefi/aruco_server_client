import cv2
from v4l2ctl import Frame
import numpy as np
def capture_and_save_frame(output_path):
    frame = Frame('/dev/video0')
    frame_data = frame.get_frame()

    np_array = np.frombuffer(frame_data, dtype=np.uint8)

    # Decode the NumPy array using OpenCV
    image = cv2.imdecode(np_array, cv2.IMREAD_COLOR)  # Change IMREAD flag as needed
    cv2.imwrite(output_path,image)

if __name__ == "__main__":
    output_file = "temp.jpg"  # Change this to your desired output file path
    capture_and_save_frame(output_file)
    print(f"Frame captured and saved as {output_file}")


