import os
os.environ["OPENCV_IO_ENABLE_OPENEXR"] = "1"

import threading
import struct
import time
import cv2
import numpy as np
import socket

# Constants
INTERRUPT = False
THREAD_LOCK = threading.Lock()
CHUNK_SIZE = 2400
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
EOM_MARKER = b'<<EOM>>'
RGB_FRAME = None
DEPTH_FRAME = None
intrinsics = None

# Color thresholds for red and yellow (and green)
COLOR_THRESHOLDS = {
    "red": ((0, 100, 100), (10, 255, 255)),  # Lower and upper bounds for red in HSV
    # "green": ((40, 40, 40), (80, 255, 255)), # Lower and upper bounds for green in HSV
    "yellow": ((20, 100, 100), (30, 255, 255))  # Lower and upper bounds for yellow in HSV
}

# UDP Configuration
UNITY_IP = '127.0.0.1'
UNITY_PORT = 65432

# Function to preprocess RGB frames
def rgb_func(ipaddr='127.0.0.1', port=65400):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.bind((ipaddr, port))
    udp_socket.settimeout(0.1)
    while not INTERRUPT:
        try:
            frame_data = bytearray()
            while frame_data.count(EOM_MARKER) < 2:
                try:
                    data, _ = udp_socket.recvfrom(CHUNK_SIZE)
                    frame_data.extend(data)
                except socket.timeout:
                    pass
            start_idx = frame_data.find(EOM_MARKER)
            end_idx = frame_data.find(EOM_MARKER, start_idx + 1)
            frame_data = frame_data[start_idx + len(EOM_MARKER):end_idx]
            nparr = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            with THREAD_LOCK:
                global RGB_FRAME
                RGB_FRAME = frame
        except Exception as e:
            print(f"Error in rgb_func: {e}")

# Function to preprocess depth frames
def depth_func(ipaddr='127.0.0.1', port=65401):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.bind((ipaddr, port))
    udp_socket.settimeout(0.1)
    while not INTERRUPT:
        try:
            frame_data = bytearray()
            while frame_data.count(EOM_MARKER) < 2:
                try:
                    data, _ = udp_socket.recvfrom(CHUNK_SIZE)
                    frame_data.extend(data)
                except socket.timeout:
                    pass
            start_idx = frame_data.find(EOM_MARKER)
            end_idx = frame_data.find(EOM_MARKER, start_idx + 1)
            frame_data = frame_data[start_idx + len(EOM_MARKER):end_idx]

            # Decode EXR ZIP image
            frame = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_UNCHANGED)

            # Check if the frame is valid
            if frame is None:
                print("Warning: Received an invalid or corrupted depth frame.")
                continue

            # Ensure the frame has the correct dimensions
            if len(frame.shape) < 3 or frame.shape[2] < 3:
                print("Warning: Received a depth frame with incorrect dimensions.")
                continue

            # Extract the depth channel
            depth_channel = frame[:, :, 2]  # Extract depth channel

            # Normalize the depth data
            normalized_depth = cv2.normalize(depth_channel, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

            # Store the processed depth frame in a thread-safe manner
            with THREAD_LOCK:
                global DEPTH_FRAME
                DEPTH_FRAME = normalized_depth

        except Exception as e:
            print(f"Error in depth_func: {e}")


# Receive camera intrinsics
def receive_intrinsics(ipaddr='127.0.0.1', port=65433):
    global intrinsics
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ipaddr, port))
    while not INTERRUPT:
        data, _ = sock.recvfrom(1024)
        values = data.decode('utf-8').split(',')
        intrinsics = np.array([
            [float(values[0]), 0, float(values[2])],
            [0, float(values[1]), float(values[3])],
            [0, 0, 1]
        ])
        print("Received Camera Intrinsics:", intrinsics)

# Convert 2D pixel coordinates to 3D world coordinates
def pixel_to_world(x, y, depth, intrinsics):
    pixel = np.array([x, y, 1])
    # print(depth)
    scale = 144  # Depth scaling factor
    world = np.linalg.inv(intrinsics) @ pixel * (depth / scale)
    return -world[0], -(world[1] - 0.36), -(world[2] - 1)

# Send detected object positions to Unity
def send_all_objects_to_unity(detections, ip=UNITY_IP, port=UNITY_PORT):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        messages = []
        for label, _, _, world_position in detections:
            message = f"{label},{world_position[0]},{world_position[1]},{world_position[2]}"
            messages.append(message)
        full_message = ";".join(messages)
        # print("Sending message...")
        # print(full_message)
        sock.sendto(full_message.encode('utf-8'), (ip, port))
    except Exception as e:
        print(f"Error sending data to Unity: {e}")
    finally:
        sock.close()

# Remove duplicate detections based on proximity
def remove_duplicate_detections(detections, threshold=0.1):
    unique_detections = []
    for i, detection in enumerate(detections):
        label, cx, cy, world_position = detection
        is_duplicate = False
        for _, _, _, unique_world_position in unique_detections:
            if np.linalg.norm(np.array(world_position) - np.array(unique_world_position)) < threshold:
                is_duplicate = True
                break
        if not is_duplicate:
            unique_detections.append(detection)
    return unique_detections

# Detect objects and send their positions to Unity
def send_thread(ipaddr='127.0.0.1', bind_port=65403, destination_port=65402):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.bind((ipaddr, bind_port))
    udp_socket.settimeout(0.1)
    while not INTERRUPT:
        try:
            rgb_image = None
            depth_image = None

            with THREAD_LOCK:
                if RGB_FRAME is not None:
                    rgb_image = RGB_FRAME.copy()
                if DEPTH_FRAME is not None:
                    depth_image = DEPTH_FRAME.copy()

            if rgb_image is None or depth_image is None or intrinsics is None:
                continue

            detections = []

            # Process each color threshold
            for label, (lower, upper) in COLOR_THRESHOLDS.items():
                hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, lower, upper)
                contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                for contour in contours:
                    x, y, w, h = cv2.boundingRect(contour)
                    cx, cy = x + w // 2, y + h // 2
                    depth = depth_image[cy, cx]

                    if depth == 0:  # Skip invalid depth values
                        continue

                    world_position = pixel_to_world(cx, cy, depth, intrinsics)
                    detections.append((label, cx, cy, world_position))

            # Remove duplicate detections
            filtered_detections = remove_duplicate_detections(detections)

            # Send detections to Unity
            send_all_objects_to_unity(filtered_detections, ip=UNITY_IP, port=UNITY_PORT)

            # Debug visualization
            for label, cx, cy, _ in filtered_detections:
                cv2.putText(rgb_image, f"{label}", (int(cx), int(cy - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.rectangle(rgb_image, (int(cx - 20), int(cy - 20)),
                              (int(cx + 20), int(cy + 20)), (0, 255, 0), 2)

            cv2.imshow('RGB Image', rgb_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print(f"Error in send_thread: {e}")

# Main function
if __name__ == '__main__':
    rgb_thread = threading.Thread(target=rgb_func)
    rgb_thread.start()
    depth_thread = threading.Thread(target=depth_func)
    depth_thread.start()
    intrinsics_thread = threading.Thread(target=receive_intrinsics)
    intrinsics_thread.start()
    send_thread = threading.Thread(target=send_thread)
    send_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        INTERRUPT = True
        rgb_thread.join()
        depth_thread.join()
        intrinsics_thread.join()
        send_thread.join()
        exit(0)
