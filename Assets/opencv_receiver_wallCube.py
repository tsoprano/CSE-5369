import os
os.environ["OPENCV_IO_ENABLE_OPENEXR"] = "1"

import threading
import struct
import time
import cv2
import numpy as np
import socket

INTERRUPT = False
THREAD_LOCK = threading.Lock()
OUTGOING_BUFFER = None
CHUNK_SIZE = 1400

# Image defaults
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
EOM_MARKER = b'<<EOM>>'

# Image for later use
RGB_FRAME = None
DEPTH_FRAME = None

# Intrinsics placeholder, assume received over network
intrinsics = None

# Thread safety lock
THREAD_LOCK = threading.Lock()

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

            cv2.imshow('RGB Image', frame)
            cv2.waitKey(1)
            
            with THREAD_LOCK:
                global RGB_FRAME
                RGB_FRAME = frame
        except:
            pass
        
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

            frame = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_UNCHANGED)
            frame = frame[:, :, 2]

            frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

            cv2.imshow('Depth Image', frame)
            cv2.waitKey(1)

            with THREAD_LOCK:
                global DEPTH_FRAME
                DEPTH_FRAME = frame
        except:
            pass

def receive_intrinsics(ipaddr='127.0.0.1', port=65433):
    global intrinsics
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ipaddr, port))
    while True:
        data, _ = sock.recvfrom(1024)
        values = data.decode('utf-8').split(',')
        intrinsics = np.array([
            [float(values[0]), 0, float(values[2])],
            [0, float(values[1]), float(values[3])],
            [0, 0, 1]
        ])
        print("Received Camera Intrinsics:", intrinsics)

def pixel_to_world(x, y, depth, intrinsics):
    pixel = np.array([x, y, 1])

    # Scale factor adjustments based on depth observations
    scale_factor_xz = 150

    # Calculate scaled depth to map pixel to world space accurately
    scaled_depth = depth / scale_factor_xz

    # Calculate the unadjusted world position
    world_coords = np.linalg.inv(intrinsics) @ pixel * scaled_depth

    # Apply transformations and offsets based on the camera's new position (1.5, 0.3, 0.2) and orientation (0, -90, 0)
    world_x = -world_coords[2] + 1.5  # Swap z and x due to rotation and adjust with offset
    world_y = -(world_coords[1] - 0.3)
    world_z = world_coords[0] + 0.2   # Invert x for z axis due to rotation and adjust with offset
    print(f" {world_x}, {world_y}, {world_z}, depth: {depth}")

    return (world_x, world_y, world_z)

def send_object_position_to_unity(position, ip='127.0.0.1', port=65432):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        message = f'{position[0]},{position[1]},{position[2]}'
        sock.sendto(message.encode('utf-8'), (ip, port))
    except Exception as e:
        print(f'Error sending position to Unity: {e}')
    finally:
        sock.close()

# Object and Wall Detection
def detect_object_and_wall(rgb_image, depth_image):
    global intrinsics
    
    # Object Detection (Red Color Detection)
    hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        cx, cy = x + w // 2, y + h // 2
        depth = depth_image[cy, cx]

        world_position = pixel_to_world(cx, cy, depth, intrinsics)
        send_object_position_to_unity(world_position)

        cv2.rectangle(rgb_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(rgb_image, (cx, cy), 5, (0, 0, 255), -1)
    
    # Wall Detection
    _, depth_thresh = cv2.threshold(depth_image, 50, 255, cv2.THRESH_BINARY)
    wall_contours, _ = cv2.findContours(depth_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for wall_contour in wall_contours:
        wall_x, wall_y, wall_w, wall_h = cv2.boundingRect(wall_contour)
        cv2.rectangle(rgb_image, (wall_x, wall_y), (wall_x + wall_w, wall_y + wall_h), (255, 0, 0), 2)
    
    cv2.imshow('RGB Image with Object and Wall Detection', rgb_image)
    cv2.imshow('Depth Image for Wall Detection', depth_thresh)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return

def send_thread(ipaddr='127.0.0.1', bind_port=65403, destination_port=65402):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((ipaddr, bind_port))
    udp_socket.settimeout(0.1)
    while not INTERRUPT:
        try:
            rgb_image, depth_image = None, None
            with THREAD_LOCK:
                if RGB_FRAME is not None:
                    rgb_image = RGB_FRAME.copy()
                if DEPTH_FRAME is not None:
                    depth_image = DEPTH_FRAME.copy()
            if rgb_image is None or depth_image is None or intrinsics is None:
                continue
            detect_object_and_wall(rgb_image, depth_image)
        except Exception:
            pass

if __name__ == '__main__':
    rbg_thread = threading.Thread(target=rgb_func)
    rbg_thread.start()
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
        rbg_thread.join()
        send_thread.join()
        exit(0)