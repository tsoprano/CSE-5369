import os
os.environ["OPENCV_IO_ENABLE_OPENEXR"] = "1"

import threading
import struct
import time
import cv2
import numpy as np
import socket
from tensorflow.keras.models import load_model
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input

# Constants
INTERRUPT = False
THREAD_LOCK = threading.Lock()
CHUNK_SIZE = 1400
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
EOM_MARKER = b'<<EOM>>'
RGB_FRAME = None
DEPTH_FRAME = None
intrinsics = None

# CNN Model
MODEL_PATH = r"D:\Unity projects\bottle_can_classifier.h5" 
# MODEL_PATH = r"D:\Unity projects\bottle_can_classifier_new.h5" 
model = load_model(MODEL_PATH)
labels = ['bottle', 'can']  # Class labels for the CNN

# Function to preprocess image for MobileNetV2
def preprocess_image(image, img_size=(224, 224)):
    """Preprocess the image for MobileNetV2."""
    resized_image = cv2.resize(image, img_size)  # Resize to 224x224
    image_array = np.expand_dims(resized_image, axis=0)  # Add batch dimension
    return preprocess_input(image_array)  # Normalize using MobileNetV2 preprocessing

# Function to classify an object using the CNN
def classify_object(image):
    """Classify the object using the MobileNetV2-based CNN."""
    preprocessed_image = preprocess_image(image)
    prediction = model.predict(preprocessed_image)
    class_idx = np.argmax(prediction)
    confidence = prediction[0][class_idx]
    return labels[class_idx], confidence

# Function to remove duplicate detections
def remove_duplicate_detections(detections, threshold=30):
    unique_detections = []
    for label, cx, cy, world_position in detections:
        is_duplicate = False
        for _, existing_cx, existing_cy, _ in unique_detections:
            if abs(cx - existing_cx) < threshold and abs(cy - existing_cy) < threshold:
                is_duplicate = True
                break
        if not is_duplicate:
            unique_detections.append((label, cx, cy, world_position))
    return unique_detections

# Function to handle RGB frame reception
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

# Function to handle depth frame reception
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
            frame = frame[:, :, 2]  # Extract depth channel
            frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            with THREAD_LOCK:
                global DEPTH_FRAME
                DEPTH_FRAME = frame
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
    scale = 76  # Depth scaling factor
    world = np.linalg.inv(intrinsics) @ pixel * (depth / scale)
    return -world[0], -(world[1] - 0.36), -(world[2] - 1)

# Send classified object position to Unity
# def send_object_position_to_unity(position, label, ip='127.0.0.1', port=65432):
#     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     try:
#         message = f'{label},{position[0]},{position[1]},{position[2]}'
#         sock.sendto(message.encode('utf-8'), (ip, port))
#     except Exception as e:
#         print(f"Error sending position to Unity: {e}")
#     finally:
#         sock.close()

# Function to send all detected objects to Unity
def send_all_objects_to_unity(detections, ip='127.0.0.1', port=65432):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Create a single message for all detections
        message = ";".join(
            f"{label},{world_position[0]},{world_position[1]},{world_position[2]}"
            for label, _, _, world_position in detections
        )
        sock.sendto(message.encode('utf-8'), (ip, port))
    except Exception as e:
        print(f"Error sending positions to Unity: {e}")
    finally:
        sock.close()

# # Object detection and communication thread
# def send_thread(ipaddr='127.0.0.1', bind_port=65403, destination_port=65402):
#     udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#     udp_socket.bind((ipaddr, bind_port))
#     udp_socket.settimeout(0.1)
#     while not INTERRUPT:
#         try:
#             rgb_image = None
#             depth_image = None

#             with THREAD_LOCK:
#                 if RGB_FRAME is not None:
#                     rgb_image = RGB_FRAME.copy()
#                 if DEPTH_FRAME is not None:
#                     depth_image = DEPTH_FRAME.copy()

#             if rgb_image is None or depth_image is None or intrinsics is None:
#                 continue

#             # Convert image to grayscale and apply edge detection
#             gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
#             blurred = cv2.GaussianBlur(gray, (5, 5), 0)
#             edges = cv2.Canny(blurred, 50, 150)

#             # Detect contours based on edges
#             contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#             for contour in contours:
#                 # Calculate bounding box for each detected contour
#                 x, y, w, h = cv2.boundingRect(contour)

#                 # Filter out very small or very large regions
#                 if w * h < 500 or w * h > 0.5 * (rgb_image.shape[0] * rgb_image.shape[1]):
#                     continue

#                 # Extract ROI for classification
#                 roi = rgb_image[y:y + h, x:x + w]
#                 if roi.shape[0] > 0 and roi.shape[1] > 0:
#                     label, confidence = classify_object(roi)
#                     if confidence > 0.7:  # Confidence threshold
#                         # Calculate center of bounding box
#                         cx, cy = x + w // 2, y + h // 2
#                         depth = depth_image[cy, cx]

#                         # Convert the pixel coordinates to world coordinates
#                         world_position = pixel_to_world(cx, cy, depth, intrinsics)

#                         # Send the detected object and its position to Unity
#                         send_object_position_to_unity(world_position, label)

#                         # Draw bounding box and label for debugging
#                         cv2.rectangle(rgb_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
#                         cv2.putText(rgb_image, f"{label} ({confidence:.2f})",
#                                     (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

#             # Display the RGB image for debugging purposes
#             cv2.imshow('RGB Image', rgb_image)
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break

#         except Exception as e:
#             print(f"Error in send_thread: {e}")

# Object detection and communication thread
def send_thread(ipaddr='127.0.0.1', bind_port=65403, destination_port=65402):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.bind((ipaddr, bind_port))
    udp_socket.settimeout(0.1)
    while not INTERRUPT:
        try:
            rgb_image = None
            depth_image = None

            # Retrieve the current RGB and depth frames
            with THREAD_LOCK:
                if RGB_FRAME is not None:
                    rgb_image = RGB_FRAME.copy()
                if DEPTH_FRAME is not None:
                    depth_image = DEPTH_FRAME.copy()

            # Skip if frames or intrinsics are not available
            if rgb_image is None or depth_image is None or intrinsics is None:
                continue

            # Preprocess the image
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)

            # Detect contours
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            detections = []

            for contour in contours:
                # Calculate bounding box for each detected contour
                x, y, w, h = cv2.boundingRect(contour)

                # Filter out very small or very large regions
                if w * h < 500 or w * h > 0.5 * (rgb_image.shape[0] * rgb_image.shape[1]):
                    continue

                # Extract ROI for classification
                roi = rgb_image[y:y + h, x:x + w]
                if roi.shape[0] > 0 and roi.shape[1] > 0:
                    label, confidence = classify_object(roi)
                    if confidence > 0.7:  # Confidence threshold
                        # Calculate center of bounding box
                        cx, cy = x + w // 2, y + h // 2
                        depth = depth_image[cy, cx]

                        # Convert the pixel coordinates to world coordinates
                        world_position = pixel_to_world(cx, cy, depth, intrinsics)

                        # Append detection
                        detections.append((label, cx, cy, world_position))

            # Remove duplicate detections
            filtered_detections = remove_duplicate_detections(detections)

            # Send all detected objects to Unity
            send_all_objects_to_unity(filtered_detections, ip=ipaddr, port=destination_port)

            # Debug visualization
            for label, cx, cy, world_position in filtered_detections:
                cv2.putText(rgb_image, f"{label}", (int(cx), int(cy - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.rectangle(rgb_image, (int(cx - 20), int(cy - 20)),
                              (int(cx + 20), int(cy + 20)), (0, 255, 0), 2)

            # Display the RGB image for debugging purposes
            cv2.imshow('RGB Image', rgb_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print(f"Error in send_thread: {e}")




# Main function
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
        depth_thread.join()
        intrinsics_thread.join()
        send_thread.join()
        exit(0)
