import socket
import numpy as np
import json
import ast

def bbox_to_center(bbox):
    """Convert bounding box [x1, y1, x2, y2] to center point [x_center, y_center]"""
    x_center = (bbox[0] + bbox[2]) / 2
    y_center = (bbox[1] + bbox[3]) / 2
    return x_center, y_center

def image_to_world_coordinates(u, v, camera_height, fx, fy, cx, cy):
    """
    Convert image coordinates (u,v) to world coordinates (x,y,z)
    Assuming camera is mounted vertically above the workspace
    """
    # Convert to normalized image coordinates
    x_normalized = (u - cx) / fx
    y_normalized = (v - cy) / fy

    # Since the camera is looking straight down at the workspace
    # Z is known (it's the camera height)
    z = camera_height

    # Calculate X and Y world coordinates
    x = x_normalized * z
    y = y_normalized * z

    return x, y, z

def process_bbox_coordinates(bbox, camera_params):
    """Process bounding box coordinates to get real world coordinates"""
    # Get center of bounding box
    u, v = bbox_to_center(bbox)

    # Convert to world coordinates
    x, y, z = image_to_world_coordinates(
        u, v,
        camera_params['height'],
        camera_params['fx'],
        camera_params['fy'],
        camera_params['cx'],
        camera_params['cy']
    )

    return x, y, z

# Camera parameters (need to be calibrated for your specific setup)
camera_params = {
    'height': 500,  # camera height in mm
    'fx': 500,      # focal length x in pixels
    'fy': 500,      # focal length y in pixels
    'cx': 320,      # principal point x
    'cy': 240       # principal point y
}

# Configure server
HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 65432      # Port to listen on

def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print(f"Server listening on {HOST}:{PORT}")

    try:
        while True:
            conn, addr = server_socket.accept()
            print(f"Connected by {addr}")

            try:
                data = conn.recv(1024).decode()
                if not data:
                    continue

                print(f"Received raw data: {data}")

                # Convert string representation of list to actual list
                try:
                    bbox = ast.literal_eval(data)
                    if not isinstance(bbox, list) or len(bbox) != 4:
                        raise ValueError("Invalid bounding box format")

                    # Process coordinates
                    x, y, z = process_bbox_coordinates(bbox, camera_params)
                    print(f"World coordinates (mm): X={x:.2f}, Y={y:.2f}, Z={z:.2f}")

                    # Send acknowledgment
                    conn.sendall("Coordinates processed successfully".encode())

                except (ValueError, SyntaxError) as e:
                    print(f"Error processing data: {e}")
                    conn.sendall("Error processing coordinates".encode())

            except Exception as e:
                print(f"Connection error: {e}")
            finally:
                conn.close()

    except KeyboardInterrupt:
        print("\nServer shutting down...")
    finally:
        server_socket.close()

if __name__ == "__main__":
    main()
