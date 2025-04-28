import cv2
import cv2.aruco as aruco
import numpy as np

# Replace with the IP address of your ESP32
ip_address = '192.168.2.63'
url = f'http://{ip_address}:81/stream'

# Feste Markergröße in Meter
MARKER_SIZE = 0.093  # 8,5 cm


def detect_markers(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(gray)

    return corners, ids

def get_camera_matrix(image_width, image_height, fov_deg=65):
    fov_rad = np.deg2rad(fov_deg)
    fx = fy = (image_width / 2) / np.tan(fov_rad / 2)
    cx = image_width / 2
    cy = image_height / 2
    camera_matrix = np.array([[fx, 0, cx],
                              [0, fy, cy],
                              [0, 0, 1]], dtype=np.float32)
    return camera_matrix

def estimate_pose(frame, corners):
    image_height, image_width = frame.shape[:2]
    camera_matrix = get_camera_matrix(image_width, image_height)
    dist_coeffs = np.zeros((4, 1))  # Keine Verzerrung angenommen

    # 3D Punkte des Markers (im Weltkoordinatensystem)
    obj_points = np.array([
        [-MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
        [ MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
        [ MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
        [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0]
    ], dtype=np.float32)

    # Ecken (corners) müssen richtig formatiert werden!
    img_points = np.array(corners, dtype=np.float32)

    # solvePnP erwartet (N,1,3) für objPoints und (N,1,2) für imagePoints
    obj_points = obj_points.reshape(-1, 1, 3)
    img_points = img_points.reshape(-1, 1, 2)

    success, rvecs, tvecs = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)

    if not success:
        raise ValueError("Could not solve PnP!")

    # Abstand x
    x = np.linalg.norm(tvecs)

    # Rotation berechnen
    R, _ = cv2.Rodrigues(rvecs)
    omega_rad = np.arctan2(R[0, 2], R[2, 2])
    omega_deg = np.degrees(omega_rad)

    return x, omega_deg

def process_frame(frame):
    corners, ids = detect_markers(frame)
    if ids is not None:
        frame = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        for i, corner in enumerate(corners):
            x, omega = estimate_pose(frame, corner)
            print(f"Marker ID {ids[i][0]}: Distance = {x:.3f} m, Rotation omega = {omega:.2f}°")
    else:
        print("No markers detected.")
    return frame

def main():
    # Open video stream
    cap = cv2.VideoCapture(url)

    if not cap.isOpened():
        print("Error: Cannot open video stream")
        exit()
    else:
        print("Success: Starting video stream")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Process frame: detect markers, estimate pose, print info and draw markers
        output_frame = process_frame(frame)

        # Show the frame with detection
        cv2.imshow('ESP32 Stream with ArUco Detection', output_frame)

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()