# === ArUco Marker Detection mit korrekter Winkelberechnung ===

import cv2
import cv2.aruco as aruco
import numpy as np
import json
import time
import paho.mqtt.client as mqtt
import math

# === Konfiguration ===
CAMERA_ID = 20
IP_ADDRESS = '192.168.2.74'
MY_IP = "192.168.2.61"
MARKER_LENGTH = 0.02  # Marker-Kantenlänge in Metern
VIDEO_SOURCE = f'http://{IP_ADDRESS}:81/stream'

# === ArUco Setup ===
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

# === Kamera öffnen ===
cap = cv2.VideoCapture(VIDEO_SOURCE)
if not cap.isOpened():
    print("❌ Kamera konnte nicht geöffnet werden.")
    exit()

# === Dummy-Kameramatrix und -verzerrung ===
frame_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
frame_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print("📏 Frame-Größe:", int(frame_width), "x", int(frame_height))
camera_matrix = np.array([[304.54, 0, 163.81],
                          [0, 312.53, 126.29],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.array([-0.048, 0.184, 0, 0], dtype=np.float32)

# MQTT-Verbindung herstellen
mqtt_client = mqtt.Client()
mqtt_client.connect(MY_IP, 1883, 60)
mqtt_client.loop_start()

last_print_time = time.time()

print("📷 Marker-Erkennung gestartet – ESC zum Beenden")

def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2,1], R[2,2])  # Roll
        y = math.atan2(-R[2,0], sy)     # Pitch
        z = math.atan2(R[1,0], R[0,0])  # Yaw
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.degrees([x, y, z])

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠ Frame konnte nicht gelesen werden.")
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        rvecs = []
        tvecs = []
        object_points = np.array([
            [-MARKER_LENGTH / 2,  MARKER_LENGTH / 2, 0],
            [ MARKER_LENGTH / 2,  MARKER_LENGTH / 2, 0],
            [ MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0],
            [-MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0]
        ])

        for corner in corners:
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                corner[0],
                camera_matrix,
                dist_coeffs
            )
            if success:
                rvecs.append(rvec)
                tvecs.append(tvec)

        now = time.time()
        if now - last_print_time >= 0.5:
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                tvec = tvecs[i].flatten()
                rvec = rvecs[i].flatten()

                # Rotation in Euler-Winkel umrechnen
                R, _ = cv2.Rodrigues(rvec)
                angles = rotationMatrixToEulerAngles(R)
                pitch, yaw, roll = angles
                # roll, pitch, yaw = angles

                json_data = {
                    "camera_id": CAMERA_ID,
                    "marker_id": marker_id,
                    "tvec": [round(x, 4) for x in tvec.tolist()],
                    "rvec": [round(x, 4) for x in rvec.tolist()]
                    # "roll": round(roll, 2),
                    # "pitch": round(pitch, 2),
                    # "yaw": round(yaw, 2)
                }

                distance = round(np.linalg.norm(tvec), 4)

                print(json.dumps(json_data))
                mqtt_client.publish(f"camera/{CAMERA_ID}/pose", json.dumps(json_data))
                print(f"➡  Marker {marker_id} | Distanz: {distance} m | Roll: {roll:.2f}° | Pitch: {pitch:.2f}° | Yaw: {yaw:.2f}°")

            last_print_time = now

        aruco.drawDetectedMarkers(frame, corners, ids)

    cv2.imshow("Marker View", frame)
    if cv2.waitKey(1) == 27:  # ESC zum Beenden
        break

cap.release()
cv2.destroyAllWindows()