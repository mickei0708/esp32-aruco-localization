import cv2
import cv2.aruco as aruco
import urllib.request
import numpy as np

# Kamera-URL eintragen (z.B. ESP32-CAM IP)
ip_address = '192.168.2.63'
CAMERA_URL = f'http://{ip_address}:81/stream'

def fetch_image():
    try:
        resp = urllib.request.urlopen(CAMERA_URL)
        image_np = np.asarray(bytearray(resp.read()), dtype="uint8")
        image = cv2.imdecode(image_np, cv2.IMREAD_COLOR)
        return image
    except Exception as e:
        print(f"Error fetching image: {e}")
        return None

def detect_markers(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        image = aruco.drawDetectedMarkers(image.copy(), corners, ids)
        print(f"Detected marker IDs: {ids.flatten()}")
    else:
        print("No markers detected.")

    return image

def main():
    image = fetch_image()
    if image is None:
        print("Failed to fetch image. Exiting...")
        return

    output_image = detect_markers(image)

    cv2.imshow('ArUco Marker Detection', output_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()