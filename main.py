import cv2
import cv2.aruco as aruco
import numpy as np
import json
import time
import paho.mqtt.client as mqtt


# === Konfiguration ===
CAMERA_ID = 5  # Eindeutige ID der Kamera für MQTT-Nachrichten
IP_ADDRESS = '192.168.137.226'  # IP-Adresse des ESP32-Streams
MARKER_LENGTH = 0.088  # Marker-Kantenlänge in Metern (für Pose-Berechnung)
VIDEO_SOURCE = f'http://{IP_ADDRESS}:81/stream'  # Videoquelle: HTTP-Stream des ESP32

# === ArUco Setup ===
# Initialisierung des ArUco-Markerdictionaries und der Detektorparameter
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

# === Kamera öffnen ===
# Öffnen des Video-Streams von der angegebenen Quelle
cap = cv2.VideoCapture(VIDEO_SOURCE)
if not cap.isOpened():
    print("❌ Kamera konnte nicht geöffnet werden.")
    exit()

# === Dummy-Kameramatrix und Verzerrungskoeffizienten ===
# Ermittlung der Frame-Größe zur Berechnung einer approximativen Kameramatrix
frame_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
frame_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print("📏 Frame-Größe:", int(frame_width), "x", int(frame_height))

# Grobe Abschätzung der Brennweite basierend auf der Frame-Breite
focal_length = 0.8 * frame_width

# Konstruktion der Kameramatrix für die Pose-Schätzung
"""camera_matrix = np.array([[focal_length, 0, frame_width / 2],
                          [0, focal_length, frame_height / 2],
                          [0, 0, 1]], dtype=np.float32)"""

camera_matrix = np.array([[304.5403, 0, 163.8102],
                          [0, 312.3525, 126.2962],
                          [0, 0, 1]], dtype=np.float32)

# Annahme: keine Verzerrung des Kamerabildes
dist_coeffs = np.zeros((4, 1))

# === MQTT-Verbindung herstellen ===
# Initialisierung des MQTT-Clients und Verbindung zum Broker
mqtt_client = mqtt.Client()
mqtt_client.connect("192.168.137.74", 1883, 60)

# Zeitstempel für die Frequenzsteuerung der Ausgabe
last_print_time = time.time()

print("📷 Marker-Erkennung gestartet – ESC zum Beenden")

# === Hauptverarbeitungsschleife ===
while True:
    # Frame vom Video-Stream lesen
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Frame konnte nicht gelesen werden.")
        continue

    # Umwandlung des Frames in Graustufen für die Marker-Erkennung
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detektion von ArUco-Markern im Graustufenbild
    corners, ids, _ = detector.detectMarkers(gray)

    # Wenn Marker erkannt wurden, Pose berechnen und Daten ausgeben
    if ids is not None:
        rvecs = []  # Liste für Rotationsvektoren
        tvecs = []  # Liste für Translationsvektoren

        # Definition der Objektpunkte des Markers im Marker-Koordinatensystem
        object_points = np.array([
            [-MARKER_LENGTH / 2,  MARKER_LENGTH / 2, 0],
            [ MARKER_LENGTH / 2,  MARKER_LENGTH / 2, 0],
            [ MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0],
            [-MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0]
        ])

        # Berechnung der Pose (Rotation und Translation) für jeden erkannten Marker
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

        # Ausgabe der Marker-Informationen einmal pro Sekunde
        now = time.time()
        if now - last_print_time >= 1.0:
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                tvec = tvecs[i].flatten()  # Translationsvektor als 1D-Array
                rvec = rvecs[i][0].tolist()  # Rotationsvektor als Liste

                # Vorbereitung der JSON-Daten für MQTT und Konsolenausgabe
                json_data = {
                    "camera_id": CAMERA_ID,
                    "marker_id": marker_id,
                    "tvec": [round(x, 4) for x in tvec.tolist()],
                    "rvec": [round(x, 4) for x in rvec]
                }

                # Berechnung des Abstands zum Marker (euklidische Norm)
                distance = round(np.linalg.norm(tvec), 4)

                # Berechnung des Winkels zwischen Blickrichtung der Kamera und Markerposition
                view_dir = np.array([0, 0, 1])  # Kamera blickt entlang der Z-Achse
                marker_vec = np.array(tvec).flatten()
                marker_vec = marker_vec / np.linalg.norm(marker_vec)  # Normalisierung

                # Skalarprodukt zur Winkelbestimmung, begrenzt auf [-1, 1]
                dot = np.clip(np.dot(view_dir, marker_vec), -1.0, 1.0)
                angle_rad = np.arccos(dot)
                angle_deg = round(np.degrees(angle_rad), 2)

                # Ausgabe der JSON-Daten und weiterer Informationen
                print(json.dumps(json_data))
                # Senden der Pose-Daten via MQTT an das Topic "test"
                # mqtt_client.publish(f"camera/{CAMERA_ID}/pose", json.dumps(json_data))
                mqtt_client.publish("test", json.dumps(json_data))
                print(f"➡️  Marker {marker_id} | Distanz: {distance} m | Winkel: {angle_deg}°")
            last_print_time = now

        # Optional: Zeichnen der erkannten Marker auf das Frame
        aruco.drawDetectedMarkers(frame, corners, ids)

    # Anzeige des Frames mit optionalen Overlays
    cv2.imshow("Marker View", frame)

    # Beenden der Schleife bei Drücken der ESC-Taste
    if cv2.waitKey(1) == 27:
        break

# Ressourcen freigeben und Fenster schließen
cap.release()
cv2.destroyAllWindows()


"""
📌 Hinweise zur Verwendung des MQTT-Systems über das Terminal:

1. 🛰️ MQTT-Subscription starten:
   mosquitto_sub -h "192.168.137.74" -t "test"

   ➤ Dieser Befehl startet einen MQTT-Client, der sich mit dem Broker unter der IP-Adresse 192.168.137.74 verbindet
     und auf Nachrichten im Topic "test" wartet. 
     Alles, was auf diesem Topic gesendet wird, erscheint live im Terminal.

2. 🚀 MQTT-Broker lokal starten (falls er noch nicht läuft):
   mosquitto -v

   ➤ Startet den Mosquitto-Broker im Vordergrund und gibt alle Aktivitäten im Terminal aus
     (z. B. Verbindungen, Veröffentlichungen, Abonnements). Wichtig für Debugging.

   Alternativ prüfen, ob der Broker läuft:
   ps aux | grep mosquitto

   ➤ Zeigt aktive Prozesse mit dem Namen "mosquitto" an, um zu prüfen, ob der Broker schon läuft.

3. ✉️ Testnachricht senden:
   mosquitto_pub -h "192.168.137.74" -t "test" -m "Hallo MQTT"

   ➤ Sendet die Textnachricht "Hallo MQTT" an den Broker unter der angegebenen IP und dem Topic "test".
     Wenn jemand mit `mosquitto_sub` auf dieses Topic hört, wird diese Nachricht dort angezeigt.

⚠️ Hinweis:
- Der Broker (Mosquitto) muss auf Port 1883 erreichbar sein (Standard).
- Andere Geräte im Netzwerk müssen die IP-Adresse des Macs kennen.
- Die Firewall muss externe Verbindungen auf Port 1883 erlauben.
"""