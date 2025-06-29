# ESP32 ArUco Localization with Python Visualization

## 🛠️ Projektüberblick

Dieses Projekt ermöglicht eine **Indoor-Lokalisierung von ESP32-CAMs mittels ArUco-Markern** und stellt die berechneten Positionen live in einem **2D-Weltkoordinatensystem** dar. Es verbindet kostengünstige Hardware mit Echtzeit-Visualisierung und bietet eine saubere Basis für Forschung, Lehre und Rapid Prototyping.

---

## 🌟 Projektziele

* Lokalisierung von ESP32-CAMs über ArUco-Marker
* Echtzeitdatenübertragung via MQTT
* Visualisierung mit Python und Matplotlib
* Farbkodierte Anzeige aller Kameras
* Verwendung einer PNG-Kamera-Grafik, die sich nach Blickrichtung rotiert
* Darstellung in Zentimetern zur praxisnahen Auswertung

---

## 💻 Hardware

* ESP32-CAM (AI Thinker o.ä.)
* WLAN Netzwerk
* Gedruckte ArUco Marker (4x4 Dictionary)
* (Optional) Router mit fester IP-Vergabe

---

## 💾 Softwarestack

### ESP32-CAM Firmware

* Erkennt ArUco Marker mittels OpenCV
* Berechnet **tvec (Translation)** und **rvec (Rotation)**
* Veröffentlicht via MQTT:

```json
{
  "camera_id": 50,
  "marker_id": 1,
  "rvec": [0.1, 0.2, 0.3],
  "tvec": [0.05, 0.1, 0.15]
}
```

### MQTT Broker

* Verwendung von **Mosquitto**
* Ports: `1883` (MQTT), optional `9001` (WebSocket)
* Topics: `camera/<id>/pose`

### Python Visualisierung

* `Python 3.9+`
* Bibliotheken: `numpy`, `opencv-python`, `paho-mqtt`, `matplotlib`, `pillow`
* Stellt Kamerapositionen und Blickrichtungen live dar
* Verwendung eines PNG-Kamerasymbols, das sich nach der Blickrichtung rotiert
* Farbzuordnung je Kamera-ID
* Live-Legende der verbundenen Kameras

---

## 🚀 Einrichtung

### 1. MQTT Broker installieren

```bash
sudo apt update
sudo apt install mosquitto mosquitto-clients
sudo systemctl enable mosquitto
```

Optional: WebSocket-Unterstützung in `/etc/mosquitto/mosquitto.conf` aktivieren.

### 2. Python Umgebung einrichten

```bash
python3 -m venv env
source env/bin/activate
pip install numpy opencv-python paho-mqtt matplotlib pillow
```

### 3. ESP32-CAM

* WLAN-Daten und MQTT-Broker-IP eintragen
* Markergröße und Kamera-Parameter kalibrieren
* Firmware flashen

### 4. Visualisierung starten

```bash
python3 visualisierung.py
```

---

## 🔄 Systemablauf

1. ESP32-CAM erkennt Marker
2. Berechnet tvec/rvec
3. Sendet über MQTT die Pose
4. Python-Visualisierung empfängt Pose-Daten
5. Zeichnet Punkte in Echtzeit im 2D-Koordinatensystem
6. Zeigt PNG-Kamera (rot) für eigene Cam, grüner Punkt im Ursprung
7. Richtungsanzeige erfolgt durch Rotation des Kamerasymbols

---

## 🔍 Technische Details

* **ArUco Marker Detection:** OpenCV verwendet `estimatePoseSingleMarkers`, Marker-Kantenlänge festgelegt, Transformation ins globale KS mittels referenzierter Marker.
* **MQTT:** Leichtgewichtiges Pub/Sub Protokoll, ideal für IoT, latenzarm und ressourcenschonend.
* **Matplotlib Visualisierung:**

  * Live-Plot alle 500ms (anpassbar)
  * Darstellung in cm mit Achsen von -100 bis 100
  * PNG wird über `AnnotationBbox` rotiert dargestellt
  * Dynamische Legende für verbundene Kameras

---

## 🚪 Troubleshooting

* **Keine Daten im Plot:** Broker mit `mosquitto_sub -t '#' -v` testen
* **Keine Marker erkannt:** Beleuchtung und Markerqualität prüfen
* **PNG nicht sichtbar:** `camera_icon.png` im Projektverzeichnis vorhanden?
* **Lags:** Animationsintervall in `FuncAnimation` anpassen

---

## 📊 Erweiterungsideen

* 3D Visualisierung mit `pythreejs` oder ROS RViz
* Automatische Marker-Mapping und Kalibrierung
* Speicherung der Trajektorien zur Analyse
* Hinderniserkennung und Visualisierung

---


## 🎉 Danksagung

Vielen Dank an alle, die zur **Fehleranalyse und Optimierung** dieses Projekts beigetragen haben.

Pull Requests und Feedback sind jederzeit willkommen, um das Projekt weiter zu verbessern.

---

**Viel Erfolg beim Verwenden und Erweitern dieses Projekts!**
