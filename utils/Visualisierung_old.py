#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Live-Viewer für Camera-/Marker-Posen (Matplotlib-Version)
--------------------------------------------------------
* empfängt rvec / tvec per MQTT
* zeichnet je Kamera eine Achse (RGB) + Label
* aktualisiert die Grafik ca. 30 ×/s
"""
import sys, json, threading, time
import numpy as np, cv2, paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib
from scipy.spatial.transform import Rotation as R, Slerp


matplotlib.use("QtAgg")

# ---------- Hypterparamer -----------------------------------------------------
CAMERA_IDS = [0,10,20,30,40,50,60,70,80,90,100,110]
ACCEPTED_MARKER_IDS = [0, 10, 20, 30, 40, 50, 60]  # Added accepted marker IDs list
MAX_T_JUMP   = 1     # m   – Positionssprung, den wir erlauben
MAX_R_JUMP   = 90        # °   – Winkel­sprung (rotational delta)
ALPHA_T = 0.6          # 0..1  – je kleiner, desto stärker geglättet
ALPHA_R = 0.6 

MY_IP = "192.168.2.61"
# ---------- MQTT --------------------------------------------------------------
#BROKER, PORT, TOPIC = "broker.hivemq.com", 1883, "camera/+/pose"
# BROKER, PORT, TOPIC = "localhost", 1883, "camera/+/pose"
BROKER, PORT, TOPIC = MY_IP, 1883, "camera/+/pose"
positions = {}         # {id: {"rvec":3×1, "tvec":3×1}}
last_see = {}
global_cam_pos = {}    # {cid: np.array(3,)} globale Kamera-Position im Weltkoordinatensystem
global_cam_rvec = {}   # {cid: np.array(3,)} globale Kamera-Rotation (rvec)

def on_message(_, __, msg):
    data          = json.loads(msg.payload.decode())
    print(f"new data: {data}")
    update_positions(data)

client = mqtt.Client()
client.on_message = on_message
client.connect(BROKER, PORT); client.subscribe(TOPIC)
client.loop_start()     # Netzwerk-Thread läuft im Hintergrund

# ---------- positions_update --------------------------------------------------
def update_positions(data):
    cid = data["camera_id"]
    marker_id = data["marker_id"]

    # Filter out messages with marker_id not in accepted list
    if marker_id not in ACCEPTED_MARKER_IDS:
        # Ignore this message
        return

    cam_marker_id =  (marker_id // 10) * 10
    direction_int = marker_id - cam_marker_id

    # Sicherstellen, dass positions[cid] initialisiert ist, um KeyError zu vermeiden (insb. für Kamera 50)
    if cid not in positions:
        positions[cid] = {"rvec": np.zeros((3,1)), "tvec": np.zeros((3,1))}

    # tvec und rvec umwandlung (Koordinatentransformation für 2D XY-Welt)
    tvec_room_camera = np.array([data["tvec"][2], -data["tvec"][0], -data["tvec"][1]], dtype=float).reshape(3, 1)
    rvec_room_camera = np.array([-data["rvec"][0], -data["rvec"][2], (-data["rvec"][1] + (2*np.pi)) % (2 * np.pi) - np.pi], dtype=float).reshape(3, 1)

    # Validate
    if cid not in positions and cam_marker_id != 0:
        return
    if direction_int > 3 or direction_int == 0:
        print(f"direction_int {direction_int} must be [1,2,3]")
        return
    if cam_marker_id not in CAMERA_IDS:
        print(f"camera_id {cam_marker_id} not in CAMERA_IDS")
        return

    if cam_marker_id == 50:
        rvec_delta = get_rvec_offset(direction_int)
        R1, _ = cv2.Rodrigues(np.asarray(rvec_room_camera, dtype=float).reshape(3, 1))
        R3, _ = cv2.Rodrigues(np.asarray(rvec_delta, dtype=float).reshape(3, 1))
        R_comb, _ = cv2.Rodrigues(np.zeros((3, 1), dtype=float))
        R2 = R_comb @ np.linalg.inv(R1 @ R3)
        rvec_marker_world, _ = cv2.Rodrigues(R2)
        tvec_marker_world = R2 @ tvec_room_camera

        # Kamera bleibt im Ursprung (0,0,0)
        positions[cid] = {"rvec": np.zeros((3, 1)), "tvec": np.zeros((3, 1))}
        # Marker wird fest im Weltkoordinatensystem gespeichert und bleibt stationär
        positions[f"marker_{marker_id}"] = {"rvec": rvec_marker_world, "tvec": tvec_marker_world}
        last_see[f"cam_{cid}"] = {"rvec": np.zeros((3, 1)), "tvec": np.zeros((3, 1))}
        last_see[f"marker_{marker_id}"] = {"rvec": rvec_marker_world, "tvec": tvec_marker_world}
        return  # update-Block überspringen

    else:
        rvec_delta = get_rvec_offset(direction_int)
        R1, _ = cv2.Rodrigues(np.asarray(rvec_room_camera, dtype=float).reshape(3, 1))
        R2, _ = cv2.Rodrigues(np.asarray(positions[cid]["rvec"], dtype=float).reshape(3, 1))
        R3, _ = cv2.Rodrigues(np.asarray(rvec_delta, dtype=float).reshape(3, 1))
        R_comb = R2 @ R1 @ R3
        rvec_new, _ = cv2.Rodrigues(R_comb)

        tvec_new = R2 @ tvec_room_camera +  positions[cid]["tvec"]

    # update
    if cam_marker_id != 50:
        if cam_marker_id not in positions:
            positions[cam_marker_id] = {"rvec": rvec_new, "tvec": tvec_new}
        # Absicherung: falls cam_marker_id noch nicht in last_see, initialisieren
        if cam_marker_id not in last_see:
            last_see[cam_marker_id] = {"rvec": np.zeros((3, 1)), "tvec": np.zeros((3, 1))}
        t_prev_see, r_prev_see = last_see[cam_marker_id]["tvec"], positions[cam_marker_id]["rvec"]
        t_prev, r_prev = positions[cam_marker_id]["tvec"], positions[cam_marker_id]["rvec"]
        if is_outlier(t_prev_see, r_prev_see, tvec_new, rvec_new):
            print("Pose-Ausreißer – ignorieren")
        else:
            tvec_filt, rvec_filt = lowpass_tvec(t_prev ,tvec_new, r_prev, rvec_new)
            positions[cam_marker_id] = {"rvec": rvec_filt, "tvec": tvec_filt}
        last_see[cam_marker_id] = {"rvec": rvec_new, "tvec": tvec_new}

    # --- Zusätzliche globale Kameraposition und Rotation bestimmen ---
    # Marker 0 wird als Referenzanker verwendet
    if marker_id == 0:
        # Berechne globale Kameraposition im Weltkoordinatensystem:
        # cam_pos_world = - R.T @ tvec
        R_cam, _ = cv2.Rodrigues(rvec_new)
        t_cam = tvec_new.reshape(3)
        cam_pos_world = - R_cam.T @ t_cam
        global_cam_pos[cid] = cam_pos_world
        global_cam_rvec[cid] = rvec_new.reshape(3)

def lowpass_tvec(t_prev: np.ndarray, t_raw: np.ndarray, r_prev: np.ndarray, r_raw: np.ndarray):
    tvec = ALPHA_T * t_raw + (1 - ALPHA_T) * t_prev

    # rvec
    for i in range(0,3):
        diff = r_raw[i] - r_prev[i]
        if diff > np.pi:
            r_raw[i] = r_raw[i] - 2 * np.pi
        elif diff < -np.pi:
            r_raw[i] = r_raw[i] + 2 * np.pi

    rvec = ALPHA_R * r_raw + (1 - ALPHA_R) * r_prev
    rvec = (rvec + np.pi) % (2 * np.pi) - np.pi
    return tvec, rvec

def is_outlier(t_old, r_old, t_new, r_new):
    # Δ-Position
    if np.linalg.norm(t_new - t_old) > MAX_T_JUMP:
        return True

    # Δ-Rotation (Rodrigues → Winkel)
    R_old, _ = cv2.Rodrigues(r_old);  R_new, _ = cv2.Rodrigues(r_new)
    R_delta  = R_old.T @ R_new
    ang_deg  = np.rad2deg(np.arccos(np.clip((np.trace(R_delta)-1)/2, -1, 1)))
    return ang_deg > MAX_R_JUMP

def get_rvec_offset(direction_int):
    if direction_int == 1:
        angle_rad = np.deg2rad(-90)
    elif direction_int == 2:
        angle_rad = np.deg2rad(0)
    elif direction_int == 3:
        angle_rad = np.deg2rad(90)
    else:
        raise ValueError
    rvec_delta = np.array([0.0, 0.0, angle_rad])
    return rvec_delta

# ---------- Matplotlib-Setup ---------------------------------------------------
plt.style.use("fast")   # weniger Overhead
fig  = plt.figure(figsize=(8, 6))
ax   = fig.add_subplot(111)
ax.set(xlim=(-2, 2), ylim=(-2, 2),
       xlabel="X [m]", ylabel="Y [m]")
ax.set_aspect('equal', 'box')

# pro Kamera halten wir Handles fest, damit wir nur die Daten aktualisieren
handles = {}   # cid → {"lines": [Line2D, Line2D], "text": Text}
marker_handles = {}  # marker_id → Circle
AXLEN   = 0.1
FONT_SIZE = 14
TEXT_OFFSET = 0.15

def ensure_artist(cid):
    """Erzeugt (falls nötig) 2 Linien + Textobjekt für eine Kamera."""
    if cid in handles:
        return
    colors = ["r", "g"]             # X, Y axes lines
    lines  = [ax.plot([], [], c=c, lw=2)[0] for c in colors]
    txt = ax.text(
        0, 0, f"{cid}",
        color="y", weight="bold",
        fontsize=FONT_SIZE,
        ha="center", va="bottom")

    handles[cid] = {"lines": lines, "text": txt}

def ensure_marker_artist(marker_id):
    # Only create marker artist if marker_id is in accepted list
    if marker_id not in ACCEPTED_MARKER_IDS:
        return
    if marker_id in marker_handles:
        return
    circle = plt.Circle((0,0), 0.05, color='cyan', fill=True)
    ax.add_patch(circle)
    txt = ax.text(0, 0, f"{marker_id}", color='cyan', fontsize=FONT_SIZE, ha='center', va='center')
    marker_handles[marker_id] = {"circle": circle, "text": txt}

def redraw(_=None):
    artists = []                                # → für FuncAnimation‐Rückgabe

    # Zeichne Kamera(s) als Ursprung mit Richtung (Yaw) und Position
    for cid, pose in positions.items():
        if isinstance(cid, str) and cid.startswith("marker_"):
            # Marker stationär im Weltkoordinatensystem als Punkte
            marker_id = cid
            # Extract numeric marker id from string
            try:
                marker_num = int(marker_id.split("_")[1])
            except Exception:
                continue
            # Only draw markers in accepted list
            if marker_num not in ACCEPTED_MARKER_IDS:
                continue
            ensure_marker_artist(marker_id)
            pos = pose["tvec"].ravel()
            # Nur X und Y für 2D
            x, y = pos[0], pos[1]
            marker_handles[marker_id]["circle"].center = (x, y)
            marker_handles[marker_id]["text"].set_position((x, y + 0.07))
            artists.append(marker_handles[marker_id]["circle"])
            artists.append(marker_handles[marker_id]["text"])
            continue

        # Kamera als Ursprung mit Richtung und Position im Weltkoordinatensystem
        ensure_artist(cid)
        R_mat, _ = cv2.Rodrigues(pose["rvec"])
        # Hole globale Position der Kamera, falls vorhanden, sonst (0,0,0)
        cam_pos = global_cam_pos.get(cid, np.zeros(3))
        # Position in 2D XY Ebene
        o_xy = np.array([cam_pos[0], cam_pos[1]])

        # Extrahiere Yaw aus rvec (Rotation um Z-Achse)
        # Yaw angle (rotation about Z axis) from rotation matrix:
        yaw = np.arctan2(R_mat[1,0], R_mat[0,0])

        # Berechne Endpunkte der Achsenlinien im 2D XY-Ebene entsprechend Yaw
        # X-Achse (rot) zeigt Richtung der Kamera (yaw)
        x_axis_end = o_xy + np.array([np.cos(yaw), np.sin(yaw)]) * AXLEN
        # Y-Achse (grün) um 90° versetzt
        y_axis_end = o_xy + np.array([-np.sin(yaw), np.cos(yaw)]) * AXLEN

        # Setze Linien-Daten
        lines = handles[cid]["lines"]
        # X axis line (rot)
        lines[0].set_data([o_xy[0], x_axis_end[0]], [o_xy[1], x_axis_end[1]])
        # Y axis line (grün)
        lines[1].set_data([o_xy[0], y_axis_end[0]], [o_xy[1], y_axis_end[1]])
        artists.extend(lines)

        # Label Kamera leicht oberhalb des Ursprungspunktes
        txt = handles[cid]["text"]
        txt.set_position((o_xy[0], o_xy[1] + TEXT_OFFSET))
        artists.append(txt)

    return artists                              # <- wichtig bei blit=True


# ---------- (optional) Dummy-Daten zum Testen ---------------------------------
# beim echten Betrieb einfach den Simulator-Thread weglassen
positions.setdefault(0, {"rvec": np.zeros((3, 1)),
                         "tvec": np.zeros((3, 1))})
# def simulator():
#     while True:
#         time.sleep(1)
#         #positions[0]["tvec"][2, 0] += 0.05   # Kamera 0 hebt sich in Z-Richtung
# threading.Thread(target=simulator, daemon=True).start()

# ---------- Animation starten --------------------------------------------------
fps = 60
ani = FuncAnimation(fig, redraw, interval=1_000//fps)
plt.show()

# (plt.show() blockiert; Graceful-Exit über das GUI-Fenster)
