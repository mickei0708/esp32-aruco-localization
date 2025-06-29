#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json, math
import numpy as np, cv2, paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from PIL import Image

# ------------------- Weltkoordinaten-Konfiguration -------------------
d = 0.02 / 2
ACCEPTED_MARKER_IDS = [0,1,2,3, 10,11,12,13, 20,21,22,23, 30,31,32,33, 40,41,42,43 ,50,51,52,53, 60,61,62,63]

R_ref0 = np.column_stack(([0,1,0], [0,0,1], [1,0,0]))
R_ref1 = np.column_stack(([-1,0,0], [0,0,1], [0,1,0]))
R_ref2 = np.column_stack(([0,-1,0], [0,0,1], [-1,0,0]))
R_ref3 = np.column_stack(([1,0,0], [0,0,1], [0,-1,0]))

reference_markers = {
    0: {"pos": np.array([ d, 0.0, 0.0]), "R": R_ref0},
    1: {"pos": np.array([0.0,  d, 0.0]), "R": R_ref1},
    2: {"pos": np.array([-d, 0.0, 0.0]), "R": R_ref2},
    3: {"pos": np.array([0.0, -d, 0.0]), "R": R_ref3}
}

camera_marker_offsets = {
    10: {
        10: {"t": np.array([0.0, 0.0, 0.10]), "R": np.eye(3)},
        11: {"t": np.array([0.10, 0.0, 0.0]), "R": cv2.Rodrigues(np.array([0, -math.pi/2, 0]))[0]},
        12: {"t": np.array([0.0, 0.0, -0.10]), "R": cv2.Rodrigues(np.array([0,  math.pi, 0]))[0]},
        13: {"t": np.array([-0.10,0.0, 0.0]), "R": cv2.Rodrigues(np.array([0,  math.pi/2, 0]))[0]}
    }
}
marker_to_camera = {}
for cam_id, markers in camera_marker_offsets.items():
    for marker_id in markers:
        marker_to_camera[marker_id] = cam_id

solved_cameras = {}
unsolved_cameras = set()

# ------------------- MQTT Setup -------------------
BROKER = "192.168.2.61"; PORT = 1883
mqtt_client = mqtt.Client()
mqtt_client.connect(BROKER, PORT)
mqtt_client.subscribe("camera/+/pose")

positions = {}
positions["cam_0"] = {"tvec": np.array([0.0, 0.0]), "label": "Cam 0"}  # Ursprung immer anzeigen

def process_pose(data):
    cid = data["camera_id"]
    mid = data["marker_id"]
    tvec = np.array(data["tvec"], dtype=float)
    rvec = np.array(data["rvec"], dtype=float)
    R_cam_marker, _ = cv2.Rodrigues(rvec)

    if mid not in ACCEPTED_MARKER_IDS:
        return

    if mid in reference_markers:
        ref = reference_markers[mid]
        R_marker_global = ref["R"]
        P_marker_global = ref["pos"]
        cam_pos_marker = - R_cam_marker.T.dot(tvec)
        cam_pos_global = P_marker_global + R_marker_global.dot(cam_pos_marker)
        cam_R_global = R_marker_global.dot(R_cam_marker.T)
        solved_cameras[cid] = {"pos": cam_pos_global, "R": cam_R_global}
        unsolved_cameras.discard(cid)
        print(f"✅ Kamera {cid} gelöst bei {cam_pos_global[:2]} (X,Y)")
        positions[f"cam_{cid}"] = {"tvec": cam_pos_global[:2], "label": f"Cam {cid}"}
        return

    if mid in marker_to_camera:
        target_cam = marker_to_camera[mid]
        if target_cam in solved_cameras and cid not in solved_cameras:
            solved = solved_cameras[target_cam]
            P_marker_global = solved["pos"] + solved["R"].dot(camera_marker_offsets[target_cam][mid]["t"])
            R_marker_global = solved["R"].dot(camera_marker_offsets[target_cam][mid]["R"])
            cam_pos_marker = - R_cam_marker.T.dot(tvec)
            cam_pos_global = P_marker_global + R_marker_global.dot(cam_pos_marker)
            cam_R_global = R_marker_global.dot(R_cam_marker.T)
            solved_cameras[cid] = {"pos": cam_pos_global, "R": cam_R_global}
            unsolved_cameras.discard(cid)
            print(f"🔗 Kamera {cid} gelöst via {target_cam} bei {cam_pos_global[:2]}")
            positions[f"cam_{cid}"] = {"tvec": cam_pos_global[:2], "label": f"Cam {cid}"}
            return

        if cid in solved_cameras and target_cam not in solved_cameras:
            solved = solved_cameras[cid]
            P_marker_global = solved["pos"] + solved["R"].dot(tvec)
            R_marker_global = solved["R"].dot(R_cam_marker)
            marker_offset = camera_marker_offsets[target_cam][mid]
            R_cam_offset = marker_offset["R"]
            t_cam_offset = marker_offset["t"]
            cam_R_global = R_marker_global.dot(R_cam_offset.T)
            cam_pos_global = P_marker_global - cam_R_global.dot(t_cam_offset)
            solved_cameras[target_cam] = {"pos": cam_pos_global, "R": cam_R_global}
            unsolved_cameras.discard(target_cam)
            print(f"🔗 Kamera {target_cam} gelöst via {cid} bei {cam_pos_global[:2]}")
            positions[f"cam_{target_cam}"] = {"tvec": cam_pos_global[:2], "label": f"Cam {target_cam}"}
            return

def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode())
    cam_id = data.get("camera_id")
    if cam_id is not None and cam_id not in solved_cameras:
        unsolved_cameras.add(cam_id)
    process_pose(data)

mqtt_client.on_message = on_message
mqtt_client.loop_start()

# ------------------- Matplotlib Visualisierung -------------------
fig, ax = plt.subplots(figsize=(6,6))
ax.set_title("2D Weltkoordinatensystem")
ax.set_xlabel("X [cm]")
ax.set_ylabel("Y [cm]")
ax.set_xlim(-100, 100)
ax.set_ylim(-100, 100)
ax.grid(True)

color_map = {
    "cam_0": "green",
    "cam_50": "red",
    "cam_10": "blue",
    "cam_20": "orange",
    "cam_30": "purple",
    "cam_40": "cyan",
    "cam_60": "magenta"
}

camera_img = Image.open("utils/camera_icon.png").convert("RGBA").resize((40, 40))

def update_plot(frame):
    ax.clear()
    ax.set_title("2D Weltkoordinatensystem")
    ax.set_xlabel("X [cm]")
    ax.set_ylabel("Y [cm]")
    ax.set_xlim(-100, 100)
    ax.set_ylim(-100, 100)
    ax.grid(True)

    handles = []
    labels = []

    for key, entry in sorted(positions.items()):
        x, y = entry["tvec"] * 100
        label = entry["label"]
        color = color_map.get(key, "black")

        if key == "cam_50" and 50 in solved_cameras:
            cam_R_global = solved_cameras[50]["R"]
            forward_vector = cam_R_global @ np.array([0, 0, 1])
            forward_2d = forward_vector[:2]
            angle = math.degrees(math.atan2(forward_2d[1], forward_2d[0]))
            rotated_img = camera_img.rotate(angle, expand=True)
            img_box = OffsetImage(rotated_img, zoom=0.5)
            ab = AnnotationBbox(img_box, (x, y), frameon=False)
            ax.add_artist(ab)
            # Dummy scatter for legend
            sc = ax.scatter([], [], color="black", s=60)
            handles.append(sc)
            labels.append(label)
        else:
            sc = ax.scatter(x, y, color=color, s=60)
            handles.append(sc)
            labels.append(label)

    ax.legend(handles, labels, loc="upper right")

ani = FuncAnimation(fig, update_plot, interval=500)
plt.show()