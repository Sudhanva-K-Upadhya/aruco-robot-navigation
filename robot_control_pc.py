"""
Overhead Camera ArUco Robot Navigation - PC Side
=================================================
- Live preview shown immediately on launch (tkinter window)
- START button enables sending commands to ESP32
- STOP button halts robot at any time
- Detects robot + target ArUco markers via overhead webcam
- Forward / Inverse kinematics, P-controller
- Sends (omega_left, omega_right) to ESP32 via UDP
- Stops automatically when target marker disappears (robot on top)
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import socket
import time
import math
import tkinter as tk
from tkinter import font as tkfont
from PIL import Image, ImageTk   # pip install Pillow

# ─────────────────────────────────────────────
#  USER PARAMETERS — edit these
# ─────────────────────────────────────────────

ROBOT_MARKER_ID  = 1
TARGET_MARKER_ID = 0

ARUCO_DICT = aruco.DICT_4X4_50

CAMERA_INDEX = 0
FRAME_WIDTH  = 1280
FRAME_HEIGHT = 720

ESP32_IP   = "192.168.4.1"
ESP32_PORT = 4210

WHEEL_RADIUS = 0.033
WHEEL_BASE   = 0.16
MAX_RPM      = 150
MAX_OMEGA    = (MAX_RPM / 60.0) * 2 * math.pi

LINEAR_KP      = 0.4
ANGULAR_KP     = 1.8
PIXEL_TO_METRE = 0.001

VISION_HZ = 30

# ─────────────────────────────────────────────
#  KINEMATICS
# ─────────────────────────────────────────────

def forward_kinematics(omega_left, omega_right):
    v_left  = omega_left  * WHEEL_RADIUS
    v_right = omega_right * WHEEL_RADIUS
    v = (v_right + v_left) / 2.0
    w = (v_right - v_left) / WHEEL_BASE
    return v, w


def inverse_kinematics(v, w):
    v_left  = v - (w * WHEEL_BASE / 2.0)
    v_right = v + (w * WHEEL_BASE / 2.0)
    omega_left  = v_left  / WHEEL_RADIUS
    omega_right = v_right / WHEEL_RADIUS
    scale = max(abs(omega_left), abs(omega_right), MAX_OMEGA) / MAX_OMEGA
    omega_left  /= scale
    omega_right /= scale
    return omega_left, omega_right


# ─────────────────────────────────────────────
#  UDP
# ─────────────────────────────────────────────

def send_command(sock, omega_left, omega_right):
    msg = f"L{omega_left:.2f},R{omega_right:.2f}\n"
    try:
        sock.sendto(msg.encode(), (ESP32_IP, ESP32_PORT))
    except Exception:
        pass


def send_stop(sock):
    send_command(sock, 0.0, 0.0)


# ─────────────────────────────────────────────
#  ARUCO HELPERS
# ─────────────────────────────────────────────

def get_marker_center(corners):
    pts = corners[0].reshape((4, 2))
    return int(pts[:, 0].mean()), int(pts[:, 1].mean())


def get_marker_angle(corners):
    pts = corners[0].reshape((4, 2))
    dx = pts[1][0] - pts[0][0]
    dy = pts[1][1] - pts[0][1]
    return math.atan2(dy, dx)


# ─────────────────────────────────────────────
#  TKINTER GUI APP
# ─────────────────────────────────────────────

class RobotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Overhead Robot Navigation")
        self.root.configure(bg="#1e1e2e")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.running      = False
        self.goal_reached = False
        self.period       = 1.0 / VISION_HZ

        # Camera
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # ArUco
        aruco_dict    = aruco.getPredefinedDictionary(ARUCO_DICT)
        aruco_params  = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(aruco_dict, aruco_params)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

        # ── Build UI ──────────────────────────────
        self.display_w = 960
        self.display_h = 540
        self.canvas = tk.Canvas(root, width=self.display_w, height=self.display_h,
                                bg="black", highlightthickness=0)
        self.canvas.pack(padx=10, pady=10)

        self.status_var = tk.StringVar(value="Preview active — press START to send commands")
        status_bar = tk.Label(root, textvariable=self.status_var,
                              bg="#313244", fg="#cdd6f4",
                              font=("Consolas", 11), anchor="w", padx=8)
        status_bar.pack(fill=tk.X, padx=10)

        btn_frame = tk.Frame(root, bg="#1e1e2e")
        btn_frame.pack(pady=10)
        btn_font = tkfont.Font(family="Helvetica", size=13, weight="bold")

        self.start_btn = tk.Button(
            btn_frame, text="▶  START", width=14, font=btn_font,
            bg="#a6e3a1", fg="#1e1e2e", activebackground="#94e2d5",
            relief="flat", cursor="hand2", command=self.on_start
        )
        self.start_btn.pack(side=tk.LEFT, padx=12)

        self.stop_btn = tk.Button(
            btn_frame, text="■  STOP", width=14, font=btn_font,
            bg="#f38ba8", fg="#1e1e2e", activebackground="#eba0ac",
            relief="flat", cursor="hand2", command=self.on_stop,
            state=tk.DISABLED
        )
        self.stop_btn.pack(side=tk.LEFT, padx=12)

        self.reset_btn = tk.Button(
            btn_frame, text="↺  RESET", width=14, font=btn_font,
            bg="#fab387", fg="#1e1e2e", activebackground="#f9e2af",
            relief="flat", cursor="hand2", command=self.on_reset
        )
        self.reset_btn.pack(side=tk.LEFT, padx=12)

        # Telemetry row
        tele_frame = tk.Frame(root, bg="#1e1e2e")
        tele_frame.pack(pady=(0, 12))
        self.tele_vars = {
            "Distance":    tk.StringVar(value="—"),
            "Heading Err": tk.StringVar(value="—"),
            "v (m/s)":     tk.StringVar(value="—"),
            "ω (°/s)":     tk.StringVar(value="—"),
            "ωL (rad/s)":  tk.StringVar(value="—"),
            "ωR (rad/s)":  tk.StringVar(value="—"),
        }
        for label, var in self.tele_vars.items():
            col = tk.Frame(tele_frame, bg="#313244", padx=10, pady=4)
            col.pack(side=tk.LEFT, padx=6)
            tk.Label(col, text=label, bg="#313244", fg="#89b4fa",
                     font=("Consolas", 9)).pack()
            tk.Label(col, textvariable=var, bg="#313244", fg="#cdd6f4",
                     font=("Consolas", 11, "bold")).pack()

        self._update()

    # ── Button callbacks ──────────────────────

    def on_start(self):
        if self.goal_reached:
            return
        self.running = True
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.status_var.set("● RUNNING — sending commands to ESP32")

    def on_stop(self):
        self.running = False
        send_stop(self.sock)
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.status_var.set("■ STOPPED by user")

    def on_reset(self):
        self.running      = False
        self.goal_reached = False
        send_stop(self.sock)
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.status_var.set("↺ RESET — press START to begin again")
        for v in self.tele_vars.values():
            v.set("—")

    def on_close(self):
        self.running = False
        send_stop(self.sock)
        self.sock.close()
        self.cap.release()
        self.root.destroy()

    # ── Main vision + control loop ────────────

    def _update(self):
        t_start = time.time()

        ret, frame = self.cap.read()
        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)

            robot_data  = None
            target_data = None

            if ids is not None:
                ids_flat = ids.flatten()
                for i, mid in enumerate(ids_flat):
                    if mid == ROBOT_MARKER_ID:
                        robot_data = (corners[i],
                                      get_marker_center(corners[i]),
                                      get_marker_angle(corners[i]))
                    elif mid == TARGET_MARKER_ID:
                        target_data = (corners[i],
                                       get_marker_center(corners[i]))

            if ids is not None:
                aruco.drawDetectedMarkers(frame, corners, ids)

            overlay_color = (255, 255, 0)
            status_line   = ""

            if self.goal_reached:
                overlay_color = (0, 255, 0)
                status_line   = "GOAL REACHED — Robot stopped"
                cv2.putText(frame, "GOAL REACHED", (20, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 4)

            elif not self.running:
                status_line = "PREVIEW — press START to enable control"

            elif robot_data is None:
                send_stop(self.sock)
                status_line   = "WARNING: Robot marker NOT detected"
                overlay_color = (0, 100, 255)

            elif target_data is None:
                send_stop(self.sock)
                self.goal_reached = True
                self.running      = False
                self.start_btn.config(state=tk.DISABLED)
                self.stop_btn.config(state=tk.DISABLED)
                self.status_var.set("✔ GOAL REACHED — press RESET to restart")
                status_line = "Target hidden → GOAL REACHED"

            else:
                _, (rx, ry), robot_angle = robot_data
                _, (tx, ty)             = target_data

                dx = tx - rx
                dy = ty - ry

                distance_px   = math.hypot(dx, dy)
                distance_m    = distance_px * PIXEL_TO_METRE
                desired_angle = math.atan2(dy, dx)
                heading_error = desired_angle - robot_angle
                heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

                v_desired = LINEAR_KP * distance_m
                w_desired = ANGULAR_KP * heading_error
                v_desired *= max(math.cos(heading_error), 0.0)

                omega_l, omega_r   = inverse_kinematics(v_desired, w_desired)
                v_actual, w_actual = forward_kinematics(omega_l, omega_r)

                send_command(self.sock, omega_l, omega_r)

                self.tele_vars["Distance"].set(f"{distance_px:.0f} px")
                self.tele_vars["Heading Err"].set(f"{math.degrees(heading_error):.1f}°")
                self.tele_vars["v (m/s)"].set(f"{v_actual:.3f}")
                self.tele_vars["ω (°/s)"].set(f"{math.degrees(w_actual):.1f}")
                self.tele_vars["ωL (rad/s)"].set(f"{omega_l:.2f}")
                self.tele_vars["ωR (rad/s)"].set(f"{omega_r:.2f}")

                status_line = (f"dist={distance_px:.0f}px  "
                               f"err={math.degrees(heading_error):.1f}°  "
                               f"v={v_actual:.3f}m/s  "
                               f"wL={omega_l:.2f}  wR={omega_r:.2f}")

                cv2.arrowedLine(frame, (rx, ry), (tx, ty), (0, 165, 255), 2)

            cv2.putText(frame, status_line,
                        (10, FRAME_HEIGHT - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, overlay_color, 1)

            preview     = cv2.resize(frame, (self.display_w, self.display_h))
            preview_rgb = cv2.cvtColor(preview, cv2.COLOR_BGR2RGB)
            img         = ImageTk.PhotoImage(Image.fromarray(preview_rgb))
            self.canvas.imgtk = img
            self.canvas.create_image(0, 0, anchor=tk.NW, image=img)

        elapsed  = time.time() - t_start
        delay_ms = max(1, int((self.period - elapsed) * 1000))
        self.root.after(delay_ms, self._update)


# ─────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────

if __name__ == "__main__":
    root = tk.Tk()
    app  = RobotApp(root)
    root.mainloop()
