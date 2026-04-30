import cv2
import math
import tkinter as tk
from tkinter import messagebox, ttk
from PIL import Image, ImageTk

class CalibratorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Camera Calibrator")
        
        self.cap = None
        self.points = []
        self.pixel_dist = 0.0

        # --- UI Layout ---
        # Top Control Bar
        top_bar = tk.Frame(root, pady=10)
        top_bar.pack(fill=tk.X)

        tk.Label(top_bar, text="Select Camera Index:").pack(side=tk.LEFT, padx=5)
        self.cam_id_var = tk.StringVar(value="0")
        self.cam_selector = ttk.Combobox(top_bar, textvariable=self.cam_id_var, width=5)
        self.cam_selector['values'] = ("0", "1", "2", "3")
        self.cam_selector.pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = tk.Button(top_bar, text="Connect Cam", command=self.init_camera, bg="#89b4fa")
        self.connect_btn.pack(side=tk.LEFT, padx=5)

        # Video Canvas
        self.canvas = tk.Canvas(root, width=1280, height=720, bg="black")
        self.canvas.pack()
        self.canvas.bind("<Button-1>", self.on_click)

        # Bottom Controls
        controls = tk.Frame(root, pady=10)
        controls.pack()

        tk.Label(controls, text="Physical Distance (meters):").pack(side=tk.LEFT)
        self.dist_entry = tk.Entry(controls)
        self.dist_entry.insert(0, "0.5")
        self.dist_entry.pack(side=tk.LEFT, padx=5)

        self.calc_btn = tk.Button(controls, text="Calculate Constant", command=self.calculate)
        self.calc_btn.pack(side=tk.LEFT, padx=5)

        self.result_var = tk.StringVar(value="Select camera and connect to begin")
        tk.Label(root, textvariable=self.result_var, font=("Consolas", 12), fg="blue").pack(pady=5)

        self.init_camera() # Attempt default
        self.update_frame()

    def init_camera(self):
        if self.cap:
            self.cap.release()
        
        try:
            cam_idx = int(self.cam_id_var.get())
            self.cap = cv2.VideoCapture(cam_idx)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            self.result_var.set(f"Connected to Cam {cam_idx}. Click two points.")
        except Exception as e:
            messagebox.showerror("Error", f"Could not open camera: {e}")

    def on_click(self, event):
        if len(self.points) >= 2:
            self.points = []
        
        self.points.append((event.x, event.y))
        
        if len(self.points) == 2:
            x1, y1 = self.points[0]
            x2, y2 = self.points[1]
            self.pixel_dist = math.hypot(x2 - x1, y2 - y1)
            self.result_var.set(f"Pixel Distance: {self.pixel_dist:.2f} px. Enter meters and calculate.")

    def calculate(self):
        if len(self.points) < 2:
            messagebox.showwarning("Error", "Please click two points first!")
            return
        
        try:
            real_dist = float(self.dist_entry.get())
            constant = real_dist / self.pixel_dist
            self.result_var.set(f"RESULT: PIXEL_TO_METRE = {constant:.6f}")
            print(f"\nCopy this to robot_control_pc.py:\nPIXEL_TO_METRE = {constant:.6f}")
        except ValueError:
            messagebox.showerror("Error", "Enter a valid number for distance.")

    def update_frame(self):
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                for p in self.points:
                    cv2.circle(frame, p, 5, (0, 255, 0), -1)
                if len(self.points) == 2:
                    cv2.line(frame, self.points[0], self.points[1], (255, 0, 0), 2)

                img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                # Resize for display if frame is larger than screen
                img_pil = Image.fromarray(img)
                img_tk = ImageTk.PhotoImage(img_pil)
                self.canvas.image = img_tk
                self.canvas.create_image(0, 0, anchor=tk.NW, image=img_tk)
        
        self.root.after(10, self.update_frame)

if __name__ == "__main__":
    root = tk.Tk()
    app = CalibratorApp(root)
    root.mainloop()