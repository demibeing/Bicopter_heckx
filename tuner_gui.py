import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import queue
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import collections

class BicopterTuner:
    def __init__(self, root):
        self.root = root
        self.root.title("Bicopter Pro PID Tuner - Large View")
        
        # 1. Increase the starting window size
        self.root.geometry("1400x900")
        
        self.serial_thread = None
        self.q = queue.Queue()
        self.is_connected = False

        # Define Large Fonts
        self.label_font = ("Arial", 12, "bold")
        self.slider_font = ("Arial", 10)
        self.header_font = ("Arial", 14, "bold")

        # --- Layout ---
        # Left Side: Controls (Wider for longer sliders)
        self.left_frame = ttk.Frame(root, padding=20)
        self.left_frame.grid(row=0, column=0, sticky="ns")

        # Right Side: Real-time Plot
        self.right_frame = ttk.Frame(root, padding=10)
        self.right_frame.grid(row=0, column=1, sticky="nsew")
        root.columnconfigure(1, weight=1)
        root.rowconfigure(0, weight=1)

        # --- Connection Setup ---
        conn_frame = ttk.LabelFrame(self.left_frame, text=" Connection ", padding=10)
        conn_frame.pack(fill="x", pady=10)
        
        self.port_cb = ttk.Combobox(conn_frame, values=[p.device for p in serial.tools.list_ports.comports()], 
                                    width=20, font=self.slider_font)
        self.port_cb.pack(side=tk.LEFT, padx=10)
        
        self.btn_connect = tk.Button(conn_frame, text="CONNECT", command=self.toggle_connection, 
                                     bg="#e1e1e1", font=self.label_font, padx=10)
        self.btn_connect.pack(side=tk.LEFT, padx=10)

        # --- Tuning Sliders (Two Massive Columns) ---
        self.sliders = {}
        tune_container = ttk.Frame(self.left_frame)
        tune_container.pack(fill="both", expand=True)

        # Roll Column
        roll_col = ttk.LabelFrame(tune_container, text=" ROLL (Servos) ", padding=15)
        roll_col.pack(side=tk.LEFT, fill="both", expand=True, padx=5)
        self.add_slider(roll_col, "Angle P (AKP)", "AKP", 0, 15, 3.0)
        self.add_slider(roll_col, "Rate P (RKP)", "RKP", 0, 2.0, 0.2)
        self.add_slider(roll_col, "Rate I (RKI)", "RKI", 0, 1.0, 0.05)
        self.add_slider(roll_col, "Rate D (RKD)", "RKD", 0, 0.5, 0.02)

        # Pitch Column
        pitch_col = ttk.LabelFrame(tune_container, text=" PITCH (Motors) ", padding=15)
        pitch_col.pack(side=tk.LEFT, fill="both", expand=True, padx=5)
        self.add_slider(pitch_col, "Angle P (PAKP)", "PAKP", 0, 15, 3.0)
        self.add_slider(pitch_col, "Rate P (PRKP)", "PRKP", 0, 2.0, 0.15)
        self.add_slider(pitch_col, "Rate I (PRKI)", "PRKI", 0, 1.0, 0.05)
        self.add_slider(pitch_col, "Rate D (PRKD)", "PRKD", 0, 0.5, 0.05)

        # Commands (Bigger Buttons)
        cmd_frame = ttk.Frame(self.left_frame, padding=10)
        cmd_frame.pack(fill="x")
        tk.Button(cmd_frame, text="SYNC FROM DRONE", font=self.label_font, 
                  command=self.sync_drone, bg="#d1e7ff").pack(side=tk.LEFT, expand=True, fill="x", padx=5)
        tk.Button(cmd_frame, text="SAVE TO EEPROM", font=self.label_font, 
                  command=self.save_drone, bg="#d1ffd1").pack(side=tk.LEFT, expand=True, fill="x", padx=5)

        # --- Telemetry Status ---
        self.lbl_telem = tk.Label(self.left_frame, text="Telemetry: Waiting...", 
                                  font=("Courier", 14, "bold"), justify=tk.LEFT, fg="#333")
        self.lbl_telem.pack(pady=20)

        # --- Plotting Setup ---
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.right_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        self.xdata = collections.deque(maxlen=300)
        self.roll_y = collections.deque(maxlen=300)
        self.pitch_y = collections.deque(maxlen=300)
        self.roll_line, = self.ax.plot([], [], label="Roll Angle", color="blue", linewidth=2)
        self.pitch_line, = self.ax.plot([], [], label="Pitch Angle", color="red", linewidth=2)
        
        self.ax.set_ylim(-50, 50)
        self.ax.set_title("Real-time Flight Data", fontsize=16)
        self.ax.legend(loc="upper right", fontsize=12)
        self.ax.grid(True, which='both', linestyle='--', alpha=0.5)
        self.start_time = time.time()

        self.root.after(50, self.process_queue)

    def add_slider(self, parent, label, tag, vmin, vmax, vdef):
        frame = ttk.Frame(parent)
        frame.pack(fill="x", pady=10)
        
        ttk.Label(frame, text=label, font=self.label_font).pack(anchor="w")
        
        # length: Horizontal size in pixels
        # width: Vertical thickness of the slider bar
        # sliderlength: Size of the actual handle you grab
        s = tk.Scale(frame, from_=vmin, to=vmax, resolution=0.01, orient=tk.HORIZONTAL,
                     length=400, width=25, sliderlength=40, font=self.slider_font,
                     command=lambda val: self.send_gain(tag, val))
        s.set(vdef)
        s.pack(fill="x", pady=5)
        self.sliders[tag] = s

    def toggle_connection(self):
        if not self.is_connected:
            try:
                port = self.port_cb.get()
                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.is_connected = True
                self.btn_connect.config(text="DISCONNECT", bg="#ffcccc")
                threading.Thread(target=self.serial_reader, daemon=True).start()
                time.sleep(1)
                self.sync_drone()
            except Exception as e:
                messagebox.showerror("Error", f"Could not connect: {e}")
        else:
            self.is_connected = False
            self.ser.close()
            self.btn_connect.config(text="CONNECT", bg="#e1e1e1")

    def send_gain(self, tag, val):
        if self.is_connected:
            msg = f"{tag} {val}\n"
            self.ser.write(msg.encode())

    def sync_drone(self):
        if self.is_connected: self.ser.write(b"REQ\n")

    def save_drone(self):
        if self.is_connected: self.ser.write(b"SAVE\n")

    def serial_reader(self):
        while self.is_connected:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line: self.q.put(line)
            except: pass

    def process_queue(self):
        while not self.q.empty():
            line = self.q.get_nowait()
            if line.startswith("TLM,"):
                parts = line.split(',')
                try:
                    r, p = float(parts[1]), float(parts[2])
                    ef, eb = parts[3], parts[4]
                    t = time.time() - self.start_time
                    self.xdata.append(t)
                    self.roll_y.append(r)
                    self.pitch_y.append(p)
                    self.roll_line.set_data(self.xdata, self.roll_y)
                    self.pitch_line.set_data(self.xdata, self.pitch_y)
                    self.ax.set_xlim(max(0, t-15), t+1) # 15 second window
                    self.canvas.draw_idle()
                    self.lbl_telem.config(text=f"ROLL: {r:>5.1f}°\nPITCH: {p:>4.1f}°\nESC F: {ef}\nESC B: {eb}")
                except: pass
            elif line.startswith("SYNC,"):
                p = line.split(',')
                tags = ["AKP", "RKP", "RKI", "RKD", "PAKP", "PRKP", "PRKI", "PRKD"]
                for i, tag in enumerate(tags):
                    if i+1 < len(p):
                        self.sliders[tag].set(float(p[i+1]))
        self.root.after(50, self.process_queue)

if __name__ == "__main__":
    root = tk.Tk()
    app = BicopterTuner(root)
    root.mainloop()
