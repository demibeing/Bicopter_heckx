# tuner_gui.py
# Requires: pyserial, matplotlib
# pip install pyserial matplotlib

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

BAUD = 115200

class SerialThread(threading.Thread):
    def __init__(self, port, baud, q):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.q = q
        self.alive = True
        self.ser = None
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except Exception as e:
            self.ser = None
            print("Serial open error:", e)
            raise

    def run(self):
        buf = ""
        while self.alive:
            try:
                data = self.ser.read(200)
            except Exception as e:
                print("Serial read error:", e)
                break
            if data:
                try:
                    s = data.decode('utf-8', errors='ignore')
                except:
                    s = str(data)
                buf += s
                # split lines
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip()
                    if line:
                        self.q.put(line)
            else:
                time.sleep(0.01)

    def write_line(self, line):
        if self.ser and self.ser.is_open:
            self.ser.write((line + '\n').encode('utf-8'))

    def close(self):
        self.alive = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except:
            pass

class TunerGUI:
    def __init__(self, root):
        self.root = root
        root.title("Bicopter PID Tuner")
        self.serial_thread = None
        self.q = queue.Queue()

        # Left frame: controls
        left = ttk.Frame(root, padding=6)
        left.grid(row=0, column=0, sticky='ns')

        # COM port selection
        ttk.Label(left, text="Serial Port:").grid(row=0, column=0, sticky='w')
        self.port_cb = ttk.Combobox(left, values=self.list_ports(), width=20)
        self.port_cb.grid(row=1, column=0, sticky='w')
        ttk.Button(left, text="Refresh", command=self.refresh_ports).grid(row=1, column=1, padx=4)

        self.connect_btn = ttk.Button(left, text="Connect", command=self.toggle_connect)
        self.connect_btn.grid(row=2, column=0, pady=6, sticky='w')

        # Sliders (AKP, RKP, RKI, RKD)
        self.sliders = {}
        sliders_def = [
            ("AKP (angle Kp)", "AKP", 0.0, 20.0, 3.0),
            ("RKP (rate Kp)", "RKP", 0.0, 5.0, 0.8),
            ("RKI (rate Ki)", "RKI", 0.0, 1.0, 0.05),
            ("RKD (rate Kd)", "RKD", 0.0, 2.0, 0.12),
        ]
        r = 3
        for label, tag, vmin, vmax, vdef in sliders_def:
            ttk.Label(left, text=label).grid(row=r, column=0, sticky='w', pady=(6,0))
            s = tk.Scale(left, from_=vmin, to=vmax, resolution=0.01, orient=tk.HORIZONTAL, length=260,
                         command=lambda val, t=tag: self.slider_changed(t, val))
            s.set(vdef)
            s.grid(row=r+1, column=0, columnspan=2)
            self.sliders[tag] = s
            r += 2

        # EEPROM save/load
        ttk.Button(left, text="Save to EEPROM", command=self.save_eeprom).grid(row=r, column=0, pady=8, sticky='w')
        ttk.Button(left, text="Load from EEPROM", command=self.load_eeprom).grid(row=r, column=1, pady=8)

        # Telemetry labels
        self.tlm_vars = {
            'roll_angle': tk.StringVar(value="roll: --"),
            'gyro_rate': tk.StringVar(value="gyro: --"),
            'target_roll': tk.StringVar(value="target: --"),
            'deflection': tk.StringVar(value="defl: --"),
        }
        rr = r + 1
        for i, (k, v) in enumerate(self.tlm_vars.items()):
            ttk.Label(left, textvariable=v).grid(row=rr + i, column=0, sticky='w')

        # Right frame: plot
        right = ttk.Frame(root, padding=6)
        right.grid(row=0, column=1, sticky='nsew')
        root.columnconfigure(1, weight=1)
        root.rowconfigure(0, weight=1)

        self.fig, self.ax = plt.subplots(figsize=(6,3))
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)

        self.xdata = collections.deque(maxlen=400)
        self.ydata = collections.deque(maxlen=400)
        self.line, = self.ax.plot([], [], label='roll_angle')
        self.ax.set_ylim(-50, 50)
        self.ax.set_xlim(0, 10)
        self.ax.grid(True)
        self.ax.legend(loc='upper right')
        self.start_time = time.time()

        # periodic GUI update
        self.root.after(50, self.process_queue)
        self.root.after(200, self.update_plot)

    def list_ports(self):
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    def refresh_ports(self):
        self.port_cb['values'] = self.list_ports()

    def toggle_connect(self):
        if self.serial_thread:
            # disconnect
            self.serial_thread.close()
            self.serial_thread = None
            self.connect_btn.config(text="Connect")
        else:
            port = self.port_cb.get()
            if not port:
                messagebox.showerror("Error", "Select a serial port first")
                return
            try:
                self.serial_thread = SerialThread(port, BAUD, self.q)
                self.serial_thread.start()
            except Exception as e:
                messagebox.showerror("Error", f"Failed to open port: {e}")
                self.serial_thread = None
                return
            self.connect_btn.config(text="Disconnect")
            # send immediate telemetry request
            time.sleep(0.05)
            self.send_line("REQTLM")

    def slider_changed(self, tag, val):
        # send command immediately when slider changes
        cmd = f"{tag}{float(val):.3f}"
        self.send_line(cmd)

    def send_line(self, line):
        if self.serial_thread:
            try:
                self.serial_thread.write_line(line)
            except Exception as e:
                print("Write error:", e)

    def save_eeprom(self):
        self.send_line("SAVE")

    def load_eeprom(self):
        self.send_line("LOAD")

    def process_queue(self):
        # called periodically on main thread
        updated = False
        while not self.q.empty():
            line = self.q.get_nowait()
            # print("RX:", line)
            if line.startswith("TLM,"):
                # TLM,roll_angle,gyro_rate,target_roll,deflection
                parts = line.split(',')
                try:
                    roll_angle = float(parts[1])
                    gyro_rate = float(parts[2])
                    target_roll = float(parts[3])
                    deflection = float(parts[4])
                except:
                    continue
                self.tlm_vars['roll_angle'].set(f"roll: {roll_angle:.2f}°")
                self.tlm_vars['gyro_rate'].set(f"gyro: {gyro_rate:.2f}°/s")
                self.tlm_vars['target_roll'].set(f"target: {target_roll:.2f}°")
                self.tlm_vars['deflection'].set(f"defl: {deflection:.0f}")

                t = time.time() - self.start_time
                self.xdata.append(t)
                self.ydata.append(roll_angle)
                updated = True
            else:
                # parse simple text responses like "AKP 4.2" or "SAVED"
                if line.startswith("AKP"):
                    try:
                        val = float(line.split()[1])
                        self.sliders['AKP'].set(val)
                    except:
                        pass
                if line.startswith("RKP"):
                    try:
                        val = float(line.split()[1])
                        self.sliders['RKP'].set(val)
                    except:
                        pass
                if line.startswith("RKI"):
                    try:
                        val = float(line.split()[1])
                        self.sliders['RKI'].set(val)
                    except:
                        pass
                if line.startswith("RKD"):
                    try:
                        val = float(line.split()[1])
                        self.sliders['RKD'].set(val)
                    except:
                        pass
                # you can inspect other lines in the console
                print("->", line)

        if updated:
            # keep x-axis window shifted to last 10 seconds
            if self.xdata:
                span = max(10.0, self.xdata[-1] - (self.xdata[0] if self.xdata else 0))
                self.ax.set_xlim(max(0, self.xdata[-1] - 10.0), self.xdata[-1] + 0.1)
            self.line.set_data(self.xdata, self.ydata)
            self.canvas.draw_idle()

        self.root.after(50, self.process_queue)

    def update_plot(self):
        # keep axes updated even if no new data
        if self.xdata:
            self.ax.relim()
            self.ax.autoscale_view(scalex=False, scaley=False)
            self.canvas.draw_idle()
        self.root.after(200, self.update_plot)

if __name__ == "__main__":
    root = tk.Tk()
    app = TunerGUI(root)
    root.mainloop()
