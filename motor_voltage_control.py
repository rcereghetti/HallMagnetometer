"""
Motor + voltage control interface (single file).

Install once:   pip install pyserial pyvisa matplotlib
Run:            python motor_voltage_control.py

Close the Arduino Serial Monitor first (only one program can use the port).

Left side : motor controls, optical sensor settings, voltage measurement
            settings, status log.
Right side: top plot    = voltage from the Keithley 2182A (optionally the
                          output current of the 6221 in a second panel)
            bottom plot = live analog value of the optical sensor, with the
                          threshold drawn as a dashed line.

Needs the matching Arduino sketch (motor_control.ino), which sends the raw
sensor value when it receives the "W 1" command.
If "Save data to CSV file" is ticked, each measurement is saved to a CSV file
(measurement_YYYYMMDD_HHMMSS.csv) in the folder you run the script from.

The motor and the voltage measurement are independent: you can use either
one alone, or both at the same time.
"""

import csv
import queue
import re
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

try:
    import pyvisa
except ImportError:          # the motor part still works without pyvisa
    pyvisa = None

# ---------- Settings (change these if needed) ----------
BAUD = 9600
DEFAULT_GPIB_2182A = "GPIB0::17::INSTR"
DEFAULT_GPIB_6221 = "GPIB0::11::INSTR"
DEFAULT_INTERVAL = 0.5       # seconds between readings
SENSOR_WINDOW_S = 30         # seconds of sensor signal shown in the plot
# --------------------------------------------------------


class App:
    def __init__(self, root):
        self.root = root
        root.title("Motor + voltage control")
        root.minsize(950, 600)

        # motor / serial state
        self.ser = None
        self.q = queue.Queue()
        self.running = False

        # measurement state
        self.mq = queue.Queue()
        self.measuring = False
        self.meas_stop = threading.Event()
        self.times, self.voltages, self.currents = [], [], []

        # optical sensor state
        self.sensor_t = deque()
        self.sensor_v = deque()
        self.sensor_t0 = None

        pad = {"padx": 8, "pady": 6}

        left = ttk.Frame(root)
        left.pack(side="left", fill="y", padx=(8, 4), pady=6)
        right = ttk.Frame(root)
        right.pack(side="right", fill="both", expand=True, padx=(4, 8), pady=6)

        # =================== MOTOR ===================
        motor = ttk.LabelFrame(left, text="Motor")
        motor.pack(fill="x")

        # --- Connection ---
        top = ttk.Frame(motor)
        top.pack(fill="x", **pad)
        ttk.Label(top, text="COM port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(top, textvariable=self.port_var,
                                     width=10, state="readonly")
        self.port_box.pack(side="left", padx=6)
        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side="left")
        self.conn_btn = ttk.Button(top, text="Connect", command=self.toggle_connect)
        self.conn_btn.pack(side="left", padx=6)

        # --- Amount ---
        mid = ttk.Frame(motor)
        mid.pack(fill="x", **pad)
        ttk.Label(mid, text="Amount (steps):").pack(side="left")
        self.amount_var = tk.StringVar(value="200")
        ttk.Entry(mid, textvariable=self.amount_var, width=10).pack(side="left", padx=6)

        # --- Buttons ---
        btns = ttk.Frame(motor)
        btns.pack(fill="x", **pad)
        btns.columnconfigure(0, weight=1)
        btns.columnconfigure(1, weight=1)
        self.buttons = []
        for i, (text, cmd) in enumerate([("Home (go to sensor)", self.home),
                                         ("Sensor?", self.ask_sensor),
                                         ("Up", self.up),
                                         ("Down", self.down)]):
            b = ttk.Button(btns, text=text, command=cmd, state="disabled")
            b.grid(row=i // 2, column=i % 2, sticky="ew", padx=4, pady=2)
            self.buttons.append(b)

        self.stop_btn = tk.Button(motor, text="STOP", bg="#d9534f", fg="white",
                                  font=("Arial", 14, "bold"), command=self.stop,
                                  state="disabled")
        self.stop_btn.pack(fill="x", **pad)

        # =================== OPTICAL SENSOR ===================
        sens = ttk.LabelFrame(left, text="Optical sensor (analog)")
        sens.pack(fill="x", pady=(8, 0))

        s1 = ttk.Frame(sens)
        s1.pack(fill="x", **pad)
        self.stream_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(s1, text="Plot sensor signal",
                        variable=self.stream_var,
                        command=self.toggle_stream).pack(side="left")
        self.sensor_label = ttk.Label(s1, text="Value: --")
        self.sensor_label.pack(side="right")

        s2 = ttk.Frame(sens)
        s2.pack(fill="x", **pad)
        ttk.Label(s2, text="Threshold (0-1023):").pack(side="left")
        self.thr_var = tk.StringVar(value="512")
        self.thr_entry = ttk.Entry(s2, textvariable=self.thr_var, width=7)
        self.thr_entry.pack(side="left", padx=6)
        thr_btn = ttk.Button(s2, text="Set", command=self.set_threshold,
                             state="disabled")
        thr_btn.pack(side="left")
        self.buttons.append(thr_btn)

        # =================== VOLTAGE ===================
        volt = ttk.LabelFrame(left, text="Voltage measurement")
        volt.pack(fill="x", pady=(8, 0))

        r1 = ttk.Frame(volt)
        r1.pack(fill="x", **pad)
        ttk.Label(r1, text="2182A address:").pack(side="left")
        self.gpib_v_var = tk.StringVar(value=DEFAULT_GPIB_2182A)
        ttk.Entry(r1, textvariable=self.gpib_v_var, width=20).pack(side="left", padx=6)

        r2 = ttk.Frame(volt)
        r2.pack(fill="x", **pad)
        self.use_6221_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(r2, text="Also read 6221 current",
                        variable=self.use_6221_var).pack(side="left")

        r3 = ttk.Frame(volt)
        r3.pack(fill="x", **pad)
        ttk.Label(r3, text="6221 address:").pack(side="left")
        self.gpib_c_var = tk.StringVar(value=DEFAULT_GPIB_6221)
        ttk.Entry(r3, textvariable=self.gpib_c_var, width=20).pack(side="left", padx=6)

        r4 = ttk.Frame(volt)
        r4.pack(fill="x", **pad)
        ttk.Label(r4, text="Interval (s):").pack(side="left")
        self.interval_var = tk.StringVar(value=str(DEFAULT_INTERVAL))
        ttk.Entry(r4, textvariable=self.interval_var, width=10).pack(side="left", padx=6)

        r5 = ttk.Frame(volt)
        r5.pack(fill="x", **pad)
        self.save_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(r5, text="Save data to CSV file",
                        variable=self.save_var).pack(side="left")

        self.meas_btn = ttk.Button(volt, text="Start measurement",
                                   command=self.toggle_measure)
        self.meas_btn.pack(fill="x", **pad)

        # =================== STATUS ===================
        ttk.Label(left, text="Status:").pack(anchor="w", pady=(8, 0))
        self.status = tk.Text(left, height=8, width=44, state="disabled")
        self.status.pack(fill="both", expand=True)

        # =================== PLOTS ===================
        # top: voltage (and 6221 current)
        self.fig = Figure(figsize=(7, 4), dpi=100, layout="tight")
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(side="top", fill="both", expand=True)
        self.setup_axes(False)

        # bottom: optical sensor signal
        self.sfig = Figure(figsize=(7, 2.6), dpi=100, layout="tight")
        self.scanvas = FigureCanvasTkAgg(self.sfig, master=right)
        self.scanvas.get_tk_widget().pack(side="top", fill="both", expand=True)
        self.setup_sensor_axes()

        if pyvisa is None:
            self.meas_btn.config(state="disabled")
            self.log("pyvisa is not installed: voltage measurement disabled.\n"
                     "Install it with: pip install pyvisa")

        self.refresh_ports()
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(100, self.poll_queue)

    # ---------- helpers ----------
    def log(self, text):
        self.status.config(state="normal")
        self.status.insert("end", text + "\n")
        self.status.see("end")
        self.status.config(state="disabled")

    def set_enabled(self, enabled):
        state = "normal" if enabled else "disabled"
        for b in self.buttons:
            b.config(state=state)
        self.stop_btn.config(state=state)

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    # ---------- motor connection ----------
    def toggle_connect(self):
        if self.ser:
            self.disconnect()
            return
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No port", "Select a COM port first.")
            return
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.2)
        except serial.SerialException as e:
            messagebox.showerror("Cannot open port",
                                 f"{e}\n\nIs the Serial Monitor still open?")
            self.ser = None
            return
        self.log(f"Opening {port}... (waiting 2 s for Arduino restart)")
        self.conn_btn.config(state="disabled")
        self.root.after(2000, self.finish_connect)

    def finish_connect(self):
        """The port opened, but that doesn't prove an Arduino is on it:
        ask it something and only say 'Connected' if it answers."""
        self.ser.reset_input_buffer()
        self.log("Checking that the Arduino answers...")
        threading.Thread(target=self.handshake, args=(self.ser,),
                         daemon=True).start()

    def handshake(self, ser):
        """Background check: send '?' and wait up to 2 s for BLOCKED/FREE.
        The reply looks like 'FREE value=123 threshold=512'."""
        ok = False
        reply = ""
        try:
            ser.write(b"?\n")
            deadline = time.time() + 2.0
            while time.time() < deadline:
                line = ser.readline().decode(errors="ignore").strip()
                if line.startswith(("BLOCKED", "FREE")):
                    ok = True
                    reply = line
                    break
        except (serial.SerialException, OSError):
            pass
        self.q.put(("handshake", ok, reply))

    def handshake_done(self, ok, reply=""):
        if not ok:
            port = self.port_var.get()
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.conn_btn.config(text="Connect", state="normal")
            self.log(f"No answer from the Arduino on {port}.")
            messagebox.showerror(
                "No Arduino found",
                f"The port {port} opened, but nothing answered.\n\n"
                "Wrong COM port, or the Arduino code is not running?")
            return
        self.running = True
        threading.Thread(target=self.reader, daemon=True).start()
        self.conn_btn.config(text="Disconnect", state="normal")
        self.set_enabled(True)
        self.log("Connected.")
        self.log("Arduino: " + reply)

        # start a fresh sensor plot, show the Arduino's threshold, start streaming
        self.clear_sensor_data()
        thr = self.find_threshold(reply)
        if thr is not None:
            self.show_threshold(thr, update_entry=True)
        if self.stream_var.get():
            self.send("W 1")

    def disconnect(self):
        self.running = False
        self.set_enabled(False)
        if self.ser:
            try:                           # best effort: stop the stream
                self.ser.write(b"W 0\n")
            except Exception:
                pass
        time.sleep(0.3)
        if self.ser:
            self.ser.close()
            self.ser = None
        self.conn_btn.config(text="Connect")
        self.log("Disconnected.")

    # ---------- motor serial ----------
    def reader(self):
        """Runs in the background so the window never freezes."""
        while self.running and self.ser:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except (serial.SerialException, OSError):
                self.q.put("Connection lost.")
                self.running = False
                return
            if not line:
                continue
            if line.startswith("A "):                 # streamed sensor value
                try:
                    self.q.put(("sensor", time.perf_counter(), int(line[2:])))
                except ValueError:
                    pass
                continue
            thr = self.find_threshold(line)           # "THRESHOLD 400" or "...threshold=400"
            if thr is not None:
                self.q.put(("threshold", thr, line.startswith("THRESHOLD")))
            self.q.put("Arduino: " + line)

    @staticmethod
    def find_threshold(line):
        m = re.search(r"threshold[= ](\d+)", line, re.IGNORECASE)
        return int(m.group(1)) if m else None

    def send(self, text):
        if not self.ser:
            return
        try:
            self.ser.write((text + "\n").encode())
            self.log("Sent: " + text)
        except serial.SerialException as e:
            self.log(f"Send error: {e}")

    # ---------- motor commands ----------
    def get_amount(self):
        try:
            n = int(float(self.amount_var.get()))
        except ValueError:
            messagebox.showwarning("Bad amount", "Enter a number of steps.")
            return None
        return abs(n)

    def home(self):
        self.send("H")

    def up(self):
        n = self.get_amount()
        if n:
            self.send(f"M {n}")

    def down(self):
        n = self.get_amount()
        if n:
            self.send(f"M -{n}")

    def ask_sensor(self):
        self.send("?")

    def stop(self):
        self.send("S")

    # ---------- optical sensor ----------
    def toggle_stream(self):
        self.send("W 1" if self.stream_var.get() else "W 0")

    def set_threshold(self):
        try:
            n = int(float(self.thr_var.get()))
        except ValueError:
            messagebox.showwarning("Bad threshold", "Enter a number from 0 to 1023.")
            return
        if not 0 <= n <= 1023:
            messagebox.showwarning("Bad threshold", "The value must be from 0 to 1023.")
            return
        self.send(f"T {n}")

    def show_threshold(self, n, update_entry):
        """Draw the threshold line; the Arduino's value is the one that counts."""
        self.sthr.set_ydata([n, n])
        self.sthr.set_visible(True)
        self.scanvas.draw_idle()
        # don't overwrite what the user is typing
        try:
            typing = self.root.focus_get() is self.thr_entry
        except KeyError:
            typing = False
        if update_entry or not typing:
            self.thr_var.set(str(n))

    def clear_sensor_data(self):
        self.sensor_t.clear()
        self.sensor_v.clear()
        self.sensor_t0 = None
        self.sline.set_data([], [])
        self.sax.set_xlim(0, SENSOR_WINDOW_S)
        self.sensor_label.config(text="Value: --")
        self.scanvas.draw_idle()

    def setup_sensor_axes(self):
        self.sax = self.sfig.add_subplot(111)
        self.sline, = self.sax.plot([], [], "-", linewidth=1.2)
        self.sthr = self.sax.axhline(0, linestyle="--", linewidth=1,
                                     color="tab:red", visible=False)
        self.sax.set_title("Optical sensor - analog value (dashed = threshold)")
        self.sax.set_xlabel("Time (s)")
        self.sax.set_ylabel("ADC value (0-1023)")
        self.sax.set_ylim(0, 1023)
        self.sax.set_xlim(0, SENSOR_WINDOW_S)
        self.sax.grid(True, alpha=0.3)
        self.scanvas.draw_idle()

    def update_sensor_plot(self):
        if not self.sensor_t:
            return
        t = self.sensor_t[-1]
        while self.sensor_t and self.sensor_t[0] < t - SENSOR_WINDOW_S:
            self.sensor_t.popleft()
            self.sensor_v.popleft()
        self.sline.set_data(list(self.sensor_t), list(self.sensor_v))
        self.sax.set_xlim(max(0, t - SENSOR_WINDOW_S), max(SENSOR_WINDOW_S, t))
        self.scanvas.draw_idle()

    # ---------- plot ----------
    def setup_axes(self, with_6221):
        """(Re)build the plot: one panel for voltage, two if the 6221 is read."""
        self.fig.clear()
        if with_6221:
            self.ax1 = self.fig.add_subplot(211)
            self.ax2 = self.fig.add_subplot(212, sharex=self.ax1)
        else:
            self.ax1 = self.fig.add_subplot(111)
            self.ax2 = None

        self.line1, = self.ax1.plot([], [], "o", markersize=3)
        self.ax1.set_title("Keithley 2182A - Live Voltage")
        self.ax1.set_ylabel("Voltage (µV)")
        self.ax1.grid(True, alpha=0.3)

        if with_6221:
            self.line2, = self.ax2.plot([], [], "o", markersize=3)
            self.ax2.set_title("Keithley 6221 - Output Current")
            self.ax2.set_xlabel("Time (s)")
            self.ax2.set_ylabel("Current (µA)")
            self.ax2.grid(True, alpha=0.3)
        else:
            self.line2 = None
            self.ax1.set_xlabel("Time (s)")

        self.canvas.draw_idle()

    def update_plot(self):
        self.line1.set_data(self.times, self.voltages)
        self.ax1.relim()
        self.ax1.autoscale_view()
        if self.line2 is not None:
            self.line2.set_data(self.times, self.currents)
            self.ax2.relim()
            self.ax2.autoscale_view()
        self.canvas.draw_idle()

    # ---------- voltage measurement ----------
    def toggle_measure(self):
        if self.measuring:
            self.meas_stop.set()
            self.meas_btn.config(text="Stopping...", state="disabled")
        else:
            self.start_measure()

    def start_measure(self):
        try:
            interval = float(self.interval_var.get())
            if interval <= 0:
                raise ValueError
        except ValueError:
            messagebox.showwarning("Bad interval",
                                   "Enter an interval in seconds (e.g. 0.5).")
            return

        addr_v = self.gpib_v_var.get().strip()
        addr_c = self.gpib_c_var.get().strip()
        with_6221 = self.use_6221_var.get()
        if not addr_v or (with_6221 and not addr_c):
            messagebox.showwarning("Missing address", "Enter the GPIB address.")
            return

        self.times, self.voltages, self.currents = [], [], []
        self.setup_axes(with_6221)

        if self.save_var.get():
            filename = datetime.now().strftime("measurement_%Y%m%d_%H%M%S.csv")
        else:
            filename = None              # None = do not save
        self.meas_stop.clear()
        self.measuring = True
        self.meas_btn.config(text="Stop measurement")
        if filename:
            self.log(f"Measurement started -> {filename}")
        else:
            self.log("Measurement started (not saving data).")
        threading.Thread(target=self.measure_worker,
                         args=(addr_v, addr_c, with_6221, interval, filename),
                         daemon=True).start()

    def measure_worker(self, addr_v, addr_c, with_6221, interval, filename):
        """Talks to the instruments in the background (GPIB can be slow)."""
        rm = nano = source = csv_file = None
        try:
            rm = pyvisa.ResourceManager()

            nano = rm.open_resource(addr_v)
            nano.timeout = 10000
            nano.read_termination = "\n"
            nano.write_termination = "\n"
            self.mq.put(("log", "2182A: " + nano.query("*IDN?").strip()))

            if with_6221:
                source = rm.open_resource(addr_c)
                source.timeout = 5000
                source.read_termination = "\n"
                source.write_termination = "\n"
                self.mq.put(("log", "6221: " + source.query("*IDN?").strip()))

            nano.write(':SENS:FUNC "VOLT"')

            writer = None
            if filename:
                csv_file = open(filename, "w", newline="")
                writer = csv.writer(csv_file)
                headers = ["Time (s)", "Voltage (V)", "Voltage (uV)"]
                if with_6221:
                    headers += ["Current (A)", "Current (uA)"]
                writer.writerow(headers)

            start = time.perf_counter()
            while not self.meas_stop.is_set():
                voltage = float(nano.query(":READ?"))
                elapsed = time.perf_counter() - start
                row = [elapsed, voltage, voltage * 1e6]

                current_ua = None
                if source is not None:
                    current = float(source.query(":SOUR:CURR?"))
                    current_ua = current * 1e6
                    row += [current, current_ua]

                if writer:
                    writer.writerow(row)
                    csv_file.flush()
                self.mq.put(("data", elapsed, voltage * 1e6, current_ua))

                # wait for the next reading, but wake up immediately on stop
                self.meas_stop.wait(interval)

        except Exception as e:
            self.mq.put(("log", f"Measurement error: {e}"))
        finally:
            for obj in (csv_file, nano, source, rm):
                try:
                    if obj is not None:
                        obj.close()
                except Exception:
                    pass
            self.mq.put(("done", filename if csv_file else None))

    # ---------- GUI update loop ----------
    def poll_queue(self):
        # motor / sensor messages
        sensor_updated = False
        try:
            while True:
                item = self.q.get_nowait()
                if not isinstance(item, tuple):
                    self.log(item)
                    continue
                kind = item[0]
                if kind == "handshake":              # ("handshake", ok, reply)
                    self.handshake_done(item[1], item[2])
                elif kind == "sensor":               # ("sensor", time, value)
                    _, ts, value = item
                    if self.sensor_t0 is None:
                        self.sensor_t0 = ts
                    self.sensor_t.append(ts - self.sensor_t0)
                    self.sensor_v.append(value)
                    self.sensor_label.config(text=f"Value: {value}")
                    sensor_updated = True
                elif kind == "threshold":            # ("threshold", n, confirmed)
                    self.show_threshold(item[1], update_entry=item[2])
        except queue.Empty:
            pass
        if sensor_updated:
            self.update_sensor_plot()

        # measurement messages
        updated = False
        try:
            while True:
                msg = self.mq.get_nowait()
                kind = msg[0]
                if kind == "data":
                    _, t, v, c = msg
                    self.times.append(t)
                    self.voltages.append(v)
                    if c is not None:
                        self.currents.append(c)
                    updated = True
                elif kind == "log":
                    self.log(msg[1])
                elif kind == "done":
                    self.measuring = False
                    self.meas_btn.config(text="Start measurement", state="normal")
                    if msg[1]:
                        self.log(f"Measurement stopped. Data saved to: {msg[1]}")
                    else:
                        self.log("Measurement stopped. Data was not saved.")
        except queue.Empty:
            pass

        if updated:
            self.update_plot()

        self.root.after(100, self.poll_queue)

    def on_close(self):
        self.running = False
        self.meas_stop.set()
        if self.ser:
            try:
                self.ser.write(b"W 0\n")
                self.ser.close()
            except Exception:
                pass
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
