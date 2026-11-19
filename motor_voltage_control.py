"""
Motor + voltage control interface (single file).

Install once:   pip install pyserial pyvisa matplotlib
Run:            python motor_voltage_control.py

Close the Arduino Serial Monitor first (only one program can use the port).
Needs the matching Arduino sketch (motor_control.ino).

Layout
  left column   : motor jog buttons, optical sensor, 6221 current source
                  (connect / set current / output ON-OFF), status log
  middle column : sweep / stop-and-go program, voltage measurement
  right side    : top plot = voltage (optionally 6221 current),
                  bottom plot = optical sensor signal

Positions
  Positions are motor steps BELOW the home sensor (0 = sensor, growing
  downwards). Press Home first; after that the app knows where the motor is.
  There is no encoder: if the motor ever loses steps, the position drifts
  until the motor reaches the sensor again (which resets it to 0).

Sweep / stop-and-go
  Continuous sweep : go to Start, then move to End at the sweep speed.
  Stop and go      : go to Start, then move toward End in jumps of "Stop every"
                     steps, waiting "Wait at each stop" seconds at every stop
                     (the first stop is Start, the last one is always End).
  Fields that are in steps (or steps/s) show the matching length in mm next
  to them, using UM_PER_STEP below (check that value for your actuator and
  driver microstepping!).

Calibrate home
  The button in the sensor panel lets the motor find the "free" level and the
  "blocked" level of the optical sensor, puts the threshold exactly halfway
  between them, and homes again from below. Position 0 is then the middle of
  the sensor edge. Needs the matching Arduino sketch (command "C").

  The voltage measurement is independent: start it first, then run the
  program. Each CSV row also gets the motor position and the stop number
  (empty while the motor is moving). Keep "Plot sensor signal" ticked if you
  want an accurate live position while the motor moves.
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
MIN_SPEED = 20               # steps per second (same limits as the Arduino)
MAX_SPEED = 3000
STROKE_MM = 19.1             # total travel of the actuator (datasheet)
MAX_CURRENT_UA = 105_000     # 6221 limit: 105 mA
UM_PER_STEP = 0.625          # micrometres per step = 5 um full step / 8 (1/8 microstepping;
                             # measured: 8000 steps = 5 mm)
CAL_MAX_STEPS = 10_000       # must match CAL_MAX_STEPS in the Arduino sketch
MAX_POS = int(STROKE_MM * 1000 / UM_PER_STEP)   # whole stroke, in steps (38200)
CAL_TIMEOUT_S = 180          # give up waiting for a calibration after this long
# --------------------------------------------------------


class App:
    def __init__(self, root):
        self.root = root
        root.title("Motor + voltage control")
        root.minsize(1100, 650)

        # motor / serial state
        self.ser = None
        self.q = queue.Queue()
        self.running = False

        # motion state
        self.position = None            # steps below the sensor (None = unknown)
        self.homed = False
        self.move_q = queue.Queue()     # "DONE pos=.." style replies to moves
        self.prog_abort = threading.Event()
        self.prog_running = False
        self.stop_idx = None            # number of the current stop (None = moving)

        # measurement state
        self.mq = queue.Queue()
        self.measuring = False
        self.meas_stop = threading.Event()
        self.times, self.voltages, self.currents = [], [], []

        # 6221 current source state (one shared GPIB session)
        self.rm = None
        self.source = None                  # pyvisa resource (None = not connected)
        self.source_lock = threading.Lock() # only one GPIB conversation at a time
        self.out_on = False
        self.src_buttons = []

        # optical sensor state
        self.sensor_t = deque()
        self.sensor_v = deque()
        self.sensor_t0 = None

        pad = {"padx": 8, "pady": 6}

        colA = ttk.Frame(root)
        colA.pack(side="left", fill="y", padx=(8, 4), pady=6)
        colB = ttk.Frame(root)
        colB.pack(side="left", fill="y", padx=4, pady=6)
        right = ttk.Frame(root)
        right.pack(side="right", fill="both", expand=True, padx=(4, 8), pady=6)

        # =================== MOTOR ===================
        motor = ttk.LabelFrame(colA, text="Motor")
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
        amount_mm = ttk.Label(mid, text="")
        amount_mm.pack(side="left")
        self.link_units(self.amount_var, amount_mm)

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
        sens = ttk.LabelFrame(colA, text="Optical sensor (analog)")
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
        self.thr_var = tk.StringVar(value="500")
        self.thr_entry = ttk.Entry(s2, textvariable=self.thr_var, width=7)
        self.thr_entry.pack(side="left", padx=6)
        thr_btn = ttk.Button(s2, text="Set", command=self.set_threshold,
                             state="disabled")
        thr_btn.pack(side="left")
        self.buttons.append(thr_btn)

        s3 = ttk.Frame(sens)
        s3.pack(fill="x", **pad)
        cal_btn = ttk.Button(s3, text="Calibrate home (find midpoint)",
                             command=self.calibrate_home, state="disabled")
        cal_btn.pack(side="left")
        self.buttons.append(cal_btn)

        # =================== 6221 CURRENT SOURCE ===================
        cur = ttk.LabelFrame(colA, text="Current source (Keithley 6221)")
        cur.pack(fill="x", pady=(8, 0))

        c1 = ttk.Frame(cur)
        c1.pack(fill="x", **pad)
        ttk.Label(c1, text="Address:").pack(side="left")
        self.gpib_c_var = tk.StringVar(value=DEFAULT_GPIB_6221)
        ttk.Entry(c1, textvariable=self.gpib_c_var, width=18).pack(side="left", padx=6)
        self.src_conn_btn = ttk.Button(c1, text="Connect",
                                       command=self.toggle_source_connect)
        self.src_conn_btn.pack(side="left")

        c2 = ttk.Frame(cur)
        c2.pack(fill="x", **pad)
        ttk.Label(c2, text="Current (uA):").pack(side="left")
        self.cur_var = tk.StringVar(value="0")
        cur_entry = ttk.Entry(c2, textvariable=self.cur_var, width=10)
        cur_entry.pack(side="left", padx=6)
        cur_entry.bind("<Return>", lambda e: self.set_source_current())
        self.cur_set_btn = ttk.Button(c2, text="Set", state="disabled",
                                      command=self.set_source_current)
        self.cur_set_btn.pack(side="left")

        c3 = ttk.Frame(cur)
        c3.pack(fill="x", **pad)
        self.out_label = ttk.Label(c3, text="Not connected")
        self.out_label.pack(side="left")
        self.out_btn = ttk.Button(c3, text="Turn ON", width=10, state="disabled",
                                  command=self.toggle_source_output)
        self.out_btn.pack(side="right")
        self.src_buttons = [self.cur_set_btn, self.out_btn]

        # =================== STATUS ===================
        ttk.Label(colA, text="Status:").pack(anchor="w", pady=(8, 0))
        self.status = tk.Text(colA, height=8, width=44, state="disabled")
        self.status.pack(fill="both", expand=True)

        # =================== SWEEP / STOPS ===================
        sw = ttk.LabelFrame(colB, text="Sweep / stops (positions in steps)")
        sw.pack(fill="x")

        p0 = ttk.Frame(sw)
        p0.pack(fill="x", **pad)
        self.pos_label = ttk.Label(p0, text="Position: --")
        self.pos_label.pack(side="left")

        self.start_var = tk.StringVar(value="0")
        self.end_var = tk.StringVar(value="2000")
        self.pspeed_var = tk.StringVar(value="600")
        self.sspeed_var = tk.StringVar(value="300")
        self.step_var = tk.StringVar(value="200")
        self.dwell_var = tk.StringVar(value="5")
        self.mode_var = tk.StringVar(value="sweep")

        g = ttk.Frame(sw)
        g.pack(fill="x", **pad)
        g.columnconfigure(1, weight=1)

        def add_row(r, text, var, button=None, unit=None):
            """unit: None = no conversion, "" = steps -> mm, "/s" = steps/s -> mm/s"""
            ttk.Label(g, text=text).grid(row=r, column=0, sticky="w", pady=2)
            ttk.Entry(g, textvariable=var, width=8).grid(
                row=r, column=1, sticky="w", padx=6, pady=2)
            if unit is not None:
                lab = ttk.Label(g, text="", width=16)
                lab.grid(row=r, column=2, sticky="w", pady=2)
                self.link_units(var, lab, unit)
            if button is not None:
                button.grid(row=r, column=3, sticky="e", pady=2)

        use_start = ttk.Button(g, text="Use current", state="disabled",
                               command=lambda: self.use_current(self.start_var))
        use_end = ttk.Button(g, text="Use current", state="disabled",
                             command=lambda: self.use_current(self.end_var))
        self.buttons += [use_start, use_end]

        add_row(0, "Start position (steps):", self.start_var, use_start, "")
        add_row(1, "End position (steps):", self.end_var, use_end, "")
        add_row(2, "Jog/positioning speed (steps/s):", self.pspeed_var, unit="/s")
        add_row(3, "Sweep speed (steps/s):", self.sspeed_var, unit="/s")
        add_row(4, "Stop every (steps):", self.step_var, unit="")
        add_row(5, "Wait at each stop (s):", self.dwell_var)

        self.span_label = ttk.Label(sw, text="")
        self.span_label.pack(anchor="w", padx=8)
        self.start_var.trace_add("write", self.update_span)
        self.end_var.trace_add("write", self.update_span)
        self.update_span()

        m = ttk.Frame(sw)
        m.pack(fill="x", **pad)
        ttk.Radiobutton(m, text="Continuous sweep (start to end)",
                        variable=self.mode_var, value="sweep").pack(anchor="w")
        ttk.Radiobutton(m, text="Stop and go (pause at each stop)",
                        variable=self.mode_var, value="stops").pack(anchor="w")

        bb = ttk.Frame(sw)
        bb.pack(fill="x", **pad)
        bb.columnconfigure(0, weight=1)
        bb.columnconfigure(1, weight=1)
        goto_btn = ttk.Button(bb, text="Go to start", command=self.go_to_start,
                              state="disabled")
        goto_btn.grid(row=0, column=0, sticky="ew", padx=4)
        run_btn = ttk.Button(bb, text="Run", command=self.run_program,
                             state="disabled")
        run_btn.grid(row=0, column=1, sticky="ew", padx=4)
        self.buttons += [goto_btn, run_btn]

        self.prog_label = ttk.Label(sw, text="Idle", wraplength=290, justify="left")
        self.prog_label.pack(anchor="w", padx=8, pady=(0, 6))

        # =================== VOLTAGE ===================
        volt = ttk.LabelFrame(colB, text="Voltage measurement")
        volt.pack(fill="x", pady=(8, 0))

        r1 = ttk.Frame(volt)
        r1.pack(fill="x", **pad)
        ttk.Label(r1, text="2182A address:").pack(side="left")
        self.gpib_v_var = tk.StringVar(value=DEFAULT_GPIB_2182A)
        ttk.Entry(r1, textvariable=self.gpib_v_var, width=20).pack(side="left", padx=6)

        r2 = ttk.Frame(volt)
        r2.pack(fill="x", **pad)
        self.use_6221_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(r2, text="Also read 6221 current (connect it first)",
                        variable=self.use_6221_var).pack(side="left")

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
            self.src_conn_btn.config(state="disabled")
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

    @staticmethod
    def steps_to_mm(n):
        return n * UM_PER_STEP / 1000.0

    def link_units(self, var, label, suffix=""):
        """Keeps 'label' showing the length (mm, or mm/s) of the steps in 'var'."""
        def update(*_):
            try:
                n = float(var.get())
            except ValueError:
                label.config(text="= ?")
                return
            label.config(text=f"= {self.steps_to_mm(n):.3f} mm{suffix}")
        var.trace_add("write", update)
        update()

    def update_span(self, *_):
        try:
            d = abs(float(self.end_var.get()) - float(self.start_var.get()))
        except ValueError:
            self.span_label.config(text="Sweep length: ?")
            return
        self.span_label.config(
            text=f"Sweep length: {d:g} steps = {self.steps_to_mm(d):.3f} mm")

    def update_pos_label(self):
        if self.position is None:
            text = "Position: --"
        else:
            state = "homed" if self.homed else "not homed"
            text = (f"Position: {self.position} steps = "
                    f"{self.steps_to_mm(self.position):.3f} mm ({state})")
        self.pos_label.config(text=text)

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
        The reply looks like 'FREE value=1 threshold=500 pos=0 homed=0'."""
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

        # position / homed state as reported by the Arduino
        m = re.search(r"pos=(-?\d+)", reply)
        self.position = int(m.group(1)) if m else None
        self.homed = "homed=1" in reply
        self.update_pos_label()

        # start a fresh sensor plot, show the Arduino's threshold, start streaming
        self.clear_sensor_data()
        thr = self.find_threshold(reply)
        if thr is not None:
            self.show_threshold(thr, update_entry=True)
        if self.stream_var.get():
            self.send("W 1")

    def disconnect(self):
        self.prog_abort.set()
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
        self.position = None
        self.homed = False
        self.update_pos_label()
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

            if line.startswith("A "):                 # "A <value> <position>"
                parts = line.split()
                try:
                    value = int(parts[1])
                    pos = int(parts[2]) if len(parts) > 2 else None
                except (ValueError, IndexError):
                    continue
                if pos is not None:
                    self.position = pos
                self.q.put(("sensor", time.perf_counter(), value, pos))
                continue

            if line.startswith("SPEED"):              # confirmation of "V", not interesting
                continue

            mc = re.search(r"CAL free=(\d+) blocked=(\d+)", line)
            if mc:
                self.q.put(("cal_levels", int(mc.group(1)), int(mc.group(2))))

            m = re.search(r"pos=(-?\d+)", line)
            if m:
                self.position = int(m.group(1))
                self.q.put(("pos",))
            if line.startswith(("HOMED", "SENSOR")) or "homed=1" in line:
                self.homed = True
            if line.startswith(("DONE", "SENSOR", "HOMED", "STOPPED", "ERR")):
                self.move_q.put(line)                 # a move has finished

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

    def write_line(self, text):
        """Like send(), but safe to call from a background thread."""
        try:
            self.ser.write((text + "\n").encode())
            self.q.put("Sent: " + text)
            return True
        except (serial.SerialException, OSError, AttributeError):
            return False

    def send_jog_speed(self):
        """Jog and Home use the 'Jog/positioning speed' field."""
        try:
            sps = int(float(self.pspeed_var.get()))
        except ValueError:
            return
        sps = max(MIN_SPEED, min(MAX_SPEED, sps))
        try:
            self.ser.write(f"V {sps}\n".encode())
        except (serial.SerialException, OSError, AttributeError):
            pass

    # ---------- motor commands ----------
    def get_amount(self):
        try:
            n = int(float(self.amount_var.get()))
        except ValueError:
            messagebox.showwarning("Bad amount", "Enter a number of steps.")
            return None
        return abs(n)

    def home(self):
        self.send_jog_speed()
        self.send("H")

    def up(self):
        n = self.get_amount()
        if n:
            self.send_jog_speed()
            self.send(f"M {n}")

    def down(self):
        n = self.get_amount()
        if n:
            self.send_jog_speed()
            self.send(f"M -{n}")

    def ask_sensor(self):
        self.send("?")

    def stop(self):
        self.prog_abort.set()          # also ends a running sweep / stop program
        self.send("S")

    # ---------- sweep / stops ----------
    def read_number(self, var, name, lo, hi, as_int=True):
        try:
            x = float(var.get())
        except ValueError:
            messagebox.showwarning("Bad value", f"{name}: enter a number.")
            return None
        if not lo <= x <= hi:
            messagebox.showwarning("Bad value", f"{name} must be between {lo} and {hi}.")
            return None
        return int(x) if as_int else x

    def check_ready(self):
        if not self.ser or not self.running:
            messagebox.showwarning("Not connected", "Connect to the Arduino first.")
            return False
        if self.prog_running:
            return False
        if not self.homed:
            messagebox.showwarning(
                "Home first",
                "Press Home first, so positions are measured from the sensor.")
            return False
        return True

    def use_current(self, var):
        if not self.homed or self.position is None:
            messagebox.showwarning(
                "Home first",
                "Press Home first, so positions are measured from the sensor.")
            return
        var.set(str(self.position))

    def go_to_start(self):
        if not self.check_ready():
            return
        start = self.read_number(self.start_var, "Start position", 0, MAX_POS)
        sp = self.read_number(self.pspeed_var, "Positioning speed", MIN_SPEED, MAX_SPEED)
        if start is None or sp is None:
            return
        self.start_program([("move", start, sp)], "Go to start")

    def run_program(self):
        if not self.check_ready():
            return
        start = self.read_number(self.start_var, "Start position", 0, MAX_POS)
        end = self.read_number(self.end_var, "End position", 0, MAX_POS)
        sp = self.read_number(self.pspeed_var, "Positioning speed", MIN_SPEED, MAX_SPEED)
        sw = self.read_number(self.sspeed_var, "Sweep speed", MIN_SPEED, MAX_SPEED)
        if None in (start, end, sp, sw):
            return
        if start == end:
            messagebox.showwarning("Bad positions", "Start and end are the same.")
            return

        if self.mode_var.get() == "sweep":
            plan = [("move", start, sp), ("move", end, sw)]
            label = "Sweep"
        else:
            step = self.read_number(self.step_var, "Stop every", 1, MAX_POS)
            dwell = self.read_number(self.dwell_var, "Wait at each stop", 0, 86400,
                                     as_int=False)
            if step is None or dwell is None:
                return
            direction = 1 if end > start else -1
            stops = list(range(start, end, direction * step)) + [end]
            n = len(stops)
            plan = [("move", start, sp)]
            for k, p in enumerate(stops, 1):
                if k > 1:
                    plan.append(("move", p, sw))
                plan.append(("wait", dwell, k, n, p))
            label = f"Stop and go ({n} stops)"
        self.start_program(plan, label)

    def start_program(self, plan, label):
        self.prog_abort.clear()
        self.prog_running = True
        for b in self.buttons:               # only STOP stays active
            b.config(state="disabled")
        self.log(f"{label} started.")
        threading.Thread(target=self.program_worker, args=(plan,),
                         daemon=True).start()

    def program_worker(self, plan):
        """Runs the plan step by step in the background."""
        msg = "Finished."
        try:
            for step in plan:
                if self.prog_abort.is_set():
                    msg = "Stopped."
                    break
                if step[0] == "move":
                    _, target, sps = step
                    self.stop_idx = None
                    self.q.put(("progress", f"Moving to {target} at {sps} steps/s..."))
                    result = self.move_to(target, sps)
                    if result != "ok":
                        msg = result
                        break
                elif step[0] == "calibrate":
                    self.stop_idx = None
                    self.q.put(("progress",
                                "Calibrating: the motor moves down, then up past the "
                                "sensor edge, then homes again..."))
                    result = self.calibrate()
                    if result != "ok":
                        msg = result
                        break
                else:
                    _, secs, k, n, p = step
                    self.stop_idx = k
                    self.q.put(("progress",
                                f"Stop {k}/{n} at position {p}: waiting {secs:g} s"))
                    if self.prog_abort.wait(secs):
                        msg = "Stopped."
                        break
                    self.stop_idx = None
        except Exception as e:
            msg = f"Program error: {e}"
        finally:
            self.stop_idx = None
            self.q.put(("program_done", msg))

    def move_to(self, target, sps):
        """Moves to an absolute position and waits until the Arduino is done.
        Returns 'ok' or a text describing what went wrong."""
        while True:                          # forget old replies
            try:
                self.move_q.get_nowait()
            except queue.Empty:
                break

        if not (self.write_line(f"V {sps}") and self.write_line(f"G {target}")):
            return "Send error."

        distance = abs(target - (self.position or 0))
        deadline = time.time() + distance / sps + 15
        while time.time() < deadline:
            if self.prog_abort.is_set():
                return "Stopped."
            try:
                line = self.move_q.get(timeout=0.2)
            except queue.Empty:
                continue
            if line.startswith("DONE"):
                return "ok"
            if line.startswith("SENSOR"):
                if target == 0:
                    return "ok"
                return ("Stopped: the sensor was reached before the target. "
                        "Steps were probably lost - press Home again.")
            if line.startswith("STOPPED"):
                return "Stopped."
            if line.startswith("ERR"):
                return f"Arduino error: {line}"
        return "No answer from the Arduino (timeout)."

    # ---------- calibration ----------
    def calibrate_home(self):
        if not self.ser or not self.running or self.prog_running:
            return
        if not messagebox.askokcancel(
                "Calibrate home",
                "The motor will move down until the sensor is free, then up past "
                "the edge until the beam is fully blocked, then back down and "
                "home again.\n\n"
                f"Each move is limited to {CAL_MAX_STEPS} steps "
                f"({self.steps_to_mm(CAL_MAX_STEPS):g} mm). "
                "Make sure there is free travel in both directions.\n\n"
                "Continue?"):
            return
        self.start_program([("calibrate",)], "Calibration")

    def calibrate(self):
        """Sends 'C' and waits until the Arduino has finished (it ends with HOMED).
        Returns 'ok' or a text describing what went wrong."""
        while True:                          # forget old replies
            try:
                self.move_q.get_nowait()
            except queue.Empty:
                break
        if not self.write_line("C"):
            return "Send error."
        deadline = time.time() + CAL_TIMEOUT_S
        while time.time() < deadline:
            if self.prog_abort.is_set():
                return "Stopped."
            try:
                line = self.move_q.get(timeout=0.2)
            except queue.Empty:
                continue
            if line.startswith("HOMED"):
                return "Calibration finished: home is now the middle of the edge."
            if line.startswith("STOPPED"):
                return "Stopped."
            if line.startswith("ERR"):
                return f"Calibration failed: {line}"
        return "No answer from the Arduino (calibration timeout)."

    def show_levels(self, free, blocked):
        self.sfree.set_ydata([free, free])
        self.sfree.set_visible(True)
        self.sblock.set_ydata([blocked, blocked])
        self.sblock.set_visible(True)
        self.scanvas.draw_idle()

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
        self.sfree = self.sax.axhline(0, linestyle=":", linewidth=1,
                                      color="tab:green", visible=False)
        self.sblock = self.sax.axhline(0, linestyle=":", linewidth=1,
                                       color="tab:orange", visible=False)
        self.sax.set_title("Optical sensor - analog value "
                           "(red dashed = threshold, dotted = free / blocked level)")
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

    # ---------- voltage plot ----------
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

    # ---------- 6221 current source ----------
    @staticmethod
    def parse_on(text):
        return text.strip().upper() in ("1", "ON")

    def toggle_source_connect(self):
        if self.source is not None:
            if self.measuring:
                messagebox.showwarning("Measurement running",
                                       "Stop the measurement before disconnecting the 6221.")
                return
            self.source_disconnect()
            return
        addr = self.gpib_c_var.get().strip()
        if not addr:
            messagebox.showwarning("Missing address", "Enter the 6221 GPIB address.")
            return
        self.src_conn_btn.config(state="disabled")
        self.log(f"Connecting to the 6221 at {addr}...")
        threading.Thread(target=self.source_connect_worker, args=(addr,),
                         daemon=True).start()

    def source_connect_worker(self, addr):
        """Opens the 6221 in the background (GPIB can be slow)."""
        try:
            if self.rm is None:
                self.rm = pyvisa.ResourceManager()
            src = self.rm.open_resource(addr)
            src.timeout = 5000
            src.read_termination = "\n"
            src.write_termination = "\n"
            idn = src.query("*IDN?").strip()
            amps = float(src.query(":SOUR:CURR?"))
            on = self.parse_on(src.query(":OUTP?"))
            self.source = src
            self.q.put(("src_connected", idn, amps, on))
        except Exception as e:
            self.q.put(("src_failed", f"6221 connection error: {e}"))

    def source_disconnect(self):
        """Closes the connection. The output is left as it is (ON stays ON)."""
        with self.source_lock:
            try:
                self.source.close()
            except Exception:
                pass
            self.source = None
        self.out_on = False
        for b in self.src_buttons:
            b.config(state="disabled")
        self.src_conn_btn.config(text="Connect", state="normal")
        self.out_label.config(text="Not connected", foreground="")
        self.log("6221 disconnected.")

    def set_source_current(self):
        if self.source is None:
            return
        try:
            ua = float(self.cur_var.get())
        except ValueError:
            messagebox.showwarning("Bad current", "Enter the current in uA (e.g. 100).")
            return
        if abs(ua) > MAX_CURRENT_UA:
            messagebox.showwarning(
                "Bad current",
                f"The 6221 can give at most +/-{MAX_CURRENT_UA:g} uA (105 mA).")
            return
        self.log(f"6221: setting current to {ua:g} uA")
        self.source_command(("current", ua * 1e-6))

    def toggle_source_output(self):
        if self.source is None:
            return
        want_on = not self.out_on
        self.log("6221: output " + ("ON" if want_on else "OFF"))
        self.source_command(("output", want_on))

    def source_command(self, action):
        for b in self.src_buttons:               # no double clicks while busy
            b.config(state="disabled")
        threading.Thread(target=self.source_worker, args=(action,),
                         daemon=True).start()

    def source_worker(self, action):
        """Sends one command, then reads back what the 6221 really does."""
        try:
            with self.source_lock:
                src = self.source
                if action[0] == "current":
                    src.write(f":SOUR:CURR {action[1]:.9g}")
                else:
                    src.write(":OUTP ON" if action[1] else ":OUTP OFF")
                err = src.query(":SYST:ERR?").strip()
                amps = float(src.query(":SOUR:CURR?"))
                on = self.parse_on(src.query(":OUTP?"))
            if not err.startswith("0"):
                self.q.put(f"6221 error: {err}")
            self.q.put(("src_state", amps, on))
        except Exception as e:
            self.q.put(f"6221 error: {e}")
            self.q.put(("src_state", None, None))

    def show_source_state(self, amps, on):
        """amps/on are what the 6221 reported (None = unknown, keep as is)."""
        if on is not None:
            self.out_on = on
        if amps is not None:
            self.cur_var.set(f"{amps * 1e6:g}")
        if self.source is None:
            return
        shown = self.cur_var.get().strip() or "?"
        if self.out_on:
            self.out_label.config(text=f"Output ON  ({shown} uA)", foreground="#c9302c")
            self.out_btn.config(text="Turn OFF")
        else:
            self.out_label.config(text=f"Output OFF ({shown} uA set)", foreground="")
            self.out_btn.config(text="Turn ON")
        for b in self.src_buttons:
            b.config(state="normal")

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
        with_6221 = self.use_6221_var.get()
        if not addr_v:
            messagebox.showwarning("Missing address", "Enter the 2182A GPIB address.")
            return
        if with_6221 and self.source is None:
            messagebox.showwarning(
                "6221 not connected",
                "Connect the 6221 first (Current source panel), "
                "or untick 'Also read 6221 current'.")
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
                         args=(addr_v, with_6221, interval, filename),
                         daemon=True).start()

    def measure_worker(self, addr_v, with_6221, interval, filename):
        """Talks to the instruments in the background (GPIB can be slow)."""
        rm = nano = csv_file = None
        source = self.source if with_6221 else None   # shared session, not closed here
        try:
            if self.rm is None:
                self.rm = pyvisa.ResourceManager()
            rm = self.rm

            nano = rm.open_resource(addr_v)
            nano.timeout = 10000
            nano.read_termination = "\n"
            nano.write_termination = "\n"
            self.mq.put(("log", "2182A: " + nano.query("*IDN?").strip()))

            nano.write(':SENS:FUNC "VOLT"')

            writer = None
            if filename:
                csv_file = open(filename, "w", newline="")
                writer = csv.writer(csv_file)
                headers = ["Time (s)", "Voltage (V)", "Voltage (uV)"]
                if with_6221:
                    headers += ["Current (A)", "Current (uA)"]
                headers += ["Position (steps)", "Position (mm)", "Stop #"]
                writer.writerow(headers)

            start = time.perf_counter()
            while not self.meas_stop.is_set():
                voltage = float(nano.query(":READ?"))
                elapsed = time.perf_counter() - start
                row = [elapsed, voltage, voltage * 1e6]

                current_ua = None
                if source is not None:
                    with self.source_lock:
                        on = self.parse_on(source.query(":OUTP?"))
                        # output OFF = no current flows, whatever the setpoint is
                        current = float(source.query(":SOUR:CURR?")) if on else 0.0
                    current_ua = current * 1e6
                    row += [current, current_ua]

                # motor position and stop number at the moment of the reading
                position = self.position
                stop_idx = self.stop_idx
                row += [position if position is not None else "",
                        self.steps_to_mm(position) if position is not None else "",
                        stop_idx if stop_idx is not None else ""]

                if writer:
                    writer.writerow(row)
                    csv_file.flush()
                self.mq.put(("data", elapsed, voltage * 1e6, current_ua))

                # wait for the next reading, but wake up immediately on stop
                self.meas_stop.wait(interval)

        except Exception as e:
            self.mq.put(("log", f"Measurement error: {e}"))
        finally:
            for obj in (csv_file, nano):
                try:
                    if obj is not None:
                        obj.close()
                except Exception:
                    pass
            self.mq.put(("done", filename if csv_file else None))

    # ---------- GUI update loop ----------
    def poll_queue(self):
        # motor / sensor / program messages
        sensor_updated = False
        pos_changed = False
        try:
            while True:
                item = self.q.get_nowait()
                if not isinstance(item, tuple):
                    self.log(item)
                    continue
                kind = item[0]
                if kind == "handshake":              # ("handshake", ok, reply)
                    self.handshake_done(item[1], item[2])
                elif kind == "sensor":               # ("sensor", time, value, pos)
                    _, ts, value, pos = item
                    if self.sensor_t0 is None:
                        self.sensor_t0 = ts
                    self.sensor_t.append(ts - self.sensor_t0)
                    self.sensor_v.append(value)
                    self.sensor_label.config(text=f"Value: {value}")
                    sensor_updated = True
                    if pos is not None:
                        pos_changed = True
                elif kind == "pos":
                    pos_changed = True
                elif kind == "threshold":            # ("threshold", n, confirmed)
                    self.show_threshold(item[1], update_entry=item[2])
                elif kind == "cal_levels":           # ("cal_levels", free, blocked)
                    self.show_levels(item[1], item[2])
                elif kind == "src_connected":        # ("src_connected", idn, amps, on)
                    self.log("6221: " + item[1])
                    self.src_conn_btn.config(text="Disconnect", state="normal")
                    self.show_source_state(item[2], item[3])
                elif kind == "src_failed":
                    self.log(item[1])
                    self.src_conn_btn.config(text="Connect", state="normal")
                elif kind == "src_state":            # ("src_state", amps, on)
                    self.show_source_state(item[1], item[2])
                elif kind == "progress":
                    self.prog_label.config(text=item[1])
                elif kind == "program_done":
                    self.prog_running = False
                    self.prog_label.config(text=item[1])
                    self.log(item[1])
                    pos_changed = True
                    if self.ser and self.running:
                        self.set_enabled(True)
        except queue.Empty:
            pass
        if sensor_updated:
            self.update_sensor_plot()
        if pos_changed:
            self.update_pos_label()

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
        self.prog_abort.set()
        self.meas_stop.set()
        if self.source is not None:        # close the session; output stays as it is
            try:
                self.source.close()
            except Exception:
                pass
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
