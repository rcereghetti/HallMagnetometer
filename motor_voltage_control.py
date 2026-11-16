"""
Motor + voltage control interface (single file).

Install once:   pip install pyserial pyvisa matplotlib
Run:            python motor_voltage_control.py

Close the Arduino Serial Monitor first (only one program can use the port).

Left side : motor controls, voltage measurement settings, status log.
Right side: live plot (voltage from the Keithley 2182A, optionally the
            output current of the Keithley 6221 in a second plot).
Each measurement is also saved to a CSV file (measurement_YYYYMMDD_HHMMSS.csv)
in the folder you run the script from.

The motor and the voltage measurement are independent: you can use either
one alone, or both at the same time.
"""

import csv
import queue
import threading
import time
import tkinter as tk
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

        self.meas_btn = ttk.Button(volt, text="Start measurement",
                                   command=self.toggle_measure)
        self.meas_btn.pack(fill="x", **pad)

        # =================== STATUS ===================
        ttk.Label(left, text="Status:").pack(anchor="w", pady=(8, 0))
        self.status = tk.Text(left, height=10, width=44, state="disabled")
        self.status.pack(fill="both", expand=True)

        # =================== PLOT ===================
        self.fig = Figure(figsize=(7, 5), dpi=100, layout="tight")
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.setup_axes(False)

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
        self.ser.reset_input_buffer()
        self.running = True
        threading.Thread(target=self.reader, daemon=True).start()
        self.conn_btn.config(text="Disconnect", state="normal")
        self.set_enabled(True)
        self.log("Connected.")

    def disconnect(self):
        self.running = False
        self.set_enabled(False)
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
            if line:
                self.q.put("Arduino: " + line)

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

        filename = datetime.now().strftime("measurement_%Y%m%d_%H%M%S.csv")
        self.meas_stop.clear()
        self.measuring = True
        self.meas_btn.config(text="Stop measurement")
        self.log(f"Measurement started -> {filename}")
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
            self.mq.put(("done", filename))

    # ---------- GUI update loop ----------
    def poll_queue(self):
        # motor messages
        try:
            while True:
                self.log(self.q.get_nowait())
        except queue.Empty:
            pass

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
                    self.log(f"Measurement stopped. Data saved to: {msg[1]}")
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
                self.ser.close()
            except Exception:
                pass
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
