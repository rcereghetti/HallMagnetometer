"""
Motor control interface for the Arduino code.

Install once:   pip install pyserial
Run:            python motor_control.py

Close the Arduino Serial Monitor first (only one program can use the port).
"""

import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports

BAUD = 9600


class App:
    def __init__(self, root):
        self.root = root
        root.title("Motor control")
        self.ser = None
        self.q = queue.Queue()
        self.running = False

        pad = {"padx": 8, "pady": 6}

        # --- Connection ---
        top = ttk.Frame(root)
        top.pack(fill="x", **pad)
        ttk.Label(top, text="COM port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(top, textvariable=self.port_var,
                                     width=14, state="readonly")
        self.port_box.pack(side="left", padx=6)
        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side="left")
        self.conn_btn = ttk.Button(top, text="Connect", command=self.toggle_connect)
        self.conn_btn.pack(side="left", padx=6)

        # --- Amount ---
        mid = ttk.Frame(root)
        mid.pack(fill="x", **pad)
        ttk.Label(mid, text="Amount (steps):").pack(side="left")
        self.amount_var = tk.StringVar(value="200")
        ttk.Entry(mid, textvariable=self.amount_var, width=10).pack(side="left", padx=6)

        # --- Buttons ---
        btns = ttk.Frame(root)
        btns.pack(fill="x", **pad)
        self.buttons = []
        for text, cmd in [("Home (go to sensor)", self.home),
                          ("Up", self.up),
                          ("Down", self.down),
                          ("Sensor?", self.ask_sensor)]:
            b = ttk.Button(btns, text=text, command=cmd, state="disabled")
            b.pack(side="left", padx=4)
            self.buttons.append(b)

        self.stop_btn = tk.Button(root, text="STOP", bg="#d9534f", fg="white",
                                  font=("Arial", 14, "bold"), command=self.stop,
                                  state="disabled")
        self.stop_btn.pack(fill="x", **pad)

        # --- Status ---
        ttk.Label(root, text="Status:").pack(anchor="w", padx=8)
        self.status = tk.Text(root, height=10, width=50, state="disabled")
        self.status.pack(fill="both", expand=True, padx=8, pady=(0, 8))

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

    # ---------- connection ----------
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

    # ---------- serial ----------
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

    def poll_queue(self):
        try:
            while True:
                self.log(self.q.get_nowait())
        except queue.Empty:
            pass
        self.root.after(100, self.poll_queue)

    def send(self, text):
        if not self.ser:
            return
        try:
            self.ser.write((text + "\n").encode())
            self.log("Sent: " + text)
        except serial.SerialException as e:
            self.log(f"Send error: {e}")

    # ---------- commands ----------
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

    def on_close(self):
        self.running = False
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
