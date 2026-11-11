"""
Simple constant-vibration GUI for the Rolling Stone A7 board (a7_bringup firmware).

Pick an actuator, a frequency and an amplitude, press Start (or Space): the tone runs until you
stop it. The live readout is the vibration RMS measured by the on-board IMU, so you can poke at
a joint and watch the level react.

The GUI re-sends the setting every 0.4 s; if it stops (closed, crashed, cable pulled) the
firmware cuts the haptics after 1.5 s.

Usage:
  .venv/Scripts/python companion/vibe_gui.py            # auto-picks the Espressif USB port
  .venv/Scripts/python companion/vibe_gui.py COM6
"""

import math
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

import serial
from serial.tools import list_ports

CHANNELS = [(1, "LFi (J3)"), (2, "MF (J4)"), (3, "HF (J5)"), (4, "LF (J6)")]
F_MIN, F_MAX = 20.0, 500.0
AMP_MAX = 0.25
PRESETS = [40, 60, 80, 125, 175, 250, 320, 400]
KEEPALIVE_S = 0.4
HISTORY_S = 10.0
LV_MIN, LV_MAX = 1.0, 3000.0  # mg, bar/plot log range


def find_port():
    for p in list_ports.comports():
        if p.vid == 0x303A:
            return p.device
    return None


class Link:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(1.5)  # opening the USB-Serial/JTAG port can reset the board
        self.ser.reset_input_buffer()
        self.q = queue.Queue()
        self.alive = True
        threading.Thread(target=self._rx, daemon=True).start()

    def _rx(self):
        while self.alive:
            try:
                line = self.ser.readline().decode(errors="replace").strip()
            except serial.SerialException as e:
                self.q.put(("err", str(e)))
                return
            if line:
                self.q.put(("line", line))

    def send(self, s):
        try:
            self.ser.write(s.encode())
        except serial.SerialException as e:
            self.q.put(("err", str(e)))

    def close(self):
        self.send("v0\n")
        time.sleep(0.1)
        self.alive = False
        self.ser.close()


class App:
    def __init__(self, root, port):
        self.root = root
        self.link = None
        self.running = False
        self.last_sent = None
        self.last_send_t = 0.0
        self.hist = []  # (t, mg)

        root.title("A7 vibration")
        root.geometry("560x520")
        root.minsize(480, 460)
        pad = dict(padx=10, pady=4)

        top = ttk.Frame(root)
        top.pack(fill="x", **pad)
        ttk.Label(top, text="Port").pack(side="left")
        self.port_var = tk.StringVar(value=port or find_port() or "")
        ports = [p.device for p in list_ports.comports()]
        ttk.Combobox(top, textvariable=self.port_var, values=ports, width=10).pack(side="left", padx=6)
        self.conn_btn = ttk.Button(top, text="Connect", command=self.connect)
        self.conn_btn.pack(side="left")
        self.vsys_lbl = ttk.Label(top, text="VSYS —")
        self.vsys_lbl.pack(side="right")

        chf = ttk.LabelFrame(root, text="Actuator")
        chf.pack(fill="x", **pad)
        self.ch_var = tk.IntVar(value=3)
        for ch, name in CHANNELS:
            ttk.Radiobutton(chf, text=name, value=ch, variable=self.ch_var).pack(side="left", padx=8, pady=4)

        ff = ttk.LabelFrame(root, text="Frequency")
        ff.pack(fill="x", **pad)
        self.f_pos = tk.DoubleVar(value=self.hz_to_pos(175))
        self.f_lbl = ttk.Label(ff, width=8, anchor="e", font=("Segoe UI", 12, "bold"))
        self.f_lbl.pack(side="right", padx=6)
        ttk.Scale(ff, from_=0, to=1, variable=self.f_pos, command=lambda _: self.refresh()).pack(
            fill="x", padx=6, pady=4)
        pf = ttk.Frame(root)
        pf.pack(fill="x", padx=10)
        for hz in PRESETS:
            ttk.Button(pf, text=str(hz), width=5, command=lambda h=hz: self.set_hz(h)).pack(side="left", padx=1)

        af = ttk.LabelFrame(root, text="Amplitude (fraction of full scale)")
        af.pack(fill="x", **pad)
        self.amp = tk.DoubleVar(value=0.10)
        self.a_lbl = ttk.Label(af, width=8, anchor="e", font=("Segoe UI", 12, "bold"))
        self.a_lbl.pack(side="right", padx=6)
        ttk.Scale(af, from_=0, to=AMP_MAX, variable=self.amp, command=lambda _: self.refresh()).pack(
            fill="x", padx=6, pady=4)

        self.go_btn = tk.Button(root, text="Start  (Space)", font=("Segoe UI", 14, "bold"),
                                bg="#2a9d55", fg="white", activebackground="#23844a",
                                command=self.toggle, state="disabled")
        self.go_btn.pack(fill="x", padx=10, pady=8)

        lf = ttk.LabelFrame(root, text="Measured vibration (on-board IMU, RMS)")
        lf.pack(fill="both", expand=True, **pad)
        self.lv_lbl = ttk.Label(lf, text="— mg", font=("Segoe UI", 20, "bold"))
        self.lv_lbl.pack(anchor="w", padx=8)
        self.cv = tk.Canvas(lf, height=120, bg="#111", highlightthickness=0)
        self.cv.pack(fill="both", expand=True, padx=8, pady=6)

        self.status = ttk.Label(root, text="", foreground="#666")
        self.status.pack(fill="x", padx=10, pady=(0, 6))

        root.bind("<space>", lambda e: self.toggle())
        root.protocol("WM_DELETE_WINDOW", self.quit)
        self.refresh()
        if self.port_var.get():
            root.after(100, self.connect)
        root.after(50, self.tick)

    # ── value mapping ──
    @staticmethod
    def hz_to_pos(hz):
        return math.log(hz / F_MIN) / math.log(F_MAX / F_MIN)

    def hz(self):
        return round(F_MIN * (F_MAX / F_MIN) ** self.f_pos.get(), 1)

    def set_hz(self, hz):
        self.f_pos.set(self.hz_to_pos(hz))
        self.refresh()

    def refresh(self):
        self.f_lbl.config(text=f"{self.hz():.0f} Hz")
        self.a_lbl.config(text=f"{self.amp.get():.2f}")

    # ── actions ──
    def connect(self):
        if self.link:
            self.link.close()
            self.link = None
        try:
            self.status.config(text=f"opening {self.port_var.get()} …")
            self.root.update_idletasks()
            self.link = Link(self.port_var.get())
            self.go_btn.config(state="normal")
            self.status.config(text=f"connected to {self.port_var.get()}")
        except (serial.SerialException, OSError) as e:
            self.status.config(text=f"cannot open port: {e}")

    def toggle(self):
        if not self.link:
            return
        self.running = not self.running
        if self.running:
            self.go_btn.config(text="Stop  (Space)", bg="#c0392b", activebackground="#a93226")
        else:
            self.link.send("v0\n")
            self.last_sent = None
            self.go_btn.config(text="Start  (Space)", bg="#2a9d55", activebackground="#23844a")

    def quit(self):
        if self.link:
            self.link.close()
        self.root.destroy()

    # ── periodic ──
    def tick(self):
        now = time.time()
        if self.link:
            while True:
                try:
                    kind, val = self.link.q.get_nowait()
                except queue.Empty:
                    break
                if kind == "err":
                    self.status.config(text=f"serial error: {val}")
                    self.running = True
                    self.toggle()
                    self.link = None
                    self.go_btn.config(state="disabled")
                    break
                self.on_line(val, now)
        if self.link and self.running:
            cmd = f"v{self.ch_var.get()} {self.hz():.1f} {self.amp.get():.3f}\n"
            if cmd != self.last_sent or now - self.last_send_t > KEEPALIVE_S:
                self.link.send(cmd)
                self.last_sent, self.last_send_t = cmd, now
        self.draw(now)
        self.root.after(50, self.tick)

    def on_line(self, line, now):
        if line.startswith("$lv,"):
            p = line.split(",")
            mg = float(p[1])
            self.hist.append((now, mg))
            self.lv_lbl.config(text=f"{mg:.0f} mg" if mg >= 10 else f"{mg:.1f} mg")
            self.vsys_lbl.config(text=f"VSYS {float(p[2]):.2f} V")
        elif line.startswith("$hold,"):
            p = line.split(",")
            self.status.config(text="stopped" if p[1] == "off" else
                               f"holding {dict(CHANNELS)[int(p[1])]} at {float(p[2]):.0f} Hz, {float(p[3]):.2f} FS")
        elif "=====" in line:
            self.status.config(text="board reset (brownout?) — press Start again")
            if self.running:
                self.toggle()
        elif line.startswith("# hold:"):
            self.status.config(text=line[2:])

    def draw(self, now):
        self.hist = [(t, v) for t, v in self.hist if now - t <= HISTORY_S]
        c = self.cv
        c.delete("all")
        w, h = max(c.winfo_width(), 10), max(c.winfo_height(), 10)

        def y(mg):
            v = math.log10(max(mg, LV_MIN) / LV_MIN) / math.log10(LV_MAX / LV_MIN)
            return h - 4 - v * (h - 8)

        for g in (1, 10, 100, 1000):
            c.create_line(0, y(g), w, y(g), fill="#333")
            c.create_text(4, y(g) - 2, text=f"{g} mg", anchor="sw", fill="#777", font=("Segoe UI", 7))
        if len(self.hist) >= 2:
            pts = []
            for t, v in self.hist:
                pts += [w - (now - t) / HISTORY_S * w, y(v)]
            c.create_line(*pts, fill="#4fc3f7", width=2)
        if self.hist and now - self.hist[-1][0] < 1:
            yy = y(self.hist[-1][1])
            c.create_rectangle(w - 14, yy, w - 2, h - 4, fill="#4fc3f7", outline="")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None
    root = tk.Tk()
    App(root, port)
    root.mainloop()


if __name__ == "__main__":
    main()
