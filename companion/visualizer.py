#!/usr/bin/env python3
"""
Rolling Stone Visualizer  —  BLE & USB edition

Connects to the ESP32-C6 "RollingStone" device over BLE or USB serial
and renders a live view of the virtual tube with the rolling ball, plus
scrolling traces and optional audio output.

Transport selection (positional argument):
    Omitted               auto-scan BLE for "RollingStone"
    COM3 or /dev/ttyACM0  USB serial  ($sin_a,x_mm,v_mps  /  # key=val protocol)
    AA:BB:CC:DD:EE:FF     BLE direct address

BLE GATT layout (firmware):
    Service  UUID : 19b10000-e8f2-537e-4f6c-d104768a1214
    Telemetry     : 19b10001-...  NOTIFY   float32[3]: sin_alpha, x_mm, v_mps (tube)
                                           OR float32[?]: grav_x/y/z, impact, phys_hz,
                                                         quat(w,x,y,z), 3x(x,y,z) (marbles)
    Command       : 19b10002-...  WRITE    1 ASCII byte: r/s/m/+/-/?
    Status        : 19b10003-...  READ+NOTIFY  ASCII "MODE,cavity_mm"

Audio synthesis:
    Replicates the Feather firmware rolling-noise wavetable in Python.
    Same position-indexed negative-arch model, same pitch–velocity law.
    Toggle with the "♪ Play audio" checkbox (requires sounddevice).

Usage:
    python visualizer.py                   # BLE auto-scan
    python visualizer.py AA:BB:CC:DD:EE:FF # BLE direct
    python visualizer.py COM3              # USB serial (Windows)
    python visualizer.py /dev/ttyACM0      # USB serial (Linux/macOS)
    python visualizer.py --list            # list serial ports + BLE devices
    python visualizer.py --baud 921600 COM4
"""

import argparse
import asyncio
import collections
import math
import queue
import struct
import sys
import threading
import time

import numpy as np
import serial
import serial.tools.list_ports
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches
from matplotlib.widgets import CheckButtons
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from bleak import BleakClient, BleakScanner

try:
    import sounddevice as sd

    _AUDIO_OK = True
except ImportError:
    _AUDIO_OK = False
    print("sounddevice not found — audio disabled  (pip install sounddevice)")


# ---------------------------------------------------------------------------
# BLE UUIDs
# ---------------------------------------------------------------------------

DEVICE_NAME = "RollingStone"
SVC_UUID = "19b10000-e8f2-537e-4f6c-d104768a1214"
TELEM_UUID = "19b10001-e8f2-537e-4f6c-d104768a1214"
CMD_UUID = "19b10002-e8f2-537e-4f6c-d104768a1214"
STAT_UUID = "19b10003-e8f2-537e-4f6c-d104768a1214"


# ---------------------------------------------------------------------------
# Audio constants  (matches Feather firmware synthesis)
# ---------------------------------------------------------------------------

_SAMPLE_RATE = 22050
_WTABLE_SIZE = 30
_IMPACT_SAMPLES = int(0.0086 * _SAMPLE_RATE + 0.5)  # ≈ 190 samples = 8.6 ms
_IMPACT_VREF = 0.5  # reference speed for impact (m/s) — softer scaling
_AUDIO_GAIN = 0.6  # overall output level

# Negative-arch wavetable: -sin(π·i / (N-1))
_WTABLE = (
    -np.sin(np.pi * np.arange(_WTABLE_SIZE, dtype=np.float32) / (_WTABLE_SIZE - 1))
).astype(np.float32)


# ---------------------------------------------------------------------------
# Colours  (same palette as the orientation visualizer)
# ---------------------------------------------------------------------------

DARK_BG = "#1a1a2e"
PANEL_BG = "#16213e"
INFO_BG = "#0f3460"
C_RED = "#e94560"
C_GREEN = "#0f9b58"
C_BLUE = "#4f8ef7"
C_TEXT = "#dce3f0"
C_MUTED = "#6a7490"
C_TUBE = "#2a3a4a"
C_WALL = "#3a5060"

# ---------------------------------------------------------------------------
# Marble box constants  (must match firmware config.h)
# ---------------------------------------------------------------------------

_MARBLE_COUNT = 3
_BOX_W_MM = 50.0
_BOX_H_MM = 25.0
_BOX_D_MM = 25.0
_MARBLE_RAD_MM = 1.0  # 2mm diameter


def _normalize_quat(qw, qx, qy, qz):
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if n < 1e-6:
        return 1.0, 0.0, 0.0, 0.0
    inv = 1.0 / n
    return qw * inv, qx * inv, qy * inv, qz * inv


def _quat_rotate(points, qw, qx, qy, qz):
    q_vec = np.array([qx, qy, qz], dtype=np.float32)
    t = 2.0 * np.cross(q_vec, points)
    return points + qw * t + np.cross(q_vec, t)


# ---------------------------------------------------------------------------
# Visualizer
# ---------------------------------------------------------------------------

HISTORY = 200  # data-points kept in the scrolling traces


class Visualizer:
    def __init__(self, target: str | None, baud: int = 115200):
        # ── Transport detection ────────────────────────────────────────────
        if target and (target.upper().startswith("COM") or target.startswith("/dev/")):
            self._mode = "serial"
        else:
            self._mode = "ble"
        self._target = target
        self._baud = baud

        # ── Connection state ───────────────────────────────────────────────
        self._connected: bool = False
        self._conn_label: str = ""  # "USB" or "BLE"
        self._ble_client: BleakClient | None = None
        self._ble_loop: asyncio.AbstractEventLoop | None = None
        self._cmd_q: queue.SimpleQueue = queue.SimpleQueue()

        # ── Physics state ──────────────────────────────────────────────────
        self.sin_alpha: float = 0.0
        self.x_mm: float = 500.0
        self.v_mps: float = 0.0
        self.cavity_mm: float = 1000.0
        self.mode: str = "ROLLING"
        self.phys_hz: int = 0

        self._x_hist = collections.deque([500.0] * HISTORY, maxlen=HISTORY)
        self._v_hist = collections.deque([0.0] * HISTORY, maxlen=HISTORY)

        self._pkt_count: int = 0
        self._hz_t0: float = time.monotonic()
        self._hz: float = 0.0

        self._flash_left: int = 0
        self._flash_right: int = 0
        self._prev_x_mm: float = 500.0

        # ── Marble mode state ──────────────────────────────────────────────
        self.marble_mode: bool = False
        self.marbles: list = [
            (_BOX_W_MM / 2, _BOX_H_MM / 2, _BOX_D_MM / 2)
        ] * _MARBLE_COUNT
        self.grav_x: float = 0.0
        self.grav_y: float = 0.0
        self.grav_z: float = 0.0
        self.qw: float = 1.0
        self.qx: float = 0.0
        self.qy: float = 0.0
        self.qz: float = 0.0
        self.impact_energy: float = 0.0
        self._marble_impact_flash: int = 0

        # ── Audio state ────────────────────────────────────────────────────
        # Written by telemetry thread + audio callback; GIL makes float
        # assignments atomic in CPython, so no explicit lock needed.
        self._aud_play: bool = False
        self._aud_x: float = 500.0  # extrapolated position [mm]
        self._aud_v: float = 0.0  # velocity [m/s]
        self._aud_impact: int = 0  # remaining impact samples
        self._aud_impact_amp: float = 1.0
        self._stream = None

        # ── Start threads ──────────────────────────────────────────────────
        threading.Thread(target=self._transport_main, daemon=True).start()
        self._start_audio()
        self._build_figure()

    # ------------------------------------------------------------------
    # Shared telemetry + status updates  (called from transport thread)
    # ------------------------------------------------------------------

    def _on_telem(self, sa: float, xmm: float, vmps: float):
        if xmm < 8.0 and self._prev_x_mm >= 8.0:
            self._flash_left = 8
            self._aud_impact = _IMPACT_SAMPLES
            # Softer scaling: square root of normalized speed (matches firmware)
            norm_speed = abs(vmps) / _IMPACT_VREF
            self._aud_impact_amp = min(math.sqrt(norm_speed), 1.0)
        if xmm > self.cavity_mm - 8.0 and self._prev_x_mm <= self.cavity_mm - 8.0:
            self._flash_right = 8
            self._aud_impact = _IMPACT_SAMPLES
            # Softer scaling: square root of normalized speed (matches firmware)
            norm_speed = abs(vmps) / _IMPACT_VREF
            self._aud_impact_amp = min(math.sqrt(norm_speed), 1.0)
        self._prev_x_mm = xmm
        self.sin_alpha = sa
        self.x_mm = xmm
        self.v_mps = vmps
        self._aud_x = xmm
        self._aud_v = vmps
        self._x_hist.append(xmm)
        self._v_hist.append(vmps)
        self._pkt_count += 1

    def _on_status(self, mode: str | None, cavity_mm: float | None):
        if mode:
            self.mode = mode
            self.marble_mode = mode == "MARBLES"
        if cavity_mm is not None:
            self.cavity_mm = cavity_mm

    def _on_telem_marble(self, grav_x, grav_y, grav_z, impact_e, positions, quat=None):
        self.grav_x = grav_x
        self.grav_y = grav_y
        self.grav_z = grav_z
        self.impact_energy = impact_e
        self.marbles = positions
        if quat is not None:
            self.qw, self.qx, self.qy, self.qz = _normalize_quat(*quat)
        else:
            self.qw, self.qx, self.qy, self.qz = 1.0, 0.0, 0.0, 0.0
        self.marble_mode = True
        self.mode = "MARBLES"
        if positions:
            self._x_hist.append(positions[0][0])
            self._v_hist.append(positions[0][1])
        if impact_e > 0.15:
            self._marble_impact_flash = 8
        self._pkt_count += 1

    def _parse_line(self, line: str):
        """Parse one telemetry or status line from the serial stream."""
        if not line:
            return
        if line.startswith("$"):
            parts = line[1:].split(",")
            if len(parts) == 3:
                try:
                    self._on_telem(*map(float, parts))
                except ValueError:
                    pass
        elif line.startswith("@"):
            parts = line[1:].split(",")
            expected_new = 8 + _MARBLE_COUNT * 3
            expected_old = 4 + _MARBLE_COUNT * 3
            if len(parts) in (expected_new, expected_old):
                try:
                    vals = list(map(float, parts))
                    if len(parts) == expected_new:
                        quat = (vals[4], vals[5], vals[6], vals[7])
                        pos_offset = 8
                    else:
                        quat = None
                        pos_offset = 4
                    positions = [
                        (
                            vals[pos_offset + i * 3],
                            vals[pos_offset + i * 3 + 1],
                            vals[pos_offset + i * 3 + 2],
                        )
                        for i in range(_MARBLE_COUNT)
                    ]
                    self._on_telem_marble(
                        vals[0], vals[1], vals[2], vals[3], positions, quat
                    )
                except ValueError:
                    pass
        elif line.startswith("#"):
            mode = None
            cav = None
            for tok in line[1:].split():
                k, _, v = tok.partition("=")
                if k == "mode" and v in ("ROLLING", "SLIDING", "MARBLES"):
                    mode = v
                elif k == "cavity_mm":
                    try:
                        cav = float(v)
                    except ValueError:
                        pass
                elif k == "phys_hz":
                    try:
                        self.phys_hz = int(v)
                    except ValueError:
                        pass
                elif k in ("grav_x", "grav_y", "grav_z"):
                    # Parse gravity/accel values for marble mode status
                    pass
            self._on_status(mode, cav)

    # ------------------------------------------------------------------
    # Transport dispatcher
    # ------------------------------------------------------------------

    def _transport_main(self):
        if self._mode == "serial":
            self._serial_loop()
        else:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            self._ble_loop = loop
            loop.run_until_complete(self._ble_main())

    # ------------------------------------------------------------------
    # Serial transport
    # ------------------------------------------------------------------

    def _serial_loop(self):
        while True:
            try:
                ser = serial.Serial(self._target, self._baud, timeout=0.05)
                self._connected = True
                self._conn_label = "USB"
                print(f"Connected to {self._target} @ {self._baud} baud")
                ser.write(b"?")  # request current params
                while True:
                    # Drain command queue → serial
                    while True:
                        try:
                            ser.write(self._cmd_q.get_nowait())
                        except queue.Empty:
                            break
                    raw = ser.readline()
                    if raw:
                        self._parse_line(raw.decode("ascii", errors="ignore").strip())
            except Exception as exc:
                print(f"Serial: {exc}")
                self._connected = False
            time.sleep(2.0)

    # ------------------------------------------------------------------
    # BLE transport
    # ------------------------------------------------------------------

    async def _ble_main(self):
        while True:
            try:
                await self._ble_connect_loop()
            except Exception as exc:
                print(f"BLE error: {exc}")
            self._connected = False
            await asyncio.sleep(2.0)

    async def _ble_connect_loop(self):
        address = self._target
        if address is None:
            print(f"Scanning for '{DEVICE_NAME}' …")
            device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
            if device is None:
                print("Not found, retrying …")
                return
            address = device.address
            print(f"Found {DEVICE_NAME} at {address}")

        async with BleakClient(
            address, disconnected_callback=self._on_ble_disconnect
        ) as client:
            self._ble_client = client
            self._connected = True
            self._conn_label = "BLE"
            print(f"BLE connected to {address}")

            stat = await client.read_gatt_char(STAT_UUID)
            self._on_ble_status(None, stat)

            await client.start_notify(TELEM_UUID, self._on_ble_telem)
            await client.start_notify(STAT_UUID, self._on_ble_status)

            # Poll queued commands at 20 Hz
            while client.is_connected:
                while True:
                    try:
                        cmd = self._cmd_q.get_nowait()
                        await client.write_gatt_char(CMD_UUID, cmd, response=False)
                    except queue.Empty:
                        break
                await asyncio.sleep(0.05)

        self._ble_client = None
        self._connected = False

    def _on_ble_disconnect(self, _client):
        self._connected = False
        self._ble_client = None
        print("BLE disconnected — reconnecting …")

    def _on_ble_telem(self, _sender, data: bytearray):
        if len(data) >= 12:
            # Tube mode: 4 floats (sin_alpha, x_mm, v_mps, phys_hz)
            if len(data) == 16:
                vals = struct.unpack_from("<ffff", data)
                self._on_telem(vals[0], vals[1], vals[2])
                self.phys_hz = int(vals[3])
            # Legacy tube mode: 3 floats (sin_alpha, x_mm, v_mps)
            elif len(data) == 12:
                self._on_telem(*struct.unpack_from("<fff", data))
            # Marble mode: 9 header floats + MARBLE_COUNT * 3 position floats
            elif len(data) == 4 * (9 + _MARBLE_COUNT * 3):
                vals = struct.unpack_from(f"<{9 + _MARBLE_COUNT * 3}f", data)
                positions = [
                    (vals[9 + i * 3], vals[9 + i * 3 + 1], vals[9 + i * 3 + 2])
                    for i in range(_MARBLE_COUNT)
                ]
                quat = (vals[5], vals[6], vals[7], vals[8])
                self._on_telem_marble(vals[0], vals[1], vals[2], vals[3], positions, quat)
                self.phys_hz = int(vals[4])
            # Legacy marble mode: 5 header floats + MARBLE_COUNT * 3 position floats
            elif len(data) == 4 * (5 + _MARBLE_COUNT * 3):
                vals = struct.unpack_from(f"<{5 + _MARBLE_COUNT * 3}f", data)
                positions = [
                    (vals[5 + i * 3], vals[5 + i * 3 + 1], vals[5 + i * 3 + 2])
                    for i in range(_MARBLE_COUNT)
                ]
                self._on_telem_marble(vals[0], vals[1], vals[2], vals[3], positions)
                self.phys_hz = int(vals[4])

    def _on_ble_status(self, _sender, data: bytearray):
        try:
            text = bytes(data).decode("ascii").strip()
            parts = text.split(",")
            mode = (
                parts[0]
                if parts and parts[0] in ("ROLLING", "SLIDING", "MARBLES")
                else None
            )
            cav = float(parts[1]) if len(parts) >= 2 else None
            # Parse phys_hz from status (format: "MODE,cavity,rebound=X,phys_hz=Y")
            for part in parts:
                if "phys_hz=" in part:
                    try:
                        self.phys_hz = int(part.split("=")[1])
                    except (ValueError, IndexError):
                        pass
            self._on_status(mode, cav)
        except (ValueError, UnicodeDecodeError):
            pass

    # ------------------------------------------------------------------
    # Commands  (thread-safe: all writes go through the queue)
    # ------------------------------------------------------------------

    def _send_cmd(self, cmd: bytes):
        self._cmd_q.put(cmd)

    # ------------------------------------------------------------------
    # Audio synthesis  (runs in sounddevice C thread at _SAMPLE_RATE Hz)
    # ------------------------------------------------------------------

    def _start_audio(self):
        if not _AUDIO_OK:
            return
        try:
            self._stream = sd.OutputStream(
                samplerate=_SAMPLE_RATE,
                channels=1,
                dtype="float32",
                callback=self._audio_cb,
                blocksize=256,
            )
            self._stream.start()
        except Exception as exc:
            print(f"Audio: {exc} — disabled")
            self._stream = None

    def _audio_cb(self, outdata, frames, time_info, status):
        # Atomic reads via GIL (CPython float/int assignment is pointer-sized)
        v = self._aud_v  # m/s
        x0 = self._aud_x  # mm
        play = self._aud_play
        imp = self._aud_impact
        imp_amp = self._aud_impact_amp

        # Always advance position estimate so audio is seamless when re-enabled
        self._aud_x = x0 + v * 1000.0 * (frames / _SAMPLE_RATE)

        if not play:
            outdata.fill(0)
            return

        # Vectorised rolling noise: wavetable indexed by ball position in mm,
        # so pitch rises naturally with velocity — same law as the firmware.
        t_arr = np.arange(frames, dtype=np.float32) / _SAMPLE_RATE
        x_arr = x0 + v * 1000.0 * t_arr  # mm, per sample
        idx_arr = np.abs(x_arr.astype(np.int32)) % _WTABLE_SIZE
        speed = float(np.clip(abs(v) / 2.0, 0.0, 1.0))
        buf = _WTABLE[idx_arr] * speed

        # Impact pulse overlay (rectangular, same 8.6 ms duration as firmware)
        if imp > 0:
            n = min(imp, frames)
            buf[:n] = imp_amp
            self._aud_impact = max(imp - frames, 0)

        outdata[:] = (buf * _AUDIO_GAIN).reshape(-1, 1)

    def _on_audio_toggle(self, _label):
        self._aud_play = not self._aud_play
        if self._aud_play:
            # Sync position on enable to avoid a startup click
            self._aud_x = self.x_mm
            self._aud_v = self.v_mps

    # ------------------------------------------------------------------
    # Figure layout
    # ------------------------------------------------------------------

    def _build_figure(self):
        matplotlib.rcParams["toolbar"] = "None"
        self.fig = plt.figure(figsize=(13, 8), facecolor=DARK_BG)

        # Main axis starts as 2D, will be replaced with 3D when marble mode activates
        self.ax = self.fig.add_axes([0.01, 0.12, 0.70, 0.80], facecolor=PANEL_BG)
        self.ax_is_3d = False

        self.fig.suptitle(
            "Virtual Rolling Stone  ·  ESP32-C6  ·  Yao & Hayward, Eurohaptics 2006",
            color=C_TEXT,
            fontsize=12,
            y=0.97,
        )

        self.info = self.fig.text(
            0.01,
            0.01,
            "",
            color=C_TEXT,
            fontsize=10,
            fontfamily="monospace",
            verticalalignment="bottom",
            bbox=dict(facecolor=INFO_BG, alpha=0.85, edgecolor="none", pad=7),
        )

        self.fig.text(
            0.73,
            0.01,
            "[r] Rolling  [s] Sliding  [m] Marbles\n[+] longer   [-] shorter   [q] Quit",
            color=C_MUTED,
            fontsize=8,
            fontfamily="monospace",
            ha="left",
            va="bottom",
        )

        # Right column: position (top), velocity (middle), audio checkbox (bottom)
        self.ax_pos = self.fig.add_axes([0.74, 0.57, 0.24, 0.34], facecolor=PANEL_BG)
        self.ax_vel = self.fig.add_axes([0.74, 0.23, 0.24, 0.28], facecolor=PANEL_BG)

        ax_chk = self.fig.add_axes([0.74, 0.12, 0.24, 0.08], facecolor=PANEL_BG)
        for sp in ax_chk.spines.values():
            sp.set_visible(False)
        ax_chk.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

        if _AUDIO_OK and self._stream is not None:
            self._chk = CheckButtons(
                ax_chk,
                ["♪  Play audio"],
                actives=[False],
                label_props={
                    "color": [C_TEXT],
                    "fontsize": [9],
                    "fontfamily": ["monospace"],
                },
                frame_props={
                    "edgecolor": [C_MUTED],
                    "facecolor": [PANEL_BG],
                    "linewidth": [1.2],
                },
                check_props={"color": [C_GREEN]},
            )
            self._chk.on_clicked(self._on_audio_toggle)
        else:
            ax_chk.text(
                0.08,
                0.5,
                "♪  audio unavailable",
                color=C_MUTED,
                fontsize=9,
                fontfamily="monospace",
                va="center",
                transform=ax_chk.transAxes,
            )

        self.fig.canvas.mpl_connect("close_event", self._on_close)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

    def _ensure_3d_axis(self):
        """Convert main axis to 3D if not already."""
        if not self.ax_is_3d:
            self.ax.remove()
            self.ax = self.fig.add_axes(
                [0.01, 0.12, 0.70, 0.80], facecolor=PANEL_BG, projection="3d"
            )
            self.ax_is_3d = True

    def _ensure_2d_axis(self):
        """Convert main axis to 2D if not already."""
        if self.ax_is_3d:
            self.ax.remove()
            self.ax = self.fig.add_axes([0.01, 0.12, 0.70, 0.80], facecolor=PANEL_BG)
            self.ax_is_3d = False

    def _on_close(self, _event):
        if self._stream:
            self._stream.stop()
            self._stream.close()

    def _on_key(self, event):
        cmd = {"r": b"r", "s": b"s", "m": b"m", "+": b"+", "=": b"+", "-": b"-"}.get(
            event.key
        )
        if cmd:
            self._send_cmd(cmd)
            if event.key == "m":
                self.marble_mode = True  # optimistic; confirmed by status reply
        elif event.key == "q":
            plt.close("all")
            sys.exit(0)

    # ------------------------------------------------------------------
    def _ensure_2d_axis(self):
        """Ensure main axis is 2D for tube mode."""
        if hasattr(self.ax, "zaxis"):
            # Recreate as 2D axis
            pos = self.ax.get_position()
            self.ax.remove()
            self.ax = self.fig.add_axes(
                [pos.x0, pos.y0, pos.width, pos.height], facecolor=PANEL_BG
            )

    def _ensure_3d_axis(self):
        """Ensure main axis is 3D for marble mode."""
        if not hasattr(self.ax, "zaxis"):
            # Recreate as 3D axis
            pos = self.ax.get_position()
            self.ax.remove()
            self.ax = self.fig.add_axes(
                [pos.x0, pos.y0, pos.width, pos.height],
                projection="3d",
                facecolor=PANEL_BG,
            )

    # ------------------------------------------------------------------
    def _draw_frame(self):
        if self.marble_mode:
            self._ensure_3d_axis()
            self._draw_marble_box()
            self._draw_marble_history()
        else:
            self._ensure_2d_axis()
            self._draw_tube()
            self._draw_history()

    def _draw_tube(self):
        ax = self.ax
        ax.cla()
        ax.set_facecolor(PANEL_BG)
        ax.axis("off")

        HL = 1.45
        THICK = 0.20
        CAP_W = 0.09

        # Negate: positive sin_alpha → right-end-down → clockwise in display coords
        angle = -math.asin(max(-1.0, min(1.0, self.sin_alpha)))
        ca, sa = math.cos(angle), math.sin(angle)
        px, py = -sa, ca

        def tube_pt(along, across):
            return (along * ca + across * px, along * sa + across * py)

        # Tube body
        ax.add_patch(
            mpatches.Polygon(
                [
                    tube_pt(-HL, THICK),
                    tube_pt(HL, THICK),
                    tube_pt(HL, -THICK),
                    tube_pt(-HL, -THICK),
                ],
                closed=True,
                facecolor=C_TUBE,
                edgecolor=C_MUTED,
                linewidth=1.2,
                alpha=0.85,
                zorder=2,
            )
        )
        ax.add_patch(
            mpatches.Polygon(
                [
                    tube_pt(-HL, THICK),
                    tube_pt(HL, THICK),
                    tube_pt(HL, THICK - 0.03),
                    tube_pt(-HL, THICK - 0.03),
                ],
                closed=True,
                facecolor="#4a6a80",
                alpha=0.5,
                zorder=3,
            )
        )

        # End caps (flash red on impact)
        lf = self._flash_left > 0
        rf = self._flash_right > 0
        if self._flash_left > 0:
            self._flash_left -= 1
        if self._flash_right > 0:
            self._flash_right -= 1

        for side, flash in ((-1, lf), (1, rf)):
            base = side * HL
            ax.add_patch(
                mpatches.Polygon(
                    [
                        tube_pt(base, THICK + 0.02),
                        tube_pt(base + side * CAP_W, THICK + 0.02),
                        tube_pt(base + side * CAP_W, -THICK - 0.02),
                        tube_pt(base, -THICK - 0.02),
                    ],
                    closed=True,
                    facecolor=C_RED if flash else C_WALL,
                    edgecolor=C_MUTED,
                    linewidth=1.0,
                    alpha=0.95,
                    zorder=3,
                )
            )

        # Ball — position mapped from [0, cavity_mm] → [−HL, +HL]
        t_ball = (self.x_mm / max(self.cavity_mm, 1.0) - 0.5) * 2.0 * HL
        bx, by = tube_pt(t_ball, 0)
        spd = abs(self.v_mps)
        r_val = min(spd / 1.5, 1.0)
        ball_c = (r_val * 0.94, 0.55 + 0.30 * (1.0 - r_val), 0.15)
        ball_r = THICK * 0.78
        ax.add_patch(
            mpatches.Circle(
                (bx, by),
                ball_r,
                facecolor=ball_c,
                edgecolor="#ffffffaa",
                linewidth=1.0,
                zorder=5,
            )
        )
        hx = bx + ball_r * 0.28 * (-ca + px) * 0.6
        hy = by + ball_r * 0.28 * (-sa + py) * 0.6
        ax.add_patch(
            mpatches.Circle((hx, hy), ball_r * 0.28, facecolor="#ffffffaa", zorder=6)
        )

        # Tilt angle arc
        arc_r = 0.55
        ax.plot(
            [-arc_r * 0.9, arc_r * 0.9],
            [0, 0],
            color="#33445555",
            linewidth=0.8,
            linestyle="--",
            zorder=1,
        )
        if abs(angle) > 0.02:
            thetas = np.linspace(0, angle, 60)
            ax.plot(
                arc_r * np.cos(thetas),
                arc_r * np.sin(thetas),
                color="#ffcc44",
                linewidth=1.8,
                alpha=0.8,
                zorder=1,
            )
            mid = angle / 2
            ax.text(
                arc_r * 1.18 * math.cos(mid),
                arc_r * 1.18 * math.sin(mid),
                f"{math.degrees(angle):+.1f}°",
                color="#ffcc44",
                fontsize=9,
                ha="center",
                va="center",
                fontfamily="monospace",
            )

        # Gravity reference arrow
        ax.annotate(
            "",
            xy=(1.75, -1.05),
            xytext=(1.75, -0.65),
            arrowprops=dict(
                arrowstyle="-|>", color="#667788", lw=1.5, mutation_scale=9
            ),
            zorder=1,
        )
        ax.text(
            1.75,
            -0.60,
            "g",
            color="#667788",
            fontsize=9,
            ha="center",
            va="bottom",
            fontfamily="monospace",
        )

        # Mode badge
        badge_col = C_BLUE if self.mode == "ROLLING" else C_GREEN
        ax.text(
            -1.75,
            1.15,
            f"◉ {self.mode}",
            color=badge_col,
            fontsize=10,
            fontfamily="monospace",
            fontweight="bold",
            va="top",
        )

        # Connection indicator
        sym = "●" if self._connected else "○"
        col = C_GREEN if self._connected else C_RED
        label = f"{self._conn_label} Connected" if self._connected else "Scanning…"
        ax.text(
            0.55,
            1.15,
            f"{sym} {label}",
            color=col,
            fontsize=9,
            fontfamily="monospace",
            ha="center",
            va="top",
        )

        # Physics Hz (colour-coded: green ≥900, yellow ≥700, red <700)
        hz = self.phys_hz
        if hz > 0:
            phz_col = C_GREEN if hz >= 900 else ("#ffcc44" if hz >= 700 else C_RED)
            ax.text(
                -2.0,
                -1.33,
                f"⚡ {hz} / 1000 Hz",
                color=phz_col,
                fontsize=9,
                fontfamily="monospace",
                va="bottom",
                zorder=10,
            )

        ax.set_xlim(-2.05, 2.05)
        ax.set_ylim(-1.40, 1.40)
        ax.set_aspect("equal", adjustable="box")

        self.info.set_text(
            f"  mode     {self.mode}\n"
            f"  cavity   {self.cavity_mm:.0f} mm\n"
            f"  position {self.x_mm:6.1f} mm\n"
            f"  velocity {self.v_mps:+.4f} m/s\n"
            f"  tilt     {math.degrees(angle):+.1f}°   sin={self.sin_alpha:+.3f}\n"
            f"  serial   {self._hz:.0f} Hz\n"
            f"  physics  {self.phys_hz} / 1000 Hz"
        )

    def _draw_marble_box(self):
        ax = self.ax
        ax.cla()
        ax.set_facecolor(PANEL_BG)
        ax.grid(False)

        W, H, D, R = _BOX_W_MM, _BOX_H_MM, _BOX_D_MM, _MARBLE_RAD_MM
        qw, qx, qy, qz = self.qw, self.qx, self.qy, self.qz
        center = np.array([W / 2, H / 2, D / 2], dtype=np.float32)

        # Draw 3D box wireframe
        vertices = np.array(
            [
                [0, 0, 0],
                [W, 0, 0],
                [W, H, 0],
                [0, H, 0],  # front face (z=0)
                [0, 0, D],
                [W, 0, D],
                [W, H, D],
                [0, H, D],  # back face (z=D)
            ]
        )

        # Rotate box to match game orientation
        rot_vertices = _quat_rotate(vertices - center, qw, qx, qy, qz) + center

        # Define the 6 faces of the box
        faces = [
            [rot_vertices[0], rot_vertices[1], rot_vertices[2], rot_vertices[3]],
            [rot_vertices[4], rot_vertices[5], rot_vertices[6], rot_vertices[7]],
            [rot_vertices[0], rot_vertices[1], rot_vertices[5], rot_vertices[4]],
            [rot_vertices[2], rot_vertices[3], rot_vertices[7], rot_vertices[6]],
            [rot_vertices[0], rot_vertices[3], rot_vertices[7], rot_vertices[4]],
            [rot_vertices[1], rot_vertices[2], rot_vertices[6], rot_vertices[5]],
        ]

        box = Poly3DCollection(
            faces, alpha=0.15, facecolor=C_TUBE, edgecolor=C_MUTED, linewidths=1.5
        )
        ax.add_collection3d(box)

        # Impact flash overlay (change alpha of box)
        if self._marble_impact_flash > 0:
            flash_alpha = self._marble_impact_flash / 8.0 * 0.5
            self._marble_impact_flash -= 1
            flash_overlay = Poly3DCollection(
                faces, alpha=flash_alpha, facecolor=C_RED, edgecolor="none"
            )
            ax.add_collection3d(flash_overlay)

        # Draw marbles as spheres
        u = np.linspace(0, 2 * np.pi, 15)
        v = np.linspace(0, np.pi, 10)
        sphere_x = R * np.outer(np.cos(u), np.sin(v))
        sphere_y = R * np.outer(np.sin(u), np.sin(v))
        sphere_z = R * np.outer(np.ones(np.size(u)), np.cos(v))

        for mx, my, mz in self.marbles:
            m = np.array([mx, my, mz], dtype=np.float32)
            m = _quat_rotate(m - center, qw, qx, qy, qz) + center
            ax.plot_surface(
                sphere_x + m[0],
                sphere_y + m[1],
                sphere_z + m[2],
                color=C_BLUE,
                alpha=0.92,
                edgecolor="#ffffff88",
                linewidth=0.3,
                shade=True,
            )

        # Gravity arrow (3D arrow from center bottom)
        arrow_len = 25.0
        base = np.array([W / 2, H / 2, 0], dtype=np.float32)
        g_vec = np.array(
            [self.grav_x * arrow_len, self.grav_y * arrow_len, self.grav_z * arrow_len],
            dtype=np.float32,
        )
        base = _quat_rotate(base - center, qw, qx, qy, qz) + center
        g_vec = _quat_rotate(g_vec, qw, qx, qy, qz)
        ax.quiver(
            base[0],
            base[1],
            base[2],
            g_vec[0],
            g_vec[1],
            g_vec[2],
            color="#ffcc44",
            arrow_length_ratio=0.3,
            linewidth=2,
        )
        ax.text(
            base[0] + g_vec[0] * 1.3,
            base[1] + g_vec[1] * 1.3,
            base[2] + g_vec[2] * 1.3,
            "a",
            color="#ffcc44",
            fontsize=10,
            fontfamily="monospace",
        )

        # Set equal aspect ratio and limits
        mins = rot_vertices.min(axis=0)
        maxs = rot_vertices.max(axis=0)
        margin = 8.0
        ax.set_xlim([mins[0] - margin, maxs[0] + margin])
        ax.set_ylim([mins[1] - margin, maxs[1] + margin])
        ax.set_zlim([mins[2] - margin, maxs[2] + margin])
        ax.set_box_aspect([maxs[0] - mins[0], maxs[1] - mins[1], maxs[2] - mins[2]])

        # Labels
        ax.set_xlabel("X [mm]", color=C_MUTED, fontsize=8)
        ax.set_ylabel("Y [mm]", color=C_MUTED, fontsize=8)
        ax.set_zlabel("Z [mm]", color=C_MUTED, fontsize=8)

        ax.tick_params(colors=C_MUTED, labelsize=7)
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor("#333")
        ax.yaxis.pane.set_edgecolor("#333")
        ax.zaxis.pane.set_edgecolor("#333")

        # View angle
        ax.view_init(elev=20, azim=45)

        # Mode badge (as 2D text overlay)
        self.fig.text(
            0.02,
            0.90,
            "◉ MARBLES",
            color=C_GREEN,
            fontsize=10,
            fontfamily="monospace",
            fontweight="bold",
        )

        # Connection indicator
        sym = "●" if self._connected else "○"
        col = C_GREEN if self._connected else C_RED
        label = f"{self._conn_label} Connected" if self._connected else "Scanning…"
        self.fig.text(
            0.35, 0.90, f"{sym} {label}", color=col, fontsize=9, fontfamily="monospace"
        )

        # Physics Hz
        hz = self.phys_hz
        if hz > 0:
            phz_col = C_GREEN if hz >= 900 else ("#ffcc44" if hz >= 700 else C_RED)
            self.fig.text(
                0.02,
                0.13,
                f"⚡ {hz} / 1000 Hz",
                color=phz_col,
                fontsize=9,
                fontfamily="monospace",
            )

        self.info.set_text(
            f"  mode     MARBLES\n"
            f"  marbles  {_MARBLE_COUNT}\n"
            f"  box      {_BOX_W_MM:.0f} x {_BOX_H_MM:.0f} x {_BOX_D_MM:.0f} mm\n"
            f"  accel_x  {self.grav_x:+.3f} g\n"
            f"  accel_y  {self.grav_y:+.3f} g\n"
            f"  accel_z  {self.grav_z:+.3f} g\n"
            f"  impact   {self.impact_energy:.2f}\n"
            f"  serial   {self._hz:.0f} Hz\n"
            f"  physics  {self.phys_hz} / 1000 Hz"
        )

    # ------------------------------------------------------------------
    def _draw_history(self):
        t = np.linspace(-HISTORY / 58.0, 0.0, HISTORY)

        ax = self.ax_pos
        ax.cla()
        ax.set_facecolor(PANEL_BG)
        ax.tick_params(colors=C_MUTED, labelsize=7)
        for sp in ax.spines.values():
            sp.set_edgecolor("#333")
        ax.plot(t, list(self._x_hist), color=C_BLUE, linewidth=1.2)
        ax.axhline(0, color="#334", linewidth=0.6)
        ax.axhline(self.cavity_mm, color="#334", linewidth=0.6)
        ax.set_ylim(-30, self.cavity_mm + 30)
        ax.set_xlim(t[0], 0)
        ax.set_ylabel("position [mm]", color=C_MUTED, fontsize=7)
        ax.set_title("History", color=C_TEXT, fontsize=9, pad=4)
        ax.xaxis.set_ticklabels([])

        ax = self.ax_vel
        ax.cla()
        ax.set_facecolor(PANEL_BG)
        ax.tick_params(colors=C_MUTED, labelsize=7)
        for sp in ax.spines.values():
            sp.set_edgecolor("#333")
        vdata = list(self._v_hist)
        ax.plot(t, vdata, color=C_RED, linewidth=1.2)
        ax.axhline(0, color="#445", linewidth=0.8, linestyle="--")
        vmax = max(abs(min(vdata, default=0)), abs(max(vdata, default=0)), 0.1)
        ax.set_ylim(-vmax * 1.15, vmax * 1.15)
        ax.set_xlim(t[0], 0)
        ax.set_ylabel("velocity [m/s]", color=C_MUTED, fontsize=7)
        ax.set_xlabel("time [s]", color=C_MUTED, fontsize=7)

    def _draw_marble_history(self):
        t = np.linspace(-HISTORY / 58.0, 0.0, HISTORY)

        ax = self.ax_pos
        ax.cla()
        ax.set_facecolor(PANEL_BG)
        ax.tick_params(colors=C_MUTED, labelsize=7)
        for sp in ax.spines.values():
            sp.set_edgecolor("#333")
        ax.plot(t, list(self._x_hist), color=C_BLUE, linewidth=1.2)
        ax.axhline(0, color="#334", linewidth=0.6)
        ax.axhline(_BOX_W_MM, color="#334", linewidth=0.6)
        ax.set_ylim(-5, _BOX_W_MM + 5)
        ax.set_xlim(t[0], 0)
        ax.set_ylabel("marble 0  x [mm]", color=C_MUTED, fontsize=7)
        ax.set_title("History", color=C_TEXT, fontsize=9, pad=4)
        ax.xaxis.set_ticklabels([])

        ax = self.ax_vel
        ax.cla()
        ax.set_facecolor(PANEL_BG)
        ax.tick_params(colors=C_MUTED, labelsize=7)
        for sp in ax.spines.values():
            sp.set_edgecolor("#333")
        ax.plot(t, list(self._v_hist), color=C_RED, linewidth=1.2)
        ax.axhline(0, color="#445", linewidth=0.8, linestyle="--")
        ax.axhline(_BOX_H_MM, color="#445", linewidth=0.6)
        ax.set_ylim(-5, _BOX_H_MM + 5)
        ax.set_xlim(t[0], 0)
        ax.set_ylabel("marble 0  y [mm]", color=C_MUTED, fontsize=7)
        ax.set_xlabel("time [s]", color=C_MUTED, fontsize=7)

    # ------------------------------------------------------------------
    def _update(self, _frame):
        now = time.monotonic()
        dt = now - self._hz_t0
        if dt >= 1.0:
            self._hz = self._pkt_count / dt
            self._pkt_count = 0
            self._hz_t0 = now
        self._draw_frame()
        return []

    def run(self):
        self._anim = animation.FuncAnimation(
            self.fig,
            self._update,
            interval=30,
            blit=False,
            cache_frame_data=False,
        )
        plt.show()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


async def _list_all():
    print("── Serial ports ──────────────────────────────────")
    ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
    if ports:
        for p in ports:
            print(f"  {p.device:<16}  {p.description}")
    else:
        print("  (none found)")

    print("\n── BLE devices (scanning 5 s) ────────────────────")
    try:
        devices = await BleakScanner.discover(timeout=5.0)
        if devices:
            for d in sorted(devices, key=lambda x: x.name or ""):
                mark = "  ← RollingStone" if d.name == DEVICE_NAME else ""
                print(f"  {d.address}  {d.name or '(unnamed)'}{mark}")
        else:
            print("  (none found)")
    except Exception as exc:
        print(f"  BLE scan failed: {exc}")


def main():
    parser = argparse.ArgumentParser(
        description="Live rolling-stone sim viewer — BLE or USB serial"
    )
    parser.add_argument(
        "target",
        nargs="?",
        help=(
            "COM3 or /dev/ttyACM0 → USB serial; "
            "AA:BB:CC:DD:EE:FF → BLE direct connect; "
            "omit → auto-scan BLE for 'RollingStone'"
        ),
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate for serial mode (default 115200)",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List available serial ports + nearby BLE devices, then exit",
    )
    args = parser.parse_args()

    if args.list:
        asyncio.run(_list_all())
        sys.exit(0)

    try:
        Visualizer(args.target, args.baud).run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
