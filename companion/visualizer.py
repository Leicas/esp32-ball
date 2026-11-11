#!/usr/bin/env python3
"""
Rolling Stone Visualizer  —  BLE & USB edition

Connects to the ESP32-C6 "RollingStone" device over BLE or USB serial
and renders a live view of the virtual tube with the rolling ball, plus
scrolling traces and optional audio output.

Transport selection (positional argument):
    Omitted               prefer one connected Espressif USB board; otherwise BLE
    COM3 or /dev/ttyACM0  USB serial  ($sin_a,x_mm,v_mps  /  # key=val protocol)
    AA:BB:CC:DD:EE:FF     BLE direct address

BLE GATT layout (firmware):
    Service  UUID : 19b10000-e8f2-537e-4f6c-d104768a1214
    Telemetry     : 19b10001-...  NOTIFY   float32[3]: sin_alpha, x_mm, v_mps (tube)
                                           OR float32[?]: grav_x/y/z, impact, phys_hz,
                                                         quat(w,x,y,z), 3x(x,y,z) (marbles)
    Command       : 19b10002-...  WRITE    1 ASCII byte: r/t/s/m/+/-/?
    Status        : 19b10003-...  READ+NOTIFY  ASCII "MODE,cavity_mm"

Audio synthesis:
    ROLLING uses the classic position-indexed wavetable. TEXTURED uses an
    approximate spatial-noise and resonant-ring model for speaker preview.
    Toggle with the "Approx preview" checkbox (requires sounddevice).

Usage:
    python visualizer.py                   # automatic USB / BLE connection
    python visualizer.py --ble             # force BLE auto-scan
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
from matplotlib.widgets import Button, CheckButtons
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

# Deterministic spatial roughness maps for the approximate TEXTURED preview.
# Each map repeats only after several metres of travel, well beyond the tube.
_TEXTURE_RNG = np.random.default_rng(0xA7)
_TEXTURE_SPACING_MM = (18.0, 3.7, 0.7, 1.2, 0.24)
_TEXTURE_TABLES = tuple(
    _TEXTURE_RNG.standard_normal(16384).astype(np.float32) for _ in _TEXTURE_SPACING_MM
)
_TEXTURE_MODES = (
    (90.0, 0.055),
    (280.0, 0.035),
    (430.0, 0.022),
    (700.0, 0.012),
    (1800.0, 0.007),
    (3600.0, 0.004),
)
_TEXTURE_RESONATORS = tuple(
    (
        2.0 * (radius := math.exp(-1.0 / (decay * _SAMPLE_RATE)))
        * math.cos(2.0 * math.pi * freq / _SAMPLE_RATE),
        -(radius * radius),
        2.0 * (1.0 - radius) * math.sin(2.0 * math.pi * freq / _SAMPLE_RATE),
    )
    for freq, decay in _TEXTURE_MODES
)
_TEXTURE_OUTPUT_GAIN = 19.5  # calibrated against the classic preview at 0.4–0.8 m/s


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
_MARBLE_RAD_MM = 1.0  # default fallback for old firmware
# Per-marble defaults matching firmware config.h [heavy steel, medium glass, light plastic]
_MARBLE_RADII_MM = [1.5, 1.0, 0.7]
_MARBLE_COLORS = ["#c0c0c0", "#44aaff", "#ff6644"]  # silver, blue, orange


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


def _spatial_texture(position_mm: np.ndarray, scale: int) -> np.ndarray:
    table = _TEXTURE_TABLES[scale]
    q = np.mod(position_mm / _TEXTURE_SPACING_MM[scale], table.size)
    idx = q.astype(np.intp)
    frac = q - idx
    return table[idx] + frac * (table[(idx + 1) % table.size] - table[idx])


def _run_resonator(signal: np.ndarray, coeffs, state) -> tuple[np.ndarray, tuple[float, float]]:
    a1, a2, b0 = coeffs
    y1, y2 = state
    output = np.empty_like(signal)
    for i, sample in enumerate(signal):
        y = a1 * y1 + a2 * y2 + b0 * float(sample)
        output[i] = y
        y2, y1 = y1, y
    return output, (y1, y2)


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
        self._transport_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._state_lock = threading.RLock()
        self._cmd_q: queue.Queue = queue.Queue(maxsize=16)
        self._pending_commands: collections.deque = collections.deque()
        self._serial_handle = None

        # Read-only transport state for the frontend. A telemetry age of None
        # means this connection has not delivered a valid sample yet.
        self.connection_state: str = "connecting"
        self.connection_detail: str = "Starting transport"
        self.telemetry_age_s: float | None = None
        self.last_command_feedback: str = ""
        self.rebound: bool | None = None
        self._last_telem_at: float | None = None
        self._connected_at: float | None = None
        self._stale_after_s = 0.5
        self._command_ack_timeout_s = 2.0

        # ── Physics state ──────────────────────────────────────────────────
        self.sin_alpha: float = 0.0
        self.x_mm: float = 500.0
        self.v_mps: float = 0.0
        self.cavity_mm: float = 1000.0
        self.mode: str = "TEXTURED"
        self.phys_hz: int = 0
        self.phys_target_hz: int = 0
        self.imu_hz: int = 0
        self.haptic_gain: float | None = None
        self._telem_time: float = time.monotonic()

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
        self.marble_radii: list = list(_MARBLE_RADII_MM)
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
        self._aud_sample_index: int = 0
        self._texture_state = ((0.0, 0.0),) * len(_TEXTURE_MODES)
        self._stream = None

        # ── Start threads ──────────────────────────────────────────────────
        self._transport_thread = threading.Thread(
            target=self._transport_main, name="visualizer-transport", daemon=True
        )
        self._transport_thread.start()
        self._start_audio()
        self._build_figure()

    # ------------------------------------------------------------------
    # Shared telemetry + status updates  (called from transport thread)
    # ------------------------------------------------------------------

    def _on_telem(self, sa: float, xmm: float, vmps: float):
        if not all(math.isfinite(value) for value in (sa, xmm, vmps)):
            return
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
        self._telem_time = time.monotonic()
        with self._state_lock:
            self._last_telem_at = self._telem_time
            self.telemetry_age_s = 0.0
            if self._connected:
                self.connection_state = "connected"
                self.connection_detail = f"{self._conn_label} telemetry active"
        self._aud_x = xmm
        self._aud_v = vmps
        self._x_hist.append(xmm)
        self._v_hist.append(vmps)
        self._pkt_count += 1

    def _on_status(
        self,
        mode: str | None,
        cavity_mm: float | None,
        rebound: bool | None = None,
        haptic_gain: float | None = None,
    ):
        if mode:
            if mode != self.mode:
                self._texture_state = ((0.0, 0.0),) * len(_TEXTURE_MODES)
                self._aud_impact = 0
            self.mode = mode
            self.marble_mode = mode == "MARBLES"
        if cavity_mm is not None:
            self.cavity_mm = cavity_mm
        if rebound is not None:
            self.rebound = rebound
        if haptic_gain is not None and math.isfinite(haptic_gain):
            self.haptic_gain = haptic_gain

    def _display_position(self) -> float:
        """Predict position between telemetry packets, bounded by tube walls."""
        telemetry_age = getattr(self, "telemetry_age_s", None)
        if telemetry_age is None or telemetry_age > 0.5:
            return min(max(self.x_mm, 0.0), self.cavity_mm)
        age = min(max(time.monotonic() - self._telem_time, 0.0), 0.10)
        return min(max(self.x_mm + self.v_mps * 1000.0 * age, 0.0), self.cavity_mm)

    @staticmethod
    def _physics_rate_color(actual_hz: int, target_hz: int) -> str:
        if target_hz <= 0:
            return C_MUTED
        if actual_hz >= target_hz * 0.9:
            return C_GREEN
        if actual_hz >= target_hz * 0.7:
            return "#ffcc44"
        return C_RED

    def _on_telem_marble(self, grav_x, grav_y, grav_z, impact_e, positions, quat=None, radii=None):
        values = [grav_x, grav_y, grav_z, impact_e]
        values.extend(value for position in positions for value in position)
        if quat is not None:
            values.extend(quat)
        if radii is not None:
            values.extend(radii)
        if not all(math.isfinite(value) for value in values):
            return
        if self.mode != "MARBLES":
            self._texture_state = ((0.0, 0.0),) * len(_TEXTURE_MODES)
            self._aud_impact = 0
        self.grav_x = grav_x
        self.grav_y = grav_y
        self.grav_z = grav_z
        self.impact_energy = impact_e
        self.marbles = positions
        if radii is not None:
            self.marble_radii = radii
        if quat is not None:
            self.qw, self.qx, self.qy, self.qz = _normalize_quat(*quat)
        else:
            self.qw, self.qx, self.qy, self.qz = 1.0, 0.0, 0.0, 0.0
        self.marble_mode = True
        self.mode = "MARBLES"
        now = time.monotonic()
        with self._state_lock:
            self._last_telem_at = now
            self.telemetry_age_s = 0.0
            if self._connected:
                self.connection_state = "connected"
                self.connection_detail = f"{self._conn_label} telemetry active"
        if positions:
            self._x_hist.append(positions[0][0])
            self._v_hist.append(positions[0][1])
        if impact_e > 0.15:
            self._marble_impact_flash = 8
        self._pkt_count += 1

    def _set_transport_state(self, state: str, detail: str, connected: bool | None = None):
        with self._state_lock:
            if connected is not None:
                self._connected = connected
                if connected:
                    self._connected_at = time.monotonic()
                    self._last_telem_at = None
                    self.telemetry_age_s = None
                elif self._last_telem_at is not None:
                    self.telemetry_age_s = max(time.monotonic() - self._last_telem_at, 0.0)
            self.connection_state = state
            self.connection_detail = detail

    def _telemetry_is_fresh(self) -> bool:
        with self._state_lock:
            if not self._connected or self._last_telem_at is None:
                return False
            return time.monotonic() - self._last_telem_at <= self._stale_after_s

    def _refresh_transport_state(self, now: float | None = None):
        now = time.monotonic() if now is None else now
        became_stale = False
        with self._state_lock:
            if not self._connected:
                return
            if self._last_telem_at is None:
                age = now - (self._connected_at or now)
                self.telemetry_age_s = None
                if age > self._stale_after_s:
                    became_stale = self.connection_state != "stale"
                    self.connection_state = "stale"
                    self.connection_detail = "Connected; waiting for telemetry"
            else:
                age = max(now - self._last_telem_at, 0.0)
                self.telemetry_age_s = age
                if age > self._stale_after_s:
                    became_stale = self.connection_state != "stale"
                    self.connection_state = "stale"
                    self.connection_detail = f"No telemetry for {age:.1f} s"
                else:
                    self.connection_state = "connected"
                    self.connection_detail = f"{self._conn_label} telemetry active"
        if became_stale:
            self._drop_commands("telemetry stale")

    def _mark_command_sent(self, cmd: bytes):
        now = time.monotonic()
        with self._state_lock:
            for pending in self._pending_commands:
                if pending["cmd"] == cmd and pending["sent_at"] is None:
                    pending["sent_at"] = now
                    self.last_command_feedback = f"Sent {pending['label']}"
                    return
            if cmd == b"?":
                self.last_command_feedback = "Status requested"

    def _confirm_command(self, key: str, value):
        now = time.monotonic()
        with self._state_lock:
            for index, pending in enumerate(self._pending_commands):
                if pending["ack_key"] != key or pending["sent_at"] is None:
                    continue
                if now - pending["sent_at"] > self._command_ack_timeout_s:
                    continue
                expected = pending["expected"]
                if expected is not None and expected != value:
                    continue
                del self._pending_commands[index]
                self.last_command_feedback = f"Confirmed {pending['label']}"
                return

    def _expire_commands(self, now: float | None = None):
        now = time.monotonic() if now is None else now
        expired = []
        with self._state_lock:
            for pending in tuple(self._pending_commands):
                sent_at = pending["sent_at"]
                if sent_at is not None and now - sent_at > self._command_ack_timeout_s:
                    self._pending_commands.remove(pending)
                    expired.append(pending)
            if expired:
                self.last_command_feedback = f"No confirmation for {expired[-1]['label']}"

    def _drop_commands(self, reason: str):
        while True:
            try:
                self._cmd_q.get_nowait()
            except queue.Empty:
                break
        with self._state_lock:
            if self._pending_commands:
                latest = self._pending_commands[-1]["label"]
                self.last_command_feedback = f"Not confirmed: {latest} ({reason})"
                self._pending_commands.clear()

    @staticmethod
    def _command_spec(cmd: bytes):
        specs = {
            b"r": ("mode TEXTURED", "mode", "TEXTURED"),
            b"l": ("mode LIGHTSABER", "mode", "LIGHTSABER"),
            b"t": ("mode TEXTURED", "mode", "TEXTURED"),
            b"s": ("mode SLIDING", "mode", "SLIDING"),
            b"m": ("mode MARBLES", "mode", "MARBLES"),
            b"g": ("gain cycle", "haptic_gain", None),
            b"b": ("rebound toggle", "rebound", None),
            b"+": ("cavity +50 mm", "cavity_mm", None),
            b"-": ("cavity -50 mm", "cavity_mm", None),
            b"?": ("status query", None, None),
        }
        return specs.get(cmd)

    def _parse_line(self, line: str):
        """Parse one telemetry or status line from the serial stream."""
        if not line:
            return
        if line.startswith("$"):
            parts = line[1:].split(",")
            if len(parts) == 3:
                try:
                    values = tuple(map(float, parts))
                    if all(math.isfinite(value) for value in values):
                        self._on_telem(*values)
                except ValueError:
                    pass
        elif line.startswith("@"):
            parts = line[1:].split(",")
            # New format: 8 header + MARBLE_COUNT * 4 (x,y,z,r per marble)
            expected_v3 = 8 + _MARBLE_COUNT * 4
            # Legacy formats: 8 header + 3 per marble, or 4 header + 3 per marble
            expected_v2 = 8 + _MARBLE_COUNT * 3
            expected_v1 = 4 + _MARBLE_COUNT * 3
            if len(parts) in (expected_v3, expected_v2, expected_v1):
                try:
                    vals = list(map(float, parts))
                    if not all(math.isfinite(value) for value in vals):
                        return
                    if len(parts) == expected_v3:
                        quat = (vals[4], vals[5], vals[6], vals[7])
                        positions = [
                            (vals[8 + i * 4], vals[8 + i * 4 + 1], vals[8 + i * 4 + 2])
                            for i in range(_MARBLE_COUNT)
                        ]
                        radii = [vals[8 + i * 4 + 3] for i in range(_MARBLE_COUNT)]
                    elif len(parts) == expected_v2:
                        quat = (vals[4], vals[5], vals[6], vals[7])
                        positions = [
                            (vals[8 + i * 3], vals[8 + i * 3 + 1], vals[8 + i * 3 + 2])
                            for i in range(_MARBLE_COUNT)
                        ]
                        radii = None
                    else:
                        quat = None
                        positions = [
                            (vals[4 + i * 3], vals[4 + i * 3 + 1], vals[4 + i * 3 + 2])
                            for i in range(_MARBLE_COUNT)
                        ]
                        radii = None
                    self._on_telem_marble(
                        vals[0], vals[1], vals[2], vals[3], positions, quat, radii
                    )
                except ValueError:
                    pass
        elif line.startswith("#"):
            mode = None
            cav = None
            rebound = None
            gain = None
            for tok in line[1:].split():
                k, _, v = tok.partition("=")
                if k == "mode" and v in ("LIGHTSABER", "TEXTURED", "SLIDING", "MARBLES"):
                    mode = v
                elif k == "cavity_mm":
                    try:
                        value = float(v)
                        if math.isfinite(value):
                            cav = value
                    except ValueError:
                        pass
                elif k == "rebound":
                    if v in ("0", "false", "False"):
                        rebound = False
                    elif v in ("1", "true", "True"):
                        rebound = True
                elif k == "phys_hz":
                    try:
                        self.phys_hz = int(v)
                    except ValueError:
                        pass
                elif k == "phys_target_hz":
                    try:
                        self.phys_target_hz = int(v)
                    except ValueError:
                        pass
                elif k == "imu_hz":
                    try:
                        self.imu_hz = int(v)
                    except ValueError:
                        pass
                elif k == "haptic_gain":
                    try:
                        value = float(v)
                        if math.isfinite(value):
                            gain = value
                    except ValueError:
                        pass
                elif k in ("grav_x", "grav_y", "grav_z"):
                    # Parse gravity/accel values for marble mode status
                    pass
            self._on_status(mode, cav, rebound, gain)
            if mode is not None:
                self._confirm_command("mode", mode)
            if cav is not None:
                self._confirm_command("cavity_mm", cav)
            if rebound is not None:
                self._confirm_command("rebound", rebound)
            if gain is not None:
                self._confirm_command("haptic_gain", gain)

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
        while not self._stop_event.is_set():
            ser = None
            try:
                ser = serial.Serial(
                    self._target, self._baud, timeout=0.02, write_timeout=0.25
                )
                self._serial_handle = ser
                self._conn_label = "USB"
                self._set_transport_state(
                    "connected", f"USB open on {self._target}; awaiting telemetry", True
                )
                print(f"Connected to {self._target} @ {self._baud} baud")
                ser.write(b"?")  # initial state request is not a user command
                while not self._stop_event.is_set():
                    self._refresh_transport_state()
                    self._expire_commands()
                    while True:
                        try:
                            cmd = self._cmd_q.get_nowait()
                        except queue.Empty:
                            break
                        if not self._telemetry_is_fresh():
                            self._drop_commands("telemetry stale")
                            break
                        ser.write(cmd)
                        self._mark_command_sent(cmd)
                    raw = ser.readline()
                    if raw:
                        self._parse_line(raw.decode("ascii", errors="ignore").strip())
            except Exception as exc:
                if not self._stop_event.is_set():
                    detail = f"USB error: {exc}"
                    self._set_transport_state("reconnecting", detail, False)
                    print(f"Serial: {exc}")
            finally:
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass
                self._serial_handle = None
                if not self._stop_event.is_set():
                    if self.connection_state != "reconnecting":
                        self._set_transport_state("reconnecting", "USB disconnected", False)
                else:
                    self._set_transport_state("stopped", "Transport stopped", False)
                self._drop_commands("USB disconnected")
            if not self._stop_event.is_set():
                self._stop_event.wait(1.0)

    # ------------------------------------------------------------------
    # BLE transport
    # ------------------------------------------------------------------

    async def _ble_main(self):
        while not self._stop_event.is_set():
            try:
                await self._ble_connect_loop()
            except Exception as exc:
                if not self._stop_event.is_set():
                    print(f"BLE error: {exc}")
                    self._set_transport_state("reconnecting", f"BLE error: {exc}", False)
            self._drop_commands("BLE disconnected")
            if not self._stop_event.is_set():
                for _ in range(10):
                    if self._stop_event.is_set():
                        break
                    await asyncio.sleep(0.05)

    async def _ble_connect_loop(self):
        address = self._target
        if address is None:
            print(f"Scanning for '{DEVICE_NAME}' …")
            device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=2.0)
            if device is None:
                self._set_transport_state("reconnecting", f"Searching for {DEVICE_NAME}", False)
                return
            address = device.address
            print(f"Found {DEVICE_NAME} at {address}")

        async with BleakClient(
            address, disconnected_callback=self._on_ble_disconnect
        ) as client:
            self._ble_client = client
            self._conn_label = "BLE"
            self._set_transport_state(
                "connected", "BLE connected; awaiting telemetry", True
            )
            print(f"BLE connected to {address}")

            stat = await client.read_gatt_char(STAT_UUID)
            self._on_ble_status(None, stat)

            await client.start_notify(TELEM_UUID, self._on_ble_telem)
            await client.start_notify(STAT_UUID, self._on_ble_status)

            # Drain promptly without busy-spinning the asyncio transport loop.
            while client.is_connected and not self._stop_event.is_set():
                self._refresh_transport_state()
                self._expire_commands()
                while True:
                    try:
                        cmd = self._cmd_q.get_nowait()
                    except queue.Empty:
                        break
                    if not self._telemetry_is_fresh():
                        self._drop_commands("telemetry stale")
                        break
                    await client.write_gatt_char(CMD_UUID, cmd, response=False)
                    self._mark_command_sent(cmd)
                await asyncio.sleep(0.01)

        self._ble_client = None
        self._set_transport_state("reconnecting", "BLE disconnected", False)
        self._drop_commands("BLE disconnected")

    def _on_ble_disconnect(self, _client):
        self._set_transport_state("reconnecting", "BLE disconnected", False)
        self._ble_client = None
        self._drop_commands("BLE disconnected")
        print("BLE disconnected — reconnecting …")

    def _on_ble_telem(self, _sender, data: bytearray):
        if len(data) >= 12:
            # Tube mode: 4 floats (sin_alpha, x_mm, v_mps, phys_hz)
            if len(data) == 16:
                vals = struct.unpack_from("<ffff", data)
                if not all(math.isfinite(value) for value in vals):
                    return
                self._on_telem(vals[0], vals[1], vals[2])
                self.phys_hz = int(vals[3])
            # Legacy tube mode: 3 floats (sin_alpha, x_mm, v_mps)
            elif len(data) == 12:
                self._on_telem(*struct.unpack_from("<fff", data))
            # New marble mode: 9 header floats + MARBLE_COUNT * 4 (x,y,z,r per marble)
            elif len(data) == 4 * (9 + _MARBLE_COUNT * 4):
                vals = struct.unpack_from(f"<{9 + _MARBLE_COUNT * 4}f", data)
                if not all(math.isfinite(value) for value in vals):
                    return
                positions = [
                    (vals[9 + i * 4], vals[9 + i * 4 + 1], vals[9 + i * 4 + 2])
                    for i in range(_MARBLE_COUNT)
                ]
                radii = [vals[9 + i * 4 + 3] for i in range(_MARBLE_COUNT)]
                quat = (vals[5], vals[6], vals[7], vals[8])
                self._on_telem_marble(vals[0], vals[1], vals[2], vals[3], positions, quat, radii)
                self.phys_hz = int(vals[4])
            # Legacy marble mode: 9 header + MARBLE_COUNT * 3 position floats
            elif len(data) == 4 * (9 + _MARBLE_COUNT * 3):
                vals = struct.unpack_from(f"<{9 + _MARBLE_COUNT * 3}f", data)
                if not all(math.isfinite(value) for value in vals):
                    return
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
                if not all(math.isfinite(value) for value in vals):
                    return
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
                if parts and parts[0] in ("LIGHTSABER", "TEXTURED", "SLIDING", "MARBLES")
                else None
            )
            cav = None
            if len(parts) >= 2:
                value = float(parts[1])
                cav = value if math.isfinite(value) else None
            rebound = None
            gain = None
            # Status format: MODE,cavity,key=value,...
            for part in parts:
                key, sep, value = part.partition("=")
                if sep and key == "phys_hz":
                    try:
                        self.phys_hz = int(value)
                    except ValueError:
                        pass
                elif sep and key == "phys_target_hz":
                    try:
                        self.phys_target_hz = int(value)
                    except ValueError:
                        pass
                elif sep and key == "imu_hz":
                    try:
                        self.imu_hz = int(value)
                    except ValueError:
                        pass
                elif sep and key == "haptic_gain":
                    try:
                        parsed_gain = float(value)
                        if math.isfinite(parsed_gain):
                            gain = parsed_gain
                    except ValueError:
                        pass
                elif sep and key == "rebound":
                    if value in ("0", "false", "False"):
                        rebound = False
                    elif value in ("1", "true", "True"):
                        rebound = True
            self._on_status(mode, cav, rebound, gain)
            if mode is not None:
                self._confirm_command("mode", mode)
            if cav is not None:
                self._confirm_command("cavity_mm", cav)
            if rebound is not None:
                self._confirm_command("rebound", rebound)
            if gain is not None:
                self._confirm_command("haptic_gain", gain)
        except (ValueError, UnicodeDecodeError):
            pass

    # ------------------------------------------------------------------
    # Commands  (thread-safe: all writes go through the queue)
    # ------------------------------------------------------------------

    def _send_cmd(self, cmd: bytes):
        cmd = bytes(cmd)
        spec = self._command_spec(cmd) if len(cmd) == 1 else None
        if spec is None:
            self.last_command_feedback = "Command not recognized"
            return
        label, ack_key, expected = spec
        if cmd == b"?":
            expected = None
        if self._stop_event.is_set() or not self._telemetry_is_fresh():
            self.last_command_feedback = f"Not sent: {label} ({self.connection_state})"
            return

        pending = None
        if ack_key is not None:
            pending = {
                "cmd": cmd,
                "label": label,
                "ack_key": ack_key,
                "expected": expected,
                "queued_at": time.monotonic(),
                "sent_at": None,
            }
            with self._state_lock:
                self._pending_commands.append(pending)
        try:
            self._cmd_q.put_nowait(cmd)
        except queue.Full:
            if pending is not None:
                with self._state_lock:
                    try:
                        self._pending_commands.remove(pending)
                    except ValueError:
                        pass
            self.last_command_feedback = f"Not sent: command queue full ({label})"
            return
        self.last_command_feedback = f"Queued {label}"

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
        mode_name = self.mode
        v = self._aud_v  # m/s
        x0 = self._aud_x  # mm
        fresh = (
            self._connected
            and self.telemetry_age_s is not None
            and self.telemetry_age_s <= self._stale_after_s
        )
        play = self._aud_play and fresh
        imp = self._aud_impact
        imp_amp = self._aud_impact_amp

        # Always advance position estimate so audio is seamless when re-enabled
        self._aud_x = x0 + v * 1000.0 * (frames / _SAMPLE_RATE)

        if not play:
            outdata.fill(0)
            self._aud_impact = 0
            self._texture_state = ((0.0, 0.0),) * len(_TEXTURE_MODES)
            return

        sample_index = self._aud_sample_index + np.arange(frames, dtype=np.float32)
        self._aud_sample_index += frames
        t_arr = np.arange(frames, dtype=np.float32) / _SAMPLE_RATE
        x_arr = x0 + v * 1000.0 * t_arr  # mm, per sample
        speed = abs(v)

        if mode_name == "TEXTURED":
            # Approximate the richer A7 sound with deterministic roughness at
            # five spatial scales, each exciting a damped resonance. The finest
            # texture fades at high ball speed; all roughness stops at rest.
            speed_gain = min(speed / 0.8, 1.0) if speed >= 0.02 else 0.0
            fine_atten = 1.0 / (1.0 + max(speed - 0.35, 0.0) * 1.5)
            detail_atten = max(0.0, 1.0 - speed / (_TEXTURE_SPACING_MM[4] * 0.4 * _SAMPLE_RATE))
            sample_time = sample_index / _SAMPLE_RATE
            excitations = [
                _spatial_texture(x_arr, 0) * (0.58 * speed_gain),
                _spatial_texture(x_arr, 1) * (0.30 * speed_gain),
                _spatial_texture(x_arr, 2) * (0.12 * speed_gain * fine_atten),
                _spatial_texture(x_arr, 3) * (0.16 * speed_gain),
                _spatial_texture(x_arr, 4) * (0.11 * speed_gain * detail_atten),
                _spatial_texture(x_arr, 4) * (0.11 * speed_gain * detail_atten),
            ]
            # At the host's lower sample rate, add quiet mode-centered detail
            # carriers, modulated by the fine spatial maps, so the upper bands
            # remain audible even when a roughness scale traverses below a mode.
            excitations[3] += 0.12 * speed_gain * _spatial_texture(x_arr, 3) * np.sin(2.0 * np.pi * 700.0 * sample_time)
            excitations[4] += 0.10 * speed_gain * detail_atten * _spatial_texture(x_arr, 4) * np.sin(2.0 * np.pi * 1800.0 * sample_time)
            excitations[5] += 0.07 * speed_gain * detail_atten * _spatial_texture(x_arr, 4) * np.sin(2.0 * np.pi * 3600.0 * sample_time)

            # A rounded 8.6 ms strike excites the resonators; their state then
            # rings down at 90, 280, and 430 Hz after the strike has ended.
            if imp > 0:
                n = min(imp, frames)
                phase = (np.arange(n, dtype=np.float32) + 0.5) / max(n, 1)
                strike = imp_amp * 0.8 * np.sin(np.pi * phase) ** 2
                for i, scale in enumerate((0.50, 0.32, 0.18, 0.18, 0.13, 0.10)):
                    excitations[i][:n] += strike * scale
                self._aud_impact = max(imp - frames, 0)

            state = self._texture_state
            modes = []
            next_state = []
            for excitation, coeffs, mode_state in zip(excitations, _TEXTURE_RESONATORS, state):
                response, updated = _run_resonator(excitation, coeffs, mode_state)
                modes.append(response)
                next_state.append(updated)
            if self.mode == mode_name:
                self._texture_state = tuple(next_state)
            existing_hf = 0.82 * (0.55 * modes[0] + 0.30 * modes[1] + 0.15 * modes[2])
            detail_hf = 0.12 * modes[3] + 0.10 * modes[4] + 0.06 * modes[5]
            buf = (existing_hf + detail_hf) * _TEXTURE_OUTPUT_GAIN
        else:
            self._texture_state = ((0.0, 0.0),) * len(_TEXTURE_MODES)
            # Classic rolling mode keeps the original wavetable preview.
            if self.mode == "ROLLING":
                idx_arr = np.abs(x_arr.astype(np.int32)) % _WTABLE_SIZE
                buf = _WTABLE[idx_arr] * float(np.clip(speed / 2.0, 0.0, 1.0))
            else:
                buf = np.zeros(frames, dtype=np.float32)

            # Sliding and classic rolling retain the simple impact monitor.
            if imp > 0:
                n = min(imp, frames)
                buf[:n] = imp_amp
                self._aud_impact = max(imp - frames, 0)

        outdata[:] = (np.clip(buf * _AUDIO_GAIN, -1.0, 1.0)).reshape(-1, 1)

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
        self.fig = plt.figure(figsize=(14, 8.5), facecolor=DARK_BG)

        # A wide instrument view on the left; histories and status stay in a
        # fixed rail so the live scene never competes with its controls.
        self.ax = self.fig.add_axes([0.02, 0.18, 0.68, 0.74], facecolor=PANEL_BG)
        self.ax_is_3d = False
        self.fig.text(0.025, 0.965, "Rolling Stone", color=C_TEXT,
                      fontsize=16, fontweight="bold", va="top")
        self.fig.text(0.16, 0.965, "LIVE SIMULATOR  /  ESP32-C6",
                      color=C_MUTED, fontsize=9, va="top")
        self._mode_header = self.fig.text(0.72, 0.965, "", color=C_GREEN,
                                          fontsize=11, fontweight="bold", va="top")
        self._connection_header = self.fig.text(0.98, 0.965, "", color=C_MUTED,
                                                fontsize=9, ha="right", va="top")
        self._feedback_header = self.fig.text(0.72, 0.925, "", color=C_MUTED,
                                              fontsize=8, ha="right", va="top")
        self.info = self.fig.text(0.035, 0.145, "", color=C_TEXT, fontsize=8.5,
                                  fontfamily="monospace", va="top")

        self.ax_pos = self.fig.add_axes([0.74, 0.60, 0.24, 0.30], facecolor=PANEL_BG)
        self.ax_vel = self.fig.add_axes([0.74, 0.31, 0.24, 0.24], facecolor=PANEL_BG)
        self._status_panel = self.fig.text(0.75, 0.265, "", color=C_MUTED,
                                           fontsize=8, fontfamily="monospace", va="top")

        # Physical controls are deliberately visible and clickable. Keyboard
        # shortcuts invoke the same command path for users at the bench.
        self._ui_mode_buttons = {}
        mode_specs = [("SABER", "l", C_GREEN), ("TEXT", "t", "#ffad45"),
                      ("SLIDE", "s", C_BLUE), ("MARBLES", "m", "#e879f9")]
        x0, gap, width = 0.025, 0.008, 0.115
        for idx, (label, cmd, color) in enumerate(mode_specs):
            bax = self.fig.add_axes([x0 + idx * (width + gap), 0.055, width, 0.055])
            btn = Button(bax, label, color=PANEL_BG, hovercolor="#35434d")
            btn.label.set_color(color)
            btn.label.set_fontsize(8)
            btn.on_clicked(lambda _evt, key=cmd: self._dispatch_ui_command(key))
            self._ui_mode_buttons[cmd] = (btn, color)
        self._ui_action_buttons = {}
        actions = [("Rebound", "b"), ("Cavity −", "-"), ("Cavity +", "+"),
                   ("Gain", "g")]
        start_x = 0.53
        action_width = 0.105
        for idx, (label, cmd) in enumerate(actions):
            bax = self.fig.add_axes([start_x + idx * (action_width + 0.008),
                                     0.055, action_width, 0.055])
            btn = Button(bax, label, color=PANEL_BG, hovercolor="#35434d")
            btn.label.set_color(C_TEXT)
            btn.label.set_fontsize(8)
            btn.on_clicked(lambda _evt, key=cmd: self._dispatch_ui_command(key))
            self._ui_action_buttons[cmd] = btn
        self.fig.text(0.025, 0.025, "R / T / S / M mode    B rebound    − / + cavity    G gain    Q quit",
                      color=C_MUTED, fontsize=7.5, va="bottom")

        ax_chk = self.fig.add_axes([0.75, 0.205, 0.20, 0.04], facecolor=PANEL_BG)
        for sp in ax_chk.spines.values():
            sp.set_visible(False)
        ax_chk.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

        if _AUDIO_OK and self._stream is not None:
            self._chk = CheckButtons(
                ax_chk,
                ["♪  Approx preview"],
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
                "♪  preview unavailable",
                color=C_MUTED,
                fontsize=9,
                fontfamily="monospace",
                va="center",
                transform=ax_chk.transAxes,
            )

        self.fig.canvas.mpl_connect("close_event", self._on_close)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self._render_cache_mode = None

    def _dispatch_ui_command(self, key):
        """Send a UI action through the same path as its keyboard shortcut."""
        self._send_cmd(key.encode("ascii"))

    def _mode_color(self, mode=None):
        return {
            "LIGHTSABER": C_GREEN,
            "TEXTURED": "#ffad45",
            "SLIDING": C_BLUE,
            "MARBLES": "#e879f9",
        }.get(mode or self.mode, C_MUTED)

    def _update_header(self):
        mode = self.mode.upper()
        color = self._mode_color(mode)
        self._mode_header.set_text(f"●  {mode}")
        self._mode_header.set_color(color)
        state = getattr(self, "connection_state", None)
        detail = getattr(self, "connection_detail", "")
        if state is None:
            state = "connected" if self._connected else "connecting"
        labels = {"connecting": "Connecting", "connected": "Connected",
                  "stale": "Telemetry stale", "reconnecting": "Reconnecting",
                  "stopped": "Stopped"}
        state_color = C_GREEN if state == "connected" else (C_RED if state == "stale" else C_MUTED)
        label = labels.get(state, str(state).capitalize())
        if detail:
            label = f"{label} · {detail}"
        age = getattr(self, "telemetry_age_s", None)
        if state == "connected" and age is not None:
            label += f" · {age * 1000:.0f} ms"
        self._connection_header.set_text(label)
        self._connection_header.set_color(state_color)
        feedback = getattr(self, "last_command_feedback", "") or ""
        self._feedback_header.set_text(feedback)
        rebound = getattr(self, "rebound", None)
        rebound_label = "—" if rebound is None else ("ON" if bool(rebound) else "OFF")
        gain = "—" if self.haptic_gain is None else f"{self.haptic_gain:.2f}"
        rate = "—" if self.phys_hz <= 0 else f"{self.phys_hz}/{self.phys_target_hz or '—'} Hz"
        imu = f"{self.imu_hz} Hz" if self.imu_hz > 0 else "—"
        self._status_panel.set_text(
            f"physics   {rate}\nIMU       {imu}\nhaptic    {gain}\nrebound   {rebound_label}"
        )
        for cmd, (button, button_color) in self._ui_mode_buttons.items():
            active = {"l": "LIGHTSABER", "t": "TEXTURED", "s": "SLIDING", "m": "MARBLES"}[cmd] == mode
            button.ax.set_facecolor("#303b43" if active else PANEL_BG)
            button.label.set_fontweight("bold" if active else "normal")
            button.label.set_color(button_color)
        rebound_button = self._ui_action_buttons.get("b")
        if rebound_button is not None:
            rebound_button.label.set_text(f"Rebound {rebound_label}")

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
        self._stop_event.set()
        self._set_transport_state("stopped", "Window closed", False)
        self._drop_commands("window closed")
        loop = self._ble_loop
        client = self._ble_client
        if client is not None and loop is not None and loop.is_running():
            try:
                asyncio.run_coroutine_threadsafe(client.disconnect(), loop)
            except Exception:
                pass
        if self._transport_thread is not None and self._transport_thread is not threading.current_thread():
            self._transport_thread.join(timeout=0.8)
        if self._stream:
            try:
                self._stream.stop()
            finally:
                self._stream.close()

    def _on_key(self, event):
        cmd = {
            "r": b"t",
            "l": b"l",
            "t": b"t",
            "s": b"s",
            "m": b"m",
            "b": b"b",
            "+": b"+",
            "=": b"+",
            "-": b"-",
            "g": b"g",
        }.get(event.key)
        if cmd:
            self._send_cmd(cmd)
        elif event.key == "q":
            plt.close("all")
            sys.exit(0)

    # ------------------------------------------------------------------
    def _ensure_2d_axis(self):
        """Ensure main axis is 2D for tube mode."""
        if hasattr(self.ax, "zaxis"):
            self.ax.remove()
            self.ax = self.fig.add_axes([0.02, 0.18, 0.68, 0.74], facecolor=PANEL_BG)
            self._render_cache_mode = None

    def _ensure_3d_axis(self):
        """Ensure main axis is 3D for marble mode."""
        if not hasattr(self.ax, "zaxis"):
            self.ax.remove()
            self.ax = self.fig.add_axes(
                [0.02, 0.18, 0.68, 0.74],
                projection="3d",
                facecolor=PANEL_BG,
            )
            self._render_cache_mode = None

    # ------------------------------------------------------------------
    def _draw_frame(self):
        self._update_header()
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
        hl, thick, cap_w = 1.45, 0.20, 0.09
        angle = -math.asin(max(-1.0, min(1.0, self.sin_alpha)))
        ca, sa = math.cos(angle), math.sin(angle)
        px, py = -sa, ca

        def pt(along, across):
            return (along * ca + across * px, along * sa + across * py)

        cache = getattr(self, "_tube_cache", None)
        if cache is None or cache["axis"] is not ax:
            ax.set_facecolor(PANEL_BG)
            ax.set_xlim(-2.05, 2.05)
            ax.set_ylim(-1.40, 1.40)
            ax.set_aspect("equal", adjustable="box")
            ax.axis("off")
            body = mpatches.Polygon(np.empty((0, 2)), closed=True, facecolor=C_TUBE,
                                    edgecolor=C_MUTED, linewidth=1.2,
                                    alpha=0.85, zorder=2)
            trim = mpatches.Polygon(np.empty((0, 2)), closed=True, facecolor="#4a6a80",
                                    alpha=0.5, zorder=3)
            caps = [mpatches.Polygon(np.empty((0, 2)), closed=True, edgecolor=C_MUTED,
                                     linewidth=1.0, alpha=0.95, zorder=3)
                    for _ in range(2)]
            ball = mpatches.Circle((0, 0), thick * 0.78,
                                   edgecolor="#ffffffaa", linewidth=1.0, zorder=5)
            shine = mpatches.Circle((0, 0), thick * 0.78 * 0.28,
                                     facecolor="#ffffffaa", zorder=6)
            for art in [body, trim, *caps, ball, shine]:
                ax.add_patch(art)
            ax.plot([-0.5, 0.5], [0, 0], color="#33445555", linewidth=0.8,
                    linestyle="--", zorder=1)
            arc, = ax.plot([], [], color="#ffcc44", linewidth=1.8,
                           alpha=0.8, zorder=1)
            tilt_text = ax.text(0, 0, "", color="#ffcc44", fontsize=9,
                                ha="center", va="center", fontfamily="monospace")
            ax.annotate("", xy=(1.75, -1.05), xytext=(1.75, -0.65),
                        arrowprops=dict(arrowstyle="-|>", color="#667788",
                                        lw=1.5, mutation_scale=9), zorder=1)
            ax.text(1.75, -0.60, "g", color="#667788", fontsize=9,
                    ha="center", va="bottom", fontfamily="monospace")
            cache = {"axis": ax, "body": body, "trim": trim, "caps": caps,
                     "ball": ball, "shine": shine, "arc": arc,
                     "tilt_text": tilt_text}
            self._tube_cache = cache

        cache["body"].set_xy([pt(-hl, thick), pt(hl, thick),
                              pt(hl, -thick), pt(-hl, -thick)])
        cache["trim"].set_xy([pt(-hl, thick), pt(hl, thick),
                              pt(hl, thick - 0.03), pt(-hl, thick - 0.03)])
        lf, rf = self._flash_left > 0, self._flash_right > 0
        for idx, (side, flash) in enumerate(((-1, lf), (1, rf))):
            base = side * hl
            cache["caps"][idx].set_xy([pt(base, thick + 0.02),
                                       pt(base + side * cap_w, thick + 0.02),
                                       pt(base + side * cap_w, -thick - 0.02),
                                       pt(base, -thick - 0.02)])
            cache["caps"][idx].set_facecolor(C_RED if flash else C_WALL)
        if self._flash_left > 0:
            self._flash_left -= 1
        if self._flash_right > 0:
            self._flash_right -= 1

        display_x = self._display_position()
        t_ball = (display_x / max(self.cavity_mm, 1.0) - 0.5) * 2.0 * hl
        bx, by = pt(t_ball, 0)
        speed_ratio = min(abs(self.v_mps) / 1.5, 1.0)
        ball = cache["ball"]
        ball.center = (bx, by)
        ball.set_facecolor((speed_ratio * 0.94, 0.55 + 0.30 * (1.0 - speed_ratio), 0.15))
        cache["shine"].center = (bx + thick * 0.78 * 0.28 * (-ca + px) * 0.6,
                                 by + thick * 0.78 * 0.28 * (-sa + py) * 0.6)
        if abs(angle) > 0.02:
            arc_r = 0.55
            theta = np.linspace(0, angle, 48)
            cache["arc"].set_data(arc_r * np.cos(theta), arc_r * np.sin(theta))
            mid = angle / 2
            cache["tilt_text"].set_position((arc_r * 1.18 * math.cos(mid),
                                             arc_r * 1.18 * math.sin(mid)))
            cache["tilt_text"].set_text(f"{math.degrees(angle):+.1f}°")
        else:
            cache["arc"].set_data([], [])
            cache["tilt_text"].set_text("")

        gain = "—" if self.haptic_gain is None else f"{self.haptic_gain:.2f}"
        telemetry = getattr(self, "telemetry_age_s", None)
        age_text = "waiting" if telemetry is None else f"{telemetry * 1000:.0f} ms"
        self.info.set_text(
            f"position {display_x:.1f} mm   velocity {self.v_mps:+.3f} m/s   tilt {math.degrees(angle):+.1f}°\n"
            f"cavity {self.cavity_mm:.0f} mm   haptic {gain}   telemetry {age_text}"
        )

    def _draw_marble_box(self):
        ax = self.ax
        W, H, D = _BOX_W_MM, _BOX_H_MM, _BOX_D_MM
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

        cache = getattr(self, "_marble_cache", None)
        if cache is None or cache["axis"] is not ax:
            ax.set_facecolor(PANEL_BG)
            ax.grid(False)
            radius = math.sqrt(W * W + H * H + D * D) / 2 + 12
            ax.set_xlim(center[0] - radius, center[0] + radius)
            ax.set_ylim(center[1] - radius, center[1] + radius)
            # Firmware/world +Z points down; Matplotlib's default +Z is up.
            ax.set_zlim(center[2] + radius, center[2] - radius)
            ax.set_box_aspect([1, 1, 1])
            ax.set_xlabel("X [mm]", color=C_MUTED, fontsize=8)
            ax.set_ylabel("Y [mm]", color=C_MUTED, fontsize=8)
            ax.set_zlabel("Z [mm]", color=C_MUTED, fontsize=8)
            ax.tick_params(colors=C_MUTED, labelsize=7)
            ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
            ax.xaxis.pane.set_edgecolor("#333")
            ax.yaxis.pane.set_edgecolor("#333")
            ax.zaxis.pane.set_edgecolor("#333")
            ax.view_init(elev=20, azim=45)
            box_artist = Poly3DCollection([], alpha=0.15, facecolor=C_TUBE,
                                          edgecolor=C_MUTED, linewidths=1.5)
            flash_artist = Poly3DCollection([], alpha=0.0, facecolor=C_RED,
                                            edgecolor="none")
            ax.add_collection3d(box_artist)
            ax.add_collection3d(flash_artist)
            u = np.linspace(0, 2 * np.pi, 13)
            v = np.linspace(0, np.pi, 9)
            ux = np.outer(np.cos(u), np.sin(v))
            uy = np.outer(np.sin(u), np.sin(v))
            uz = np.outer(np.ones(np.size(u)), np.cos(v))
            sphere_collections = []
            for idx in range(_MARBLE_COUNT):
                mc = _MARBLE_COLORS[idx] if idx < len(_MARBLE_COLORS) else C_BLUE
                coll = Poly3DCollection([], facecolor=mc, alpha=0.92,
                                        edgecolor="#ffffff88", linewidths=0.25)
                ax.add_collection3d(coll)
                sphere_collections.append(coll)
            gravity_line, = ax.plot([], [], [], color="#ffcc44", linewidth=2.2,
                                     marker="v", markersize=5)
            gravity_text = ax.text(0, 0, 0, "g", color="#ffcc44", fontsize=10,
                                   fontfamily="monospace")
            cache = {"axis": ax, "box": box_artist, "flash": flash_artist,
                     "spheres": sphere_collections, "sphere_grid": (ux, uy, uz),
                     "gravity_line": gravity_line, "gravity_text": gravity_text}
            self._marble_cache = cache

        cache["box"].set_verts(faces)
        flash_alpha = self._marble_impact_flash / 8.0 * 0.5
        cache["flash"].set_verts(faces)
        cache["flash"].set_alpha(flash_alpha)
        if self._marble_impact_flash > 0:
            self._marble_impact_flash -= 1

        ux, uy, uz = cache["sphere_grid"]
        for idx, (mx, my, mz) in enumerate(self.marbles):
            mr = self.marble_radii[idx] if idx < len(self.marble_radii) else _MARBLE_RAD_MM
            points = np.stack((ux * mr + mx, uy * mr + my, uz * mr + mz), axis=-1)
            flat = points.reshape(-1, 3)
            rotated = _quat_rotate(flat - center, qw, qx, qy, qz) + center
            points = rotated.reshape(points.shape)
            quads = [
                [points[i, j], points[i + 1, j], points[i + 1, j + 1], points[i, j + 1]]
                for i in range(points.shape[0] - 1)
                for j in range(points.shape[1] - 1)
            ]
            cache["spheres"][idx].set_verts(quads)

        # Gravity reference (updated in place; no per-frame collection churn).
        arrow_len = 25.0
        base = np.array([W / 2, H / 2, 0], dtype=np.float32)
        g_vec = np.array(
            [self.grav_x * arrow_len, self.grav_y * arrow_len, self.grav_z * arrow_len],
            dtype=np.float32,
        )
        base = _quat_rotate(base - center, qw, qx, qy, qz) + center
        g_vec = _quat_rotate(g_vec, qw, qx, qy, qz)
        tip = base + g_vec
        cache["gravity_line"].set_data_3d([base[0], tip[0]], [base[1], tip[1]],
                                          [base[2], tip[2]])
        cache["gravity_text"].set_position((tip[0], tip[1]))
        cache["gravity_text"].set_3d_properties(tip[2])

        marble_info = "  marbles  "
        labels = ["heavy", "medium", "light"]
        for idx in range(_MARBLE_COUNT):
            r = self.marble_radii[idx] if idx < len(self.marble_radii) else _MARBLE_RAD_MM
            lbl = labels[idx] if idx < len(labels) else f"#{idx}"
            marble_info += f"{lbl}({r:.1f}mm) " if idx < _MARBLE_COUNT - 1 else f"{lbl}({r:.1f}mm)"
        gain = f"{self.haptic_gain:.2f}" if self.haptic_gain is not None else "—"
        imu_line = f"IMU {self.imu_hz} Hz" if self.imu_hz > 0 else "IMU waiting"
        self.info.set_text(
            f"MARBLES · box {_BOX_W_MM:.0f} × {_BOX_H_MM:.0f} × {_BOX_D_MM:.0f} mm\n"
            f"{marble_info.strip()}\n"
            f"accel {self.grav_x:+.2f}, {self.grav_y:+.2f}, {self.grav_z:+.2f} g\n"
            f"impact {self.impact_energy:.2f} · haptic {gain} · {imu_line}"
        )

    # ------------------------------------------------------------------
    def _draw_history(self):
        self._draw_history_lines("tube")

    def _draw_marble_history(self):
        self._draw_history_lines("marbles")

    def _draw_history_lines(self, view):
        """Update history traces in place; axes are rebuilt only on mode changes."""
        t = np.linspace(-HISTORY / 58.0, 0.0, HISTORY)
        cache = getattr(self, "_history_cache", None)
        if (cache is None or cache["view"] != view or
                cache["pos_ax"] is not self.ax_pos or cache["vel_ax"] is not self.ax_vel):
            axp, axv = self.ax_pos, self.ax_vel
            axp.cla()
            axv.cla()
            for ax in (axp, axv):
                ax.set_facecolor(PANEL_BG)
                ax.tick_params(colors=C_MUTED, labelsize=7)
                for sp in ax.spines.values():
                    sp.set_edgecolor("#333")
            pos_line, = axp.plot(t, [0] * HISTORY, color=C_BLUE, linewidth=1.2)
            pos_refs = [axp.axhline(0, color="#334", linewidth=0.6),
                        axp.axhline(0, color="#334", linewidth=0.6)]
            vel_line, = axv.plot(t, [0] * HISTORY, color=C_RED, linewidth=1.2)
            zero = axv.axhline(0, color="#445", linewidth=0.8, linestyle="--")
            boundary = axv.axhline(0, color="#445", linewidth=0.6)
            axp.set_xlim(t[0], 0)
            axv.set_xlim(t[0], 0)
            axp.set_title("History", color=C_TEXT, fontsize=9, pad=4)
            axp.xaxis.set_ticklabels([])
            axv.set_xlabel("time [s]", color=C_MUTED, fontsize=7)
            cache = {"view": view, "pos_ax": axp, "vel_ax": axv,
                     "pos_line": pos_line, "pos_refs": pos_refs,
                     "vel_line": vel_line, "zero": zero, "boundary": boundary}
            self._history_cache = cache

        pos = list(self._x_hist)
        vel = list(self._v_hist)
        cache["pos_line"].set_data(t, pos)
        cache["vel_line"].set_data(t, vel)
        if view == "tube":
            cache["pos_refs"][0].set_ydata([0, 0])
            cache["pos_refs"][1].set_ydata([self.cavity_mm, self.cavity_mm])
            self.ax_pos.set_ylim(-30, self.cavity_mm + 30)
            self.ax_pos.set_ylabel("position [mm]", color=C_MUTED, fontsize=7)
            self.ax_vel.set_ylabel("velocity [m/s]", color=C_MUTED, fontsize=7)
            cache["boundary"].set_visible(False)
            vmax = max(max(map(abs, vel), default=0), 0.1)
            self.ax_vel.set_ylim(-vmax * 1.15, vmax * 1.15)
        else:
            cache["pos_refs"][0].set_ydata([0, 0])
            cache["pos_refs"][1].set_ydata([_BOX_W_MM, _BOX_W_MM])
            self.ax_pos.set_ylim(-5, _BOX_W_MM + 5)
            self.ax_pos.set_ylabel("marble 0  x [mm]", color=C_MUTED, fontsize=7)
            self.ax_vel.set_ylabel("marble 0  y [mm]", color=C_MUTED, fontsize=7)
            cache["boundary"].set_visible(True)
            cache["boundary"].set_ydata([_BOX_H_MM, _BOX_H_MM])
            self.ax_vel.set_ylim(-5, _BOX_H_MM + 5)

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
            interval=16,
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
            "omit → prefer a connected Espressif USB board, otherwise scan BLE"
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
    parser.add_argument("--ble", action="store_true",
                        help="Use Bluetooth even when a USB board is connected")
    args = parser.parse_args()

    if args.list:
        asyncio.run(_list_all())
        sys.exit(0)

    if args.ble and args.target and (args.target.upper().startswith("COM") or args.target.startswith("/dev/")):
        parser.error("--ble cannot be combined with a USB serial port")
    target = args.target
    if target is None and not args.ble:
        boards = [p for p in serial.tools.list_ports.comports()
                  if p.vid == 0x303A and p.pid == 0x1001]
        if len(boards) == 1:
            target = boards[0].device
            print(f"Using Rolling Stone USB board on {target}")
        elif len(boards) > 1:
            parser.error("Multiple Espressif boards found; choose a port explicitly: "
                         + ", ".join(p.device for p in boards))
    try:
        Visualizer(target, args.baud).run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
