#!/usr/bin/env python3
"""
BNO085 Orientation Visualizer
Reads quaternion data from the Feather ESP32-C6 over serial and renders a
live 3D view of sensor orientation along with roll/pitch/yaw readout and
a compass rose showing magnetic north.

Usage:
    python visualizer.py COM3          # Windows
    python visualizer.py /dev/ttyACM0  # Linux / macOS
    python visualizer.py --list        # list available ports
"""

import argparse
import math
import sys
import time

import numpy as np
import serial
import serial.tools.list_ports
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401 (registers projection)
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# ---------------------------------------------------------------------------
# Quaternion math
# ---------------------------------------------------------------------------

def quat_to_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Unit quaternion (w, x, y, z) → 3×3 rotation matrix."""
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),       2*(x*z + w*y)],
        [2*(x*y + w*z),        1 - 2*(x*x + z*z),   2*(y*z - w*x)],
        [2*(x*z - w*y),        2*(y*z + w*x),        1 - 2*(x*x + y*y)],
    ])


def quat_to_euler_deg(w: float, x: float, y: float, z: float):
    """Quaternion → (roll, pitch, yaw) in degrees (ZYX / aerospace convention)."""
    roll  = np.degrees(np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
    pitch = np.degrees(np.arcsin(np.clip(2*(w*y - z*x), -1.0, 1.0)))
    yaw   = np.degrees(np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
    return roll, pitch, yaw


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def box_faces(R: np.ndarray, size=(2.0, 1.0, 0.4)):
    """Return the 6 faces of a box (default PCB-ish shape) rotated by R."""
    sx, sy, sz = (s / 2 for s in size)
    corners = np.array([
        [-sx, -sy, -sz], [ sx, -sy, -sz], [ sx,  sy, -sz], [-sx,  sy, -sz],
        [-sx, -sy,  sz], [ sx, -sy,  sz], [ sx,  sy,  sz], [-sx,  sy,  sz],
    ])
    c = (R @ corners.T).T
    return [
        [c[0], c[1], c[2], c[3]],  # -Z face
        [c[4], c[5], c[6], c[7]],  # +Z face
        [c[0], c[1], c[5], c[4]],  # -Y face
        [c[3], c[2], c[6], c[7]],  # +Y face
        [c[0], c[3], c[7], c[4]],  # -X face
        [c[1], c[2], c[6], c[5]],  # +X face
    ]


# ---------------------------------------------------------------------------
# Colours
# ---------------------------------------------------------------------------

DARK_BG      = "#1a1a2e"
PANEL_BG     = "#16213e"
INFO_BG      = "#0f3460"
C_RED        = "#e94560"
C_GREEN      = "#0f9b58"
C_BLUE       = "#4f8ef7"
C_TEXT       = "#dce3f0"
C_MUTED      = "#6a7490"
C_NORTH      = "#e8e8ff"   # magnetic-north arrow colour


def _accuracy_colour(acc_rad: float) -> tuple[str, str]:
    """Return (cone_fill, label) colour based on heading accuracy in radians."""
    if math.isnan(acc_rad) or acc_rad > 1.0:
        return "#e94560", "poor"       # red  — >57°
    if acc_rad > 0.35:
        return "#ff8c00", "medium"     # orange — 20–57°
    return "#00c8a0", "good"           # teal/green — <20°


# ---------------------------------------------------------------------------
# Visualizer
# ---------------------------------------------------------------------------

class Visualizer:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.02)

        # Current state
        self.quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
        self.accuracy_rad: float = float("nan")

        # Frequency counter (1-second sliding window)
        self._pkt_count: int   = 0
        self._hz_t0: float     = time.monotonic()
        self._hz: float        = 0.0

        self._build_figure()

    # ------------------------------------------------------------------
    def _build_figure(self):
        matplotlib.rcParams["toolbar"] = "None"
        self.fig = plt.figure(figsize=(13, 8), facecolor=DARK_BG)

        # Main 3D axes — leave the right ~25 % for the compass rose
        self.ax = self.fig.add_axes([0.00, 0.00, 0.74, 1.00],
                                    projection="3d", facecolor=PANEL_BG)
        for spine in self.ax.spines.values():
            spine.set_visible(False)

        self.fig.suptitle(
            "BNO085  ·  ESP32-C6  ·  Rotation Vector (accel + gyro + mag)",
            color=C_TEXT, fontsize=13, y=0.97,
        )

        # Info text overlay (bottom-left)
        self.info = self.fig.text(
            0.01, 0.02, "",
            color=C_TEXT, fontsize=10, fontfamily="monospace",
            verticalalignment="bottom",
            bbox=dict(facecolor=INFO_BG, alpha=0.85, edgecolor="none", pad=7),
        )

        # Compass rose inset (top-right panel)
        self.ax_compass = self.fig.add_axes(
            [0.74, 0.30, 0.24, 0.55],
            facecolor=PANEL_BG,
        )
        self.ax_compass.set_aspect("equal")
        self.ax_compass.axis("off")

        self.fig.canvas.mpl_connect("close_event", lambda _: self.ser.close())

    # ------------------------------------------------------------------
    def _drain_serial(self):
        """Consume all waiting bytes; keep the most recent valid quaternion."""
        try:
            while self.ser.in_waiting:
                raw = self.ser.readline()
                line = raw.decode("ascii", errors="ignore").strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split(",")
                if len(parts) != 5:
                    continue
                w, x, y, z, acc = map(float, parts)
                norm = np.sqrt(w*w + x*x + y*y + z*z)
                if norm < 0.5:           # ignore obviously bad packets
                    continue
                self.quat = (w / norm, x / norm, y / norm, z / norm)
                self.accuracy_rad = acc
                self._pkt_count += 1
        except (ValueError, UnicodeDecodeError, serial.SerialException):
            pass

    # ------------------------------------------------------------------
    def _draw_frame(self):
        ax = self.ax
        w, x, y, z = self.quat
        R = quat_to_matrix(w, x, y, z)

        ax.cla()
        lim = 1.6
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
        ax.set_facecolor(PANEL_BG)
        ax.set_xlabel("X", color=C_MUTED, labelpad=2)
        ax.set_ylabel("Y", color=C_MUTED, labelpad=2)
        ax.set_zlabel("Z", color=C_MUTED, labelpad=2)
        ax.tick_params(colors=C_MUTED, labelsize=7)
        ax.set_box_aspect((1, 1, 1))

        # Faint world-axis reference lines
        for v, c in [(np.array([1,0,0]), "#2a1818"),
                     (np.array([0,1,0]), "#182a18"),
                     (np.array([0,0,1]), "#18182a")]:
            ax.plot(*zip(-lim*v, lim*v), color=c, linewidth=0.5, zorder=0)

        # ── Magnetic north — fixed world +X arrow ────────────────────────────
        # BNO085 SH2_ROTATION_VECTOR defines world +X as magnetic north.
        # This arrow never moves; the board rotates around it.
        north = np.array([1.0, 0.0, 0.0])
        ax.quiver(0, 0, 0, *north,
                  color=C_NORTH, linewidth=2.5,
                  arrow_length_ratio=0.15, zorder=7,
                  alpha=0.9)
        ax.text(*(north * 1.22), "N", color=C_NORTH,
                fontsize=13, fontweight="bold", zorder=8)
        # faint south tail
        ax.quiver(0, 0, 0, *(-north * 0.6),
                  color=C_NORTH, linewidth=1.0,
                  arrow_length_ratio=0.0, zorder=7, alpha=0.35)

        # ── Body-frame axes (thick arrows) ────────────────────────────────────
        origin = np.zeros(3)
        for col_vec, color, label in [
            (R[:, 0], C_RED,   "X"),
            (R[:, 1], C_GREEN, "Y"),
            (R[:, 2], C_BLUE,  "Z"),
        ]:
            ax.quiver(*origin, *col_vec,
                      color=color, linewidth=3,
                      arrow_length_ratio=0.18, zorder=5)
            ax.text(*(col_vec * 1.18), label,
                    color=color, fontsize=12, fontweight="bold", zorder=6)

        # ── Board representation (coloured box) ───────────────────────────────
        faces = box_faces(R)
        face_colors = [C_RED, "#b03050", "#c03555", "#c03555", "#a02545", "#a02545"]
        poly = Poly3DCollection(faces,
                                facecolors=face_colors,
                                edgecolors="#ff9aaf",
                                linewidths=0.6,
                                alpha=0.50,
                                zorder=3)
        ax.add_collection3d(poly)

        # ── Heading indicator — project sensor X-axis onto XY plane ──────────
        # Shows where the board's X axis points on the horizontal plane.
        fwd = R[:, 0].copy(); fwd[2] = 0
        fn = np.linalg.norm(fwd)
        if fn > 0.05:
            fwd /= fn
            ax.quiver(*origin, *(fwd * 1.4),
                      color="#ffcc44", linewidth=1.5,
                      arrow_length_ratio=0.12, linestyle="dashed",
                      alpha=0.6, zorder=4)

        # ── Info overlay ──────────────────────────────────────────────────────
        roll, pitch, yaw = quat_to_euler_deg(w, x, y, z)
        _, cal_label = _accuracy_colour(self.accuracy_rad)
        if math.isnan(self.accuracy_rad):
            acc_str = "  Heading accuracy  ---"
        else:
            acc_deg = math.degrees(self.accuracy_rad)
            acc_str = f"  Heading accuracy  ±{acc_deg:.1f}°  [{cal_label}]"
        self.info.set_text(
            f"  Quat  w={w:+.4f}  x={x:+.4f}  y={y:+.4f}  z={z:+.4f}\n"
            f"  Roll  {roll:+7.2f}°   Pitch {pitch:+7.2f}°   Yaw {yaw:+7.2f}°\n"
            f"{acc_str}\n"
            f"  Rate  {self._hz:.0f} Hz  "
        )

        # ── Compass rose ──────────────────────────────────────────────────────
        self._draw_compass(yaw)

    # ------------------------------------------------------------------
    def _draw_compass(self, yaw_deg: float):
        """
        Compass rose: fixed ring with N/S/E/W labels.
        The white arrow always points to magnetic north (top of dial).
        The red needle shows the sensor's current heading (body +X projected
        to horizontal). The shaded cone shows heading accuracy.
        """
        ax = self.ax_compass
        ax.cla()
        ax.set_facecolor(PANEL_BG)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.7, 1.7)

        # ── Outer ring ───────────────────────────────────────────────────────
        theta = np.linspace(0, 2 * math.pi, 256)
        ax.plot(np.cos(theta), np.sin(theta), color=C_MUTED, linewidth=1.2, zorder=1)

        # Tick marks every 10°, longer every 45°
        for i in range(36):
            angle = math.radians(i * 10)
            r_inner = 0.82 if i % 9 == 0 else 0.90
            lw = 1.2 if i % 9 == 0 else 0.5
            ax.plot([r_inner * math.sin(angle), math.sin(angle)],
                    [r_inner * math.cos(angle), math.cos(angle)],
                    color=C_MUTED, linewidth=lw, zorder=1)

        # Cardinal labels  (N top, E right — standard compass layout)
        for label, ang_deg in [("N", 0), ("E", 90), ("S", 180), ("W", 270)]:
            a = math.radians(ang_deg)
            lx, ly = 1.28 * math.sin(a), 1.28 * math.cos(a)
            color  = C_NORTH if label == "N" else C_MUTED
            weight = "bold"  if label == "N" else "normal"
            ax.text(lx, ly, label, color=color, fontsize=11,
                    ha="center", va="center",
                    fontweight=weight, fontfamily="monospace", zorder=3)

        # ── Accuracy cone (shaded wedge around heading needle) ────────────────
        cone_color, cal_label = _accuracy_colour(self.accuracy_rad)
        if not math.isnan(self.accuracy_rad):
            half = min(self.accuracy_rad, math.pi)
            needle_ang = math.radians(yaw_deg)
            mid_math   = math.pi / 2 - needle_ang
            cone_t = np.linspace(mid_math - half, mid_math + half, 80)
            cone_x = np.concatenate([[0], np.cos(cone_t)])
            cone_y = np.concatenate([[0], np.sin(cone_t)])
            ax.fill(cone_x, cone_y, color=cone_color, alpha=0.20, zorder=2)
            for edge in [mid_math - half, mid_math + half]:
                ax.plot([0, math.cos(edge)], [0, math.sin(edge)],
                        color=cone_color, linewidth=0.6, alpha=0.5, zorder=2)

        # ── Magnetic north marker — fixed white arrow pointing up ─────────────
        # This represents the world: north is always at the top.
        ax.annotate(
            "", xy=(0, 0.88), xytext=(0, 0),
            arrowprops=dict(arrowstyle="-|>", color=C_NORTH,
                            lw=2.0, mutation_scale=12),
            zorder=5,
        )
        ax.plot([0, 0], [0, -0.55], color=C_NORTH, linewidth=1.5,
                alpha=0.5, zorder=5)

        # ── Heading needle (sensor body +X projected to horizontal) ───────────
        # Tip (red) points in the direction the board's X-axis faces.
        yaw_rad = math.radians(yaw_deg)
        nx =  math.sin(yaw_rad)   # east component
        ny =  math.cos(yaw_rad)   # north component
        # Red tip
        ax.annotate(
            "", xy=(nx * 0.82, ny * 0.82), xytext=(0, 0),
            arrowprops=dict(arrowstyle="-|>", color=C_RED,
                            lw=2.5, mutation_scale=14),
            zorder=6,
        )
        # White tail (opposite end)
        ax.plot([0, -nx * 0.55], [0, -ny * 0.55],
                color=C_TEXT, linewidth=2.5, zorder=6)

        # Centre pivot dot
        ax.plot(0, 0, "o", color=C_TEXT, markersize=5, zorder=7)

        # ── Labels ────────────────────────────────────────────────────────────
        ax.text(0, 1.58, "Magnetic North", color=C_NORTH,
                fontsize=8, ha="center", va="center",
                fontfamily="monospace", zorder=3)
        heading_norm = yaw_deg % 360
        ax.text(0, -1.55, f"{heading_norm:.1f}°",
                color=C_TEXT, fontsize=10, ha="center", va="center",
                fontfamily="monospace", fontweight="bold", zorder=3)

        # Small legend below the dial
        ax.text(-1.45, -1.68, "— board +X heading",
                color=C_RED, fontsize=7, va="bottom",
                fontfamily="monospace", zorder=3)
        ax.text(-1.45, -1.58, "— mag north",
                color=C_NORTH, fontsize=7, va="bottom",
                fontfamily="monospace", zorder=3)
        # Calibration quality label
        ax.text(1.45, -1.68, f"cal: {cal_label}",
                color=cone_color, fontsize=7, va="bottom", ha="right",
                fontfamily="monospace", zorder=3)

    # ------------------------------------------------------------------
    def _update(self, _frame):
        self._drain_serial()

        # Update Hz once per second
        now = time.monotonic()
        dt  = now - self._hz_t0
        if dt >= 1.0:
            self._hz     = self._pkt_count / dt
            self._pkt_count = 0
            self._hz_t0  = now

        self._draw_frame()
        return []

    def run(self):
        self._anim = animation.FuncAnimation(
            self.fig, self._update,
            interval=30,          # ~33 fps target
            blit=False,
            cache_frame_data=False,
        )
        plt.tight_layout(rect=[0, 0.08, 0.74, 0.96])
        plt.show()
        if self.ser.is_open:
            self.ser.close()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def list_ports():
    ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
    if not ports:
        print("  (no serial ports found)")
        return
    for p in ports:
        print(f"  {p.device:<12}  {p.description}")


def main():
    parser = argparse.ArgumentParser(
        description="Live 3-D orientation viewer for BNO085 + Feather ESP32-C6"
    )
    parser.add_argument("port",   nargs="?",  help="Serial port, e.g. COM3 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int,   default=115200, help="Baud rate (default 115200)")
    parser.add_argument("--list", action="store_true", help="List available serial ports and exit")
    args = parser.parse_args()

    if args.list or not args.port:
        print("Available serial ports:")
        list_ports()
        if not args.port:
            sys.exit(0)

    print(f"Opening {args.port} @ {args.baud} baud …")
    try:
        Visualizer(args.port, args.baud).run()
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
