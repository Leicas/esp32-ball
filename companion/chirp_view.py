"""
Live continuous-chirp viewer for the Rolling Stone A7 actuators.

Starts the a7_bringup firmware's 'C' stream (exponential chirp 20-500 Hz on each fitted
actuator, then both together, looping) and shows, live:

  top     scrolling spectrogram of the on-board IMU (all axes), with the drive frequency
          and its 2nd/3rd harmonics overlaid — rattles show as broadband smear, dropouts
          as gaps in the drive line
  middle  response at the drive frequency vs frequency, per mode; latest pass bold,
          previous passes faded, so an intermittent actuator is obvious pass-to-pass
  bottom  broadband vibration RMS vs time

Closing the window stops the stream and saves raw data + a snapshot PNG to
companion/sweeps/<timestamp>-chirp/.

Usage:
  .venv/Scripts/python companion/chirp_view.py COM6 --amp 0.10
  .venv/Scripts/python companion/chirp_view.py COM6 --seconds 25 --headless   # capture + PNG only
"""

import argparse
import datetime as dt
import os
import threading
import time

import numpy as np
import matplotlib

MODE_STYLE = {"1": ("#2f6fdf", "LFi (J3)"), "2": ("#8e44ad", "MF (J4)"), "3": ("#e0622c", "HF (J5)"),
              "4": ("#16a085", "LF (J6)")}
PAIR_COLOR = "#2a9d55"
NFFT, HOP = 128, 32
SPEC_SECONDS = 14.0


class Stream:
    """Serial reader: accumulates raw accel and chirp markers."""

    def __init__(self, port, amp):
        import serial

        self.ser = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(1.5)  # port open may reset the board
        self.ser.reset_input_buffer()
        for c in "-" * 6 + "+" * int(round(amp / 0.05)):
            self.ser.write(c.encode())
            time.sleep(0.02)
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.lock = threading.Lock()
        self.chunks, self.n = [], 0
        self.odr, self.mg = 1666.0, 0.244
        self.f0, self.f1, self.chirp_ms = 20.0, 500.0, 6000.0
        self.starts, self.ends = [], []  # (mode, idx)
        self.ts = []  # (count, us)
        self.log = []
        self.alive = True
        self.ser.write(b"C")
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def _run(self):
        pend = []
        while self.alive:
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if line.startswith("%"):
                try:
                    pend.append([int(v) for v in line[1:].split(",")])
                except ValueError:
                    pass
                if len(pend) >= 64:
                    self._flush(pend)
                    pend = []
                continue
            self._flush(pend)
            pend = []
            p = line.split(",")
            with self.lock:
                if line.startswith("$ci"):
                    self.odr, self.mg = float(p[1]), float(p[2])
                    self.f0, self.f1, self.chirp_ms = float(p[3]), float(p[4]), float(p[5])
                elif line.startswith("$cs"):
                    self.starts.append((p[1], int(p[2])))
                elif line.startswith("$ce"):
                    self.ends.append((p[1], int(p[2])))
                elif line.startswith("$ts"):
                    self.ts.append((int(p[1]), int(p[2])))
                    if len(self.ts) >= 4:
                        (c0, u0), (c1, u1) = self.ts[0], self.ts[-1]
                        if u1 > u0:
                            self.odr = (c1 - c0) / ((u1 - u0) * 1e-6)
                elif line.startswith("#"):
                    self.log.append(line)
                    if "=====" in line:
                        print("board reset —", line)

    def _flush(self, pend):
        if not pend:
            return
        a = np.asarray(pend, dtype=np.int16)
        with self.lock:
            self.chunks.append(a)
            self.n += len(a)

    def data(self):
        with self.lock:
            if len(self.chunks) > 1:
                self.chunks = [np.concatenate(self.chunks)]
            x = self.chunks[0] if self.chunks else np.zeros((0, 3), np.int16)
            return x, list(self.starts), list(self.ends), self.odr

    def stop(self):
        try:
            self.ser.write(b"x")
            time.sleep(0.3)
        finally:
            self.alive = False
            self.th.join(timeout=1)
            self.ser.close()


def stft_power(x_mg, odr):
    """Summed-over-axes STFT power and a sinusoid-amplitude calibration factor."""
    n = (len(x_mg) - NFFT) // HOP + 1
    if n <= 0:
        return None
    w = np.hanning(NFFT)
    idx = np.arange(NFFT)[None, :] + HOP * np.arange(n)[:, None]
    P = np.zeros((n, NFFT // 2 + 1))
    for ax in range(3):
        seg = x_mg[idx, ax]
        seg = seg - seg.mean(axis=1, keepdims=True)
        P += np.abs(np.fft.rfft(seg * w, axis=1)) ** 2
    centres = HOP * np.arange(n) + NFFT // 2
    freqs = np.fft.rfftfreq(NFFT, 1 / odr)
    cal = NFFT * np.sum(w ** 2)  # sinusoid of amplitude A: sum over its lobe of |X|^2 = A^2/4 * cal
    return P, centres, freqs, cal


def chirp_freq(idx, s, e, f0, f1, lat):
    t = (idx - s - lat) / max(e - s, 1)
    f = f0 * (f1 / f0) ** np.clip(t, 0, 1)
    return np.where((t >= 0) & (t <= 1), f, np.nan)


def response(P, centres, freqs, cal, s, e, f0, f1, lat):
    """Amplitude (mg peak) at the drive frequency for each STFT frame inside chirp [s, e]."""
    m = (centres >= s + lat) & (centres <= e + lat)
    if not m.any():
        return np.array([]), np.array([])
    fd = chirp_freq(centres[m], s, e, f0, f1, lat)
    amp = np.empty(len(fd))
    df = freqs[1]
    for i, (row, f) in enumerate(zip(P[m], fd)):
        band = (freqs >= f * 0.85 - df) & (freqs <= f * 1.15 + df) & (freqs > 0)
        amp[i] = 2 * np.sqrt(row[band].sum() / cal)
    return fd, amp


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port")
    ap.add_argument("--amp", type=float, default=0.10, help="drive per channel, FS (0.05-0.25)")
    ap.add_argument("--lat-ms", type=float, default=35.0, help="audio output latency (DMA queue)")
    ap.add_argument("--seconds", type=float, help="stop after this long")
    ap.add_argument("--headless", action="store_true", help="no window; save PNG at the end")
    ap.add_argument("--out", help="output directory")
    args = ap.parse_args()
    if args.headless:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "sweeps",
                                   dt.datetime.now().strftime("%Y%m%d-%H%M%S") + f"-chirp-amp{args.amp:.2f}")
    st = Stream(args.port, args.amp)
    t_start = time.time()

    fig = plt.figure(figsize=(13, 9))
    gs = fig.add_gridspec(3, 1, height_ratios=[2.2, 2, 1])
    ax_s, ax_r, ax_t = fig.add_subplot(gs[0]), fig.add_subplot(gs[1]), fig.add_subplot(gs[2])
    fig.suptitle(f"A7 continuous chirp — drive {args.amp:.2f} FS — close window to stop & save")

    def draw():
        x, starts, ends, odr = st.data()
        if len(x) < NFFT * 4:
            return
        lat = args.lat_ms * 1e-3 * odr
        x_mg = x.astype(np.float32) * st.mg
        nspec = int(SPEC_SECONDS * odr)
        off = max(0, len(x_mg) - nspec)
        S = stft_power(x_mg[off:], odr)
        if S is None:
            return
        P, cen, freqs, cal = S
        cen = cen + off

        # ── spectrogram ──
        ax_s.clear()
        tt = cen / odr
        amp_db = 10 * np.log10(np.maximum(4 * P / cal, 1e-6))  # dB re 1 mg^2
        ax_s.pcolormesh(tt, freqs[1:], amp_db[:, 1:].T, shading="auto", cmap="magma", vmin=-10, vmax=60)
        chirps = []
        for k, (mode, s) in enumerate(starts):
            e = next((ei for (mo, ei) in ends if ei > s), None)
            done = e is not None
            if e is None:
                e = s + int(st.chirp_ms * 1e-3 * odr)
            chirps.append((mode, s, e, done))
            if e + lat < off:
                continue
            ii = np.arange(max(s, off), min(e + int(lat), len(x_mg)), 16)
            f = chirp_freq(ii, s, e, st.f0, st.f1, lat)
            col = MODE_STYLE.get(mode, (PAIR_COLOR, mode))[0] if "+" not in mode else PAIR_COLOR
            for h, a in ((1, 0.9), (2, 0.35), (3, 0.2)):
                ax_s.plot(ii / odr, f * h, c=col, lw=1, alpha=a)
            ax_s.text(max(s, off) / odr, 560, mode, color=col, fontsize=9, va="bottom")
        ax_s.set_yscale("log")
        ax_s.set_ylim(15, 800)
        ax_s.set_ylabel("Hz")
        ax_s.set_title("IMU spectrogram (dB re 1 mg); lines = drive f, 2f, 3f")
        ax_s.set_xlim(off / odr, max(len(x_mg) / odr, off / odr + SPEC_SECONDS))

        # ── response per pass (computed on full data for completed + current chirps) ──
        ax_r.clear()
        per_mode = {}
        for mode, s, e, done in chirps:
            per_mode.setdefault(mode, []).append((s, e, done))
        for mode, passes in per_mode.items():
            col = PAIR_COLOR if "+" in mode else MODE_STYLE.get(mode, ("k", mode))[0]
            name = "both (in phase)" if "+" in mode else MODE_STYLE.get(mode, ("k", mode))[1]
            for j, (s, e, done) in enumerate(passes[-4:]):
                lo, hi = max(0, s - NFFT), min(len(x_mg), e + int(lat) + NFFT)
                Sx = stft_power(x_mg[lo:hi], odr)
                if Sx is None:
                    continue
                Pp, cp, fq, cl = Sx
                fd, a = response(Pp, cp + lo, fq, cl, s, e, st.f0, st.f1, lat)
                latest = j == len(passes[-4:]) - 1
                ax_r.plot(fd, a, c=col, lw=2.2 if latest else 1, alpha=1 if latest else 0.3,
                          label=f"{name} (pass {len(passes)})" if latest else None)
        ax_r.set_xscale("log")
        ax_r.set_yscale("log")
        ax_r.set_xlim(18, 550)
        ax_r.set_ylim(1, 5000)
        ax_r.set_xticks([20, 30, 50, 80, 125, 200, 300, 500])
        ax_r.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
        ax_r.get_xaxis().set_minor_formatter(matplotlib.ticker.NullFormatter())
        ax_r.grid(True, which="both", alpha=0.25)
        ax_r.set_ylabel("mg peak at drive f")
        ax_r.set_title("Response vs frequency — latest pass bold, previous 3 faded")
        if per_mode:
            ax_r.legend(fontsize=8, loc="upper left")

        # ── broadband RMS vs time ──
        ax_t.clear()
        rms = np.sqrt(P[:, 1:].sum(axis=1) * 2 / cal) / np.sqrt(2)
        ax_t.semilogy(tt, rms, c="k", lw=1)
        ax_t.set_xlim(ax_s.get_xlim())
        ax_t.set_ylim(1, 5000)
        ax_t.set_xlabel("s")
        ax_t.set_ylabel("RMS mg")
        ax_t.grid(True, which="both", alpha=0.25)
        fig.tight_layout()

    def save():
        os.makedirs(out, exist_ok=True)
        x, starts, ends, odr = st.data()
        np.savez_compressed(os.path.join(out, "chirp.npz"), accel_lsb=x, odr=odr, mg_per_lsb=st.mg,
                            f0=st.f0, f1=st.f1, chirp_ms=st.chirp_ms,
                            starts=np.array([(m, i) for m, i in starts], dtype=object),
                            ends=np.array([(m, i) for m, i in ends], dtype=object))
        draw()
        fig.savefig(os.path.join(out, "chirp.png"), dpi=120)
        print(f"saved {len(x)} samples ({len(x) / odr:.1f} s, ODR {odr:.0f} Hz) → {out}")

    try:
        if args.headless:
            while args.seconds is None or time.time() - t_start < args.seconds:
                time.sleep(0.5)
        else:
            plt.ion()
            plt.show()
            while plt.fignum_exists(fig.number):
                draw()
                fig.canvas.draw_idle()
                plt.pause(0.3)
                if args.seconds and time.time() - t_start > args.seconds:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        st.stop()
        save()


if __name__ == "__main__":
    main()
