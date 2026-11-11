"""
Actuator frequency-response sweep + crossover design for the Rolling Stone A7 board.

Drives the a7_bringup firmware's 'W' command (each fitted actuator solo, then the pair in phase
and in anti-phase, 20-500 Hz), records the on-board IMU lock-in response, and produces:

  response.png    sensitivity, distortion, pair summation and effective relative phase
  crossover.png   crossover choice, predicted combined output, per-channel drive (EQ)
  sweep.csv       raw data (re-analyse with --csv)
  crossover_table.h  per-frequency drive gains/polarity for firmware use
  report.txt      numbers behind the recommendation

Usage:
  .venv/Scripts/python companion/actuator_sweep.py COM6 --amp 0.10
  .venv/Scripts/python companion/actuator_sweep.py --csv companion/sweeps/<run>/sweep.csv
"""

import argparse
import csv
import datetime as dt
import os
import sys
import time

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

CH_NAME = {1: "LFi (J3)", 2: "MF (J4)", 3: "HF (J5)", 4: "LF (J6)"}
AMP_MAX = 1.0  # firmware hard cap, FS
NOISE_MARGIN = 3.0  # a reading counts as signal when > NOISE_MARGIN x baseline


# ─────────────────────────────────────────────────────────────────────────────
# Acquisition
# ─────────────────────────────────────────────────────────────────────────────
def acquire(port, amp, timeout_s=240):
    import serial

    s = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(1.5)  # opening the USB-Serial/JTAG port can reset the board
    s.reset_input_buffer()

    def send(cmd, wait=0.3):
        s.write(cmd.encode())
        t = time.time()
        while time.time() - t < wait:
            s.readline()

    send("-" * 21)  # amplitude to 0, then step up in 0.05 FS
    send("+" * int(round(amp / 0.05)))
    s.write(b"W")
    rows, t0 = [], time.time()
    while time.time() - t0 < timeout_s:
        line = s.readline().decode(errors="replace").strip()
        if not line:
            continue
        if line.startswith("$sw,"):
            f = line[4:].split(",")
            rows.append(
                dict(mode=f[0], hz=float(f[1]), amp=float(f[2]), h1=float(f[3]), h2=float(f[4]),
                     h3=float(f[5]), rms=float(f[6]), vsys=float(f[7]), n=int(f[8]))
            )
            if rows[-1]["hz"] > 0:
                print(f"\r  {rows[-1]['mode']:>4} {rows[-1]['hz']:6.1f} Hz  {rows[-1]['h1']:8.2f} mg   ",
                      end="", flush=True)
        elif line.startswith("# sweep begin"):
            print(line)
        elif line.startswith("# sweep end"):
            print("\n" + line)
            break
        elif "=====" in line or line.startswith("rst:"):
            sys.exit(f"\nboard reset during sweep ({line}) — likely brownout; retry with a lower --amp")
        elif "skipped" in line:
            sys.exit(line)
    else:
        sys.exit("timed out waiting for sweep end")
    s.close()
    return rows


def save_csv(rows, path):
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def load_csv(path):
    with open(path) as fh:
        out = []
        for r in csv.DictReader(fh):
            out.append({k: (v if k == "mode" else float(v)) for k, v in r.items()})
        return out


# ─────────────────────────────────────────────────────────────────────────────
# Analysis
# ─────────────────────────────────────────────────────────────────────────────
def split(rows):
    """mode → (freqs, h1, h2, h3, vsys, baseline_h1)"""
    modes = {}
    for m in dict.fromkeys(r["mode"] for r in rows):
        rr = [r for r in rows if r["mode"] == m]
        base = next((r["h1"] for r in rr if r["hz"] == 0), np.nan)
        sw = [r for r in rr if r["hz"] > 0]
        modes[m] = dict(
            f=np.array([r["hz"] for r in sw]),
            h1=np.array([r["h1"] for r in sw]),
            h2=np.array([r["h2"] for r in sw]),
            h3=np.array([r["h3"] for r in sw]),
            rms=np.array([r["rms"] for r in sw]),
            vsys=np.array([r["vsys"] for r in sw]),
            amp=sw[0]["amp"],
            base=base,
        )
    return modes


def lr_mag(f, fc, order=4):
    """Linkwitz-Riley magnitude responses (low, high); |L|+|H| sums flat in phase."""
    x = (f / fc) ** (order / 2)
    lo = 1 / (1 + x ** 2) ** 0.5
    hi = x / (1 + x ** 2) ** 0.5
    # LR = squared Butterworth → square the Butterworth magnitudes
    return lo ** 2, hi ** 2


def analyse(rows, band_hz=(30.0, 400.0)):
    modes = split(rows)
    common = set.intersection(*(set(m["f"]) for m in modes.values()))
    for m in modes.values():
        keep = np.isin(m["f"], sorted(common))
        for key in ("f", "h1", "h2", "h3", "rms", "vsys"):
            m[key] = m[key][keep]
    solo = [m for m in modes if m.isdigit()]
    if len(solo) < 2:
        sys.exit("need two fitted channels for crossover analysis")
    a, b = solo[0], solo[1]
    plus, minus = f"{a}+{b}", f"{a}-{b}"
    A, B = modes[a], modes[b]
    f = A["f"]
    amp = A["amp"]
    SA, SB = A["h1"] / amp, B["h1"] / amp  # sensitivity, mg per FS
    floorA, floorB = A["base"] / amp, B["base"] / amp
    for m, M_ in ((a, A), (b, B)):
        live = int(np.sum(M_["h1"] > 10 * M_["base"]))
        if live < 3:
            print(f"WARNING: {CH_NAME[int(m)]} never exceeds 10x its noise floor ({live} points) — "
                  "actuator not responding; crossover results are meaningless until it is fixed")

    # Effective relative phase from |S+|² - |S-|² = 4 Re(A·B*)  (vector sum over the 3 axes)
    if plus in modes and minus in modes:
        Sp, Sm = modes[plus]["h1"] / amp, modes[minus]["h1"] / amp
        re_ab = (Sp ** 2 - Sm ** 2) / 4
        cosphi = np.clip(re_ab / np.maximum(SA * SB, 1e-9), -1, 1)
        valid = (A["h1"] > NOISE_MARGIN * A["base"]) & (B["h1"] > NOISE_MARGIN * B["base"])
    else:
        Sp = Sm = None
        re_ab = np.zeros_like(f)
        cosphi = np.full_like(f, np.nan)
        valid = np.zeros_like(f, bool)

    band = (f >= band_hz[0]) & (f <= band_hz[1])

    def design(SL, SH, fc, pol):
        """LR4 split at fc, high channel polarity pol, then a common EQ gain per frequency
        that holds the output flat. Returns the largest flat level reachable in `band` with
        every drive <= AMP_MAX, and the drives that produce it."""
        re_lh = re_ab * pol

        def output(dL, dH):  # |dL·L + p·dH·H| over the 3-axis vector
            return np.sqrt(np.maximum(dL ** 2 * SL ** 2 + dH ** 2 * SH ** 2 + 2 * dL * dH * re_lh, 0))

        wL, wH = lr_mag(f, fc)
        raw = output(wL, wH)  # output per unit of common gain
        max_out = raw * AMP_MAX / np.maximum(np.maximum(wL, wH), 1e-9)
        target = float(np.min(max_out[band]))
        g = target / np.maximum(raw, 1e-9)
        dL, dH = np.minimum(g * wL, AMP_MAX), np.minimum(g * wH, AMP_MAX)
        # spare drive: how far below the cap the busier channel sits, averaged in log over the band
        headroom = float(np.mean(np.log(AMP_MAX / np.maximum(np.maximum(dL, dH), 1e-9))[band]))
        return dict(target=target, headroom=headroom, wL=wL, wH=wH, dL=dL, dH=dH, eq_out=output(dL, dH),
                    both_cap=output(np.full_like(f, AMP_MAX), np.full_like(f, AMP_MAX)))

    # Search: which actuator takes the low band, crossover frequency, polarity of the high one
    # The flat level is often pinned by one weak frequency (e.g. LFi at 30 Hz) that no crossover
    # changes, so many designs tie: among those within 10% (<1 dB, set by noisy low-f points) of
    # the best level, prefer the one that leaves the most drive headroom across the band.
    cands = []
    for low, high, SL, SH in ((a, b, SA, SB), (b, a, SB, SA)):
        for fc in np.geomspace(35, 350, 80):
            for pol in (1, -1):
                cands.append(dict(design(SL, SH, fc, pol), low=low, high=high, SL=SL, SH=SH, fc=float(fc), pol=pol))
    top = max(c["target"] for c in cands)
    best = max((c for c in cands if c["target"] >= 0.90 * top), key=lambda c: c["headroom"])

    # Where the sensitivities cross, for reference
    dlog = np.log(best["SL"] + 1e-6) - np.log(best["SH"] + 1e-6)
    cross = [float(np.sqrt(f[i] * f[i + 1])) for i in np.where(np.sign(dlog[:-1]) != np.sign(dlog[1:]))[0]]
    fc_note = "max flat output, then max drive headroom; sensitivity curves cross at " + (
        ", ".join(f"{c:.0f}" for c in cross) + " Hz" if cross else "no frequency")

    return dict(best, band_hz=band_hz, modes=modes, a=a, b=b, plus=plus, minus=minus, f=f, amp=amp, SA=SA, SB=SB,
                floorA=floorA, floorB=floorB, Sp=Sp, Sm=Sm, cosphi=cosphi, valid=valid, fc_note=fc_note)


# ─────────────────────────────────────────────────────────────────────────────
# Output
# ─────────────────────────────────────────────────────────────────────────────
def fmt_ax(ax, ylabel, logy=True):
    ax.set_xscale("log")
    if logy:
        ax.set_yscale("log")
    ax.set_xlim(18, 550)
    ax.set_xticks([20, 30, 50, 80, 125, 200, 300, 500])
    ax.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
    ax.get_xaxis().set_minor_formatter(matplotlib.ticker.NullFormatter())
    ax.grid(True, which="both", alpha=0.25)
    ax.set_ylabel(ylabel)


def plot_response(R, path):
    f, M = R["f"], R["modes"]
    ca, cb = "#2f6fdf", "#e0622c"
    fig, axs = plt.subplots(2, 2, figsize=(13, 8.5))
    fig.suptitle(f"A7 actuator response at the on-board IMU  (drive {R['amp']:.2f} FS per channel)")

    ax = axs[0, 0]
    ax.plot(f, R["SA"], "-o", ms=3, c=ca, label=CH_NAME[int(R["a"])])
    ax.plot(f, R["SB"], "-o", ms=3, c=cb, label=CH_NAME[int(R["b"])])
    ax.axhline(R["floorA"], c=ca, ls=":", lw=1, label="noise floor")
    ax.axhline(R["floorB"], c=cb, ls=":", lw=1)
    ax.axvline(R["fc"], c="k", ls="--", lw=0.8)
    ax.set_title("Sensitivity (fundamental)")
    fmt_ax(ax, "mg peak per FS")
    ax.legend(fontsize=8)

    ax = axs[0, 1]
    for m, c in ((R["a"], ca), (R["b"], cb)):
        h1, h2, h3 = M[m]["h1"], M[m]["h2"], M[m]["h3"]
        ok = (h1 > NOISE_MARGIN * M[m]["base"]) & (h2 >= 0)
        thd = np.sqrt(np.where(h2 > 0, h2, 0) ** 2 + np.where(h3 > 0, h3, 0) ** 2) / np.maximum(h1, 1e-9)
        ax.plot(f[ok], 100 * thd[ok], "-o", ms=3, c=c, label=CH_NAME[int(m)])
    ax.set_title("Distortion (H2+H3)/H1 where above noise, harmonic < Nyquist")
    fmt_ax(ax, "%", logy=False)
    ax.legend(fontsize=8)

    ax = axs[1, 0]
    if R["Sp"] is not None:
        ax.plot(f, R["Sp"], "-", c="#2a9d55", lw=2, label="both, in phase")
        ax.plot(f, R["Sm"], "-", c="#9b59b6", lw=2, label="both, anti-phase")
    ax.plot(f, R["SA"] + R["SB"], "k:", lw=1, label="|A|+|B| (ideal coherent sum)")
    ax.plot(f, np.maximum(R["SA"], R["SB"]), c="0.6", lw=1, label="best single")
    ax.axvline(R["fc"], c="k", ls="--", lw=0.8)
    ax.set_title("Driving both together")
    fmt_ax(ax, "mg peak per FS (each channel)")
    ax.legend(fontsize=8)

    ax = axs[1, 1]
    v = R["valid"]
    ax.plot(f[v], np.degrees(np.arccos(R["cosphi"][v])), "-o", ms=3, c="#444")
    ax.axhline(90, c="0.6", lw=0.8)
    ax.axvline(R["fc"], c="k", ls="--", lw=0.8)
    ax.set_ylim(0, 180)
    ax.set_yticks([0, 45, 90, 135, 180])
    ax.set_title("Effective relative phase (0° adds, 180° cancels)")
    fmt_ax(ax, "degrees", logy=False)
    ax.set_xlabel("Hz")
    axs[1, 0].set_xlabel("Hz")

    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_crossover(R, path):
    f = R["f"]
    cl, ch = ("#2f6fdf", "#e0622c") if R["low"] == R["a"] else ("#e0622c", "#2f6fdf")
    nl, nh = CH_NAME[int(R["low"])], CH_NAME[int(R["high"])]
    fig, axs = plt.subplots(1, 2, figsize=(13, 4.8))
    pol = "same polarity" if R["pol"] > 0 else f"{nh} polarity inverted"
    fig.suptitle(f"Crossover: LR4 at {R['fc']:.0f} Hz, {nl} low / {nh} high, {pol}")

    ax = axs[0]
    cap = AMP_MAX
    ax.plot(f, R["SL"] * cap, c=cl, lw=1, alpha=0.6, label=f"{nl} alone @ {cap:.2f} FS")
    ax.plot(f, R["SH"] * cap, c=ch, lw=1, alpha=0.6, label=f"{nh} alone @ {cap:.2f} FS")
    ax.plot(f, R["both_cap"], c="#2a9d55", lw=1, ls="--", label="both @ cap (max possible)")
    ax.plot(f, R["eq_out"], c="k", lw=2.5, label=f"crossover + EQ → {R['target']:.0f} mg flat")
    ax.axvline(R["fc"], c="k", ls="--", lw=0.8)
    ax.axvspan(18, R["band_hz"][0], color="0.9")
    ax.axvspan(R["band_hz"][1], 550, color="0.9")
    ax.set_title("Predicted output at the IMU (shaded = outside flat-target band)")
    fmt_ax(ax, "mg peak")
    ax.set_xlabel("Hz")
    ax.legend(fontsize=8)

    ax = axs[1]
    ax.plot(f, R["dL"], c=cl, lw=2, label=f"{nl} drive")
    ax.plot(f, R["dH"], c=ch, lw=2, label=f"{nh} drive")
    ax.plot(f, R["wL"] * AMP_MAX, c=cl, ls=":", lw=1, label="LR4 low shape (unequalised)")
    ax.plot(f, R["wH"] * AMP_MAX, c=ch, ls=":", lw=1, label="LR4 high shape")
    ax.axhline(AMP_MAX, c="r", lw=0.8, ls="--", label="firmware cap")
    ax.axvline(R["fc"], c="k", ls="--", lw=0.8)
    ax.set_title("Per-channel drive for flat output (crossover × EQ)")
    fmt_ax(ax, "FS", logy=False)
    ax.set_ylim(0, AMP_MAX * 1.1)
    ax.set_xlabel("Hz")
    ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def write_table(R, path):
    f = R["f"]
    lo, hi = int(R["low"]) - 1, int(R["high"]) - 1
    with open(path, "w") as fh:
        fh.write("// Generated by companion/actuator_sweep.py — do not edit by hand.\n")
        fh.write(f"// LR4 crossover at {R['fc']:.1f} Hz ({R['fc_note']}), flat target {R['target']:.1f} mg\n")
        fh.write("// at the on-board IMU. Gains are drive amplitude in FS (linear interpolation in log f).\n")
        fh.write("#pragma once\n\n")
        fh.write(f"static constexpr int XO_LOW_CH = {lo};   // {CH_NAME[lo + 1]}\n")
        fh.write(f"static constexpr int XO_HIGH_CH = {hi};  // {CH_NAME[hi + 1]}\n")
        fh.write(f"static constexpr bool XO_HIGH_INVERT = {'true' if R['pol'] < 0 else 'false'};\n")
        fh.write(f"static constexpr float XO_FC_HZ = {R['fc']:.1f}f;\n")
        fh.write(f"static constexpr int XO_N = {len(f)};\n")
        fh.write("static const float XO_HZ[XO_N] = {" + ", ".join(f"{x:.1f}f" for x in f) + "};\n")
        fh.write("static const float XO_GAIN_LOW[XO_N] = {" + ", ".join(f"{x:.4f}f" for x in R["dL"]) + "};\n")
        fh.write("static const float XO_GAIN_HIGH[XO_N] = {" + ", ".join(f"{x:.4f}f" for x in R["dH"]) + "};\n")


def write_report(R, path):
    f = R["f"]
    lines = []
    for m in (R["a"], R["b"]):
        S = R["SA"] if m == R["a"] else R["SB"]
        i = int(np.argmax(S))
        above = f[S > 0.5 * S[i]]
        lines.append(f"{CH_NAME[int(m)]}: peak {S[i]:.0f} mg/FS at {f[i]:.0f} Hz; "
                     f"-6 dB band {above.min():.0f}-{above.max():.0f} Hz; floor {R['floorA' if m == R['a'] else 'floorB']:.0f} mg/FS")
    lines.append(f"Low actuator: {CH_NAME[int(R['low'])]}, high actuator: {CH_NAME[int(R['high'])]}")
    lines.append(f"Crossover: LR4 at {R['fc']:.0f} Hz ({R['fc_note']})")
    lines.append("Polarity at crossover: " + ("same" if R["pol"] > 0 else "invert the high channel"))
    v = R["valid"]
    if v.any():
        ph = np.degrees(np.arccos(R["cosphi"][v]))
        lines.append(f"Effective relative phase: {ph.min():.0f}-{ph.max():.0f} deg over {f[v].min():.0f}-{f[v].max():.0f} Hz")
    lines.append(f"Flat target achievable {R['band_hz'][0]:.0f}-{R['band_hz'][1]:.0f} Hz within {AMP_MAX:.2f} FS/channel: {R['target']:.0f} mg peak")
    bmax = R["both_cap"]
    lines.append(f"Max output (both @ cap): {bmax.min():.0f}-{bmax.max():.0f} mg; "
                 f"best single @ cap: {np.maximum(R['SA'], R['SB']).min() * AMP_MAX:.0f}-"
                 f"{np.maximum(R['SA'], R['SB']).max() * AMP_MAX:.0f} mg")
    vs = np.concatenate([R["modes"][m]["vsys"] for m in R["modes"]])
    lines.append(f"VSYS during sweep: {vs.min():.2f}-{vs.max():.2f} V")
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", help="serial port of the A7 board (e.g. COM6)")
    ap.add_argument("--amp", type=float, default=0.10, help="drive per channel, FS (0.05-1.00)")
    ap.add_argument("--csv", help="re-analyse an existing sweep.csv instead of acquiring")
    ap.add_argument("--out", help="output directory (default companion/sweeps/<timestamp>)")
    ap.add_argument("--band", type=float, nargs=2, default=(30.0, 400.0), metavar=("LO", "HI"),
                    help="frequency band the crossover+EQ holds flat (default 30 400)")
    args = ap.parse_args()

    if args.csv:
        rows = load_csv(args.csv)
        out = args.out or os.path.join(os.path.dirname(os.path.abspath(args.csv)),
                                       f"band{args.band[0]:.0f}-{args.band[1]:.0f}")
        os.makedirs(out, exist_ok=True)
    else:
        if not args.port:
            ap.error("port required unless --csv is given")
        if not 0.05 <= args.amp <= AMP_MAX:
            ap.error(f"--amp must be in 0.05..{AMP_MAX}")
        out = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "sweeps",
                                       dt.datetime.now().strftime("%Y%m%d-%H%M%S") + f"-amp{args.amp:.2f}")
        os.makedirs(out, exist_ok=True)
        rows = acquire(args.port, args.amp)
        save_csv(rows, os.path.join(out, "sweep.csv"))

    R = analyse(rows, tuple(args.band))
    plot_response(R, os.path.join(out, "response.png"))
    plot_crossover(R, os.path.join(out, "crossover.png"))
    write_table(R, os.path.join(out, "crossover_table.h"))
    for ln in write_report(R, os.path.join(out, "report.txt")):
        print(ln)
    print(f"→ {out}")


if __name__ == "__main__":
    main()
