# Asymmetric three-actuator playback

The `a7_ball_threeway` build uses the installed 8-ohm LF, LFi and HF actuators.
LFi is **not** a symmetric woofer: its positive side is the impact end.

* LFi / J3 / slot 0: positive-only raised-edge impacts, existing 0.25 FS and
  2â€“9 ms envelope (rattle contacts use the existing 4 ms envelope). No sine EQ,
  no bipolar sustained input, no DC-blocking filter on the impact pulse.
* LF / J6 / slot 3 and HF / J5 / slot 2: a pair of 65-tap, equal-delay FIRs at
  2 kHz. They jointly implement frequency allocation and enclosure EQ. Their
  delay is 16 ms, plus interpolation/audio buffering. LFi remains immediate
  to preserve contact latency; impacts are not phase-aligned to a delayed
  sinusoidal sustained response.
* HF is penalized below 145 Hz because the measured harmonic response there
  was poor. The optimized split is not one ideal textbook crossover frequency. LF retains a
  drive floor through the overlap to avoid the measured low-drive dead zone.
* A linked LF/HF peak limiter caps outputs at 0.25 FS. Overload reduces both
  together. This does not promise undistorted arbitrary multitone signals.
  The physical GAIN control scales the result. Response depends on drive level.
* This calibrated profile uses 50â€“240 Hz as the flatness target. It does not add
  the old unmeasured kilohertz texture path. The ordinary `a7_ball` build remains
  available. No claim is made about flatness above the measured/usable band.

## Design and evidence

`sweeps/enclosure-design-010/` and `sweeps/enclosure-design-025/` contain new
10% and 25% sweeps. `sweeps/enclosure-fullscale/` contains the earlier 100%
measurement. The latter has substantial harmonic distortion and is not a
linear calibration or a clean-output target.

The final candidate and its actual filtered sweep are in
`sweeps/threeway-asymmetric-robust/`. Read `validation.json` for **measured**
ripple and partial harmonic ratios. `design.json` is only a prediction.
The initial symmetric-three-path candidates in other folders are superseded.
An optimal full-scale/clean/flat solution is not established: these objectives
conflict, and friction, impacts and saturation are nonlinear.

The model uses solo measurements and all in/anti-phase pairs. Those pairs
identify a real three-axis acceleration Gram matrix, not absolute complex
phase or force in the hand. Nonphysical correlations are projected onto a
positive-semidefinite matrix, with correction/error metrics recorded. A
two-level power-law fit accounts approximately for amplitude dependence. Only
equal-delay symmetric FIR paths are supported by this phase information.
H2/H3 are partial distortion indicators, not total THD; unavailable harmonics
are not counted as zero distortion. Grip, mounting and battery changes require
another validation. Current sweeps had USB present.

## Reproduce

From the repository root, install `companion/requirements-calibration.txt` in
the project venv. Then:

```powershell
.venv/Scripts/python companion/design_threeway.py companion/sweeps/enclosure-design-025/sweep.csv --low-reference companion/sweeps/enclosure-design-010/sweep.csv --out companion/sweeps/threeway-asymmetric-robust
Copy-Item companion/sweeps/threeway-asymmetric-robust/threeway_coefficients.h src/threeway_coefficients.h
pio run -e a7_bringup --target upload --upload-port COM6
.venv/Scripts/python companion/validate_threeway.py COM6 companion/sweeps/threeway-asymmetric-robust/validation.csv
.venv/Scripts/python companion/report_threeway.py companion/sweeps/threeway-asymmetric-robust
pio run -e a7_ball_threeway --target upload --upload-port COM6
```

Bringup command `E` runs the **actual** FIR + interpolation path with a unit
sine input. It stops automatically, accepts `x` between tones, and saves every
received row immediately. It does not exercise LFi's separate impact envelope.
`tests/native/threeway_test.cpp` checks DC rejection, impulse coefficients,
matched symmetry, arbitrary-input output bounds and reset behavior.

FIR construction reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.firwin2.html

## Validated result and deployment

The final 40-point sweep measured 99.2–185.5 mg fundamental over the 50–240 Hz
target band: 5.43 dB peak-to-peak (about +/-2.72 dB). Median H2/H3 magnitude
ratio was 14.8%, maximum 24.6%; this is a cleaner compromise, not distortion-free
or a demonstrated global maximum-output optimum. LF/HF outputs remain capped
at 25% digital full scale because the 100% sweeps were substantially nonlinear.

Flashed `a7_ball_threeway` and verified 1999–2000 physics updates/s, zero audio
drops and zero overruns during an eight-second stationary check. The retained
brownout counter stayed at 9 throughout; no new reset was observed. Dynamic
handheld feel and battery-only performance still need user evaluation.

## Active full-output override

User requested full drive. Active FIR taps are 4x the archived robust profile,
with a linked 1.0 FS limiter. LFi positive impacts now reach 1.0 FS while
retaining raised edges and bounded duration. GAIN still scales output.
The earlier 25% flatness/distortion measurements do not apply at this level.
Regenerating the archived header restores its lower-drive calibration.
