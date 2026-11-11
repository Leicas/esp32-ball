# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Companion App

`companion/visualizer.py` — live rolling-stone visualization (adapted from the BNO085 orientation visualizer in the same folder).

```bash
# Install dependencies (one-time, uses the project venv)
.venv/Scripts/python -m pip install -r companion/requirements.txt

# Run (port auto-selected if only one is available)
.venv/Scripts/python companion/visualizer.py COM3
.venv/Scripts/python companion/visualizer.py --list   # list ports
```

**What it shows:** tilted tube with the ball animated at its current position; speed-coloured ball; wall-impact flash; scrolling position and velocity traces on the right panel.

**Key bindings (sent to firmware as single-char commands):**

| Key | Action |
|-----|--------|
| `r` | ROLLING mode |
| `s` | SLIDING mode |
| `b` | toggle rebound |
| `+` / `=` | cavity +5 cm |
| `-` | cavity −5 cm |
| `q` | quit |

**Serial protocol:**
- Firmware → host: `$sin_a,x_mm,v_mps\n` at 20 Hz; `# key=val ...\n` at 1 Hz and on command replies
- Host → firmware: single ASCII char (`r`, `s`, `b`, `+`, `-`, `?`)
- `?` requests an immediate status dump (`mode=`, `cavity_mm=`, `G=`, `mu=`)

## Build & Flash Commands

This project uses PlatformIO. All commands run from the repo root.

```bash
# Build only (default env: adafruit_feather_esp32c6)
pio run

# Build a specific environment
pio run -e adafruit_feather_esp32c6
pio run -e codecell_drive

# Build and flash
pio run -e adafruit_feather_esp32c6 --target upload
pio run -e codecell_drive --target upload

# Open serial monitor (115200 baud)
pio device monitor

# Build + flash + monitor in one step
pio run -e codecell_drive --target upload && pio device monitor
```

There are no automated tests — this is embedded firmware verified by deploying to hardware and reading serial output.

### `a7_bringup` — custom A7 PCB self-test

`src/a7_bringup/main.cpp` is a standalone bring-up firmware for the custom Rolling Stone A7 board (repo `esp32-ball-hardware`). It is excluded from the other envs via `build_src_filter`. The board enumerates as an Espressif USB-Serial/JTAG port (VID:PID 303A:1001), so pass the port explicitly:

```bash
pio run -e a7_bringup --target upload --upload-port COM50
pio device monitor -p COM50
```

Pin map (from `esp32-ball-hardware/firmware-interface.md`, verified against `kicad/circuit.json` and on hardware 2026-10-07): PCA9632 handle RGB on I2C IO6/IO7 @0x62; LSM6DS3TR-C on SPI SCK=IO14 MOSI=IO15 MISO=IO18 CS=IO19 INT1=IO2 (WHO_AM_I reads **0x6A**); VSYS ADC IO4 (÷2); USB_GOOD_N IO3; buttons MODE=IO0 GAIN=IO1 BOOT=IO9; HAPTIC_EN IO5; TDM BCLK=IO21 FS=IO22 DATA=IO23 to four MAX98357A (48 kHz, 8×16-bit slots, slot 0..3 = LFi/MF/HF/LF); blade pixels IO20. Serial commands: `?` help, `s` self-test, `m` IMU probe, `r g b w 0 c` LED, `a` accel stream, `1`–`4` tone per channel (auto-off 3 s, amplitude capped at 0.25 FS), `t` actuator test (sweeps the fitted channels — currently LFi and HF, see `CH_FITTED` — 50–320 Hz and reports PASS/FAIL from IMU vibration RMS at 1.66 kHz vs a silent baseline; result also shown in `s`), `T` same on all four channels, `W` full 20–500 Hz lock-in sweep (solo + pair in/anti-phase, IMU at ±8 g; driven by `companion/actuator_sweep.py COM6 --amp 0.10`, which writes response/crossover plots, `crossover_table.h` and a report to `companion/sweeps/<run>/`), `C` continuous chirp loop (20→500 Hz in 6 s on each fitted channel, then both; raw accel streamed from the IMU FIFO — view live with `companion/chirp_view.py COM6 --amp 0.10`, add `--seconds N --headless` to just capture), `y` diag: mirror HF onto TDM slots 4–7, `v<ch> <hz> <amp>\n` constant tone with no auto-off (host must repeat within 1.5 s or it cuts out; prints `$lv,<rms_mg>,<vsys>` at 5 Hz; `v0` stops) — GUI: `.venv/Scripts/python companion/vibe_gui.py [COM6]`, `u` toggle USB500/USB100 input limit (USB500 by default — USB100 browns out at 0.25 FS), `x` stop, `+`/`-` amplitude, `f` frequency, `p`/`k` blade rainbow/black.

### `a7_ball` — rolling ball / marbles on the A7 board (`-DA7_BOARD`)

Same `main.cpp` / physics / marbles / comms as the Feather and H-bridge targets; A7 backends in `imu.cpp`, `haptic.cpp` and `main.cpp` (`#if defined(A7_BOARD)`), pins and levels in `config.h`.

- IMU: LSM6DS3TR-C over SPI at 833 Hz (±4 g, 500 dps) fused with a Mahony filter into the same outputs as the BNO085 reports (gravity, `imu_sin_alpha`, linear accel, quaternion). Gyro bias and initial attitude are taken at boot — **keep the board still for ~0.5 s after reset**. Tube axis = IMU Y, along the board (`A7_TUBE_AXIS` / `A7_TUBE_SIGN`).
- Haptics: TDM 48 kHz, 48 frames per 1 ms physics tick (ramped). TacHammer Drake drive plan: wall/marble impacts = positive pulse on LFi (slot 0, "tick" into the impact end, ≤ 9 ms, `A7_IMPACT_FS` 0.25 with 1 ms raised-cosine edges — 0.40 FS square ticks browned out on USB power), rolling rumble = negative arch on LFi (`A7_ROLL_FS` 0.25, away from the end stop), rolling texture on HF (slot 2, DC-free, `A7_HF_ROLL_FS` 0.20).
- UI: MODE button cycles ROLLING → SLIDING → MARBLES; GAIN cycles intensity 0.4 / 0.7 / 1.0. Handle RGB: green rolling / blue sliding / magenta marbles, brighter with ball speed, white flash on impacts; 3 red blinks at boot = the last reset was a brownout (persistent count in the `# up_s=… brownouts=N` status line). Serial/BLE protocol unchanged, so `companion/visualizer.py` works as-is.

```bash
pio run -e a7_ball --target upload --upload-port COM6
```

### Music player (separate repo)

The Squeezebox / Music Assistant haptic player for the A7 board lives in `D:\dev\esp32ball\a7-music` (its own git repo). Note that its `custom_sdkconfig` rebuilds the *shared* Arduino framework libs: building an env here afterwards restores the stock libs (~4 min first build).

## Toolchain Notes

The `fix_toolchain_path.py` pre-script runs automatically before every build and handles two known issues with the espressif32 platform on Windows:

1. **RISC-V compiler not found**: `toolchain-riscv32-esp >= 14.x` moved binaries to `riscv32-esp-elf/bin/` — the script prepends that nested path.
2. **`littlefs-python` version drift**: PlatformIO keeps upgrading to 0.17.x which has a circular-import bug; the script pins it back to 0.12.0 automatically.

If builds fail with missing compiler or littlefs import errors, these workarounds are the first place to investigate.

## Hardware Targets

Both environments target the same ESP32-C6 (RISC-V, 160 MHz) with the Arduino + ESP-IDF framework.

### `adafruit_feather_esp32c6`
- STEMMA QT I2C: SDA=IO19, SCL=IO18, power enable=IO20
- NeoPixel: IO9
- I2S amp: BCLK=IO21 (SCK header), LRCLK=IO22 (MOSI header), DOUT=IO23 (MISO header)
- I2S uses native `driver/i2s_std.h`, not an Arduino wrapper

### `codecell_drive`  (-DCODECELL_DRIVE)
- [CodeCell ESP32-C6 Drive board](https://github.com/microbotsio/CodeCell) — 8 MB flash
- Onboard BNO085: I2C SDA=IO8, SCL=IO9 (managed internally by `CodeCell.Init()`)
- H-bridge (DriveCell, members of CodeCell object):
  - `myCodeCell.Drive1`: IN1=IO22, IN2=IO21 — primary haptic actuator
  - `myCodeCell.Drive2`: IN1=IO2, IN2=IO3 — spare
- No I2S; no external NeoPixel; no STEMMA power pin
- Physics runs at 1 kHz (vs. 22 050 Hz for Feather); DriveCell uses 20 kHz LEDC PWM internally

GPIO 4-7 are JTAG on the ESP32-C6 — avoid for output on either board.

## Architecture

The entire firmware lives in `src/main.cpp` as a single-file Arduino sketch implementing a **haptic simulation of a ball rolling/sliding inside a tube**, based on Yao & Hayward, Eurohaptics 2006. A single `#define CODECELL_DRIVE` build flag (set via `platformio.ini`) switches all hardware-specific code paths; the physics equations are identical between profiles.

### Shared physics

- Trapezoidal integration; `sin_alpha` (sine of tube tilt from BNO085 X-axis) drives the equations
- `ROLLING` mode: `ẍ = (g/1.4)·sin(α)` — solid sphere
- `SLIDING` mode: `ẍ = g·sin(α) − g·µ·sgn(sin(α))·cos(α)` — Coulomb friction
- Ball constrained to `[0, D_CAVITY]`; wall contact triggers impact event

### Feather data flow (each `loop()` call, ~8 ms)
1. Drain BNO085 events (up to 8) → update `sin_alpha`
2. `fillAudioBuf()` — 128 physics steps at H = 1/22050 s, writes `audio_buf[]`
3. `i2s_channel_write()` — blocks ~8 ms pushing buffer to DMA (natural rate limiter)

Haptic synthesis:
- **Rolling**: negative sine arch wavetable (30-sample period), index = `pos_mm % 30` → pitch ∝ velocity; amplitude ∝ speed
- **Impact**: 190-sample (8.6 ms) rectangular pulse on wall contact

### CodeCell data flow (`-DCODECELL_DRIVE`, each `loop()` call)
1. `myCodeCell.Run(1000)` returns `true` at 1 kHz; reads BNO085 → update `sin_alpha`
   - CodeCell returns g-units directly, so `sin_alpha = ax_g` (no `/9.8` needed)
2. `ccPhysicsAndHaptic()` — one physics step at H = 1/1000 s, commands DriveCell

Haptic synthesis via `myCodeCell.Drive1`:
- **Rolling**: `Drive1.Run(smooth=true, pct, flip_ms)` — `flip_ms = 15/|v|` ms preserves the same pitch-velocity law as the wavetable; `pct` ∝ speed
- **Impact**: `Drive1.Pulse(direction, 9 ms)` — edge-triggered on first wall contact only (`cc_at_left/right_wall` flags prevent re-firing while ball rests against wall); `cc_impact_left` cooldown prevents `Run()`/`Drive()` calls from interrupting the pulse

**NeoPixel status codes (Feather only):**
- White blink (3x): boot
- Yellow: I2C scan in progress
- Green blink (3x): BNO085 found
- Red steady: fatal (sensor not found)
- Orange blink (2x): sensor reset detected
- Cycling hue: normal running

**Key tunable constants** (all `constexpr` near top of `main.cpp`):
- `SIM_MODE`: `ROLLING` or `SLIDING`
- `D_CAVITY`: virtual tube length in meters
- `FRICTION_MU`: Coulomb friction coefficient (sliding mode)
- `WTABLE_SIZE`: rolling noise spatial period in mm (shared between profiles)
- Feather only: `GAIN`, `AUDIO_CHUNK`
- CodeCell only: `PHYS_HZ`, `CC_MIN_SPEED` (speed threshold below which actuator is silenced), `CC_IMPACT_MS`
