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
