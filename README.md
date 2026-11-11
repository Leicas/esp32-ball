# esp32-ball — Virtual Rolling Stone

A handheld ESP32-C6 device that synthesizes the feeling of objects moving inside
a tube that does not physically exist. You tilt it, and a virtual marble runs
from one end to the other, hits the wall, and rolls back. Nothing moves inside:
the sensation is generated entirely in firmware from the motion sensed by an IMU.

It started as a reproduction of the haptic illusion from Hsin-Yun Yao and Vincent
Hayward, *An Experiment on Length Perception with a Virtual Rolling Stone*,
Eurohaptics 2006 ([PDF](https://cim.mcgill.ca/~haptic/pub/HY-VH-EH-06.pdf)). On
the custom A7 board the same handle is now also a textured slide, a box of
marbles, a lightsaber, a rainstick and a moo box.

Write-up on the blog:
- [Part 1: the illusion, the physics, and the I2S build](https://antoine.weill-duflos.fr/en/post/esp32-rolling-stone/)
- [Part 2: driving the ball with an H-bridge](https://antoine.weill-duflos.fr/en/post/esp32-rolling-stone-hbridge/)

## Hardware targets

Build flags in `platformio.ini` select the hardware backend; the ball physics is shared.

| Environment | Board | Haptic output |
|---|---|---|
| `feather` | Adafruit ESP32-C6 Feather + MAX98357A I2S amp, BNO085 | Audio to a voice-coil actuator |
| `hbridge` | Microbots CodeCell C6 Drive, BNO085 | H-bridge PWM to a motor / actuator |
| `a7_ball_threeway` | Rolling Stone A7 PCB, LSM6DS3TR-C | 48 kHz TDM to four MAX98357A amplifiers |
| `a7_ball` | Same board, earlier two-actuator profile | See the note below |
| `a7_bringup` | Same board | Self-test and actuator measurement firmware |

`a7_ball_threeway` is the build to use on the A7 board. `a7_ball` currently
compiles but no longer fits its default flash partition.

The A7 board has four amplifier channels on TDM slots 0–3: **LFi**, **MF**,
**HF** and **LF**. LFi is asymmetric: positive drive pushes its hammer into an
end stop, so it receives positive impact pulses and only the negative side of
any sustained bass. The other three carry sustained vibration and sound.

## Effects (A7)

| Mode | LED | Serial | What it is |
|---|---|---|---|
| `ROLLING_BALL` | green | `r` | A 10 mm metal ball in a plastic tube. Boot default. |
| `SLIDING` | blue | `s` | A block with Coulomb friction: silent in motion, end-stop knocks only. |
| `TEXTURED_SLIDE` | red-orange | `t` | The same ball physics with procedural surface roughness driving damped resonances. |
| `MARBLES` | magenta | `m` / `n` | A blind shake-box game with 0–5 hidden marbles; `v` reveals the count. |
| `LIGHTSABER` | cyan | `l` | Ignition, idle hum, motion-driven swings and clashes. |
| `RAINSTICK` | yellow | `y` | Grains pouring through nails, in four characters: sand, rice, beans, moo box. |

### Buttons

- **GAIN** switches family: ball → lightsaber → rainstick → back to the ball.
- **MODE** acts inside the family:
  - ball: cycles Rolling → Sliding → Textured → Marbles;
  - rainstick: picks the character; the LED blinks 1–4 times for sand, rice, beans, moo box;
  - lightsaber: replays the ignition.

Output intensity (0.4 / 0.7 / 1.0) is on the serial/BLE command `g`. During play
the mode colour brightens with ball speed and wall impacts flash white. Three red
blinks at boot mean the previous reset was a brownout.

### Rolling ball

The rolling signal has no random source. It reads a fixed signature of five
bumps, one ball circumference long (31.4 mm), by ball position, so it repeats
exactly once per revolution, its flutter rate follows speed, and it is silent
when the ball stops. A second fixed table of fine surface ripple carries the slow
roll and fades out above 0.6 m/s. The roll drives one damped tube-wall mode
(150 Hz); wall hits ring a low plastic thud (190 Hz, LF) and a short bright tick
(1.15 kHz, HF) alongside the positive LFi pulse. See `src/rolling_ball.h`.

The cavity matches the handle: `TUBE_LENGTH_M` is 0.20 m (`+` / `-` resize it at
run time). An ideal ball crosses that in 0.57 s at a 10° tilt, which feels
abrupt, so the ball has rolling resistance (`ROLLING_RESISTANCE`,
`ROLLING_DRAG`): about 0.73 s at 10°, arriving at 0.44 m/s. Wall rebound returns
30 % of the incoming speed. Impact strength follows a square-root law and is
full at 0.8 m/s. All of these are feel settings, not measured material values.

### Rainstick

`src/rain_tube.h` simulates 72 grains falling through nine staggered rows of
nails in a 0.20 m tube. The grains are a sample of the fill: the model reports a
contact *flow* (how often and how hard grains are hitting things), and
`src/rainstick_audio.h` renders it as randomly timed contacts, each a short noise
burst plus a tap on one of six short resonators. A thinned, low-passed copy of
the same events is the tactile bed on LF. A few pebble grains and the grains
landing on the end cap are separate knocks on LFi/LF.

The pile lets go grain by grain, so a pour swells in over a couple of seconds,
runs about 6 s when vertical and tails off. Shaking the stick along its axis
rattles it. Fall speed is tuned, not real gravity.

### Moo box

The fourth rainstick character (`src/moo_box.h`). A piston with mass travels the
tube once it is tipped past about 15°. Gravity accelerates it and the air it
pushes through a reed resists in proportion to its speed, so it keeps gathering
speed. That airflow drives a sawtooth reed through two vocal formants; loudness,
pitch (about 120 → 180 Hz) and mouth opening all follow it, giving a swelling
"mmmeooOOO" that stops when the piston lands with a clunk. It moos in both
directions.

### Lightsaber samples

The lightsaber plays recorded ignition, idle and swing excerpts from
`src/lightsaber_samples.h`. Those recordings are **not distributed** with this
repository. The build falls back to `src/lightsaber_samples_stub.h`, which has
the same arrays filled with silence, so everything compiles and the mode runs
without a recorded voice. To add sound, generate your own
`src/lightsaber_samples.h` with `companion/prepare_lightsaber_samples.py`; it is
git-ignored and picked up automatically.

## Firmware architecture

The ESP32-C6 is single core with no floating-point unit.

- **Feather / H-bridge:** `taskIMU` reads the BNO085, `taskPhysics` runs at
  1 kHz, `taskComms` handles serial/BLE and telemetry at ~58 Hz.
- **A7:** `taskPhysics` runs at 2 kHz from a microsecond timer. Each tick reads
  the IMU in one SPI burst (1.66 kHz output rate, Mahony fusion), advances the
  active simulation and synthesises 24 frames of 48 kHz audio for four channels.
  Every effect ends in one shared output stage (`mixAndWrite` in
  `src/haptic.cpp`): a look-ahead limiter, the intensity setting and the
  per-channel trim in a single pass, then a non-blocking TDM write into a 3 ms
  DMA queue.

Source is split across `main.cpp`, `physics.cpp`, `haptic.cpp`, `imu.cpp`,
`comms.cpp` and `marbles.cpp` in `src/`, with each synthesis voice in its own
header.

### Timing

From motion to amplifier the chain is roughly 3–5 ms (IMU interval, one physics
tick, a one-tick ramp, 0.5 ms of limiter look-ahead and the output queue). This
is derived from the code, not measured with a probe.

Mean time per 500 µs tick, measured on the board at rest:

| Mode | Mean | Mode | Mean |
|---|---|---|---|
| Sliding | 173 µs | Rainstick | 287 µs |
| Rolling | 238 µs | Textured | 291 µs |
| Lightsaber | 286 µs | Marbles | 387 µs |

The IMU read and attitude filter take about 107 µs of each tick. The 1 Hz status
line reports `step_us_avg`, `imu_us_avg`, `step_us_max`, `overruns`,
`audio_drops`, `phys_hz` and `imu_hz`.

An earlier calibrated profile equalised LF and HF with a matched FIR pair
([design and validation](companion/THREEWAY.md)). It added 16 ms of delay and is
no longer in the signal path; the filter and its test remain in the tree.

## Build and flash

This project uses [PlatformIO](https://platformio.org/). From the repo root:

```bash
pio run -e a7_ball_threeway --target upload --upload-port COM6
pio run -e feather --target upload
pio run -e hbridge --target upload
pio device monitor        # 115200 baud
```

On the A7 board keep the handle still for half a second after reset while the
gyro calibrates.

A `fix_toolchain_path.py` pre-script runs before every build to work around two
known issues on Windows (the RISC-V compiler path and a `littlefs-python`
version pin).

## Tests

Native tests cover the ball and marble physics, the texture and detail
synthesis, the rainstick, rolling ball and moo box, the FIR filter and the
battery recorder. They need CMake and a C++ compiler:

```bash
cmake -S tests/native -B .pio/native-tests
cmake --build .pio/native-tests --config Release
ctest --test-dir .pio/native-tests -C Release --output-on-failure
```

`lightsaber_tests` currently fails one assertion on the idle voice.

## Companion tools

- **Desktop app** in [`companion/desktop`](companion/desktop/README.md): Tauri,
  native USB serial, a Canvas motion display, mode / gain / rebound / cavity
  controls and live diagnostics. `npm install`, then `npm run tauri dev`.
- **`companion/visualizer.py`**: live view over USB serial or BLE, with a
  computer-speaker preview of the ball modes.

  ```bash
  pip install -r companion/requirements.txt
  python companion/visualizer.py COM3      # USB serial
  python companion/visualizer.py --ble     # Bluetooth
  python companion/visualizer.py --list    # list ports and devices
  ```

  Keys: `r` rolling, `t` textured, `s` sliding, `m` marbles, `b` rebound,
  `+` / `-` cavity size, `g` intensity (A7), `q` quit.
- **Actuator measurement** (with the `a7_bringup` firmware):
  `actuator_sweep.py`, `chirp_view.py`, `capture_signature.py`, `vibe_gui.py`,
  and the three-way filter design and validation scripts described in
  [`companion/THREEWAY.md`](companion/THREEWAY.md). Measurement data sets are
  not included in the repository.
- **Battery runtime recorder** (A7): `D` arms a test that starts once USB power
  has been absent for 2 s, `Q` prints the result, `X` cancels. A checkpoint is
  saved to NVS every 30 s with elapsed time and the sampled system voltage. A
  run interrupted by a reboot is kept as a lower bound, not proof of an empty
  battery. `companion/battery_server` is an optional logger for the UDP reports
  the board sends when Wi-Fi credentials have been provisioned over USB.

## License

MIT. See [LICENSE](LICENSE).
