# esp32-ball — Virtual Rolling Stone

A handheld ESP32-C6 device that synthesizes the feeling of a ball rolling and
sliding inside a tube that does not physically exist. You tilt it, and a virtual
marble runs from one end to the other, hits the wall, and rolls back. Nothing
moves inside: the sensation is generated entirely in firmware from tilt sensed
by a BNO085 IMU.

It reproduces the haptic illusion from Hsin-Yun Yao and Vincent Hayward,
*An Experiment on Length Perception with a Virtual Rolling Stone*, Eurohaptics
2006 ([PDF](https://cim.mcgill.ca/~haptic/pub/HY-VH-EH-06.pdf)).

Full write-up on the blog:
- [Part 1: the illusion, the physics, and the I2S build](https://antoine.weill-duflos.fr/en/post/esp32-rolling-stone/)
- [Part 2: driving the ball with an H-bridge](https://antoine.weill-duflos.fr/en/post/esp32-rolling-stone-hbridge/)

## Two hardware targets

A single `#define HBRIDGE` build flag (set per environment in `platformio.ini`)
switches all hardware-specific code. The physics is identical between the two.

| Environment | Board | Haptic output |
|---|---|---|
| `feather` | Adafruit ESP32-C6 Feather + MAX98357A I2S amp | Audio to a voice-coil actuator (TITAN Haptics) |
| `hbridge` | Microbots CodeCell C6 Drive | H-bridge PWM to a motor / actuator |

Both use a BNO085 9-DOF IMU for tilt sensing.

## Firmware architecture

The ESP32-C6 is single core, so timing is handled with three FreeRTOS tasks:

- `taskIMU` (priority 3): reads the BNO085 as fast as I2C allows.
- `taskPhysics` (priority 5): runs the simulation at exactly 1 kHz.
- `taskComms` (priority 1): serial and BLE commands, telemetry at ~58 Hz.

Three simulation modes, selectable at runtime: `ROLLING` (solid sphere),
`SLIDING` (Coulomb friction), and `MARBLES` (a 3D box of three marbles of
different mass and radius). Source is split across `main.cpp`, `physics.cpp`,
`haptic.cpp`, `imu.cpp`, `comms.cpp`, and `marbles.cpp` in `src/`.

## Build and flash

This project uses [PlatformIO](https://platformio.org/). From the repo root:

```bash
# Build
pio run -e feather
pio run -e hbridge

# Build and flash
pio run -e feather --target upload
pio run -e hbridge --target upload

# Serial monitor (115200 baud)
pio device monitor
```

A `fix_toolchain_path.py` pre-script runs automatically before every build to
work around two known issues on Windows (the RISC-V compiler path and a
`littlefs-python` version pin). See `CLAUDE.md` for details.

## Companion app

`companion/visualizer.py` connects over USB serial or BLE and draws a live view
of the simulation: the tilted tube and ball, scrolling position and velocity
traces, and the live telemetry. It can also replay the rolling-sound synthesis
through your computer speakers.

```bash
pip install -r companion/requirements.txt
python companion/visualizer.py COM3      # USB serial
python companion/visualizer.py           # auto-scan BLE for "RollingStone"
python companion/visualizer.py --list    # list serial ports + BLE devices
```

Keys: `r` rolling, `s` sliding, `m` marbles, `+`/`-` cavity size, `q` quit.

## License

MIT. See [LICENSE](LICENSE).
