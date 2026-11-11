# Rolling Stone desktop

A Tauri 2 desktop companion for the Rolling Stone firmware. It connects directly
through USB serial: the firmware owns the simulation and haptic output; the app
renders incoming telemetry and sends small control commands.

## Run

Requirements: Node.js, Rust, and the Tauri platform prerequisites (MSVC build
tools and WebView2 on Windows).

```powershell
cd companion/desktop
npm install
npm run tauri dev
```

For a standalone executable:

```powershell
npm run tauri build -- --no-bundle
```

The Windows executable is `src-tauri/target/release/rolling-stone.exe`.
Close any serial monitor before connecting. Choose the board's port in the app;
the connected A7 normally appears as an Espressif USB device. This desktop app
currently uses USB; `../visualizer.py --ble` retains the Bluetooth workflow.

The browser development preview (`npm run dev`) cannot access the native serial
bridge. It shows a disconnected state and offers an explicitly labelled demo.
Demo does not control the board.

## Controls

- R / T / S / M: rolling, textured, sliding, or marbles.
- B: toggle rebound.
- − / +: shorten / lengthen the virtual tube by 50 mm.
- G: cycle A7 haptic gain through 0.4, 0.7, and 1.0.

Settings follow firmware status, including physical MODE and GAIN button presses.
Commands are rejected while the stream is stale or disconnected; they are not
saved for later replay. USB reconnects to the selected port after interruption.

## Protocol

Firmware sends newline-delimited ASCII at 115200 baud:

- `$sin_alpha,x_mm,v_mps` for tube telemetry (~58 Hz).
- `@gx,gy,gz,impact,qw,qx,qy,qz,x0,y0,z0,r0,...` with zero to five marble tuples after reveal (no tuples while sealed).
- `# key=value ...` for mode, cavity, rebound, gain, and rate diagnostics.

Native Tauri commands: `list_ports`, `connect`, `disconnect`, `send_command`.
Events: `device-line` (complete line) and `connection` (state, port, detail).
The Canvas renderer paints the latest authoritative samples on animation frames;
it does not run an independent haptic simulation.

## Validation

```powershell
npm test
npm run build
cd src-tauri
cargo test
```

The native tests cover line fragmentation, oversized-frame recovery, command
whitelisting, finite packet validation and telemetry expiry. Frontend tests cover
serial telemetry parsing. Real-board verification additionally checks commands,
status synchronization, mode switches and reconnect behavior.

MARBLES is a blind shake-box game: press **M** to enter a hidden round with
0–5 identical 10 mm balls. Shake the device, select your guess, then press **V**
to reveal. **N** starts a new hidden round. Zero balls produces no rattle.
The board withholds the count and positions until reveal; game status uses
`game_round`, `game_revealed`, and `game_count` (-1 while hidden). The current
Python fallback does not provide this game UI; use this USB desktop app.

Feedback uses summed wall contacts, softer inter-ball collisions, and overlapping
short LFi pulses with HF contact rings. There is no continuous rolling bed in
MARBLES. Tube modes retain their separate rolling synthesis and rebound setting.
