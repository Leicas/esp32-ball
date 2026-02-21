#pragma once
#include <stdint.h>

// ── Telemetry snapshot ────────────────────────────────────────────────────────
// Written by the physics task each tick; read by the comms task for streaming.
extern volatile float    telem_sin_alpha;
extern volatile float    telem_x_mm;
extern volatile float    telem_x_vel;
extern volatile uint32_t telem_phys_hz;  // measured physics steps/sec (updated 1 Hz)

// Marble-mode telemetry (written by physics task in MARBLES mode)
extern volatile float    telem_grav_x;        // gravity X / g  [−1, 1]
extern volatile float    telem_grav_y;        // gravity Y / g  [−1, 1]
extern volatile float    telem_grav_z;        // gravity Z / g  [−1, 1]
extern volatile float    telem_impact_energy; // max impact energy this frame [0, 1]

// Initialise BLE GATT server and start advertising.
void commsInit();

// Process any pending serial command bytes.
void commsHandleSerial();

// Emit "$sin_a,x_mm,v_mps\n" over serial (and BLE notify if connected).
void commsStreamTelemetry();

// Emit "# key=val …\n" debug line over serial and update BLE status char.
void commsSendStatus();

// Emit "@grav_x,grav_y,impact_e,x0,y0,…\n" marble telemetry line over serial.
void commsStreamMarbles();

