#pragma once
#include <stdint.h>

// ── Telemetry snapshot ────────────────────────────────────────────────────────
// Written by the physics task each tick; read by the comms task for streaming.
extern volatile float telem_sin_alpha;
extern volatile float telem_x_mm;
extern volatile float telem_x_vel;
extern volatile uint32_t telem_phys_hz; // measured physics steps/sec (updated 1 Hz)
extern volatile uint32_t telem_step_us_max; // max physics step time in current 1 s window
extern volatile uint32_t telem_step_us_avg; // mean physics step time over the last 1 s window
extern volatile uint32_t telem_imu_us_avg;  // of which IMU read + fusion
extern volatile uint32_t telem_step_overruns; // cumulative missed timer ticks / long steps

// Marble-mode telemetry (written by physics task in MARBLES mode)
extern volatile float telem_grav_x;        // gravity X / g  [−1, 1]
extern volatile float telem_grav_y;        // gravity Y / g  [−1, 1]
extern volatile float telem_grav_z;        // gravity Z / g  [−1, 1]
extern volatile float telem_impact_energy; // max impact energy this frame [0, 1]
extern volatile float telem_qw;            // game rotation quaternion w
extern volatile float telem_qx;            // game rotation quaternion x
extern volatile float telem_qy;            // game rotation quaternion y
extern volatile float telem_qz;            // game rotation quaternion z

// Initialise BLE GATT server and start advertising.
void commsInit();

// Process any pending serial command bytes.
void commsHandleSerial();

// Emit "$sin_a,x_mm,v_mps\n" over serial (and BLE notify if connected).
void commsStreamTelemetry();

// Emit "# key=val …\n" debug line over serial and update BLE status char.
void commsSendStatus();

// Emit "@grav_x,grav_y,grav_z,impact_e,qw,qx,qy,qz,x0,y0,z0,…\n" marble telemetry line over serial.
void commsStreamMarbles();

#if defined(A7_BOARD)
// Cycle the A7 gain control through the same states and LED acknowledgement
// used by the physical GAIN button. Call from the comms task.
void a7CycleGain();
#endif

// Physics-task-only: publish a coherent marble snapshot for the comms reader.
void commsPublishMarbles();
