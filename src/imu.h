#pragma once
#include <Wire.h>

// Initialise BNO085 on the given I2C bus.
// Returns true on success.
bool imuInit(TwoWire &wire);

// Drain pending BNO085 events and update the globals below.
// Call once per physics tick.
void imuUpdate();

// Latest sensor readings (written by imuUpdate, read by physics task)
extern volatile float imu_sin_alpha;    // sin of tube tilt (gravity X / g)
extern volatile float imu_lin_accel_x;  // tube linear acceleration [m/s²]
