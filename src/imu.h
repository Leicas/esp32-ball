#pragma once
#include <Wire.h>

// Initialise BNO085 on the given I2C bus.
// Returns true on success.
bool imuInit(TwoWire &wire);

// Drain pending BNO085 events and update the globals below.
// Call once per physics tick.
void imuUpdate();

// Latest sensor readings (written by imuUpdate, read by physics task)
extern volatile float imu_sin_alpha;   // sin of tube tilt (gravity X / g)
extern volatile float imu_lin_accel_x; // tube linear acceleration X [m/s²]
extern volatile float imu_lin_accel_y; // tube linear acceleration Y [m/s²]
extern volatile float imu_lin_accel_z; // tube linear acceleration Z [m/s²]
extern volatile float imu_grav_y;      // raw gravity Y [m/s²]
extern volatile float imu_grav_z;      // raw gravity Z [m/s²]
extern volatile float imu_qw;          // game rotation vector quaternion w
extern volatile float imu_qx;          // game rotation vector quaternion x
extern volatile float imu_qy;          // game rotation vector quaternion y
extern volatile float imu_qz;          // game rotation vector quaternion z
