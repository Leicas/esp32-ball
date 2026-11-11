#pragma once
#include <Wire.h>

// Initialise the board IMU (A7 SPI LSM6DS3TR-C, otherwise I2C BNO085).
// Returns true on success.
bool imuInit(TwoWire &wire);

// Read fresh sensor data and update the globals below.
// A7 calls from physics; BNO085 uses its own I2C task.
void imuUpdate();

// Latest sensor readings (written by imuUpdate, read by physics task)
extern volatile float imu_sin_alpha;   // sin of tube tilt (gravity X / g)
extern volatile float imu_angular_speed; // bias-corrected rad/s magnitude (A7)
extern volatile float imu_swing_speed; // rad/s perpendicular to handle, excludes axial roll
extern volatile float imu_tube_accel;  // linear acceleration along the configured tube axis
extern volatile uint32_t imu_sample_hz; // measured fusion update rate (A7)
extern volatile float imu_lin_accel_x; // body linear acceleration X [m/s²]
extern volatile float imu_lin_accel_y; // tube linear acceleration Y [m/s²]
extern volatile float imu_lin_accel_z; // tube linear acceleration Z [m/s²]
extern volatile float imu_grav_x;      // raw gravity X [m/s²]
extern volatile float imu_grav_y;      // raw gravity Y [m/s²]
extern volatile float imu_grav_z;      // raw gravity Z [m/s²]
extern volatile float imu_qw;          // game rotation vector quaternion w
extern volatile float imu_qx;          // game rotation vector quaternion x
extern volatile float imu_qy;          // game rotation vector quaternion y
extern volatile float imu_qz;          // game rotation vector quaternion z

#if defined(A7_BOARD)
void imuCaptureStart(char mode);
void imuCaptureService();
float imuCaptureAngular(float live);
float imuCaptureGain(float live);
#endif

#if defined(A7_BOARD)
bool imuCaptureBoost();
#endif

#if defined(A7_BOARD)
int imuCaptureMask();
bool imuCaptureScripted();
#endif
