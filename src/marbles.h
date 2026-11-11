#pragma once
#include "config.h"
#include <stdint.h>

struct Marble
{
    float x, y, z, vx, vy, vz;
    float mass; // relative mass [arbitrary units, 1.0 = reference]
    float r;    // radius [m]
};

// Capacity is MARBLE_COUNT; only [0, marblesCount()) contains active balls.
extern Marble g_marbles[MARBLE_COUNT];

// Seeded, non-overlapping placement. count is clamped to [0, MARBLE_COUNT].
// Defaults preserve the original three-ball call sites.
void marblesInit(int count = MARBLE_DEFAULT_COUNT, uint32_t seed = 1);
int marblesCount();

// Advance by dt seconds under body-frame gravity and box linear acceleration.
// Gravity uses the IMU body axes; level gravity is +Z (down).
// Returns summed wall impulse plus a reduced inter-ball cue [relative-mass · m/s].
float marblesStep(float dt,
                  float grav_x_mps2, float grav_y_mps2, float grav_z_mps2,
                  float accel_x_mps2, float accel_y_mps2, float accel_z_mps2);

// Mass-weighted RMS tangential speed while touching the box [m/s]. Free-flight
// velocity is excluded; normal impacts use the separate impact haptic path.
float marblesContactSpeed();

// Telemetry-only impact peak hold/decay (~25 ms); never feed this back to haptics.
float marblesImpactEnvelopeStep(float current, float impact_momentum, float dt);
