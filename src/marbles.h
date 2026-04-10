#pragma once
#include "config.h"

struct Marble
{
    float x, y, z, vx, vy, vz;
    float mass; // relative mass [arbitrary units, 1.0 = reference]
    float r;    // radius [m]
};

// All marble positions and velocities (written by physics task)
extern Marble g_marbles[MARBLE_COUNT];

// Reset marble positions to a 3×2×1 grid; zero velocities.
void marblesInit();

// Advance by dt seconds under acceleration (m/s²) and gravity (m/s²).
// Returns the maximum impact speed seen this tick [m/s].
float marblesStep(float dt, float accel_x_mps2, float accel_y_mps2, float accel_z_mps2, float grav_z_mps2);
