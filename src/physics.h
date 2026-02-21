#pragma once

enum class SimMode { ROLLING, SLIDING };

struct ImpactInfo {
    bool  hit;
    bool  left;
    float speed; // magnitude at impact [m/s]
};

// Initialise physics state (resets position to cavity centre)
void physicsInit();

// Advance simulation by dt seconds.
// sin_alpha    : sine of tube tilt from IMU gravity X-axis
// lin_accel_x  : tube linear acceleration [m/s²] from IMU
// Returns impact info if a wall was struck this step.
ImpactInfo physicsStep(float dt, float sin_alpha, float lin_accel_x);

// ── Ball state ────────────────────────────────────────────────────────────────
// Written by physics task. Read by comms task and haptic module.
extern volatile float phys_x_pos; // [m]
extern volatile float phys_x_vel; // [m/s]

// ── Runtime-adjustable parameters ────────────────────────────────────────────
// Written by comms task. Read by physics task.
extern volatile float   g_cavity;    // virtual tube length [m]
extern volatile SimMode g_sim_mode;  // ROLLING or SLIDING
extern volatile bool    g_rebound;   // enable/disable bounce
