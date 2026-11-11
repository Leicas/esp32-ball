#include "physics.h"
#include "config.h"
#include <math.h>

volatile float   phys_x_pos = 0.50f;
volatile float   phys_x_vel = 0.0f;
volatile float   g_cavity   = TUBE_LENGTH_M;
volatile SimMode g_sim_mode = SimMode::ROLLING;
volatile bool    g_rebound  = true;

// Integration carry-over (must not be reset between ticks)
static float s_accel_k  = 0.0f;
static SimMode s_prev_mode = SimMode::TEXTURED;
// Edge-detection for wall contact (prevent re-triggering while resting)
static bool  s_at_left  = false;
static bool  s_at_right = false;

void physicsInit()
{
    phys_x_pos = g_cavity * 0.5f;
    phys_x_vel = 0.0f;
    s_accel_k  = 0.0f;
    s_prev_mode = g_sim_mode;
    s_at_left  = false;
    s_at_right = false;
}

ImpactInfo physicsStep(float dt, float sin_alpha, float lin_accel_x)
{
    ImpactInfo impact = {false, false, 0.0f};

    // Local copies of volatile params — consistent for this step
    const float   cavity = g_cavity;
    const SimMode mode   = g_sim_mode;

    // ── Acceleration ──────────────────────────────────────────────────────────
    float accel_new;
    bool static_friction = false;
    if (isRollingMode(mode)) {
        // Eq. 2  ẍ = (g/1.4)·sin(α)  (tube inertia compensated)
        // plus rolling resistance: a constant part that holds the ball still on
        // a nearly level tube, and a speed-proportional part that makes it
        // settle to a steady roll instead of accelerating all the way.
        const float s = fmaxf(-1.0f, fminf(1.0f, sin_alpha));
        const float c = sqrtf(fmaxf(1.0f - s * s, 0.0f));
        const float drive = G_FACTOR * s - (G_FACTOR / 9.8f) * lin_accel_x;
        const float resistance = ROLLING_RESISTANCE * G_FACTOR * c;
        constexpr float kStickSpeed = 0.005f; // [m/s]
        if (fabsf(phys_x_vel) <= kStickSpeed && fabsf(drive) <= resistance) {
            accel_new = 0.0f;
            static_friction = true;
        } else {
            const float direction = fabsf(phys_x_vel) > kStickSpeed ? phys_x_vel : drive;
            accel_new = drive - copysignf(resistance, direction) - ROLLING_DRAG * phys_x_vel;
        }
    } else {
        // Coulomb friction acts against motion, while static friction can
        // cancel the complete drive (gravity plus the tube's linear accel).
        const float s = fmaxf(-1.0f, fminf(1.0f, sin_alpha));
        const float c = sqrtf(fmaxf(1.0f - s * s, 0.0f));
        const float drive = 9.8f * s - lin_accel_x;
        const float friction_limit = 9.8f * FRICTION_MU * c;
        constexpr float kStickSpeed = 0.01f; // [m/s], suppress numerical creep at rest

        if (fabsf(phys_x_vel) <= kStickSpeed && fabsf(drive) <= friction_limit) {
            accel_new = 0.0f;
            static_friction = true;
        } else {
            const float friction_direction = fabsf(phys_x_vel) > kStickSpeed
                                                 ? phys_x_vel
                                                 : drive;
            accel_new = drive - copysignf(friction_limit, friction_direction);
        }
    }

    // Mode changes may follow a long period in another simulation (including
    // MARBLES); do not average against an acceleration from the old model.
    if (mode != s_prev_mode) {
        s_accel_k = accel_new;
        s_prev_mode = mode;
    }
    if (static_friction)
        s_accel_k = 0.0f;

    // ── Trapezoidal integration ───────────────────────────────────────────────
    float v_new = phys_x_vel + dt * (s_accel_k + accel_new) * 0.5f;
    float x_new = phys_x_pos + dt * (phys_x_vel + v_new) * 0.5f;

    // Static friction holds the marble at rest instead of preserving a tiny
    // residual velocity forever. Kinetic friction can also stop a marble in a
    // step; clamp that zero crossing so it cannot numerically reverse direction.
    if (static_friction) {
        v_new = 0.0f;
        x_new = phys_x_pos;
    } else if (mode == SimMode::SLIDING && phys_x_vel != 0.0f && phys_x_vel * v_new < 0.0f) {
        v_new = 0.0f;
    }
    s_accel_k = accel_new;

    // ── Wall constraint + optional rebound ───────────────────────────────────
    if (x_new <= 0.0f) {
        if (v_new < 0.0f) {
            if (!s_at_left) {
                impact = {true, true, fabsf(v_new)};
                if (g_rebound) {
                    v_new  = -RESTITUTION_E * v_new;
                } else {
                    v_new = 0.0f;
                }
            } else {
                v_new = 0.0f;
            }
        }
        x_new     = 0.0f;
        s_at_left  = true;
        s_at_right = false;
    } else if (x_new >= cavity) {
        if (v_new > 0.0f) {
            if (!s_at_right) {
                impact = {true, false, fabsf(v_new)};
                if (g_rebound) {
                    v_new  = -RESTITUTION_E * v_new;
                } else {
                    v_new = 0.0f;
                }
            } else {
                v_new = 0.0f;
            }
        }
        x_new      = cavity;
        s_at_right  = true;
        s_at_left   = false;
    } else {
        s_at_left  = false;
        s_at_right = false;
    }

    phys_x_vel = v_new;
    phys_x_pos = x_new;
    return impact;
}
