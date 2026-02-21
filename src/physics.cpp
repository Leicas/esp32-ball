#include "physics.h"
#include "config.h"
#include <math.h>

volatile float   phys_x_pos = 0.50f;
volatile float   phys_x_vel = 0.0f;
volatile float   g_cavity   = 1.00f;
volatile SimMode g_sim_mode = SimMode::ROLLING;
volatile bool    g_rebound  = true;

// Integration carry-over (must not be reset between ticks)
static float s_accel_k  = 0.0f;
// Edge-detection for wall contact (prevent re-triggering while resting)
static bool  s_at_left  = false;
static bool  s_at_right = false;

void physicsInit()
{
    phys_x_pos = g_cavity * 0.5f;
    phys_x_vel = 0.0f;
    s_accel_k  = 0.0f;
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
    if (mode == SimMode::ROLLING) {
        // Eq. 2  ẍ = (g/1.4)·sin(α)  (tube inertia compensated)
        accel_new = G_FACTOR * sin_alpha - (G_FACTOR / 9.8f) * lin_accel_x;
    } else {
        // Eq. 3  ẍ = g·sin(α) − g·µ·sgn(sin(α))·cos(α)
        float s  = sin_alpha;
        float c  = sqrtf(fmaxf(1.0f - s * s, 0.0f));
        if (s * s <= FRICTION_MU * FRICTION_MU * (1.0f - s * s))
            accel_new = 0.0f;
        else
            accel_new = 9.8f * s - 9.8f * FRICTION_MU * copysignf(c, s);
        accel_new -= lin_accel_x;
    }

    // ── Trapezoidal integration ───────────────────────────────────────────────
    float v_new = phys_x_vel + dt * (s_accel_k + accel_new) * 0.5f;
    float x_new = phys_x_pos + dt * (phys_x_vel + v_new) * 0.5f;
    s_accel_k = accel_new;

    // ── Wall constraint + optional rebound ───────────────────────────────────
    if (x_new <= 0.0f) {
        if (v_new < 0.0f) {
            if (!s_at_left) {
                if (g_rebound) {
                    impact = {true, true, fabsf(v_new)};
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
                if (g_rebound) {
                    impact = {true, false, fabsf(v_new)};
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
