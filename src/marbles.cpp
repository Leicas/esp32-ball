#include "marbles.h"
#include "config.h"
#include <math.h>

Marble g_marbles[MARBLE_COUNT];

void marblesInit()
{
    // Assign per-marble mass and radius from config arrays
    for (int i = 0; i < MARBLE_COUNT; i++)
    {
        g_marbles[i].mass = MARBLE_MASSES[i];
        g_marbles[i].r    = MARBLE_RADII[i];
        g_marbles[i].vx   = 0.0f;
        g_marbles[i].vy   = 0.0f;
        g_marbles[i].vz   = 0.0f;
    }

    // Starting positions (spread across box)
    g_marbles[0].x = MARBLE_BOX_W * 0.25f;
    g_marbles[0].y = MARBLE_BOX_H * 0.25f;
    g_marbles[0].z = MARBLE_BOX_D * 0.5f;

    g_marbles[1].x = MARBLE_BOX_W * 0.75f;
    g_marbles[1].y = MARBLE_BOX_H * 0.25f;
    g_marbles[1].z = MARBLE_BOX_D * 0.5f;

    g_marbles[2].x = MARBLE_BOX_W * 0.5f;
    g_marbles[2].y = MARBLE_BOX_H * 0.75f;
    g_marbles[2].z = MARBLE_BOX_D * 0.5f;
}

float marblesStep(float dt, float accel_x_mps2, float accel_y_mps2, float accel_z_mps2, float grav_z_mps2)
{
    float max_impact = 0.0f;

    // 1. Apply box acceleration and gravity (mass-independent in non-inertial frame)
    for (int i = 0; i < MARBLE_COUNT; i++)
    {
        g_marbles[i].vx -= accel_x_mps2 * dt;
        g_marbles[i].vy -= accel_y_mps2 * dt;
        g_marbles[i].vz += grav_z_mps2 * dt;
        g_marbles[i].x += g_marbles[i].vx * dt;
        g_marbles[i].y += g_marbles[i].vy * dt;
        g_marbles[i].z += g_marbles[i].vz * dt;
    }

    // 2. Wall collisions — per-marble radius, impact weighted by mass
    for (int i = 0; i < MARBLE_COUNT; i++)
    {
        const float r = g_marbles[i].r;
        const float m = g_marbles[i].mass;

        // X walls
        if (g_marbles[i].x < r)
        {
            if (g_marbles[i].vx < 0.0f)
            {
                float imp = m * (-g_marbles[i].vx);
                if (imp > max_impact) max_impact = imp;
                g_marbles[i].vx = MARBLE_RESTITUTION * (-g_marbles[i].vx);
            }
            g_marbles[i].x = r;
        }
        if (g_marbles[i].x > MARBLE_BOX_W - r)
        {
            if (g_marbles[i].vx > 0.0f)
            {
                float imp = m * g_marbles[i].vx;
                if (imp > max_impact) max_impact = imp;
                g_marbles[i].vx = -MARBLE_RESTITUTION * g_marbles[i].vx;
            }
            g_marbles[i].x = MARBLE_BOX_W - r;
        }

        // Y walls
        if (g_marbles[i].y < r)
        {
            if (g_marbles[i].vy < 0.0f)
            {
                float imp = m * (-g_marbles[i].vy);
                if (imp > max_impact) max_impact = imp;
                g_marbles[i].vy = MARBLE_RESTITUTION * (-g_marbles[i].vy);
            }
            g_marbles[i].y = r;
        }
        if (g_marbles[i].y > MARBLE_BOX_H - r)
        {
            if (g_marbles[i].vy > 0.0f)
            {
                float imp = m * g_marbles[i].vy;
                if (imp > max_impact) max_impact = imp;
                g_marbles[i].vy = -MARBLE_RESTITUTION * g_marbles[i].vy;
            }
            g_marbles[i].y = MARBLE_BOX_H - r;
        }

        // Z walls
        if (g_marbles[i].z < r)
        {
            if (g_marbles[i].vz < 0.0f)
            {
                float imp = m * (-g_marbles[i].vz);
                if (imp > max_impact) max_impact = imp;
                g_marbles[i].vz = MARBLE_RESTITUTION * (-g_marbles[i].vz);
            }
            g_marbles[i].z = r;
        }
        if (g_marbles[i].z > MARBLE_BOX_D - r)
        {
            if (g_marbles[i].vz > 0.0f)
            {
                float imp = m * g_marbles[i].vz;
                if (imp > max_impact) max_impact = imp;
                g_marbles[i].vz = -MARBLE_RESTITUTION * g_marbles[i].vz;
            }
            g_marbles[i].z = MARBLE_BOX_D - r;
        }
    }

    // 3. Marble-marble collisions (unequal mass, 3D)
    for (int i = 0; i < MARBLE_COUNT - 1; i++)
    {
        for (int j = i + 1; j < MARBLE_COUNT; j++)
        {
            float dx = g_marbles[j].x - g_marbles[i].x;
            float dy = g_marbles[j].y - g_marbles[i].y;
            float dz = g_marbles[j].z - g_marbles[i].z;
            float d2 = dx * dx + dy * dy + dz * dz;

            float rSum = g_marbles[i].r + g_marbles[j].r;
            if (d2 >= rSum * rSum || d2 < 1e-10f)
                continue;

            float d = sqrtf(d2);
            float nx = dx / d;
            float ny = dy / d;
            float nz = dz / d;

            // Relative velocity of j w.r.t. i along normal
            float dvn = (g_marbles[j].vx - g_marbles[i].vx) * nx
                      + (g_marbles[j].vy - g_marbles[i].vy) * ny
                      + (g_marbles[j].vz - g_marbles[i].vz) * nz;

            if (dvn < 0.0f)
            {
                // Impulse for unequal masses
                float inv_mi = 1.0f / g_marbles[i].mass;
                float inv_mj = 1.0f / g_marbles[j].mass;
                float j_imp = -(1.0f + MARBLE_RESTITUTION) * dvn / (inv_mi + inv_mj);

                // Impact metric: reduced mass * relative speed
                float reduced_mass = 1.0f / (inv_mi + inv_mj);
                float imp = reduced_mass * fabsf(dvn);
                if (imp > max_impact) max_impact = imp;

                g_marbles[i].vx -= j_imp * nx * inv_mi;
                g_marbles[i].vy -= j_imp * ny * inv_mi;
                g_marbles[i].vz -= j_imp * nz * inv_mi;
                g_marbles[j].vx += j_imp * nx * inv_mj;
                g_marbles[j].vy += j_imp * ny * inv_mj;
                g_marbles[j].vz += j_imp * nz * inv_mj;
            }

            // Positional correction — distribute by inverse mass
            float overlap = rSum - d;
            float inv_mi = 1.0f / g_marbles[i].mass;
            float inv_mj = 1.0f / g_marbles[j].mass;
            float total_inv = inv_mi + inv_mj;
            float corr_i = overlap * inv_mi / total_inv;
            float corr_j = overlap * inv_mj / total_inv;
            g_marbles[i].x -= corr_i * nx;
            g_marbles[i].y -= corr_i * ny;
            g_marbles[i].z -= corr_i * nz;
            g_marbles[j].x += corr_j * nx;
            g_marbles[j].y += corr_j * ny;
            g_marbles[j].z += corr_j * nz;
        }
    }

    return max_impact;
}
