#include "marbles.h"
#include "config.h"
#include <math.h>

Marble g_marbles[MARBLE_COUNT];

// Effective boundary (marble centre must stay inside)
static constexpr float kMinX = MARBLE_RAD;
static constexpr float kMaxX = MARBLE_BOX_W - MARBLE_RAD;
static constexpr float kMinY = MARBLE_RAD;
static constexpr float kMaxY = MARBLE_BOX_H - MARBLE_RAD;
static constexpr float kMinZ = MARBLE_RAD;
static constexpr float kMaxZ = MARBLE_BOX_D - MARBLE_RAD;

void marblesInit()
{
    // 3 marbles at different positions
    g_marbles[0].x = MARBLE_BOX_W * 0.25f;
    g_marbles[0].y = MARBLE_BOX_H * 0.25f;
    g_marbles[0].z = MARBLE_BOX_D * 0.5f;

    g_marbles[1].x = MARBLE_BOX_W * 0.75f;
    g_marbles[1].y = MARBLE_BOX_H * 0.25f;
    g_marbles[1].z = MARBLE_BOX_D * 0.5f;

    g_marbles[2].x = MARBLE_BOX_W * 0.5f;
    g_marbles[2].y = MARBLE_BOX_H * 0.75f;
    g_marbles[2].z = MARBLE_BOX_D * 0.5f;

    for (int i = 0; i < MARBLE_COUNT; i++)
    {
        g_marbles[i].vx = 0.0f;
        g_marbles[i].vy = 0.0f;
        g_marbles[i].vz = 0.0f;
    }
}

float marblesStep(float dt, float accel_x_mps2, float accel_y_mps2, float accel_z_mps2, float grav_z_mps2)
{
    float max_impact = 0.0f;

    // 1. Apply box acceleration (opposite direction - marbles move relative to box)
    //    and gravity in Z direction
    for (int i = 0; i < MARBLE_COUNT; i++)
    {
        g_marbles[i].vx -= accel_x_mps2 * dt; // opposite of box motion
        g_marbles[i].vy -= accel_y_mps2 * dt; // opposite of box motion
        g_marbles[i].vz += grav_z_mps2 * dt;  // gravity pulls down
        g_marbles[i].x += g_marbles[i].vx * dt;
        g_marbles[i].y += g_marbles[i].vy * dt;
        g_marbles[i].z += g_marbles[i].vz * dt;
    }

    // 2. Wall collisions with restitution (X walls)
    for (int i = 0; i < MARBLE_COUNT; i++)
    {
        if (g_marbles[i].x < kMinX)
        {
            if (g_marbles[i].vx < 0.0f)
            {
                float spd = -g_marbles[i].vx;
                if (spd > max_impact)
                    max_impact = spd;
                g_marbles[i].vx = MARBLE_RESTITUTION * spd;
            }
            g_marbles[i].x = kMinX;
        }
        if (g_marbles[i].x > kMaxX)
        {
            if (g_marbles[i].vx > 0.0f)
            {
                float spd = g_marbles[i].vx;
                if (spd > max_impact)
                    max_impact = spd;
                g_marbles[i].vx = -MARBLE_RESTITUTION * spd;
            }
            g_marbles[i].x = kMaxX;
        }
        // Y walls
        if (g_marbles[i].y < kMinY)
        {
            if (g_marbles[i].vy < 0.0f)
            {
                float spd = -g_marbles[i].vy;
                if (spd > max_impact)
                    max_impact = spd;
                g_marbles[i].vy = MARBLE_RESTITUTION * spd;
            }
            g_marbles[i].y = kMinY;
        }
        if (g_marbles[i].y > kMaxY)
        {
            if (g_marbles[i].vy > 0.0f)
            {
                float spd = g_marbles[i].vy;
                if (spd > max_impact)
                    max_impact = spd;
                g_marbles[i].vy = -MARBLE_RESTITUTION * spd;
            }
            g_marbles[i].y = kMaxY;
        }
        // Z walls
        if (g_marbles[i].z < kMinZ)
        {
            if (g_marbles[i].vz < 0.0f)
            {
                float spd = -g_marbles[i].vz;
                if (spd > max_impact)
                    max_impact = spd;
                g_marbles[i].vz = MARBLE_RESTITUTION * spd;
            }
            g_marbles[i].z = kMinZ;
        }
        if (g_marbles[i].z > kMaxZ)
        {
            if (g_marbles[i].vz > 0.0f)
            {
                float spd = g_marbles[i].vz;
                if (spd > max_impact)
                    max_impact = spd;
                g_marbles[i].vz = -MARBLE_RESTITUTION * spd;
            }
            g_marbles[i].z = kMaxZ;
        }
    }

    // 3. Marble-marble elastic collisions (equal mass, O(n²), 3D)
    const float diam = 2.0f * MARBLE_RAD;
    const float diam2 = diam * diam;

    for (int i = 0; i < MARBLE_COUNT - 1; i++)
    {
        for (int j = i + 1; j < MARBLE_COUNT; j++)
        {
            float dx = g_marbles[j].x - g_marbles[i].x;
            float dy = g_marbles[j].y - g_marbles[i].y;
            float dz = g_marbles[j].z - g_marbles[i].z;
            float d2 = dx * dx + dy * dy + dz * dz;
            if (d2 >= diam2 || d2 < 1e-10f)
                continue;

            float d = sqrtf(d2);
            float nx = dx / d;
            float ny = dy / d;
            float nz = dz / d;

            // Relative velocity of j w.r.t. i along n (n points i→j)
            float dvn = (g_marbles[j].vx - g_marbles[i].vx) * nx + (g_marbles[j].vy - g_marbles[i].vy) * ny + (g_marbles[j].vz - g_marbles[i].vz) * nz;

            if (dvn < 0.0f)
            {
                // Approaching — apply impulse
                float imp = (1.0f + MARBLE_RESTITUTION) * 0.5f * dvn;
                float spd = fabsf(dvn);
                if (spd > max_impact)
                    max_impact = spd;
                g_marbles[i].vx += imp * nx;
                g_marbles[i].vy += imp * ny;
                g_marbles[i].vz += imp * nz;
                g_marbles[j].vx -= imp * nx;
                g_marbles[j].vy -= imp * ny;
                g_marbles[j].vz -= imp * nz;
            }

            // Positional correction — push apart to remove overlap
            float overlap = (diam - d) * 0.5f;
            g_marbles[i].x -= overlap * nx;
            g_marbles[i].y -= overlap * ny;
            g_marbles[i].z -= overlap * nz;
            g_marbles[j].x += overlap * nx;
            g_marbles[j].y += overlap * ny;
            g_marbles[j].z += overlap * nz;
        }
    }

    return max_impact;
}
