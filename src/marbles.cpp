#include "marbles.h"
#include "config.h"
#include <math.h>

Marble g_marbles[MARBLE_COUNT];

namespace
{
int s_active_count = MARBLE_DEFAULT_COUNT;
constexpr float kDiameter = 2.0f * MARBLE_RADIUS;
constexpr float kMinCenterDistance = kDiameter + 0.0005f;
constexpr float kPositionEpsilon = 0.00001f;

uint32_t nextRandom(uint32_t &state)
{
    // Xorshift32 is small, deterministic, and sufficient for placement jitter.
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

float randomUnit(uint32_t &state)
{
    return (nextRandom(state) >> 8) * (1.0f / 16777216.0f);
}

void setMarble(int i, float x, float y, float z)
{
    g_marbles[i] = {x, y, z, 0.0f, 0.0f, 0.0f, MARBLE_MASS, MARBLE_RADIUS};
}

bool overlapsExisting(float x, float y, float z, int placed)
{
    const float min_d2 = kMinCenterDistance * kMinCenterDistance;
    for (int i = 0; i < placed; ++i)
    {
        const float dx = x - g_marbles[i].x;
        const float dy = y - g_marbles[i].y;
        const float dz = z - g_marbles[i].z;
        if (dx * dx + dy * dy + dz * dz < min_d2)
            return true;
    }
    return false;
}

void fallbackPosition(int i, uint32_t &rng)
{
    // A 3D lattice leaves at least 12.5 mm between centers before jitter,
    // even across the two rows. This is a safe deterministic fallback for
    // every active count when rejection sampling runs out of attempts.
    const float x[MARBLE_COUNT] = {0.0075f, 0.0225f, 0.0375f, 0.0075f, 0.0225f};
    const float y[MARBLE_COUNT] = {0.0060f, 0.0060f, 0.0060f, 0.0185f, 0.0185f};
    const float z[MARBLE_COUNT] = {0.0060f, 0.0060f, 0.0060f, 0.0185f, 0.0185f};
    const float jitter = 0.0002f;
    setMarble(i,
              fmaxf(MARBLE_RADIUS, fminf(MARBLE_BOX_W - MARBLE_RADIUS,
                                         x[i] + (randomUnit(rng) * 2.0f - 1.0f) * jitter)),
              fmaxf(MARBLE_RADIUS, fminf(MARBLE_BOX_H - MARBLE_RADIUS,
                                         y[i] + (randomUnit(rng) * 2.0f - 1.0f) * jitter)),
              fmaxf(MARBLE_RADIUS, fminf(MARBLE_BOX_D - MARBLE_RADIUS,
                                         z[i] + (randomUnit(rng) * 2.0f - 1.0f) * jitter)));
}

void resetMarble(int i)
{
    uint32_t rng = (uint32_t)(i + 1) * 0x9e3779b9u;
    fallbackPosition(i, rng);
}

bool finiteState(const Marble &m)
{
    return isfinite(m.x) && isfinite(m.y) && isfinite(m.z) &&
           isfinite(m.vx) && isfinite(m.vy) && isfinite(m.vz) &&
           fabsf(m.vx) <= 100.0f && fabsf(m.vy) <= 100.0f && fabsf(m.vz) <= 100.0f;
}

float stepAxis(float &position, float &velocity, float acceleration, float dt,
               float lower, float upper)
{
    if (position <= lower && velocity <= 0.0f && acceleration < 0.0f)
    {
        position = lower;
        velocity = 0.0f;
        return 0.0f;
    }
    if (position >= upper && velocity >= 0.0f && acceleration > 0.0f)
    {
        position = upper;
        velocity = 0.0f;
        return 0.0f;
    }

    velocity += acceleration * dt;
    position += velocity * dt;

    if (position < lower)
    {
        const float impact_speed = velocity < 0.0f ? -velocity : 0.0f;
        position = lower;
        velocity = impact_speed <= MARBLE_REST_SPEED && acceleration < 0.0f
                     ? 0.0f : MARBLE_WALL_RESTITUTION * impact_speed;
        return impact_speed >= MARBLE_MIN_IMPACT_SPEED ? impact_speed : 0.0f;
    }
    if (position > upper)
    {
        const float impact_speed = velocity > 0.0f ? velocity : 0.0f;
        position = upper;
        velocity = impact_speed <= MARBLE_REST_SPEED && acceleration > 0.0f
                     ? 0.0f : -MARBLE_WALL_RESTITUTION * impact_speed;
        return impact_speed >= MARBLE_MIN_IMPACT_SPEED ? impact_speed : 0.0f;
    }
    return 0.0f;
}

void applyWallFriction(Marble &m, float ax, float ay, float az, float dt)
{
    float normal_support = 0.0f;
    bool wall_x = false, wall_y = false, wall_z = false;
    if (m.x <= m.r + kPositionEpsilon && m.vx <= 0.0f && ax < 0.0f)
    { normal_support += -ax; wall_x = true; }
    else if (m.x >= MARBLE_BOX_W - m.r - kPositionEpsilon && m.vx >= 0.0f && ax > 0.0f)
    { normal_support += ax; wall_x = true; }
    if (m.y <= m.r + kPositionEpsilon && m.vy <= 0.0f && ay < 0.0f)
    { normal_support += -ay; wall_y = true; }
    else if (m.y >= MARBLE_BOX_H - m.r - kPositionEpsilon && m.vy >= 0.0f && ay > 0.0f)
    { normal_support += ay; wall_y = true; }
    if (m.z <= m.r + kPositionEpsilon && m.vz <= 0.0f && az < 0.0f)
    { normal_support += -az; wall_z = true; }
    else if (m.z >= MARBLE_BOX_D - m.r - kPositionEpsilon && m.vz >= 0.0f && az > 0.0f)
    { normal_support += az; wall_z = true; }
    if (normal_support <= 0.0f)
        return;

    float tx = wall_x ? 0.0f : m.vx;
    float ty = wall_y ? 0.0f : m.vy;
    float tz = wall_z ? 0.0f : m.vz;
    const float tangent_speed = sqrtf(tx * tx + ty * ty + tz * tz);
    const float friction_delta = MARBLE_WALL_FRICTION * normal_support * dt;
    if (tangent_speed <= friction_delta)
    {
        if (!wall_x) m.vx = 0.0f;
        if (!wall_y) m.vy = 0.0f;
        if (!wall_z) m.vz = 0.0f;
    }
    else
    {
        const float scale = (tangent_speed - friction_delta) / tangent_speed;
        if (!wall_x) m.vx *= scale;
        if (!wall_y) m.vy *= scale;
        if (!wall_z) m.vz *= scale;
    }
}

void constrainToBox(Marble &m)
{
    const float xmin = m.r, xmax = MARBLE_BOX_W - m.r;
    const float ymin = m.r, ymax = MARBLE_BOX_H - m.r;
    const float zmin = m.r, zmax = MARBLE_BOX_D - m.r;
    if (m.x < xmin) { m.x = xmin; if (m.vx < 0.0f) m.vx = 0.0f; }
    if (m.x > xmax) { m.x = xmax; if (m.vx > 0.0f) m.vx = 0.0f; }
    if (m.y < ymin) { m.y = ymin; if (m.vy < 0.0f) m.vy = 0.0f; }
    if (m.y > ymax) { m.y = ymax; if (m.vy > 0.0f) m.vy = 0.0f; }
    if (m.z < zmin) { m.z = zmin; if (m.vz < 0.0f) m.vz = 0.0f; }
    if (m.z > zmax) { m.z = zmax; if (m.vz > 0.0f) m.vz = 0.0f; }
}
} // namespace

void marblesInit(int count, uint32_t seed)
{
    s_active_count = count < 0 ? 0 : count > MARBLE_COUNT ? MARBLE_COUNT : count;
    uint32_t rng = seed ? seed : 0x6d2b79f5u;
    for (int i = 0; i < MARBLE_COUNT; ++i)
        g_marbles[i] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, MARBLE_MASS, MARBLE_RADIUS};

    const float xmin = MARBLE_RADIUS + 0.0003f;
    const float xmax = MARBLE_BOX_W - MARBLE_RADIUS - 0.0003f;
    const float ymin = MARBLE_RADIUS + 0.0003f;
    const float ymax = MARBLE_BOX_H - MARBLE_RADIUS - 0.0003f;
    const float zmin = MARBLE_RADIUS + 0.0003f;
    const float zmax = MARBLE_BOX_D - MARBLE_RADIUS - 0.0003f;

    bool use_fallback_set = false;
    for (int i = 0; i < s_active_count; ++i)
    {
        bool placed = false;
        for (int attempt = 0; attempt < 256; ++attempt)
        {
            const float x = xmin + randomUnit(rng) * (xmax - xmin);
            const float y = ymin + randomUnit(rng) * (ymax - ymin);
            const float z = zmin + randomUnit(rng) * (zmax - zmin);
            if (!overlapsExisting(x, y, z, i))
            {
                setMarble(i, x, y, z);
                placed = true;
                break;
            }
        }
        if (!placed)
        {
            use_fallback_set = true;
            break;
        }
    }

    if (use_fallback_set)
    {
        // Do not mix random placements with lattice placements: that can put
        // a fallback ball on top of a previously randomized one. Replace the
        // entire active set so all pairwise spacing guarantees hold.
        uint32_t fallback_rng = seed ? seed ^ 0xa511e9b3u : 0x6d2b79f5u;
        for (int i = 0; i < s_active_count; ++i)
            fallbackPosition(i, fallback_rng);
    }
}

int marblesCount()
{
    return s_active_count;
}

float marblesStep(float dt,
                  float grav_x_mps2, float grav_y_mps2, float grav_z_mps2,
                  float accel_x_mps2, float accel_y_mps2, float accel_z_mps2)
{
    if (s_active_count == 0)
        return 0.0f;
    if (!isfinite(dt) || dt <= 0.0f || dt > 0.05f ||
        !isfinite(grav_x_mps2) || !isfinite(grav_y_mps2) || !isfinite(grav_z_mps2) ||
        !isfinite(accel_x_mps2) || !isfinite(accel_y_mps2) || !isfinite(accel_z_mps2) ||
        fabsf(grav_x_mps2) > 100.0f || fabsf(grav_y_mps2) > 100.0f || fabsf(grav_z_mps2) > 100.0f ||
        fabsf(accel_x_mps2) > 100.0f || fabsf(accel_y_mps2) > 100.0f || fabsf(accel_z_mps2) > 100.0f)
        return 0.0f;

    for (int i = 0; i < s_active_count; ++i)
        if (!finiteState(g_marbles[i]))
            resetMarble(i);

    float wall_impulse = 0.0f;
    const float ax = grav_x_mps2 - accel_x_mps2;
    const float ay = grav_y_mps2 - accel_y_mps2;
    const float az = grav_z_mps2 - accel_z_mps2;
    for (int i = 0; i < s_active_count; ++i)
    {
        Marble &m = g_marbles[i];
        wall_impulse += stepAxis(m.x, m.vx, ax, dt, m.r, MARBLE_BOX_W - m.r);
        wall_impulse += stepAxis(m.y, m.vy, ay, dt, m.r, MARBLE_BOX_H - m.r);
        wall_impulse += stepAxis(m.z, m.vz, az, dt, m.r, MARBLE_BOX_D - m.r);
        applyWallFriction(m, ax, ay, az, dt);
    }

    float pair_cues = 0.0f;
    const float diameter2 = kDiameter * kDiameter;
    for (int i = 0; i < s_active_count - 1; ++i)
    {
        for (int j = i + 1; j < s_active_count; ++j)
        {
            Marble &a = g_marbles[i];
            Marble &b = g_marbles[j];
            float dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
            const float d2 = dx * dx + dy * dy + dz * dz;
            if (d2 >= diameter2)
                continue;

            float d, nx, ny, nz;
            if (d2 < 1e-10f)
            {
                d = 0.0f;
                nx = (i + j) & 1 ? 1.0f : -1.0f;
                ny = nz = 0.0f;
            }
            else
            {
                d = sqrtf(d2);
                const float inv_d = 1.0f / d;
                nx = dx * inv_d; ny = dy * inv_d; nz = dz * inv_d;
            }

            const float rvx = b.vx - a.vx;
            const float rvy = b.vy - a.vy;
            const float rvz = b.vz - a.vz;
            const float dvn = rvx * nx + rvy * ny + rvz * nz;
            if (dvn < 0.0f)
            {
                const float closing_speed = -dvn;
                const float restitution = closing_speed <= MARBLE_REST_SPEED
                                            ? 0.0f : MARBLE_PAIR_RESTITUTION;
                const float impulse = 0.5f * (1.0f + restitution) * closing_speed;
                a.vx -= impulse * nx; a.vy -= impulse * ny; a.vz -= impulse * nz;
                b.vx += impulse * nx; b.vy += impulse * ny; b.vz += impulse * nz;
                if (closing_speed >= MARBLE_MIN_IMPACT_SPEED)
                    pair_cues += 0.5f * closing_speed * MARBLE_PAIR_HAPTIC_GAIN;

                // Plastic-wall/ball contact loses tangential slip as well as
                // normal bounce energy; equal masses share the correction.
                const float tvx = rvx - dvn * nx;
                const float tvy = rvy - dvn * ny;
                const float tvz = rvz - dvn * nz;
                const float tangential_loss = 0.25f;
                a.vx += 0.5f * tangential_loss * tvx;
                a.vy += 0.5f * tangential_loss * tvy;
                a.vz += 0.5f * tangential_loss * tvz;
                b.vx -= 0.5f * tangential_loss * tvx;
                b.vy -= 0.5f * tangential_loss * tvy;
                b.vz -= 0.5f * tangential_loss * tvz;
            }

            const float overlap = kDiameter - d;
            const float correction = 0.5f * overlap;
            a.x -= correction * nx; a.y -= correction * ny; a.z -= correction * nz;
            b.x += correction * nx; b.y += correction * ny; b.z += correction * nz;
            constrainToBox(a);
            constrainToBox(b);
        }
    }

    return wall_impulse + pair_cues;
}

float marblesContactSpeed()
{
    float total_mass = 0.0f, weighted_speed2 = 0.0f;
    for (int i = 0; i < s_active_count; ++i)
    {
        const Marble &m = g_marbles[i];
        const bool wall_x = m.x <= m.r + kPositionEpsilon || m.x >= MARBLE_BOX_W - m.r - kPositionEpsilon;
        const bool wall_y = m.y <= m.r + kPositionEpsilon || m.y >= MARBLE_BOX_H - m.r - kPositionEpsilon;
        const bool wall_z = m.z <= m.r + kPositionEpsilon || m.z >= MARBLE_BOX_D - m.r - kPositionEpsilon;
        if (!wall_x && !wall_y && !wall_z)
            continue;
        const float vx = wall_x ? 0.0f : m.vx;
        const float vy = wall_y ? 0.0f : m.vy;
        const float vz = wall_z ? 0.0f : m.vz;
        total_mass += MARBLE_MASS;
        weighted_speed2 += MARBLE_MASS * (vx * vx + vy * vy + vz * vz);
    }
    return total_mass > 0.0f ? sqrtf(weighted_speed2 / total_mass) : 0.0f;
}

float marblesImpactEnvelopeStep(float current, float impact_momentum, float dt)
{
    if (!isfinite(current) || !isfinite(impact_momentum) || !isfinite(dt) || dt < 0.0f)
        return 0.0f;
    const float decay = fmaxf(0.0f, 1.0f - dt / 0.025f);
    const float peak = fmaxf(0.0f, fminf(impact_momentum / MARBLE_IMPACT_REF, 1.0f));
    return fmaxf(current * decay, peak);
}
