#include "physics.h"
#include "marbles.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>

namespace {

void require(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(EXIT_FAILURE);
    }
}

void near(float actual, float expected, float tolerance, const char *message)
{
    if (std::fabs(actual - expected) > tolerance) {
        std::fprintf(stderr, "FAIL: %s (got %.9f, expected %.9f ± %.9f)\n",
                     message, actual, expected, tolerance);
        std::exit(EXIT_FAILURE);
    }
}

void reset(SimMode mode, bool rebound = true)
{
    g_cavity = 1.0f;
    g_sim_mode = mode;
    g_rebound = rebound;
    physicsInit();
}

void testStaticFrictionStopsResidualVelocity()
{
    reset(SimMode::SLIDING);
    phys_x_pos = 0.5f;
    phys_x_vel = 0.005f;
    const ImpactInfo impact = physicsStep(0.001f, 0.10f, 0.0f);

    near(phys_x_vel, 0.0f, 0.0f, "static friction clears residual velocity");
    near(phys_x_pos, 0.5f, 0.0f, "static friction prevents residual drift");
    require(!impact.hit, "sticking away from a wall is not an impact");
}

void testStaticFrictionUsesEffectiveDrive()
{
    reset(SimMode::SLIDING);
    phys_x_pos = 0.5f;
    // Gravity drive is 1.96 m/s², but 1.0 m/s² of box acceleration leaves
    // only 0.96 m/s², below the approximately 1.92 m/s² static limit.
    physicsStep(0.001f, 0.20f, 1.0f);
    near(phys_x_vel, 0.0f, 0.0f, "static friction includes tube acceleration");
}

void testSlidingFrictionOpposesVelocity()
{
    reset(SimMode::SLIDING);
    phys_x_pos = 0.5f;
    phys_x_vel = -0.5f;
    physicsStep(0.001f, 0.30f, 0.0f);
    require(phys_x_vel > -0.5f,
            "positive drive and friction increase velocity while ball moves backward");
    require(phys_x_vel < 0.0f, "one small step does not reverse the ball");
}

void testKineticFrictionZeroCrossingDoesNotReverse()
{
    reset(SimMode::SLIDING);
    phys_x_pos = 0.5f;
    phys_x_vel = 0.02f;
    // With alpha=0.1 the kinetic deceleration is about -0.97 m/s². This
    // deliberately large step overshoots zero unless the integrator clamps it.
    physicsStep(0.05f, 0.10f, 0.0f);
    require(phys_x_vel >= 0.0f, "kinetic friction must not reverse velocity at zero crossing");
}

void testRollingTrajectoryAtPhysicsRates()
{
    for (float dt : {0.001f, 0.0005f}) {
        reset(SimMode::ROLLING);
        g_cavity = 10.0f;
        physicsInit();
        const int steps = dt > 0.00075f ? 1000 : 2000;
        for (int i = 0; i < steps; ++i)
            physicsStep(dt, 0.20f, 0.0f);

        // Gravity less the constant rolling resistance, opposed by linear
        // drag: v(t) = (a/k)(1 - e^-kt). Both rates must land on the same
        // closed-form trajectory, which confirms the 1/2 kHz time scale.
        const float elapsed = steps * dt;
        const float a = G_FACTOR * (0.20f - ROLLING_RESISTANCE * std::sqrt(1.0f - 0.04f));
        const float k = ROLLING_DRAG, decay = std::exp(-k * elapsed);
        near(phys_x_vel, a / k * (1.0f - decay), 2e-3f,
             "rolling velocity integrates correctly at 1 and 2 kHz");
        near(phys_x_pos, 5.0f + a / k * (elapsed - (1.0f - decay) / k),
             0.002f, "rolling position remains physically scaled at 1 and 2 kHz");
    }
}

void testSecondImpactTimingAtPhysicsRates()
{
    constexpr float sin_alpha = 0.20f;
    constexpr float acceleration = 7.0f * sin_alpha;
    for (float dt : {0.001f, 0.0005f}) {
        reset(SimMode::TEXTURED);
        phys_x_pos = 0.9f;
        phys_x_vel = 0.3f;

        int first_tick = -1;
        float first_impact_speed = 0.0f;
        int second_tick = -1;
        for (int tick = 0; tick < 2000; ++tick) {
            const ImpactInfo impact = physicsStep(dt, sin_alpha, 0.0f);
            if (!impact.hit)
                continue;
            if (first_tick < 0) {
                first_tick = tick;
                first_impact_speed = impact.speed;
            } else {
                second_tick = tick;
                break;
            }
        }

        require(first_tick >= 0 && second_tick > first_tick,
                "TEXTURED mode reports successive wall impacts under steady tilt");
        const float measured = (second_tick - first_tick) * dt;
        // Time for the ball to leave the wall at e * impact speed and come
        // back, under gravity, rolling resistance and drag (fine-step reference).
        auto flight = [&](float e) {
            const double resistance = ROLLING_RESISTANCE * G_FACTOR * std::sqrt(1.0 - sin_alpha * sin_alpha);
            double x = 0, v = -e * first_impact_speed, t = 0;
            const double h = 1e-5;
            do {
                v += (acceleration - (v > 0 ? resistance : -resistance) - ROLLING_DRAG * v) * h;
                x += v * h;
                t += h;
            } while (x < 0 && t < 5);
            return (float)t;
        };
        const float expected_at_e = flight(RESTITUTION_E);
        const float expected_at_e_half = flight(0.50f);
        std::printf("second impact dt=%.4f s: measured %.4f s, e=0.30 prediction %.4f s, e=0.50 %.4f s\n",
                    dt, measured, expected_at_e, expected_at_e_half);

        // The integrator detects wall crossing on a tick boundary and uses
        // that tick's end velocity, so allow two samples for event quantization.
        // (plus 1.5 ms for the reference's simpler treatment of the bounce tick).
        near(measured, expected_at_e, 2.1f * dt + 0.0015f,
             "second-impact interval follows restitution and the rolling law at both rates");
        near(measured / expected_at_e_half, expected_at_e / expected_at_e_half, 0.03f,
             "restitution 0.30 returns about 40 percent sooner than restitution 0.50");
    }
}

struct PhysicsTrace
{
    float pos[256];
    float vel[256];
    bool hit[256];
    bool left[256];
    float impact_speed[256];
};

void recordRollingTrajectory(SimMode mode, PhysicsTrace &trace)
{
    reset(mode);
    g_cavity = 10.0f;
    physicsInit();
    phys_x_pos = 5.0f;
    phys_x_vel = -0.3f;
    for (int i = 0; i < 256; ++i) {
        const float sin_alpha = static_cast<float>((i % 81) - 40) * 0.0025f;
        const float lin_accel_x = static_cast<float>((i % 17) - 8) * 0.03f;
        const ImpactInfo impact = physicsStep(0.001f, sin_alpha, lin_accel_x);
        trace.pos[i] = phys_x_pos;
        trace.vel[i] = phys_x_vel;
        trace.hit[i] = impact.hit;
        trace.left[i] = impact.left;
        trace.impact_speed[i] = impact.speed;
    }
}

void testTexturedModeMatchesRollingPhysics()
{
    PhysicsTrace rolling{};
    PhysicsTrace textured{};
    recordRollingTrajectory(SimMode::ROLLING, rolling);
    recordRollingTrajectory(SimMode::TEXTURED, textured);

    for (int i = 0; i < 256; ++i) {
        near(textured.pos[i], rolling.pos[i], 0.0f,
             "TEXTURED position follows the ROLLING physical model");
        near(textured.vel[i], rolling.vel[i], 0.0f,
             "TEXTURED velocity follows the ROLLING physical model");
        require(textured.hit[i] == rolling.hit[i],
                "TEXTURED impact flags follow the ROLLING physical model");
        require(textured.left[i] == rolling.left[i],
                "TEXTURED impact sides follow the ROLLING physical model");
        near(textured.impact_speed[i], rolling.impact_speed[i], 0.0f,
             "TEXTURED impact speed follows the ROLLING physical model");
    }
}

void testNoReboundStillReportsOneTactileImpact()
{
    reset(SimMode::ROLLING, false);
    phys_x_pos = 0.0002f;
    phys_x_vel = -1.0f;

    const ImpactInfo impact = physicsStep(0.001f, 0.0f, 0.0f);
    require(impact.hit && impact.left, "no-rebound left-wall contact reports an impact");
    // One 1 ms tick of rolling resistance and drag acts before the wall.
    near(impact.speed, 1.0f, 0.005f, "impact speed is captured before stopping");
    near(phys_x_pos, 0.0f, 0.0f, "no-rebound contact clamps position");
    near(phys_x_vel, 0.0f, 0.0f, "no-rebound contact stops velocity");

    const ImpactInfo resting = physicsStep(0.001f, 0.0f, 0.0f);
    require(!resting.hit, "resting against the wall does not retrigger impact");
}

void testReboundReportsImpactAndReverses()
{
    reset(SimMode::ROLLING, true);
    phys_x_pos = 0.0002f;
    phys_x_vel = -1.0f;
    const ImpactInfo impact = physicsStep(0.001f, 0.0f, 0.0f);

    require(impact.hit && impact.left, "rebound wall contact reports impact");
    near(impact.speed, 1.0f, 0.005f, "rebound impact speed is preserved"); // less one tick of drag
    require(phys_x_vel > 0.0f, "rebound reverses velocity into the cavity");
}

void testMarbleAccelerationAxesAreIndependent()
{
    marblesInit(1, 31);
    const float x0 = g_marbles[0].x;
    const float y0 = g_marbles[0].y;
    const float z0 = g_marbles[0].z;

    marblesStep(0.001f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f);
    near(g_marbles[0].vx, 0.0f, 0.0f, "Z acceleration does not change X velocity");
    near(g_marbles[0].vy, 0.0f, 0.0f, "Z acceleration does not change Y velocity");
    near(g_marbles[0].vz, -0.002f, 1e-8f, "box Z acceleration opposes box motion");
    near(g_marbles[0].x, x0, 0.0f, "Z acceleration does not move X position");
    near(g_marbles[0].y, y0, 0.0f, "Z acceleration does not move Y position");
    near(g_marbles[0].z, z0 - 0.000002f, 1e-8f, "Z acceleration moves Z position");

    marblesInit(1, 31);
    marblesStep(0.001f, 0.0f, 0.0f, 9.8f, 0.0f, 0.0f, 0.0f);
    near(g_marbles[0].vz, 0.0098f, 1e-8f, "positive Z gravity accelerates marble downward");
}

void testMarbleGravityActsOnAllBodyAxes()
{
    marblesInit(1, 31);
    const float x0 = g_marbles[0].x;
    const float y0 = g_marbles[0].y;
    const float z0 = g_marbles[0].z;
    marblesStep(0.01f, 2.0f, -3.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    near(g_marbles[0].vx, 0.02f, 1e-7f, "positive X gravity accelerates marble in positive X");
    near(g_marbles[0].vy, -0.03f, 1e-7f, "negative Y gravity accelerates marble in negative Y");
    near(g_marbles[0].vz, 0.0f, 0.0f, "X/Y gravity does not alter Z velocity");
    near(g_marbles[0].x, x0 + 0.0002f, 1e-7f, "X gravity moves marble in X");
    near(g_marbles[0].y, y0 - 0.0003f, 1e-7f, "Y gravity moves marble in Y");
    near(g_marbles[0].z, z0, 0.0f, "X/Y gravity does not move marble in Z");
}

void testMarbleRestingContactDoesNotBuzz()
{
    marblesInit(5, 31);
    Marble &m = g_marbles[0];
    const float resting[5][3] = {
        {0.006f, 0.006f, 0.020f}, {0.018f, 0.006f, 0.020f},
        {0.030f, 0.006f, 0.020f}, {0.042f, 0.006f, 0.020f},
        {0.024f, 0.018f, 0.020f}
    };
    for (int i = 0; i < marblesCount(); ++i) {
        g_marbles[i].x = resting[i][0];
        g_marbles[i].y = resting[i][1];
        g_marbles[i].z = resting[i][2];
        g_marbles[i].vx = g_marbles[i].vy = g_marbles[i].vz = 0.0f;
    }

    float max_impact = 0.0f;
    for (int i = 0; i < 4000; ++i)
        max_impact = fmaxf(max_impact,
                          marblesStep(0.0005f, 0.0f, 0.0f, 9.80665f,
                                      0.0f, 0.0f, 0.0f));
    near(max_impact, 0.0f, 0.0f, "settled wall contacts do not generate haptic microimpacts");
    near(m.z, MARBLE_BOX_D - m.r, 0.0f, "marble remains resting against Z wall");
    near(m.vz, 0.0f, 0.0f, "resting Z contact has no residual velocity");
    near(m.vx, 0.0f, 0.0f, "resting Z contact leaves X velocity unchanged");
    near(m.vy, 0.0f, 0.0f, "resting Z contact leaves Y velocity unchanged");
}

void testSupportedMarbleRespondsToTangentialShake()
{
    marblesInit(1, 3);
    Marble &m = g_marbles[0];
    m.x = 0.006f;
    m.y = 0.0125f;
    m.z = MARBLE_BOX_D - m.r;

    float rest_peak = 0.0f;
    for (int i = 0; i < 2000; ++i)
        rest_peak = fmaxf(rest_peak,
                          marblesStep(0.0005f, 0.0f, 0.0f, 9.80665f,
                                      0.0f, 0.0f, 0.0f));
    near(rest_peak, 0.0f, 0.0f,
         "normal gravity alone leaves the supported marble quiet");
    near(m.x, 0.006f, 0.0f, "normal gravity alone does not drift tangentially");

    // A 2 m/s² box acceleration is only slightly above the configured
    // Coulomb limit (mu*g ≈ 1.96 m/s²). It must accumulate motion over
    // multiple ticks instead of being erased by a per-tick rest-speed cutoff.
    for (int i = 0; i < 200; ++i)
        marblesStep(0.0005f, 0.0f, 0.0f, 9.80665f,
                    2.0f, 0.0f, 0.0f);
    require(m.x < 0.006f && m.vx < 0.0f,
            "sustained tangential acceleration overcomes Coulomb wall friction");

    bool reached_wall = false;
    for (int i = 0; i < 3000; ++i) {
        marblesStep(0.0005f, 0.0f, 0.0f, 9.80665f,
                    2.0f, 0.0f, 0.0f);
        if (m.x <= m.r) {
            reached_wall = true;
            break;
        }
    }
    require(reached_wall,
            "sustained shaking eventually carries the supported marble to a wall");
    require(isfinite(m.x) && isfinite(m.vx),
            "supported tangential motion remains finite through wall contact");
}

void testMarbleLongRunRemainsFiniteAndInsideBox()
{
    marblesInit(5, 2026);
    float max_impact = 0.0f;
    for (int i = 0; i < 20000; ++i)
    {
        const float t = i * 0.0005f;
        const float impact = marblesStep(0.0005f,
                                         7.0f * sinf(4.0f * t),
                                         5.0f * cosf(3.0f * t),
                                         8.0f,
                                         2.0f * sinf(7.0f * t),
                                         2.0f * cosf(5.0f * t),
                                         1.0f * sinf(2.0f * t));
        if (impact > max_impact) max_impact = impact;
    }
    require(isfinite(max_impact) && max_impact >= 0.0f && max_impact <= 20.0f,
            "long-running shake produces finite bounded impact momentum");
    for (int i = 0; i < marblesCount(); ++i)
    {
        const Marble &m = g_marbles[i];
        require(isfinite(m.x) && isfinite(m.y) && isfinite(m.z) &&
                isfinite(m.vx) && isfinite(m.vy) && isfinite(m.vz),
                "long-running shake keeps all marble state finite");
        require(m.x >= m.r && m.x <= MARBLE_BOX_W - m.r &&
                m.y >= m.r && m.y <= MARBLE_BOX_H - m.r &&
                m.z >= m.r && m.z <= MARBLE_BOX_D - m.r,
                "long-running shake keeps marbles inside the box");
    }
}

void testMarbleMotionScalarTracksActualWeightedSpeed()
{
    marblesInit(3, 7);
    require(marblesContactSpeed() == 0.0f, "stationary marbles produce a silent motion scalar");
    g_marbles[0].vx = 0.4f;
    require(marblesContactSpeed() == 0.0f, "free-flight motion does not drive surface texture");
    g_marbles[0].x = g_marbles[0].r;
    g_marbles[1].x = g_marbles[1].r;
    g_marbles[2].x = g_marbles[2].r;
    near(marblesContactSpeed(), 0.0f, 1e-6f,
         "normal wall impact motion is excluded from rolling texture");
    g_marbles[0].vx = 0.0f;
    g_marbles[0].vy = 0.4f;
    const float expected = sqrtf(0.16f / 3.0f);
    near(marblesContactSpeed(), expected, 1e-6f,
         "marble texture uses mass-weighted tangential speed at box contact");
}

void testMarbleImpactTelemetryEnvelopeKeepsPeakBriefly()
{
    float env = marblesImpactEnvelopeStep(0.0f, MARBLE_IMPACT_REF * 0.8f, 0.0005f);
    near(env, 0.8f, 1e-6f, "impact telemetry captures normalized contact peak");
    near(marblesImpactEnvelopeStep(env, 0.0f, 0.0005f), 0.784f, 1e-6f,
         "telemetry-only contact peak decays over roughly 25 ms");
    near(marblesImpactEnvelopeStep(env, MARBLE_IMPACT_REF * 1.2f, 0.0005f), 1.0f, 1e-6f,
         "impact telemetry envelope is bounded at one");
}

void testMarbleBounceTrainDampsAtConfiguredFeelRate()
{
    marblesInit(1, 17);
    Marble &m = g_marbles[0];
    m.x = MARBLE_BOX_W * 0.5f;
    m.vx = 1.0f;

    float impacts[4] = {};
    int count = 0;
    for (int i = 0; i < 2000 && count < 4; ++i)
    {
        const float momentum = marblesStep(0.001f, 0.0f, 0.0f, 0.0f,
                                           0.0f, 0.0f, 0.0f);
        if (momentum > 0.0f)
            impacts[count++] = momentum / m.mass;
    }
    require(count == 4, "isolated marble completes four wall bounces");
    near(impacts[0], 1.0f, 1e-4f, "first marble wall impact preserves incoming speed");
    for (int i = 1; i < count; ++i)
        near(impacts[i] / impacts[i - 1], MARBLE_WALL_RESTITUTION, 1e-4f,
             "successive wall impacts follow configured perceptual restitution");
}

void testMarbleCountClampsAndEmptyIsInert()
{
    marblesInit();
    require(marblesCount() == MARBLE_DEFAULT_COUNT,
            "default init preserves the three-marble call-site behavior");
    marblesInit(-4, 2);
    require(marblesCount() == 0, "negative requested count clamps to empty");
    near(marblesStep(0.001f, NAN, NAN, NAN, NAN, NAN, NAN), 0.0f, 0.0f,
         "empty apparatus returns zero without evaluating motion");
    marblesInit(MARBLE_COUNT + 4, 2);
    require(marblesCount() == MARBLE_COUNT,
            "requested count clamps to five-ball capacity");
}

void testMarbleSeededPlacementIsReproducibleAndSeparated()
{
    Marble first[MARBLE_COUNT];
    marblesInit(MARBLE_COUNT, 0x13579bdfu);
    for (int i = 0; i < marblesCount(); ++i)
        first[i] = g_marbles[i];
    marblesInit(MARBLE_COUNT, 0x13579bdfu);
    const float min_distance = 2.0f * MARBLE_RADIUS;
    for (int i = 0; i < marblesCount(); ++i) {
        const Marble &m = g_marbles[i];
        near(m.x, first[i].x, 0.0f, "same seed reproduces marble X placement");
        near(m.y, first[i].y, 0.0f, "same seed reproduces marble Y placement");
        near(m.z, first[i].z, 0.0f, "same seed reproduces marble Z placement");
        require(m.x >= m.r && m.x <= MARBLE_BOX_W - m.r &&
                m.y >= m.r && m.y <= MARBLE_BOX_H - m.r &&
                m.z >= m.r && m.z <= MARBLE_BOX_D - m.r,
                "seeded spawn remains inside the enclosure");
        for (int j = 0; j < i; ++j) {
            const float dx = m.x - g_marbles[j].x;
            const float dy = m.y - g_marbles[j].y;
            const float dz = m.z - g_marbles[j].z;
            require(dx * dx + dy * dy + dz * dz >= min_distance * min_distance,
                    "seeded spawn does not overlap another marble");
        }
    }
    marblesInit(MARBLE_COUNT, 0x2468ace1u);
    bool differs = false;
    for (int i = 0; i < marblesCount(); ++i)
        differs |= std::fabs(g_marbles[i].x - first[i].x) > 1e-6f ||
                   std::fabs(g_marbles[i].y - first[i].y) > 1e-6f ||
                   std::fabs(g_marbles[i].z - first[i].z) > 1e-6f;
    require(differs, "different seed changes the initial arrangement");
}

void testManyMarbleSeedsNeverSpawnOverlapping()
{
    const float min_distance = 2.0f * MARBLE_RADIUS;
    for (uint32_t seed = 1; seed <= 2048; ++seed) {
        marblesInit(MARBLE_COUNT, seed * 0x9e3779b9u);
        require(marblesCount() == MARBLE_COUNT,
                "five-ball seed sweep retains the requested active count");
        for (int i = 0; i < marblesCount(); ++i) {
            const Marble &m = g_marbles[i];
            require(m.x >= m.r && m.x <= MARBLE_BOX_W - m.r &&
                    m.y >= m.r && m.y <= MARBLE_BOX_H - m.r &&
                    m.z >= m.r && m.z <= MARBLE_BOX_D - m.r,
                    "seed sweep keeps every marble inside the box");
            for (int j = 0; j < i; ++j) {
                const float dx = m.x - g_marbles[j].x;
                const float dy = m.y - g_marbles[j].y;
                const float dz = m.z - g_marbles[j].z;
                require(dx * dx + dy * dy + dz * dz >= min_distance * min_distance,
                        "seed sweep never initializes overlapping marbles");
            }
        }
    }
}

void testWallImpulseSumsAndPairCueIsReduced()
{
    marblesInit(2, 11);
    for (int i = 0; i < 2; ++i) {
        g_marbles[i].x = g_marbles[i].r;
        g_marbles[i].y = i == 0 ? 0.008f : 0.017f;
        g_marbles[i].z = 0.0125f;
        g_marbles[i].vx = -1.0f;
        g_marbles[i].vy = g_marbles[i].vz = 0.0f;
    }
    const float summed_walls = marblesStep(0.001f, 0, 0, 0, 0, 0, 0);
    near(summed_walls, 2.0f, 1e-5f,
         "simultaneous distinct wall hits sum both incoming impulses");

    marblesInit(2, 11);
    g_marbles[0].x = 0.0199f;
    g_marbles[1].x = 0.0297f;
    g_marbles[0].y = g_marbles[1].y = 0.0125f;
    g_marbles[0].z = g_marbles[1].z = 0.0125f;
    g_marbles[0].vx = 0.5f;
    g_marbles[1].vx = -0.5f;
    const float pair_cue = marblesStep(0.001f, 0, 0, 0, 0, 0, 0);
    near(pair_cue, 0.15f, 1e-5f,
         "equal-mass ball collision produces the configured reduced tactile cue");
    require(pair_cue < summed_walls,
            "inter-ball collision cue is weaker than summed enclosure impacts");
}

void testShakeCountChangesDeterministicRattleStream()
{
    auto run = [](int count, float *energy, int *events) {
        marblesInit(count, 0x51a7u);
        *energy = 0.0f;
        *events = 0;
        for (int i = 0; i < 6000; ++i) {
            const float t = static_cast<float>(i) * 0.0005f;
            const float cue = marblesStep(0.0005f,
                                          0.0f, 0.0f, 9.80665f,
                                          8.0f * sinf(2.0f * 3.14159265f * 2.3f * t),
                                          5.0f * sinf(2.0f * 3.14159265f * 1.7f * t + 0.4f),
                                          3.0f * cosf(2.0f * 3.14159265f * 1.1f * t));
            *energy += cue;
            if (cue > 0.0f) ++*events;
        }
    };
    float energy_one, energy_five;
    int events_one, events_five;
    run(1, &energy_one, &events_one);
    run(5, &energy_five, &events_five);
    require(events_one > 0 && events_five > 0,
            "both one- and five-marble apparatuses rattle under full-axis shaking");
    require(events_one != events_five || std::fabs(energy_one - energy_five) > 0.05f,
            "five-marble event stream differs measurably from one-marble stream");
}

void testMarbleSolverRejectsNonfiniteSamplesAndRecoversState()
{
    marblesInit(3, 41);
    const float original_x = g_marbles[0].x;
    const float original_vx = g_marbles[0].vx;
    const float invalid_input_impact = marblesStep(0.0005f,
                                                   NAN, 0.0f, 9.80665f,
                                                   0.0f, 0.0f, 0.0f);
    near(invalid_input_impact, 0.0f, 0.0f, "nonfinite IMU sample is rejected");
    near(g_marbles[0].x, original_x, 0.0f, "rejected IMU sample does not move state");
    near(g_marbles[0].vx, original_vx, 0.0f, "rejected IMU sample does not poison velocity");

    g_marbles[0].vx = NAN;
    marblesStep(0.0005f, 0.0f, 0.0f, 9.80665f,
                0.0f, 0.0f, 0.0f);
    for (int i = 0; i < marblesCount(); ++i)
    {
        const Marble &m = g_marbles[i];
        require(isfinite(m.x) && isfinite(m.y) && isfinite(m.z) &&
                isfinite(m.vx) && isfinite(m.vy) && isfinite(m.vz),
                "nonfinite state is recovered before packet publication");
    }
    require(g_marbles[0].x >= g_marbles[0].r &&
            g_marbles[0].x <= MARBLE_BOX_W - g_marbles[0].r,
         "corrupt marble is reset to its deterministic start position");
}

} // namespace

int main()
{
    testStaticFrictionStopsResidualVelocity();
    testStaticFrictionUsesEffectiveDrive();
    testSlidingFrictionOpposesVelocity();
    testKineticFrictionZeroCrossingDoesNotReverse();
    testRollingTrajectoryAtPhysicsRates();
    testSecondImpactTimingAtPhysicsRates();
    testTexturedModeMatchesRollingPhysics();
    testNoReboundStillReportsOneTactileImpact();
    testReboundReportsImpactAndReverses();
    testMarbleAccelerationAxesAreIndependent();
    testMarbleGravityActsOnAllBodyAxes();
    testMarbleRestingContactDoesNotBuzz();
    testSupportedMarbleRespondsToTangentialShake();
    testMarbleLongRunRemainsFiniteAndInsideBox();
    testMarbleMotionScalarTracksActualWeightedSpeed();
    testMarbleImpactTelemetryEnvelopeKeepsPeakBriefly();
    testMarbleBounceTrainDampsAtConfiguredFeelRate();
    testMarbleSolverRejectsNonfiniteSamplesAndRecoversState();
    testMarbleCountClampsAndEmptyIsInert();
    testMarbleSeededPlacementIsReproducibleAndSeparated();
    testManyMarbleSeedsNeverSpawnOverlapping();
    testWallImpulseSumsAndPairCueIsReduced();
    testShakeCountChangesDeterministicRattleStream();
    std::puts("All native physics regressions passed.");
    return EXIT_SUCCESS;
}
