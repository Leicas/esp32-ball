#pragma once
#include <math.h>

// Perceptual low-speed tuning for a 10 mm metal ball in a plastic tube.
// Keep a small rest deadband; smoothly fade the lift away above 0.20 m/s.
// This changes the drive envelope, not the actuator caps or impact response.
constexpr float ROLLING_ONSET_MPS = 0.002f;
inline float rollingSlowLift(float speed) {
    if (speed <= ROLLING_ONSET_MPS || speed >= 0.20f) return 0.0f;
    const float onset = fminf((speed - ROLLING_ONSET_MPS) / 0.018f, 1.0f);
    return onset * (1.0f - speed * 5.0f);
}
inline float rollingTextureEnvelope(float speed) {
    if (!isfinite(speed) || speed <= ROLLING_ONSET_MPS) return 0.0f;
    const float blend = fmaxf(0.0f, 1.0f - speed * 5.0f);
    const float deadband = 0.015f - 0.013f * blend;
    const float base = sqrtf(fminf(fmaxf((speed - deadband) / 1.5f, 0.0f), 1.0f));
    return fminf(base + 0.07f * rollingSlowLift(speed), 1.0f);
}
inline float rollingClassicEnvelope(float speed) {
    if (!isfinite(speed) || speed <= ROLLING_ONSET_MPS) return 0.0f;
    const float onset = fminf((speed - ROLLING_ONSET_MPS) / 0.018f, 1.0f);
    return fminf(speed * 0.5f, 1.0f) * onset + 0.08f * rollingSlowLift(speed);
}
