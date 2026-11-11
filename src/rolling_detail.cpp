#include "rolling_response.h"
#include "rolling_detail.h"
#include <math.h>

namespace {
int32_t bounded(int32_t x) { return x < -32767 ? -32767 : x > 32767 ? 32767 : x; }
float clamp(float x, float a, float b) { return fmaxf(a, fminf(b, x)); }
int32_t grain(uint32_t x) {
    x ^= x >> 16; x *= 0x7feb352du;
    x ^= x >> 15; x *= 0x846ca68bu;
    x ^= x >> 16;
    return (int32_t)(x & 65535u) - 32768;
}
}

void RollingDetail::Resonator::configure(float hz, float decay, float rate) {
    const float r = expf(-1.0f / (decay * rate));
    const float w = 6.28318530718f * hz / rate;
    a = (int32_t)(2.0f * r * cosf(w) * 16384.0f);
    b = (int32_t)(-r * r * 16384.0f);
    gain = (int32_t)(2.0f * (1.0f - r) * sinf(w) * 16384.0f);
    y1 = y2 = 0;
}
int32_t RollingDetail::Resonator::tick(int32_t input) {
    // Coefficients and Q15 states keep this sum inside signed 32-bit range.
    const int32_t y = bounded((a * y1 + b * y2 + gain * input) / 16384);
    y2 = y1; y1 = y;
    return y;
}
void RollingDetail::init(float sample_rate) {
    rate_ = sample_rate;
    mid_.configure(700.0f, 0.0015f, rate_);
    bright_.configure(1800.0f, 0.0012f, rate_);
    ring_.configure(3600.0f, 0.0008f, rate_);
    reset();
}
void RollingDetail::reset() {
    amplitude_ = fine_amplitude_ = strike_ = 0;
    phase1_ = phase2_ = delta1_ = delta2_ = 0;
    mid_.y1 = mid_.y2 = bright_.y1 = bright_.y2 = ring_.y1 = ring_.y2 = 0;
}
void RollingDetail::begin(float position_m, float velocity_mps, float impact_strength, float control_rate) {
    if (!isfinite(position_m) || !isfinite(velocity_mps) || !isfinite(impact_strength)) {
        reset(); return;
    }
    const float x = clamp(position_m, -4.0f, 4.0f);
    const float v = clamp(velocity_mps, -10.0f, 10.0f);
    // Unsigned phases wrap predictably; the fine field repeats only after 15.7 m.
    phase1_ = (uint32_t)(int32_t)(x * (65536.0f / 0.0012f));
    phase2_ = (uint32_t)(int32_t)(x * (65536.0f / 0.00024f));
    delta1_ = (uint32_t)(int32_t)(v * (65536.0f / (rate_ * 0.0012f)));
    delta2_ = (uint32_t)(int32_t)(v * (65536.0f / (rate_ * 0.00024f)));
    const float target = rollingTextureEnvelope(fabsf(v));
    const float slew = clamp(1.0f / (0.008f * control_rate), 0.0f, 1.0f);
    const int32_t goal = (int32_t)(target * 32767.0f);
    amplitude_ += (int32_t)((goal - amplitude_) * slew);
    if (goal == 0 && amplitude_ < 32) amplitude_ = 0;
    fine_amplitude_ = (int32_t)(amplitude_ * clamp(1.0f - fabsf(v) / (0.00024f * rate_ * 0.4f), 0.0f, 1.0f));
    // Excite the metal-like HF ring with a brief sample-rate pulse.
    const int32_t hit = (int32_t)(clamp(impact_strength, 0.0f, 1.0f) * 32767.0f);
    if (hit > strike_) strike_ = hit;
}
int32_t RollingDetail::roughness(uint32_t phase, uint32_t seed) {
    const uint32_t cell = phase >> 16;
    const int32_t a = grain(cell ^ seed), b = grain((cell + 1) ^ seed);
    // Q15 interpolation avoids a 64-bit multiply in the audio loop.
    return a + ((b - a) * (int32_t)((phase & 65535u) >> 1)) / 32768;
}
int16_t RollingDetail::next() {
    const int32_t coarse = roughness(phase1_, 0x71afu) * amplitude_ / 32768;
    const int32_t fine = roughness(phase2_, 0x39d1u) * fine_amplitude_ / 32768;
    phase1_ += delta1_; phase2_ += delta2_;
    const int32_t hit = strike_;
    strike_ = strike_ * 15 / 16;
    const int32_t lo = mid_.tick(bounded(coarse + hit));
    const int32_t hi = bright_.tick(bounded(fine + hit));
    const int32_t ring = ring_.tick(bounded(fine + hit));
    return (int16_t)bounded((lo + hi + ring) * 2);
}
