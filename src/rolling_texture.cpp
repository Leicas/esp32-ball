#include "rolling_response.h"
#include "rolling_texture.h"
#include <math.h>

namespace {
float clamp(float x, float low, float high) { return fmaxf(low, fminf(high, x)); }
float grain(int32_t index, uint32_t seed) {
    uint32_t h = (uint32_t)index ^ seed;
    h ^= h >> 16; h *= 0x7feb352du;
    h ^= h >> 15; h *= 0x846ca68bu;
    h ^= h >> 16;
    return (float)(h & 0xffffu) * (2.0f / 65535.0f) - 1.0f;
}
}

void RollingTexture::Mode::configure(float hz, float decay_s, float rate) {
    const float r = expf(-1.0f / (rate * decay_s));
    a = 2.0f * r * cosf(6.28318530718f * hz / rate);
    b = -r * r;
    drive = 1.0f - r;
    y1 = y2 = 0;
}
float RollingTexture::Mode::tick(float input) {
    const float y = a * y1 + b * y2 + drive * input;
    y2 = y1;
    y1 = y;
    return y;
}

void RollingTexture::init(float sample_rate) {
    rate_ = sample_rate;
    // Frequencies stay below Nyquist even on the 1 kHz legacy profiles.
    body_.configure(90.0f, 0.018f, rate_);
    shell_.configure(280.0f, 0.010f, rate_);
    grit_.configure(430.0f, 0.006f, rate_);
    slew_ = 1.0f - expf(-1.0f / (rate_ * 0.012f));
    reset();
}
void RollingTexture::reset() {
    envelope_ = 0;
    body_.y1 = body_.y2 = shell_.y1 = shell_.y2 = grit_.y1 = grit_.y2 = 0;
}

float RollingTexture::surface(float position, float spacing, uint32_t seed) {
    const float p = position / spacing;
    const int32_t cell = (int32_t)floorf(p);
    float f = p - (float)cell;
    f = f * f * (3.0f - 2.0f * f); // smooth joins between deterministic surface grains
    const float a = grain(cell, seed);
    return a + (grain(cell + 1, seed) - a) * f;
}

RollingTexture::Output RollingTexture::step(float position_m, float velocity_mps, float impact_strength) {
    if (!isfinite(position_m) || !isfinite(velocity_mps) || !isfinite(impact_strength)) {
        reset();
        return {0, 0};
    }
    // Firmware positions are 0..2 m; constrain malformed callers before float-to-int conversion.
    position_m = clamp(position_m, -10.0f, 10.0f);
    const float speed = fabsf(velocity_mps);
    const float target = rollingTextureEnvelope(speed);
    envelope_ += slew_ * (target - envelope_);
    if (envelope_ < 1e-6f) envelope_ = 0;

    // Suppress a roughness band before its traversal rate reaches Nyquist.
    const float fine_weight = clamp(1.0f - speed / (0.0007f * rate_ * 0.4f), 0.0f, 1.0f);
    const float mid_weight = clamp(1.0f - speed / (0.0037f * rate_ * 0.4f), 0.0f, 1.0f);
    const float coarse = surface(position_m, 0.018f, 0x83a5u);
    const float mid = surface(position_m, 0.0037f, 0xb74du) * mid_weight;
    const float fine = surface(position_m, 0.0007f, 0x19f3u) * fine_weight;
    const float contact = envelope_ * (0.55f * coarse + 0.30f * mid + 0.15f * fine);
    const float hit = clamp(impact_strength, 0.0f, 1.0f);
    const float body = body_.tick(contact);
    // Impact excites a short ring on HF; the existing positive LFi hammer pulse stays intact.
    const float shell = shell_.tick(contact + hit * 3.0f);
    const float grit = grit_.tick(envelope_ * fine + hit * 2.0f);

    // Keep the low-frequency hammer on its negative side during rolling.
    // Hard output bounds are independent of resonance accumulation and speed.
    const float low = -clamp(envelope_ * (0.38f + 0.18f * coarse) + 0.65f * body, 0.0f, 0.80f);
    const float high = clamp(1.8f * shell + 0.9f * grit + 0.12f * envelope_ * mid, -0.80f, 0.80f);
    return {low, high};
}
