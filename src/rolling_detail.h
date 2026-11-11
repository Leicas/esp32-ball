#pragma once
#include <stdint.h>

// Audio-rate HF contact detail. Per-sample path is integer-only for ESP32-C6.
class RollingDetail {
public:
    void init(float sample_rate);
    void reset();
    void begin(float position_m, float velocity_mps, float impact_strength, float control_rate);
    int16_t next(); // Q15, bounded signed audio
private:
    struct Resonator {
        int32_t a = 0, b = 0, gain = 0, y1 = 0, y2 = 0;
        void configure(float hz, float decay, float rate);
        int32_t tick(int32_t input);
    };
    Resonator mid_, bright_, ring_;
    uint32_t phase1_ = 0, phase2_ = 0, delta1_ = 0, delta2_ = 0;
    int32_t amplitude_ = 0, fine_amplitude_ = 0, strike_ = 0;
    float rate_ = 48000;
    static int32_t roughness(uint32_t phase, uint32_t seed);
};
