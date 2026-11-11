#pragma once
#include <stdint.h>

// Procedural rolling contact: spatial surface roughness excites damped modes.
// Portable DSP; no allocation, random clock noise, or Arduino dependency.
class RollingTexture {
public:
    struct Output { float low; float high; }; // low [-1,0], high [-1,1]
    void init(float sample_rate);
    void reset();
    Output step(float position_m, float velocity_mps, float impact_strength);

private:
    struct Mode {
        float a = 0, b = 0, drive = 0, y1 = 0, y2 = 0;
        void configure(float hz, float decay_s, float rate);
        float tick(float input);
    };
    Mode body_, shell_, grit_;
    float envelope_ = 0, slew_ = 0, rate_ = 2000;
    static float surface(float position, float spacing, uint32_t seed);
};
