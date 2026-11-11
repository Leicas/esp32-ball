#pragma once
#include <cmath>
#include <cstdint>
#include "rolling_response.h"

// Metal ball rolling in a plastic tube, 48 kHz, integer audio path.
//
// A hard smooth ball on a smooth wall is not noisy: what the hand feels is the
// ball's own few imperfections coming round once per revolution. The rolling
// signal is therefore a fixed "signature" of five bumps, one ball circumference
// long, read out by ball position. It is strictly periodic in distance, so its
// flutter rate tracks speed exactly and it is silent when the ball stops. There
// is no random source in this class.
//
// The bumps alone fall below the actuators' band when the ball creeps, so a
// second table holds the ball's fine finish (ripples of 0.7–1.2 mm, also fixed
// and periodic per revolution). It carries the slow roll and fades out above
// kFineFadeMps, where the bumps take over.
//
// The signature also drives one damped mode of the tube wall (the hollow
// plastic tone). Wall impacts ring two more modes: a low plastic thud and a
// short bright tick from the ball.
class RollingBall {
public:
    static constexpr float kCircumference = 0.0314f; // 10 mm ball [m]
    static constexpr int kTable = 1024;
    static constexpr float kFineFadeMps = 0.6f;
    static constexpr float kRollLevel = 0.70f;      // full-speed drive, fraction of full scale
    static constexpr int kOutputGain = 9;            // quarters: 2.25x
    static constexpr int kThudPeak = 14000, kTickPeak = 16000;

    void init(float sample_rate)
    {
        rate_ = sample_rate;
        // Bump centre, width (fractions of a revolution) and height. Irregular
        // spacing keeps one revolution from sounding like a five-toothed gear.
        static constexpr float kBumps[5][3] = {
            {.05f, .10f, 1.0f}, {.27f, .07f, -.6f}, {.41f, .14f, .8f}, {.68f, .08f, -.9f}, {.86f, .11f, .5f}};
        static float shape[kTable]; // scratch, kept off the setup() stack
        float mean = 0, peak = 0;
        for (int i = 0; i < kTable; i++) {
            float y = 0;
            for (const auto &b : kBumps) {
                float d = (float)i / kTable - b[0];
                d -= std::floor(d + .5f); // nearest image on the circle
                if (std::fabs(d) < b[1] * .5f) y += b[2] * (.5f + .5f * std::cos(6.28318530718f * d / b[1]));
            }
            shape[i] = y;
            mean += y / kTable;
        }
        for (float y : shape) peak = std::fmax(peak, std::fabs(y - mean));
        for (int i = 0; i < kTable; i++) table_[i] = (int16_t)((shape[i] - mean) / peak * 32000.f);

        // Fine finish: harmonics 26..44 of the revolution with fixed weights and phases.
        uint32_t seed = 0x51f3u;
        peak = 0;
        for (int i = 0; i < kTable; i++) shape[i] = 0;
        for (int h = 26; h <= 44; h++) {
            seed = seed * 1664525u + 1013904223u;
            const float weight = .5f + .5f * ((seed >> 16) & 255u) / 255.f;
            const float offset = 6.28318530718f * ((seed >> 24) & 255u) / 256.f;
            for (int i = 0; i < kTable; i++) shape[i] += weight * std::sin(6.28318530718f * h * i / kTable + offset);
        }
        for (float y : shape) peak = std::fmax(peak, std::fabs(y));
        for (int i = 0; i < kTable; i++) fine_[i] = (int16_t)(shape[i] / peak * 26000.f);

        body_.configure(150.f, .010f, rate_);
        thud_.configure(190.f, .014f, rate_);
        tick_.configure(1150.f, .0035f, rate_);
        reset();
    }

    void reset()
    {
        body_.clear(); thud_.clear(); tick_.clear();
        phase_ = delta_ = 0;
        amplitude_ = step_ = fineWeight_ = thudOut_ = tickOut_ = 0;
    }

    // Once per physics tick, before `frames` calls to next().
    // impact_strength: 0 = none, 1 = hardest wall hit.
    void begin(float position_m, float velocity_mps, float impact_strength, int frames)
    {
        if (!std::isfinite(position_m) || !std::isfinite(velocity_mps) || !std::isfinite(impact_strength)) {
            reset();
            return;
        }
        const float turns = position_m / kCircumference;
        phase_ = (uint32_t)((turns - std::floor(turns)) * 4294967040.f);
        const float v = std::fmax(-10.f, std::fmin(10.f, velocity_mps));
        delta_ = (uint32_t)(int32_t)(v / (rate_ * kCircumference) * 4294967296.f);
        const int32_t goal = (int32_t)(rollingTextureEnvelope(std::fabs(v)) * kRollLevel * 32767.f);
        // 8 ms slew at the 2 kHz control rate, spread evenly over this tick.
        step_ = (goal - amplitude_) / 16 / frames;
        // Always close the last few counts: no parked offset once the ball stops.
        if (step_ == 0 && goal != amplitude_) step_ = goal > amplitude_ ? 1 : -1;
        if (step_ * frames > goal - amplitude_ && step_ > 0) step_ = 0, amplitude_ = goal;
        if (step_ * frames < goal - amplitude_ && step_ < 0) step_ = 0, amplitude_ = goal;
        fineWeight_ = (int32_t)(std::fmax(0.f, 1.f - std::fabs(v) / kFineFadeMps) * 32767.f);
        if (impact_strength > 0) {
            const float hit = std::fmin(impact_strength, 1.f);
            thud_.strike((int32_t)(hit * kThudPeak));
            tick_.strike((int32_t)(hit * kTickPeak));
        }
    }

    // Rolling signal for one 48 kHz frame; thud()/tick() are the impact rings.
    int32_t next()
    {
        const int index = phase_ >> 22, after = (index + 1) & (kTable - 1);
        const int32_t fraction = (int32_t)((phase_ >> 7) & 32767u);
        const int32_t bumps = table_[index] + (table_[after] - table_[index]) * fraction / 32768;
        const int32_t finish = fine_[index] + (fine_[after] - fine_[index]) * fraction / 32768;
        const int32_t surface = bumps + finish * fineWeight_ / 32768;
        phase_ += delta_;
        amplitude_ += step_;
        const int32_t contact = surface * amplitude_ / 32768;
        thudOut_ = thud_.tick(0);
        tickOut_ = tick_.tick(0);
        // Drive hard: the LF actuator does not move at all below a certain level.
        return (contact * 5 / 8 + body_.tick(contact)) * kOutputGain / 4;
    }
    int32_t thud() const { return thudOut_; } // low plastic ring -> LF
    int32_t tick() const { return tickOut_; } // bright metal tick -> HF

private:
    // Two-pole mode with Q24 coefficients: low modes at 48 kHz need the precision.
    struct Mode {
        int64_t a = 0, b = 0, gain = 0;
        int32_t impulse = 0, y1 = 0, y2 = 0;
        void configure(float hz, float decay_s, float rate)
        {
            const float r = std::exp(-1.f / (decay_s * rate)), w = 6.28318530718f * hz / rate;
            a = (int64_t)(2.0 * r * std::cos(w) * 16777216.0);
            b = (int64_t)((double)r * r * 16777216.0);
            gain = (int64_t)(2.0 * (1.0 - r) * std::sin(w) * 16777216.0); // unity at resonance
            impulse = (int32_t)(std::sin(w) * 32768.f);                   // unit ring peak
        }
        void clear() { y1 = y2 = 0; }
        void strike(int32_t peak) { y1 += peak * impulse / 32768; }
        int32_t tick(int32_t input)
        {
            int64_t y = (a * y1 - b * y2 + gain * input) / 16777216;
            y = y < -1000000 ? -1000000 : y > 1000000 ? 1000000 : y;
            y2 = y1;
            y1 = (int32_t)y;
            return y1;
        }
    };

    int16_t table_[kTable] = {}, fine_[kTable] = {};
    Mode body_, thud_, tick_;
    uint32_t phase_ = 0, delta_ = 0;
    int32_t amplitude_ = 0, step_ = 0, fineWeight_ = 0, thudOut_ = 0, tickOut_ = 0;
    float rate_ = 48000;
};
