#pragma once
#include <cmath>
#include <cstdint>
#include <algorithm>

// Rainstick synthesis: a stochastic cloud of grain contacts.
//
// RainTube reports how often and how hard grains are hitting things. This
// class renders that flow as Poisson-triggered contacts, each one a short
// noise burst plus an impulse into one of six short inharmonic resonators
// (the nails and tube wall). Dense flow sums to a bright hiss, sparse flow
// leaves separate "tiks" of random pitch; nothing is ever re-struck on a
// clock, so there is no drone and no fixed tick rate.
//
// A second output, bed(), is the same event stream thinned and low-passed to
// 50–250 Hz for the LF actuator: the patter felt in the hand.
//
// 24 kHz internal rate, linearly interpolated to 48 kHz. Integer audio path.
class RainstickAudio {
public:
    static constexpr int kModes = 6;
    static constexpr int kRate = 24000;

    RainstickAudio() { setCharacter(1); }

    // 0 fine sand, 1 rice, 2 beans. Float maths: call on change, not per sample.
    void setCharacter(int variant)
    {
        struct Character { float baseHz, decayS, eventsPerS, bedPerS; int level, bedLevel; };
        static constexpr Character kCharacters[3] = {
            {2200.f, .0012f, 4200.f, 320.f, 36000, 14000},  // sand: dense, bright, smooth
            {1500.f, .0022f, 1600.f, 170.f, 44000, 19600},  // rice
            {1100.f, .0045f, 420.f, 70.f, 48000, 26600},   // beans: sparse, lower, distinct
        };
        static constexpr float kRatios[kModes] = {1.f, 1.31f, 1.72f, 2.19f, 2.74f, 3.37f};
        const Character &c = kCharacters[std::max(0, std::min(2, variant))];
        for (int m = 0; m < kModes; m++) {
            const float w = 6.28318530718f * c.baseHz * kRatios[m] / kRate;
            const float r = std::exp(-1.f / (c.decayS / (1.f + .25f * m) * kRate));
            mode_[m].a = (int32_t)(2.f * r * std::cos(w) * 16384.f);
            mode_[m].b = (int32_t)(r * r * 16384.f);
            mode_[m].in = (int32_t)(std::sin(w) * 16384.f); // unit peak per unit impulse
        }
        eventsPerS_ = c.eventsPerS;
        bedShare_ = (uint32_t)(c.bedPerS / c.eventsPerS * 65536.f);
        level_ = c.level;
        bedLevel_ = c.bedLevel;
        reset();
    }

    void reset()
    {
        for (auto &m : mode_) m.y1 = m.y2 = 0;
        probability_ = 0;
        amplitude_ = burst_ = hissLow_ = 0;
        bed1_ = bed2_ = bedDc_ = 0;
        previous_ = current_ = bedPrevious_ = bedCurrent_ = bedOut_ = 0;
        half_ = false;
    }

    // Once per physics tick. density ~1 for a full pour, energy 0..1.
    void flow(float density, float energy)
    {
        density = std::max(0.f, std::min(2.f, density));
        energy = std::max(0.f, std::min(1.f, energy));
        // Contact probability per internal sample, 16-bit, capped at one in two.
        probability_ = (uint32_t)std::min(32768.f, density * eventsPerS_ * (65536.f / kRate));
        // Gentle contacts are quieter but never inaudible.
        amplitude_ = (int32_t)((.25f + .75f * energy) * 256.f);
    }

    // 48 kHz mono cloud. bed() then holds the tactile sample for the same frame.
    int32_t next()
    {
        if (half_) {
            half_ = false;
            bedOut_ = (bedPrevious_ + bedCurrent_) / 2;
            return (previous_ + current_) / 2;
        }
        previous_ = current_;
        bedPrevious_ = bedCurrent_;

        int32_t bedIn = 0;
        const uint32_t r = random();
        if ((r & 65535u) < probability_) {
            // Skewed sizes: many small contacts, a few large ones.
            const int32_t u = (int32_t)((r >> 16) & 255u);
            int32_t size = (16 + u * u / 272) * amplitude_ / 256; // 0..255
            const uint32_t q = random();
            const int32_t sign = (q & 1u) ? 1 : -1;
            Mode &m = mode_[(q >> 1) % kModes];
            m.y1 += sign * (level_ * size / 256) * m.in / 16384;
            burst_ = std::max(burst_, level_ * size / 256);
            if (((q >> 8) & 65535u) < bedShare_) bedIn = sign * bedLevel_ * size / 256 * kBedDrive;
        }

        int32_t sum = 0;
        for (auto &m : mode_) {
            const int32_t y = (int32_t)(((int64_t)m.a * m.y1 - (int64_t)m.b * m.y2) / 16384);
            m.y2 = m.y1;
            m.y1 = y;
            sum += y;
        }
        // Broadband part of each contact: 0.7 ms of noise, high-passed near
        // 1 kHz so it stays out of the tactile bands.
        const int32_t noise = ((int32_t)(random() & 65535u) - 32768) * burst_ / 32768;
        burst_ -= (burst_ + 15) / 16;
        approach(hissLow_, noise, 8192);
        sum += (noise - hissLow_) * 3 / 4;

        // Tactile bed: thinned contacts through two 200 Hz poles, 50 Hz DC block.
        approach(bed1_, bedIn, 1700);
        approach(bed2_, bed1_, 1700);
        approach(bedDc_, bed2_, 430);
        bedCurrent_ = bed2_ - bedDc_;

        current_ = soften(sum);
        half_ = true;
        bedOut_ = bedPrevious_;
        return previous_;
    }

    int32_t bed() const { return bedOut_; }

private:
    struct Mode { int32_t a = 0, b = 0, in = 0, y1 = 0, y2 = 0; };
    static constexpr int32_t kBedDrive = 24; // impulse gain into the 200 Hz poles

    // The cloud is a sum of random spikes, so its peaks are ~7x its RMS. Fold
    // everything above the knee 4:1 (hard stop at 30000): the pour can then run
    // about 8 dB hotter before the output limiter has to pull the level down.
    static int32_t soften(int32_t x)
    {
        constexpr int32_t kKnee = 14000, kCeiling = 30000;
        const int32_t magnitude = x < 0 ? -x : x;
        if (magnitude <= kKnee) return x;
        const int32_t folded = std::min(kCeiling, kKnee + (magnitude - kKnee) / 4);
        return x < 0 ? -folded : folded;
    }

    // One-pole step that always closes the last few counts, so the bed
    // settles to exactly zero instead of parking a DC offset on the actuator.
    static void approach(int32_t &state, int32_t target, int32_t coefficient)
    {
        int32_t step = (int32_t)(((int64_t)(target - state) * coefficient) / 32768);
        if (step == 0 && target != state) step = target > state ? 1 : -1;
        state += step;
    }

    uint32_t random()
    {
        rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
        return rng_;
    }

    Mode mode_[kModes];
    float eventsPerS_ = 0;
    uint32_t rng_ = 71923, probability_ = 0, bedShare_ = 0;
    int32_t level_ = 0, bedLevel_ = 0, amplitude_ = 0, burst_ = 0, hissLow_ = 0;
    int32_t bed1_ = 0, bed2_ = 0, bedDc_ = 0;
    int32_t previous_ = 0, current_ = 0, bedPrevious_ = 0, bedCurrent_ = 0, bedOut_ = 0;
    bool half_ = false;
};
