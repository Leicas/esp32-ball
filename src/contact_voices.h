#pragma once
#include <cstdint>
#include <algorithm>
#include <cmath>

// 48 kHz tactile voice for distinct rainstick contacts (pebbles, end-cap
// landings). Strength sets the amplitude; LFi stays positive-only.
class RainContact {
public:
    static constexpr int kPeakLfi = 8000; // 0.24 FS, just inside the USB-safe 0.25 FS impact level
    static constexpr int kDuration = 480; // 10 ms

    void reset() { age_ = kDuration; cooldown_ = amp_ = 0; }

    // Returns true when the knock was accepted. A short irregular recovery gap
    // keeps individual contacts distinguishable without clocking them.
    bool excite(float strength)
    {
        if (cooldown_ || strength < .08f) return false;
        rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
        amp_ = (int)(std::min(1.f, strength) * kPeakLfi);
        age_ = 0;
        cooldown_ = 288 + (rng_ % 480); // 6–16 ms
        return true;
    }

    // out[0]: LFi, out[1]: LF.
    void next(int32_t out[2])
    {
        if (cooldown_) --cooldown_;
        out[0] = out[1] = 0;
        if (age_ >= kDuration) return;
        // Positive-only 3 ms LFi knock, followed by zero drive.
        if (age_ < 144) out[0] = age_ < 48 ? amp_ * age_ / 48 : amp_ * (144 - age_) / 96;
        // One balanced 10 ms LF displacement pulse, not an oscillating hum.
        const int t = age_ % 240, triangle = t < 120 ? t : 240 - t;
        out[1] = (age_ < 240 ? 1 : -1) * amp_ * triangle / 160;
        ++age_;
    }

private:
    int age_ = kDuration, cooldown_ = 0, amp_ = 0;
    uint32_t rng_ = 19817;
};
