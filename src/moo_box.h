#pragma once
#include <cmath>
#include <cstdint>
#include <algorithm>

// Moo box (the toy can that moos when you turn it over), 48 kHz.
//
// Inside the real thing a weighted bellows sinks through the can and pushes
// air past a reed. Here a piston travels the tube whenever the stick is tipped
// past about 15 degrees. It has mass, so it starts slowly and speeds up; its
// airflow drives a reed (sawtooth) through two vocal formants. Loudness, pitch
// and mouth opening all follow the airflow, which gives the swelling
// "mmmeooOOO", cut short when the piston lands with a clunk. Tipping it back sends the piston the other way: it moos
// in both directions.
//
// control() once per physics tick (float, cheap polynomials only), then
// next() for each audio frame (integer).
class MooBox {
public:
    static constexpr int kStepHz = 2000;
    static constexpr float kRate = 48000.f;

    void reset()
    {
        position_ = velocity_ = 0;
        breath_ = lfo_ = 0;
        phase_ = delta_ = 0;
        level_ = levelStep_ = noiseLevel_ = 0;
        low_.clear(); high_.clear();
        moving_ = false;
    }

    // tilt: sine of the tube angle. Returns the landing strength (0 = none) on
    // the tick the piston reaches an end.
    float control(float tilt, int frames)
    {
        if (!std::isfinite(tilt)) tilt = 0;
        tilt = std::max(-1.f, std::min(1.f, tilt));
        const float lean = tilt > kTipOver ? tilt - kTipOver : tilt < -kTipOver ? tilt + kTipOver : 0;
        // The piston is a weight on a bellows: gravity accelerates it, the air
        // it has to push through the reed resists in proportion to its speed.
        // From rest it keeps gathering speed for most of the travel, so the
        // moo swells and climbs ("mmmeooOOO") instead of holding one note.
        const float push = lean / (1 - kTipOver);
        const float acceleration = lean != 0 ? kWeight * push - kAirDrag * velocity_ : -kHoldDrag * velocity_;
        velocity_ += acceleration / kStepHz;
        position_ += velocity_ / kStepHz;
        float landed = 0;
        if (position_ <= 0 || position_ >= 1) {
            // Bellows fully closed: the weight lands and the air stops at once.
            position_ = position_ <= 0 ? 0 : 1;
            if (std::fabs(velocity_) > .05f) landed = std::min(1.f, .3f + .6f * std::fabs(velocity_) / kFullSpeed);
            velocity_ = 0;
        }
        moving_ = velocity_ != 0;

        // Breath follows the airflow: 25 ms to build, 60 ms to cut off.
        const float flow = std::min(1.2f, std::fabs(velocity_) / kFullSpeed);
        breath_ += (flow - breath_) * (flow > breath_ ? 1.f / 50 : 1.f / 120);
        if (breath_ < 1e-4f) breath_ = 0;

        lfo_ += 6.28318530718f * 5.5f / kStepHz; // slight wobble, like a real reed
        if (lfo_ > 6.28318530718f) lfo_ -= 6.28318530718f;
        const float wobble = 1 + .012f * sine(lfo_);
        // More air, higher note: about 120 Hz as it starts, 195 Hz at full flow.
        const float pitch = (118 + 65 * breath_) * wobble;
        delta_ = (uint32_t)(pitch * (4294967296.f / kRate));

        // The mouth opens with the airflow: closed "mm", a brief bright "e" as
        // it parts, then round "oo".
        const float open = smooth(std::min(1.f, breath_ * 1.5f));
        const float bright = std::max(0.f, 1 - std::fabs(open - .35f) * (1 / .3f));
        low_.tune(250 + 190 * open, kRate);
        high_.tune(720 + 300 * open + 650 * bright, kRate);
        highShare_ = (int32_t)((.5f + 1.3f * open) * 256);

        const int32_t goal = (int32_t)(std::min(1.f, breath_) * 32767.f);
        levelStep_ = (goal - level_) / frames;
        if (levelStep_ == 0 && goal != level_) level_ = goal;
        noiseLevel_ = goal / 14;
        return landed;
    }

    int32_t next()
    {
        phase_ += delta_;
        level_ += levelStep_;
        // Reed: a sawtooth with a little breath noise on top.
        rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
        const int32_t saw = (int32_t)(phase_ >> 16) - 32768;
        const int32_t noise = ((int32_t)(rng_ & 65535u) - 32768) * noiseLevel_ / 32768;
        const int32_t reed = saw * level_ / 32768 + noise;
        const int32_t voice = low_.tick(reed) + high_.tick(reed) * highShare_ / 256 + reed / 10;
        return voice * kOutputGain / 8;
    }

    float position() const { return position_; }
    bool sounding() const { return level_ != 0 || levelStep_ != 0; }

private:
    static constexpr float kTipOver = .25f;  // sine of the tilt needed to start the piston
    // Tube lengths and seconds. Vertical: about 1.9 s end to end, arriving at
    // roughly kFullSpeed and still accelerating.
    static constexpr float kWeight = .8f;    // acceleration when vertical [1/s²]
    static constexpr float kAirDrag = .6f;   // reed resistance [1/s]
    static constexpr float kHoldDrag = 6.f;  // friction once it is no longer tipped [1/s]
    static constexpr float kFullSpeed = .95f; // speed that gives full breath [1/s]
    static constexpr int kOutputGain = 10;   // eighths; full flow peaks just under the output limiter

    // Formant: two-pole resonator, fixed bandwidth, retuned every tick with
    // short polynomials instead of cos/sin (soft-float core).
    struct Formant {
        int64_t a = 0, b = 0, gain = 0;
        int32_t y1 = 0, y2 = 0;
        float r = 0;
        void tune(float hz, float rate)
        {
            if (r == 0) { r = std::exp(-3.14159265f * 110.f / rate); b = (int64_t)((double)r * r * 16777216.0); }
            const float w = 6.28318530718f * hz / rate, w2 = w * w;
            const float cosine = 1 - w2 * (.5f - w2 * (1.f / 24));
            const float sineW = w * (1 - w2 * (1.f / 6));
            a = (int64_t)(2.f * r * cosine * 16777216.f);
            gain = (int64_t)(2.f * (1 - r) * sineW * 16777216.f); // unity at resonance
        }
        void clear() { y1 = y2 = 0; }
        int32_t tick(int32_t x)
        {
            int64_t y = (a * y1 - b * y2 + gain * x) / 16777216;
            y = y < -1000000 ? -1000000 : y > 1000000 ? 1000000 : y;
            y2 = y1;
            y1 = (int32_t)y;
            return y1;
        }
    };

    static float smooth(float x) { return x * x * (3 - 2 * x); }
    // Bhaskara approximation, x in [0, 2pi).
    static float sine(float x)
    {
        const float pi = 3.14159265f;
        const bool negative = x > pi;
        if (negative) x -= pi;
        const float p = x * (pi - x);
        const float y = 16 * p / (5 * pi * pi - 4 * p);
        return negative ? -y : y;
    }

    Formant low_, high_;
    float position_ = 0, velocity_ = 0, breath_ = 0, lfo_ = 0;
    uint32_t phase_ = 0, delta_ = 0, rng_ = 90217;
    int32_t level_ = 0, levelStep_ = 0, noiseLevel_ = 0, highShare_ = 64;
    bool moving_ = false;
};
