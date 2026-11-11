#pragma once
#include <cmath>
#include <cstdint>
#include <algorithm>

// Rainstick physics: a finite 2D section of a tube, staggered circular nails,
// impulse-resolved grain contacts. Inspired by arcade.pirillo.com/infinite-rainstick.html
// (no infinite wrapping).
//
// The 72 simulated grains are a *sample* of the fill, not the fill itself. The
// model therefore reports a contact flow (how often and how hard grains are
// hitting things) and leaves it to the synthesis to render the thousands of
// contacts a real stick produces. Only a few "pebble" grains and the end-cap
// landings are reported as distinct knocks.
//
// Call step() once per physics tick (kStepHz).
//
// Scale: one model unit is 0.4 m, so the 0.5-unit tube is the real 0.20 m
// handle: nails every 13 mm (nine staggered rows), 0.8–1.2 mm grains. A
// vertical pour swells in, runs a few seconds and tails off, as a stick this
// short does; it used to be a metre-long stick squeezed into the handle.
class RainTube {
public:
    static constexpr int kGrains = 72;
    static constexpr int kStepHz = 2000;

    struct Output {
        float position; // mean grain position along the tube, 0..1
        float speed;    // rms grain speed [model units / s]
        float density;  // smoothed contact rate; ~1 for a steady vertical pour
        float energy;   // smoothed mean contact strength, 0..1
        float knock;    // strongest distinct contact this tick, 0 = none
        float activity; // total kinetic energy, 0..1
    };

    RainTube() { reset(); }

    void reset()
    {
        float sum = 0;
        for (int i = 0; i < kGrains; i++) {
            // At rest against the low wall, as in a stick lying level, so that
            // entering the mode does not start with a settling burst.
            const float r = .002f + .001f * rnd();
            g_[i] = {kWidth - r, .006f + (i / 12) * .005f + (i % 12) * .0004f, 0, 0, r};
            sum += g_[i].y;
            // Release order through the pile; the square root makes the number
            // of grains on the move grow as the square of the pour progress.
            hold_[i] = std::sqrt(((i * 29) % kGrains + .5f) / kGrains);
        }
        pour_ = 0;
        pourUp_ = true;
        next_ = 0;
        position_ = sum / (kGrains * kLength);
        kinetic_ = cycleY_ = cycleKinetic_ = 0;
        density_ = strength_ = shake_ = 0;
    }

    // tilt   : sine of the tube angle (+1 = grains fall towards position 1)
    // axialG : linear acceleration of the stick along its axis [g]; shaking flings grains
    // variant: 0 fine sand (no pebbles), 1 rice, 2 beans (many pebbles)
    Output step(float tilt, float axialG = 0.0f, int variant = 1)
    {
        if (!std::isfinite(tilt)) tilt = 0;
        if (!std::isfinite(axialG)) axialG = 0;
        tilt = clamp(tilt, -1.f, 1.f);

        // The accelerometer also hears the actuators. Smooth (30 ms) and ignore
        // small values so the output cannot excite its own input.
        shake_ += (clamp(axialG, -kShakeLimitG, kShakeLimitG) - shake_) * kShakeSmooth;
        const float shake = deadband(shake_, kShakeDeadbandG);
        // Grains sit still until the stick is tilted a little (static friction).
        const float lean = deadband(tilt, kTiltDeadband);
        const float drive = lean - kShakeGain * shake;
        // A pile does not leave all at once: grains at the upper end let go one
        // after another, faster the steeper the stick, so a pour swells in
        // instead of starting at full flow. Reversing the tilt starts a new pour.
        if (lean != 0) {
            if ((lean > 0) != pourUp_) { pourUp_ = lean > 0; pour_ = 0; }
            pour_ = std::min(1.f, pour_ + (.35f + .65f * std::fabs(lean)) * kPourPerTick);
        }
        const bool jammed = lean != 0 && shake == 0; // shaking frees the whole pile
        const float ay = kGravity * drive;
        // Axial section: most transverse gravity is supported by the tube wall.
        const float ax = kGravity * .1f * std::sqrt(std::max(0.f, 1 - tilt * tilt));
        const int pebbleEvery = variant <= 0 ? 0 : variant == 1 ? 6 : 3;

        int hits = 0;
        float hitSum = 0, knock = 0;
        for (int batch = 0; batch < kBatch; batch++) {
            const int i = next_;
            Grain &s = g_[i];
            const bool pebble = pebbleEvery && i % pebbleEvery == 0;
            const bool inPile = pourUp_ ? s.y < kPileZone : s.y > kLength - kPileZone;
            if (jammed && inPile && pour_ < hold_[i]) {
                s.vx = s.vy = 0; // still held in the pile
            } else {
                s.vx = (s.vx + ax * kDt) * kDrag;
                s.vy = (s.vy + ay * kDt) * kDrag;
                s.x += s.vx * kDt;
                s.y += s.vy * kDt;

                auto contact = [&](float nx, float ny, float overlap, float restitution, bool distinct) {
                    s.x += nx * overlap;
                    s.y += ny * overlap;
                    const float vn = s.vx * nx + s.vy * ny;
                    if (vn >= 0) return;
                    if (vn < -kHitSpeed) {
                        const float strength = std::min(1.f, -vn * kStrengthGain * (.7f + s.r * 50));
                        hits++;
                        hitSum += strength;
                        if (distinct) knock = std::max(knock, strength);
                    }
                    s.vx -= (1 + restitution) * vn * nx;
                    s.vy -= (1 + restitution) * vn * ny;
                    // A real hit scrubs speed off; a grain merely resting on the wall
                    // or leaning on a nail keeps sliding, so a shallow tilt still pours.
                    const float keep = vn < -kHitSpeed ? kHitFriction : kSlideFriction;
                    s.vx *= keep;
                    s.vy *= keep;
                };

                if (s.x < s.r) contact(1, 0, s.r - s.x, .12f, pebble);
                if (s.x > kWidth - s.r) contact(-1, 0, s.x - (kWidth - s.r), .12f, pebble);
                // Every grain landing on an end cap is felt: the pile arriving.
                if (s.y < s.r) contact(0, 1, s.r - s.y, .08f, true);
                if (s.y > kLength - s.r) contact(0, -1, s.y - (kLength - s.r), .08f, true);

                const int row = (int)((s.y - kNailY0) / kNailPitchY);
                for (int j = std::max(0, row - 1); j <= std::min(kNailRows - 1, row + 1); j++) {
                    // Only the two nearest columns can touch this small grain.
                    const float offset = (j % 2) * kNailPitchX * .5f;
                    const int column = (int)std::floor((s.x - offset) / kNailPitchX);
                    for (int k = std::max(0, column); k <= std::min(3, column + 1); k++) {
                        const float px = k * kNailPitchX + offset, py = kNailY0 + j * kNailPitchY;
                        if (px > kWidth) continue;
                        const float dx = s.x - px, dy = s.y - py, rr = s.r + kNailRadius, d2 = dx * dx + dy * dy;
                        if (d2 < rr * rr && d2 > 1e-10f) {
                            const float d = std::sqrt(d2);
                            contact(dx / d, dy / d, rr - d, .38f, pebble);
                        }
                    }
                }
                // Dilute small grains collide with nails and walls independently. Avoid
                // quadratic grain-pair work on the audio core; end piles may overlap.
                s.x = clamp(s.x, s.r, kWidth - s.r);
                s.y = clamp(s.y, s.r, kLength - s.r);
            }

            cycleY_ += s.y;
            cycleKinetic_ += s.vx * s.vx + s.vy * s.vy;
            if (++next_ == kGrains) {
                // Whole-population statistics, refreshed once per pass (83 Hz).
                next_ = 0;
                position_ = cycleY_ / (kGrains * kLength);
                kinetic_ = cycleKinetic_;
                cycleY_ = cycleKinetic_ = 0;
            }
        }

        // 40 ms flow estimate. A steady vertical pour is ~250 contacts/s.
        density_ += (hits * kDensityPerHit - density_) * kFlowSmooth;
        strength_ += (hitSum * kDensityPerHit - strength_) * kFlowSmooth;
        if (density_ < 1e-4f) density_ = strength_ = 0;
        float density = density_;
        float energy = density_ > 0 ? std::min(1.f, strength_ / density_) : 0;
        // Shaking rattles the whole fill against the wall like a maraca. The
        // slow grain model cannot follow a 4 Hz shake, so add it to the flow.
        const float agitation = std::min(kAgitationMax, kAgitationPerG * std::fabs(shake));
        if (agitation > 0) {
            energy = (energy * density + std::min(1.f, .4f + .3f * std::fabs(shake)) * agitation) / (density + agitation);
            density += agitation;
        }
        return {position_, std::sqrt(kinetic_ / kGrains), density, energy, knock,
                std::min(1.f, kinetic_ / 5.f)};
    }

private:
    struct Grain { float x, y, vx, vy, r; };

    // Each tick advances kBatch grains by kDt, so a grain is stepped every
    // 12 ms: simulated time runs at 2/3 real time, which sets the pour duration
    // (about 8 s end to end when vertical).
    static constexpr int kBatch = 3;
    static constexpr float kDt = .008f;
    static constexpr float kGravity = 1.6f; // model units / s² per g (slow: grains are held up by nails)
    static constexpr float kDrag = .994f;
    static constexpr float kWidth = .15f;
    static constexpr float kNailRadius = .009f, kNailPitchX = .05f, kNailPitchY = .0325f, kNailY0 = .11f;
    static constexpr float kLength = .5f; // 0.20 m handle at 0.4 m per unit
    static constexpr int kNailRows = 9;   // .11 .. .37, mirrored clearance at both ends
    static constexpr float kHitSpeed = .05f;     // slower contacts are sliding, not hits
    static constexpr float kStrengthGain = 2.2f; // normal speed -> 0..1 strength
    static constexpr float kTiltDeadband = .05f;
    static constexpr float kShakeGain = 3.0f, kShakeLimitG = 3.0f, kShakeDeadbandG = .25f;
    static constexpr float kAgitationPerG = .6f, kAgitationMax = 1.2f;
    static constexpr float kHitFriction = .92f, kSlideFriction = .99f;
    static constexpr float kPileZone = .07f;                      // end cap to first nails
    static constexpr float kPourPerTick = 1.0f / (1.4f * kStepHz); // 1.4 s to free the pile when vertical
    static constexpr float kShakeSmooth = 1.0f / 60;
    static constexpr float kFlowSmooth = 1.0f / 80;
    static constexpr float kDensityPerHit = kStepHz / 250.0f;

    static float clamp(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }
    static float deadband(float v, float band) { return v > band ? v - band : v < -band ? v + band : 0; }
    float rnd()
    {
        rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
        return (rng_ & 65535) / 65535.f;
    }

    Grain g_[kGrains];
    int next_ = 0;
    float position_ = 0, kinetic_ = 0, cycleY_ = 0, cycleKinetic_ = 0;
    float density_ = 0, strength_ = 0, shake_ = 0, pour_ = 0;
    float hold_[kGrains];
    bool pourUp_ = true;
    uint32_t rng_ = 19871;
};
