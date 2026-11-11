#include "contact_voices.h"
#include "rain_tube.h"
#include "rainstick_audio.h"
#include "rolling_ball.h"
#include "moo_box.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

// One simulated pour: physics at 2 kHz, synthesis at 48 kHz.
struct Pour {
    double contactRate = 0, energy = 0, cloudRms = 0, bedRms = 0;
    int knocks = 0;
    float duration = 0, finalPosition = 0;
    long long cloudPeak = 0, lfPeak = 0, lfiPeak = 0, lfiMin = 0, tailPeak = 0;
};

static Pour pour(float tilt, int variant, int seconds = 28)
{
    RainTube rain; RainContact contact; RainstickAudio audio;
    audio.setCharacter(variant);
    Pour p; double cloud = 0, bed = 0; long samples = 0, steady = 0;
    for (int i = 0; i < seconds * 2000; i++) {
        const auto v = rain.step(tilt, 0, variant);
        assert(v.position >= -.001f && v.position <= 1.001f);
        assert(v.density >= 0 && v.energy >= 0 && v.energy <= 1 && v.knock >= 0 && v.knock <= 1);
        if (v.density > .05f) p.duration = i / 2000.f;
        if (i >= 3000 && i < 7000) { p.contactRate += v.density; p.energy += v.energy; steady++; }
        if (contact.excite(v.knock)) p.knocks++;
        audio.flow(v.density, v.energy);
        for (int f = 0; f < 24; f++) {
            int32_t out[2]; contact.next(out);
            const long long s = audio.next(), b = audio.bed();
            p.lfiPeak = std::max<long long>(p.lfiPeak, out[0]);
            p.lfiMin = std::min<long long>(p.lfiMin, out[0]);
            p.lfPeak = std::max(p.lfPeak, std::llabs(b + out[1]));
            p.cloudPeak = std::max(p.cloudPeak, std::llabs(s));
            if (i >= 3000 && i < 7000) { cloud += (double)s * s; bed += (double)b * b; samples++; }
            if (i >= (seconds - 1) * 2000) p.tailPeak = std::max(p.tailPeak, std::max(std::llabs(s), std::llabs(b + out[1])));
        }
        p.finalPosition = v.position;
    }
    p.contactRate /= steady; p.energy /= steady;
    p.cloudRms = std::sqrt(cloud / samples); p.bedRms = std::sqrt(bed / samples);
    return p;
}

static void testRainstick()
{
    // A single knock: positive-only LFi, bounded, and over within 10 ms.
    RainContact c; int32_t out[2]; int peak = 0; long long tactile = 0;
    assert(!c.excite(.05f));
    assert(c.excite(.6f));
    for (int i = 0; i < 4800; i++) {
        c.next(out);
        assert(out[0] >= 0 && out[0] <= RainContact::kPeakLfi);
        peak = std::max(peak, (int)out[0]); tactile += std::abs(out[1]);
        if (i >= RainContact::kDuration) assert(out[0] == 0 && out[1] == 0);
    }
    assert(peak > 2000 && tactile > 0);
    // Knock amplitude follows strength.
    RainContact soft; soft.excite(.2f); int softPeak = 0;
    for (int i = 0; i < 480; i++) { soft.next(out); softPeak = std::max(softPeak, (int)out[0]); }
    assert(softPeak * 2 < peak);

    const Pour steep = pour(1.f, 1), shallow = pour(.4f, 1), resting = pour(.02f, 1);
    printf("rice  tilt 1.0: rate=%.2f energy=%.2f cloud=%.0f bed=%.0f knocks=%d duration=%.1fs\n",
           steep.contactRate, steep.energy, steep.cloudRms, steep.bedRms, steep.knocks, steep.duration);
    printf("rice  tilt 0.4: rate=%.2f energy=%.2f cloud=%.0f bed=%.0f knocks=%d duration=%.1fs\n",
           shallow.contactRate, shallow.energy, shallow.cloudRms, shallow.bedRms, shallow.knocks, shallow.duration);
    // The pour is audible, tilt changes it, and it runs for seconds rather than a blip.
    assert(steep.contactRate > .6 && steep.contactRate < 1.5);
    assert(shallow.contactRate > .25 && shallow.contactRate < steep.contactRate * .85);
    assert(shallow.energy < steep.energy * .8);
    assert(steep.cloudRms > 7000 && shallow.cloudRms > 3000 && shallow.cloudRms < steep.cloudRms * .8);
    assert(steep.bedRms > 400 && shallow.bedRms < steep.bedRms);
    assert(steep.duration > 3 && steep.duration < 8); // a 0.20 m stick, not a metre-long one
    assert(shallow.duration > steep.duration + 2);
    assert(steep.finalPosition > .9f && shallow.finalPosition > .9f);
    // Output stays inside the mixer and the USB-safe tactile levels.
    for (const Pour *p : {&steep, &shallow}) {
        assert(p->cloudPeak <= 30000 && p->lfPeak < 16000);
        assert(p->lfiPeak <= RainContact::kPeakLfi && p->lfiMin == 0);
        assert(p->tailPeak == 0); // everything has landed: exact silence, no DC
    }
    // A level stick with nothing moving is silent.
    assert(resting.cloudPeak == 0 && resting.lfPeak == 0 && resting.knocks == 0);

    // A pour swells in: the pile lets go grain by grain rather than all at once.
    {
        RainTube rain; double early = 0, middle = 0, full = 0;
        for (int i = 0; i < 8000; i++) {
            const float d = rain.step(1.f, 0, 1).density;
            if (i < 1500) early += d / 1500; else if (i >= 2000 && i < 3000) middle += d / 1000; else if (i >= 5000 && i < 7000) full += d / 2000;
        }
        printf("onset: 0-0.75s=%.2f 1-1.5s=%.2f 2.5-3.5s=%.2f\n", early, middle, full);
        assert(early < .15 * full && middle > .1 * full && middle < .7 * full && full > .7);
    }

    // Characters: sand has no pebbles until the pile lands, beans have the most knocks.
    const Pour sand = pour(1.f, 0), beans = pour(1.f, 2);
    printf("sand knocks=%d  rice knocks=%d  beans knocks=%d\n", sand.knocks, steep.knocks, beans.knocks);
    assert(sand.knocks < steep.knocks && steep.knocks < beans.knocks);
    assert(sand.cloudRms > 6000 && beans.cloudRms > 4000);

    // Grains pour both ways.
    for (float tilt : {-1.f, 1.f, -.7f, .7f}) {
        RainTube rain; float position = 0;
        for (int i = 0; i < 40000; i++) rain.step(-tilt, 0, 1);
        for (int i = 0; i < 40000; i++) position = rain.step(tilt, 0, 1).position;
        assert(tilt > 0 ? position > .9f : position < .1f);
    }

    // Shaking a level stick rattles it; sensor noise and actuator-scale vibration do not.
    double shaken = 0, noisy = 0, buzzed = 0; uint32_t r = 7;
    RainTube a, b, d;
    for (int i = 0; i < 8000; i++) {
        shaken += a.step(0, 1.5f * std::sin(6.2831853f * 4 * i / 2000.f), 1).density / 8000;
        r = r * 1664525u + 1013904223u; const float n = (r >> 16) / 65535.f - .5f;
        noisy += b.step(.03f * n, .3f * n, 1).density;
        buzzed += d.step(0, 2.f * std::sin(6.2831853f * 80 * i / 2000.f), 1).density;
    }
    printf("shake 1.5 g at 4 Hz: rate=%.2f\n", shaken);
    assert(shaken > .2 && noisy == 0 && buzzed == 0);
}

static void testRollingBall()
{
    RollingBall ball; ball.init(48000);
    auto roll = [&](float v, int ticks, std::vector<int32_t> *keep) {
        float x = .1f; long long peak = 0;
        for (int i = 0; i < ticks; i++) {
            ball.begin(x, v, 0, 24);
            for (int f = 0; f < 24; f++) { const int32_t s = ball.next(); peak = std::max(peak, std::llabs(s)); if (keep) keep->push_back(s); }
            x += v / 2000.f;
        }
        return peak;
    };
    // Steady rolling repeats exactly once per ball revolution: it is periodic, not noise.
    std::vector<int32_t> s;
    roll(.5f, 2000, nullptr); // settle the envelope and tube mode
    const long long peak = roll(.5f, 2000, &s);
    const int period = (int)std::lround(48000 * RollingBall::kCircumference / .5f);
    double error = 0, power = 0;
    for (size_t i = 0; i + period < s.size(); i++) { const double d = s[i] - s[i + period]; error += d * d; power += (double)s[i] * s[i]; }
    printf("rolling 0.5 m/s: peak=%lld period=%d samples, repeat error=%.4f\n", peak, period, error / power);
    assert(peak > 15000 && peak < 30000);
    assert(error / power < .02);
    // Faster is stronger; a stopped ball is exactly silent.
    ball.reset(); roll(.1f, 2000, nullptr); const long long slow = roll(.1f, 2000, nullptr);
    assert(slow > 4000 && slow < peak); // a creeping ball is still felt
    roll(0, 4000, nullptr);
    assert(roll(0, 200, nullptr) == 0);
    // A wall hit rings a bounded thud and tick that die away.
    ball.reset(); ball.begin(0, 0, 1, 24);
    long long thud = 0, tick = 0;
    for (int i = 0; i < 48000; i++) {
        if (i % 24 == 0 && i) ball.begin(0, 0, 0, 24);
        ball.next();
        thud = std::max<long long>(thud, std::abs(ball.thud())); tick = std::max<long long>(tick, std::abs(ball.tick()));
        if (i > 24000) assert(ball.thud() == 0 && ball.tick() == 0);
    }
    printf("impact: thud peak=%lld tick peak=%lld\n", thud, tick);
    assert(thud > 8000 && thud <= RollingBall::kThudPeak * 11 / 10);
    assert(tick > 9000 && tick <= RollingBall::kTickPeak * 11 / 10);
}

static void testMooBox()
{
    MooBox moo; moo.reset();
    auto run = [&](float tilt, int ticks, int *landings) {
        long long peak = 0;
        for (int i = 0; i < ticks; i++) {
            if (moo.control(tilt, 24) > 0 && landings) ++*landings;
            for (int f = 0; f < 24; f++) peak = std::max<long long>(peak, std::abs(moo.next()));
        }
        return peak;
    };
    // Level, or tipped less than about 15 degrees: nothing moves, nothing sounds.
    assert(run(0, 1000, nullptr) == 0 && run(.2f, 2000, nullptr) == 0 && moo.position() == 0);
    // Tipped over: the piston accelerates from rest, so the moo swells in.
    int landings = 0;
    const long long start = run(1.f, 600, &landings);  // first 0.3 s
    const long long peak = run(1.f, 2400, &landings);  // 0.3 .. 1.5 s
    assert(start > 300 && start * 3 < peak);
    assert(moo.position() > 0 && moo.position() < 1 && landings == 0);
    run(1.f, 3000, &landings);
    printf("moo: peak=%lld landings=%d\n", peak, landings);
    assert(peak > 12000 && peak < 30000 && landings == 1 && moo.position() == 1);
    run(1.f, 6000, &landings);
    assert(run(1.f, 2000, &landings) == 0 && landings == 1);
    // Tipped back it moos again on the way down.
    assert(run(-1.f, 3000, &landings) > 12000);
}

int main()
{
    testRainstick();
    testRollingBall();
    testMooBox();
    printf("PASS: rainstick flow, levels and silence; rolling ball periodicity and impact ring.\n");
}
