#include "rattle_pulse.h"
#include "rolling_detail.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <vector>

namespace {

constexpr float kSampleRate = 48000.0f;
constexpr float kControlRate = 2000.0f;
constexpr int kSamplesPerControl = 24;

void require(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(EXIT_FAILURE);
    }
}

void checkSample(int16_t sample)
{
    require(sample >= -32767 && sample <= 32767, "sample stays inside signed Q15 bounds");
}

void controlFrame(RollingDetail &detail, float &position, float velocity, float impact = 0.0f)
{
    detail.begin(position, velocity, impact, kControlRate);
    position += velocity * (static_cast<float>(kSamplesPerControl) / kSampleRate);
}

void testInitAndRestSilence()
{
    RollingDetail detail;
    detail.init(kSampleRate);
    for (int i = 0; i < 4096; ++i) {
        const int16_t sample = detail.next();
        checkSample(sample);
        require(sample == 0, "fresh initialized detail path is silent");
    }

    for (int frame = 0; frame < 100; ++frame) {
        detail.begin(0.25f, 0.0f, 0.0f, kControlRate);
        for (int n = 0; n < kSamplesPerControl; ++n)
            require(detail.next() == 0, "resting detail path stays silent");
    }
}

void testSustainedBoundsAndDirections()
{
    constexpr float speeds[] = {-10.0f, -5.0f, -0.8f, -0.4f, 0.4f, 0.8f, 5.0f, 10.0f};
    RollingDetail detail;
    detail.init(kSampleRate);
    float position = 0.5f;
    for (float speed : speeds) {
        for (int frame = 0; frame < 160; ++frame) {
            const float impact = frame == 0 ? 1.0f : 0.0f;
            controlFrame(detail, position, speed, impact);
            for (int n = 0; n < kSamplesPerControl; ++n)
                checkSample(detail.next());
        }
    }
}

void testNoIntegerLimitCycleAfterStop()
{
    RollingDetail detail;
    detail.init(kSampleRate);
    float position = 0.1f;
    for (int frame = 0; frame < 500; ++frame) {
        controlFrame(detail, position, 0.8f);
        for (int n = 0; n < kSamplesPerControl; ++n)
            checkSample(detail.next());
    }

    // Continue feeding a stopped control state so the amplitude and strike
    // envelopes can decay while the resonators ring out.
    for (int frame = 0; frame < 200; ++frame) {
        detail.begin(position, 0.0f, 0.0f, kControlRate);
        for (int n = 0; n < kSamplesPerControl; ++n)
            checkSample(detail.next());
    }

    for (int i = 0; i < 8192; ++i)
        require(detail.next() == 0, "integer resonators decay to exact zero without limit cycles");
}

void testResetDeterministic()
{
    RollingDetail detail;
    detail.init(kSampleRate);
    std::vector<int16_t> first;
    float position = 0.3f;
    for (int frame = 0; frame < 150; ++frame) {
        controlFrame(detail, position, 0.4f, frame == 13 ? 0.7f : 0.0f);
        for (int n = 0; n < kSamplesPerControl; ++n)
            first.push_back(detail.next());
    }

    detail.reset();
    position = 0.3f;
    size_t sample_index = 0;
    for (int frame = 0; frame < 150; ++frame) {
        controlFrame(detail, position, 0.4f, frame == 13 ? 0.7f : 0.0f);
        for (int n = 0; n < kSamplesPerControl; ++n)
            require(detail.next() == first[sample_index++], "reset reproduces an identical Q15 sequence");
    }
}

double goertzelPower(const std::vector<int16_t> &samples, float frequency)
{
    const double omega = 6.283185307179586 * frequency / kSampleRate;
    const double coeff = 2.0 * std::cos(omega);
    double s1 = 0.0, s2 = 0.0;
    for (int16_t sample : samples) {
        const double s0 = sample + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    return s1 * s1 + s2 * s2 - coeff * s1 * s2;
}

struct SignalStats {
    double rms = 0.0;
    int peak = 0;
};

SignalStats sustainedSignal(float speed)
{
    RollingDetail detail;
    detail.init(kSampleRate);
    float position = 0.2f;
    for (int frame = 0; frame < 500; ++frame) {
        controlFrame(detail, position, speed);
        for (int n = 0; n < kSamplesPerControl; ++n)
            detail.next();
    }

    SignalStats stats;
    double square_sum = 0.0;
    int count = 0;
    for (int frame = 0; frame < 2000; ++frame) {
        controlFrame(detail, position, speed);
        for (int n = 0; n < kSamplesPerControl; ++n) {
            const int sample = detail.next();
            square_sum += static_cast<double>(sample) * sample;
            stats.peak = std::max(stats.peak, std::abs(sample));
            ++count;
        }
    }
    stats.rms = std::sqrt(square_sum / count);
    return stats;
}

void testAudioRateEnergyAboveOneKilohertz()
{
    RollingDetail detail;
    detail.init(kSampleRate);
    float position = 0.15f;
    for (int frame = 0; frame < 600; ++frame) {
        controlFrame(detail, position, 0.30f);
        for (int n = 0; n < kSamplesPerControl; ++n)
            detail.next();
    }

    std::vector<int16_t> samples;
    samples.reserve(32768);
    for (int frame = 0; frame < 32768 / kSamplesPerControl; ++frame) {
        controlFrame(detail, position, 0.30f);
        for (int n = 0; n < kSamplesPerControl; ++n) {
            const int16_t sample = detail.next();
            checkSample(sample);
            samples.push_back(sample);
        }
    }

    const double total_energy = goertzelPower(samples, 700.0f);
    const double high_energy = goertzelPower(samples, 1800.0f) + goertzelPower(samples, 3600.0f);
    require(total_energy > 1.0, "700 Hz resonator provides a nonzero spectral reference");
    std::printf("detail Goertzel 1.8+3.6 kHz / 700 Hz energy ratio %.4f\n",
                high_energy / total_energy);
    require(high_energy > total_energy * 0.002,
            "Goertzel detects resonant energy above 1 kHz during fine-surface traversal");
}

void testImpactRing()
{
    RollingDetail detail;
    detail.init(kSampleRate);
    float position = 0.5f;
    detail.begin(position, 0.0f, 1.0f, kControlRate);
    int peak = 0;
    for (int i = 0; i < 2048; ++i) {
        const int sample = detail.next();
        checkSample(static_cast<int16_t>(sample));
        peak = std::max(peak, std::abs(sample));
    }
    require(peak > 100, "stationary impact excites a measurable high-frequency ring");
    for (int i = 0; i < 4096; ++i)
        require(detail.next() == 0, "impact resonators decay to exact zero");
}

void testReportTypicalLevel()
{
    const SignalStats at_04 = sustainedSignal(0.4f);
    const SignalStats at_08 = sustainedSignal(0.8f);
    std::printf("detail output: 0.4 m/s RMS %.4f FS, peak %.4f FS; 0.8 m/s RMS %.4f FS, peak %.4f FS\n",
                at_04.rms / 32767.0, at_04.peak / 32767.0,
                at_08.rms / 32767.0, at_08.peak / 32767.0);
    require(at_04.rms > 1.0 && at_08.rms > 1.0, "typical rolling speeds produce measurable audio detail");
}

} // namespace

int main()
{
    RattlePulse one, many;
    one.reset(); many.reset();
    float single_energy=0, dense_energy=0;
    for (int i=0;i<32;++i) {
        const float a=one.step(i==0 ? .2f : 0,2000);
        const float b=many.step(i==0 || i==3 ? .2f : 0,2000);
        require(a>=0 && a<=1 && b>=0 && b<=1,"rattle pulses stay bounded");
        single_energy+=a; dense_energy+=b;
        if (i>12) require(a==0 && b==0,"rattle contacts decay to exact silence");
    }
    require(dense_energy>single_energy*1.8f,"distinct close contacts accumulate rather than restart");
    many.step(1,2000); many.reset();
    require(many.step(0,2000)==0,"new empty round clears preceding contacts");
    testInitAndRestSilence();
    testSustainedBoundsAndDirections();
    testNoIntegerLimitCycleAfterStop();
    testResetDeterministic();
    testAudioRateEnergyAboveOneKilohertz();
    testImpactRing();
    for (float speed : {0.005f, 0.010f, 0.020f})
        require(sustainedSignal(speed).rms > 1.0, "gentle rolling produces full-rate HF detail");
    require(sustainedSignal(0.001f).peak == 0, "HF deadband stays silent");
    testReportTypicalLevel();
    std::puts("All rolling detail regressions passed.");
    return EXIT_SUCCESS;
}
