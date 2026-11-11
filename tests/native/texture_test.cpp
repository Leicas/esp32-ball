#include "rolling_texture.h"
#include "rolling_response.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

namespace {

void require(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(EXIT_FAILURE);
    }
}

void near(float actual, float expected, float tolerance, const char *message)
{
    if (fabsf(actual - expected) > tolerance) {
        std::fprintf(stderr, "FAIL: %s (got %.9f, expected %.9f)\n", message, actual, expected);
        std::exit(EXIT_FAILURE);
    }
}

void checkOutput(RollingTexture::Output out)
{
    require(std::isfinite(out.low) && std::isfinite(out.high), "DSP outputs remain finite");
    require(out.low >= -0.80001f && out.low <= 0.00001f, "low output stays negative and bounded");
    require(out.high >= -0.80001f && out.high <= 0.80001f, "high output stays bounded");
}

void testSilenceAtRestAndDecay(float rate)
{
    RollingTexture texture;
    texture.init(rate);
    for (int i = 0; i < static_cast<int>(rate * 0.1f); ++i) {
        const auto out = texture.step(0.25f, 0.0f, 0.0f);
        checkOutput(out);
        require(out.low == 0.0f && out.high == 0.0f, "fresh texture is silent at rest");
    }

    float position = 0.25f;
    for (int i = 0; i < static_cast<int>(rate); ++i) {
        position += 0.35f / rate;
        checkOutput(texture.step(position, 0.35f, 0.0f));
    }

    float tail_peak = 0.0f;
    for (int i = 0; i < static_cast<int>(rate * 2.5f); ++i) {
        const auto out = texture.step(position, 0.0f, 0.0f);
        checkOutput(out);
        if (i >= static_cast<int>(rate * 2.4f))
            tail_peak = fmaxf(tail_peak, fmaxf(fabsf(out.low), fabsf(out.high)));
    }
    require(tail_peak < 1e-5f, "rolling texture decays to silence after motion stops");
}

void testImpactExcitesBoundedRing(float rate)
{
    RollingTexture texture;
    texture.init(rate);
    float peak = 0.0f;
    for (int i = 0; i < static_cast<int>(rate * 0.5f); ++i) {
        const float hit = i == 0 ? 1.0f : 0.0f;
        const auto out = texture.step(0.4f, 0.0f, hit);
        checkOutput(out);
        peak = fmaxf(peak, fabsf(out.high));
    }
    require(peak > 0.05f, "impact excites the resonant high-frequency output at rest");

    for (int i = 0; i < static_cast<int>(rate * 1.5f); ++i) {
        const auto out = texture.step(0.4f, 0.0f, 0.0f);
        checkOutput(out);
        if (i > rate)
            require(fabsf(out.high) < 1e-4f, "impact ring decays instead of sustaining");
    }
}

struct TextureStats {
    float low_rms = 0.0f;
    int positive_high = 0;
    int negative_high = 0;
    int changed_high = 0;
    float high_peak = 0.0f;
};

TextureStats runTexture(float rate, float speed, bool inject_impacts)
{
    RollingTexture texture;
    texture.init(rate);
    TextureStats stats;
    float position = 0.02f;
    float low_square_sum = 0.0f;
    float previous_high = 0.0f;
    const int count = static_cast<int>(rate * 2.0f);
    const int start_measure = static_cast<int>(rate * 0.5f);
    for (int i = 0; i < count; ++i) {
        position += speed / rate;
        const float hit = inject_impacts && i % (static_cast<int>(rate * 0.2f)) == 0 ? 1.0f : 0.0f;
        const auto out = texture.step(position, speed, hit);
        checkOutput(out);
        if (i >= start_measure) {
            low_square_sum += out.low * out.low;
            stats.positive_high += out.high > 1e-4f;
            stats.negative_high += out.high < -1e-4f;
            stats.changed_high += fabsf(out.high - previous_high) > 1e-4f;
            stats.high_peak = fmaxf(stats.high_peak, fabsf(out.high));
        }
        previous_high = out.high;
    }
    stats.low_rms = sqrtf(low_square_sum / static_cast<float>(count - start_measure));
    return stats;
}

void testSustainedTextureAndSpeedResponse(float rate)
{
    const TextureStats slow = runTexture(rate, 0.10f, false);
    const TextureStats fast = runTexture(rate, 0.80f, true);

    require(slow.low_rms > 0.02f, "moving texture produces a low-frequency rolling body");
    require(fast.low_rms > slow.low_rms * 1.5f,
            "higher rolling speed produces higher sustained low-channel energy");
    require(fast.positive_high > 20 && fast.negative_high > 20,
            "rolling and impacts produce bipolar resonant texture");
    require(fast.changed_high > static_cast<int>(rate * 0.25f),
            "high output varies over the surface instead of repeating one arch");
    require(fast.high_peak > 0.05f, "impacts create a visible resonant response during motion");
}

void testTextureDoesNotRepeatAtThirtyMillimeters(float rate)
{
    RollingTexture texture;
    texture.init(rate);
    const float speed = 0.30f;
    const int count = static_cast<int>(rate * 2.0f);
    const int period_samples = static_cast<int>(rate * (0.030f / speed) + 0.5f);
    std::vector<RollingTexture::Output> samples;
    samples.reserve(count);

    float position = 0.02f;
    for (int i = 0; i < count; ++i) {
        position += speed / rate;
        const auto out = texture.step(position, speed, 0.0f); // no impacts
        checkOutput(out);
        samples.push_back(out);
    }

    float difference_energy = 0.0f;
    float reference_energy = 0.0f;
    int comparisons = 0;
    const int settled_start = static_cast<int>(rate * 0.5f);
    for (int i = settled_start; i + period_samples < count; ++i) {
        const auto a = samples[i];
        const auto b = samples[i + period_samples];
        const float d_low = a.low - b.low;
        const float d_high = a.high - b.high;
        difference_energy += d_low * d_low + d_high * d_high;
        reference_energy += a.low * a.low + a.high * a.high;
        ++comparisons;
    }

    require(comparisons > 0 && reference_energy > 1e-4f,
            "30 mm repeat comparison has settled, nonzero rolling samples");
    const float normalized_difference = sqrtf(difference_energy / reference_energy);
    require(normalized_difference > 0.05f,
            "texture does not repeat after the classic 30 mm travel period");
}

void testExtremeSignedSpeedsStayBounded(float rate)
{
    RollingTexture texture;
    texture.init(rate);
    constexpr float speeds[] = {-5.0f, -2.0f, -0.1f, 0.0f, 0.1f, 2.0f, 5.0f};
    float position = 0.5f;
    for (float speed : speeds) {
        for (int i = 0; i < static_cast<int>(rate * 0.1f); ++i) {
            position += speed / rate;
            const float impact = i == 0 ? 1.0f : 0.0f;
            checkOutput(texture.step(position, speed, impact));
        }
    }
}

void testResetIsDeterministic(float rate)
{
    RollingTexture texture;
    texture.init(rate);
    std::vector<RollingTexture::Output> first;
    float position = 0.1f;
    for (int i = 0; i < static_cast<int>(rate * 0.3f); ++i) {
        position += 0.42f / rate;
        first.push_back(texture.step(position, 0.42f, i == 37 ? 0.8f : 0.0f));
    }

    texture.reset();
    position = 0.1f;
    for (size_t i = 0; i < first.size(); ++i) {
        position += 0.42f / rate;
        const auto actual = texture.step(position, 0.42f, i == 37 ? 0.8f : 0.0f);
        near(actual.low, first[i].low, 0.0f, "reset reproduces low output deterministically");
        near(actual.high, first[i].high, 0.0f, "reset reproduces high output deterministically");
    }
}

void testInvalidInputResets(float rate)
{
    RollingTexture texture;
    texture.init(rate);
    const auto bad = texture.step(std::numeric_limits<float>::quiet_NaN(), 1.0f, 0.0f);
    require(bad.low == 0.0f && bad.high == 0.0f, "non-finite input resets to silence");
    const auto next = texture.step(0.2f, 0.0f, 0.0f);
    require(next.low == 0.0f && next.high == 0.0f, "invalid input clears resonator state");
}

void runRateSuite(float rate)
{
    for (float speed : {0.005f, 0.010f, 0.020f}) {
        const auto slow = runTexture(rate, speed, false);
        require(slow.low_rms > 0.005f && slow.high_peak > 0.0001f,
                "gentle rolling drives both body and texture without impacts");
    }
    require(runTexture(rate, 0.001f, false).low_rms == 0.0f,
            "sub-deadband motion remains silent");
    for (float speed : {0.20f, 0.4f, 0.8f, 2.0f}) {
        near(rollingTextureEnvelope(speed), sqrtf(fminf((speed - .015f) / 1.5f, 1.0f)), 1e-6f,
             "faster texture envelope is unchanged");
        near(rollingClassicEnvelope(speed), fminf(speed / 2.0f, 1.0f), 1e-6f,
             "faster classic envelope is unchanged");
    }
    require(rollingTextureEnvelope(.02f) > 2 * sqrtf(.005f / 1.5f),
            "20 mm/s texture has at least twice the previous drive envelope");
    testSilenceAtRestAndDecay(rate);
    testImpactExcitesBoundedRing(rate);
    testSustainedTextureAndSpeedResponse(rate);
    testTextureDoesNotRepeatAtThirtyMillimeters(rate);
    testExtremeSignedSpeedsStayBounded(rate);
    testResetIsDeterministic(rate);
    testInvalidInputResets(rate);
}

} // namespace

int main()
{
    runRateSuite(1000.0f);
    runRateSuite(2000.0f);
    std::puts("All rolling texture regressions passed.");
    return EXIT_SUCCESS;
}
