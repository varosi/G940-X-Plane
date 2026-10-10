#include "g940CueModel.h"
#include <cassert>
#include <cstdio>
#include <limits>

namespace {
using namespace g940;
constexpr float dt = .01f;
constexpr float quietNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float infinity = std::numeric_limits<float>::infinity();
bool near(float a, float b, float tolerance = 1e-6f) {
    return std::abs(a - b) <= tolerance;
}
void silent(CueOffsets cue, float tolerance = 1e-6f) {
    assert(std::isfinite(cue.roll) && std::isfinite(cue.pitch));
    assert(std::abs(cue.roll) <= tolerance && std::abs(cue.pitch) <= tolerance);
}
void bounded(CueOffsets cue) {
    assert(std::isfinite(cue.roll) && std::isfinite(cue.pitch));
    assert(std::abs(cue.roll) <= maximumCueOffset);
    assert(std::abs(cue.pitch) <= maximumCueOffset);
}
CueSample flight() {
    CueSample sample;
    sample.airborne = sample.windValid = true;
    sample.pressurePa = 1000.0f;
    return sample;
}
AircraftProfile enabled() {
    auto profile = defaultProfile;
    profile.turbulenceGain = .02f;
    profile.stallBuffetGain = .08f;
    return profile;
}
CueOffsets gust(float heading, float bank, std::array<float, 3> wind, float pitch = 0.0f,
                float gain = .02f) {
    FlightCues cues;
    auto sample = flight();
    sample.heading = heading;
    sample.roll = bank;
    sample.pitch = pitch;
    auto profile = enabled();
    profile.turbulenceGain = gain;
    silent(cues.update(sample, profile, dt)); // baseline, not a gust
    sample.wind = wind;
    CueOffsets output;
    for (int i = 0; i < 20; ++i) output = cues.update(sample, profile, dt);
    bounded(output);
    return output;
}
struct BuffetStatistics { float rms, mean, peak; };
BuffetStatistics buffet(float power, float separation = .2f, float pressure = 1000.0f,
                        float frequency = 5.0f) {
    FlightCues cues;
    auto sample = flight();
    sample.windValid = false;
    sample.stalledFraction = separation;
    sample.enginePower = power;
    sample.pressurePa = pressure;
    auto profile = enabled();
    profile.stallBuffetHz = frequency;
    float sum = 0.0f, squares = 0.0f, peak = 0.0f;
    CueOffsets previous;
    constexpr int warmup = 300, samples = 6000;
    for (int i = 0; i < warmup + samples; ++i) {
        const auto output = cues.update(sample, profile, dt);
        bounded(output);
        assert(output.roll == 0.0f);
        assert(std::abs(output.pitch - previous.pitch) <= 3.0f * dt + 1e-6f);
        previous = output;
        if (i >= warmup) {
            sum += output.pitch;
            squares += output.pitch * output.pitch;
            peak = std::max(peak, std::abs(output.pitch));
        }
    }
    return {std::sqrt(squares / samples), sum / samples, peak};
}
}

int main() {
    using namespace g940;
    // The 80 main-wing slots contribute by area; tail/control-surface slots
    // must not activate buffet even if their telemetry is invalid.
    std::array<float, 100> stalled{}, area{};
    area[0] = 1.0f; area[1] = 3.0f;
    stalled[1] = 1.0f;
    area[80] = stalled[80] = quietNaN;
    assert(near(stalledWingFraction(stalled, area), .75f));
    stalled[0] = .5f; stalled[1] = .25f;
    assert(near(stalledWingFraction(stalled, area), .3125f));
    assert(std::isnan(stalledWingFraction({}, area)));
    assert(std::isnan(stalledWingFraction(stalled, {})));
    assert(std::isnan(stalledWingFraction(std::span(stalled).first(79), area)));
    assert(std::isnan(stalledWingFraction(stalled, std::span(area).first(79))));
    area[0] = area[1] = 0.0f;
    assert(std::isnan(stalledWingFraction(stalled, area)));
    area[0] = 1.0f;
    for (float invalid : {quietNaN, infinity, -1.0f}) {
        area[1] = invalid;
        assert(std::isnan(stalledWingFraction(stalled, area)));
    }
    area[1] = 3.0f;
    for (float invalid : {quietNaN, infinity, -.001f, 1.001f}) {
        stalled[2] = invalid; // invalid zero-area slots are not trustworthy
        assert(std::isnan(stalledWingFraction(stalled, area)));
    }
    stalled[2] = 0.0f;
    area[0] = area[1] = std::numeric_limits<float>::max();
    assert(std::isnan(stalledWingFraction(stalled, area)));

    assert(defaultProfile.turbulenceGain == 0.0f && defaultProfile.stallBuffetGain == 0.0f);
    auto profile = enabled();
    auto sample = flight();
    FlightCues cues;
    sample.wind = {12.0f, -2.0f, 5.0f};
    for (int i = 0; i < 1200; ++i) {
        sample.heading = static_cast<float>(i) * 1.3f;
        sample.pitch = std::sin(i * .1f) * 25.0f;
        sample.roll = std::sin(i * .07f) * 60.0f;
        silent(cues.update(sample, profile, dt)); // turning in steady wind is silent
    }
    cues.reset();
    sample = flight();
    sample.windValid = false;
    for (int i = 0; i < 3000; ++i) silent(cues.update(sample, profile, dt));
    sample.stalledFraction = .2f; sample.enginePower = 1.0f;
    for (int i = 0; i < 300; ++i) silent(cues.update(sample, defaultProfile, dt));

    const auto east = gust(0, 0, {3, 0, 0});
    const auto westHeading = gust(180, 0, {3, 0, 0});
    assert(east.roll < -.01f && near(east.pitch, 0.0f));
    assert(near(gust(0, 0, {6, 0, 0}).roll, 2 * east.roll));
    assert(near(gust(0, 0, {3, 0, 0}, 0, .04f).roll, 2 * east.roll));
    silent(gust(0, 0, {3, 0, 0}, 0, 0));
    assert(near(east.roll, -westHeading.roll) && near(westHeading.pitch, 0.0f));
    assert(near(gust(360, 0, {3, 0, 0}).roll, east.roll));
    silent(gust(90, 0, {3, 0, 0}), 1e-5f); // same gust along the longitudinal axis
    const auto up = gust(0, 0, {0, 3, 0});
    assert(up.pitch < -.01f && near(up.roll, 0.0f));
    const auto banked = gust(0, 90, {3, 0, 0});
    assert(near(banked.pitch, east.roll) && near(banked.roll, 0.0f));
    const auto bankedUp = gust(0, 90, {0, 3, 0});
    assert(near(bankedUp.roll, -up.pitch) && near(bankedUp.pitch, 0.0f));
    assert(gust(0, 0, {0, 0, 3}, 30).pitch < 0.0f); // pitch projects vertical flow

    cues.reset(); sample = flight();
    silent(cues.update(sample, profile, dt));
    sample.wind[0] = 3.0f;
    CueOffsets output;
    for (int i = 0; i < 20; ++i) output = cues.update(sample, profile, dt);
    assert(output.roll < -.01f);
    for (int i = 0; i < 1200; ++i) output = cues.update(sample, profile, dt);
    silent(output, 1e-5f); // a lasting new wind becomes the steady baseline

    const auto idle = buffet(0.0f), powered = buffet(1.0f), missingPower = buffet(quietNaN);
    assert(powered.rms > .025f && powered.rms < .07f);
    assert(near(idle.rms / powered.rms, profile.stallBuffetIdleRatio, .002f));
    assert(near(idle.rms, missingPower.rms));
    assert(near(idle.rms, buffet(infinity).rms));
    assert(std::abs(powered.mean) < .001f && std::abs(idle.mean) < .001f);
    assert(powered.peak <= profile.stallBuffetGain + 1e-6f);
    assert(buffet(1.0f, 0.0f).rms == 0.0f && buffet(1.0f, quietNaN).rms == 0.0f);
    assert(near(buffet(1.0f, .1f).rms / powered.rms, .5f, .002f));
    assert(near(buffet(1.0f, .2f, 50).rms / powered.rms, .5f, .002f));
    assert(buffet(1.0f, .2f, 1000, 2).rms > 0.0f);
    assert(buffet(1.0f, .2f, 1000, 6).rms > 0.0f);

    // Ground or missing pressure inhibits both cues, even with a stalled wing
    // and a large weather change. Optional channels fail independently.
    for (float pressure : {0.0f, -1.0f, quietNaN, infinity}) {
        cues.reset(); sample = flight(); sample.pressurePa = pressure;
        sample.stalledFraction = .2f; sample.enginePower = 1.0f;
        for (int i = 0; i < 100; ++i) silent(cues.update(sample, profile, dt));
    }
    cues.reset(); sample = flight(); sample.airborne = false;
    sample.stalledFraction = .2f; sample.enginePower = 1.0f;
    for (int i = 0; i < 100; ++i) silent(cues.update(sample, profile, dt));
    for (float invalidWind : {quietNaN, infinity, 201.0f}) {
        cues.reset(); sample = flight(); sample.wind[0] = invalidWind;
        for (int i = 0; i < 100; ++i) silent(cues.update(sample, profile, dt));
    }
    for (auto angle : {&CueSample::heading, &CueSample::pitch, &CueSample::roll}) {
        cues.reset(); sample = flight(); sample.*angle = quietNaN;
        for (int i = 0; i < 100; ++i) silent(cues.update(sample, profile, dt));
    }
    cues.reset(); sample = flight();
    silent(cues.update(sample, profile, dt));
    sample.wind[0] = 3.0f;
    for (int i = 0; i < 20; ++i) cues.update(sample, profile, dt);
    sample.windValid = false;
    for (int i = 0; i < 100; ++i) output = cues.update(sample, profile, dt);
    silent(output);
    sample.windValid = true; sample.wind[0] = 8.0f;
    silent(cues.update(sample, profile, dt)); // missing telemetry resumes at a fresh baseline
    // Missing wind leaves genuine wing-separation buffet available.
    cues.reset(); sample = flight(); sample.windValid = false;
    sample.stalledFraction = .2f; sample.enginePower = 1.0f;
    bool buffetPresent = false;
    for (int i = 0; i < 300; ++i) {
        output = cues.update(sample, profile, dt);
        buffetPresent |= std::abs(output.pitch) > .01f;
    }
    assert(buffetPresent);

    for (float invalidElapsed : {quietNaN, infinity, 0.0f, -.01f}) {
        cues.reset(); sample = flight();
        silent(cues.update(sample, profile, dt));
        sample.wind[0] = 3.0f;
        for (int i = 0; i < 20; ++i) cues.update(sample, profile, dt);
        silent(cues.update(sample, profile, invalidElapsed));
        silent(cues.update(sample, profile, dt)); // re-prime, no recovery impulse
    }
    // Slow callbacks suppress the waveform while fading the existing load.
    // They must not drop a nonzero cue instantly or create a recovery gust.
    for (float delayedElapsed : {.051f, .06f, .1f, 1.0f, 10.0f}) {
        cues.reset(); sample = flight();
        silent(cues.update(sample, profile, dt));
        sample.wind = {3, 3, 0};
        for (int i = 0; i < 20; ++i) output = cues.update(sample, profile, dt);
        const auto beforeDelay = output;
        assert(beforeDelay.roll < -.01f && beforeDelay.pitch < -.01f);
        output = cues.update(sample, profile, delayedElapsed);
        assert(output.roll <= 0 && output.pitch <= 0);
        if (delayedElapsed <= .1f) assert(output.roll < 0 && output.pitch < 0);
        assert(std::abs(output.roll) < std::abs(beforeDelay.roll));
        assert(std::abs(output.pitch) < std::abs(beforeDelay.pitch));
        sample.wind = {8, 8, 0};
        for (int i = 0; i < 100; ++i) {
            const auto previous = output;
            output = cues.update(sample, profile, dt);
            assert(std::abs(output.roll) <= std::abs(previous.roll) + 1e-6f);
            assert(std::abs(output.pitch) <= std::abs(previous.pitch) + 1e-6f);
        }
        silent(output);
    }
    for (float invalidElapsed : {quietNaN, infinity, 0.0f, -.01f}) {
        cues.reset(); sample = flight();
        silent(cues.update(sample, profile, dt));
        sample.wind = {3, 3, 0};
        for (int i = 0; i < 20; ++i) output = cues.update(sample, profile, dt);
        assert(output.roll < -.01f && output.pitch < -.01f);
        silent(cues.release(invalidElapsed));
        silent(cues.update(sample, profile, dt));
    }
    cues.reset(); sample = flight();
    silent(cues.update(sample, profile, dt));
    sample.wind[0] = 3.0f;
    for (int i = 0; i < 20; ++i) output = cues.update(sample, profile, dt);
    const auto beforePause = output;
    output = cues.release(dt);
    assert(near(output.roll / beforePause.roll, std::exp(-dt / .12f)));
    for (int i = 0; i < 100; ++i) output = cues.release(dt);
    silent(output, .0001f);
    silent(cues.update(sample, profile, dt)); // paused weather changes do not become a gust
    cues.reset();
    silent(cues.update(sample, profile, dt));
    sample.wind[0] = 40.0f; // weather edit/teleport, not a physical gust
    silent(cues.update(sample, profile, dt));
    for (int i = 0; i < 50; ++i) silent(cues.update(sample, profile, dt));

    for (auto member : {&AircraftProfile::turbulenceGain, &AircraftProfile::stallBuffetGain,
                        &AircraftProfile::stallBuffetIdleRatio, &AircraftProfile::stallBuffetHz}) {
        for (float invalid : {quietNaN, infinity, -1.0f}) {
            auto invalidProfile = profile;
            invalidProfile.*member = invalid;
            cues.reset();
            silent(cues.update(sample, invalidProfile, dt));
        }
    }
    auto invalidProfile = profile;
    invalidProfile.stallBuffetHz = 6.01f;
    silent(cues.update(sample, invalidProfile, dt));
    invalidProfile.stallBuffetHz = 1.99f;
    silent(cues.update(sample, invalidProfile, dt));
    invalidProfile = profile; invalidProfile.turbulenceGain = .101f;
    silent(cues.update(sample, invalidProfile, dt));
    invalidProfile = profile; invalidProfile.stallBuffetGain = maximumCueOffset + .001f;
    silent(cues.update(sample, invalidProfile, dt));
    invalidProfile = profile; invalidProfile.stallBuffetIdleRatio = 1.001f;
    silent(cues.update(sample, invalidProfile, dt));

    // Cue offsets are applied after trim smoothing without changing the trim
    // target, measured velocity, or restoring demand stored by the base model.
    ForceState base = calculateForce(.1f, -.1f, defaultProfile.referencePressurePa, 0, .2f, .1f);
    const auto decorated = withCues(base, {.04f, -.05f});
    assert(decorated.roll == base.roll && decorated.pitch == base.pitch);
    assert(decorated.rollForce == base.rollForce && decorated.pitchForce == base.pitchForce);
    assert(decorated.rollVelocity == base.rollVelocity && decorated.pitchVelocity == base.pitchVelocity);
    assert(decorated.springRatio() == base.springRatio() && decorated.dampingRatio == base.dampingRatio);
    assert(decorated.effectScale == base.effectScale);
    assert(near(decorated.center(0), base.roll + .04f));
    assert(near(decorated.center(1), base.pitch - .05f));
    base.roll = .98f; base.pitch = -.97f;
    const auto positive = withCues(base, {.12f, .12f});
    const auto negative = withCues(base, {-.12f, -.12f});
    assert(near(positive.rollCue, .02f) && near(negative.rollCue, -.02f));
    assert(near(positive.pitchCue, .03f) && near(negative.pitchCue, -.03f));
    assert(near((positive.center(0) + negative.center(0)) / 2, base.roll));
    assert(near((positive.center(1) + negative.center(1)) / 2, base.pitch));
    base.roll = 1.0f; base.pitch = -1.0f;
    silent({withCues(base, {.1f, -.1f}).rollCue, withCues(base, {.1f, -.1f}).pitchCue});
    base.mechanicalRatio = base.aerodynamicRatio = 0.0f; base.dampingRatio = .5f;
    assert(base.hasLoad());
    const auto dampingOnly = withCues(base, {.1f, -.1f});
    assert(dampingOnly.rollCue == 0 && dampingOnly.pitchCue == 0);

    base = calculateForce(0, 0, defaultProfile.referencePressurePa, 0, 0, 0);
    const auto actual = withCues(base, {.04f, -.05f});
    const auto report = forceReport(actual);
    const auto idleCenters = idleCenterReport(report);
    for (unsigned axis = 0; axis < 2; ++axis) {
        const unsigned centerByte = 7 + 30 * axis;
        assert(idleCenters[1 + 2 * axis] == report[centerByte]);
        assert(idleCenters[2 + 2 * axis] == report[centerByte + 1]);
        const int expected = static_cast<int>(actual.center(axis) * 32767.0f);
        assert(report[centerByte] == static_cast<uint8_t>(expected & 0xff));
        assert(report[centerByte + 1] == static_cast<uint8_t>((expected >> 8) & 0xff));
    }
    const auto constant = constantForceComponents(actual);
    assert(near(constant[0], .04f * springCoefficient(base.springRatio(), 0) / 64.0f));
    assert(near(constant[1], -.05f * springCoefficient(base.springRatio(), 1) / 64.0f));
    auto faded = actual; faded.effectScale = 0.0f;
    assert(forceReport(faded) == stopReport());

    std::puts("Wing separation, wind response, bounded buffet, cue lifecycle and backend parity passed.");
}
