#include "g940GroundModel.h"
#include <cassert>
#include <cstdio>
#include <limits>

namespace {
using namespace g940;
constexpr float dt = .01f;
constexpr float invalid = std::numeric_limits<float>::quiet_NaN();
constexpr float infinity = std::numeric_limits<float>::infinity();
AircraftProfile enabled() {
    auto profile = defaultProfile;
    profile.groundBumpGain = .03f;
    profile.landingBumpGain = .06f;
    return profile;
}
GroundSample sample(bool ground = true, float speed = 5.0f) {
    GroundSample result;
    result.contactValid = true;
    result.onGround = ground;
    result.supportG = ground ? 1.0f : 0.0f;
    result.rollAcceleration = result.pitchAcceleration = 0.0f;
    result.groundSpeed = speed;
    result.verticalSpeed = 0.0f;
    return result;
}
void quiet(CueOffsets value, float tolerance = 1e-6f) {
    assert(std::isfinite(value.roll) && std::isfinite(value.pitch));
    assert(std::abs(value.roll) <= tolerance && std::abs(value.pitch) <= tolerance);
}
void bounded(CueOffsets value) {
    assert(std::isfinite(value.roll) && std::isfinite(value.pitch));
    assert(std::abs(value.roll) <= maximumCueOffset && std::abs(value.pitch) <= maximumCueOffset);
}
CueOffsets advance(GroundCues& cues, const GroundSample& state, const AircraftProfile& profile,
                   int frames = 100) {
    CueOffsets result;
    for (int i = 0; i < frames; ++i) {
        const auto previous = result;
        result = cues.update(state, profile, dt);
        bounded(result);
        if (i) {
            assert(std::abs(result.roll - previous.roll) <= 3 * dt + 1e-6f);
            assert(std::abs(result.pitch - previous.pitch) <= 3 * dt + 1e-6f);
        }
    }
    return result;
}
float landingPeak(float load, float speed = 0.0f, int transferFrames = 1) {
    GroundCues cues;
    auto state = sample(false, speed);
    auto profile = enabled();
    quiet(advance(cues, state, profile, 30));
    state.onGround = true;
    float peak = 0.0f;
    for (int i = 0; i < transferFrames + 30; ++i) {
        state.supportG = load * std::min(1.0f, (i + 1.0f) / transferFrames);
        const auto output = cues.update(state, profile, dt);
        bounded(output);
        assert(output.roll == 0 && output.pitch <= 1e-6f);
        peak = std::max(peak, -output.pitch);
    }
    return peak;
}
float taxiPeak(float speed) {
    GroundCues cues;
    auto state = sample(true, speed);
    auto profile = enabled();
    quiet(advance(cues, state, profile));
    state.supportG = 2.0f;
    float peak = 0;
    for (int i = 0; i < 30; ++i) {
        const auto output = cues.update(state, profile, dt);
        bounded(output);
        peak = std::max(peak, -output.pitch);
    }
    return peak;
}
}

int main() {
    using namespace g940;
    assert(defaultProfile.groundBumpGain == 0 && defaultProfile.landingBumpGain == 0);
    auto profile = enabled();
    GroundCues cues;
    auto state = sample();
    quiet(advance(cues, state, profile, 1000)); // steady support on a smooth runway is silent
    state.supportG = 3;
    state.rollAcceleration = 60;
    state.pitchAcceleration = 30;
    cues.reset();
    quiet(advance(cues, state, profile)); // startup values prime histories, rather than pulling the stick
    state.groundSpeed = 0;
    state.supportG = 4;
    quiet(advance(cues, state, profile)); // parked aircraft cannot invent a taxi bump
    state.groundSpeed = 5;
    quiet(advance(cues, state, profile, 500)); // steady telemetry at the new speed remains quiet
    cues.reset(); state = sample();
    for (int i = 0; i < 300; ++i) {
        state.supportG = 1 + std::sin(i * .2f);
        state.rollAcceleration = std::sin(i * .4f) * 60;
        quiet(cues.update(state, defaultProfile, dt));
    }

    const float sharp = landingPeak(1.2f), soft = landingPeak(.4f);
    assert(sharp > .01f && soft > .001f && sharp > soft * 2.5f);
    assert(landingPeak(1.2f, 0, 30) < sharp); // gradual weight transfer feels softer than a sharp impulse
    assert(std::abs(landingPeak(1.2f, 40) - sharp) < 1e-6f); // actual landing load, not taxi speed, sets the impact
    assert(taxiPeak(0) == 0 && taxiPeak(.5f) == 0);
    assert(taxiPeak(invalid) == 0);
    const float taxiFast = taxiPeak(2);
    assert(taxiFast > .005f && std::abs(taxiPeak(1.25f) / taxiFast - .5f) < .01f);
    assert(std::abs(taxiPeak(10) - taxiFast) < 1e-6f);
    assert(sharp > taxiFast); // independently configured landing and taxi strengths

    cues.reset(); state = sample(false, 0); state.supportG = 1;
    quiet(advance(cues, state, profile, 30));
    state.onGround = true;
    quiet(advance(cues, state, profile)); // contact alone never creates an artificial landing pulse
    cues.reset(); state = sample(false, 0);
    quiet(advance(cues, state, profile, 10)); // too little observed flight to arm a landing
    state.onGround = true; state.supportG = 2;
    quiet(advance(cues, state, profile));

    cues.reset(); state = sample(false, 0);
    quiet(advance(cues, state, profile, 30));
    state.onGround = true; state.supportG = 2;
    assert(advance(cues, state, profile, 10).pitch < -.005f);
    state.onGround = false; state.supportG = 0;
    advance(cues, state, profile, 10); // short bounce retains the recent landing window
    state.onGround = true; state.supportG = 1.5f;
    assert(advance(cues, state, profile, 10).pitch < -.001f);
    quiet(advance(cues, state, profile, 400)); // stationary support and the landing window settle away
    state.supportG = 3;
    quiet(advance(cues, state, profile)); // landing boost does not remain armed forever
    cues.reset(); state = sample(false, 0);
    quiet(advance(cues, state, profile, 30));
    state.onGround = true; state.supportG = 1;
    auto expiryOutput = advance(cues, state, profile, 196);
    state.supportG = 2; // a genuine load change while the landing window is ending
    float expiryPeak = 0;
    for (int i = 0; i < 20; ++i) {
        const auto previous = expiryOutput;
        expiryOutput = cues.update(state, profile, dt);
        bounded(expiryOutput);
        assert(std::abs(expiryOutput.pitch - previous.pitch) < .003f);
        expiryPeak = std::max(expiryPeak, std::abs(expiryOutput.pitch));
    }
    assert(expiryPeak > .00001f);
    quiet(expiryOutput); // active loads fade with the window instead of switching off abruptly

    // Periodic roughness and asymmetric rotational acceleration come from
    // actual channels; smooth samples never receive random shake.
    cues.reset(); state = sample(); quiet(advance(cues, state, profile));
    float squares = 0, sum = 0;
    bool positiveRoll = false, negativeRoll = false;
    constexpr int roughFrames = 1000;
    for (int i = 0; i < roughFrames; ++i) {
        state.supportG = 1 + .2f * std::sin(i * .18f);
        state.rollAcceleration = 12 * std::sin(i * .25f);
        const auto output = cues.update(state, profile, dt);
        bounded(output);
        positiveRoll |= output.roll > .001f;
        negativeRoll |= output.roll < -.001f;
        squares += output.pitch * output.pitch;
        sum += output.pitch;
    }
    assert(std::sqrt(squares / roughFrames) > .001f);
    assert(std::abs(sum / roughFrames) < .001f && positiveRoll && negativeRoll);
    cues.reset(); state = sample(); quiet(advance(cues, state, profile));
    state.rollAcceleration = 60;
    const auto rollBump = advance(cues, state, profile, 10);
    assert(rollBump.roll < -.005f && rollBump.pitch == 0);
    cues.reset(); state = sample(); quiet(advance(cues, state, profile));
    state.pitchAcceleration = 60;
    const auto pitchBump = advance(cues, state, profile, 10);
    assert(pitchBump.pitch < -.005f && pitchBump.roll == 0);

    // Each missing channel is independent, and its recovery establishes a
    // new baseline. Bad support must not disable valid rotational feedback.
    for (float bad : {invalid, infinity}) {
        cues.reset(); state = sample(); state.supportG = bad;
        quiet(advance(cues, state, profile));
        state.rollAcceleration = 60;
        assert(advance(cues, state, profile, 10).roll < -.005f);
        cues.reset(); state = sample(); state.rollAcceleration = bad;
        quiet(advance(cues, state, profile));
        state.supportG = 2;
        assert(advance(cues, state, profile, 10).pitch < -.005f);
    }
    for (auto channel : {&GroundSample::supportG, &GroundSample::rollAcceleration,
                         &GroundSample::pitchAcceleration}) {
        cues.reset(); state = sample(); quiet(advance(cues, state, profile));
        state.*channel = invalid;
        quiet(advance(cues, state, profile));
        state.*channel = channel == &GroundSample::supportG ? 4 : 120;
        quiet(advance(cues, state, profile)); // finite data returning at a new value is not a bump
    }

    cues.reset(); state = sample(); quiet(advance(cues, state, profile));
    state.supportG = 2;
    auto output = advance(cues, state, profile, 10);
    assert(output.pitch < -.005f);
    state.contactValid = false;
    const auto faded = cues.update(state, profile, dt);
    assert(std::abs(faded.pitch) < std::abs(output.pitch));
    quiet(advance(cues, state, profile, 200));
    state.contactValid = true; state.supportG = 4;
    quiet(advance(cues, state, profile));
    cues.reset(); state = sample(); quiet(advance(cues, state, profile));
    state.supportG = 2; output = advance(cues, state, profile, 10);
    assert(std::abs(cues.release(dt).pitch) < std::abs(output.pitch));
    for (int i = 0; i < 200; ++i) output = cues.release(dt);
    quiet(output);
    state.supportG = 4; quiet(advance(cues, state, profile)); // pause/resume reprimes changed telemetry

    for (float badTime : {invalid, infinity, 0.0f, -.01f}) {
        cues.reset(); state = sample(); quiet(advance(cues, state, profile));
        state.supportG = 2; advance(cues, state, profile, 10);
        quiet(cues.update(state, profile, badTime));
        quiet(cues.update(state, profile, dt));
    }
    cues.reset(); state = sample(); quiet(advance(cues, state, profile));
    state.supportG = 2; output = advance(cues, state, profile, 10);
    const auto delayed = cues.update(state, profile, .06f);
    assert(delayed.pitch < 0 && std::abs(delayed.pitch) < std::abs(output.pitch));
    state.supportG = 4;
    quiet(advance(cues, state, profile));

    for (unsigned axis : {0u, 1u, 2u}) {
        cues.reset(); state = sample(); state.positionValid = true;
        state.position = {1000000.001, 5000.001, -2000000.001};
        quiet(advance(cues, state, profile));
        state.position[axis] += 1000; state.supportG = 4;
        quiet(advance(cues, state, profile)); // reposition is rebaselined, not treated as a hard impact
    }
    cues.reset(); state = sample(); state.positionValid = true; state.verticalSpeed = invalid;
    quiet(advance(cues, state, profile));
    state.position[0] += 1000; state.supportG = 4;
    quiet(advance(cues, state, profile)); // missing vertical speed cannot disable the horizontal jump guard
    cues.reset(); state = sample(false, invalid); state.positionValid = true;
    state.supportG = 1;
    quiet(advance(cues, state, profile, 30));
    state.onGround = true;
    quiet(advance(cues, state, profile, 10)); // arm landing gain without generating a contact pulse
    state.position[1] += 1000; state.supportG = 4;
    quiet(advance(cues, state, profile)); // valid vertical speed detects reposition even when taxi speed is missing
    cues.reset(); state = sample(true, 250); state.positionValid = true;
    quiet(advance(cues, state, profile));
    state.position[0] += 26; state.supportG = 2;
    assert(advance(cues, state, profile, 10).pitch < -.005f); // speed-scaled position threshold
    cues.reset(); state = sample(); state.positionValid = true; state.verticalSpeed = 200;
    quiet(advance(cues, state, profile));
    state.position[1] += 28; state.supportG = 2;
    assert(advance(cues, state, profile, 10).pitch < -.005f); // vertical motion uses vertical, not taxi, speed

    for (auto gain : {&AircraftProfile::groundBumpGain, &AircraftProfile::landingBumpGain}) {
        for (float bad : {invalid, infinity, -.001f, maximumCueOffset + .001f}) {
            cues.reset(); state = sample(); quiet(advance(cues, state, profile));
            state.supportG = 2; advance(cues, state, profile, 10);
            auto badProfile = profile; badProfile.*gain = bad;
            quiet(cues.update(state, badProfile, dt));
        }
    }
    cues.reset(); state = sample(); quiet(advance(cues, state, profile));
    state.supportG = 1000; state.rollAcceleration = 60000;
    bounded(advance(cues, state, profile, 10));

    CueMixer mixer;
    auto mixed = mixer.update({.1f, -.1f}, {.1f, -.1f}, dt);
    assert(std::abs(mixed.roll - .03f) < 1e-6f && std::abs(mixed.pitch + .03f) < 1e-6f);
    for (int i = 0; i < 10; ++i) mixed = mixer.update({.1f, -.1f}, {.1f, -.1f}, dt);
    assert(mixed.roll == maximumCueOffset && mixed.pitch == -maximumCueOffset);
    for (int i = 0; i < 10; ++i) mixed = mixer.update({.1f, -.1f}, {-.1f, .1f}, dt);
    quiet(mixed); // opposing cues cancel before the shared cap
    mixer.reset(); quiet(mixer.update({}, {}, dt));
    for (float badTime : {invalid, infinity, 0.0f, -.01f}) {
        mixer.update({.1f, .1f}, {.1f, .1f}, dt);
        quiet(mixer.update({.1f, .1f}, {.1f, .1f}, badTime));
    }
    std::puts("Actual ground loads, taxi and landing envelopes, channel recovery, reposition and shared cue limits passed.");
}
