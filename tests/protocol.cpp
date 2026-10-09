#include "g940Protocol.h"
#include "g940ForceModel.h"
#include <cassert>
#include <cstdio>
#include <limits>

int main() {
    using namespace g940;
    const LEDState colours = {{RED, GREEN, AMBER, OFF, RED, GREEN, AMBER, OFF}};
    const auto leds = ledReport(colours);
    assert((leds == std::array<uint8_t, 3>{{3, 0x55, 0x66}}));
    const auto force = forceReport({-1.0, 1.0, 1.0});
    assert(force[0] == 2 && force.size() == 64);
    assert(force[7] == 0x01 && force[8] == 0x80); // signed roll center
    assert(force[9] == 0x01 && force[10] == 0x80);
    assert(force[37] == 0xff && force[38] == 0x7f); // pitch center
    assert(force[13] == 0 && force[14] == 0x50); // intermediate roll saturation
    assert(force[43] == 0 && force[44] == 0x70); // reduced pitch saturation
    const auto halfPressure = forceReport({0, 0, .5});
    assert(halfPressure[11] == 40 && halfPressure[41] == 56); // stiffness varies too
    assert(halfPressure[13] == 0 && halfPressure[14] == 0x32); // 62.5% of roll cap
    assert(halfPressure[43] == 0 && halfPressure[44] == 0x62); // 87.5% of pitch cap
    assert(springSaturationRatio(.9, 1) == 1); // strength never exceeds the device limit
    for (unsigned axis = 0; axis < 2; ++axis) {
        assert(force[1 + 30 * axis + 10] == (axis == 0 ? 80 : 112));
        assert(force[1 + 30 * axis + 11] == (axis == 0 ? 80 : 112));
        assert(force[1 + 30 * axis + 22] > 0);
        assert(force[1 + 30 * axis + 23] > 0);
    }
    const auto stop = stopReport();
    assert(stop[0] == 2);
    for (unsigned i = 1; i < stop.size(); ++i) assert(stop[i] == 0);
    assert(forceReport({.5, -.5, 1, 0, 0, 0}) == stop);
    const auto halfEffect = forceReport({0, 0, .5, 0, 0, .5});
    assert(halfEffect[11] == 20 && halfEffect[41] == 28); // stiffness fades too
    assert(halfEffect[43] == 0 && halfEffect[44] == 0x31);
    const float referencePressure = defaultProfile.referencePressurePa;
    assert(referencePressure > 2530 && referencePressure < 2540); // 125 KEAS, Pa
    const auto stationary = calculateForce(.2, -.2, 0, -121, .2, .1);
    assert(stationary.pressureRatio == minimumForceRatio);
    assert(stationary.roll == 0 && stationary.pitch == 0); // ground AoA/trim cannot pull
    assert(stationary.rollForce < 0 && stationary.pitchForce > 0);
    const auto groundReport = forceReport(stationary);
    assert(groundReport[11] == 16 && groundReport[41] == 22);
    assert(groundReport[14] != 0 && groundReport[44] != 0);
    const auto taxi = calculateForce(0, 0, 1, -121, .2, .1);
    assert(taxi.pressureRatio > minimumForceRatio && taxi.pitch < .01);
    const auto blended = calculateForce(0, 0, referencePressure / 2, 5, .2, .1);
    assert(std::abs(blended.pressureRatio - .6) < 1e-6);
    assert(std::abs(blended.roll - .06666667) < 1e-6);
    assert(std::abs(blended.pitch - .03333333) < 1e-6);

    // q = rho*V^2/2: doubling airflow quadruples aerodynamic stiffness;
    // halving density halves it. The mechanical component remains at zero q.
    const float slowSpeed = 62.5f * knotsToMps;
    const float slowQ = .5f * seaLevelDensity * slowSpeed * slowSpeed;
    const auto slow = calculateForce(0, 0, slowQ, 0, 0, 0);
    const auto fast = calculateForce(0, 0, 4 * slowQ, 0, 0, 0);
    const auto thinAir = calculateForce(0, 0, 2 * slowQ, 0, 0, 0);
    assert(std::abs(slow.pressureRatio - .4) < 1e-6);
    assert(std::abs(fast.pressureRatio - 1) < 1e-6);
    assert(std::abs(thinAir.pressureRatio - .6) < 1e-6);
    const auto slowReport = forceReport(slow), fastReport = forceReport(fast);
    assert(slowReport[11] < fastReport[11] && slowReport[41] < fastReport[41]);
    assert(slowReport[14] < fastReport[14] && slowReport[44] < fastReport[44]);
    assert(calculateForce(0, 0, 100 * referencePressure, 0, 0, 0).pressureRatio == 1);
    auto profile = defaultProfile;
    profile.referencePressurePa *= 2;
    assert(std::abs(calculateForce(0, 0, referencePressure, 0, 0, 0, profile).pressureRatio - .6) < 1e-6);
    profile.referencePressurePa = 0;
    assert(calculateForce(0, 0, 100, 0, 0, 0, profile).pressureRatio == 0);
    profile = defaultProfile;
    profile.mechanicalRatio = -1;
    assert(calculateForce(0, 0, 100, 0, 0, 0, profile).pressureRatio == 0);
    profile.mechanicalRatio = 2;
    assert(calculateForce(0, 0, 100, 0, 0, 0, profile).pressureRatio == 0);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    assert(calculateForce(0, 0, -1, 0, 0, 0).pressureRatio == 0);
    assert(calculateForce(0, 0, nan, 0, 0, 0).pressureRatio == 0);
    assert(calculateForce(0, 0, infinity, 0, 0, 0).pressureRatio == 0);
    assert(calculateForce(0, 0, 100, infinity, 0, 0).pressureRatio == 0);
    assert(calculateForce(nan, 0, 100, 0, 0, 0).pressureRatio == 0);
    assert(calculateForce(0, 0, 100, 0, infinity, 0).pressureRatio == 0);
    profile = defaultProfile;
    profile.pitchTrimGain = nan;
    assert(calculateForce(0, 0, 100, 0, 0, 0, profile).pressureRatio == 0);
    profile = defaultProfile;
    profile.referencePressurePa = .00001f;
    assert(calculateForce(0, 0, std::numeric_limits<float>::max(), 0, 0, 0, profile).pressureRatio == 1);
    const auto invalid = forceReport({nan, nan, nan});
    assert(invalid[7] == 0 && invalid[37] == 0 && invalid[13] == 0);
    assert(forceReport({0, 0, 1, 0, 0, nan}) == stop);

    const auto cappedConstant = constantForceComponents({0, 0, 1, 10, -10});
    assert(std::abs(cappedConstant[0] - 20480.0 / 32767) < 1e-6);
    assert(std::abs(cappedConstant[1] + 28672.0 / 32767) < 1e-6);
    const auto smallConstant = constantForceComponents({0, 0, .5, .1, .1});
    assert(std::abs(smallConstant[0] - .0625) < 1e-6);
    assert(std::abs(smallConstant[1] - .0875) < 1e-6);
    const auto invalidConstant = constantForceComponents({0, 0, nan, infinity, nan});
    assert(invalidConstant[0] == 0 && invalidConstant[1] == 0);

    assert((idleCenterReport(force) == std::array<uint8_t, 5>{{10, 1, 128, 255, 127}}));
    assert((idleForceReport(force, 0) == std::array<uint8_t, 4>{{5, 80, 80, 8}}));
    assert((idleForceReport(force, 1) == std::array<uint8_t, 4>{{6, 112, 112, 8}}));
    for (int i = 0; i <= 100; ++i) {
        const auto sample = forceReport({.1, -.3, i / 100.0f, 0, 0, .6});
        for (unsigned axis = 0; axis < 2; ++axis) {
            const auto idle = idleForceReport(sample, axis);
            const unsigned start = 1 + 30 * axis;
            assert(sample[start + 12] == 0); // same /256 cap in both grip modes
            assert(idle[1] == sample[start + 10] && idle[2] == sample[start + 13]);
            assert(idle[3] == sample[start + 22]);
            assert(idle[2] < 128); // positive signed firmware saturation
        }
    }

    // Stick motion must not move the spring's trim target along with the
    // physical stick: doing so creates feedback around a moving neutral point.
    const auto trimTarget = calculateForce(0, 0, referencePressure / 2, 0, .5, .1);
    const auto displaced = calculateForce(.5, trimTarget.pitch, referencePressure / 2, 0, .5, .1);
    assert(trimTarget.roll == displaced.roll && trimTarget.pitch == displaced.pitch);
    assert(trimTarget.rollForce > displaced.rollForce);
    assert(trimTarget.pitchForce > displaced.pitchForce);
    assert(std::abs(displaced.pitchForce) < 1e-6); // held at the pitch trim target
    const auto balanced = calculateForce(trimTarget.roll, trimTarget.pitch,
        referencePressure / 2, 0, .5, .1);
    const auto balancedConstant = constantForceComponents(balanced);
    assert(balancedConstant[0] == 0 && balancedConstant[1] == 0);
    const auto aboveCenter = calculateForce(trimTarget.roll + .05f, trimTarget.pitch + .05f,
        referencePressure / 2, 0, .5, .1);
    const auto belowCenter = calculateForce(trimTarget.roll - .05f, trimTarget.pitch - .05f,
        referencePressure / 2, 0, .5, .1);
    assert(aboveCenter.rollForce < 0 && aboveCenter.pitchForce < 0);
    assert(belowCenter.rollForce > 0 && belowCenter.pitchForce > 0);
    const auto untrimmed = calculateForce(0, .5, referencePressure / 2, 0, 0, 0);
    assert(untrimmed.pitchForce < displaced.pitchForce);

    ForceSmoother smoother;
    ForceState previous;
    for (int i = 0; i < 250; ++i) {
        const auto next = smoother.update(trimTarget, .02);
        assert(next.pitch >= previous.pitch);
        assert(next.pitch - previous.pitch <= .005001);
        assert(next.pressureRatio - previous.pressureRatio <= .020001);
        previous = next;
    }
    assert(std::abs(previous.pitch - trimTarget.pitch) < 1e-4);
    assert(std::abs(previous.pressureRatio - trimTarget.pressureRatio) < 1e-6);
    const auto reverseTrim = calculateForce(0, 0, referencePressure / 2, 0, -.5, -.1);
    const auto afterLongFrame = smoother.update(reverseTrim, 100);
    assert(previous.pitch - afterLongFrame.pitch <= .025001);
    const auto invalidPressure = calculateForce(0, 0, -1, 0, 0, 0);
    const auto invalidState = smoother.update(invalidPressure, .02);
    assert(invalidState.pressureRatio == 0); // invalid data stops immediately, even during a ramp
    smoother.reset();
    const auto restarted = smoother.update(trimTarget, .02);
    assert(restarted.pitch <= .005001 && restarted.pressureRatio <= .020001);
    smoother.reset(trimTarget);
    const auto primed = smoother.update(trimTarget, .02f);
    assert(primed.pitch == trimTarget.pitch && primed.roll == trimTarget.roll);
    assert(primed.pressureRatio <= .020001f && primed.effectScale <= .020001f);
    assert(std::abs(primed.pitchForce - trimTarget.pitchForce) < 1e-6f);

    // Small trim steps must be filtered as well as large changes: the old
    // slew limiter applied a small step in one frame.
    ForceSmoother smallTrim;
    const ForceState smallTarget{0.0f, .004f, .5f};
    const auto firstSmallStep = smallTrim.update(smallTarget, .02);
    assert(firstSmallStep.pitch > 0 && firstSmallStep.pitch < .001);
    assert(firstSmallStep.pitch < smallTarget.pitch);
    auto settled = firstSmallStep;
    for (int i = 0; i < 100; ++i) settled = smallTrim.update(smallTarget, .02);
    assert(std::abs(settled.pitch - smallTarget.pitch) < 1e-5);
    assert(settled.pitch <= smallTarget.pitch); // no overshoot

    float lastScale = 1.0f;
    for (int i = 0; i < 25; ++i) {
        settled = smallTrim.release(.02);
        assert(settled.effectScale <= lastScale && settled.effectScale >= 0);
        assert(lastScale - settled.effectScale <= .031);
        lastScale = settled.effectScale;
    }
    assert(std::abs(settled.effectScale - .5) < 1e-6);
    const auto resumed = smallTrim.update(smallTarget, .02);
    assert(resumed.effectScale > settled.effectScale);
    assert(resumed.effectScale - settled.effectScale <= .020001);
    assert(!smallTrim.releasing());
    const auto stopped = smallTrim.release(5); // expired pause after a long frame
    assert(stopped.effectScale == 0 && forceReport(stopped) == stop);
    smallTrim.reset();
    assert(!smallTrim.releasing());
    std::puts("Force packets, pressure-dependent stiffness, trim equilibrium and transitions passed.");
}
