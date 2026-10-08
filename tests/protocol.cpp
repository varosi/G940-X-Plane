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
    assert(force[13] == 0 && force[14] == 0x40); // roll saturation
    assert(force[43] == 0xff && force[44] == 0x7f); // pitch saturation
    const auto halfSpeed = forceReport({0, 0, .5});
    assert(halfSpeed[13] == 0 && halfSpeed[14] == 0x20); // 50% roll saturation
    assert(halfSpeed[43] == 0xff && halfSpeed[44] == 0x5f); // 75% pitch saturation
    assert(springSaturationRatio(.9, 1) == 1); // strength never exceeds the device limit
    for (unsigned axis = 0; axis < 2; ++axis) {
        assert(force[1 + 30 * axis + 10] == (axis == 0 ? 64 : 96));
        assert(force[1 + 30 * axis + 11] == (axis == 0 ? 64 : 96));
        assert(force[1 + 30 * axis + 22] > 0);
        assert(force[1 + 30 * axis + 23] > 0);
    }
    const auto stop = stopReport();
    assert(stop[0] == 2);
    for (unsigned i = 1; i < stop.size(); ++i) assert(stop[i] == 0);
    const auto stationary = calculateForce(0, 0, 0, 100, 0, 0, 0);
    assert(stationary.speedRatio == 0);
    assert(calculateForce(0, 0, 100, 0, 0, 0, 0).speedRatio == 0);
    assert(calculateForce(0, 0, -100, 100, 0, 0, 0).speedRatio == 0);
    assert(calculateForce(0, 0, 1000, 100, 0, 0, 0).speedRatio == 1);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto invalid = forceReport({nan, nan, nan});
    assert(invalid[7] == 0 && invalid[37] == 0 && invalid[13] == 0);

    // Stick motion must not move the spring's trim target along with the
    // physical stick: doing so creates feedback around a moving neutral point.
    const auto trimTarget = calculateForce(0, 0, 25.722222, 100, 0, .5, .1);
    const auto displaced = calculateForce(.5, .5, 25.722222, 100, 0, .5, .1);
    assert(trimTarget.roll == displaced.roll && trimTarget.pitch == displaced.pitch);
    assert(trimTarget.rollForce > displaced.rollForce);
    assert(trimTarget.pitchForce > displaced.pitchForce);
    assert(displaced.pitchForce == 0); // held at the pitch trim target
    const auto untrimmed = calculateForce(0, .5, 25.722222, 100, 0, 0, 0);
    assert(untrimmed.pitchForce < displaced.pitchForce);

    ForceSmoother smoother;
    ForceState previous;
    for (int i = 0; i < 250; ++i) {
        const auto next = smoother.update(trimTarget, .02);
        assert(next.pitch >= previous.pitch);
        assert(next.pitch - previous.pitch <= .005001);
        assert(next.speedRatio - previous.speedRatio <= .020001);
        previous = next;
    }
    assert(std::abs(previous.pitch - trimTarget.pitch) < 1e-4);
    assert(std::abs(previous.speedRatio - trimTarget.speedRatio) < 1e-6);
    const auto reverseTrim = calculateForce(0, 0, 25.722222, 100, 0, -.5, -.1);
    const auto afterLongFrame = smoother.update(reverseTrim, 100);
    assert(previous.pitch - afterLongFrame.pitch <= .025001);
    const auto noAirspeed = smoother.update(stationary, .02);
    assert(noAirspeed.speedRatio == 0); // stop immediately, even during a ramp
    smoother.reset();
    const auto restarted = smoother.update(trimTarget, .02);
    assert(restarted.pitch <= .005001 && restarted.speedRatio <= .020001);

    // Small trim steps must be filtered as well as large changes: the old
    // slew limiter applied a small step in one frame.
    ForceSmoother smallTrim;
    const ForceState smallTarget(0, .004, .5);
    const auto firstSmallStep = smallTrim.update(smallTarget, .02);
    assert(firstSmallStep.pitch > 0 && firstSmallStep.pitch < .001);
    assert(firstSmallStep.pitch < smallTarget.pitch);
    auto settled = firstSmallStep;
    for (int i = 0; i < 100; ++i) settled = smallTrim.update(smallTarget, .02);
    assert(std::abs(settled.pitch - smallTarget.pitch) < 1e-5);
    assert(settled.pitch <= smallTarget.pitch); // no overshoot
    std::puts("Force packets, stable trim targets, and bounded transitions passed.");
}
