#include "g940Protocol.h"
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
    for (unsigned axis = 0; axis < 2; ++axis) {
        assert(force[1 + 30 * axis + 10] == 64);
        assert(force[1 + 30 * axis + 11] == 64);
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
    std::puts("Protocol and force bounds passed.");
}
