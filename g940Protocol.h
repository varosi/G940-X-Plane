#ifndef G940_PROTOCOL_H
#define G940_PROTOCOL_H

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace g940 {
enum LEDColour { OFF = 0, RED = 1, GREEN = 2, AMBER = RED | GREEN };
typedef std::array<LEDColour, 8> LEDState;
struct ForceState {
    double roll;
    double pitch;
    double speedRatio;
};

inline double clamp(double value, double low, double high) {
    return std::isfinite(value) ? std::max(low, std::min(value, high)) : 0.0;
}

inline ForceState calculateForce(double roll, double pitch, double speed,
                                double vneKnots, double alpha,
                                double elevatorTrim, double aileronTrim) {
    const double vne = vneKnots * 0.51444444;
    const double rollForce = (-roll + aileronTrim) * 3.0;
    const double pitchForce = (-pitch - alpha / 50.0 + elevatorTrim) * 1.5;
    ForceState result = {
        std::isfinite(rollForce) ? rollForce : 0.0,
        std::isfinite(pitchForce) ? pitchForce : 0.0,
        vne > 0.0 ? clamp(speed / vne, 0.0, 1.0) : 0.0
    };
    return result;
}

// G940 reports include their report ID. Feature 3 has eight red bits then
// eight green bits. Output 2 has two 30-byte axes and three reserved bytes.
// See tools/PROTOCOL.md for the byte layout.
inline std::array<uint8_t, 3> ledReport(const LEDState& state) {
    std::array<uint8_t, 3> report = {{3, 0, 0}};
    for (unsigned i = 0; i < state.size(); ++i) {
        if (state[i] & RED) report[1] |= 1u << i;
        if (state[i] & GREEN) report[2] |= 1u << i;
    }
    return report;
}

inline void put16(uint8_t *target, int value) {
    const uint16_t bits = static_cast<uint16_t>(value);
    target[0] = bits & 0xff;
    target[1] = bits >> 8;
}

inline std::array<uint8_t, 64> forceReport(const ForceState& state) {
    std::array<uint8_t, 64> report = {{2}};
    const double centers[] = {state.roll, state.pitch};
    const int maximums[] = {0x4000, 0x7fff};
    for (unsigned axis = 0; axis < 2; ++axis) {
        uint8_t *data = report.data() + 1 + 30 * axis;
        const int center = clamp(centers[axis], -1.0, 1.0) * 0x7fff;
        put16(data + 6, center);
        put16(data + 8, center);
        data[10] = data[11] = 0x40;
        put16(data + 12, clamp(state.speedRatio, 0.0, 1.0) * maximums[axis]);
    }
    return report;
}
inline std::array<uint8_t, 64> stopReport() {
    std::array<uint8_t, 64> report = {{2}};
    return report;
}
}
#endif
