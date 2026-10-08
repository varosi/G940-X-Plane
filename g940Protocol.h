#ifndef G940_PROTOCOL_H
#define G940_PROTOCOL_H

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace g940 {
enum LEDColour { OFF = 0, RED = 1, GREEN = 2, AMBER = RED | GREEN };
typedef std::array<LEDColour, 8> LEDState;
// The G940 uses signed 8-bit spring coefficients; evdev uses these values
// shifted left by eight bits. Keep the native and Linux stiffness consistent.
constexpr unsigned springCoefficients[2] = {96, 112};
constexpr unsigned springMaximums[2] = {0x6000, 0x7000};
constexpr double minimumForceRatio = 0.2;
struct ForceState {
    // Spring centers are independent of the measured stick position.
    double roll;
    double pitch;
    // Airspeed strength with a ground baseline; zero means invalid flight data.
    double speedRatio;
    // Devices without a native spring need the restoring force explicitly.
    double rollForce;
    double pitchForce;
    // Scale the whole effect during a pause fade, including stiffness.
    double effectScale;
    ForceState(double rollCenter = 0.0, double pitchCenter = 0.0, double ratio = 0.0,
               double rollDemand = 0.0, double pitchDemand = 0.0, double scale = 1.0)
        : roll(rollCenter), pitch(pitchCenter), speedRatio(ratio),
          rollForce(rollDemand), pitchForce(pitchDemand), effectScale(scale) {}
};

inline double clamp(double value, double low, double high) {
    return std::isfinite(value) ? std::max(low, std::min(value, high)) : 0.0;
}

inline double springGain(unsigned axis) {
    return springCoefficients[axis] / 64.0;
}

inline double springSaturationRatio(double speedRatio, unsigned axis) {
    return clamp(speedRatio * springGain(axis), 0.0, 1.0);
}

inline ForceState calculateForce(double roll, double pitch, double speed,
                                double vneKnots, double alpha,
                                double elevatorTrim, double aileronTrim) {
    const double vne = vneKnots * 0.51444444;
    // At taxi speeds the reported AoA can be meaningless. Ground resistance
    // is centered; blend aerodynamic trim targets in between 5 and 15 m/s.
    const double airflow = clamp((speed - 5.0) / 10.0, 0.0, 1.0);
    const double centerWeight = airflow * airflow * (3.0 - 2.0 * airflow);
    const double rollCenter = clamp(aileronTrim * 3.0 * centerWeight, -1.0, 1.0);
    const double pitchCenter = clamp((-alpha / 50.0 + elevatorTrim) * 1.5 * centerWeight, -1.0, 1.0);
    const double rollForce = rollCenter - clamp(roll, -1.0, 1.0) * 3.0;
    const double pitchForce = pitchCenter - clamp(pitch, -1.0, 1.0) * 1.5;
    const bool validSpeed = std::isfinite(speed) && speed >= 0.0 &&
        std::isfinite(vne) && vne > 0.0;
    ForceState result(rollCenter, pitchCenter,
        validSpeed ? std::max(minimumForceRatio, clamp(speed / vne, 0.0, 1.0)) : 0.0,
        rollForce, pitchForce);
    return result;
}

// Keep the Linux constant-force fallback's per-axis caps consistent with the
// native spring profile. The backend applies the pause scale after mixing.
inline std::array<double, 2> constantForceComponents(const ForceState& state) {
    std::array<double, 2> force = {{0.0, 0.0}};
    const double demands[] = {state.rollForce, state.pitchForce};
    const double ratio = clamp(state.speedRatio, 0.0, 1.0);
    for (unsigned axis = 0; axis < force.size(); ++axis) {
        const double cap = springSaturationRatio(ratio, axis) * springMaximums[axis] / 32767.0;
        const double demand = std::isfinite(demands[axis]) ? demands[axis] : 0.0;
        force[axis] = clamp(demand * springGain(axis) * ratio, -cap, cap);
    }
    return force;
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
    const double scale = clamp(state.effectScale, 0.0, 1.0);
    if (scale == 0.0) return report;
    const double centers[] = {state.roll, state.pitch};
    for (unsigned axis = 0; axis < 2; ++axis) {
        uint8_t *data = report.data() + 1 + 30 * axis;
        const int center = clamp(centers[axis], -1.0, 1.0) * 0x7fff;
        put16(data + 6, center);
        put16(data + 8, center);
        data[10] = data[11] = springCoefficients[axis] * scale;
        put16(data + 12, springSaturationRatio(state.speedRatio, axis) * springMaximums[axis] * scale);
        // The firmware's velocity channel opposes motion for positive
        // coefficients. Light damping helps settle near the trimmed center.
        data[22] = data[23] = 8 * scale;
        put16(data + 24, clamp(state.speedRatio, 0.0, 1.0) * 4096 * scale);
    }
    return report;
}
inline std::array<uint8_t, 64> stopReport() {
    std::array<uint8_t, 64> report = {{2}};
    return report;
}
}
#endif
