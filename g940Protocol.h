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
constexpr unsigned springCoefficients[2] = {80, 112};
constexpr unsigned springMaximums[2] = {0x5000, 0x7000};
constexpr float minimumForceRatio = 0.2f;
constexpr unsigned dampingCoefficient = 8;
constexpr unsigned dampingMaximum = 4096;
inline float clamp(float value, float low, float high) {
    return std::isfinite(value) ? std::max(low, std::min(value, high)) : 0.0f;
}
struct ForceState {
    // Spring centers are independent of the measured stick position.
    float roll = 0.0f;
    float pitch = 0.0f;
    // Keep the pressure-independent spring separate from aerodynamic load.
    float mechanicalRatio = 0.0f;
    // Devices without a native spring need the restoring force explicitly.
    float rollForce = 0.0f;
    float pitchForce = 0.0f;
    // Scale the whole effect during a pause fade, including stiffness.
    float effectScale = 1.0f;
    float aerodynamicRatio = 0.0f;
    float dampingRatio = 0.0f;
    // Used only for software damping when the driver has no damper effect.
    float rollVelocity = 0.0f, pitchVelocity = 0.0f;
    float springRatio() const { return clamp(mechanicalRatio + aerodynamicRatio, 0.0f, 1.0f); }
    bool hasLoad() const {
        return std::isfinite(mechanicalRatio) && mechanicalRatio >= 0.0f &&
            std::isfinite(aerodynamicRatio) && aerodynamicRatio >= 0.0f &&
            std::isfinite(dampingRatio) && dampingRatio >= 0.0f &&
            (springRatio() > 0.0f || dampingRatio > 0.0f);
    }
};

inline unsigned springCoefficient(float strength, unsigned axis) {
    return static_cast<unsigned>(std::lround(springCoefficients[axis] *
        clamp(strength, 0.0f, 1.0f)));
}

inline float springSaturationRatio(float strength, unsigned axis) {
    return clamp(strength * springCoefficients[axis] / 64.0f, 0.0f, 1.0f);
}
inline unsigned velocityCoefficient(float strength) {
    return static_cast<unsigned>(std::lround(dampingCoefficient * clamp(strength, 0.0f, 1.0f)));
}
inline unsigned quantizedCap(float cap) {
    return std::min(127u, static_cast<unsigned>(std::lround(clamp(cap, 0.0f, 32512.0f) / 256.0f))) * 256;
}
inline unsigned springCap(const ForceState& state, unsigned axis, float scale = 1.0f) {
    return quantizedCap(springSaturationRatio(state.springRatio(), axis) * springMaximums[axis] * scale);
}
inline unsigned dampingCap(const ForceState& state, unsigned axis, float scale = 1.0f) {
    const unsigned cap = quantizedCap(clamp(state.dampingRatio, 0.0f, 1.0f) * dampingMaximum * scale);
    // Idle firmware shares the spring cap with damping. Do not increase its
    // spring saturation just to accommodate a stronger damper. A damper-only
    // effect can use that shared cap without introducing a spring.
    return state.springRatio() > 0.0f ? std::min(cap, springCap(state, axis, scale)) : cap;
}

// Keep the Linux constant-force fallback's per-axis caps consistent with the
// native spring profile. The backend applies the pause scale after mixing.
inline std::array<float, 2> dampingForceComponents(const ForceState& state) {
    if (!state.hasLoad()) return {{0, 0}};
    const float ratio = clamp(state.dampingRatio, 0.0f, 1.0f);
    const float rollCap = dampingCap(state, 0) / 32767.0f, pitchCap = dampingCap(state, 1) / 32767.0f;
    return {{clamp(-state.rollVelocity * velocityCoefficient(ratio) / 64.0f, -rollCap, rollCap),
             clamp(-state.pitchVelocity * velocityCoefficient(ratio) / 64.0f, -pitchCap, pitchCap)}};
}
inline std::array<float, 2> constantForceComponents(const ForceState& state) {
    std::array<float, 2> force = {{0.0f, 0.0f}};
    const float demands[] = {state.rollForce, state.pitchForce};
    if (!state.hasLoad()) return force;
    const float ratio = state.springRatio();
    const auto damping = dampingForceComponents(state);
    for (unsigned axis = 0; axis < force.size(); ++axis) {
        const float cap = springCap(state, axis) / 32767.0f;
        const float demand = std::isfinite(demands[axis]) ? demands[axis] : 0.0f;
        force[axis] = clamp(clamp(demand * springCoefficient(ratio, axis) / 64.0f, -cap, cap) + damping[axis], -1.0f, 1.0f);
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
    const float scale = clamp(state.effectScale, 0.0f, 1.0f);
    if (scale == 0.0f || !state.hasLoad()) return report;
    const float centers[] = {state.roll, state.pitch};
    for (unsigned axis = 0; axis < 2; ++axis) {
        uint8_t *data = report.data() + 1 + 30 * axis;
        const int center = clamp(centers[axis], -1.0f, 1.0f) * 0x7fff;
        put16(data + 6, center);
        put16(data + 8, center);
        data[10] = data[11] = springCoefficient(state.springRatio() * scale, axis);
        // The firmware's hands-off channel has an 8-bit cap in units of 256.
        // Use that same cap in both modes so releasing the grip keeps the load.
        put16(data + 12, springCap(state, axis, scale));
        // The firmware's velocity channel opposes motion for positive
        // coefficients. Light damping helps settle near the trimmed center.
        const float damping = clamp(state.dampingRatio, 0.0f, 1.0f) * scale;
        data[22] = data[23] = velocityCoefficient(damping);
        put16(data + 24, dampingCap(state, axis, scale));
    }
    return report;
}
inline std::array<uint8_t, 4> idleForceReport(const std::array<uint8_t, 64>& force, unsigned axis) {
    const uint8_t *data = force.data() + 1 + 30 * axis;
    return {{static_cast<uint8_t>(5 + axis), data[10],
        data[10] ? data[13] : data[25], data[22]}};
}
inline std::array<uint8_t, 5> idleCenterReport(const std::array<uint8_t, 64>& force) {
    return {{10, force[7], force[8], force[37], force[38]}};
}
inline std::array<uint8_t, 64> stopReport() {
    std::array<uint8_t, 64> report = {{2}};
    return report;
}
}
#endif
