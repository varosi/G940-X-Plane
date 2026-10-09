#ifndef G940_FORCE_MODEL_H
#define G940_FORCE_MODEL_H

#include "g940Protocol.h"

namespace g940 {
constexpr float pascalsPerPsf = 47.88026f;
constexpr float knotsToMps = 0.51444444f;
constexpr float seaLevelDensity = 1.225f;

struct AircraftProfile {
    // Initial TB10/TB20 tuning, not measured hinge moments or grip forces.
    // Full spring stiffness at 125 knots equivalent airspeed.
    float referencePressurePa = 0.5f * seaLevelDensity *
        (125.0f * knotsToMps) * (125.0f * knotsToMps);
    float mechanicalRatio = minimumForceRatio;
    float rollTrimGain = 3.0f;
    float pitchTrimGain = 1.5f;
    float pitchAoAGain = 0.03f;
    float neutralAoADegrees = 0.0f;
};
inline constexpr AircraftProfile tb10tb20Profile{};

inline ForceState calculateForce(float roll, float pitch, float pressurePa,
                                float alpha, float elevatorTrim, float aileronTrim,
                                const AircraftProfile& profile = tb10tb20Profile) {
    if (!std::isfinite(roll) || !std::isfinite(pitch) ||
        !std::isfinite(pressurePa) || pressurePa < 0.0f ||
        !std::isfinite(alpha) || !std::isfinite(elevatorTrim) || !std::isfinite(aileronTrim) ||
        !std::isfinite(profile.referencePressurePa) || profile.referencePressurePa <= 0.0f ||
        !std::isfinite(profile.mechanicalRatio) || profile.mechanicalRatio <= 0.0f ||
        profile.mechanicalRatio > 1.0f || !std::isfinite(profile.rollTrimGain) ||
        !std::isfinite(profile.pitchTrimGain) || !std::isfinite(profile.pitchAoAGain) ||
        !std::isfinite(profile.neutralAoADegrees)) return {};
    const float aerodynamic = (1.0f - profile.mechanicalRatio) *
        (std::min(pressurePa, profile.referencePressurePa) / profile.referencePressurePa);
    const float stiffness = profile.mechanicalRatio + aerodynamic;
    // Balance an aerodynamic spring about its trim neutral against a small
    // mechanical spring about zero. The equilibrium is independent of stick
    // movement and blends to a centered, gentle mechanical load at zero q.
    const float aerodynamicWeight = aerodynamic / stiffness;
    const float rollNeutral = profile.rollTrimGain * clamp(aileronTrim, -1.0f, 1.0f);
    const float pitchNeutral = profile.pitchTrimGain * clamp(elevatorTrim, -1.0f, 1.0f) -
        profile.pitchAoAGain * (alpha - profile.neutralAoADegrees);
    if (!std::isfinite(rollNeutral) || !std::isfinite(pitchNeutral)) return {};
    const float rollCenter = clamp(rollNeutral * aerodynamicWeight, -1.0f, 1.0f);
    const float pitchCenter = clamp(pitchNeutral * aerodynamicWeight, -1.0f, 1.0f);
    // Native springs and the constant-force fallback must share the same
    // zero-force position; do not apply another stick gain after this center.
    return {rollCenter, pitchCenter, stiffness,
        rollCenter - clamp(roll, -1.0f, 1.0f), pitchCenter - clamp(pitch, -1.0f, 1.0f)};
}

class ForceSmoother {
public:
    void reset() {
        state_ = ForceState();
        releasing_ = false;
        releaseElapsed_ = 0.0f;
    }
    void reset(const ForceState& target) {
        reset();
        // Establish the current trim position while force is zero, rather
        // than pulling toward an artificial zero center after connecting.
        state_ = target;
        state_.pressureRatio = 0.0f;
        state_.effectScale = 0.0f;
    }
    bool releasing() const { return releasing_; }
    ForceState update(const ForceState& target, float elapsed) {
        // A long simulator frame must not turn a trim change into one jump.
        const float dt = clamp(elapsed, 0.0f, 0.1f);
        releasing_ = false;
        // Resuming partway through a release must not jump to full strength.
        state_.effectScale = approach(state_.effectScale, 1.0f, dt);
        state_.roll = approach(state_.roll, clamp(target.roll, -1.0f, 1.0f), 0.5f * dt);
        // Filter small trim/AoA changes too, and taper into the new center.
        // The lower slew rate also offsets the stronger pitch spring.
        const float pitchTarget = clamp(target.pitch, -1.0f, 1.0f);
        const float filteredPitch = state_.pitch +
            (pitchTarget - state_.pitch) * (-std::expm1(-dt / 0.25f));
        state_.pitch = approach(state_.pitch, filteredPitch, 0.25f * dt);
        const float ratio = clamp(target.pressureRatio, 0.0f, 1.0f);
        state_.pressureRatio = ratio == 0.0f ? 0.0f : approach(state_.pressureRatio, ratio, dt);
        // Keep immediate stick-dependent restoring force on the Linux
        // constant-force fallback, using the same smoothed trim centers.
        state_.rollForce = target.rollForce + state_.roll - target.roll;
        state_.pitchForce = target.pitchForce + state_.pitch - target.pitch;
        return state_;
    }
    ForceState release(float elapsed) {
        if (!releasing_) {
            releasing_ = true;
            releaseElapsed_ = 0.0f;
            releaseStartScale_ = state_.effectScale;
        }
        // Honor the one-second release even after a delayed callback.
        releaseElapsed_ = clamp(releaseElapsed_ + clamp(elapsed, 0.0f, 1.0f), 0.0f, 1.0f);
        if (releaseElapsed_ > 1.0f - 1e-6f) releaseElapsed_ = 1.0f;
        const float t = releaseElapsed_;
        state_.effectScale = releaseStartScale_ * (1.0f - t * t * (3.0f - 2.0f * t));
        return state_;
    }
private:
    static float approach(float current, float target, float step) {
        return current + clamp(target - current, -step, step);
    }
    ForceState state_;
    bool releasing_ = false;
    float releaseElapsed_ = 0.0f;
    float releaseStartScale_ = 1.0f;
};
}
#endif
