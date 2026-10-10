#ifndef G940_FORCE_MODEL_H
#define G940_FORCE_MODEL_H

#include "g940Protocol.h"

namespace g940 {
constexpr float pascalsPerPsf = 47.88026f;
constexpr float knotsToMps = 0.51444444f;
constexpr float seaLevelDensity = 1.225f;
constexpr float fallbackElevatorDegrees = 15.0f;
enum class PitchTrimMode { aerodynamic, spring, stabilizer };

inline float pitchDeflection(float ratio, float up, float down) {
    // Positive pitch means elevator up, unlike X-Plane's trailing-edge-down
    // surface angles. Combine loads in degrees before choosing either travel.
    return ratio * (ratio >= 0.0f ? up : down);
}
inline float pressureLoad(float pressure, float reference, float gain) {
    if (pressure == 0.0f || gain == 0.0f) return 0.0f;
    if (pressure >= reference / gain) return 1.0f;
    // The result is below one. Choose an order that avoids overflow and
    // premature underflow without needing double-precision calculations.
    return pressure >= 1.0f ? (gain * pressure) / reference :
        (std::min(gain, pressure) / reference) * std::max(gain, pressure);
}

struct AircraftProfile {
    // Generic fallback tuning when no configuration/simulator reference is available.
    float referencePressurePa = 0.5f * seaLevelDensity *
        (125.0f * knotsToMps) * (125.0f * knotsToMps);
    float mechanicalRatio = minimumForceRatio;
    float aerodynamicGain = 0.8f;
    float mechanicalDamping = 0.2f;
    float aerodynamicDamping = 0.8f;
    // Optional tactile cues; disabled until an aircraft preset enables them.
    float turbulenceGain = 0.0f; // normalized center displacement per m/s gust
    float stallBuffetGain = 0.0f; // peak normalized pitch-center displacement
    float stallBuffetIdleRatio = 0.25f;
    float stallBuffetHz = 5.0f;
    float groundBumpGain = 0.0f; // normalized center displacement per transient support g
    float landingBumpGain = 0.0f;
    float rollTrimGain = 1.0f;
    float pitchTrimGain = 1.0f;
    float pitchAoADeflectionGain = 0.45f; // degrees of elevator / degree of AoA
    float neutralAoADegrees = 0.0f;
    float elevatorUpDegrees = fallbackElevatorDegrees;
    float elevatorDownDegrees = fallbackElevatorDegrees;
    float staticPitchTrim = 0.0f;
    PitchTrimMode pitchTrimMode = PitchTrimMode::aerodynamic;
};
inline constexpr AircraftProfile defaultProfile{};

inline ForceState calculateForce(float roll, float pitch, float pressurePa,
                                float alpha, float elevatorTrim, float aileronTrim,
                                const AircraftProfile& profile = defaultProfile,
                                float stabilizerDegrees = 0.0f) {
    if (!std::isfinite(roll) || !std::isfinite(pitch) ||
        !std::isfinite(pressurePa) || pressurePa < 0.0f ||
        !std::isfinite(alpha) || !std::isfinite(elevatorTrim) || !std::isfinite(aileronTrim) ||
        !std::isfinite(profile.referencePressurePa) || profile.referencePressurePa <= 0.0f ||
        !std::isfinite(profile.mechanicalRatio) || profile.mechanicalRatio < 0.0f ||
        profile.mechanicalRatio > 1.0f || !std::isfinite(profile.aerodynamicGain) || profile.aerodynamicGain < 0.0f ||
        profile.aerodynamicGain > 4.0f || !std::isfinite(profile.mechanicalDamping) || profile.mechanicalDamping < 0.0f ||
        profile.mechanicalDamping > 1.0f || !std::isfinite(profile.aerodynamicDamping) || profile.aerodynamicDamping < 0.0f ||
        profile.aerodynamicDamping > 4.0f || !std::isfinite(profile.rollTrimGain) ||
        !std::isfinite(profile.pitchTrimGain) || !std::isfinite(profile.pitchAoADeflectionGain) ||
        !std::isfinite(profile.neutralAoADegrees) ||
        !std::isfinite(profile.elevatorUpDegrees) || profile.elevatorUpDegrees <= 0.0f ||
        !std::isfinite(profile.elevatorDownDegrees) || profile.elevatorDownDegrees <= 0.0f ||
        !std::isfinite(profile.staticPitchTrim) || std::abs(profile.staticPitchTrim) > 1.0f ||
        !std::isfinite(stabilizerDegrees)) return {};
    const float m = profile.mechanicalRatio;
    const float aerodynamic = pressureLoad(pressurePa, profile.referencePressurePa, profile.aerodynamicGain);
    const float damping = clamp(profile.mechanicalDamping +
        pressureLoad(pressurePa, profile.referencePressurePa, profile.aerodynamicDamping), 0.0f, 1.0f);
    // Balance aerodynamic loading against mechanical resistance. Aerodynamic
    // trim fades to a centered ground load; spring trim retains its preload.
    // Neither equilibrium depends on the current stick position.
    // Hardware strength is capped, but the physical force balance is not.
    // Scale both pressure terms before balancing, avoiding overflow for large
    // q, tiny gains or reference pressure while retaining the uncapped ratio.
    const float pressureScale = std::max(pressurePa, profile.referencePressurePa);
    const float mechanicalBalance = m * (profile.referencePressurePa / pressureScale);
    const float aerodynamicBalance = profile.aerodynamicGain * (pressurePa / pressureScale);
    const float aerodynamicWeight = pressurePa == 0.0f || profile.aerodynamicGain == 0.0f ? 0.0f : m == 0.0f ? 1.0f :
        aerodynamicBalance / (mechanicalBalance + aerodynamicBalance);
    const float rollNeutral = profile.rollTrimGain * clamp(aileronTrim, -1.0f, 1.0f);
    const float up = profile.elevatorUpDegrees, down = profile.elevatorDownDegrees;
    const float tabDegrees = pitchDeflection(profile.staticPitchTrim, up, down);
    const float trimDegrees = profile.pitchTrimGain *
        pitchDeflection(clamp(elevatorTrim, -1.0f, 1.0f), up, down);
    // THS trim changes incidence, not the elevator's mechanical center.
    // Do not also add elevatorTrim for that system. This remains a linear
    // hinge-load approximation, not a measured hinge moment or jet feel law.
    const float incidence = profile.pitchTrimMode == PitchTrimMode::stabilizer ? stabilizerDegrees : 0.0f;
    const float airflowDegrees = -profile.pitchAoADeflectionGain *
        (alpha + incidence - profile.neutralAoADegrees);
    const float pitchDegrees = aerodynamicWeight * (tabDegrees + airflowDegrees) +
        (profile.pitchTrimMode == PitchTrimMode::spring ? (1.0f - aerodynamicWeight) * trimDegrees :
         profile.pitchTrimMode == PitchTrimMode::aerodynamic ? aerodynamicWeight * trimDegrees : 0.0f);
    const float pitchNeutral = pitchDegrees / (pitchDegrees >= 0.0f ? up : down);
    if (!std::isfinite(rollNeutral) || !std::isfinite(pitchNeutral)) return {};
    const float rollCenter = clamp(rollNeutral * aerodynamicWeight, -1.0f, 1.0f);
    const float pitchCenter = clamp(pitchNeutral, -1.0f, 1.0f);
    // Native springs and the constant-force fallback must share the same
    // zero-force position; do not apply another stick gain after this center.
    return {rollCenter, pitchCenter, m,
        rollCenter - clamp(roll, -1.0f, 1.0f), pitchCenter - clamp(pitch, -1.0f, 1.0f),
        1.0f, aerodynamic, damping};
}

class ForceSmoother {
public:
    void reset() {
        state_ = ForceState();
        releasing_ = false;
        releaseElapsed_ = 0.0f;
        haveStick_ = false;
    }
    void reset(const ForceState& target) {
        reset();
        // Establish the current trim position while force is zero, rather
        // than pulling toward an artificial zero center after connecting.
        state_ = target;
        state_.mechanicalRatio = state_.aerodynamicRatio = state_.dampingRatio = 0.0f;
        state_.effectScale = 0.0f;
        lastStick_ = {{target.roll - target.rollForce, target.pitch - target.pitchForce}};
        haveStick_ = true;
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
        if (!target.hasLoad()) { reset(); state_.effectScale = 0.0f; return state_; }
        // Share the original 1/s stiffness slew budget between the two terms.
        const float mechanicalChange = target.mechanicalRatio - state_.mechanicalRatio;
        const float aerodynamicChange = target.aerodynamicRatio - state_.aerodynamicRatio;
        const float distance = std::abs(mechanicalChange) + std::abs(aerodynamicChange);
        const float fraction = distance > 0.0f ? std::min(dt / distance, 1.0f) : 1.0f;
        state_.mechanicalRatio += mechanicalChange * fraction;
        state_.aerodynamicRatio += aerodynamicChange * fraction;
        state_.dampingRatio = approach(state_.dampingRatio, target.dampingRatio, dt);
        // Keep immediate stick-dependent restoring force on the Linux
        // constant-force fallback, using the same smoothed trim centers.
        state_.rollForce = target.rollForce + state_.roll - target.roll;
        state_.pitchForce = target.pitchForce + state_.pitch - target.pitch;
        const std::array<float, 2> stick = {{target.roll - target.rollForce, target.pitch - target.pitchForce}};
        const float velocityDt = clamp(elapsed, 0.0f, 1.0f);
        float *velocities[] = {&state_.rollVelocity, &state_.pitchVelocity};
        for (unsigned axis = 0; axis < stick.size(); ++axis) {
            const float velocity = haveStick_ && velocityDt > 0.0f ?
                clamp((stick[axis] - lastStick_[axis]) / elapsed, -4.0f, 4.0f) : 0.0f;
            *velocities[axis] += (velocity - *velocities[axis]) * (-std::expm1(-velocityDt / .05f));
        }
        lastStick_ = stick;
        haveStick_ = true;
        return state_;
    }
    ForceState release(float elapsed) {
        haveStick_ = false; // resume must not differentiate across a pause
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
        // Do not freeze the last motion sample into a steady fallback pull.
        const float decay = std::exp(-clamp(elapsed, 0.0f, 1.0f) / .05f);
        state_.rollVelocity *= decay;
        state_.pitchVelocity *= decay;
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
    std::array<float, 2> lastStick_{};
    bool haveStick_ = false;
};
}
#endif
