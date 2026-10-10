#ifndef G940_CUE_MODEL_H
#define G940_CUE_MODEL_H

#include "g940ForceModel.h"
#include <limits>
#include <span>

namespace g940 {
constexpr size_t mainWingElements = 80; // eight main-wing surfaces, ten slots each
constexpr float maximumCueOffset = 0.12f;

inline float stalledWingFraction(std::span<const float> stalled, std::span<const float> area) {
    if (stalled.size() < mainWingElements || area.size() < mainWingElements)
        return std::numeric_limits<float>::quiet_NaN();
    float total = 0.0f, separated = 0.0f;
    for (size_t i = 0; i < mainWingElements; ++i) {
        if (!std::isfinite(area[i]) || area[i] < 0.0f ||
            !std::isfinite(stalled[i]) || stalled[i] < 0.0f || stalled[i] > 1.0f)
            return std::numeric_limits<float>::quiet_NaN();
        total += area[i];
        separated += area[i] * stalled[i];
    }
    return total > 0.0f && std::isfinite(total) && std::isfinite(separated) ?
        separated / total : std::numeric_limits<float>::quiet_NaN();
}

struct CueSample {
    std::array<float, 3> wind{}; // world east/up/south, m/s
    float heading = 0.0f, pitch = 0.0f, roll = 0.0f; // degrees
    float pressurePa = 0.0f;
    float stalledFraction = 0.0f, enginePower = 0.0f;
    bool windValid = false, airborne = false;
};
struct CueOffsets { float roll = 0.0f, pitch = 0.0f; };

// A tactile approximation driven by simulator wind and wing separation.
// It never changes the simulator's weather, controls or flight-model forces.
class FlightCues {
public:
    void reset() { *this = FlightCues{}; }
    CueOffsets release(float elapsed) {
        if (!std::isfinite(elapsed) || elapsed <= 0.0f) { reset(); return {}; }
        haveWind_ = false;
        envelope_ = phase_ = secondaryPhase_ = 0.0f;
        const float decay = std::exp(-clamp(elapsed, 0.0f, 1.0f) / 0.12f);
        output_.roll *= decay;
        output_.pitch *= decay;
        return output_;
    }
    CueOffsets update(const CueSample& sample, const AircraftProfile& profile, float elapsed) {
        // Below 20 updates/s, suppress the sampled waveform instead of aliasing
        // it into slow pulls. Re-prime after delayed frames or discontinuities.
        if (!std::isfinite(elapsed) || elapsed <= 0.0f) {
            reset(); return {};
        }
        if (elapsed > 0.05001f) return release(elapsed);
        if (!sample.airborne || !std::isfinite(sample.pressurePa) || sample.pressurePa <= 0.0f)
            return release(elapsed);
        if (!std::isfinite(profile.turbulenceGain) || profile.turbulenceGain < 0.0f || profile.turbulenceGain > .1f ||
            !std::isfinite(profile.stallBuffetGain) || profile.stallBuffetGain < 0.0f || profile.stallBuffetGain > maximumCueOffset ||
            !std::isfinite(profile.stallBuffetIdleRatio) || profile.stallBuffetIdleRatio < 0.0f || profile.stallBuffetIdleRatio > 1.0f ||
            !std::isfinite(profile.stallBuffetHz) || profile.stallBuffetHz < 2.0f || profile.stallBuffetHz > 6.0f) {
            reset(); return {};
        }
        CueOffsets target;
        bool windValid = sample.windValid && std::isfinite(sample.heading) &&
            std::isfinite(sample.pitch) && std::isfinite(sample.roll);
        for (float wind : sample.wind) windValid &= std::isfinite(wind) && std::abs(wind) <= 200.0f;
        if (windValid && profile.turbulenceGain > 0.0f) {
            if (!haveWind_) { meanWind_ = sample.wind; filteredGust_.fill(0.0f); haveWind_ = true; }
            bool discontinuity = false;
            for (unsigned axis = 0; axis < 3; ++axis)
                discontinuity |= std::abs(sample.wind[axis] - lastWind_[axis]) > 20.0f;
            if (discontinuity) { meanWind_ = sample.wind; filteredGust_.fill(0.0f); }
            for (unsigned axis = 0; axis < 3; ++axis) {
                meanWind_[axis] += (sample.wind[axis] - meanWind_[axis]) * (-std::expm1(-elapsed / .8f));
                filteredGust_[axis] += (sample.wind[axis] - meanWind_[axis] - filteredGust_[axis]) *
                    (-std::expm1(-elapsed / .08f));
            }
            lastWind_ = sample.wind;
            // Filter in world coordinates before rotating: steady wind must
            // not become a fake gust when the pilot turns or banks.
            constexpr float degrees = 0.01745329252f;
            const float h = std::remainder(sample.heading, 360.0f) * degrees;
            const float p = std::remainder(sample.pitch, 360.0f) * degrees;
            const float r = std::remainder(sample.roll, 360.0f) * degrees;
            const float xh = filteredGust_[0] * std::cos(h) + filteredGust_[2] * std::sin(h);
            const float zh = filteredGust_[2] * std::cos(h) - filteredGust_[0] * std::sin(h);
            const float up = filteredGust_[1] * std::cos(p) + zh * std::sin(p);
            target.roll = -profile.turbulenceGain * (xh * std::cos(r) - up * std::sin(r));
            target.pitch = -profile.turbulenceGain * (xh * std::sin(r) + up * std::cos(r));
        } else {
            haveWind_ = false;
            filteredGust_.fill(0.0f);
        }
        // The horn is deliberately not an input. Incipient separation on
        // actual wing elements, not indicated speed, drives natural buffet.
        const float separation = std::isfinite(sample.stalledFraction) ?
            clamp(sample.stalledFraction / .2f, 0.0f, 1.0f) : 0.0f;
        const float power = clamp(sample.enginePower, 0.0f, 1.0f); // missing power uses weak idle feel
        const float strength = profile.stallBuffetGain * separation *
            (profile.stallBuffetIdleRatio + (1.0f - profile.stallBuffetIdleRatio) * power);
        envelope_ += (strength - envelope_) * (-std::expm1(-elapsed / .2f));
        constexpr float turn = 6.28318530718f;
        phase_ = std::fmod(phase_ + turn * profile.stallBuffetHz * elapsed, turn);
        secondaryPhase_ = std::fmod(secondaryPhase_ + turn * profile.stallBuffetHz * 1.37f * elapsed, turn);
        // Two different frequencies give bounded, irregular natural
        // buffet rather than an artificial stick-shaker alarm.
        target.pitch += envelope_ * (std::sin(phase_) + .25f * std::sin(secondaryPhase_)) / 1.25f;
        const float airflow = clamp(sample.pressurePa / 100.0f, 0.0f, 1.0f);
        target.roll = clamp(target.roll * airflow, -maximumCueOffset, maximumCueOffset);
        target.pitch = clamp(target.pitch * airflow, -maximumCueOffset, maximumCueOffset);
        // Bound per-update movement as well as amplitude, including cue removal.
        output_.roll += clamp(target.roll - output_.roll, -3.0f * elapsed, 3.0f * elapsed);
        output_.pitch += clamp(target.pitch - output_.pitch, -3.0f * elapsed, 3.0f * elapsed);
        return output_;
    }
private:
    bool haveWind_ = false;
    std::array<float, 3> meanWind_{}, lastWind_{}, filteredGust_{};
    float envelope_ = 0.0f, phase_ = 0.0f, secondaryPhase_ = 0.0f;
    CueOffsets output_;
};

inline ForceState withCues(ForceState state, CueOffsets cues) {
    // Use symmetric headroom, preserving the mean trim center near a stop.
    // Damping-only profiles have no spring with which to render these cues.
    const float rollRoom = std::max(0.0f, 1.0f - std::abs(state.roll));
    const float pitchRoom = std::max(0.0f, 1.0f - std::abs(state.pitch));
    state.rollCue = state.springRatio() > 0.0f ? clamp(cues.roll, -std::min(rollRoom, maximumCueOffset), std::min(rollRoom, maximumCueOffset)) : 0.0f;
    state.pitchCue = state.springRatio() > 0.0f ? clamp(cues.pitch, -std::min(pitchRoom, maximumCueOffset), std::min(pitchRoom, maximumCueOffset)) : 0.0f;
    return state;
}
}
#endif
