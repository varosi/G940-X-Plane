#ifndef G940_GROUND_MODEL_H
#define G940_GROUND_MODEL_H

#include "g940CueModel.h"

namespace g940 {
struct GroundSample {
    bool contactValid = false, onGround = false;
    float supportG = std::numeric_limits<float>::quiet_NaN(); // gear force / current weight
    float rollAcceleration = std::numeric_limits<float>::quiet_NaN(); // degrees/s²
    float pitchAcceleration = std::numeric_limits<float>::quiet_NaN();
    float groundSpeed = std::numeric_limits<float>::quiet_NaN(); // m/s
    // World coordinates retain the SDK's precision for relocation detection.
    // All force calculations use floats. Missing position does not disable cues.
    bool positionValid = false;
    std::array<double, 3> position{}; // local east/up/south, metres
    float verticalSpeed = std::numeric_limits<float>::quiet_NaN(); // world up, m/s
};

// Translate measured gear-load changes and aircraft jolts into bounded tactile
// motion. Contact selects gain; it never triggers a synthetic bump waveform.
class GroundCues {
public:
    void reset() { *this = GroundCues{}; }
    CueOffsets release(float elapsed) {
        if (!std::isfinite(elapsed) || elapsed <= 0.0f) { reset(); return {}; }
        filters_ = {};
        haveContact_ = havePosition_ = false;
        airborneTime_ = landingTime_ = 0.0f;
        const float decay = std::exp(-clamp(elapsed, 0.0f, 1.0f) / .12f);
        output_.roll *= decay;
        output_.pitch *= decay;
        return output_;
    }
    CueOffsets update(const GroundSample& sample, const AircraftProfile& profile, float elapsed) {
        if (!std::isfinite(elapsed) || elapsed <= 0.0f) { reset(); return {}; }
        if (elapsed > .05001f || !sample.contactValid) return release(elapsed);
        if (!validGain(profile.groundBumpGain) || !validGain(profile.landingBumpGain)) {
            reset(); return {};
        }
        if (profile.groundBumpGain == 0.0f && profile.landingBumpGain == 0.0f) return release(elapsed);

        bool positionValid = sample.positionValid;
        for (double coordinate : sample.position) positionValid &= std::isfinite(coordinate);
        if (positionValid && havePosition_) {
            const double horizontal = std::hypot(sample.position[0] - lastPosition_[0],
                                                  sample.position[2] - lastPosition_[2]);
            const double vertical = std::abs(sample.position[1] - lastPosition_[1]);
            const bool horizontalJump = std::isfinite(sample.groundSpeed) && sample.groundSpeed >= 0.0f &&
                horizontal > 25.0 + 2.0 * sample.groundSpeed * elapsed;
            const bool verticalJump = std::isfinite(sample.verticalSpeed) &&
                vertical > 25.0 + 2.0 * std::abs(sample.verticalSpeed) * elapsed;
            if (horizontalJump || verticalJump) {
                // A teleport or local-origin shift is not a hard landing.
                // Do not reject large forces: those can be genuine impacts.
                release(elapsed);
            }
        }
        havePosition_ = positionValid;
        if (positionValid) lastPosition_ = sample.position;

        landingTime_ = std::max(0.0f, landingTime_ - elapsed);
        if (haveContact_ && sample.onGround && !lastOnGround_ && airborneTime_ >= .25f)
            landingTime_ = 2.0f; // includes main/nose-wheel contact and short bounces
        airborneTime_ = sample.onGround ? 0.0f : std::min(.25f, airborneTime_ + elapsed);
        lastOnGround_ = sample.onGround;
        haveContact_ = true;

        // Keep sampling in the air so a real touchdown's load change survives.
        // Independent filters let rotation cues work without valid mass/load.
        const float support = filters_[0].update(sample.supportG, 20.0f, elapsed);
        const float roll = filters_[1].update(sample.rollAcceleration, 1200.0f, elapsed);
        const float pitch = filters_[2].update(sample.pitchAcceleration, 1200.0f, elapsed);
        if (!sample.onGround) {
            const float decay = std::exp(-elapsed / .12f);
            output_.roll *= decay;
            output_.pitch *= decay;
            return output_;
        }
        // Suppress parked solver chatter. Touchdown may be vertical with no
        // ground speed, so its measured impact uses a separate gain window.
        const float moving = std::isfinite(sample.groundSpeed) ?
            clamp((sample.groundSpeed - .5f) / 1.5f, 0.0f, 1.0f) : 0.0f;
        const float landing = clamp(landingTime_ / .5f, 0.0f, 1.0f);
        const float landingBlend = landing * landing * (3.0f - 2.0f * landing);
        const float gain = std::max(profile.groundBumpGain * moving,
                                    profile.landingBumpGain * landingBlend);
        // 60 degrees/s² is tactile scaling, not a measured yoke inertia.
        const CueOffsets target{-gain * roll / 60.0f, -gain * (support + pitch / 60.0f)};
        output_.roll += clamp(clamp(target.roll, -maximumCueOffset, maximumCueOffset) - output_.roll,
                              -3.0f * elapsed, 3.0f * elapsed);
        output_.pitch += clamp(clamp(target.pitch, -maximumCueOffset, maximumCueOffset) - output_.pitch,
                               -3.0f * elapsed, 3.0f * elapsed);
        return output_;
    }
private:
    struct Filter {
        bool primed = false;
        float mean = 0.0f, transient = 0.0f;
        float update(float value, float limit, float elapsed) {
            if (!std::isfinite(value)) { *this = Filter{}; return 0.0f; }
            value = clamp(value, -limit, limit);
            if (!primed) { mean = value; transient = 0.0f; primed = true; }
            mean += (value - mean) * (-std::expm1(-elapsed / .35f));
            transient += (value - mean - transient) * (-std::expm1(-elapsed / .03f));
            return transient;
        }
    };
    static bool validGain(float gain) {
        return std::isfinite(gain) && gain >= 0.0f && gain <= maximumCueOffset;
    }
    std::array<Filter, 3> filters_{};
    bool haveContact_ = false, lastOnGround_ = false, havePosition_ = false;
    float airborneTime_ = 0.0f, landingTime_ = 0.0f;
    std::array<double, 3> lastPosition_{};
    CueOffsets output_;
};

// Air and ground tails can overlap at touchdown. Share one amplitude and slew
// budget before applying the existing symmetric trim-center headroom.
class CueMixer {
public:
    void reset() { output_ = {}; }
    CueOffsets update(CueOffsets air, CueOffsets ground, float elapsed) {
        if (!std::isfinite(elapsed) || elapsed <= 0.0f) { reset(); return {}; }
        const float step = 3.0f * std::min(elapsed, .05f);
        output_.roll += clamp(clamp(air.roll + ground.roll, -maximumCueOffset, maximumCueOffset) - output_.roll, -step, step);
        output_.pitch += clamp(clamp(air.pitch + ground.pitch, -maximumCueOffset, maximumCueOffset) - output_.pitch, -step, step);
        return output_;
    }
private:
    CueOffsets output_;
};
}
#endif
