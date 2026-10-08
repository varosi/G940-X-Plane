#ifndef G940_FORCE_MODEL_H
#define G940_FORCE_MODEL_H

#include "g940Protocol.h"

namespace g940 {
class ForceSmoother {
public:
    void reset() { state_ = ForceState(); }
    ForceState update(const ForceState& target, double elapsed) {
        // A long simulator frame must not turn a trim change into one jump.
        const double dt = clamp(elapsed, 0.0, 0.1);
        state_.roll = approach(state_.roll, clamp(target.roll, -1.0, 1.0), 0.5 * dt);
        // Filter small trim/AoA changes too, and taper into the new center.
        // The lower slew rate also offsets the stronger pitch spring.
        const double pitchTarget = clamp(target.pitch, -1.0, 1.0);
        const double filteredPitch = state_.pitch +
            (pitchTarget - state_.pitch) * (-std::expm1(-dt / 0.25));
        state_.pitch = approach(state_.pitch, filteredPitch, 0.25 * dt);
        const double ratio = clamp(target.speedRatio, 0.0, 1.0);
        state_.speedRatio = ratio == 0.0 ? 0.0 : approach(state_.speedRatio, ratio, dt);
        // Keep immediate stick-dependent restoring force on the Linux
        // constant-force fallback, using the same smoothed trim centers.
        state_.rollForce = target.rollForce + state_.roll - target.roll;
        state_.pitchForce = target.pitchForce + state_.pitch - target.pitch;
        return state_;
    }
private:
    static double approach(double current, double target, double step) {
        return current + clamp(target - current, -step, step);
    }
    ForceState state_;
};
}
#endif
