#ifndef G940_LED_CONFIG_H
#define G940_LED_CONFIG_H

#include "g940Protocol.h"
#include <map>
#include <optional>
#include <string>

namespace g940 {
enum class LEDFunction {
    off, red, green, amber, speedbrakeUpper, flapsUpper, carbHeat, autopilot,
    speedbrakeLower, flapsLower, landingLights, gear, dataref,
    engineRunning, navigationLights, parkingBrake, brakes
};
struct LEDAssignment {
    LEDFunction function = LEDFunction::off;
    std::string dataref = {};
    std::optional<int> index = {};
    bool hasThresholds = false;
    float low = 0.0f, high = 1.0f;
};
// Keys are the physical P1-P8 labels, matching led_1 through led_8 in the INI.
using LEDAssignments = std::map<unsigned, LEDAssignment>;
inline const LEDAssignments defaultLEDAssignments = {
    {1, {LEDFunction::speedbrakeUpper}}, {2, {LEDFunction::flapsUpper}},
    {3, {LEDFunction::carbHeat}}, {4, {LEDFunction::autopilot}},
    {5, {LEDFunction::speedbrakeLower}}, {6, {LEDFunction::flapsLower}},
    {7, {LEDFunction::landingLights}}, {8, {LEDFunction::gear}}
};
template<class Number>
inline LEDColour datarefLEDColour(Number value, const LEDAssignment& assignment) {
    if (!std::isfinite(value)) return OFF;
    if (!assignment.hasThresholds) return value == 0.0f ? RED : GREEN;
    return value <= assignment.low ? RED : value >= assignment.high ? GREEN : AMBER;
}
}
#endif
