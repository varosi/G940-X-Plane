#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include "XPLMPlugin.h"
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPLMPlanes.h"
#include "XPLMUtilities.h"
#include "g940Backend.h"
#include "g940Config.h"

#ifndef XPLM300
#error This plugin requires the XPLM300 API
#endif

namespace {
XPLMDataRef autopilotRef, engTypeRef, gliderRef, carbHeatRef, haveFlapsRef,
    flapsRef, isRetractRef, gearRef, landLightRef, haveSbrkRef, speedBrakeRef, icaoRef;
XPLMDataRef engineRunningRef, legacyEngineRunningRef, engineCountRef, navigationLightsRef;
XPLMDataRef masterBrakeRef, oldMasterBrakeRef, legacyBrakeRef, leftBrakeRef, rightBrakeRef,
    parkingBrakeTrapRef, parkingBrakeValveRef;
std::filesystem::path configFile;
std::vector<g940::ConfigProfile> profiles;
g940::LEDAssignments assignments;
struct Binding { XPLMDataRef ref = nullptr; int types = 0; bool warned = false; };
std::array<Binding, 8> bindings;
bool enabled = false, ledsReady = false, profileDirty = true, missingBindings = false;
float retryIn = 0.0f;

g940::LEDColour greenOn(bool value) { return value ? g940::GREEN : g940::RED; }
g940::LEDColour green1(float value) {
    return !std::isfinite(value) ? g940::OFF :
        value <= 0.0f ? g940::RED : value >= 1.0f ? g940::GREEN : g940::AMBER;
}
g940::LEDColour topHalf(float value) {
    return !std::isfinite(value) ? g940::OFF :
        value <= 0.125f ? g940::RED : value <= 0.375f ? g940::AMBER : g940::GREEN;
}
g940::LEDColour bottomHalf(float value) {
    return !std::isfinite(value) ? g940::OFF :
        value <= 0.625f ? g940::RED : value <= 0.875f ? g940::AMBER : g940::GREEN;
}
g940::LEDColour brakeColour(float value) {
    return !std::isfinite(value) ? g940::OFF :
        value <= 0.0f ? g940::GREEN : value >= 1.0f ? g940::RED : g940::AMBER;
}
bool on(XPLMDataRef ref) { return ref && XPLMGetDatai(ref) != 0; }
float arrayValue(XPLMDataRef ref) {
    float value = std::numeric_limits<float>::quiet_NaN();
    if (!ref || XPLMGetDatavf(ref, &value, 0, 1) != 1)
        return std::numeric_limits<float>::quiet_NaN();
    return value;
}
void reportError() {
    const std::string message = std::string("G940 LEDs: ") + g940::backendError() + "\n";
    XPLMDebugString(message.c_str());
}

bool loadProfiles() {
    try {
        std::ifstream input(configFile);
        if (input) profiles = g940::readAircraftConfig(input);
        else {
            if (std::filesystem::exists(configFile)) throw std::runtime_error("cannot read configuration");
            profiles = {g940::ConfigProfile{}};
            XPLMDebugString("G940 LEDs: aircraft.ini missing; using default LED assignments\n");
        }
        return true;
    } catch (const std::exception& error) {
        const auto message = std::string("G940 LEDs: aircraft.ini: ") + error.what() + "\n";
        XPLMDebugString(message.c_str());
        return false;
    }
}

void resolveBindings() {
    struct Reference { XPLMDataRef *target; const char *name; };
    const Reference references[] = {
        {&engTypeRef, "sim/aircraft/prop/acf_en_type"},
        {&gliderRef, "sim/aircraft2/metadata/is_glider"},
        {&haveFlapsRef, "sim/aircraft/parts/acf_flapEQ"},
        {&isRetractRef, "sim/aircraft/gear/acf_gear_retract"},
        {&haveSbrkRef, "sim/aircraft/parts/acf_sbrkEQ"},
        {&autopilotRef, "sim/cockpit2/autopilot/autopilot_on_or_cws"},
        {&carbHeatRef, "sim/cockpit2/engine/actuators/carb_heat_ratio"},
        {&flapsRef, "sim/cockpit2/controls/flap_handle_deploy_ratio"},
        {&gearRef, "sim/flightmodel2/gear/deploy_ratio"},
        {&landLightRef, "sim/cockpit/electrical/landing_lights_on"},
        {&speedBrakeRef, "sim/flightmodel2/controls/speedbrake_ratio"},
        {&engineRunningRef, "sim/flightmodel2/engines/engine_is_burning_fuel"},
        {&legacyEngineRunningRef, "sim/flightmodel/engine/ENGN_running"},
        {&engineCountRef, "sim/aircraft/engine/acf_num_engines"},
        {&navigationLightsRef, "sim/cockpit2/switches/navigation_lights_on"},
        {&masterBrakeRef, "sim/cockpit2/controls/wheel_brake_ratio"},
        {&oldMasterBrakeRef, "sim/cockpit2/controls/parking_brake_ratio"},
        {&legacyBrakeRef, "sim/flightmodel/controls/parkbrake"},
        {&leftBrakeRef, "sim/cockpit2/controls/left_brake_ratio"},
        {&rightBrakeRef, "sim/cockpit2/controls/right_brake_ratio"},
        {&parkingBrakeTrapRef, "sim/aircraft/gear/acf_park_brake_trap"},
        {&parkingBrakeValveRef, "sim/cockpit2/controls/park_brake_valve"}
    };
    missingBindings = false;
    for (const auto& reference : references) {
        *reference.target = XPLMFindDataRef(reference.name);
        missingBindings |= !*reference.target;
    }
    for (const auto& [led, assignment] : assignments) {
        if (assignment.function != g940::LEDFunction::dataref) continue;
        auto& binding = bindings[led - 1];
        binding.ref = XPLMFindDataRef(assignment.dataref.c_str());
        binding.types = binding.ref && XPLMIsDataRefGood(binding.ref) ? XPLMGetDataRefTypes(binding.ref) : 0;
        const int numericTypes = assignment.index ? xplmType_IntArray | xplmType_FloatArray :
            xplmType_Int | xplmType_Float | xplmType_Double | xplmType_IntArray | xplmType_FloatArray;
        if (!(binding.types & numericTypes)) {
            binding.ref = nullptr;
            missingBindings = true;
            if (!binding.warned) {
                const auto message = "G940 LEDs: led_" + std::to_string(led) +
                    " unavailable or incompatible dataref: " + assignment.dataref + "\n";
                XPLMDebugString(message.c_str());
                binding.warned = true;
            }
        } else binding.warned = false;
    }
    retryIn = 5.0f;
}

void selectProfile() {
    char filename[256] = {}, path[512] = {}, icao[41] = {};
    XPLMGetNthAircraftModel(0, filename, path);
    icaoRef = XPLMFindDataRef("sim/aircraft/view/acf_ICAO");
    if (icaoRef) {
        const int count = XPLMGetDatab(icaoRef, icao, 0, sizeof(icao) - 1);
        if (count < 0 || count >= static_cast<int>(sizeof(icao))) icao[0] = 0;
        else icao[count] = 0;
    }
    const auto& profile = g940::selectAircraftProfile(profiles, icao, filename);
    assignments = profile.leds;
    bindings = {};
    resolveBindings();
    const auto message = "G940 LEDs: profile '" + profile.name + "' for " + filename + "\n";
    XPLMDebugString(message.c_str());
    profileDirty = false;
}

g940::LEDColour customColour(unsigned led) {
    const auto& assignment = assignments.at(led);
    auto& binding = bindings[led - 1];
    if (!binding.ref) return g940::OFF;
    if (!XPLMIsDataRefGood(binding.ref)) {
        binding.ref = nullptr;
        missingBindings = true;
        return g940::OFF;
    }
    float value = 0.0f;
    // Explicit indices require an array. Unindexed numeric scalars take
    // precedence; an array-only dataref uses its first element.
    if (!assignment.index && (binding.types & xplmType_Float)) value = XPLMGetDataf(binding.ref);
    else if (!assignment.index && (binding.types & xplmType_Double))
        return g940::datarefLEDColour(XPLMGetDatad(binding.ref), assignment);
    else if (!assignment.index && (binding.types & xplmType_Int)) value = static_cast<float>(XPLMGetDatai(binding.ref));
    else if (binding.types & xplmType_FloatArray) {
        if (XPLMGetDatavf(binding.ref, &value, assignment.index.value_or(0), 1) != 1) return g940::OFF;
    } else if (binding.types & xplmType_IntArray) {
        int integer = 0;
        if (XPLMGetDatavi(binding.ref, &integer, assignment.index.value_or(0), 1) != 1) return g940::OFF;
        value = static_cast<float>(integer);
    } else return g940::OFF;
    return g940::datarefLEDColour(value, assignment);
}

g940::LEDColour engineColour() {
    const int count = engineCountRef ? XPLMGetDatai(engineCountRef) : 0;
    const auto ref = engineRunningRef ? engineRunningRef : legacyEngineRunningRef;
    std::array<int, 16> running{};
    if (count < 1 || count > static_cast<int>(running.size()) || !ref ||
        XPLMGetDatavi(ref, running.data(), 0, count) != count) return g940::OFF;
    int started = 0;
    for (int engine = 0; engine < count; ++engine) {
        if (running[engine] != 0 && running[engine] != 1) return g940::OFF;
        started += running[engine];
    }
    return started == 0 ? g940::RED : started == count ? g940::GREEN : g940::AMBER;
}

float masterBrake() {
    const auto ref = masterBrakeRef ? masterBrakeRef : oldMasterBrakeRef ? oldMasterBrakeRef : legacyBrakeRef;
    return ref ? XPLMGetDataf(ref) : std::numeric_limits<float>::quiet_NaN();
}

g940::LEDColour parkingBrakeColour() {
    const int trap = parkingBrakeTrapRef ? XPLMGetDatai(parkingBrakeTrapRef) : 0;
    if (trap < 0 || trap > 2) return g940::OFF;
    if (trap == 0) return brakeColour(masterBrake());
    if (!parkingBrakeValveRef) return g940::OFF;
    const int closed = XPLMGetDatai(parkingBrakeValveRef);
    // This shows the parking control, not whether a closed valve trapped
    // usable pressure. Do not replace its status with current pedal demand.
    return closed == 0 ? g940::GREEN : closed == 1 ? g940::RED : g940::OFF;
}

g940::LEDColour brakesColour() {
    if (!leftBrakeRef || !rightBrakeRef) return g940::OFF;
    const float master = masterBrake(), left = XPLMGetDataf(leftBrakeRef), right = XPLMGetDataf(rightBrakeRef);
    if (!std::isfinite(master) || !std::isfinite(left) || !std::isfinite(right)) return g940::OFF;
    return brakeColour(std::max({master, left, right}));
}

g940::LEDColour colour(unsigned led) {
    using enum g940::LEDFunction;
    switch (assignments.at(led).function) {
    case off: return g940::OFF;
    case red: return g940::RED;
    case green: return g940::GREEN;
    case amber: return g940::AMBER;
    case speedbrakeUpper: return on(haveSbrkRef) && speedBrakeRef ? topHalf(XPLMGetDataf(speedBrakeRef)) : g940::OFF;
    case speedbrakeLower: return on(haveSbrkRef) && speedBrakeRef ? bottomHalf(XPLMGetDataf(speedBrakeRef)) : g940::OFF;
    case flapsUpper: return on(haveFlapsRef) && flapsRef ? topHalf(XPLMGetDataf(flapsRef)) : g940::OFF;
    case flapsLower: return on(haveFlapsRef) && flapsRef ? bottomHalf(XPLMGetDataf(flapsRef)) : g940::OFF;
    case carbHeat: {
        int engineType = -1;
        return gliderRef && !on(gliderRef) && engTypeRef &&
            XPLMGetDatavi(engTypeRef, &engineType, 0, 1) == 1 && engineType == 0 ?
            green1(arrayValue(carbHeatRef)) : g940::OFF;
    }
    case autopilot: return gliderRef && !on(gliderRef) && autopilotRef ? greenOn(on(autopilotRef)) : g940::OFF;
    case landingLights: return gliderRef && !on(gliderRef) && landLightRef ? greenOn(on(landLightRef)) : g940::OFF;
    case engineRunning: return engineColour();
    case navigationLights: return navigationLightsRef ? greenOn(on(navigationLightsRef)) : g940::OFF;
    case parkingBrake: return parkingBrakeColour();
    case brakes: return brakesColour();
    case gear: return on(isRetractRef) ? green1(arrayValue(gearRef)) : g940::OFF;
    case dataref: return customColour(led);
    }
    return g940::OFF;
}

float flightLoopCallback(float elapsed, float, int, void *) {
    if (profileDirty) selectProfile();
    else if (missingBindings) {
        if (std::isfinite(elapsed) && elapsed > 0.0f) retryIn -= elapsed;
        if (retryIn <= 0.0f) resolveBindings();
    }
    if (!ledsReady) {
        if (!g940::openLEDs()) { reportError(); return 5.0f; }
        ledsReady = true;
        XPLMDebugString("G940 LEDs: LED device connected\n");
    }
    g940::LEDState wanted{};
    for (const auto& [led, assignment] : assignments) wanted[led - 1] = colour(led);
    if (!g940::updateLEDs(wanted)) {
        reportError(); ledsReady = false; return 5.0f;
    }
    return 0.2f;
}
}

PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc) {
    std::strcpy(outName, "G940 LEDs");
    std::strcpy(outSig, "name.boyle.chris.xpg940leds");
    std::strcpy(outDesc, "Sets the Logitech G940 throttle LEDs from aircraft state.");
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);
    char pluginPath[512] = {};
    XPLMGetPluginInfo(XPLMGetMyID(), nullptr, pluginPath, nullptr, nullptr);
    if (!pluginPath[0]) {
        XPLMDebugString("G940 LEDs: cannot locate plugin configuration directory\n");
        return 0;
    }
    auto folder = std::filesystem::path(std::u8string(pluginPath, pluginPath + std::strlen(pluginPath))).parent_path();
    if (folder.filename() == "64") folder = folder.parent_path();
    configFile = folder.parent_path() / "g940FF" / "aircraft.ini";
    return 1;
}

PLUGIN_API int XPluginEnable() {
    if (!enabled) {
        if (!loadProfiles()) return 0;
        profileDirty = true;
        XPLMRegisterFlightLoopCallback(flightLoopCallback, 0.01f, nullptr);
        enabled = true;
    }
    return 1;
}

PLUGIN_API void XPluginDisable() {
    if (enabled) XPLMUnregisterFlightLoopCallback(flightLoopCallback, nullptr);
    enabled = false;
    g940::closeLEDs();
    ledsReady = false;
    bindings = {};
}

PLUGIN_API void XPluginStop() { XPluginDisable(); }
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID sender, int message, void *parameter) {
    if (sender == XPLM_PLUGIN_XPLANE && message == XPLM_MSG_PLANE_LOADED && parameter == nullptr)
        profileDirty = true;
}
