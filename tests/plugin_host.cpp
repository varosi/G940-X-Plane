// Test-only simulator/backend replacements. Never linked into a plugin.
#include "XPLMDefs.h"
// These definitions emulate the host; they must not be marked dllimport.
#undef XPLM_API
#define XPLM_API
#include "XPLMPlugin.h"
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPLMPlanes.h"
#include "XPLMUtilities.h"
#include "g940Backend.h"
#include "g940ForceModel.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

PLUGIN_API int XPluginStart(char *, char *, char *);
PLUGIN_API int XPluginEnable();
PLUGIN_API void XPluginDisable();
PLUGIN_API void XPluginStop();
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int, void *);

namespace {
enum Type { DATA_INTEGER, DATA_FLOAT, DATA_DOUBLE, DATA_INTEGERS, DATA_FLOATS, DATA_BYTES };
struct Ref {
    Type type;
    float value;
    std::string text = {};
    std::vector<float> values = {};
    std::optional<double> doubleValue = {};
};
std::map<std::string, Ref> refs;
XPLMFlightLoop_f callback = nullptr;
int registrations = 0, unregistrations = 0;
int opens = 0, closes = 0;
int releases = 0;
bool deviceOpen = false, allowOpen = true, allowUpdate = true;
bool missingRef = false;
bool nativePaths = false;
std::string pluginFile, aircraftFile = "JF_Socata_TB10+TB20.acf", debugLog;
g940::LEDState observedLEDs;
g940::ForceState observedForce;
Ref& ref(XPLMDataRef data) { assert(data); return *static_cast<Ref *>(data); }
}

extern "C" {
XPLMDataRef XPLMFindDataRef(const char *name) {
    if (missingRef) return nullptr;
    auto entry = refs.find(name);
    return entry == refs.end() ? nullptr : &entry->second;
}
XPLMPluginID XPLMGetMyID() { return 1; }
void XPLMEnableFeature(const char *feature, int enable) {
    assert(std::strcmp(feature, "XPLM_USE_NATIVE_PATHS") == 0 && enable == 1);
    nativePaths = true;
}
void XPLMGetPluginInfo(XPLMPluginID id, char *, char *path, char *, char *) {
    assert(id == 1 && nativePaths);
    std::strcpy(path, pluginFile.c_str());
}
void XPLMGetNthAircraftModel(int index, char *name, char *path) {
    assert(index == 0 && nativePaths);
    std::strcpy(name, aircraftFile.c_str());
    std::strcpy(path, ("/Aircraft/" + aircraftFile).c_str());
}
int XPLMGetDatai(XPLMDataRef data) { assert(ref(data).type == DATA_INTEGER); return ref(data).value; }
float XPLMGetDataf(XPLMDataRef data) { assert(ref(data).type == DATA_FLOAT); return ref(data).value; }
double XPLMGetDatad(XPLMDataRef data) {
    assert(ref(data).type == DATA_DOUBLE);
    return ref(data).doubleValue.value_or(ref(data).value);
}
int XPLMGetDatab(XPLMDataRef data, void *out, int offset, int count) {
    assert(ref(data).type == DATA_BYTES && offset == 0);
    const int copied = std::min(count, static_cast<int>(ref(data).text.size()));
    if (out) std::memcpy(out, ref(data).text.data(), copied);
    return out ? copied : ref(data).text.size();
}
int XPLMGetDatavi(XPLMDataRef data, int *out, int offset, int count) {
    assert(ref(data).type == DATA_INTEGERS && offset == 0 && count == 1);
    *out = ref(data).value; return 1;
}
int XPLMGetDatavf(XPLMDataRef data, float *out, int offset, int count) {
    const auto& source = ref(data);
    assert(source.type == DATA_FLOATS && offset >= 0 && count >= 0);
    if (source.values.empty()) { assert(offset == 0 && count == 1); *out = source.value; return 1; }
    if (!out) return source.values.size();
    const int copied = std::min(count, std::max(0, static_cast<int>(source.values.size()) - offset));
    std::copy_n(source.values.begin() + std::min(offset, static_cast<int>(source.values.size())), copied, out);
    return copied;
}
void XPLMRegisterFlightLoopCallback(XPLMFlightLoop_f flightLoop, float, void *) {
    assert(!callback); callback = flightLoop; ++registrations;
}
void XPLMUnregisterFlightLoopCallback(XPLMFlightLoop_f flightLoop, void *) {
    assert(callback == flightLoop); callback = nullptr; ++unregistrations;
}
void XPLMDebugString(const char *text) { debugLog += text; }
}

namespace g940 {
bool prepareForceFeedback() { return true; }
bool openForceFeedback() { ++opens; return deviceOpen = allowOpen; }
bool releaseForceFeedback() { ++releases; return true; }
bool closeForceFeedback() { if (deviceOpen) ++closes; deviceOpen = false; return true; }
bool updateForceFeedback(const ForceState& state) {
    assert(deviceOpen); observedForce = state;
    if (!allowUpdate) deviceOpen = false;
    return allowUpdate;
}
bool openLEDs() { ++opens; return deviceOpen = allowOpen; }
void closeLEDs() { if (deviceOpen) ++closes; deviceOpen = false; }
bool updateLEDs(const LEDState& state) {
    assert(deviceOpen); observedLEDs = state;
    if (!allowUpdate) deviceOpen = false;
    return allowUpdate;
}
const char *backendError() { return "simulated device error"; }
}

int main() {
    char name[256], signature[256], description[256];
#ifdef TEST_LEDS
    refs = {
        {"sim/aircraft/prop/acf_en_type", {DATA_INTEGERS, 1}},
        {"sim/aircraft2/metadata/is_glider", {DATA_INTEGER, 0}},
        {"sim/aircraft/parts/acf_flapEQ", {DATA_INTEGER, 1}},
        {"sim/aircraft/gear/acf_gear_retract", {DATA_INTEGER, 1}},
        {"sim/aircraft/parts/acf_sbrkEQ", {DATA_INTEGER, 1}},
        {"sim/cockpit2/autopilot/autopilot_on_or_cws", {DATA_INTEGER, 1}},
        {"sim/cockpit2/engine/actuators/carb_heat_ratio", {DATA_FLOATS, 1}},
        {"sim/cockpit2/controls/flap_handle_deploy_ratio", {DATA_FLOAT, .25}},
        {"sim/flightmodel2/gear/deploy_ratio", {DATA_FLOATS, .5}},
        {"sim/cockpit/electrical/landing_lights_on", {DATA_INTEGER, 0}},
        {"sim/flightmodel2/controls/speedbrake_ratio", {DATA_FLOAT, .75}}
    };
#else
    const auto configRoot = std::filesystem::temp_directory_path() /
        ("g940-host-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    const auto configFolder = configRoot / u8"G940 profiles é" / "g940FF";
    std::filesystem::create_directories(configFolder / "64");
    std::filesystem::copy_file("aircraft.ini", configFolder / "aircraft.ini");
    const auto pluginPath = (configFolder / "64/mac.xpl").generic_u8string();
    pluginFile.assign(pluginPath.begin(), pluginPath.end());
    refs = {
        {"sim/joystick/yoke_roll_ratio", {DATA_FLOAT, .1}},
        {"sim/joystick/yoke_pitch_ratio", {DATA_FLOAT, .2}},
        {"sim/flightmodel/misc/Qstatic", {DATA_FLOAT, g940::defaultProfile.referencePressurePa / (2 * g940::pascalsPerPsf)}},
        {"sim/aircraft/view/acf_Vne", {DATA_FLOAT, 187}},
        {"sim/aircraft/view/acf_ICAO", {DATA_BYTES, 0, "TOBA"}},
        {"sim/aircraft/controls/acf_elev_up", {DATA_FLOAT, 15}},
        {"sim/aircraft/controls/acf_elev_dn", {DATA_FLOAT, 15}},
        {"sim/aircraft/controls/acf_elev_tab", {DATA_FLOAT, .1f}},
        {"sim/aircraft/controls/acf_hstb_trim_up", {DATA_FLOAT, 0}},
        {"sim/aircraft/controls/acf_hstb_trim_dn", {DATA_FLOAT, 0}},
        {"sim/flightmodel2/controls/stabilizer_deflection_degrees", {DATA_FLOAT, 0}},
        {"sim/flightmodel/position/alpha", {DATA_FLOAT, 5}},
        {"sim/flightmodel2/controls/elevator_trim", {DATA_FLOAT, .1}},
        {"sim/flightmodel2/controls/aileron_trim", {DATA_FLOAT, 0}},
        {"sim/time/paused", {DATA_INTEGER, 0}}
    };
    for (const char *wind : {"sim/weather/aircraft/wind_now_x_msc", "sim/weather/aircraft/wind_now_y_msc",
                            "sim/weather/aircraft/wind_now_z_msc", "sim/flightmodel/position/psi",
                            "sim/flightmodel/position/theta", "sim/flightmodel/position/phi"})
        refs.emplace(wind, Ref{DATA_FLOAT, 0});
    refs.emplace("sim/flightmodel/failures/onground_any", Ref{DATA_INTEGER, 0});
    refs.emplace("sim/cockpit2/annunciators/stall_warning", Ref{DATA_INTEGER, 0});
    refs.emplace("sim/time/is_in_replay", Ref{DATA_INTEGER, 0});
    refs.emplace("sim/flightmodel2/misc/has_crashed", Ref{DATA_INTEGER, 0});
    refs.emplace("sim/aircraft/engine/acf_num_engines", Ref{DATA_INTEGER, 1});
    refs.emplace("sim/aircraft/engine/acf_pmax", Ref{DATA_FLOAT, 100000});
    for (const char *array : {"sim/flightmodel2/wing/elements/element_is_stalled",
                             "sim/flightmodel2/wing/elements/element_surface_area_mtr_sq"})
        refs.emplace(array, Ref{DATA_FLOATS, 0, {}, std::vector<float>(480, 0)});
    for (const char *array : {"sim/cockpit2/engine/indicators/power_watts", "sim/aircraft/engine/acf_pmax_per_engine"})
        refs.emplace(array, Ref{DATA_FLOATS, 0, {}, std::vector<float>(16, 0)});
    auto& wingArea = refs["sim/flightmodel2/wing/elements/element_surface_area_mtr_sq"].values;
    std::fill_n(wingArea.begin(), 20, 1.0f);
    // Positive unused power limits must not dilute this single-engine ratio.
    std::fill(refs["sim/aircraft/engine/acf_pmax_per_engine"].values.begin(),
              refs["sim/aircraft/engine/acf_pmax_per_engine"].values.end(), 100000);
#endif
    missingRef = true;
    assert(XPluginStart(name, signature, description) == 0);
    assert(!callback);
    missingRef = false;
    assert(XPluginStart(name, signature, description) == 1);
    assert(!callback);
    assert(XPluginEnable() == 1 && callback);
    assert(XPluginEnable() == 1 && registrations == 1);
    assert(callback(0, 0, 0, nullptr) > 0);
#ifdef TEST_LEDS
    const g940::LEDState expected = {{g940::GREEN, g940::AMBER, g940::OFF, g940::GREEN,
                                     g940::AMBER, g940::RED, g940::RED, g940::AMBER}};
    assert(observedLEDs == expected); // injected engine must not show carb heat
    refs["sim/aircraft/prop/acf_en_type"].value = 0;
    callback(0, 0, 0, nullptr);
    assert(observedLEDs[2] == g940::GREEN);
#else
    for (int i = 0; i < 100; ++i) callback(.02f, 0, 0, nullptr);
    assert(std::abs(observedForce.springRatio() - .6) < .001); // Qstatic psf converted to Pa
    assert(std::abs(observedForce.mechanicalRatio - .2f) < .001 && std::abs(observedForce.aerodynamicRatio - .4f) < .001);
    assert(std::abs(observedForce.dampingRatio - .6f) < .001);
    assert(std::abs(observedForce.roll) < .001);
    assert(std::abs(observedForce.pitch - 1.0f / 15.0f) < .001); // static tab, live trim and AoA
    assert(std::abs(observedForce.rollForce + .1) < .001);
    assert(std::abs(observedForce.pitchForce + 2.0f / 15.0f) < .001);
    assert(debugLog.find("pitch trim aerodynamic, elevator travel +15.0/-15.0 deg, static tab 0.100") != std::string::npos);
    assert(observedForce.rollCue == 0 && observedForce.pitchCue == 0);
    // Actual local wind, not merely a weather setting, produces a cue.
    refs["sim/weather/aircraft/wind_now_y_msc"].value = 2;
    callback(.02f, 0, 0, nullptr);
    assert(observedForce.pitchCue < 0 && observedForce.rollCue == 0);
    for (int i = 0; i < 500; ++i) callback(.02f, 0, 0, nullptr);
    assert(std::abs(observedForce.pitchCue) < 1e-5f);
    refs["sim/cockpit2/annunciators/stall_warning"].value = 1;
    callback(.02f, 0, 0, nullptr);
    assert(std::abs(observedForce.pitchCue) < 1e-5f); // horn alone is not a shaker
    auto& stalled = refs["sim/flightmodel2/wing/elements/element_is_stalled"].values;
    stalled[0] = 1;
    const auto peakBuffet = [&]() {
        for (int i = 0; i < 100; ++i) callback(.02f, 0, 0, nullptr);
        float peak = 0;
        for (int i = 0; i < 200; ++i) {
            callback(.02f, 0, 0, nullptr);
            peak = std::max(peak, std::abs(observedForce.pitchCue));
        }
        return peak;
    };
    const float idleBuffet = peakBuffet();
    assert(idleBuffet > .001f);
    refs["sim/cockpit2/engine/indicators/power_watts"].values[0] = 100000;
    assert(peakBuffet() > idleBuffet * 3);
    stalled[0] = 0; stalled[80] = 1; wingArea[80] = 1;
    for (int i = 0; i < 150; ++i) callback(.02f, 0, 0, nullptr);
    assert(std::abs(observedForce.pitchCue) < 1e-5f); // tail separation excluded
    stalled[0] = 1;
    assert(peakBuffet() > .001f);
    stalled.resize(79); // incomplete telemetry disables buffet, not the spring
    for (int i = 0; i < 150; ++i) callback(.02f, 0, 0, nullptr);
    assert(std::abs(observedForce.pitchCue) < 1e-5f && observedForce.springRatio() > 0);
    stalled.assign(480, 0);
    refs["sim/cockpit2/annunciators/stall_warning"].value = 0;
    refs["sim/flightmodel/misc/Qstatic"].value = 0;
    refs["sim/flightmodel/position/alpha"].value = -121;
    refs["sim/flightmodel2/controls/elevator_trim"].value = .2;
    for (int i = 0; i < 100; ++i) callback(.02f, 0, 0, nullptr);
    assert(std::abs(observedForce.springRatio() - g940::minimumForceRatio) < .001);
    assert(observedForce.aerodynamicRatio == 0 && std::abs(observedForce.dampingRatio - .2f) < .001);
    assert(std::abs(observedForce.pitch) < .001); // no spurious ground AoA pull
    refs["sim/flightmodel/misc/Qstatic"].value = -1;
    callback(0, 0, 0, nullptr);
    assert(observedForce.springRatio() == 0);
    refs["sim/time/paused"].value = 1;
    callback(0, 0, 0, nullptr);
    assert(deviceOpen && releases == 1 && closes == 0);
    refs["sim/time/paused"].value = 0;
    callback(0, 0, 0, nullptr);
    assert(deviceOpen);
    refs["sim/flightmodel/misc/Qstatic"].value = g940::defaultProfile.referencePressurePa / (2 * g940::pascalsPerPsf);
    refs["sim/flightmodel/position/alpha"].value = 5;
    refs["sim/flightmodel2/controls/elevator_trim"].value = .1;
    for (int i = 0; i < 100; ++i) callback(.02f, 0, 0, nullptr);
    refs["sim/time/paused"].value = 1;
    for (int i = 0; i < 25; ++i) callback(.02f, 0, 0, nullptr);
    assert(deviceOpen && releases == 1 && closes == 0);
    assert(std::abs(observedForce.effectScale - .5) < .001);
    refs["sim/time/paused"].value = 0;
    callback(.02f, 0, 0, nullptr);
    assert(observedForce.effectScale > .5 && observedForce.effectScale < .53);
    refs["sim/time/paused"].value = 1;
    for (int i = 0; i < 60; ++i) callback(.02f, 0, 0, nullptr);
    assert(deviceOpen && releases == 2 && closes == 0);
    const int opensBeforePausedCallback = opens;
    callback(.2f, 0, 0, nullptr);
    assert(opens == opensBeforePausedCallback); // paused callback must not restart live effects
    refs["sim/time/paused"].value = 0;
    refs["sim/flightmodel2/controls/elevator_trim"].value = .3f;
    refs["sim/flightmodel2/controls/aileron_trim"].value = -.1f;
    callback(.02f, 0, 0, nullptr);
    assert(deviceOpen && observedForce.springRatio() <= .020001);
    // Resume establishes the current trim center before raising the force,
    // so neither grip mode briefly pulls toward a stale neutral position.
    assert(std::abs(observedForce.pitch - 4.0f / 15.0f) < .00001f);
    assert(std::abs(observedForce.roll + .2f) < .00001f);
    assert(observedForce.effectScale <= .020001f);
    for (int i = 0; i < 100; ++i) callback(.02f, 0, 0, nullptr);
    refs["sim/aircraft/view/acf_ICAO"].text = "B738";
    refs["sim/aircraft/view/acf_Vne"].value = 340;
    aircraftFile = "Boeing 737.acf";
    refs["sim/aircraft/controls/acf_elev_up"].value = 30;
    refs["sim/aircraft/controls/acf_elev_dn"].value = 15;
    refs["sim/aircraft/controls/acf_elev_tab"].value = 0;
    refs["sim/aircraft/controls/acf_hstb_trim_up"].value = 4;
    refs["sim/aircraft/controls/acf_hstb_trim_dn"].value = 2;
    refs["sim/flightmodel2/controls/stabilizer_deflection_degrees"].value = 3;
    XPluginReceiveMessage(XPLM_PLUGIN_XPLANE, XPLM_MSG_PLANE_LOADED, reinterpret_cast<void *>(1));
    callback(.02f, 0, 0, nullptr);
    assert(std::abs(observedForce.springRatio() - .6f) < .001); // ignore AI plane changes
    XPluginReceiveMessage(XPLM_PLUGIN_XPLANE, XPLM_MSG_PLANE_LOADED, nullptr);
    callback(.02f, 0, 0, nullptr);
    assert(observedForce.effectScale <= .020001f && observedForce.springRatio() <= .020001f);
    for (int i = 0; i < 100; ++i) callback(.02f, 0, 0, nullptr);
    assert(std::abs(observedForce.springRatio() - .2540657f) < .001); // General at Vne=340
    assert(debugLog.find("profile 'General' for Boeing 737.acf") != std::string::npos);
    assert(debugLog.find("340.0 kt (X-Plane Vne scaling)") != std::string::npos);
    assert(debugLog.find("pitch trim stabilizer, elevator travel +30.0/-15.0 deg, static tab 0.000") != std::string::npos);
    assert(observedForce.pitch < 0);
    const float thsPitch = observedForce.pitch;
    refs["sim/flightmodel2/controls/elevator_trim"].value = -.3f;
    for (int i = 0; i < 100; ++i) callback(.02f, 0, 0, nullptr);
    assert(std::abs(observedForce.pitch - thsPitch) < 1e-6f); // no extra elevator offset for THS
    refs["sim/flightmodel2/controls/elevator_trim"].value = .3f;
    refs["sim/flightmodel2/controls/stabilizer_deflection_degrees"].value = 0;
    callback(.02f, 0, 0, nullptr);
    assert(observedForce.pitch > thsPitch && observedForce.pitch - thsPitch <= .005001f);
    for (int i = 0; i < 100; ++i) callback(.02f, 0, 0, nullptr);
    assert(observedForce.pitch > thsPitch); // incidence changes the force balance smoothly
#endif
    allowUpdate = false;
    assert(callback(0, 0, 0, nullptr) == 5.0f);
    allowOpen = false;
    assert(callback(0, 0, 0, nullptr) == 5.0f);
    allowOpen = allowUpdate = true;
    callback(0, 0, 0, nullptr);
    assert(deviceOpen);
    XPluginDisable();
    assert(!callback && !deviceOpen && unregistrations == 1);
    XPluginDisable();
    assert(unregistrations == 1);
    XPluginEnable();
    callback(0, 0, 0, nullptr);
    XPluginStop();
    assert(!callback && !deviceOpen && registrations == 2 && unregistrations == 2);
#ifndef TEST_LEDS
    std::ofstream(configFolder / "aircraft.ini") << "[General]\nreference_speed_knots=150\npitch_aoa_gain=0\npitch_trim_mode=spring\n";
    assert(XPluginEnable() == 1);
    for (int i = 0; i < 100; ++i) callback(.02f, 0, 0, nullptr);
    assert(std::abs(observedForce.springRatio() - .4777778f) < .001); // reread config on enable
    assert(observedForce.pitch > 0);
    XPluginDisable();
    const int registrationsBeforeBadConfig = registrations;
    std::ofstream(configFolder / "aircraft.ini") << "[General]\nmechanical_ratio=nan\n";
    assert(XPluginEnable() == 0 && !callback && !deviceOpen);
    assert(registrations == registrationsBeforeBadConfig);
    assert(debugLog.find("line 2 [General]") != std::string::npos);
    std::filesystem::remove(configFolder / "aircraft.ini");
    refs.erase("sim/aircraft/view/acf_ICAO");
    refs.erase("sim/aircraft/view/acf_Vne");
    for (const char *optional : {"sim/aircraft/controls/acf_elev_up", "sim/aircraft/controls/acf_elev_dn",
         "sim/aircraft/controls/acf_elev_tab", "sim/aircraft/controls/acf_hstb_trim_up", "sim/aircraft/controls/acf_hstb_trim_dn",
         "sim/flightmodel2/controls/stabilizer_deflection_degrees"}) refs.erase(optional);
    assert(XPluginStart(name, signature, description) == 1); // optional metadata absent
    assert(XPluginEnable() == 1);
    for (int i = 0; i < 100; ++i) callback(.02f, 0, 0, nullptr);
    assert(std::abs(observedForce.springRatio() - .6f) < .001); // built-in reference fallback
    assert(debugLog.find("aircraft.ini missing") != std::string::npos);
    assert(debugLog.find("pitch trim aerodynamic, elevator travel +15.0/-15.0 deg, static tab 0.000") != std::string::npos);
    XPluginStop();
    // A damping-only profile must survive the flight/pause lifecycle even
    // though its spring strength is zero. It must not restart while paused.
    std::ofstream(configFolder / "aircraft.ini") << "[General]\nmechanical_ratio=0\naerodynamic_gain=0\n"
        "mechanical_damping=.5\naerodynamic_damping=0\n";
    assert(XPluginEnable() == 1);
    for (int i = 0; i < 100; ++i) callback(.02f, 0, 0, nullptr);
    assert(observedForce.hasLoad() && observedForce.springRatio() == 0 && observedForce.dampingRatio == .5f);
    refs["sim/time/paused"].value = 1;
    callback(.02f, 0, 0, nullptr);
    assert(observedForce.effectScale > .99f); // starts the requested one-second fade
    for (int i = 0; i < 60; ++i) callback(.02f, 0, 0, nullptr);
    const int dampedOpens = opens;
    callback(.2f, 0, 0, nullptr);
    assert(opens == dampedOpens);
    XPluginStop();
    // Cue telemetry is optional. Exercise the real callback through separate
    // lifecycles so erased datarefs can never leave a running plugin pointer.
    std::ofstream(configFolder / "aircraft.ini") << "[General]\nreference_speed_knots=125\n"
        "pitch_aoa_gain=0\nturbulence_gain=.015\nstall_buffet_gain=.06\n";
    constexpr const char *stalledName = "sim/flightmodel2/wing/elements/element_is_stalled";
    constexpr const char *areaName = "sim/flightmodel2/wing/elements/element_surface_area_mtr_sq";
    constexpr const char *powerName = "sim/cockpit2/engine/indicators/power_watts";
    constexpr const char *maximumName = "sim/aircraft/engine/acf_pmax_per_engine";
    constexpr const char *countName = "sim/aircraft/engine/acf_num_engines";
    constexpr const char *legacyName = "sim/aircraft/engine/acf_pmax";
    constexpr const char *windName = "sim/weather/aircraft/wind_now_y_msc";
    const float invalidTelemetry = std::numeric_limits<float>::quiet_NaN();
    const auto resetTelemetry = [&]() {
        assert(!callback);
        refs["sim/time/paused"].value = 0;
        refs["sim/time/is_in_replay"].value = 0;
        refs["sim/flightmodel2/misc/has_crashed"].value = 0;
        refs["sim/flightmodel/failures/onground_any"].value = 0;
        for (const char *wind : {"sim/weather/aircraft/wind_now_x_msc", windName,
             "sim/weather/aircraft/wind_now_z_msc", "sim/flightmodel/position/psi",
             "sim/flightmodel/position/theta", "sim/flightmodel/position/phi"}) refs[wind].value = 0;
        refs[stalledName].values.assign(480, 0);
        refs[stalledName].values[0] = 1;
        refs[areaName].values.assign(480, 0);
        std::fill_n(refs[areaName].values.begin(), 20, 1.0f);
        refs[powerName].values.assign(16, 100000);
        refs[maximumName].values.assign(16, 100000);
        refs[countName].value = 1;
        refs[legacyName].value = 100000;
    };
    const auto beginCueFlight = [&]() {
        XPluginStop();
        assert(XPluginStart(name, signature, description) == 1);
        assert(XPluginEnable() == 1 && callback);
        for (int i = 0; i < 100; ++i) callback(.02f, 0, 0, nullptr);
        assert(observedForce.springRatio() > .59f); // optional cues never disable the base load
    };
    const auto buffetRms = [&]() {
        beginCueFlight();
        float squares = 0;
        for (int i = 0; i < 300; ++i) {
            callback(.02f, 0, 0, nullptr);
            assert(std::isfinite(observedForce.pitchCue));
            squares += observedForce.pitchCue * observedForce.pitchCue;
        }
        return std::sqrt(squares / 300);
    };
    resetTelemetry();
    const float fullPowerRms = buffetRms();
    assert(fullPowerRms > .005f);
    XPluginStop();
    refs[powerName].values[0] = 0;
    const float idlePowerRms = buffetRms();
    assert(std::abs(idlePowerRms / fullPowerRms - .25f) < .001f);
    // Huge/invalid unused engine slots cannot dilute or contaminate the one
    // installed engine. Only the reported engine count participates.
    XPluginStop(); resetTelemetry();
    refs[powerName].values[15] = invalidTelemetry;
    refs[maximumName].values[15] = 1e30f;
    assert(std::abs(buffetRms() / fullPowerRms - 1.0f) < .001f);
    XPluginStop(); resetTelemetry();
    refs[countName].value = 2;
    refs[powerName].values[0] = 50000; refs[powerName].values[1] = 100000;
    refs[maximumName].values[1] = 300000;
    assert(std::abs(buffetRms() / fullPowerRms - (.25f + .75f * .375f)) < .001f);
    XPluginStop();
    refs[maximumName].values.resize(1); // short modern limit array falls back to legacy per-engine limit
    assert(std::abs(buffetRms() / fullPowerRms - (.25f + .75f * .75f)) < .001f);
    XPluginStop();
    refs[powerName].values.resize(1); // short actual power cannot invent the second engine's output
    assert(std::abs(buffetRms() / idlePowerRms - 1.0f) < .001f);
    for (float invalidCount : {0.0f, 17.0f}) {
        XPluginStop(); resetTelemetry(); refs[countName].value = invalidCount;
        assert(std::abs(buffetRms() / idlePowerRms - 1.0f) < .001f);
    }
    for (const char *invalidArray : {powerName, maximumName}) {
        XPluginStop(); resetTelemetry(); refs[invalidArray].values[0] = invalidTelemetry;
        assert(std::abs(buffetRms() / idlePowerRms - 1.0f) < .001f);
    }
    XPluginStop(); resetTelemetry();
    refs[maximumName].values[0] = -1;
    assert(std::abs(buffetRms() / idlePowerRms - 1.0f) < .001f);

    // Missing power uses weak idle buffet; a missing modern power limit uses
    // the legacy scalar. Missing count conservatively assumes one engine.
    for (const char *missing : {powerName, maximumName, countName}) {
        XPluginStop(); resetTelemetry();
        const Ref backup = refs.at(missing);
        refs.erase(missing);
        const float expected = missing == powerName ? idlePowerRms : fullPowerRms;
        assert(std::abs(buffetRms() / expected - 1.0f) < .001f);
        XPluginStop(); refs.emplace(missing, backup);
    }
    XPluginStop(); resetTelemetry();
    const Ref savedMaximum = refs.at(maximumName);
    refs.erase(maximumName); refs[legacyName].value = invalidTelemetry;
    assert(std::abs(buffetRms() / idlePowerRms - 1.0f) < .001f);
    XPluginStop(); refs.emplace(maximumName, savedMaximum);

    for (const char *array : {stalledName, areaName}) {
        XPluginStop(); resetTelemetry(); refs[array].values.resize(79);
        assert(buffetRms() < 1e-6f); // partial wing telemetry suppresses only buffet
        XPluginStop(); resetTelemetry(); refs[array].values[0] = invalidTelemetry;
        assert(buffetRms() < 1e-6f);
        XPluginStop(); resetTelemetry();
        const Ref backup = refs.at(array); refs.erase(array);
        assert(buffetRms() < 1e-6f);
        refs[windName].value = 2;
        callback(.02f, 0, 0, nullptr);
        assert(observedForce.pitchCue < 0); // wind remains independently available
        XPluginStop(); refs.emplace(array, backup);
    }
    for (const char *missing : {windName, "sim/flightmodel/position/phi"}) {
        XPluginStop(); resetTelemetry();
        const Ref backup = refs.at(missing); refs.erase(missing);
        assert(std::abs(buffetRms() / fullPowerRms - 1.0f) < .001f);
        refs[stalledName].values[0] = 0;
        refs["sim/weather/aircraft/wind_now_x_msc"].value = 3;
        for (int i = 0; i < 150; ++i) callback(.02f, 0, 0, nullptr);
        assert(std::abs(observedForce.rollCue) < 1e-6f && std::abs(observedForce.pitchCue) < 1e-6f);
        XPluginStop(); refs.emplace(missing, backup);
    }
    XPluginStop(); resetTelemetry(); refs[windName].value = invalidTelemetry;
    assert(std::abs(buffetRms() / fullPowerRms - 1.0f) < .001f); // bad wind does not erase real buffet
    for (const char *suppressed : {"sim/flightmodel/failures/onground_any", "sim/time/is_in_replay",
                                  "sim/flightmodel2/misc/has_crashed"}) {
        XPluginStop(); resetTelemetry();
        assert(buffetRms() > .005f);
        refs[suppressed].value = 1;
        refs[windName].value = 3;
        for (int i = 0; i < 150; ++i) callback(.02f, 0, 0, nullptr);
        assert(std::abs(observedForce.rollCue) < 1e-6f && std::abs(observedForce.pitchCue) < 1e-6f);
        assert(observedForce.springRatio() > .59f);
    }
    XPluginStop(); resetTelemetry();
    const Ref savedGround = refs.at("sim/flightmodel/failures/onground_any");
    refs.erase("sim/flightmodel/failures/onground_any"); // cannot safely establish airborne state
    assert(buffetRms() < 1e-6f);
    XPluginStop(); refs.emplace("sim/flightmodel/failures/onground_any", savedGround);
    resetTelemetry(); assert(buffetRms() > .005f);
    const int releasesBeforeCuePause = releases;
    refs["sim/time/paused"].value = 1;
    for (int i = 0; i < 150; ++i) callback(.02f, 0, 0, nullptr);
    assert(releases > releasesBeforeCuePause && std::abs(observedForce.pitchCue) < .0001f);
    XPluginStop();

    // Ground cues use actual gear/support loads and body angular acceleration,
    // independently of airspeed and of the optional airborne cue channels.
    constexpr const char *gearLoadName = "sim/flightmodel/forces/fnrml_gear";
    constexpr const char *massName = "sim/flightmodel/weight/m_total";
    constexpr const char *groundSpeedName = "sim/flightmodel/position/groundspeed";
    constexpr const char *rollAccelerationName = "sim/flightmodel/position/P_dot";
    constexpr const char *pitchAccelerationName = "sim/flightmodel/position/Q_dot";
    constexpr const char *verticalVelocityName = "sim/flightmodel/position/local_vy";
    constexpr const char *groundStateName = "sim/flightmodel/failures/onground_any";
    constexpr float weightNewtons = 1000.0f * 9.80665f;
    refs.emplace(gearLoadName, Ref{DATA_FLOAT, weightNewtons});
    refs.emplace(massName, Ref{DATA_FLOAT, 1000});
    refs.emplace(groundSpeedName, Ref{DATA_FLOAT, 5});
    refs.emplace(rollAccelerationName, Ref{DATA_FLOAT, 0});
    refs.emplace(pitchAccelerationName, Ref{DATA_FLOAT, 0});
    refs.emplace(verticalVelocityName, Ref{DATA_FLOAT, 0});
    for (const char *position : {"sim/flightmodel/position/local_x", "sim/flightmodel/position/local_y",
                                "sim/flightmodel/position/local_z"})
        refs.emplace(position, Ref{DATA_DOUBLE, 0, {}, {}, 1000000000000.125});
    const auto resetGroundTelemetry = [&]() {
        assert(!callback);
        resetTelemetry();
        refs[stalledName].values.assign(480, 0);
        refs["sim/flightmodel/misc/Qstatic"].value = 0;
        refs["sim/flightmodel2/controls/elevator_trim"].value = 0;
        refs["sim/flightmodel2/controls/aileron_trim"].value = 0;
        refs[groundStateName].value = 1;
        refs[gearLoadName].value = weightNewtons;
        refs[massName].value = 1000;
        refs[groundSpeedName].value = 5;
        refs[rollAccelerationName].value = refs[pitchAccelerationName].value = 0;
        refs[verticalVelocityName].value = 0;
        for (const char *position : {"sim/flightmodel/position/local_x", "sim/flightmodel/position/local_y",
                                    "sim/flightmodel/position/local_z"})
            refs[position].doubleValue = 1000000000000.125;
    };
    const auto groundStep = [&]() {
        callback(.02f, 0, 0, nullptr);
        assert(deviceOpen && observedForce.hasLoad());
        assert(std::isfinite(observedForce.rollCue) && std::isfinite(observedForce.pitchCue));
        assert(std::abs(observedForce.rollCue) <= .120001f && std::abs(observedForce.pitchCue) <= .120001f);
    };
    const auto settleGround = [&]() {
        for (int i = 0; i < 150; ++i) groundStep();
        assert(std::abs(observedForce.rollCue) < 1e-5f && std::abs(observedForce.pitchCue) < 1e-5f);
    };
    const auto beginGroundFlight = [&]() {
        XPluginStop();
        std::ofstream(configFolder / "aircraft.ini") << "[General]\nreference_speed_knots=125\npitch_aoa_gain=0\n"
            "turbulence_gain=0\nstall_buffet_gain=0\nground_bump_gain=.02\nlanding_bump_gain=.04\n";
        assert(XPluginStart(name, signature, description) == 1);
        assert(XPluginEnable() == 1 && callback);
        settleGround();
        assert(observedForce.aerodynamicRatio == 0 && std::abs(observedForce.springRatio() - .2f) < .001f);
    };
    resetGroundTelemetry();
    beginGroundFlight(); // initially loaded on the ground is not a landing
    refs[gearLoadName].value = weightNewtons * 1.8f;
    groundStep();
    assert(std::abs(observedForce.pitchCue) > 1e-5f && observedForce.rollCue == 0);
    const auto groundForceReport = g940::forceReport(observedForce);
    const auto groundIdleCenter = g940::idleCenterReport(groundForceReport);
    const auto signed16 = [](uint8_t low, uint8_t high) {
        return static_cast<int16_t>(static_cast<uint16_t>(low) | static_cast<uint16_t>(high) << 8);
    };
    const int livePitchCenter = signed16(groundForceReport[37], groundForceReport[38]);
    const int idlePitchCenter = signed16(groundIdleCenter[3], groundIdleCenter[4]);
    assert(livePitchCenter == idlePitchCenter && livePitchCenter != 0);
    assert(std::abs(livePitchCenter / 32767.0f - observedForce.center(1)) < 1.0f / 32767.0f);
    settleGround(); // a steady load is not a continuous invented vibration
    refs[rollAccelerationName].value = 30;
    groundStep();
    assert(observedForce.rollCue != 0);
    const float positiveRollCue = observedForce.rollCue;
    refs[rollAccelerationName].value = 0;
    settleGround();
    refs[rollAccelerationName].value = -30;
    groundStep();
    assert(observedForce.rollCue * positiveRollCue < 0);

    // Stationary controls keep mechanical resistance without taxi vibration.
    XPluginStop(); resetGroundTelemetry(); refs[groundSpeedName].value = 0;
    beginGroundFlight();
    refs[gearLoadName].value = weightNewtons * 2;
    refs[rollAccelerationName].value = 30;
    groundStep();
    assert(observedForce.rollCue == 0 && observedForce.pitchCue == 0);
    const auto landingPeak = [&](float supportG) {
        XPluginStop(); resetGroundTelemetry();
        refs[groundStateName].value = 0;
        refs[gearLoadName].value = 0;
        beginGroundFlight(); // observed airborne history arms the touchdown detector
        refs[groundStateName].value = 1;
        refs[gearLoadName].value = weightNewtons * supportG;
        float peak = 0;
        for (int i = 0; i < 100; ++i) {
            groundStep();
            peak = std::max(peak, std::abs(observedForce.pitchCue));
        }
        return peak;
    };
    const float softLanding = landingPeak(1.2f);
    const float hardLanding = landingPeak(3.0f);
    assert(softLanding > 1e-5f && hardLanding > softLanding);
    XPluginStop(); resetGroundTelemetry(); beginGroundFlight();
    refs["sim/flightmodel/position/local_x"].doubleValue = 1000000000026.125;
    refs[gearLoadName].value = weightNewtons * 3;
    groundStep();
    assert(observedForce.rollCue == 0 && observedForce.pitchCue == 0); // teleport, including at large coordinates
    settleGround();

    // Support telemetry is optional: angular cues still work without either
    // mass or normal gear force. Missing contact disables only these cues.
    for (const char *missing : {massName, gearLoadName}) {
        XPluginStop(); resetGroundTelemetry();
        const Ref backup = refs.at(missing); refs.erase(missing);
        beginGroundFlight();
        refs[rollAccelerationName].value = 30;
        groundStep();
        assert(observedForce.rollCue != 0);
        XPluginStop(); refs.emplace(missing, backup);
    }
    for (float invalidMass : {0.0f, invalidTelemetry}) {
        XPluginStop(); resetGroundTelemetry(); refs[massName].value = invalidMass;
        beginGroundFlight();
        refs[pitchAccelerationName].value = 30;
        groundStep();
        assert(observedForce.pitchCue != 0 && observedForce.rollCue == 0);
    }
    XPluginStop(); resetGroundTelemetry();
    const Ref savedPosition = refs.at("sim/flightmodel/position/local_x");
    refs.erase("sim/flightmodel/position/local_x");
    beginGroundFlight();
    refs[gearLoadName].value = weightNewtons * 2;
    groundStep();
    assert(observedForce.pitchCue != 0); // optional teleport monitoring does not gate the force cue
    XPluginStop(); refs.emplace("sim/flightmodel/position/local_x", savedPosition);
    XPluginStop(); resetGroundTelemetry();
    const Ref savedContact = refs.at(groundStateName); refs.erase(groundStateName);
    beginGroundFlight();
    refs[gearLoadName].value = weightNewtons * 3;
    refs[rollAccelerationName].value = 30;
    for (int i = 0; i < 30; ++i) groundStep();
    assert(observedForce.rollCue == 0 && observedForce.pitchCue == 0);
    XPluginStop(); refs.emplace(groundStateName, savedContact);

    // Pause/replay and scenery reloads re-prime loads instead of treating a
    // stale baseline as a new bump. Neither changes the user's force profile.
    XPluginStop(); resetGroundTelemetry(); beginGroundFlight();
    refs[gearLoadName].value = weightNewtons * 2;
    groundStep();
    assert(observedForce.pitchCue != 0);
    refs["sim/time/paused"].value = 1;
    for (int i = 0; i < 100; ++i) callback(.02f, 0, 0, nullptr);
    assert(std::abs(observedForce.pitchCue) < 1e-5f);
    refs[gearLoadName].value = weightNewtons * 3;
    refs["sim/time/paused"].value = 0;
    groundStep();
    assert(observedForce.rollCue == 0 && observedForce.pitchCue == 0);
    settleGround();
    refs["sim/time/is_in_replay"].value = 1;
    refs[gearLoadName].value = weightNewtons;
    settleGround();
    refs["sim/time/is_in_replay"].value = 0;
    groundStep();
    assert(observedForce.rollCue == 0 && observedForce.pitchCue == 0);
    refs[gearLoadName].value = weightNewtons * 2;
    XPluginReceiveMessage(XPLM_PLUGIN_XPLANE, XPLM_MSG_AIRPORT_LOADED, nullptr);
    groundStep();
    assert(observedForce.rollCue == 0 && observedForce.pitchCue == 0);
    assert(std::abs(observedForce.springRatio() - .2f) < .001f);
    XPluginStop();

    // Air cues remain available with incomplete ground telemetry. At touchdown
    // their decaying tail shares the same motor-offset budget with gear loads.
    resetGroundTelemetry();
    refs[groundStateName].value = 0;
    refs[gearLoadName].value = 0;
    refs["sim/flightmodel/misc/Qstatic"].value = g940::defaultProfile.referencePressurePa / (2 * g940::pascalsPerPsf);
    refs[stalledName].values[0] = 1;
    std::ofstream(configFolder / "aircraft.ini") << "[General]\nreference_speed_knots=125\npitch_aoa_gain=0\n"
        "turbulence_gain=.015\nstall_buffet_gain=.06\nground_bump_gain=.12\nlanding_bump_gain=.12\n";
    const Ref savedMass = refs.at(massName); refs.erase(massName);
    assert(buffetRms() > .005f);
    XPluginStop(); refs.emplace(massName, savedMass);
    beginCueFlight();
    refs[groundStateName].value = 1;
    refs[gearLoadName].value = weightNewtons * 10;
    refs[rollAccelerationName].value = -1200;
    refs[pitchAccelerationName].value = -1200;
    float previousRollCue = observedForce.rollCue, previousPitchCue = observedForce.pitchCue;
    for (int i = 0; i < 100; ++i) {
        groundStep();
        assert(std::abs(observedForce.rollCue - previousRollCue) <= .060001f);
        assert(std::abs(observedForce.pitchCue - previousPitchCue) <= .060001f);
        previousRollCue = observedForce.rollCue; previousPitchCue = observedForce.pitchCue;
    }
    XPluginStop();
    std::filesystem::remove_all(configRoot);
#endif
    std::puts("Dataref types, reconnect, pause, and plugin lifecycle passed.");
}

// Compile the real plugin against this host's declarations. On Windows this
// also avoids importing SDK symbols from a DLL in the standalone test runner.
#ifdef TEST_LEDS
#include "../g940LEDs.cpp"
#else
#include "../g940FF.cpp"
#endif
