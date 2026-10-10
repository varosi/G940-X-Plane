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
    bool good = true;
};
std::map<std::string, Ref> refs;
std::map<std::string, unsigned> lookups;
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
    ++lookups[name];
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
int XPLMIsDataRefGood(XPLMDataRef data) { return data && ref(data).good; }
int XPLMGetDatai(XPLMDataRef data) {
    assert(ref(data).type == DATA_INTEGER);
    return ref(data).good ? ref(data).value : 0;
}
float XPLMGetDataf(XPLMDataRef data) {
    assert(ref(data).type == DATA_FLOAT);
    return ref(data).good ? ref(data).value : 0.0f;
}
double XPLMGetDatad(XPLMDataRef data) {
    assert(ref(data).type == DATA_DOUBLE);
    return ref(data).good ? ref(data).doubleValue.value_or(ref(data).value) : 0.0;
}
XPLMDataTypeID XPLMGetDataRefTypes(XPLMDataRef data) {
    switch (ref(data).type) {
    case DATA_INTEGER: return xplmType_Int;
    case DATA_FLOAT: return xplmType_Float;
    case DATA_DOUBLE: return xplmType_Double;
    case DATA_INTEGERS: return xplmType_IntArray;
    case DATA_FLOATS: return xplmType_FloatArray;
    case DATA_BYTES: return xplmType_Data;
    }
    assert(false); return xplmType_Unknown;
}
int XPLMGetDatab(XPLMDataRef data, void *out, int offset, int count) {
    assert(ref(data).type == DATA_BYTES && offset == 0);
    const int copied = std::min(count, static_cast<int>(ref(data).text.size()));
    if (out) std::memcpy(out, ref(data).text.data(), copied);
    return out ? copied : ref(data).text.size();
}
int XPLMGetDatavi(XPLMDataRef data, int *out, int offset, int count) {
    const auto& source = ref(data);
    assert(source.type == DATA_INTEGERS && offset >= 0 && count >= 0);
    if (!out) return source.values.empty() ? 1 : source.values.size();
    if (source.values.empty()) {
        if (offset != 0 || count == 0) return 0;
        *out = source.value; return 1;
    }
    const int copied = std::min(count, std::max(0, static_cast<int>(source.values.size()) - offset));
    for (int i = 0; i < copied; ++i) out[i] = source.values[offset + i];
    return copied;
}
int XPLMGetDatavf(XPLMDataRef data, float *out, int offset, int count) {
    const auto& source = ref(data);
    assert(source.type == DATA_FLOATS && offset >= 0 && count >= 0);
    if (!out) return source.values.empty() ? 1 : source.values.size();
    if (source.values.empty()) {
        if (offset != 0 || count == 0) return 0;
        *out = source.value; return 1;
    }
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
    const auto configRoot = std::filesystem::temp_directory_path() /
        ("g940-host-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    const auto bundleRoot = configRoot / u8"G940 profiles é";
    const auto configFolder = bundleRoot / "g940FF";
    std::filesystem::create_directories(configFolder / "64");
    std::filesystem::copy_file("aircraft.ini", configFolder / "aircraft.ini");
#ifdef TEST_LEDS
    const auto pluginFolder = bundleRoot / "g940LEDs";
    std::filesystem::create_directories(pluginFolder / "64");
    const auto pluginPath = (pluginFolder / "64/mac.xpl").generic_u8string();
    pluginFile.assign(pluginPath.begin(), pluginPath.end());
    refs = {
        {"sim/aircraft/view/acf_ICAO", {DATA_BYTES, 0, "TOBA"}},
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
#ifdef TEST_LEDS
    assert(XPluginStart(name, signature, description) == 1); // optional indicators may be unavailable
#else
    assert(XPluginStart(name, signature, description) == 0);
#endif
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
#else
    // Both plugins share the FF sibling's configuration, including Unicode
    // paths; an accidentally read LED-local config would mask these mappings.
    std::ofstream(pluginFolder / "aircraft.ini") << "[General]\nled_1=red\n";
    const auto restartLEDs = [&]() {
        XPluginStop();
        assert(XPluginStart(name, signature, description) == 1);
        assert(XPluginEnable() == 1 && callback);
        assert(callback(.2f, 0, 0, nullptr) == .2f);
        assert(deviceOpen);
    };
    const auto configureLEDs = [&](const char *configuration) {
        XPluginStop();
        std::ofstream(configFolder / "aircraft.ini") << configuration;
        restartLEDs();
    };
    // Remap every legacy indication and inherit the General settings. Aircraft
    // changes choose a new preset only for the user's plane, not an AI plane.
    configureLEDs("[General]\nled_1=landing_lights\nled_2=gear\n"
        "led_3=flaps_lower\nled_4=flaps_upper\nled_5=speedbrake_lower\n"
        "led_6=speedbrake_upper\nled_7=autopilot\nled_8=carb_heat\n"
        "[Socata]\nmatch_icao=TOBA\nmatch_acf=*Socata*.acf\nled_8=amber\n");
    const g940::LEDState mapped = {{g940::RED, g940::AMBER, g940::RED, g940::AMBER,
        g940::AMBER, g940::GREEN, g940::GREEN, g940::AMBER}};
    assert(observedLEDs == mapped);
    refs["sim/aircraft/view/acf_ICAO"].text = "B738";
    aircraftFile = "Boeing 737.acf";
    XPluginReceiveMessage(XPLM_PLUGIN_XPLANE, XPLM_MSG_PLANE_LOADED, reinterpret_cast<void *>(1));
    callback(.2f, 0, 0, nullptr);
    assert(observedLEDs == mapped);
    XPluginReceiveMessage(XPLM_PLUGIN_XPLANE, XPLM_MSG_PLANE_LOADED, nullptr);
    callback(.2f, 0, 0, nullptr);
    auto generalMapped = mapped; generalMapped[7] = g940::GREEN;
    assert(observedLEDs == generalMapped);
    // Missing ICAO still allows filename matching, and does not prevent load.
    XPluginStop();
    const Ref savedICAO = refs.at("sim/aircraft/view/acf_ICAO");
    refs.erase("sim/aircraft/view/acf_ICAO");
    aircraftFile = "JF_Socata_TB10+TB20.acf";
    restartLEDs();
    assert(observedLEDs == mapped);
    XPluginStop(); refs.emplace("sim/aircraft/view/acf_ICAO", savedICAO);

    configureLEDs("[General]\nled_1=red\nled_2=green\nled_3=amber\n"
        "led_4=off\nled_5=off\nled_6=green\nled_7=red\nled_8=amber\n");
    const g940::LEDState constantColours = {{g940::RED, g940::GREEN, g940::AMBER, g940::OFF,
        g940::OFF, g940::GREEN, g940::RED, g940::AMBER}};
    assert(observedLEDs == constantColours);
    // A mixed-case aircraft-owned name must be preserved; its lower-case
    // spelling is a different, absent dataref in this host.
    refs.emplace("Custom/State", Ref{DATA_FLOAT, 0});
    refs.emplace("Custom/Double", Ref{DATA_DOUBLE, 1});
    refs.emplace("Custom/Integer", Ref{DATA_INTEGER, 0});
    refs.emplace("Custom/Floats", Ref{DATA_FLOATS, 0, {}, {1, 1, .5f}});
    refs.emplace("Custom/Integers", Ref{DATA_INTEGERS, 0, {}, {2, 2, 1}});
    refs.emplace("Custom/Bytes", Ref{DATA_BYTES, 0, "unreadable as a number"});
    const char *customConfiguration = "[General]\nled_1=dataref:Custom/State\nled_2=dataref:Custom/Double\n"
        "led_3=dataref:Custom/Integer\nled_4=dataref:Custom/Floats[2], 0.1, 0.9\n"
        "led_5=dataref:Custom/Integers[2], 0, 2\nled_6=dataref:Custom/Floats\n"
        "led_7=dataref:Custom/State[0]\nled_8=dataref:Custom/Bytes\n";
    configureLEDs(customConfiguration);
    const g940::LEDState customColours = {{g940::RED, g940::GREEN, g940::RED, g940::AMBER,
        g940::AMBER, g940::GREEN, g940::OFF, g940::OFF}};
    assert(observedLEDs == customColours);
    assert(lookups["Custom/State"] > 0 && lookups["custom/state"] == 0);
    refs["Custom/State"].value = -.25f; // any nonzero binary signal is green
    refs["Custom/Integer"].value = 1;
    callback(.2f, 0, 0, nullptr);
    assert(observedLEDs[0] == g940::GREEN && observedLEDs[2] == g940::GREEN);
    refs["Custom/Floats"].values[2] = .1f;
    refs["Custom/Integers"].values[2] = 0;
    callback(.2f, 0, 0, nullptr);
    assert(observedLEDs[3] == g940::RED && observedLEDs[4] == g940::RED);
    refs["Custom/Floats"].values[2] = .9f;
    refs["Custom/Integers"].values[2] = 2;
    callback(.2f, 0, 0, nullptr);
    assert(observedLEDs[3] == g940::GREEN && observedLEDs[4] == g940::GREEN);
    refs["Custom/Floats"].values[2] = .1001f;
    callback(.2f, 0, 0, nullptr);
    assert(observedLEDs[3] == g940::AMBER);
    const float badLEDValue = std::numeric_limits<float>::quiet_NaN();
    refs["Custom/State"].value = badLEDValue;
    refs["Custom/Double"].value = badLEDValue;
    refs["Custom/Floats"].values[2] = badLEDValue;
    callback(.2f, 0, 0, nullptr);
    assert(observedLEDs[0] == g940::OFF && observedLEDs[1] == g940::OFF && observedLEDs[3] == g940::OFF);
    assert(observedLEDs[2] == g940::GREEN && observedLEDs[5] == g940::GREEN);
    // Native double datarefs must retain their range and precision. Casting
    // huge/tiny finite values to float would falsely turn this binary LED OFF
    // or RED; actual nonfinite values remain unavailable.
    for (double finiteValue : {std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(),
                               std::numeric_limits<double>::min(), std::numeric_limits<double>::denorm_min()}) {
        refs["Custom/Double"].doubleValue = finiteValue;
        callback(.2f, 0, 0, nullptr);
        assert(observedLEDs[1] == g940::GREEN && observedLEDs[2] == g940::GREEN);
    }
    for (double nonfiniteValue : {std::numeric_limits<double>::infinity(),
                                  -std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::quiet_NaN()}) {
        refs["Custom/Double"].doubleValue = nonfiniteValue;
        callback(.2f, 0, 0, nullptr);
        assert(observedLEDs[1] == g940::OFF && observedLEDs[2] == g940::GREEN);
    }
    refs["Custom/Double"].doubleValue = 0.0;
    callback(.2f, 0, 0, nullptr);
    assert(observedLEDs[1] == g940::RED);
    refs["Custom/Floats"].values.resize(2);
    refs["Custom/Integers"].values.resize(2);
    callback(.2f, 0, 0, nullptr);
    assert(observedLEDs[3] == g940::OFF && observedLEDs[4] == g940::OFF);
    assert(observedLEDs[5] == g940::GREEN); // implicit array index is zero

    // The SDK may retain an orphaned handle and its original type after an
    // aircraft plugin unloads. Its scalar getter then returns zero, which
    // must not masquerade as a valid RED indication.
    refs["Custom/State"].value = 1;
    refs["Custom/Double"].value = 1;
    refs["Custom/Double"].doubleValue.reset();
    configureLEDs(customConfiguration);
    assert(observedLEDs[0] == g940::GREEN);
    refs["Custom/State"].good = false;
    assert(XPLMGetDataRefTypes(&refs["Custom/State"]) == xplmType_Float);
    assert(XPLMGetDataf(&refs["Custom/State"]) == 0.0f);
    callback(.2f, 0, 0, nullptr);
    assert(observedLEDs[0] == g940::OFF && observedLEDs[1] == g940::GREEN);
    // Keep the orphan present until a scheduled resolution sees it too.
    for (int i = 0; i < 30; ++i) callback(.2f, 0, 0, nullptr);
    assert(observedLEDs[0] == g940::OFF);
    refs["Custom/State"].good = true;
    for (int i = 0; i < 30; ++i) callback(.2f, 0, 0, nullptr);
    assert(observedLEDs[0] == g940::GREEN);
    refs["Custom/State"].good = false;
    XPluginReceiveMessage(XPLM_PLUGIN_XPLANE, XPLM_MSG_PLANE_LOADED, nullptr);
    callback(.2f, 0, 0, nullptr);
    assert(observedLEDs[0] == g940::OFF);
    refs["Custom/State"].good = true;
    XPluginReceiveMessage(XPLM_PLUGIN_XPLANE, XPLM_MSG_PLANE_LOADED, nullptr);
    callback(.2f, 0, 0, nullptr);
    assert(observedLEDs[0] == g940::GREEN);
    configureLEDs("[General]\nled_1=green\nled_2=dataref:Custom/Double, 0.1, 0.9\n");
    refs["Custom/Double"].doubleValue = std::numeric_limits<double>::max();
    callback(.2f, 0, 0, nullptr);
    assert(observedLEDs[0] == g940::GREEN && observedLEDs[1] == g940::GREEN);
    refs["Custom/Double"].doubleValue = -std::numeric_limits<double>::max();
    callback(.2f, 0, 0, nullptr);
    assert(observedLEDs[0] == g940::GREEN && observedLEDs[1] == g940::RED);

    // A failed configuration reload must not register a callback or touch
    // hardware. A missing shared config restores the legacy defaults.
    XPluginStop();
    const int registeredBeforeMalformed = registrations, openedBeforeMalformed = opens;
    std::ofstream(configFolder / "aircraft.ini") << "[General]\nled_1=unknown-indication\n";
    assert(XPluginEnable() == 0 && !callback && !deviceOpen);
    assert(registrations == registeredBeforeMalformed && opens == openedBeforeMalformed);
    std::filesystem::remove(configFolder / "aircraft.ini");
    restartLEDs();
    auto defaultColours = expected; defaultColours[2] = g940::GREEN;
    assert(observedLEDs == defaultColours);
    assert(debugLog.find("aircraft.ini missing") != std::string::npos);
    XPluginStop();
    missingRef = true;
    restartLEDs();
    assert(observedLEDs == g940::LEDState{}); // every missing indicator is OFF
    XPluginStop(); missingRef = false;

    // Optional metadata suppresses only dependent indicators. Corrupt sensor
    // values likewise affect only the LEDs that use that channel.
    const Ref savedGlider = refs.at("sim/aircraft2/metadata/is_glider");
    refs.erase("sim/aircraft2/metadata/is_glider");
    restartLEDs();
    auto missingMetadata = defaultColours;
    missingMetadata[2] = missingMetadata[3] = missingMetadata[6] = g940::OFF;
    assert(observedLEDs == missingMetadata);
    XPluginStop(); refs.emplace("sim/aircraft2/metadata/is_glider", savedGlider);
    refs["sim/cockpit2/controls/flap_handle_deploy_ratio"].value = badLEDValue;
    restartLEDs();
    auto badFlaps = defaultColours; badFlaps[1] = badFlaps[5] = g940::OFF;
    assert(observedLEDs == badFlaps);
    XPluginStop(); refs["sim/cockpit2/controls/flap_handle_deploy_ratio"].value = .25f;

    // Add-on datarefs can appear after plugin startup. Retry missing channels
    // every five seconds, and retry immediately when the user changes planes.
    constexpr const char *flapName = "sim/cockpit2/controls/flap_handle_deploy_ratio";
    const Ref savedFlaps = refs.at(flapName);
    refs.erase(flapName);
    configureLEDs("[General]\nled_1=dataref:Late/Ready\nled_2=flaps_upper\n"
        "led_3=off\nled_4=green\nled_5=off\nled_6=off\nled_7=off\nled_8=off\n");
    assert(observedLEDs[0] == g940::OFF && observedLEDs[1] == g940::OFF && observedLEDs[3] == g940::GREEN);
    const auto firstLookupCount = lookups["Late/Ready"];
    refs.emplace("Late/Ready", Ref{DATA_FLOAT, 1});
    refs.emplace(flapName, savedFlaps);
    for (int i = 0; i < 15; ++i) callback(.2f, 0, 0, nullptr);
    assert(observedLEDs[0] == g940::OFF && observedLEDs[1] == g940::OFF);
    assert(lookups["Late/Ready"] == firstLookupCount);
    for (int i = 0; i < 15; ++i) callback(.2f, 0, 0, nullptr);
    assert(observedLEDs[0] == g940::GREEN && observedLEDs[1] == g940::AMBER);
    assert(lookups["Late/Ready"] > firstLookupCount);
    configureLEDs("[General]\nled_1=dataref:Late/Plane\nled_2=off\n"
        "led_3=off\nled_4=off\nled_5=off\nled_6=off\nled_7=off\nled_8=off\n");
    assert(observedLEDs[0] == g940::OFF);
    refs.emplace("Late/Plane", Ref{DATA_INTEGER, 1});
    XPluginReceiveMessage(XPLM_PLUGIN_XPLANE, XPLM_MSG_PLANE_LOADED, nullptr);
    callback(.2f, 0, 0, nullptr);
    assert(observedLEDs[0] == g940::GREEN);
    XPluginStop();

    // Engine status uses only the aircraft's configured engines. The modern
    // channel takes precedence; a legacy channel is a missing-data fallback.
    constexpr const char *engineCountName = "sim/aircraft/engine/acf_num_engines";
    constexpr const char *modernRunningName = "sim/flightmodel2/engines/engine_is_burning_fuel";
    constexpr const char *legacyRunningName = "sim/flightmodel/engine/ENGN_running";
    constexpr const char *navigationName = "sim/cockpit2/switches/navigation_lights_on";
    refs.emplace(engineCountName, Ref{DATA_INTEGER, 2});
    refs.emplace(modernRunningName, Ref{DATA_INTEGERS, 0, {}, std::vector<float>(16, 0)});
    refs.emplace(legacyRunningName, Ref{DATA_INTEGERS, 0, {}, std::vector<float>(16, 1)});
    refs.emplace(navigationName, Ref{DATA_INTEGER, 0});
    const char *statusConfiguration = "[General]\nled_1=engine_running\nled_2=navigation_lights\n"
        "led_3=off\nled_4=green\nled_5=off\nled_6=off\nled_7=off\nled_8=off\n";
    const auto expectStatusLEDs = [&](g940::LEDColour engines, g940::LEDColour navigation) {
        callback(.2f, 0, 0, nullptr);
        assert(observedLEDs[0] == engines && observedLEDs[1] == navigation);
        assert(observedLEDs[3] == g940::GREEN);
    };
    configureLEDs(statusConfiguration);
    expectStatusLEDs(g940::RED, g940::RED); // contradictory legacy values are ignored
    refs[modernRunningName].values[0] = 1;
    expectStatusLEDs(g940::AMBER, g940::RED);
    refs[modernRunningName].values[1] = 1;
    refs[modernRunningName].values[15] = 2; // unused slots are not aircraft engines
    refs[navigationName].value = 1;
    expectStatusLEDs(g940::GREEN, g940::GREEN);
    refs["sim/aircraft2/metadata/is_glider"].value = 1;
    expectStatusLEDs(g940::GREEN, g940::GREEN); // these roles have no glider gate
    refs["sim/aircraft2/metadata/is_glider"].value = 0;
    refs[modernRunningName].values[0] = 2;
    expectStatusLEDs(g940::OFF, g940::GREEN);
    refs[modernRunningName].values[0] = 1;
    for (float invalidCount : {0.0f, -1.0f, 17.0f}) {
        refs[engineCountName].value = invalidCount;
        expectStatusLEDs(g940::OFF, g940::GREEN);
    }
    refs[engineCountName].value = 16;
    refs[modernRunningName].values.assign(16, 1);
    expectStatusLEDs(g940::GREEN, g940::GREEN);
    refs[engineCountName].value = 2;
    refs[modernRunningName].values.resize(1);
    expectStatusLEDs(g940::OFF, g940::GREEN); // short modern data must not use legacy
    refs[modernRunningName].values.assign(16, 1);

    XPluginStop();
    const Ref savedEngineCount = refs.at(engineCountName);
    refs.erase(engineCountName);
    configureLEDs(statusConfiguration);
    expectStatusLEDs(g940::OFF, g940::GREEN);
    XPluginStop();
    refs.emplace(engineCountName, savedEngineCount);
    const Ref savedModernRunning = refs.at(modernRunningName);
    refs.erase(modernRunningName);
    configureLEDs(statusConfiguration);
    expectStatusLEDs(g940::GREEN, g940::GREEN);
    refs[legacyRunningName].values[1] = 0;
    expectStatusLEDs(g940::AMBER, g940::GREEN);
    refs[legacyRunningName].values[0] = 0;
    refs[navigationName].value = 0;
    expectStatusLEDs(g940::RED, g940::RED);
    refs[legacyRunningName].values.resize(1);
    expectStatusLEDs(g940::OFF, g940::RED);
    XPluginStop();
    refs.erase(legacyRunningName);
    configureLEDs(statusConfiguration);
    expectStatusLEDs(g940::OFF, g940::RED);
    XPluginStop();
    refs.emplace(modernRunningName, savedModernRunning);
    const Ref savedNavigation = refs.at(navigationName);
    refs.erase(navigationName);
    configureLEDs(statusConfiguration);
    expectStatusLEDs(g940::GREEN, g940::OFF); // missing lights leave other roles working
    XPluginStop();
    refs.emplace(navigationName, savedNavigation);

    // Parking brakes indicate either master demand or the trapping valve;
    // combined brakes also include the independent left/right pedal demands.
    constexpr const char *masterBrakeName = "sim/cockpit2/controls/wheel_brake_ratio";
    constexpr const char *parkingBrakeName = "sim/cockpit2/controls/parking_brake_ratio";
    constexpr const char *legacyBrakeName = "sim/flightmodel/controls/parkbrake";
    constexpr const char *leftBrakeName = "sim/cockpit2/controls/left_brake_ratio";
    constexpr const char *rightBrakeName = "sim/cockpit2/controls/right_brake_ratio";
    constexpr const char *brakeTrapName = "sim/aircraft/gear/acf_park_brake_trap";
    constexpr const char *brakeValveName = "sim/cockpit2/controls/park_brake_valve";
    refs.emplace(masterBrakeName, Ref{DATA_FLOAT, 0});
    refs.emplace(parkingBrakeName, Ref{DATA_FLOAT, 1});
    refs.emplace(legacyBrakeName, Ref{DATA_FLOAT, 1});
    refs.emplace(leftBrakeName, Ref{DATA_FLOAT, 0});
    refs.emplace(rightBrakeName, Ref{DATA_FLOAT, 0});
    refs.emplace(brakeTrapName, Ref{DATA_INTEGER, 0});
    refs.emplace(brakeValveName, Ref{DATA_INTEGER, 0});
    const char *brakeConfiguration = "[General]\nled_1=parking_brake\nled_2=brakes\n"
        "led_3=off\nled_4=green\nled_5=off\nled_6=off\nled_7=off\nled_8=off\n";
    const auto expectBrakeLEDs = [&](g940::LEDColour parking, g940::LEDColour brakes) {
        callback(.2f, 0, 0, nullptr);
        assert(observedLEDs[0] == parking && observedLEDs[1] == brakes);
        assert(observedLEDs[3] == g940::GREEN);
    };
    configureLEDs(brakeConfiguration);
    expectBrakeLEDs(g940::GREEN, g940::GREEN); // modern zero overrides legacy demands
    refs[masterBrakeName].value = .5f;
    expectBrakeLEDs(g940::AMBER, g940::AMBER);
    refs[masterBrakeName].value = 1;
    expectBrakeLEDs(g940::RED, g940::RED);
    refs[masterBrakeName].value = 1.5f;
    expectBrakeLEDs(g940::RED, g940::RED);
    refs[masterBrakeName].value = -.5f;
    expectBrakeLEDs(g940::GREEN, g940::GREEN);
    refs[masterBrakeName].value = 0;
    refs[leftBrakeName].value = .3f;
    expectBrakeLEDs(g940::GREEN, g940::AMBER);
    refs[rightBrakeName].value = 1;
    expectBrakeLEDs(g940::GREEN, g940::RED);
    refs[leftBrakeName].value = refs[rightBrakeName].value = 0;
    for (float trap : {1.0f, 2.0f}) {
        refs[brakeTrapName].value = trap;
        refs[brakeValveName].value = 1;
        expectBrakeLEDs(g940::RED, g940::GREEN); // closed even without master pressure
        refs[masterBrakeName].value = 1;
        refs[brakeValveName].value = 0;
        expectBrakeLEDs(g940::GREEN, g940::RED); // open even with braking demand
        refs[masterBrakeName].value = 0;
        for (float invalidValve : {-1.0f, 2.0f}) {
            refs[brakeValveName].value = invalidValve;
            expectBrakeLEDs(g940::OFF, g940::GREEN);
        }
    }
    refs[brakeValveName].value = 0;
    for (float invalidTrap : {-1.0f, 3.0f}) {
        refs[brakeTrapName].value = invalidTrap;
        expectBrakeLEDs(g940::OFF, g940::GREEN);
    }
    refs[brakeTrapName].value = 0;
    for (const char *input : {masterBrakeName, leftBrakeName, rightBrakeName}) {
        for (float invalidDemand : {badLEDValue, std::numeric_limits<float>::infinity(),
                                   -std::numeric_limits<float>::infinity()}) {
            refs[input].value = invalidDemand;
            expectBrakeLEDs(input == masterBrakeName ? g940::OFF : g940::GREEN, g940::OFF);
        }
        refs[input].value = 0;
    }

    XPluginStop();
    const Ref savedBrakeTrap = refs.at(brakeTrapName);
    refs.erase(brakeTrapName);
    refs[masterBrakeName].value = .5f;
    configureLEDs(brakeConfiguration);
    expectBrakeLEDs(g940::AMBER, g940::AMBER); // old simulators have no trapping metadata
    XPluginStop();
    refs.emplace(brakeTrapName, savedBrakeTrap);
    const Ref savedBrakeValve = refs.at(brakeValveName);
    refs.erase(brakeValveName);
    refs[brakeTrapName].value = 1;
    configureLEDs(brakeConfiguration);
    expectBrakeLEDs(g940::OFF, g940::AMBER);
    refs[brakeTrapName].value = 0;
    expectBrakeLEDs(g940::AMBER, g940::AMBER); // ordinary brakes do not require a valve
    XPluginStop();
    refs.emplace(brakeValveName, savedBrakeValve);
    refs.erase(masterBrakeName);
    refs[parkingBrakeName].value = .25f;
    configureLEDs(brakeConfiguration);
    expectBrakeLEDs(g940::AMBER, g940::AMBER); // cockpit2 legacy precedes flightmodel legacy
    refs[parkingBrakeName].value = 0;
    expectBrakeLEDs(g940::GREEN, g940::GREEN);
    XPluginStop();
    refs.erase(parkingBrakeName);
    configureLEDs(brakeConfiguration);
    expectBrakeLEDs(g940::RED, g940::RED);
    refs[legacyBrakeName].value = .5f;
    expectBrakeLEDs(g940::AMBER, g940::AMBER);
    refs[legacyBrakeName].value = 0;
    expectBrakeLEDs(g940::GREEN, g940::GREEN);
    XPluginStop();
    refs.erase(legacyBrakeName);
    refs[brakeTrapName].value = 1;
    refs[brakeValveName].value = 1;
    configureLEDs(brakeConfiguration);
    expectBrakeLEDs(g940::RED, g940::OFF); // valve position does not require master demand
    refs[brakeTrapName].value = 0;
    expectBrakeLEDs(g940::OFF, g940::OFF); // ordinary parking requires a master channel
    XPluginStop();
    refs.emplace(masterBrakeName, Ref{DATA_FLOAT, 0});
    refs[brakeTrapName].value = refs[brakeValveName].value = 0;
    const Ref savedLeftBrake = refs.at(leftBrakeName), savedRightBrake = refs.at(rightBrakeName);
    refs.erase(leftBrakeName);
    configureLEDs(brakeConfiguration);
    expectBrakeLEDs(g940::GREEN, g940::OFF);
    XPluginStop();
    refs.emplace(leftBrakeName, savedLeftBrake);
    refs.erase(rightBrakeName);
    configureLEDs(brakeConfiguration);
    expectBrakeLEDs(g940::GREEN, g940::OFF);
    XPluginStop();
    refs.emplace(rightBrakeName, savedRightBrake);
#endif
    std::filesystem::remove_all(configRoot);
    std::puts("Dataref types, reconnect, pause, and plugin lifecycle passed.");
}

// Compile the real plugin against this host's declarations. On Windows this
// also avoids importing SDK symbols from a DLL in the standalone test runner.
#ifdef TEST_LEDS
#include "../g940LEDs.cpp"
#else
#include "../g940FF.cpp"
#endif
