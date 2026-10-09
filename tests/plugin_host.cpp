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
#include <map>
#include <string>

PLUGIN_API int XPluginStart(char *, char *, char *);
PLUGIN_API int XPluginEnable();
PLUGIN_API void XPluginDisable();
PLUGIN_API void XPluginStop();
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int, void *);

namespace {
enum Type { DATA_INTEGER, DATA_FLOAT, DATA_INTEGERS, DATA_FLOATS, DATA_BYTES };
struct Ref { Type type; float value; std::string text = {}; };
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
    assert(ref(data).type == DATA_FLOATS && offset == 0 && count == 1);
    *out = ref(data).value; return 1;
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
        {"sim/flightmodel/position/alpha", {DATA_FLOAT, 5}},
        {"sim/flightmodel2/controls/elevator_trim", {DATA_FLOAT, .1}},
        {"sim/flightmodel2/controls/aileron_trim", {DATA_FLOAT, 0}},
        {"sim/time/paused", {DATA_INTEGER, 0}}
    };
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
    assert(std::abs(observedForce.pressureRatio - .6) < .001); // Qstatic psf converted to Pa
    assert(std::abs(observedForce.roll) < .001);
    assert(std::abs(observedForce.pitch) < .001);
    assert(std::abs(observedForce.rollForce + .1) < .001);
    assert(std::abs(observedForce.pitchForce + .2) < .001);
    refs["sim/flightmodel/misc/Qstatic"].value = 0;
    refs["sim/flightmodel/position/alpha"].value = -121;
    refs["sim/flightmodel2/controls/elevator_trim"].value = .2;
    for (int i = 0; i < 100; ++i) callback(.02f, 0, 0, nullptr);
    assert(std::abs(observedForce.pressureRatio - g940::minimumForceRatio) < .001);
    assert(std::abs(observedForce.pitch) < .001); // no spurious ground AoA pull
    refs["sim/flightmodel/misc/Qstatic"].value = -1;
    callback(0, 0, 0, nullptr);
    assert(observedForce.pressureRatio == 0);
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
    assert(deviceOpen && observedForce.pressureRatio <= .020001);
    // Resume establishes the current trim center before raising the force,
    // so neither grip mode briefly pulls toward a stale neutral position.
    assert(std::abs(observedForce.pitch - .2f) < .00001f);
    assert(std::abs(observedForce.roll + .2f) < .00001f);
    assert(observedForce.effectScale <= .020001f);
    for (int i = 0; i < 100; ++i) callback(.02f, 0, 0, nullptr);
    refs["sim/aircraft/view/acf_ICAO"].text = "B738";
    refs["sim/aircraft/view/acf_Vne"].value = 340;
    aircraftFile = "Boeing 737.acf";
    XPluginReceiveMessage(XPLM_PLUGIN_XPLANE, XPLM_MSG_PLANE_LOADED, reinterpret_cast<void *>(1));
    callback(.02f, 0, 0, nullptr);
    assert(std::abs(observedForce.pressureRatio - .6f) < .001); // ignore AI plane changes
    XPluginReceiveMessage(XPLM_PLUGIN_XPLANE, XPLM_MSG_PLANE_LOADED, nullptr);
    callback(.02f, 0, 0, nullptr);
    assert(observedForce.effectScale <= .020001f && observedForce.pressureRatio <= .020001f);
    for (int i = 0; i < 100; ++i) callback(.02f, 0, 0, nullptr);
    assert(std::abs(observedForce.pressureRatio - .2540657f) < .001); // General at Vne=340
    assert(debugLog.find("profile 'General' for Boeing 737.acf") != std::string::npos);
    assert(debugLog.find("340.0 kt (X-Plane Vne scaling)") != std::string::npos);
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
    std::ofstream(configFolder / "aircraft.ini") << "[General]\nreference_speed_knots=150\npitch_aoa_gain=0\n";
    assert(XPluginEnable() == 1);
    for (int i = 0; i < 100; ++i) callback(.02f, 0, 0, nullptr);
    assert(std::abs(observedForce.pressureRatio - .4777778f) < .001); // reread config on enable
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
    assert(XPluginStart(name, signature, description) == 1); // optional metadata absent
    assert(XPluginEnable() == 1);
    for (int i = 0; i < 100; ++i) callback(.02f, 0, 0, nullptr);
    assert(std::abs(observedForce.pressureRatio - .6f) < .001); // built-in reference fallback
    assert(debugLog.find("aircraft.ini missing") != std::string::npos);
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
