#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#ifdef G940_DEBUG_FORCE
#include <chrono>
#endif
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
XPLMDataRef rollRef, pitchRef, pressureRef, alphaRef, eTrimRef, aTrimRef, pausedRef;
XPLMDataRef icaoRef, vneRef;
std::filesystem::path configFile;
std::vector<g940::ConfigProfile> profiles = {g940::ConfigProfile{}};
g940::AircraftProfile aircraftProfile;
bool profileDirty = true;
bool enabled = false;
bool forceReady = false;
g940::ForceSmoother forceSmoother;
#ifdef G940_DEBUG_FORCE
std::chrono::steady_clock::time_point nextForceTrace;
#endif

void reportError() {
    const std::string message = std::string("G940 FF: ") + g940::backendError() + "\n";
    XPLMDebugString(message.c_str());
}

bool loadProfiles() {
    try {
        std::ifstream input(configFile);
        if (input) profiles = g940::readAircraftConfig(input);
        else {
            if (std::filesystem::exists(configFile)) throw std::runtime_error("cannot read configuration");
            profiles = {g940::ConfigProfile{}};
            XPLMDebugString("G940 FF: aircraft.ini missing; using General defaults and X-Plane Vne\n");
        }
        return true;
    } catch (const std::exception& error) {
        const auto message = std::string("G940 FF: aircraft.ini: ") + error.what() + "\n";
        XPLMDebugString(message.c_str());
        return false;
    }
}

void selectProfile() {
    char filename[256] = {}, path[512] = {}, icao[41] = {};
    XPLMGetNthAircraftModel(0, filename, path);
    if (icaoRef) {
        const int count = XPLMGetDatab(icaoRef, icao, 0, sizeof(icao) - 1);
        if (count < 0 || count >= static_cast<int>(sizeof(icao))) icao[0] = 0;
        else icao[count] = 0;
    }
    const float vne = vneRef ? XPLMGetDataf(vneRef) : 0.0f;
    const auto& profile = g940::selectAircraftProfile(profiles, icao, filename);
    aircraftProfile = g940::resolveAircraftProfile(profile, vne);
    const bool fromVne = profile.referenceKnots == 0 && std::isfinite(vne) && vne >= 1 && vne <= 1000;
    char message[512];
    std::snprintf(message, sizeof(message), "G940 FF: profile '%s' for %s (ICAO %s), reference %.1f kt (%s)\n",
        profile.name.c_str(), filename, icao, g940::referenceSpeed(profile, vne),
        fromVne ? "X-Plane Vne scaling" : profile.referenceKnots > 0 ? "configuration" : "fallback");
    XPLMDebugString(message);
}

float flightLoopCallback(float elapsed, float, int, void *) {
    if (profileDirty) {
        if (forceReady && !g940::releaseForceFeedback()) {
            reportError(); g940::closeForceFeedback();
        }
        forceReady = false;
        forceSmoother.reset();
        selectProfile();
        profileDirty = false;
    }
    if (!g940::prepareForceFeedback()) {
        reportError(); forceReady = false; forceSmoother.reset(); return 5.0f;
    }
    if (XPLMGetDatai(pausedRef)) {
        if (!forceReady) return 0.2f;
#ifdef G940_DEBUG_FORCE
        const bool startingRelease = !forceSmoother.releasing();
#endif
        const g940::ForceState state = forceSmoother.release(elapsed);
        if (state.pressureRatio <= 0.0f || state.effectScale <= 0.0f) {
            if (!g940::releaseForceFeedback()) {
                reportError(); g940::closeForceFeedback();
            }
            forceReady = false;
            forceSmoother.reset();
#ifdef G940_DEBUG_FORCE
            XPLMDebugString("G940 FF trace: paused force released\n");
#endif
            return 0.2f;
        }
#ifdef G940_DEBUG_FORCE
        if (startingRelease) XPLMDebugString("G940 FF trace: one-second pause fade started\n");
#endif
        if (!g940::updateForceFeedback(state)) {
            reportError(); forceReady = false; forceSmoother.reset(); return 0.2f;
        }
        return 0.02f;
    }
    const bool startingForce = !forceReady;
    if (startingForce) {
        if (!g940::openForceFeedback()) { reportError(); return 5.0f; }
        forceReady = true;
        forceSmoother.reset();
#ifdef G940_DEBUG_FORCE
        nextForceTrace = std::chrono::steady_clock::time_point();
#endif
        XPLMDebugString("G940 FF: force-feedback device connected\n");
    }
    const float roll = XPLMGetDataf(rollRef), pitch = XPLMGetDataf(pitchRef);
    // Qstatic is dynamic pressure in pounds-force per square foot, not Pa.
    const float pressurePa = XPLMGetDataf(pressureRef) * g940::pascalsPerPsf;
    const float alpha = XPLMGetDataf(alphaRef);
    const float elevatorTrim = XPLMGetDataf(eTrimRef), aileronTrim = XPLMGetDataf(aTrimRef);
    const g940::ForceState target = g940::calculateForce(
        roll, pitch, pressurePa, alpha, elevatorTrim, aileronTrim, aircraftProfile);
    if (startingForce) forceSmoother.reset(target);
    const g940::ForceState state = forceSmoother.update(target, elapsed);
#ifdef G940_DEBUG_FORCE
    const auto now = std::chrono::steady_clock::now();
    if (now >= nextForceTrace) {
        char message[384];
        std::snprintf(message, sizeof(message),
            "G940 FF trace: q=%.2f Pa ratio=%.3f "
            "yoke=(%.3f,%.3f) trim=(%.3f,%.3f) alpha=%.2f centers=(%.3f,%.3f) "
            "coeff=(%u,%u) spring=(%.3f,%.3f) scale=%.3f\n",
            pressurePa, state.pressureRatio, roll, pitch, aileronTrim, elevatorTrim,
            alpha, state.roll, state.pitch,
            g940::springCoefficient(state.pressureRatio * state.effectScale, 0),
            g940::springCoefficient(state.pressureRatio * state.effectScale, 1),
            g940::springSaturationRatio(state.pressureRatio, 0) * state.effectScale,
            g940::springSaturationRatio(state.pressureRatio, 1) * state.effectScale,
            state.effectScale);
        XPLMDebugString(message);
        nextForceTrace = now + std::chrono::seconds(2);
    }
#endif
    if (!g940::updateForceFeedback(state)) {
        reportError(); forceReady = false; return 5.0f;
    }
    return 0.02f;
}
}

PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc) {
    std::strcpy(outName, "G940 Force Feedback");
    std::strcpy(outSig, "name.boyle.chris.xpff");
    std::strcpy(outDesc, "Connects X-Plane to force-feedback hardware.");
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);
    char pluginPath[4096] = {};
    XPLMGetPluginInfo(XPLMGetMyID(), nullptr, pluginPath, nullptr, nullptr);
    if (!pluginPath[0]) {
        XPLMDebugString("G940 FF: cannot locate plugin configuration directory\n");
        return 0;
    }
    const auto folder = std::filesystem::path(std::u8string(pluginPath, pluginPath + std::strlen(pluginPath))).parent_path();
    configFile = (folder.filename() == "64" ? folder.parent_path() : folder) / "aircraft.ini";
    struct Reference { XPLMDataRef *target; const char *name; };
    const Reference references[] = {
        {&rollRef, "sim/joystick/yoke_roll_ratio"},
        {&pitchRef, "sim/joystick/yoke_pitch_ratio"},
        {&pressureRef, "sim/flightmodel/misc/Qstatic"},
        {&alphaRef, "sim/flightmodel/position/alpha"},
        {&eTrimRef, "sim/flightmodel2/controls/elevator_trim"},
        {&aTrimRef, "sim/flightmodel2/controls/aileron_trim"},
        {&pausedRef, "sim/time/paused"}
    };
    for (const auto& reference : references) {
        *reference.target = XPLMFindDataRef(reference.name);
        if (!*reference.target) {
            XPLMDebugString("G940 FF: required dataref missing: ");
            XPLMDebugString(reference.name); XPLMDebugString("\n");
            return 0;
        }
    }
    icaoRef = XPLMFindDataRef("sim/aircraft/view/acf_ICAO");
    vneRef = XPLMFindDataRef("sim/aircraft/view/acf_Vne");
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
    if (!g940::closeForceFeedback()) reportError();
    forceReady = false;
    forceSmoother.reset();
}

PLUGIN_API void XPluginStop() { XPluginDisable(); }
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID sender, int message, void *parameter) {
    if (sender == XPLM_PLUGIN_XPLANE && message == XPLM_MSG_PLANE_LOADED && parameter == nullptr)
        profileDirty = true;
}
