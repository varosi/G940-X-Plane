#include <cstdio>
#include <cstring>
#include <string>
#include "XPLMPlugin.h"
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
#include "g940Backend.h"

#ifndef XPLM300
#error This plugin requires the XPLM300 API
#endif

namespace {
XPLMDataRef rollRef, pitchRef, speedRef, vneRef, alphaRef, eTrimRef, aTrimRef, pausedRef;
bool enabled = false;
bool forceReady = false;

void reportError() {
    const std::string message = std::string("G940 FF: ") + g940::backendError() + "\n";
    XPLMDebugString(message.c_str());
}

float flightLoopCallback(float, float, int, void *) {
    if (XPLMGetDatai(pausedRef)) {
        if (forceReady) g940::closeForceFeedback();
        forceReady = false;
        return 0.2f;
    }
    if (!forceReady) {
        if (!g940::openForceFeedback()) { reportError(); return 5.0f; }
        forceReady = true;
        XPLMDebugString("G940 FF: force-feedback device connected\n");
    }
    const g940::ForceState state = g940::calculateForce(
        XPLMGetDataf(rollRef), XPLMGetDataf(pitchRef),
        XPLMGetDataf(speedRef), XPLMGetDataf(vneRef),
        XPLMGetDataf(alphaRef), XPLMGetDataf(eTrimRef), XPLMGetDataf(aTrimRef));
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
    struct Reference { XPLMDataRef *target; const char *name; };
    const Reference references[] = {
        {&rollRef, "sim/joystick/yoke_roll_ratio"},
        {&pitchRef, "sim/joystick/yoke_pitch_ratio"},
        {&speedRef, "sim/flightmodel/position/true_airspeed"},
        {&vneRef, "sim/aircraft/view/acf_Vne"},
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
    return 1;
}

PLUGIN_API int XPluginEnable() {
    if (!enabled) {
        XPLMRegisterFlightLoopCallback(flightLoopCallback, 0.01f, nullptr);
        enabled = true;
    }
    return 1;
}

PLUGIN_API void XPluginDisable() {
    if (enabled) XPLMUnregisterFlightLoopCallback(flightLoopCallback, nullptr);
    enabled = false;
    g940::closeForceFeedback();
    forceReady = false;
}

PLUGIN_API void XPluginStop() { XPluginDisable(); }
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int, void *) {}
