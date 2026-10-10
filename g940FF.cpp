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
#include "g940GroundModel.h"

#ifndef XPLM300
#error This plugin requires the XPLM300 API
#endif

namespace {
XPLMDataRef rollRef, pitchRef, pressureRef, alphaRef, eTrimRef, aTrimRef, pausedRef;
XPLMDataRef icaoRef, vneRef;
XPLMDataRef elevatorUpRef, elevatorDownRef, staticPitchTrimRef, stabilizerUpRef, stabilizerDownRef, stabilizerRef;
XPLMDataRef windRefs[3], attitudeRefs[3], stalledRef, wingAreaRef, powerRef, maximumPowerRef;
XPLMDataRef onGroundRef, replayRef, crashedRef;
XPLMDataRef engineCountRef, legacyMaximumPowerRef;
XPLMDataRef gearForceRef, massRef, rollAccelerationRef, pitchAccelerationRef, groundSpeedRef;
XPLMDataRef positionRefs[3], verticalSpeedRef;
std::filesystem::path configFile;
std::vector<g940::ConfigProfile> profiles = {g940::ConfigProfile{}};
g940::AircraftProfile aircraftProfile;
bool profileDirty = true;
bool enabled = false;
bool forceReady = false;
bool cuesDirty = false;
g940::ForceSmoother forceSmoother;
g940::FlightCues flightCues;
g940::GroundCues groundCues;
g940::CueMixer cueMixer;
#ifdef G940_DEBUG_FORCE
std::chrono::steady_clock::time_point nextForceTrace;
#endif

void resetCues() {
    flightCues.reset();
    groundCues.reset();
    cueMixer.reset();
    cuesDirty = false;
}

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
    const auto optionalFloat = [](XPLMDataRef reference) { return reference ? XPLMGetDataf(reference) : 0.0f; };
    const g940::AircraftGeometry geometry{optionalFloat(elevatorUpRef), optionalFloat(elevatorDownRef),
        optionalFloat(staticPitchTrimRef), optionalFloat(stabilizerUpRef), optionalFloat(stabilizerDownRef)};
    aircraftProfile = g940::resolveAircraftProfile(profile, vne, geometry);
    const bool fromVne = profile.referenceKnots == 0 && std::isfinite(vne) && vne >= 1 && vne <= 1000;
    char message[512];
    std::snprintf(message, sizeof(message), "G940 FF: profile '%s' for %s (ICAO %s), reference %.1f kt (%s)\n",
        profile.name.c_str(), filename, icao, g940::referenceSpeed(profile, vne),
        fromVne ? "X-Plane Vne scaling" : profile.referenceKnots > 0 ? "configuration" : "fallback");
    XPLMDebugString(message);
    const char *mode = aircraftProfile.pitchTrimMode == g940::PitchTrimMode::spring ? "spring" :
        aircraftProfile.pitchTrimMode == g940::PitchTrimMode::stabilizer ? "stabilizer" : "aerodynamic";
    std::snprintf(message, sizeof(message), "G940 FF: pitch trim %s, elevator travel +%.1f/-%.1f deg, static tab %.3f\n",
        mode, aircraftProfile.elevatorUpDegrees, aircraftProfile.elevatorDownDegrees, aircraftProfile.staticPitchTrim);
    XPLMDebugString(message);
}

g940::CueSample readCueSample(float pressurePa) {
    g940::CueSample sample;
    sample.pressurePa = pressurePa;
    sample.airborne = onGroundRef && !XPLMGetDatai(onGroundRef) &&
        (!replayRef || !XPLMGetDatai(replayRef)) && (!crashedRef || !XPLMGetDatai(crashedRef));
    if (!sample.airborne) return sample;
    sample.windValid = true;
    float *attitude[] = {&sample.heading, &sample.pitch, &sample.roll};
    for (unsigned axis = 0; axis < 3; ++axis) {
        sample.windValid &= windRefs[axis] && attitudeRefs[axis];
        if (windRefs[axis]) sample.wind[axis] = XPLMGetDataf(windRefs[axis]);
        if (attitudeRefs[axis]) *attitude[axis] = XPLMGetDataf(attitudeRefs[axis]);
    }
    sample.stalledFraction = std::numeric_limits<float>::quiet_NaN();
    if (stalledRef && wingAreaRef && aircraftProfile.stallBuffetGain > 0.0f) {
        std::array<float, g940::mainWingElements> stalled{}, area{};
        // Main-wing surfaces 0..7; tail surfaces 8/9 must not trigger buffet.
        if (XPLMGetDatavf(stalledRef, stalled.data(), 0, stalled.size()) == static_cast<int>(stalled.size()) &&
            XPLMGetDatavf(wingAreaRef, area.data(), 0, area.size()) == static_cast<int>(area.size()))
            sample.stalledFraction = g940::stalledWingFraction(stalled, area);
    }
    const int engines = engineCountRef ? XPLMGetDatai(engineCountRef) : 1;
    if (powerRef && engines > 0 && engines <= 16 && aircraftProfile.stallBuffetGain > 0.0f) {
        std::array<float, 16> power{}, maximum{};
        bool haveMaximum = maximumPowerRef && XPLMGetDatavf(maximumPowerRef, maximum.data(), 0, engines) == engines;
        if (!haveMaximum && legacyMaximumPowerRef) {
            const float maximumPower = XPLMGetDataf(legacyMaximumPowerRef);
            haveMaximum = std::isfinite(maximumPower) && maximumPower > 0.0f;
            std::fill_n(maximum.begin(), engines, maximumPower);
        }
        if (haveMaximum && XPLMGetDatavf(powerRef, power.data(), 0, engines) == engines) {
            float total = 0.0f, limit = 0.0f;
            bool valid = true;
            for (int engine = 0; engine < engines; ++engine) {
                valid &= std::isfinite(power[engine]) && std::isfinite(maximum[engine]) && maximum[engine] >= 0.0f;
                if (maximum[engine] > 0.0f) { total += std::max(0.0f, power[engine]); limit += maximum[engine]; }
            }
            if (valid && std::isfinite(total) && std::isfinite(limit) && limit > 0.0f)
                sample.enginePower = g940::clamp(total / limit, 0.0f, 1.0f);
        }
    }
    return sample;
}

g940::GroundSample readGroundSample() {
    g940::GroundSample sample;
    sample.contactValid = onGroundRef && (!replayRef || !XPLMGetDatai(replayRef)) &&
        (!crashedRef || !XPLMGetDatai(crashedRef));
    if (!sample.contactValid) return sample;
    sample.onGround = XPLMGetDatai(onGroundRef) != 0;
    const auto optionalFloat = [](XPLMDataRef ref) {
        return ref ? XPLMGetDataf(ref) : std::numeric_limits<float>::quiet_NaN();
    };
    const float force = optionalFloat(gearForceRef), mass = optionalFloat(massRef);
    if (std::isfinite(force) && std::isfinite(mass) && mass > 0.0f)
        sample.supportG = (force / mass) / 9.80665f;
    sample.rollAcceleration = optionalFloat(rollAccelerationRef);
    sample.pitchAcceleration = optionalFloat(pitchAccelerationRef);
    sample.groundSpeed = optionalFloat(groundSpeedRef);
    sample.verticalSpeed = optionalFloat(verticalSpeedRef);
    sample.positionValid = positionRefs[0] && positionRefs[1] && positionRefs[2];
    if (sample.positionValid)
        for (unsigned axis = 0; axis < 3; ++axis) sample.position[axis] = XPLMGetDatad(positionRefs[axis]);
    return sample;
}

float flightLoopCallback(float elapsed, float, int, void *) {
    if (profileDirty) {
        if (forceReady && !g940::releaseForceFeedback()) {
            reportError(); g940::closeForceFeedback();
        }
        forceReady = false;
        forceSmoother.reset();
        resetCues();
        selectProfile();
        profileDirty = false;
    }
    if (cuesDirty) resetCues();
    if (!g940::prepareForceFeedback()) {
        reportError(); forceReady = false; forceSmoother.reset(); resetCues(); return 5.0f;
    }
    if (XPLMGetDatai(pausedRef)) {
        if (!forceReady) return 0.2f;
#ifdef G940_DEBUG_FORCE
        const bool startingRelease = !forceSmoother.releasing();
#endif
        const g940::ForceState state = g940::withCues(forceSmoother.release(elapsed),
            cueMixer.update(flightCues.release(elapsed), groundCues.release(elapsed), elapsed));
        if (!state.hasLoad() || state.effectScale <= 0.0f) {
            if (!g940::releaseForceFeedback()) {
                reportError(); g940::closeForceFeedback();
            }
            forceReady = false;
            forceSmoother.reset();
            resetCues();
#ifdef G940_DEBUG_FORCE
            XPLMDebugString("G940 FF trace: paused force released\n");
#endif
            return 0.2f;
        }
#ifdef G940_DEBUG_FORCE
        if (startingRelease) XPLMDebugString("G940 FF trace: one-second pause fade started\n");
#endif
        if (!g940::updateForceFeedback(state)) {
            reportError(); forceReady = false; forceSmoother.reset(); resetCues(); return 0.2f;
        }
        return 0.02f;
    }
    const bool startingForce = !forceReady;
    if (startingForce) {
        if (!g940::openForceFeedback()) { reportError(); return 5.0f; }
        forceReady = true;
        forceSmoother.reset();
        resetCues();
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
    const float stabilizer = aircraftProfile.pitchTrimMode == g940::PitchTrimMode::stabilizer && stabilizerRef ?
        XPLMGetDataf(stabilizerRef) : 0.0f;
    const g940::ForceState target = g940::calculateForce(
        roll, pitch, pressurePa, alpha, elevatorTrim, aileronTrim, aircraftProfile, stabilizer);
    if (startingForce) forceSmoother.reset(target);
    g940::ForceState state = forceSmoother.update(target, elapsed);
    if (target.hasLoad()) state = g940::withCues(state, cueMixer.update(
        flightCues.update(readCueSample(pressurePa), aircraftProfile, elapsed),
        groundCues.update(readGroundSample(), aircraftProfile, elapsed), elapsed));
    else resetCues();
#ifdef G940_DEBUG_FORCE
    const auto now = std::chrono::steady_clock::now();
    if (now >= nextForceTrace) {
        char message[512];
        std::snprintf(message, sizeof(message),
            "G940 FF trace: q=%.2f Pa ratio=%.3f mechanical=%.3f aerodynamic=%.3f damping=%.3f "
            "yoke=(%.3f,%.3f) trim=(%.3f,%.3f) alpha=%.2f stab=%.2f centers=(%.3f,%.3f) "
            "coeff=(%u,%u) spring=(%.3f,%.3f) scale=%.3f cues=(%.4f,%.4f)\n",
            pressurePa, state.springRatio(), state.mechanicalRatio, state.aerodynamicRatio, state.dampingRatio,
            roll, pitch, aileronTrim, elevatorTrim,
            alpha, stabilizer, state.roll, state.pitch,
            g940::springCoefficient(state.springRatio() * state.effectScale, 0),
            g940::springCoefficient(state.springRatio() * state.effectScale, 1),
            g940::springSaturationRatio(state.springRatio(), 0) * state.effectScale,
            g940::springSaturationRatio(state.springRatio(), 1) * state.effectScale,
            state.effectScale, state.rollCue, state.pitchCue);
        XPLMDebugString(message);
        nextForceTrace = now + std::chrono::seconds(2);
    }
#endif
    if (!g940::updateForceFeedback(state)) {
        reportError(); forceReady = false; resetCues(); return 5.0f;
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
    elevatorUpRef = XPLMFindDataRef("sim/aircraft/controls/acf_elev_up");
    elevatorDownRef = XPLMFindDataRef("sim/aircraft/controls/acf_elev_dn");
    staticPitchTrimRef = XPLMFindDataRef("sim/aircraft/controls/acf_elev_tab");
    stabilizerUpRef = XPLMFindDataRef("sim/aircraft/controls/acf_hstb_trim_up");
    stabilizerDownRef = XPLMFindDataRef("sim/aircraft/controls/acf_hstb_trim_dn");
    stabilizerRef = XPLMFindDataRef("sim/flightmodel2/controls/stabilizer_deflection_degrees");
    const char *windNames[] = {"sim/weather/aircraft/wind_now_x_msc", "sim/weather/aircraft/wind_now_y_msc", "sim/weather/aircraft/wind_now_z_msc"};
    const char *attitudeNames[] = {"sim/flightmodel/position/psi", "sim/flightmodel/position/theta", "sim/flightmodel/position/phi"};
    for (unsigned axis = 0; axis < 3; ++axis) {
        windRefs[axis] = XPLMFindDataRef(windNames[axis]);
        attitudeRefs[axis] = XPLMFindDataRef(attitudeNames[axis]);
    }
    stalledRef = XPLMFindDataRef("sim/flightmodel2/wing/elements/element_is_stalled");
    wingAreaRef = XPLMFindDataRef("sim/flightmodel2/wing/elements/element_surface_area_mtr_sq");
    powerRef = XPLMFindDataRef("sim/cockpit2/engine/indicators/power_watts");
    maximumPowerRef = XPLMFindDataRef("sim/aircraft/engine/acf_pmax_per_engine");
    legacyMaximumPowerRef = XPLMFindDataRef("sim/aircraft/engine/acf_pmax");
    engineCountRef = XPLMFindDataRef("sim/aircraft/engine/acf_num_engines");
    onGroundRef = XPLMFindDataRef("sim/flightmodel/failures/onground_any");
    replayRef = XPLMFindDataRef("sim/time/is_in_replay");
    crashedRef = XPLMFindDataRef("sim/flightmodel2/misc/has_crashed");
    gearForceRef = XPLMFindDataRef("sim/flightmodel/forces/fnrml_gear");
    massRef = XPLMFindDataRef("sim/flightmodel/weight/m_total");
    rollAccelerationRef = XPLMFindDataRef("sim/flightmodel/position/P_dot");
    pitchAccelerationRef = XPLMFindDataRef("sim/flightmodel/position/Q_dot");
    groundSpeedRef = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
    const char *positionNames[] = {"sim/flightmodel/position/local_x", "sim/flightmodel/position/local_y", "sim/flightmodel/position/local_z"};
    for (unsigned axis = 0; axis < 3; ++axis) positionRefs[axis] = XPLMFindDataRef(positionNames[axis]);
    verticalSpeedRef = XPLMFindDataRef("sim/flightmodel/position/local_vy");
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
    resetCues();
}

PLUGIN_API void XPluginStop() { XPluginDisable(); }
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID sender, int message, void *parameter) {
    if (sender == XPLM_PLUGIN_XPLANE && message == XPLM_MSG_PLANE_LOADED && parameter == nullptr)
        profileDirty = true;
    if (sender == XPLM_PLUGIN_XPLANE && message == XPLM_MSG_AIRPORT_LOADED)
        cuesDirty = true;
}
