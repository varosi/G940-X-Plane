#include "g940Config.h"
#include <cassert>
#include <cstdio>
#include <fstream>
#include <limits>

int main() {
    using namespace g940;
    std::ifstream input("aircraft.ini");
    assert(input);
    const auto profiles = readAircraftConfig(input);
    assert(profiles.size() == 2 && profiles.front().name == "General");
    const auto& general = selectAircraftProfile(profiles, "B738", "Boeing 737.acf");
    assert(general.name == "General");
    assert(general.referenceKnots == 0 && general.force.rollTrimGain == 1 && general.force.pitchTrimGain == 1);
    assert(referenceSpeed(general, 340) == 340);
    assert(resolveAircraftProfile(general, 340).referencePressurePa > 18000);
    for (float invalidVne : {0.0f, -1.0f, 1001.0f, std::numeric_limits<float>::infinity(),
                            std::numeric_limits<float>::quiet_NaN()}) {
        assert(referenceSpeed(general, invalidVne) == 125);
        assert(std::abs(resolveAircraftProfile(general, invalidVne).referencePressurePa - 2532.813f) < .01f);
    }
    const auto& tb10 = selectAircraftProfile(profiles, " toba ", "renamed.acf");
    const auto& tb20 = selectAircraftProfile(profiles, "TRIN", "renamed.acf");
    assert(tb10.name == "Socata TB10/TB20" && &tb10 == &tb20);
    assert(&selectAircraftProfile(profiles, "CUSTOM", "JF_Socata_TB10+TB20.acf") == &tb10);
    assert(&selectAircraftProfile(profiles, "", "socata tb-20.ACF") == &tb10);
    assert(referenceSpeed(tb10, 187) == 125); // explicit calibration beats Vne
    const auto tbForce = resolveAircraftProfile(tb10, 187);
    assert(tbForce.rollTrimGain == 3 && tbForce.pitchTrimGain == 1.5f);
    const auto trimmed = calculateForce(0, 0, tbForce.referencePressurePa / 2, 0, .5f, .1f, tbForce);
    assert(std::abs(trimmed.pitch - .5f) < 1e-6 && std::abs(trimmed.roll - .2f) < 1e-6);

    // General can appear last; aircraft presets inherit its final values.
    std::istringstream custom("\xef\xbb\xbf[Jet]\r\nmatch_icao=B738\r\nreference_speed_knots=250\r\n"
        "pitch_aoa_gain=0 # artificial feel tuning\r\n[Filename]\nmatch_acf=*.acf\n"
        "[General]\nmechanical_ratio=.3\nroll_trim_gain=2\nfallback_reference_speed_knots=145\n");
    const auto inherited = readAircraftConfig(custom);
    const auto& jet = selectAircraftProfile(inherited, "b738", "unknown.acf");
    assert(jet.name == "Jet"); // ICAO takes precedence over filename matchers
    assert(jet.force.mechanicalRatio == .3f && jet.force.rollTrimGain == 2);
    assert(jet.force.pitchAoADeflectionGain == 0 && referenceSpeed(jet, 340) == 250);
    const auto jetForce = resolveAircraftProfile(jet, 340);
    assert(calculateForce(0, 0, 1000, 10, 0, 0, jetForce).pitch == 0);
    assert(referenceSpeed(inherited.front(), 0) == 145);

    const AircraftGeometry geometry{25, 12, .1f, 10, 4};
    const auto automatic = resolveAircraftProfile(general, 340, geometry);
    assert(automatic.elevatorUpDegrees == 25 && automatic.elevatorDownDegrees == 12);
    assert(automatic.staticPitchTrim == .1f && automatic.pitchTrimMode == PitchTrimMode::stabilizer);
    assert(resolveAircraftProfile(tb10, 187, geometry).pitchTrimMode == PitchTrimMode::aerodynamic);
    std::istringstream overrideInput("[General]\nelevator_up_degrees=20\nelevator_down_degrees=10\n"
        "static_pitch_trim=-.05\npitch_trim_mode=spring\npitch_aoa_deflection_gain=.6\n");
    const auto overrides = readAircraftConfig(overrideInput);
    const auto resolved = resolveAircraftProfile(overrides.front(), 187, geometry);
    assert(resolved.elevatorUpDegrees == 20 && resolved.elevatorDownDegrees == 10);
    assert(resolved.staticPitchTrim == -.05f && resolved.pitchTrimMode == PitchTrimMode::spring);
    assert(resolved.pitchAoADeflectionGain == .6f);
    for (float bad : {0.0f, -1.0f, 100.0f, std::numeric_limits<float>::quiet_NaN()}) {
        const auto fallback = resolveAircraftProfile(general, 0, {bad, bad, std::isfinite(bad) ? 2.0f : bad, bad, bad});
        assert(fallback.elevatorUpDegrees == 15 && fallback.elevatorDownDegrees == 15);
        assert(fallback.staticPitchTrim == 0 && fallback.pitchTrimMode == PitchTrimMode::aerodynamic);
    }
    std::istringstream legacyInput("[General]\npitch_aoa_gain=.03\n");
    assert(std::abs(readAircraftConfig(legacyInput).front().force.pitchAoADeflectionGain - .45f) < 1e-6f);
    std::istringstream autoInput("[General]\nelevator_up_degrees=20\npitch_trim_mode=spring\n"
        "[Reset]\nmatch_icao=RESET\nelevator_up_degrees=auto\npitch_trim_mode=auto\n");
    const auto resetProfiles = readAircraftConfig(autoInput);
    const auto reset = resolveAircraftProfile(resetProfiles.back(), 187, geometry);
    assert(reset.elevatorUpDegrees == 25 && reset.pitchTrimMode == PitchTrimMode::stabilizer);

    for (const char *bad : {
        "", "reference_speed_knots=125", "[General", "[]", "[General]\n[general]",
        "[General]\nmechanical_ratio=.2\nmechanical_ratio=.3", "[General]\nunknown=1",
        "[General]\nreference_speed_knots=0", "[General]\nreference_speed_knots=125abc",
        "[General]\nreference_speed_knots=nan", "[General]\nreference_speed_knots=inf",
        "[General]\nreference_speed_knots=1e100", "[General]\nmechanical_ratio=1.1",
        "[General]\npitch_aoa_gain=2", "[General]\nmatch_icao=TOBA",
        "[General]\n[Aircraft]\npitch_trim_gain=2", "[General]\n[Aircraft]\nmatch_icao=TOBA,",
        "[General]\n[Aircraft]\nmatch_acf=,*.acf",
        "[General]\nelevator_up_degrees=0", "[General]\nelevator_down_degrees=91",
        "[General]\nstatic_pitch_trim=1.1", "[General]\npitch_trim_mode=jet",
        "[General]\npitch_aoa_deflection_gain=16",
        "[General]\npitch_aoa_deflection_gain=.45\npitch_aoa_gain=.03"
    }) {
        std::istringstream invalid(bad);
        bool rejected = false;
        try { readAircraftConfig(invalid); }
        catch (const std::runtime_error&) { rejected = true; }
        assert(rejected);
    }
    std::istringstream typo("[General]\npitch_trim_gian=2");
    try { readAircraftConfig(typo); assert(false); }
    catch (const std::runtime_error& error) {
        assert(std::string(error.what()).find("line 2 [General]: unknown key") != std::string::npos);
    }
    assert(configGlob("*tb??*.acf", "jf_tb20.acf"));
    assert(!configGlob("*tb20*.acf", "tb10.acf"));
    assert(!configGlob("*tb20*.acf", "tb20.acf.backup"));
    std::puts("Aircraft config, selection, inheritance, simulator references and invalid settings passed.");
}
