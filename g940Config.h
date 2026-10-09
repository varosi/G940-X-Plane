#ifndef G940_CONFIG_H
#define G940_CONFIG_H

#include "g940ForceModel.h"
#include <istream>
#include <iterator>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace g940 {
struct ConfigProfile {
    std::string name = "General";
    std::vector<std::string> icao, acf;
    // Zero requests X-Plane Vne as a provisional pressure-scaling reference.
    float referenceKnots = 0.0f;
    float fallbackKnots = 125.0f;
    AircraftProfile force;
};

inline std::string trimConfigText(const std::string& text) {
    const auto first = text.find_first_not_of(" \t\r\n");
    return first == std::string::npos ? "" : text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
}
inline std::string lowerConfigText(std::string text) {
    for (char& c : text) if (c >= 'A' && c <= 'Z') c = c - 'A' + 'a';
    return text;
}
inline std::vector<std::string> configList(const std::string& text) {
    std::vector<std::string> result;
    std::istringstream input(text);
    std::string item;
    while (std::getline(input, item, ',')) {
        item = lowerConfigText(trimConfigText(item));
        if (item.empty()) throw std::runtime_error("empty aircraft matcher");
        result.push_back(item);
    }
    if (result.empty() || text.back() == ',') throw std::runtime_error("empty aircraft matcher");
    return result;
}
inline float configNumber(const std::string& text, float low, float high) {
    std::istringstream input(text);
    input.imbue(std::locale::classic());
    float value;
    if (!(input >> value) || !std::isfinite(value) || value < low || value > high)
        throw std::runtime_error("invalid or out-of-range number: " + text);
    input >> std::ws;
    if (!input.eof()) throw std::runtime_error("unexpected text after number: " + text);
    return value;
}

inline std::vector<ConfigProfile> readAircraftConfig(std::istream& input) {
    struct Setting { std::string value; unsigned line; };
    struct Section { std::string name; std::map<std::string, Setting> settings; };
    std::vector<Section> sections;
    std::string text;
    unsigned line = 0;
    while (std::getline(input, text)) {
        ++line;
        if (line == 1 && text.starts_with("\xef\xbb\xbf")) text.erase(0, 3);
        text = trimConfigText(text.substr(0, text.find_first_of("#;")));
        if (text.empty()) continue;
        const auto fail = [line](const std::string& reason) {
            throw std::runtime_error("line " + std::to_string(line) + ": " + reason);
        };
        if (text.front() == '[') {
            if (text.back() != ']') fail("unfinished profile section");
            const auto name = trimConfigText(text.substr(1, text.size() - 2));
            if (name.empty()) fail("empty profile name");
            for (const auto& section : sections)
                if (lowerConfigText(section.name) == lowerConfigText(name)) fail("duplicate profile: " + name);
            sections.push_back({name, {}});
        } else {
            const auto equals = text.find('=');
            if (sections.empty() || equals == std::string::npos) fail("expected a profile and key=value");
            const auto key = lowerConfigText(trimConfigText(text.substr(0, equals)));
            const auto value = trimConfigText(text.substr(equals + 1));
            if (key.empty() || value.empty()) fail("empty key or value");
            if (!sections.back().settings.emplace(key, Setting{value, line}).second) fail("duplicate key: " + key);
        }
    }
    if (input.bad()) throw std::runtime_error("configuration read failed");
    const auto general = std::find_if(sections.begin(), sections.end(), [](const auto& section) {
        return lowerConfigText(section.name) == "general";
    });
    if (general == sections.end()) throw std::runtime_error("missing [General] profile");
    const auto apply = [](const Section& section, ConfigProfile profile) {
        profile.name = section.name;
        struct Field { const char *key; float AircraftProfile::*member; float low, high; };
        const Field fields[] = {
            {"mechanical_ratio", &AircraftProfile::mechanicalRatio, .001f, 1.0f},
            {"roll_trim_gain", &AircraftProfile::rollTrimGain, -10.0f, 10.0f},
            {"pitch_trim_gain", &AircraftProfile::pitchTrimGain, -10.0f, 10.0f},
            {"pitch_aoa_gain", &AircraftProfile::pitchAoAGain, -1.0f, 1.0f},
            {"neutral_aoa_degrees", &AircraftProfile::neutralAoADegrees, -90.0f, 90.0f}
        };
        for (const auto& entry : section.settings) {
            const auto& key = entry.first;
            const auto& setting = entry.second;
            try {
                const auto& value = setting.value;
                if (key == "match_icao") profile.icao = configList(value);
                else if (key == "match_acf") profile.acf = configList(value);
                else if (key == "reference_speed_knots")
                    profile.referenceKnots = lowerConfigText(value) == "auto" ? 0.0f : configNumber(value, 1, 1000);
                else if (key == "fallback_reference_speed_knots") profile.fallbackKnots = configNumber(value, 1, 1000);
                else {
                    const auto field = std::find_if(std::begin(fields), std::end(fields), [&](const auto& f) { return key == f.key; });
                    if (field == std::end(fields)) throw std::runtime_error("unknown key: " + key);
                    profile.force.*(field->member) = configNumber(value, field->low, field->high);
                }
            } catch (const std::runtime_error& error) {
                throw std::runtime_error("line " + std::to_string(setting.line) + " [" + section.name + "]: " + error.what());
            }
        }
        return profile;
    };
    std::vector<ConfigProfile> result = {apply(*general, ConfigProfile{})};
    if (!result.front().icao.empty() || !result.front().acf.empty())
        throw std::runtime_error("[General] is the fallback and cannot have aircraft matchers");
    for (const auto& section : sections) {
        if (&section == &*general) continue;
        auto profile = apply(section, result.front());
        if (profile.icao.empty() && profile.acf.empty())
            throw std::runtime_error("[" + section.name + "] needs match_icao or match_acf");
        result.push_back(profile);
    }
    return result;
}

inline bool configGlob(const std::string& pattern, const std::string& name) {
    size_t p = 0, n = 0, star = std::string::npos, retry = 0;
    while (n < name.size()) {
        if (p < pattern.size() && (pattern[p] == '?' || pattern[p] == name[n])) { ++p; ++n; }
        else if (p < pattern.size() && pattern[p] == '*') { star = p++; retry = n; }
        else if (star != std::string::npos) { p = star + 1; n = ++retry; }
        else return false;
    }
    while (p < pattern.size() && pattern[p] == '*') ++p;
    return p == pattern.size();
}
inline const ConfigProfile& selectAircraftProfile(const std::vector<ConfigProfile>& profiles,
                                                 std::string icao, std::string filename) {
    icao = lowerConfigText(trimConfigText(icao));
    filename = lowerConfigText(filename);
    // ICAO is preferred globally; filename matching handles missing/custom codes.
    for (const auto& profile : profiles)
        if (!icao.empty() && std::find(profile.icao.begin(), profile.icao.end(), icao) != profile.icao.end()) return profile;
    for (const auto& profile : profiles)
        for (const auto& pattern : profile.acf)
            if (configGlob(pattern, filename)) return profile;
    return profiles.front();
}
inline float referenceSpeed(const ConfigProfile& profile, float vneKnots) {
    if (profile.referenceKnots > 0) return profile.referenceKnots;
    return std::isfinite(vneKnots) && vneKnots >= 1 && vneKnots <= 1000 ? vneKnots : profile.fallbackKnots;
}
inline AircraftProfile resolveAircraftProfile(const ConfigProfile& profile, float vneKnots) {
    auto force = profile.force;
    const float speed = referenceSpeed(profile, vneKnots) * knotsToMps;
    force.referencePressurePa = 0.5f * seaLevelDensity * speed * speed;
    return force;
}
}
#endif
