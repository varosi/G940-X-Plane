// Exercise the real native backend without USB access or motor movement.
#include "g940Backend.h"
#include "g940HID.h"
#include <cassert>
#include <cstdio>
#include <vector>

namespace {
bool failRead = false, failOutput = false;
int connections = 0;
std::vector<std::vector<uint8_t>> features, outputs;
}

namespace g940 {
HIDDevice::HIDDevice() : handle_(nullptr)
#if IBM
    , featureLength_(0), outputLength_(0), inputLength_(0), writeEvent_(nullptr)
#endif
{}
HIDDevice::~HIDDevice() { close(); }
bool HIDDevice::open() {
    if (!isOpen()) { handle_ = this; ++connections; }
    return true;
}
void HIDDevice::close() { if (isOpen()) --connections; handle_ = nullptr; }
bool HIDDevice::getFeature(uint8_t *report, size_t length) {
    assert(isOpen() && length == 3 && report[0] == 3);
    if (failRead) { error_ = "simulated feature read failure"; return false; }
    report[1] = 0; report[2] = 0xff;
    return true;
}
bool HIDDevice::setFeature(const uint8_t *report, size_t length) {
    assert(isOpen()); features.emplace_back(report, report + length); return true;
}
bool HIDDevice::setOutput(const uint8_t *report, size_t length) {
    assert(isOpen()); outputs.emplace_back(report, report + length);
    if (failOutput) { failOutput = false; error_ = "simulated output failure"; return false; }
    return true;
}
}

int main() {
    using namespace g940;
    assert(openForceFeedback() && connections == 1);
    const auto stop = stopReport();
    assert(outputs.back() == std::vector<uint8_t>(stop.begin(), stop.end()));
    assert(updateForceFeedback({.2, -.3, .5}));
    assert(outputs.back()[7] != 0);
    assert(updateForceFeedback({.2, -.3, .5, 0, 0, 0}));
    assert(outputs.back() == std::vector<uint8_t>(stop.begin(), stop.end()));
    failOutput = true;
    assert(!updateForceFeedback({.2, -.3, .5}));
    assert(connections == 0);
    assert(outputs.back() == std::vector<uint8_t>(stop.begin(), stop.end()));
    assert(std::string(backendError()) == "simulated output failure");

    failRead = true;
    assert(!openLEDs() && connections == 0);
    failRead = false;
    assert(openLEDs() && connections == 1);
    LEDState colours;
    colours.fill(RED);
    assert(updateLEDs(colours));
    assert(updateLEDs(colours)); // unchanged state still checks the USB device
    assert(features.size() == 2);
    closeLEDs();
    assert(connections == 0);
    assert((features.back() == std::vector<uint8_t>{3, 0, 0xff}));
    std::puts("Native HID refresh, restore, and force-error cleanup passed.");
}
