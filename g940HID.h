#ifndef G940_HID_H
#define G940_HID_H
#include <cstddef>
#include <cstdint>
#include <string>

namespace g940 {
// Owns a non-exclusive connection so X-Plane can continue reading controls.
class HIDDevice {
public:
    HIDDevice();
    ~HIDDevice();
    bool open();
    void close();
    bool isOpen() const { return handle_ != nullptr; }
    bool setFeature(const uint8_t *report, size_t length);
    bool getFeature(uint8_t *report, size_t length);
    bool setOutput(const uint8_t *report, size_t length);
    const std::string& error() const { return error_; }
private:
    HIDDevice(const HIDDevice&) = delete;
    HIDDevice& operator=(const HIDDevice&) = delete;
    void *handle_;
#if IBM
    size_t featureLength_;
    size_t outputLength_;
#endif
    std::string error_;
};
}
#endif
