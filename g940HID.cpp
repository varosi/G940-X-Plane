#include "g940HID.h"
#include <algorithm>
#include <array>
#include <cstdio>
#include <vector>

#if APL
#include <IOKit/hid/IOHIDManager.h>
#elif IBM
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <setupapi.h>
#include <hidsdi.h>
#include <hidpi.h>
#endif

namespace g940 {
namespace {
#if APL || IBM
const unsigned vendorID = 0x046d;
const unsigned productID = 0xc287;
std::string deviceError(const char *operation, unsigned code) {
    char message[160];
    std::snprintf(message, sizeof(message), "%s failed (0x%08x)", operation, code);
    return message;
}
#endif
}

HIDDevice::HIDDevice() : handle_(nullptr)
#if IBM
    , featureLength_(0), outputLength_(0), inputLength_(0), writeEvent_(nullptr)
#endif
{}
HIDDevice::~HIDDevice() { close(); }

bool HIDDevice::open() {
    if (isOpen()) return true;
    error_ = "Logitech G940 (046d:c287) not found";
#if APL
    IOHIDManagerRef manager = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone);
    if (!manager) { error_ = "Cannot create HID manager"; return false; }
    int vendor = vendorID, product = productID;
    CFNumberRef vendorNumber = CFNumberCreate(nullptr, kCFNumberIntType, &vendor);
    CFNumberRef productNumber = CFNumberCreate(nullptr, kCFNumberIntType, &product);
    const void *keys[] = {CFSTR(kIOHIDVendorIDKey), CFSTR(kIOHIDProductIDKey)};
    const void *values[] = {vendorNumber, productNumber};
    CFDictionaryRef match = CFDictionaryCreate(nullptr, keys, values, 2,
        &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    IOHIDManagerSetDeviceMatching(manager, match);
    CFSetRef devices = IOHIDManagerCopyDevices(manager);
    if (devices) {
        std::vector<const void *> list(CFSetGetCount(devices));
        CFSetGetValues(devices, list.data());
        for (const void *entry : list) {
            IOHIDDeviceRef device = static_cast<IOHIDDeviceRef>(const_cast<void *>(entry));
            const IOReturn result = IOHIDDeviceOpen(device, kIOHIDOptionsTypeNone);
            if (result == kIOReturnSuccess) {
                handle_ = const_cast<void *>(CFRetain(device));
                break;
            }
            error_ = deviceError("IOHIDDeviceOpen", result);
        }
        CFRelease(devices);
    }
    CFRelease(match); CFRelease(vendorNumber); CFRelease(productNumber);
    CFRelease(manager);
#elif IBM
    GUID hidGUID;
    HidD_GetHidGuid(&hidGUID);
    HDEVINFO devices = SetupDiGetClassDevsW(&hidGUID, nullptr, nullptr,
                                         DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (devices == INVALID_HANDLE_VALUE) {
        error_ = deviceError("SetupDiGetClassDevs", GetLastError());
        return false;
    }
    SP_DEVICE_INTERFACE_DATA info = {};
    info.cbSize = sizeof(info);
    for (DWORD index = 0; SetupDiEnumDeviceInterfaces(devices, nullptr, &hidGUID, index, &info); ++index) {
        DWORD size = 0;
        SetupDiGetDeviceInterfaceDetailW(devices, &info, nullptr, 0, &size, nullptr);
        if (size < sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W)) continue;
        std::vector<uint8_t> storage(size);
        auto detail = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W *>(storage.data());
        detail->cbSize = sizeof(*detail);
        if (!SetupDiGetDeviceInterfaceDetailW(devices, &info, detail, size, nullptr, nullptr)) continue;
        // Inspect collections without taking input access from X-Plane.
        HANDLE device = CreateFileW(detail->DevicePath, 0, FILE_SHARE_READ | FILE_SHARE_WRITE,
                                    nullptr, OPEN_EXISTING, 0, nullptr);
        if (device == INVALID_HANDLE_VALUE) continue;
        HIDD_ATTRIBUTES attributes = {};
        attributes.Size = sizeof(attributes);
        PHIDP_PREPARSED_DATA parsed = nullptr;
        HIDP_CAPS caps = {};
        if (HidD_GetAttributes(device, &attributes) && attributes.VendorID == vendorID &&
            attributes.ProductID == productID && HidD_GetPreparsedData(device, &parsed)) {
            const NTSTATUS result = HidP_GetCaps(parsed, &caps);
            HidD_FreePreparsedData(parsed);
            if (result == HIDP_STATUS_SUCCESS && caps.OutputReportByteLength >= 64 &&
                caps.FeatureReportByteLength >= 5) {
                CloseHandle(device);
                device = CreateFileW(detail->DevicePath, GENERIC_WRITE,
                    FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING,
                    FILE_FLAG_OVERLAPPED, nullptr);
                if (device == INVALID_HANDLE_VALUE) {
                    error_ = deviceError("Open G940 for output", GetLastError());
                    continue;
                }
                HANDLE event = CreateEventW(nullptr, TRUE, FALSE, nullptr);
                if (event) {
                    handle_ = device;
                    writeEvent_ = event;
                    featureLength_ = caps.FeatureReportByteLength;
                    outputLength_ = caps.OutputReportByteLength;
                    inputLength_ = caps.InputReportByteLength;
                    break;
                }
                error_ = deviceError("Create output event", GetLastError());
            }
        }
        CloseHandle(device);
    }
    SetupDiDestroyDeviceInfoList(devices);
#else
    error_ = "Direct HID is only used on macOS and Windows; Linux uses evdev/sysfs";
#endif
    if (isOpen()) error_.clear();
    return isOpen();
}

void HIDDevice::close() {
    if (!isOpen()) return;
#if APL
    auto device = static_cast<IOHIDDeviceRef>(handle_);
    IOHIDDeviceClose(device, kIOHIDOptionsTypeNone);
    CFRelease(device);
#elif IBM
    CloseHandle(static_cast<HANDLE>(handle_));
    CloseHandle(static_cast<HANDLE>(writeEvent_));
    writeEvent_ = nullptr;
#endif
    handle_ = nullptr;
}

bool HIDDevice::setFeature(const uint8_t *report, size_t length) {
    if (!isOpen() || !length) { error_ = "HID device is not open"; return false; }
#if APL
    IOReturn result = IOHIDDeviceSetReport(static_cast<IOHIDDeviceRef>(handle_),
        kIOHIDReportTypeFeature, report[0], report, length);
    if (result == kIOReturnSuccess) return true;
    error_ = deviceError("Set feature report", result);
#elif IBM
    if (length > featureLength_) { error_ = "Feature report is too large"; return false; }
    std::vector<uint8_t> padded(featureLength_, 0);
    std::copy(report, report + length, padded.begin());
    if (HidD_SetFeature(static_cast<HANDLE>(handle_), padded.data(), padded.size())) return true;
    error_ = deviceError("Set feature report", GetLastError());
#else
    (void)report;
#endif
    return false;
}

bool HIDDevice::getFeature(uint8_t *report, size_t length) {
    if (!isOpen() || !length) { error_ = "HID device is not open"; return false; }
#if APL
    CFIndex actual = length;
    IOReturn result = IOHIDDeviceGetReport(static_cast<IOHIDDeviceRef>(handle_),
        kIOHIDReportTypeFeature, report[0], report, &actual);
    if (result == kIOReturnSuccess && actual == static_cast<CFIndex>(length)) return true;
    error_ = result == kIOReturnSuccess ? "Incomplete feature report" : deviceError("Get feature report", result);
#elif IBM
    if (length > featureLength_) { error_ = "Feature report is too large"; return false; }
    std::vector<uint8_t> padded(featureLength_, 0);
    padded[0] = report[0];
    if (HidD_GetFeature(static_cast<HANDLE>(handle_), padded.data(), padded.size())) {
        std::copy(padded.begin(), padded.begin() + length, report);
        return true;
    }
    error_ = deviceError("Get feature report", GetLastError());
#else
    (void)report;
#endif
    return false;
}

bool HIDDevice::setOutput(const uint8_t *report, size_t length) {
    if (!isOpen() || !length) { error_ = "HID device is not open"; return false; }
#if APL
    IOReturn result = IOHIDDeviceSetReport(static_cast<IOHIDDeviceRef>(handle_),
        kIOHIDReportTypeOutput, report[0], report, length);
    if (result == kIOReturnSuccess) return true;
    error_ = deviceError("Set output report", result);
#elif IBM
    if (length > outputLength_) { error_ = "Output report is too large"; return false; }
    std::vector<uint8_t> padded(outputLength_, 0);
    std::copy(report, report + length, padded.begin());
    // Firmware 1.42 rejects SET_REPORT(Output, 2) on the control endpoint.
    // WriteFile delivers the report through the USB interrupt OUT endpoint.
    HANDLE device = static_cast<HANDLE>(handle_);
    OVERLAPPED write = {};
    write.hEvent = static_cast<HANDLE>(writeEvent_);
    ResetEvent(write.hEvent);
    DWORD written = 0;
    BOOL result = WriteFile(device, padded.data(), static_cast<DWORD>(padded.size()),
                            &written, &write);
    if (!result && GetLastError() == ERROR_IO_PENDING) {
        const DWORD wait = WaitForSingleObject(write.hEvent, 100);
        if (wait != WAIT_OBJECT_0) {
            const DWORD code = wait == WAIT_TIMEOUT ? ERROR_TIMEOUT : GetLastError();
            CancelIoEx(device, &write);
            // Keep the buffer and OVERLAPPED alive until cancellation completes.
            GetOverlappedResult(device, &write, &written, TRUE);
            error_ = deviceError("Wait for output report", code);
            return false;
        }
        result = GetOverlappedResult(device, &write, &written, FALSE);
    }
    if (result && written == padded.size()) return true;
    error_ = result ? "Incomplete output report" : deviceError("Write output report", GetLastError());
#else
    (void)report;
#endif
    return false;
}

bool HIDDevice::readGrip(bool& covered) {
    if (!isOpen()) { error_ = "HID device is not open"; return false; }
#if APL
    std::array<uint8_t, 21> report = {{1}};
    CFIndex actual = report.size();
    const IOReturn result = IOHIDDeviceGetReport(static_cast<IOHIDDeviceRef>(handle_),
        kIOHIDReportTypeInput, 1, report.data(), &actual);
    if (result != kIOReturnSuccess) {
        error_ = deviceError("Read grip sensor", result);
        return false;
    }
    if (actual != static_cast<CFIndex>(report.size()) || report[0] != 1) {
        error_ = "Incomplete grip sensor report";
        return false;
    }
#elif IBM
    if (inputLength_ < 21) { error_ = "Grip sensor report unavailable"; return false; }
    std::vector<uint8_t> report(inputLength_, 0);
    report[0] = 1;
    if (!HidD_GetInputReport(static_cast<HANDLE>(handle_), report.data(), report.size())) {
        error_ = deviceError("Read grip sensor", GetLastError());
        return false;
    }
    if (report[0] != 1) { error_ = "Invalid grip sensor report"; return false; }
#else
    (void)covered;
    error_ = "Grip sensor diagnostics require the native macOS or Windows backend";
    return false;
#endif
#if APL || IBM
    // Report 1 has 20 payload bytes. Its vendor-defined grip bit is 157.
    covered = (report[20] & 0x20) != 0;
    return true;
#endif
}
}
