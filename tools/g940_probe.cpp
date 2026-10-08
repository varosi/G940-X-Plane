#include "g940HID.h"
#include "g940Protocol.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

int main(int argc, char **argv) {
    const bool ledTest = argc >= 2 && std::strcmp(argv[1], "--led-test") == 0;
    const bool forceTest = argc >= 2 && std::strcmp(argv[1], "--force-test") == 0;
    int seconds = 3;
    if (argc == 4 && std::strcmp(argv[2], "--seconds") == 0) {
        char *end = nullptr;
        seconds = std::strtol(argv[3], &end, 10);
        if (*end) seconds = 0;
    }
    if ((argc != 1 && argc != 2 && argc != 4) || (argc >= 2 && !ledTest && !forceTest) ||
        (argc == 4 && std::strcmp(argv[2], "--seconds") != 0) || seconds < 1 || seconds > 30) {
        std::fprintf(stderr, "Usage: %s [--led-test | --force-test] [--seconds 1..30]\n", argv[0]);
        return 2;
    }
    g940::HIDDevice device;
    if (!device.open()) { std::fprintf(stderr, "%s\n", device.error().c_str()); return 1; }
    std::puts("Connected to Logitech G940 (046d:c287), without exclusive access.");
    std::array<uint8_t, 3> original = {{3, 0, 0}};
    if (!device.getFeature(original.data(), original.size())) {
        std::fprintf(stderr, "%s\n", device.error().c_str()); return 1;
    }
    std::printf("LED report: ID=%02x red=%02x green=%02x\n", original[0], original[1], original[2]);
    if (ledTest) {
        const g940::LEDState colours = {{g940::RED, g940::GREEN, g940::AMBER, g940::OFF,
                                        g940::RED, g940::GREEN, g940::AMBER, g940::OFF}};
        const auto report = g940::ledReport(colours);
        if (!device.setFeature(report.data(), report.size())) {
            std::fprintf(stderr, "%s\n", device.error().c_str()); return 1;
        }
        std::printf("P1-P4 and P5-P8: red, green, amber, off for %d seconds.\n", seconds);
        std::fflush(stdout);
        std::this_thread::sleep_for(std::chrono::seconds(seconds));
        std::array<uint8_t, 3> observed = {{3, 0, 0}};
        const bool readBack = device.getFeature(observed.data(), observed.size());
        const bool restored = device.setFeature(original.data(), original.size());
        if (!readBack || observed != report || !restored) {
            std::fprintf(stderr, "LED verification/restore failed: %s\n", device.error().c_str());
            return 1;
        }
        std::puts("LED report readback matches; original LED state restored.");
    }
    if (forceTest) {
        const auto report = g940::forceReport({0.0, 0.0, 0.1});
        const auto stop = g940::stopReport();
        if (!device.setOutput(report.data(), report.size())) {
            const std::string error = device.error();
            device.setOutput(stop.data(), stop.size());
            std::fprintf(stderr, "%s\n", error.c_str()); return 1;
        }
        std::printf("Centered spring at 10%% saturation for %d seconds; hold the stick grip.\n", seconds);
        std::fflush(stdout);
        std::this_thread::sleep_for(std::chrono::seconds(seconds));
        if (!device.setOutput(stop.data(), stop.size())) {
            std::fprintf(stderr, "%s\n", device.error().c_str()); return 1;
        }
        std::puts("Force output accepted; stop report sent. Confirm resistance physically.");
    }
}
