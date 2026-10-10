// Linux-only: exercise the real backend with simulated evdev syscalls.
// No devices are opened and no motors move.
#include "g940Backend.h"
#include "g940HID.h"
#include "g940ForceModel.h"
#include <cassert>
#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <linux/input.h>
#include <map>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

namespace {
bool spring = true, constant = true, rejectSpring = false;
bool enumerated = false, opened = false;
bool g940Device = false;
int nextID = 1, failUpdateID = 0, failPlayID = 0, failRemoveID = 0;
std::map<int, ff_effect> uploaded;
std::vector<int> removed, stopped;
void resetDevice() {
    assert(!opened);
    spring = constant = true;
    rejectSpring = enumerated = false;
    g940Device = false;
    failUpdateID = failPlayID = failRemoveID = 0;
    nextID = 1; uploaded.clear(); removed.clear(); stopped.clear();
}
DIR *fakeOpendir(const char *path) {
    assert(std::strcmp(path, "/dev/input") == 0);
    enumerated = false; return reinterpret_cast<DIR *>(1);
}
dirent *fakeReaddir(DIR *) {
    if (enumerated) return nullptr;
    enumerated = true;
    static dirent entry{};
    std::strcpy(entry.d_name, "event0"); return &entry;
}
int fakeClosedir(DIR *) { return 0; }
int fakeOpen(const char *path, int, ...) {
    assert(std::strcmp(path, "/dev/input/event0") == 0 && !opened);
    opened = true; return 42;
}
int fakeClose(int file) {
    assert(file == 42 && opened);
    opened = false; uploaded.clear(); return 0;
}
ssize_t fakeWrite(int file, const void *data, size_t count) {
    assert(file == 42 && opened && count == sizeof(input_event));
    const auto& event = *static_cast<const input_event *>(data);
    assert(event.type == EV_FF && uploaded.count(event.code));
    if (event.value == 0) stopped.push_back(event.code);
    if (event.value == 1 && failPlayID == event.code) { failPlayID = 0; errno = EIO; return -1; }
    return count;
}
int fakeIoctl(int file, unsigned long request, ...) {
    assert(file == 42 && opened);
    va_list args;
    va_start(args, request);
    int result = 0;
    if (_IOC_NR(request) == _IOC_NR(EVIOCGBIT(EV_FF, 0))) {
        auto *bits = va_arg(args, unsigned long *);
        const auto set = [bits](unsigned type) { bits[type / (8 * sizeof(unsigned long))] |= 1ul << (type % (8 * sizeof(unsigned long))); };
        if (spring) set(FF_SPRING);
        if (constant) set(FF_CONSTANT);
        set(FF_DAMPER); // even a driver advertising a damper may overwrite its spring
    } else if (request == EVIOCGID) {
        auto& identity = *va_arg(args, input_id *);
        identity.vendor = g940Device ? 0x046d : 0x1234;
        identity.product = g940Device ? 0xc287 : 1;
    } else if (request == EVIOCSFF) {
        auto& effect = *va_arg(args, ff_effect *);
        if ((effect.type == FF_SPRING && rejectSpring) || effect.id == failUpdateID) {
            errno = EIO; result = -1;
        } else {
            if (effect.id == -1) effect.id = nextID++;
            uploaded[effect.id] = effect;
        }
    } else {
        assert(request == EVIOCRMFF);
        const int id = va_arg(args, int);
        removed.push_back(id);
        if (id == failRemoveID) { errno = EIO; result = -1; }
        else assert(uploaded.erase(id) == 1);
    }
    va_end(args);
    return result;
}
}

#define opendir(...) fakeOpendir(__VA_ARGS__)
#define readdir(...) fakeReaddir(__VA_ARGS__)
#define closedir(...) fakeClosedir(__VA_ARGS__)
#define open(...) fakeOpen(__VA_ARGS__)
#define close(...) fakeClose(__VA_ARGS__)
#define write(...) fakeWrite(__VA_ARGS__)
#define ioctl(...) fakeIoctl(__VA_ARGS__)
#include "../g940Backend.cpp"
#undef opendir
#undef readdir
#undef closedir
#undef open
#undef close
#undef write
#undef ioctl

int main() {
    using namespace g940;
    auto state = calculateForce(0, 0, defaultProfile.referencePressurePa / 2, 0, 0, 0);
    state.rollVelocity = 1; state.pitchVelocity = -1;
    resetDevice();
    assert(openForceFeedback() && uploaded.size() == 1 && uploaded[1].type == FF_SPRING);
    assert(updateForceFeedback(state));
    assert(uploaded[1].u.condition[0].left_coeff == 48 << 8);
    state.rollCue = .03f; state.pitchCue = -.04f;
    assert(updateForceFeedback(state));
    assert(uploaded[1].u.condition[0].center == static_cast<int>(.03f * 32767));
    assert(uploaded[1].u.condition[1].center == static_cast<int>(-.04f * 32767));
    state.rollCue = state.pitchCue = 0;
    state.effectScale = 0;
    assert(updateForceFeedback(state) && uploaded[1].u.condition[0].left_coeff == 0);
    assert(releaseForceFeedback() && !opened && removed.size() == 1 && stopped.size() == 1);

    // On the G940, one composite constant effect preserves both loads and
    // works with drivers that advertise only a single effect slot.
    resetDevice(); g940Device = true;
    state.effectScale = 1;
    assert(openForceFeedback() && uploaded.size() == 1 && uploaded[1].type == FF_CONSTANT);
    assert(updateForceFeedback(state));
    assert(uploaded[1].u.constant.level > 0 && uploaded[1].direction == 8192);
    state.rollVelocity = state.pitchVelocity = 0;
    assert(updateForceFeedback(state) && uploaded[1].u.constant.level == 0);
    state.rollCue = .03f;
    assert(updateForceFeedback(state));
    assert(uploaded[1].u.constant.level > 0 && uploaded[1].direction == 49152);
    state.rollCue = 0;
    assert(closeForceFeedback() && removed.size() == 1);

    resetDevice(); spring = false;
    state.rollVelocity = 1; state.pitchVelocity = 0;
    assert(openForceFeedback() && uploaded.size() == 1 && uploaded[1].type == FF_CONSTANT);
    assert(updateForceFeedback(state));
    assert(uploaded[1].u.constant.level > 0 && uploaded[1].direction == 16384);
    state.effectScale = 0;
    assert(updateForceFeedback(state) && uploaded[1].u.constant.level == 0);
    assert(closeForceFeedback());

    resetDevice(); g940Device = true; constant = false;
    assert(openForceFeedback() && uploaded.size() == 1 && uploaded[1].type == FF_SPRING);
    assert(closeForceFeedback()); // preserve spring feedback on spring-only drivers
    resetDevice(); rejectSpring = true;
    assert(openForceFeedback() && uploaded.size() == 1 && uploaded[1].type == FF_CONSTANT);
    assert(closeForceFeedback());
    resetDevice();
    assert(openForceFeedback()); failUpdateID = 1;
    assert(!updateForceFeedback(state) && !opened && removed.size() == 1 && stopped.size() == 1);
    resetDevice();
    assert(openForceFeedback()); failRemoveID = 1;
    assert(!closeForceFeedback() && !opened && removed.size() == 1);
    resetDevice(); constant = false; failPlayID = 1;
    assert(!openForceFeedback() && !opened && uploaded.empty());
    std::puts("Evdev composite damping, cue centers, spring fallback, pause and cleanup passed.");
}
