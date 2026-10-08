#ifndef G940_BACKEND_H
#define G940_BACKEND_H
#include "g940Protocol.h"
namespace g940 {
bool openForceFeedback();
bool updateForceFeedback(const ForceState& state);
void closeForceFeedback();
bool openLEDs();
bool updateLEDs(const LEDState& state);
void closeLEDs();
const char *backendError();
}
#endif
