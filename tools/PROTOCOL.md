# G940 HID reports

The native macOS and Windows backends address Logitech USB vendor `046d`,
product `c287`. They use shared access, leaving joystick input available to
X-Plane. Feature reports use control transfers. Force output report 2 requires
the interrupt OUT endpoint (`01`, 64-byte packets): the connected firmware 1.42
rejects `SET_REPORT(Output, 2)` on the control endpoint. macOS uses
`IOHIDDeviceSetReport`; Windows uses overlapped `WriteFile`, checks the full
report length, and cancels writes that exceed 100 ms. Linux retains evdev force
feedback and sysfs LEDs.

The layout below is based on the connected device's HID descriptor and the
original author's [G940 Linux driver](https://github.com/chrisboyle/G940-linux).
The protocol encoder is implemented independently using byte arrays; no kernel
driver code is included. This is a G940-specific vendor protocol, not HID PID.

Feature report **3** contains two data bytes: a red bitmask for P1-P8, followed
by a green bitmask for P1-P8. Bit 0 is P1. Set both bits for amber. A transfer
contains three bytes including the report ID. Windows pads this to the maximum
feature report length returned by `HidP_GetCaps`.

Output report **2** contains 63 data bytes: two 30-byte axis records (roll,
then pitch), followed by three reserved bytes. A transfer contains 64 bytes
including the report ID. Each axis record has the following spring fields:

| Offset within axis | Size | Meaning |
| --- | --- | --- |
| 0 | 2 | Constant force, signed little endian |
| 3 | 1 | Autocenter coefficient, signed |
| 4 | 1 | Autocenter saturation, signed, in units of 256 |
| 5 | 1 | Autocenter damping coefficient, signed |
| 6 | 2 | Negative spring boundary, signed little endian |
| 8 | 2 | Positive spring boundary, signed little endian |
| 10 | 1 | Negative coefficient, signed |
| 11 | 1 | Positive coefficient, signed |
| 12 | 2 | Spring saturation, signed little endian |

Equal boundaries move the spring center without a deadband. Centers range
from -32767 to 32767. Coefficients are 64, matching the existing Linux model.
Maximum saturation is 16384 for roll and 32767 for pitch, scaled by airspeed
relative to Vne. Unused constant-force, autocenter and damper fields remain
zero. Disabling or pausing the plugin sends a zeroed report with ID 2.

Input report **1** is 21 bytes including its ID. Bit mask `0x20` in byte 20
is set while the grip sensor is covered. With the sensor uncovered the firmware
uses its idle centering settings, so an apparent centering force then does not
validate the application's live force output.

`make probe` only opens the device and reads its LED report and grip sensor.
`build/tools/g940_probe --led-test` briefly writes a known pattern, reads it
back, and restores the original state. `--force-test` compares a centered spring
at 10% saturation with zero force. `--roll-test` and `--pitch-test` compare a
constant force of 4000 with zero force on one axis; `--reverse` uses -4000.
`--magnitude 1..16384` adjusts constant-force comparisons up to half the nominal
range. It is accepted only with `--roll-test` or `--pitch-test`; the selected
level is displayed before the readiness prompt. Nominal range is not a
measurement of physical torque.
Each stage lasts three seconds by default; `--seconds 10` extends each stage
(1-30 seconds). Each test describes the next comparison and waits for Enter.
Force tests then require two seconds of continuous grip-sensor coverage,
stream reports at approximately 50 Hz, and stop when the grip is released,
input/output fails, or the process receives SIGINT/SIGTERM. Keep X-Plane closed.
Successful USB transfers do not prove that a powered motor produces the
intended force; that requires checking the stick physically.

See [hardware test findings](HARDWARE_TESTS.md) for the macOS comparisons and
the remaining unresolved force-output issue.
