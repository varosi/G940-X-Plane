# G940 HID reports

The native macOS and Windows backends address Logitech USB vendor `046d`,
product `c287`. They use control transfers with shared access, leaving joystick
input available to X-Plane. Linux retains evdev force feedback and sysfs LEDs.

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

`make probe` only opens the device and reads its LED report.
`build/tools/g940_probe --led-test` briefly writes a known pattern, reads it
back, and restores the original state. `--force-test` applies a centered spring
at 10% saturation for three seconds, then sends the stop report. Successful USB
transfers do not prove that a powered motor produces the intended force; that
requires checking the stick physically. Use `--seconds 10` to extend either
test (1-30 seconds). Keep X-Plane closed during these tests.
