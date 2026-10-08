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
| 22 | 1 | Negative velocity damping coefficient, signed |
| 23 | 1 | Positive velocity damping coefficient, signed |
| 24 | 2 | Velocity damping saturation, signed little endian |

Equal boundaries move the spring center without a deadband. Centers range
from -32767 to 32767. Coefficients are 96 for roll and 112 for pitch. Linux evdev
uses the equivalent values shifted left by eight bits.
Maximum saturation is 24576 for roll and 28672 for pitch. The base strength
ratio is the greater of 0.2 and true airspeed divided by Vne, capped at 1.0,
when airspeed is finite and nonnegative and Vne is finite and positive.
Roll saturation uses 1.5 times the ratio; pitch uses 1.75 times the ratio,
each capped at 1.0. Stiffness and force capacity are balanced per axis.
The first velocity damping
channel uses coefficient 8 and
saturation up to 4096, also scaled by airspeed. Constant-force, autocenter,
second spring and second damping fields remain zero. Disabling or pausing the
plugin sends a zeroed report with ID 2. Pausing fades spring stiffness,
saturation, and native damping together over about one second before sending
zero and releasing the live effect. The native connection stays open across
pauses to retain its idle-centering backup; Linux releases its evdev effect
and device as before. A cubic fade tapers both ends. Resuming
during the fade restores strength gradually; disabling and backend errors
still stop immediately.

The flight model derives roll center from aileron trim and pitch center from
elevator trim and angle of attack. Below 5 m/s the spring centers are zero,
providing gentle mechanical resistance while stationary or taxiing. A cubic
weight blends aerodynamic trim targets in between 5 and 15 m/s, avoiding
spurious low-speed AoA values. Stick movement does not change the spring
center; the device's spring supplies the restoring force locally. Center
changes are limited to 0.5 normalized units per second for roll. Pitch uses a
250 ms exponential filter and a lower limit of 0.25 per second, so small trim
steps are filtered and movement tapers into the new center. Airspeed-dependent
saturation ramps at up to 1.0 per second. A callback interval is capped at
100 ms for these ramps so a delayed simulator frame cannot cause a large
single-step change. Invalid airspeed or Vne stops the force immediately;
valid zero airspeed keeps the ground baseline until the plugin is paused.
The Linux constant-force fallback retains stick-dependent restoring force,
with the same smoothed trim target, stiffness gains and per-axis force caps.

Input report **1** is 21 bytes including its ID. Bit mask `0x20` in byte 20
is set while the grip sensor is covered. With the sensor uncovered the firmware
uses its idle centering settings, so an apparent centering force then does not
validate the application's live force output. The macOS/Windows plugin now
temporarily disables that idle centering to avoid switching to a different
neutral position when the hand is removed. It reads and backs up features
**5** (roll) and **6** (pitch), each four bytes including its ID, before writing
`05 00 00 00` and `06 00 00 00`. Each write is read back and checked. Pausing
keeps the backup and disabled idle settings; disabling or normal shutdown
restores both originals and checks them before closing the connection. If
initialization fails after modifying an axis, both restores are attempted.
The LED plugin does not change these settings. No firmware or saved settings
are written. The firmware still stops live force immediately when the grip
is uncovered; this change removes the replacement hands-off centering force.

`make probe` only opens the device and reads its LED report and grip sensor.
`build/tools/g940_probe --led-test` briefly writes a known pattern, reads it
back, and restores the original state. `--force-test` compares a centered spring
at 10% of each configured axis cap with zero force, independently of the
flight gains and ground baseline. `--roll-test` and `--pitch-test` compare a
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
the ongoing pitch-trim feel investigation.
