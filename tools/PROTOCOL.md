# G940 HID reports

Native backends use shared access to Logitech `046d:c287`, preserving X-Plane
input. Feature reports use control transfers. Firmware 1.42 requires force
output through interrupt OUT endpoint `01` (64 bytes), rejecting control
`SET_REPORT(Output, 2)`. macOS uses `IOHIDDeviceSetReport`; Windows uses
100 ms overlapped `WriteFile`, checks completion length and waits for cancelled
writes before freeing their buffers. Windows pads reports to collection lengths.
Linux retains evdev force feedback and sysfs LEDs.

The connected descriptor and the original author's [G940 Linux driver](https://github.com/chrisboyle/G940-linux)
informed this independent byte-array encoder; no kernel code is included.
This is a vendor protocol, not HID PID. All lengths below include the report ID.

| Report | Length | Payload |
| --- | --- | --- |
| Feature 3 | 3 | Red bitmask, green bitmask for P1–P8; both bits mean amber |
| Feature 5/6 | 4 | Idle roll/pitch coefficient, saturation ×256, damping; signed bytes |
| Feature 10 | 5 | Signed little-endian idle roll center, then pitch center |
| Input 1 | 21 | Grip covered when byte 20 mask `0x20` is set |
| Output 2 | 64 | Two 30-byte axis records (roll, pitch), three reserved bytes |

Each output axis uses these offsets, excluding its report ID:

| Offset | Bytes | Meaning |
| --- | --- | --- |
| 0 | 2 | Signed little-endian constant force |
| 3/4/5 | 1 each | Signed autocenter coefficient / saturation ×256 / damping |
| 6/8 | 2 each | Signed little-endian negative/positive spring boundary |
| 10/11 | 1 each | Signed negative/positive spring coefficient |
| 12 | 2 | Signed little-endian spring saturation |
| 22/23 | 1 each | Signed negative/positive velocity damping coefficient |
| 24 | 2 | Signed little-endian velocity damping saturation |

Equal boundaries define the spring center without deadband. Native centers
span ±32767; maximum roll/pitch coefficients are 80/112 and caps 20480/28672.
Damping uses coefficient 8 and saturation 4096 × pressure strength. Unused fields
remain zero; a zero-filled report with ID 2 stops live force. Linux evdev
coefficients are shifted by eight bits and saturation uses twice the native
cap. Its constant-force fallback supplies stick-dependent restoring force,
with the same smoothed equilibrium, pressure strength and per-axis caps.

## Model and lifecycle

Calculations use `float`. The experimental `feature/realism` model reads ambient
dynamic pressure from `sim/flightmodel/misc/Qstatic` in psf and converts it to Pa
with factor 47.88026. This already accounts for air density and airflow through
`q = rho × V² / 2`; it replaces the earlier TAS/Vne strength estimate. Negative
or nonfinite pressure, nonfinite control/trim/AoA data or an invalid profile
stops force immediately. Zero pressure is valid and retains mechanical load.

The initial [TB10/TB20 profile](../g940ForceModel.h) has reference pressure about
2533 Pa (125 knots equivalent airspeed), mechanical ratio `m = 0.2`, roll trim
gain 3, pitch trim gain 1.5, AoA gain 0.03/degree and neutral AoA zero. It is the
default profile for every loaded aircraft in this experimental branch; aircraft
detection and other aircraft profiles are not implemented. These are starting
gains, not measured hinge moments or grip forces.

Aerodynamic stiffness is `a = (1 − m) × clamp(q / qref, 0, 1)` and total
strength is `s = m + a`. Roll/pitch coefficients are `round(80 × s)` and
`round(112 × s)`. Each axis cap is its configured maximum times
`clamp(s × maximumCoefficient / 64, 0, 1)`, so caps can reach their limits before
stiffness reaches its maximum. Neither coefficient nor cap exceeds the preceding
flight-tested maximum. This gives a gentle spring at rest and increasing stiffness
as well as increasing caps with aerodynamic load.

Mechanical resistance is centered at zero; the aerodynamic spring is centered
at aileron trim ×3 for roll and elevator trim ×1.5 − AoA ×0.03 for pitch. Balance
these springs at equilibrium `a / s × aerodynamicCenter`, clamped within ±1.
At zero pressure the equilibrium is zero even with arbitrary ground AoA/trim.
Stick movement does not change this equilibrium. The Linux constant-force
fallback uses equilibrium minus stick position on each axis, fixing the earlier
extra stick gain that gave it a different zero-force position from native springs.
Whole-aircraft AoA is still an approximation; local tail airflow, hinge geometry,
propwash and flap effects await further modeling and physical calibration.

Roll centers slew at 0.5 normalized units/s; pitch combines a 250 ms exponential
filter and 0.25/s limit. Strength ramps at 1/s. Callback intervals are capped
at 100 ms for these ramps. Connection/full pause-resume primes the current
center at zero strength. Pause scales stiffness, saturation and native damping
with a one-second cubic fade; partial resume ramps back up, while disable and
backend errors stop immediately. Linux releases its evdev effect on pause.

The firmware selects idle centering when the grip is uncovered. Native backends
back up features 5, 6 and 10 before any write, then zero live and idle force.
Each update mirrors live centers, coefficients, caps and damping into idle
settings, rounding native caps to the nearest 256. Changed features are read
back and checked before live output; unchanged values are cached. Pausing zeros
both channels while retaining the connection and backup. Normal close restores
original centers with idle force zero, then both profiles, attempting all
restores after failures. LEDs never modify these features. No persistent
settings, firmware updates or simulator control overrides are used.

Idle firmware caps spring and damping together; the live channel caps them
separately. Static emulation and a bench comparison agree in both grip modes,
but the latest flight reported early near-center grip-release kicks. Their
cause remains unisolated; see [hardware findings](HARDWARE_TESTS.md).

## Probe

`make probe` only reads LED state and grip. Optional LED/spring/constant-force
comparisons require preparation, electronically verify grip coverage, stop
on grip release/interruption and restore LEDs or send zero force. The spring
probe retains the full configured stiffness with 10% of configured caps;
constant force defaults to 4000/32767 and
accepts `--magnitude 1..16384`. Keep X-Plane closed and distinguish firmware
hands-off resistance from live force. Commands are in [README.md](../README.md#test-and-diagnose).
