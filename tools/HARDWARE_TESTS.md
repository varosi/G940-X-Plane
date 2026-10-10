# G940 hardware findings

Status on 2026-10-09 for the preceding `feature/mac` model: native macOS force feedback and LEDs operate in X-Plane 12
on Apple Silicon with G940 `046d:c287`, device version `0x0142`, motor power and
a USB hub. Windows/Linux builds pass; their hardware behavior remains untested.
Trim and pause release are smooth. The latest flight reported hands-off kicks
near center early in the flight, disappearing later. This is still unresolved;
the quieter bench comparison does not establish kick-free flight behavior.

## Bench evidence

Tests used verified grip coverage unless specifically hands off. Early tests
without known sensor coverage were inconclusive. Force comparisons require a
preparation warning and readiness before starting; USB success is insufficient
proof of physical effect.

| Comparison | Observation |
| --- | --- |
| Hands-off pitch idle profile: original, zero, restored | Centering disappeared and returned |
| Low-level live spring/autocenter or ±4000 pitch / +4000 roll constant force | Stayed free |
| Alternate payload/prefix/short packets, element API, original Linux packet | Stayed free |
| Exclusive access or direct USB roll/pitch | Stayed free; direct USB wrote fully and restored native driver |
| Volatile force reset, power/USB cycle, second hub | Reset gave a brief force; subsequent low-level effects stayed free |
| Native pitch constant force 16000/32767, 5 s on/off | Pull felt and stopped |
| Native pitch spring 50% saturation, 10 s on/off | Centering came and went |
| Damped pitch spring 50%, immediate zero | Smooth resistance, small release jump |
| Pitch coefficient 64 →96, same 50% cap | Little strength change; release kick |
| Pitch cap 50% →75%, center first and fade for 2 s | Stronger; perfectly smooth release |
| Pitch cap 75% →100%, same fade | Stronger; release could be smoother |
| Live/idle matched center and profile, 10 s in each grip state | Same resting position and resistance, no kick |

The last comparison used a 4% pitch center, base ratio 0.4, caps 10240/19968,
standard damping, a 2 s start ramp, 3 s center return and 1 s release. Writes
read back correctly; independent reads verified all original profiles/centers
following close. Normal macOS operation requires no administrator password;
earlier direct USB experiments did, and are not part of normal plugin use.

## Flight evidence

Recorded flights used the TB10/TB20 (Vne 187 knots) and diagnostic traces,
with no force-backend errors. Counts below are numeric force samples;
the latest flight ended with a normal shutdown.

| Model stage | Samples | Observations |
| --- | --- | --- |
| Initial diagnostic | 121 | Forces felt; pitch trim kicked near zero vertical speed |
| Stick-independent centers, slew and damping | 132 | Trim much smoother, no trim kicks; force still weak |
| Stronger pitch, filtered small trim steps, one-second pause fade | 245 | Trim/pause smooth; grip-release kicks; two completed fades |
| Maximum pitch stiffness, idle centering disabled | 106 | Requested stronger roll, lighter pitch and stationary resistance; grip release not checked |
| Roll 96/24576, pitch 112/28672, ground baseline | 134 | Roll too strong; requested trim-only hands-off flight; 34 ground samples, one fade |
| Roll 80/20480, matched live/idle trim centers | 319 | Near-center hands-off kicks early, absent later; 18 ground samples, max TAS 162.4 kt, one fade |

During the last flight a read-only query detected HAND OFF and feature 10
matched the logged roll/pitch centers. The user has not separately confirmed
that the revised roll strength is ideal or assessed trim-only flight stability.
The `feature/mac` refactor preserved that model and its unresolved early kick.

## Experimental realism model

`feature/realism` starts with a TB10/TB20 profile. It replaces linear TAS/Vne
strength and fixed stiffness with ambient dynamic pressure, a mechanical spring
baseline and a pressure-weighted trim equilibrium. Maximum coefficients/caps,
trim filtering, pause fading and live/idle mirroring are retained. Linux's
constant-force fallback now reaches zero force at the native spring equilibrium.
Aircraft configuration now supplies General and Socata presets, selected by
ICAO or filename. Socata retains its initial gains; General uses normalized
trim gains of 1 and X-Plane Vne as a provisional pressure-scaling reference.

The trim-equilibrium revision separates aerodynamic, spring and stabilizer
trim, combines pitch loads in degrees using X-Plane elevator travel/static
tab metadata, and keeps the force balance responsive above the capped motor
reference speed. The installed Socata model declares ±15-degree elevator
travel, a +0.1 static tab ratio and zero stabilizer trim travel. It therefore
uses the aerodynamic preset; these values are metadata, not force calibration.
A read-only check found all saved `_joy_ffb_axis*` flags zero. X-Plane 12's
documented control-loaded `.joy` setting is required to prevent simultaneous
simulator/plugin trim-center offsets. No joystick preferences were changed.

Mechanical/aerodynamic spring loads now remain separate through smoothing;
independent gains also separate mechanical and airflow-dependent damping.
The default spring curve and maximum motor settings are retained. Native
ground damping now has coefficient 2 (formerly a fixed 8), increasing to 8
at reference pressure; its cap is quantized like the spring cap. Damping-only
profiles and pause fades are covered by mock tests. The original Linux driver
sends fresh reports per effect, so the G940 uses one software-mixed constant
effect to retain both spring and velocity resistance, with native spring
fallback for spring-only drivers. No new hardware test has validated these
damping changes or the Linux software path.

Software tests cover the pressure units, speed-squared/density scaling, ground
baseline, increasing stiffness/caps, bounded maximums, trim equilibrium and
asymmetric travel, static bias, trim-type behavior, steady out-of-trim pressure,
profile parsing/selection/reload, metadata fallback, installation preserving
user settings, and existing lifecycle/rollback behavior. No new motor or flight test has run for
this model; preceding flight observations do not validate it. The initial 125
KEAS reference and trim/AoA gains require tuning, and the grip-release kick
remains open. Next flight comparisons should use the same tested TB10/TB20;
warn and obtain readiness before applying forces.

## Turbulence and buffet revision, 2026-10-10

Manufacturer TB20 evidence supports weak natural buffet at idle, stronger with
power, and an aural stall horn rather than a stick shaker. The new Socata cues
follow local wind variation and main-wing element separation; the warning horn
alone does not create shaking. See [research and tuning limits](TB20_EFFECTS.md).
Existing trim, motor caps and native live/idle center mirroring are retained.

The G940 is disconnected. Software verification cannot establish perceptible
strength, correct physical sign, a measured TB20 spectrum, feature-10 transfer
rate or smooth grip transitions in flight. No new hardware observation resolves
the prior early near-center kick. The source installer preserves existing
`aircraft.ini`; older configurations need the new Socata gains enabled explicitly.

## Settings and firmware analysis

Original volatile reports were independently verified after comparisons:

```text
Feature 4:  04 00
Feature 5:  05 14 3c 7f
Feature 6:  06 1a 64 5a
Feature 10: 0a 00 00 00 00
```

Feature 4 bit `0x04` resets volatile force state; the plugin does not write it.
Features 5/6 contain four bytes despite the descriptor's five-byte maximum;
feature 10 supplies two signed 16-bit idle centers. All current centering
writes are volatile, backed up and read back. Normal close restores centers
while idle force is zero, then both profiles. Pause retains the backup.

Offline ARM emulation of the official Logitech 1.42 image decoded full output
report 2 and modeled normal power with grip coverage. ±4000 constant force gave
pitch PWM magnitude 341; zero gave 0. A 10% spring at positions ±10000 gave
330 and zero at center. Coefficients 64/96 shared a cap at larger deflections,
explaining why increasing stiffness alone need not raise maximum force.
For coefficient 112/cap 28672 at zero velocity, live and idle motor commands
matched at centers −8000/0/8000 and positions −16000/−8000/0/8000/16000.
These are modeled outputs, not measured torque or a read of installed firmware.
No firmware was flashed. Idle combines spring/damping under one cap; live caps
them separately, so matching static load does not prove equality during motion.
