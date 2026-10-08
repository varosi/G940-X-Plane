# G940 macOS force-feedback investigation

Status on 2026-10-09: live pitch force and spring centering are physically
confirmed at higher levels and in flight. The latest flight confirmed smooth
trim and pause release. The following flight requested stronger roll, slightly
lower pitch and gentle stationary resistance. Native idle settings were read
back as disabled during that flight and restored after exit. Grip release was
not checked physically; the new ground/axis balance also awaits validation.
Accepted USB writes alone are not evidence that the motors rendered an effect.

The connected device reports USB ID `046d:c287` and device version `0x0142`.
Tests used Apple Silicon and a USB 2.0 hub, with X-Plane closed. Motor power
was connected. Early subjective comparisons without verified grip coverage
were inconclusive; the comparisons below monitored the grip sensor.

| Comparison | Physical observation |
| --- | --- |
| Idle pitch centering, grip sensor uncovered: original feature 6, zero feature 6, original restored | Centering disappeared and returned |
| Live spring and autocenter, grip sensor covered | Stick stayed free |
| Live constant force of +4000 and -4000 on pitch; +4000 on roll | Stick stayed free |
| Payload-only, extra ID prefix, shortened packet, and output-element API variants | Stick stayed free |
| Original Linux driver packet with first data byte `0x51` | Stick stayed free |
| Exclusive HID access | Stick stayed free |
| Direct USB interrupt OUT, separately on roll and pitch | Stick stayed free; full writes completed and native driver reattached |
| Force-state reset through feature 4, then pitch force versus zero | Brief force during reset, then stick stayed free |
| USB and motor power disconnected, then reconnected through the same hub; native probe pitch comparison | Stick stayed free |
| Other USB hub and different USB path; identical native probe pitch comparison | Stick stayed free |
| Native probe pitch constant force at 16000/32767, five seconds on and five seconds zero, grip covered | Pull was felt and stopped |
| Plugin encoder's pitch spring at 50% saturation, ten seconds on and ten seconds zero, grip covered | Centering resistance came and went |

The higher-level comparisons confirm that the native macOS HID path and the
plugin's spring packet can drive the pitch motor. The earlier low-level tests
do not establish a transport failure. A strength threshold or weak physical
response is a possibility; the exact threshold has not been measured.

Feature reports were backed up before the idle-centering and reset comparisons.
The original settings were restored and read back afterward:

```text
Feature 4: 04 00
Feature 5: 05 14 3c 7f
Feature 6: 06 1a 64 5a
```

Feature 4 bit `0x04` requests a reset of the volatile force state and clears
itself after the reset. Features 5 and 6 each contain four transfer bytes,
including the report ID, despite the descriptor's larger maximum feature size.
These are investigation findings, not initialization changes in the plugin.

Offline ARM emulation of the original Logitech 1.42 firmware image checked its
interrupt-report decoder and force calculation. A full 64-byte report with ID
2 and pitch constant force at bytes 31-32 decoded as intended. With simulated
grip coverage, the reset delay expired, and normal motor power state, +4000 and
-4000 produced pitch PWM magnitude 341; zero produced 0. A centered spring at
10% pitch saturation produced magnitude 330 at simulated positions +/-10000
and 0 at center. This checks the packet layout under those modeled conditions;
it does not observe the physical USB bus or read the installed firmware image.
No firmware was flashed.

Changing the hub alone did not make the lower-level effect perceptible. Raising
the constant-force magnitude did.

## First diagnostic flight

The TB10/TB20 flight produced 121 force trace samples. Aircraft Vne was 187
knots, and the higher-speed portion covered 97.2-149.8 knots true airspeed with
52.0-80.1% spring saturation. There were no force-backend errors, and a separate
read-only check detected the grip as covered during flight. The user felt force
feedback but reported strange pitch-trim kicks near zero vertical speed and
expected a stronger steady load when out of trim. Pause behavior was not
checked physically.

The original spring model made its center a function of the measured stick
position on every frame and applied trim changes immediately. The updated
model removes stick position from the spring-center calculation, limits trim
and angle-of-attack center changes, ramps force saturation after connecting,
and uses light damping on the native G940 path. The Linux constant-force
fallback still calculates stick-dependent restoring force separately. Software
tests check target independence, bounded transitions, restart ramping and
immediate zero-airspeed stops. The flight observation below confirms improved
trim feel with this model.

A separate pitch-only bench comparison used 50% spring saturation with the
new damping for 10 seconds, followed by 10 seconds of zero force. The grip
sensor remained covered throughout. The user reported smooth resistance
during the active spring, with a small jump when force switched off. This
comparison does not validate trim changes in flight; force removal was
immediate and could release a loaded stick.

## Flight with the updated spring model

The next TB10/TB20 flight used the updated diagnostic plugin and produced 132
force trace samples, including 64 above 97 knots. That portion covered
97.3-129.8 knots true airspeed and 52.0-69.4% spring saturation. Elevator trim
varied from -0.175 to 0.399, with pitch spring centers from -0.328 to 0.552.
There were no force-backend errors. A read-only grip check during flight
detected the sensor as covered, and X-Plane subsequently shut down normally.
The user reported much smoother trim changes without kicks, while requesting
even smoother transitions. Pausing with the grip covered released the force;
with the grip uncovered, the firmware's idle centering remained active. Force
was still much weaker than on the user's real airplane.

## Stronger pitch spring and further smoothing

An initial bench comparison raised the pitch spring coefficient from 64 to 96
while keeping 50% saturation. The grip remained covered throughout. The user
felt little change in strength and again noted a kick when switching to zero
force. Offline firmware emulation found that both settings reach the same cap
at larger deflections: at position 8000 both produced PWM magnitude 525.
Raising the stronger setting's cap to 75% produced magnitude 646 instead.
These are modeled controller outputs, not measurements of physical torque.

The next candidate raises both pitch stiffness and pitch saturation by a
factor of 1.5, capped at the device's existing maximum. Roll retains its
coefficient of 64 and its previous saturation. Linux evdev uses the equivalent
coefficients and saturation; its constant-force fallback increases the pitch
demand by the same factor of 1.5. Diagnostic traces now distinguish the base
airspeed ratio from each axis's actual spring saturation.

Pitch-center changes use a 250 ms exponential filter, with a reduced slew
limit of 0.25 normalized units per second. This filters small trim steps too
and tapers toward the new neutral position. Roll retains its previous slew
limit of 0.5 per second. At this stage force release on pause was immediate.

The follow-up grip-verified bench comparison used coefficient 64 with 50%
saturation, then coefficient 96 with 75% saturation. After the stronger stage,
the user centered the stick for three seconds, the effect faded over two
seconds, and the test held zero force for ten seconds. The user felt stronger
resistance and reported a perfectly smooth release. The 1.5 pitch strength factor permits
100% spring saturation at two-thirds of Vne, approximately 125 knots true
airspeed in the tested aircraft with Vne 187 knots.

A further grip-verified comparison kept coefficient 96 and compared 75% with
100% pitch spring saturation. The user felt stronger resistance at 100%, but
requested a slightly smoother release after the two-second fade. No firmware
or idle-centering settings were changed during these spring comparisons.

## One-second pause release

The user selected a smooth one-second release when pausing. The plugin now
fades spring stiffness and saturation together, including native damping,
then sends zero force and closes the device. Resuming partway through the
fade restores strength gradually. Disabling, backend errors, zero airspeed
and invalid Vne retain their immediate stops. Tests cover the fade midpoint,
zero-force completion, delayed callbacks and resume during a fade. This pause
release and the additional pitch filter were checked in the flight below.

## Flight with stronger pitch and one-second release

The TB10/TB20 flight produced 245 force samples and ended with a normal X-Plane
shutdown. Of those samples, 200 were above 97 knots with the connection ramp
settled and no pause fade in progress. They covered 100.0-172.9 knots true
airspeed and 80.2-100% pitch spring saturation. The log recorded two completed
pause fades and no force-backend errors. The user reported smooth trimming
and smooth pausing, but requested greater force. Kicks occurred only when
transitioning from hand on to hand off.

## Maximum positive pitch stiffness and native idle-centering ownership

The next candidate raises pitch coefficient 96 to 127, about 32% more stiffness,
while keeping the existing saturation ceiling. The associated airspeed strength
factor is now 127/64, allowing full pitch saturation at about 94 knots true
airspeed for Vne 187 knots. The trim filter, pause fade, damping and roll setting
are unchanged. Offline firmware emulation at full pitch saturation produced
PWM magnitudes 459 and 517 for coefficients 96 and 127 at position 2000;
at position 8000 both were capped at 768. This predicts increased resistance
near the trim center, not increased maximum torque once the effect is capped.

The user selected temporary removal of hands-off centering while the plugin
is enabled. The native backend backs up both idle-axis features before
modifying either, writes zero idle settings, and checks their readback. The
original settings are retained across pause/resume and restored and checked
on disable or normal exit. Failure while initializing triggers a restore of
both axes. Tests cover pause retention, failure rollback, incorrect readback,
unexpected report IDs and attempting both restores after a failure. Linux
retains its evdev pause/close behavior and receives the same stronger pitch
model. The flight below checked this strength setting; grip-release validation
remains pending.

## Flight with maximum pitch stiffness and idle centering disabled

The flight produced 106 force samples and shut down normally, with no backend
errors. The 45 settled samples above 97 knots covered 99.4-139.1 knots true
airspeed, 53.1-74.4% roll saturation and 100% pitch saturation relative to each
axis's configured cap. No pause fade was recorded in this flight. A read-only
check detected the grip covered during flight. Separate reads confirmed both
idle features were zero while connected and that the original settings
returned after shutdown: `05 14 3c 7f` and `06 1a 64 5a`. The user requested
stronger roll, slightly lower pitch and gentle resistance when stationary.
The user did not check grip release, so disappearance of that kick is unconfirmed.

## Ground resistance and revised roll/pitch balance

The next candidate raises roll coefficient 64 to 96 and its saturation ceiling
16384 to 24576. Pitch coefficient drops from 127 to 112 and its ceiling from
32767 to 28672, reducing stiffness and capped capacity by about 12%. Both axes
use their coefficient divided by 64 as the airspeed saturation gain. Linux
evdev uses equivalent coefficients and caps; its constant-force fallback now
limits each component to the corresponding axis's airspeed-dependent cap.

Valid flight data retains a minimum base strength of 0.2 even at zero airspeed.
This is a model parameter, not a measurement of torque. Ground centering is
neutral below 5 m/s. A cubic weight blends trim and AoA targets in between 5
and 15 m/s, preventing the previous flight's approximately -121 degree AoA
while nearly stationary from driving the ground spring to an extreme center.
Invalid or negative airspeed and invalid Vne still stop immediately. The
existing trim filtering, one-second pause release and native idle ownership
are unchanged. Tests cover the centered baseline, target blending, invalid
inputs and bounded Linux fallback components. Physical validation is pending.
