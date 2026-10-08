# G940 macOS force-feedback investigation

Status on 2026-10-09: live pitch force and spring centering are physically
confirmed at higher levels and in flight. The updated model felt smoother
without trim kicks; further smoothing and stronger pitch resistance were
requested.
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
limit of 0.5 per second. Force-release behavior on pause remains immediate.
The additional pitch filter awaits flight validation.

The follow-up grip-verified bench comparison used coefficient 64 with 50%
saturation, then coefficient 96 with 75% saturation. After the stronger stage,
the user centered the stick for three seconds, the effect faded over two
seconds, and the test held zero force for ten seconds. The user felt stronger
resistance and reported a perfectly smooth release. The additional pitch trim
filter still needs flight validation. The 1.5 pitch strength factor permits
100% spring saturation at two-thirds of Vne, approximately 125 knots true
airspeed in the tested aircraft with Vne 187 knots.

A further grip-verified comparison kept coefficient 96 and compared 75% with
100% pitch spring saturation. The user felt stronger resistance at 100%, but
requested a slightly smoother release after the two-second fade. No firmware
or idle-centering settings were changed during these spring comparisons.
