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
Damping uses maximum coefficient 8 and saturation 4096, with independent load settings. Unused fields
remain zero; a zero-filled report with ID 2 stops live force. Linux evdev
coefficients are shifted by eight bits and saturation uses twice the native
cap. Its constant-force path supplies restoring force and motion damping,
with the same smoothed equilibrium, spring strength and per-axis caps.

## Model and lifecycle

Calculations use `float`. The experimental `feature/realism` model reads ambient
dynamic pressure from `sim/flightmodel/misc/Qstatic` in psf and converts it to Pa
with factor 47.88026. This already accounts for air density and airflow through
`q = rho × V² / 2`; it replaces the earlier TAS/Vne strength estimate. Negative
or nonfinite pressure, nonfinite control/trim/AoA data or an invalid profile
stops force immediately. Zero pressure is valid and retains mechanical load.

The initial [TB10/TB20 profile](../aircraft.ini) has reference pressure about
2533 Pa (125 knots equivalent airspeed), mechanical ratio `m = 0.2`, roll trim
gain 3, pitch trim gain 1.5, free-elevator/AoA gain 0.45 degrees/degree and neutral AoA zero. Aircraft
identity selects this preset for Socata; unmatched aircraft use General with
trim gains 1 and an automatic Vne-based pressure reference. These are starting
gains, not measured hinge moments or grip forces. See configuration below.

Mechanical stiffness `m = mechanical_ratio` is independent of pressure.
Aerodynamic stiffness is `a = min(aerodynamic_gain × q / qref, 1)`; the two
terms remain separate through smoothing, with total rendered strength
`s = clamp(m + a, 0, 1)`. Roll/pitch coefficients are `round(80 × s)` and
`round(112 × s)`. Each axis cap is its configured maximum times
`clamp(s × maximumCoefficient / 64, 0, 1)`, so caps can reach their limits before
stiffness reaches its maximum. Neither coefficient nor cap exceeds the preceding
flight-tested maximum. This gives a gentle spring at rest and increasing stiffness
as well as increasing caps with aerodynamic load.

The equilibrium balance uses **uncapped** aerodynamic load, with weight
`w = aerodynamic_gain × q / (m × qref + aerodynamic_gain × q)`. Zero pressure
or zero aerodynamic gain gives zero aerodynamic weight; with zero mechanical
stiffness and positive airflow it is one. Motor strength remains capped
independently; reaching full stiffness must not freeze the equilibrium at an
80% aerodynamic blend. Roll center is `w × aileronTrim × rollTrimGain`.

Pitch uses a linear hinge-load surrogate in degrees. Positive pitch means
elevator up. A trim ratio is multiplied by the corresponding up/down elevator
travel; airflow contributes `−pitchAoADeflectionGain × (alpha − neutralAlpha)`.
Static tab deflection is read separately from `acf_elev_tab`. These contributions
are combined before converting back to a normalized center using the travel
in the resulting direction, so asymmetric travel and opposing trim/AoA loads
can cross zero correctly. Aerodynamic mode weights trim, static tab and airflow
by `w`. Spring mode preloads only the mechanical spring: its trim contribution
is weighted by `1 - w`, retaining trim at zero airflow. Stabilizer mode excludes the elevator trim input and adds the
actual stabilizer incidence (positive leading edge up) to alpha instead.
All centers are clamped to ±1. At zero pressure aerodynamic/stabilizer trim
has no influence, even with arbitrary ground AoA. Stick movement does not
change this equilibrium. The Linux constant-force
fallback uses equilibrium minus stick position on each axis, fixing the earlier
extra stick gain that gave it a different zero-force position from native springs.
Whole-aircraft AoA plus stabilizer incidence is still an approximation; downwash,
local tail airflow, hinge coefficients, propwash and flap effects await further
modeling and physical calibration. This is not a measured hinge moment, and
airframe pitching torque is not used as one. Powered/FBW jets need their own
artificial-feel profile. The [X-Plane trim types](https://developer.x-plane.com/article/types-of-flight-control-trim/)
explain why trim tabs, spring trim and THS cannot share one center-offset rule.

Roll centers slew at 0.5 normalized units/s; pitch combines a 250 ms exponential
filter and 0.25/s limit. Mechanical/aerodynamic spring terms share a 1/s slew
budget; changing one does not silently change the other's configured gain.
Callback intervals are capped
at 100 ms for these ramps. Connection/full pause-resume primes the current
center at zero strength. Pause scales stiffness, saturation and native damping
with a one-second cubic fade; partial resume ramps back up, while disable and
backend errors stop immediately. Linux releases its evdev effect on pause.

Damping is a separate movement-dependent load:
`d = clamp(mechanical_damping + aerodynamic_damping × q / qref, 0, 1)`.
The native velocity channel uses `round(8 × d × pauseScale)` and a 4096×d cap,
rounded to 256 like the spring cap. Idle firmware shares its spring/damper cap,
so a live damper is limited to the spring cap when a spring is present. A
damper-only effect uses its own cap in both grip modes, with spring coefficient
zero. Damping is slewed at 1/s independently and participates in the pause fade.
Neither stiffness nor damping can exceed the existing motor limits.

The [original G940 Linux driver](https://github.com/chrisboyle/G940-linux/blob/main/drivers/hid/hid-lg3ff.c)
constructs a fresh report for each effect, clearing other effect channels.
The backend therefore prefers a single `FF_CONSTANT` effect for Logitech
046d:c287 (also when identity is unavailable), mixing spring and damping in
software. Constant-only drivers use the same path; spring-only drivers retain
the original spring without damping. Other devices retain spring preference.
This uses the [evdev effect lifecycle](https://docs.kernel.org/input/ff.html)
and requires only one effect slot. Failed uploads/playback/updates clean up the
effect and connection. Software damping opposes filtered normalized stick
velocity, with coefficient `round(8 × d)/64` and the same caps. Velocity uses
elapsed time, a 50 ms filter and ±4 units/s bound. Connection, invalid data and
pause reset its history so stale movement cannot create a resume impulse.
This callback-rate estimate is an approximation to native velocity damping;
no dry friction, inertia or simulator input override is introduced.

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

## Gust and natural buffet cues

The [TB20 research](TB20_EFFECTS.md) distinguishes aerodynamic buffet from its
aural stall warning. Optional cues leave X-Plane responsible for weather and
aircraft physics; no aircraft force, control or weather dataref is written.

Gust cues read the modern `sim/weather/aircraft/wind_now_*_msc` vectors (m/s,
world east/up/south) and `sim/flightmodel/position/psi`, `theta`, `phi` (degrees).
A 0.8 s world-coordinate low-pass removes steady wind; an 80 ms gust filter
attenuates rapid steps. Rotate the residual into body right/up coordinates only
after filtering, so turning in constant wind creates no synthetic turbulence.
Negative gain times those two components supplies roll/pitch center offsets.
The vectors' coverage of wakes and thermals is not guaranteed; this is a local
weather wind-variation cue, not a measured turbulent hinge moment.

Buffet reads paired `sim/flightmodel2/wing/elements/element_is_stalled` and
`element_surface_area_mtr_sq` float arrays. The first 80 slots cover main-wing
surfaces 0..7, ten elements each; horizontal tails 8/9 are excluded. Complete,
finite arrays and positive total area are required. Area-weighted separation
sets `clamp(stalledArea / totalArea / 0.2, 0, 1)`. This can respond to incipient
partial-wing separation; it does not assert that the horn implies separation.
The cockpit stall-warning status is deliberately not used as a shaker trigger.

Actual `sim/cockpit2/engine/indicators/power_watts` is divided by configured
maximum power from `sim/aircraft/engine/acf_pmax_per_engine`, summing only
`acf_num_engines` engines. Legacy scalar `acf_pmax` is the fallback; missing
engine count uses the first engine. Missing/invalid power uses the weak idle
baseline. Buffet strength multiplies the separation signal by
`stall_buffet_gain * (idleRatio + (1 - idleRatio) * clamp(powerRatio, 0, 1))`.
A 200 ms envelope ramps it into two sine components at the configured frequency
and 1.37 times that frequency, combined with weights 1 and 0.25 and divided by
1.25. This bounded irregular waveform is a tuning approximation, not a measured
TB20 spectrum. It adds pitch buffet without a scripted wing-drop impulse.

Apply offsets after trim and stick-velocity filtering, preserving the base
trim equilibrium and avoiding false software damping. Combined cues are bounded
to ±0.12 normalized center travel, 3 units/s slew, and symmetric remaining
center headroom. Native output and idle feature 10 share the resulting center;
Linux native spring and composite constant force use the same disturbance.
There is no second effect slot, new vendor waveform or direct constant command
that disappears on grip release. Existing motor coefficients and caps remain.
Damping-only profiles cannot render spring-center cues.

Missing ground state, ground contact, replay or crash suppresses air cues. Optional
missing wind/attitude or stall arrays disables the corresponding channel.
Near-zero airflow scales cues with `clamp(q / 100 Pa, 0, 1)`. Cue output fades
with a 120 ms time constant on suppression/pause, alongside the existing
one-second whole-force pause fade. Reconnect and aircraft/profile reload reset
history. Updates slower than 20 Hz fade the sampled waveform to avoid aliasing;
invalid timing clears cues. World-wind jumps greater than 20 m/s re-prime the
wind baseline. These filters, thresholds and gains require hardware tuning,
including feature-10 transfer performance during buffet.

## Ground and landing cues

Read `sim/flightmodel/forces/fnrml_gear` (upward gear support, N) and
`sim/flightmodel/weight/m_total` (current mass, kg). Their ratio divided by
9.80665 m/s² gives support in weight units. This is gear loading, not net
occupant acceleration: lift can still support weight at touchdown. Do not
subtract a fixed 1g threshold. `sim/flightmodel/position/P_dot` and `Q_dot`
(degrees/s², positive right roll/nose up) add the aircraft's actual rotational
response. They include all aircraft forces, not just gear moments; current
inertia is already reflected in that response.

Each independent channel removes a 350 ms low-pass mean and filters the
residual with a 30 ms low-pass. Static support settles to zero; compression,
unloading, nose-wheel contact and bounce transients follow the flight model.
Pitch offset is `-gain * (supportResidual + Q_dotResidual / 60)`; roll is
`-gain * P_dotResidual / 60`. The sign and 60 degrees/s² normalization are
provisional tactile mappings, not measured yoke inertia. Inputs are bounded
to ±20 weight units and ±1200 degrees/s² before filtering; output retains the
shared ±0.12 center-travel and 3 units/s slew budget with air cues.

Contact comes from `sim/flightmodel/failures/onground_any`. Moving-ground gain
ramps from zero at 0.5 m/s to `ground_bump_gain` at 2 m/s using `groundspeed`.
After at least 250 ms of observed airborne state, ground contact selects
`landing_bump_gain` for two seconds, independent of forward speed. The larger
of landing and moving-ground gain is used; landing gain fades with a smoothstep
over the final 500 ms. This window covers the initial
main/nose-wheel sequence and short bounces; later jolts still use ground gain.
It changes gain only: contact without a measured transient creates no pulse.
Signals keep updating airborne, preserving real touchdown load changes.

Missing mass/load disables only support cues; missing rotation disables only
that axis. Invalid/recovered channels re-prime independently. Missing ground
state, replay, crash, pause or callbacks slower than 20 Hz fade and re-prime
the ground model. Startup, reconnect, aircraft reload and `AIRPORT_LOADED`
clear history. Ordinary `SCENERY_LOADED` tile streaming does not reset cues.
Optional `local_x/y/z` (metres) and `local_vy` (m/s) detect position jumps:
horizontal displacement over `25 m + 2 * groundspeed * dt`, or vertical
displacement over `25 m + 2 * abs(local_vy) * dt`, re-primes the sample.
The SDK's double coordinates are retained only for this comparison; force
math remains float. Missing position telemetry leaves this jump guard
unavailable; missing ground/vertical speed disables only its horizontal/vertical
comparison. Large genuine forces are capped, not mistaken for relocation.

## Aircraft configuration

`Resources/plugins/g940FF/aircraft.ini` contains an INI `[General]` fallback
and aircraft presets inheriting its values. Selection prefers exact ICAO codes
from `sim/aircraft/view/acf_ICAO`, then filename globs from
`XPLMGetNthAircraftModel(0)`; first match in file order wins within each level.
The tested combined Socata reports `TOBA` and its filename also matches the
Socata preset. Native UTF-8 paths locate the file beside the plugin's `64`
directory. A user-aircraft load message resets forces and primes the new
profile's current center before ramping in; AI load messages do not switch it.
Enabling the plugin reloads the file, while normal callbacks do no file I/O.

| Setting | Values / meaning |
| --- | --- |
| `reference_speed_knots` | `auto` or 1–1000 KEAS; aerodynamic pressure normalization reference |
| `fallback_reference_speed_knots` | 1–1000 KEAS when auto has no usable Vne |
| `mechanical_ratio` | 0–1; stationary spring stiffness fraction, default 0.2 |
| `aerodynamic_gain` | `auto` or 0–4; added spring stiffness at qref; supplied 0.8, omitted/auto retains legacy `1 - mechanical_ratio` |
| `mechanical_damping` | 0–1; pressure-independent movement resistance, default 0.2 |
| `aerodynamic_damping` | 0–4; added damping at qref, default 0.8; total damping capped at 1 |
| `turbulence_gain` | 0–0.1 normalized center displacement per m/s local wind change; General 0, Socata 0.015 |
| `stall_buffet_gain` | 0–0.12 peak normalized pitch-center displacement; General 0, Socata 0.06 |
| `stall_buffet_idle_ratio` | 0–1 fraction of powered buffet at idle; default 0.25 |
| `stall_buffet_hz` | 2–6 Hz main frequency; default 5; second component at 1.37 times this |
| `ground_bump_gain` | 0–0.12 center displacement per transient support weight unit; General 0, Socata 0.02; rotation also scaled by gain |
| `landing_bump_gain` | 0–0.12; same scaling during the two-second touchdown window; General 0, Socata 0.04 |
| `roll_trim_gain`, `pitch_trim_gain` | −10–10; multiplier of live normalized trim |
| `pitch_aoa_deflection_gain` | −15–15 degrees free elevator per degree AoA; default 0.45 |
| `pitch_aoa_gain` | Legacy alias, −1–1 normalized units at 15-degree travel; multiplied by 15; do not specify both gains in one section |
| `neutral_aoa_degrees` | −90–90 degrees; AoA reference for pitch-center offset |
| `elevator_up_degrees`, `elevator_down_degrees` | `auto` or 0.1–90 degrees; simulator travel or 15-degree fallback |
| `static_pitch_trim` | `auto` or −1–1 fraction of elevator travel; simulator static tab or zero fallback |
| `pitch_trim_mode` | `auto`, `aerodynamic`, `spring`, `stabilizer`; auto detects THS travel, otherwise aerodynamic |
| `match_icao` | Comma separated exact aircraft codes; unavailable metadata falls back to filename |
| `match_acf` | Comma separated filename patterns supporting `*` and `?` |

`auto` uses positive finite `sim/aircraft/view/acf_Vne` within 1–1000 knots,
otherwise the configured fallback. Vne is indicated airspeed; treating it as
equivalent airspeed is a low-Mach approximation used for pressure scaling.
This is a provisional tuning heuristic, not a measured control-force reference.
An explicit numeric setting overrides it; live pressure/trim/AoA still come
from X-Plane. Existing trim fractions already include aircraft trim authority,
so the maximum trim-deflection datarefs must not multiply them a second time.
Force-response gains, mechanical resistance and neutral AoA require configuration
and G940 calibration rather than direct aircraft-specification datarefs.

Optional geometry refs are `sim/aircraft/controls/acf_elev_up`, `acf_elev_dn`,
`acf_elev_tab`, `acf_hstb_trim_up` and `acf_hstb_trim_dn`; invalid/missing values
use the defaults above. They are read on profile selection, not every frame.
Stabilizer mode reads `sim/flightmodel2/controls/stabilizer_deflection_degrees`
live; missing incidence falls back to zero, nonfinite incidence stops force.
There is no documented reversible-trim flag dataref, so select `spring` in
the preset explicitly. Existing configuration files remain accepted.
X-Plane 12 pitch/roll axes should be marked `ffb` so it does not also apply its
own trim-center mapping; [setup instructions](../README.md#x-plane-12-control-loaded-axes)
preserve platform-specific axis assignments.

Missing files use built-in General defaults (`auto`, fallback 125 KEAS,
mechanical 0.2, aerodynamic gain 0.8, mechanical/aerodynamic damping 0.2/0.8,
trim gains 1, AoA deflection gain 0.45, neutral AoA zero,
automatic geometry/trim mode). Invalid files
prevent enabling force feedback and log errors with line numbers where applicable.
The source installer preserves existing configuration; ZIP upgrades must keep
the existing file manually. Configuration and simulator metadata cannot raise
the transport's existing maximum coefficients or force caps.

## Probe

`make probe` only reads LED state and grip. Optional LED/spring/constant-force
comparisons require preparation, electronically verify grip coverage, stop
on grip release/interruption and restore LEDs or send zero force. The spring
probe retains the full configured stiffness with 10% of configured caps;
constant force defaults to 4000/32767 and
accepts `--magnitude 1..16384`. Keep X-Plane closed and distinguish firmware
hands-off resistance from live force. Commands are in [README.md](../README.md#test-and-diagnose).
