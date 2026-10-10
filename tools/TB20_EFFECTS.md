# TB20 tactile cues: evidence and limits

This research guides experimental G940 feedback. It is not a measured TB20
yoke-force model. The sources describe aircraft handling qualitatively; they
do not provide control-force spectra or a calibrated mapping to joystick torque.

## Manufacturer evidence

- [SOCATA TB20 Pilot's Information Manual, section 4.21, PDF page 83](https://s24b6e027ad256e5c.jimcontent.com/download/version/1435869037/module/6697101954/name/TB20PIM.pdf#page=83)
  describes natural pre-stall buffet as weak at idle and more pronounced with
  higher power. Near aft CG, wing rocking or a wing drop can occur when the
  stabilator approaches its stop. Buffet is an airflow cue, not a guaranteed
  abrupt control kick.
- [The same manual, section 7.57, PDF page 301](https://s24b6e027ad256e5c.jimcontent.com/download/version/1435869037/module/6697101954/name/TB20PIM.pdf#page=301)
  describes a left-wing leading-edge vane connected to a continuous aural tone
  through the alarm speaker. The documented warning margin is 5–10 knots above
  stall across configurations. This is a horn, not an artificial stick shaker;
  its activation alone does not establish buffet intensity.
- [The same manual, sections 7.6–7.7, PDF pages 224–225](https://s24b6e027ad256e5c.jimcontent.com/download/version/1435869037/module/6697101954/name/TB20PIM.pdf#page=224)
  identifies an all-moving stabilator with an anti-servo tab, and rods/bellcranks
  connecting the control wheel to the ailerons and stabilator. This mechanical
  coupling distinguishes control feedback from shaking the cockpit image.
- [SOCATA Trinidad Comprehensive Guide, January 2000, printed pages 8–10](https://www.peter2000.co.uk/aviation/tb20-experience/TB20_Comprehensive_guide.pdf#page=8)
  associates high wing loading with a smoother rough-air ride and turbulence
  stability, and describes gentle stalls with useful low-speed controllability.
  These are manufacturer marketing claims, not measured force limits.

The PIM is a historical manufacturer publication hosted by a third party. Its
section numbers differ from other TB20 revisions; use the linked edition for
the references above.

## Pilot corroboration

[A TB20GT owner's direct operating account](https://www.peter2000.co.uk/aviation/tb20-experience/index.html)
reports performing chandelles near stall buffet while retaining effective
control surfaces. This corroborates natural buffet and continued control
authority, but supplies no force amplitude or frequency. It should not be read
as evidence that every loading or uncoordinated stall is benign.

## Implications for the plugin

X-Plane remains responsible for stalls, lift loss, wing drops and the aircraft's
response to turbulent air. The plugin only translates simulator signals into
bounded tactile cues; it does not recreate flight physics or change control
positions directly. Turbulence feedback should follow disturbed-air signals,
rather than add random shaking whenever a weather setting is nonzero.

The TB20 preset should represent natural buffet, with a gradual, power-dependent
envelope. The horn is not a buffet trigger, and this research does not justify
an artificial shaker, roll impulse or complete unloading at stall. Frequency,
amplitude, onset, filtering and signal-to-force gains remain configurable
simulator tuning assumptions requiring later hardware and flight comparison.
None is a measured TB20 value.

The tested combined `JF_Socata_TB10+TB20.acf` reports conventional elevator
metadata (symmetric travel, static tab bias and no stabilizer trim travel),
whereas the real TB20 uses a stabilator. See [the recorded metadata](HARDWARE_TESTS.md#experimental-realism-model).
This geometry mismatch limits the present trim/airflow-force approximation;
buffet cues do not correct it. Preserve the model's actual data rather than
silently impose different control geometry.

## Implemented tactile model

The shared Socata TB10/TB20 preset now enables TB20-based provisional cues;
separate TB10 tuning has not been established. General leaves both effects off.
The preset uses `turbulence_gain = 0.015` center travel per m/s wind disturbance
and `stall_buffet_gain = 0.06` peak pitch-center travel. Idle buffet is 25% of
powered buffet, using actual engine watts relative to maximum configured power.
The main waveform starts at 5 Hz with a smaller second component at 6.85 Hz.
All four settings can be changed in `aircraft.ini`; these values are not from
the manual and have not been felt on the disconnected G940.

Gust feedback follows filtered changes in X-Plane's world-coordinate local wind,
then rotates into body axes. A turn in steady wind produces no cue. Natural
buffet follows the area-weighted separation of the main wing's elements, with
a smooth power-dependent envelope. No vibration is generated merely because
the stall horn sounds. The model cannot reconstruct pre-stall buffet before
X-Plane reports any element separation, nor every wake/thermal disturbance.

Cues perturb the spring center after slow trim filtering, with symmetric travel
headroom, bounded movement and the existing force caps. The same center is
sent to the native live and hands-off channels and Linux force paths. Ground,
pause, replay, missing telemetry and slow updates suppress or fade cues; no
aircraft control or weather value is overwritten. See the
[protocol](PROTOCOL.md#gust-and-natural-buffet-cues) for exact signals and filters.

Simulator references: [main-wing indices](https://developer.x-plane.com/2013/05/using-the-right-wing-datarefs/),
[coordinate transforms](https://developer.x-plane.com/article/screencoordinates/),
[local weather samples](https://developer.x-plane.com/article/weather-datarefs-in-x-plane-12/)
and [array-read counts](https://developer.x-plane.com/sdk/XPLMGetDatavf/).

Software tests cover steady wind, turns and gust direction, area weighting,
tail exclusion, horn-only warnings, power scaling, missing/short/invalid data,
frame delays, pause/reconnect, trim headroom and backend parity. Later tests
must assess actual feel, sign, strength, timing, transfer rate and grip release.
No physical test was run and the earlier grip-release kick remains unresolved.

## Ground and landing cues

Ground roughness and landing impacts follow X-Plane's actual gear support force,
normalized by current aircraft mass, plus measured roll/pitch acceleration.
This captures the model's response to its gear, loading and surface conditions
rather than imposing a generic runway vibration or a fixed touchdown kick.
Gradual support transfer produces a smaller transient than a sharp change;
static parked weight produces none. Landing gain stays active briefly for the
main/nose-wheel sequence and short bounces. No pulse is generated from a
ground-contact flag alone.

Laminar documents [force units and rotation signs](https://developer.x-plane.com/article/movingtheplane/)
and [aircraft body axes](https://www.x-plane.com/kb/data-set-output-table/).
The installed X-Plane 11/12 dataref registries confirm `fnrml_gear` in newtons,
`m_total` in kilograms and `P_dot`/`Q_dot` in degrees/s². Measured angular
acceleration avoids inferring current inertia from static aircraft metadata;
it includes aerodynamic and other forces as well as gear reaction.

Support force is not net cockpit acceleration, and actual yoke vibration
depends on mechanical coupling and pilot grip. The chosen filters, signs,
rotation scaling and Socata gains (`ground_bump_gain = 0.02`,
`landing_bump_gain = 0.04`) are provisional simulator tuning. They are not
manufacturer measurements or a calibrated TB20 suspension/control-column
model. General leaves these effects disabled. Software checks cannot confirm
their physical feel while the G940 is disconnected.

The cues share the existing motor caps, trim headroom and live/idle center
mapping with air cues. Startup, pause/replay, reloads and detected position
jumps re-prime history. Missing telemetry suppresses its own channel; ordinary
force feedback remains available. See [exact filters and guards](PROTOCOL.md#ground-and-landing-cues).
