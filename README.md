# G940-X-Plane

Two X-Plane plugins for Logitech Flight System G940:

- **g940FF** adds airspeed-dependent force feedback, with pitch/roll trim and
  angle of attack influencing the spring center. The force model remains a
  work in progress.
- **g940LEDs** maps aircraft state to the eight throttle button LEDs.

The original implementation supported Linux. Native G940 HID backends now add
Windows and macOS without replacing Linux's evdev and sysfs backends.

| Platform | Architecture | Force feedback | Throttle LEDs |
| --- | --- | --- | --- |
| Linux | x86-64 | Linux evdev; spring or constant-force fallback | G940 sysfs LED driver |
| Windows | x86-64 | G940 vendor HID output reports | G940 HID feature reports |
| macOS 11+ | Intel and Apple Silicon, universal binary | G940 vendor HID output reports | G940 HID feature reports |

Both plugins build on all three platforms. macOS has been tested with a connected
G940, and both plugins loaded and connected in X-Plane 12 on Apple Silicon.
In-flight testing confirmed that the LEDs respond to flaps and landing lights.
No force feedback was felt during the initial flight. Subsequent bench testing
confirmed pitch constant force and spring centering at higher levels, with zero
force stopping both effects. A subsequent flight confirmed force feedback but
revealed pitch-trim kicks. Keeping trim targets independent of stick motion
and smoothing target changes produced a subsequent flight without trim kicks.
Flight testing confirmed smooth trim and a smooth one-second pause release.
The latest roll increase felt too strong. The current candidate reduces roll
strength, retains gentle centered ground resistance, and mirrors the trimmed
spring into native hands-off centering. This should retain the stick's resting
position when the grip sensor is uncovered, allowing trim-only hands-off flight
without switching to an unrelated center. Original idle profiles and centers
are restored on disable or normal exit. A bench comparison confirmed unchanged
resting position and resistance with no grip-release kick. Trim-only hands-off
flight and the revised roll balance still need a flight check; see the
[hardware test findings](tools/HARDWARE_TESTS.md).
Windows and Linux builds have been cross-compiled, but the new Windows
backend still needs hardware testing on Windows. macOS/Windows support is
G940-specific; Linux force feedback may also work with other evdev devices.

Linux LEDs require the original author's
[G940 kernel patches](https://github.com/chrisboyle/G940-linux) or equivalent
driver support, plus write permission to the LED brightness files. Force feedback
requires read/write permission to the joystick's `/dev/input/event*` device.

## Build and install

```sh
make
make test
make install XP_INSTALL_PATH="/path/to/X-Plane 12"
```

The build downloads the [official X-Plane SDK](https://developer.x-plane.com/sdk/plugin-sdk-downloads/)
if it is missing. It retains the XPLM 3.0.1 API target for X-Plane 11.20+ and
X-Plane 12. No locally implemented XPLM stubs are linked into the plugins.

The output is `build/g940FF/64/` and `build/g940LEDs/64/`, with
`lin.xpl`, `win.xpl`, or `mac.xpl` for the selected platform. Platform objects
are stored separately, so building another OS does not overwrite an existing
OS's plugin. Each build creates both plugins.

See [INSTALL.md](INSTALL.md) for prerequisites, Windows setup, SDK overrides,
installation hints, and hardware testing.

## LED mapping

Map buttons P1-P8 to these X-Plane actions to match their indicator colours:

| P1 | P2 | P3 | P4 |
| --- | --- | --- | --- |
| Speedbrakes: retract one | Flaps: retract one | Carb heat: toggle | Autopilot: servos toggle |
| **P5** | **P6** | **P7** | **P8** |
| Speedbrakes: extend one | Flaps: extend one | Landing light: toggle | Landing gear: toggle |

Red indicates the low/off state, green the high/on state, and amber an
intermediate position. Indicators for absent aircraft equipment are off.
The mapping is currently fixed in `g940LEDs.cpp`.

The plugins stop their flight loops and release hardware when disabled. Force
output also stops when the simulator pauses. Missing/disconnected hardware is
retried every five seconds and reported in X-Plane's `Log.txt`.
