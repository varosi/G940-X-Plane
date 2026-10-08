# Installation

## Prerequisites

Use GNU make, a C++11 compiler, `curl`, `unzip`, and Python 3.8 or newer.
The plugins are 64-bit and target X-Plane 11.20+ and X-Plane 12.

- **macOS:** install Xcode Command Line Tools with `xcode-select --install`.
  The default build uses Apple's Clang and produces an Intel/Apple Silicon
  universal binary for macOS 11 or later.
- **Linux:** install GCC/G++, make, curl, unzip, Python 3, and Linux input
  development headers (normally included by the distribution).
- **Windows:** use the MSYS2 **UCRT64** shell. Install dependencies there:

  ```sh
  pacman -S --needed make curl unzip mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-python
  ```

Windows force feedback uses HID interrupt output through `WriteFile`; LEDs use
HID feature control transfers to the G940.
Avoid running another program that writes G940 force or LED reports at the same
time. Linux retains the original evdev force-feedback support; throttle LEDs
require the [G940 kernel patches](https://github.com/chrisboyle/G940-linux) or
a driver exposing equivalent `/sys/class/leds/g940:*` brightness files.

## Build

From the project directory:

```sh
make -j2
make test
```

The host OS is detected automatically. Explicit targets are:

```sh
make PLATFORM=linux
make PLATFORM=windows
make PLATFORM=mac
```

Selecting a platform does not supply a cross compiler or system headers.
For cross builds, also set `CXX` to an appropriate toolchain. Examples using
a separately installed Zig toolchain:

```sh
make PLATFORM=linux CXX="zig c++ -target x86_64-linux-gnu.2.31"
make PLATFORM=windows CXX="zig c++ -target x86_64-windows-gnu"
```

The build downloads SDK 4.3.0 from Laminar Research if `SDK` is missing.
The API target stays at XPLM 3.0.1; newer SDK headers do not require a newer
simulator unless newer API macros/functions are enabled. An existing official
SDK can be symlinked to `SDK` or selected with `SDK_DIR=/path/to/SDK`.
Apple Silicon builds require the universal XPLM framework from SDK 4.x.

Do not use handwritten replacement SDK headers or link `xplm_stubs.cpp` into
a plugin. macOS links the official XPLM framework, Windows the SDK's import
library, and Linux resolves XPLM symbols from the simulator.

| Platform | Force plugin | LED plugin |
| --- | --- | --- |
| Linux | `build/g940FF/64/lin.xpl` | `build/g940LEDs/64/lin.xpl` |
| Windows | `build/g940FF/64/win.xpl` | `build/g940LEDs/64/win.xpl` |
| macOS | `build/g940FF/64/mac.xpl` | `build/g940LEDs/64/mac.xpl` |

For a single macOS architecture, use `make MAC_ARCHS=arm64` or
`make MAC_ARCHS=x86_64`. Plain `make` restores the universal build.
Run `make clean` before changing compilers, SDK paths, or compiler flags.
The `build/previous-sdk-stubs` backup is preserved by clean.

## Install

Close X-Plane, then pass its installation directory explicitly:

```sh
make install XP_INSTALL_PATH="/Users/YourUsername/Applications/X-Plane 12"
```

On Windows, a forward-slash path works from UCRT64, for example
`XP_INSTALL_PATH="C:/X-Plane 12"`.

Both plugins are copied into `Resources/plugins/<plugin>/64/`.
The installer backs up existing binaries as `<platform>.xpl.backup-<timestamp>`
and preserves binaries for other platforms.

Plain `make install` searches X-Plane 12 install hints first, then X-Plane 11:

| Platform | Primary hint location |
| --- | --- |
| macOS | `~/Library/Preferences/x-plane_install_12.txt` |
| Linux | `~/.x-plane/x-plane_install_12.txt` |
| Windows | `%LOCALAPPDATA%/x-plane_install_12.txt` |

It also accepts a hint in your home directory, skips stale paths, and handles
multiple lines and spaces. If several valid installations are found, use
`XP_INSTALL_PATH` to select one. For a custom hint, use
`make install HINTFILE=/path/to/hint.txt`.

Restart X-Plane, load a flight, and check `Log.txt` for these messages:

```text
Loaded: .../g940FF/64/mac.xpl (name.boyle.chris.xpff).
Loaded: .../g940LEDs/64/mac.xpl (name.boyle.chris.xpg940leds).
G940 FF: force-feedback device connected
G940 LEDs: LED device connected
```

The filename changes on Linux/Windows. Loading the library confirms the SDK
link; the connection messages confirm that a flight loop opened the hardware.

## Hardware tests

With X-Plane closed, on macOS or Windows:

```sh
make probe
build/tools/g940_probe --led-test --seconds 10
build/tools/g940_probe --force-test --seconds 10
build/tools/g940_probe --pitch-test --seconds 10
build/tools/g940_probe --pitch-test --reverse --seconds 10
```

The first command only reads device information and the grip sensor. Each test
prints a preparation warning and waits for Enter. The LED test displays red,
green, amber, off across P1-P4 and P5-P8, checks the report readback, and restores
the previous colours. The force test compares a centered spring at 10% saturation
with zero force. The pitch test compares a gentle constant force on the
forward/backward axis with zero force; `--reverse` reverses its direction.
`--roll-test` does the same on the sideways axis. Each force comparison starts
after two seconds of continuous grip-sensor coverage and stops if the grip is
released. Hold the grip throughout both stages and move it gently to check
resistance. Keep the hand sensor covered: firmware centering with the hand
removed does not confirm that live force feedback works.
The G940 needs motor power for force feedback. Successful USB commands alone
do not establish the force's physical direction or magnitude.

Current macOS flight testing confirms LED changes for flaps and landing lights,
but no force feedback was felt during the initial flight. Subsequent bench tests
confirmed live pitch constant force at 16000/32767 and pitch spring centering at
50% saturation; both effects stopped with zero force. Lower-level comparisons
were not felt, including direct USB transfers. A subsequent flight confirmed
force feedback and exposed pitch-trim kicks. The updated spring model separates
the trim target from stick motion, limits target changes, and adds light native
G940 damping. A subsequent flight confirmed smoother trim without kicks and
force release when paused with the grip covered. The next flight confirmed
smooth trim and a smooth one-second pause release, but requested more force
and reported a kick when uncovering the grip. The latest candidate uses
pitch coefficient 127, the highest positive value, and disables native idle
centering while the plugin is enabled and connected. Both original idle
settings are backed up before either is changed, read back after writes,
kept across pauses, and restored on disable or normal exit. This affects the
macOS/Windows backend; Linux retains its evdev lifecycle. Physical validation
of the latest strength and grip-release behavior is pending; see the
[hardware test findings](tools/HARDWARE_TESTS.md).

For an optional diagnostic build, use a separate build directory so ordinary
and diagnostic objects are not mixed:

```sh
make BUILDDIR=build/force-debug CPPFLAGS=-DG940_DEBUG_FORCE=1
make install BUILDDIR=build/force-debug CPPFLAGS=-DG940_DEBUG_FORCE=1 XP_INSTALL_PATH="/path/to/X-Plane 12"
```

This logs `G940 FF trace:` lines in X-Plane's `Log.txt` every two seconds while
the force device is connected and the simulator is unpaused. The lines record
airspeed, aircraft Vne, the calculated speed ratio, yoke inputs, trim, angle of
attack, spring centers, and each axis's spring saturation after the strength
adjustment. This adds logging without changing the force model.
Diagnostic builds also log when the one-second pause fade starts and when
force is released.
To return to an ordinary build, install from the default `build` directory.

In a loaded flight, test flap and landing-light changes, then compare stick
resistance at low and higher airspeed. Pause the simulator and disable/re-enable
each plugin through Plugin Admin to check that forces stop and indicators
resume. Unplug/reconnect the G940 to check the five-second retry.

## Troubleshooting

- **SDK download failure:** download the official SDK ZIP manually from
  [Laminar Research](https://developer.x-plane.com/sdk/plugin-sdk-downloads/)
  and extract its `SDK` folder here.
- **X-Plane not detected:** supply `XP_INSTALL_PATH`; it must contain
  `Resources/plugins`.
- **Plugin does not load:** inspect `Log.txt`. On macOS use
  `lipo -archs build/g940FF/64/mac.xpl` to confirm the required architecture.
- **Device access fails:** inspect the plugin's logged error. On Linux, check
  evdev/sysfs permissions and LED driver support.
- **No forces while stationary or paused:** the model scales force by airspeed
  relative to Vne and fades force output over about one second when paused.
- **No resistance in the standalone force test:** verify motor power and hold
  the grip. Report the test output and whether the LEDs work; Windows hardware
  operation still needs validation on a Windows system.

`make test` checks report encoding, invalid force inputs, dataref types,
plugin lifecycle/reconnection, and installer behavior without moving hardware.
The GitHub Actions workflow is configured to build and run these tests on Linux,
macOS, and Windows; it does not perform physical hardware tests.
