"""Create an installable ZIP containing both plugins for one platform."""
import argparse
from pathlib import Path
import sys
from zipfile import ZIP_DEFLATED, ZipFile

from tools.install import FILENAMES, build_files


def package(build_dir, platform, output):
    sources = build_files(build_dir, platform)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(output, "w", compression=ZIP_DEFLATED) as archive:
        for source, relative, _ in sources:
            archive.write(source, "Resources/plugins/" + relative.as_posix())
        archive.write(Path(__file__).resolve().parents[1] / "LICENSE", "LICENSE")
        archive.writestr("INSTALL.txt", "Close X-Plane and back up existing G940 plugins.\n"
                          "Copy this archive's Resources folder into your X-Plane 11/12 folder,\n"
                          "merging folders and replacing only the matching .xpl files.\n"
                          "Keep your existing g940FF/aircraft.ini when upgrading.\n"
                          "New installations include General and Socata TB10/TB20 presets.\n"
                          "For X-Plane 12 control loading, export the G940's .joy defaults,\n"
                          "then close X-Plane and add ffb to the pitch/roll axis assignments.\n"
                          "Keep their existing numbers and reverse flags, then restart and\n"
                          "apply Reset to Defaults for the G940. Back up the .joy file first.\n"
                          "Details: https://developer.x-plane.com/article/types-of-flight-control-trim/\n"
                          "Start X-Plane and check Log.txt for g940FF and g940LEDs.\n"
                          "Linux requires writable evdev devices and a G940 LED sysfs driver.\n"
                          "See https://github.com/chrisboyle/G940-X-Plane for documentation.\n")
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", default="build")
    parser.add_argument("--platform", choices=FILENAMES, required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    try:
        print(package(args.build_dir, args.platform, args.output))
    except (OSError, ValueError) as error:
        print(f"Packaging failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
