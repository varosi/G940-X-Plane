"""Install both plugins using an explicit path or X-Plane's install hints."""
import argparse
from datetime import datetime
import os
from pathlib import Path
import shutil
import sys
import tempfile

PLUGINS = ("g940FF", "g940LEDs")
FILENAMES = {"mac": "mac.xpl", "linux": "lin.xpl", "windows": "win.xpl"}
CONFIG_FILES = (("g940FF", "aircraft.ini"),)


def build_files(build_dir, platform):
    """Return all bundle files and whether installation preserves existing copies."""
    files = [(Path(plugin) / "64" / FILENAMES[platform], False) for plugin in PLUGINS]
    files += [(Path(plugin) / filename, True) for plugin, filename in CONFIG_FILES]
    sources = [(Path(build_dir) / relative, relative, preserve) for relative, preserve in files]
    for source, _, _ in sources:
        if not source.is_file():
            raise ValueError(f"Plugin bundle file missing: {source}")
    return sources


def hint_files(home, platform, version):
    if platform == "mac":
        folders = [home / "Library/Preferences", home, home / ".x-plane"]
    elif platform == "windows":
        folders = [Path(os.environ.get("LOCALAPPDATA", home / "AppData/Local")), home]
    else:
        folders = [home / ".x-plane", home]
    return [folder / f"x-plane_install_{version}.txt" for folder in folders]


def find_installation(platform, explicit=None, hint=None, home=None):
    if explicit:
        path = Path(explicit).expanduser().resolve()
        if not (path / "Resources/plugins").is_dir():
            raise ValueError(f"X-Plane Resources/plugins directory not found: {path}")
        return path
    home = Path.home() if home is None else Path(home)
    for version in (12, 11):
        candidates = []
        files = [Path(hint).expanduser()] if hint else hint_files(home, platform, version)
        for file in files:
            if not file.is_file():
                continue
            for line in file.read_text(encoding="utf-8-sig").splitlines():
                line = line.strip().strip('"')
                if not line:
                    continue
                path = Path(line).expanduser().resolve()
                if (path / "Resources/plugins").is_dir() and path not in candidates:
                    candidates.append(path)
        if len(candidates) == 1:
            return candidates[0]
        if len(candidates) > 1:
            raise ValueError("Multiple X-Plane installations found; set XP_INSTALL_PATH explicitly: " +
                             ", ".join(str(path) for path in candidates))
        if hint:
            break
    raise ValueError("X-Plane not detected. Run make install XP_INSTALL_PATH=\"/path/to/X-Plane 12\".")


def install(build_dir, platform, destination):
    sources = build_files(build_dir, platform)
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    for source, relative, preserve in sources:
        target = destination / "Resources/plugins" / relative
        if preserve and target.exists():
            print(f"Kept existing configuration: {target}")
            continue
        folder = target.parent
        filename = target.name
        folder.mkdir(parents=True, exist_ok=True)
        if target.exists():
            backup = target.with_name(f"{filename}.backup-{timestamp}")
            shutil.copy2(target, backup)
            print(f"Backup: {backup}")
        with tempfile.NamedTemporaryFile(dir=folder, prefix=f".{filename}-", delete=False) as file:
            temporary = Path(file.name)
        try:
            shutil.copy2(source, temporary)
            temporary.replace(target)
        finally:
            temporary.unlink(missing_ok=True)
        print(f"Installed: {target}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", default="build")
    parser.add_argument("--platform", choices=FILENAMES, required=True)
    parser.add_argument("--x-plane")
    parser.add_argument("--hint-file")
    args = parser.parse_args()
    try:
        destination = find_installation(args.platform, args.x_plane, args.hint_file)
        install(args.build_dir, args.platform, destination)
    except (OSError, ValueError) as error:
        print(f"Installation failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
