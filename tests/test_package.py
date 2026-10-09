from pathlib import Path
import tempfile
import unittest
from zipfile import ZipFile

from tools.install import FILENAMES, PLUGINS
from tools.package import package


class PackageTests(unittest.TestCase):
    def test_each_platform_extracts_into_xplane(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for plugin in PLUGINS:
                folder = root / "build" / plugin / "64"
                folder.mkdir(parents=True)
                for filename in FILENAMES.values():
                    (folder / filename).write_bytes(filename.encode())
            config_text = "[General]\nreference_speed_knots=auto\n"
            (root / "build/g940FF/aircraft.ini").write_text(config_text, encoding="utf-8")
            for platform, filename in FILENAMES.items():
                with self.subTest(platform=platform):
                    output = package(root / "build", platform, root / "dist" / f"{platform}.zip")
                    with ZipFile(output) as archive:
                        expected = {f"Resources/plugins/{plugin}/64/{filename}" for plugin in PLUGINS}
                        self.assertEqual(set(archive.namelist()), expected | {
                            "Resources/plugins/g940FF/aircraft.ini", "LICENSE", "INSTALL.txt"})
                        self.assertIsNone(archive.testzip())
                        destination = root / "X-Plane 12" / platform
                        archive.extractall(destination)
                        for path in expected:
                            self.assertEqual((destination / path).read_bytes(), filename.encode())
                        self.assertEqual((destination / "Resources/plugins/g940FF/aircraft.ini").read_text(), config_text)
                        self.assertIn("Keep your existing", archive.read("INSTALL.txt").decode())

    def test_incomplete_build_leaves_existing_archive_intact(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "build/g940FF/64"
            source.mkdir(parents=True)
            (source / "mac.xpl").write_bytes(b"only one plugin")
            output = root / "existing.zip"
            output.write_bytes(b"previous archive")
            with self.assertRaisesRegex(ValueError, "g940LEDs"):
                package(root / "build", "mac", output)
            self.assertEqual(output.read_bytes(), b"previous archive")

    def test_missing_config_leaves_existing_archive_intact(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for plugin in PLUGINS:
                source = root / "build" / plugin / "64"
                source.mkdir(parents=True)
                (source / "mac.xpl").write_bytes(b"plugin")
            output = root / "existing.zip"
            output.write_bytes(b"previous archive")
            with self.assertRaisesRegex(ValueError, "aircraft.ini"):
                package(root / "build", "mac", output)
            self.assertEqual(output.read_bytes(), b"previous archive")
