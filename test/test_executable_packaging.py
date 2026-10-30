"""Checks for platform layout, relocated archives, and wheel validation."""

import hashlib
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import unittest
import zipfile


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("package_executable", ROOT / "misc/package-executable.py")
PACKAGER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PACKAGER)


class ExecutablePackagingTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def wheel(self, tag, files):
        path = self.root / f"shotpy-2.0.0rc1-cp311-cp311-{tag}.whl"
        with zipfile.ZipFile(path, "w") as archive:
            for name, contents in {
                "shotpy-2.0.0rc1.dist-info/licenses/LICENSE": "SHOT license",
                "shotpy-2.0.0rc1.dist-info/licenses/solvers/CBC/COPYING": "Cbc license",
                "SHOTpy_bin/__init__.py": "unneeded Python launcher",
                "SHOTpy.cpython-311.so": "unneeded Python module",
                **files,
            }.items():
                archive.writestr(name, contents)
        return path

    def test_unix_layout_and_modes_survive_archive(self):
        for platform, tag, library in (
            ("linux-x86_64", "manylinux_2_28_x86_64", "shotpy.libs/libipopt-123.so.3"),
            ("linux-aarch64", "manylinux_2_28_aarch64", "shotpy.libs/libipopt-123.so.3"),
            ("macos-arm64", "macosx_14_0_arm64", ".dylibs/libipopt.3.dylib"),
        ):
            with self.subTest(platform=platform):
                wheel = self.wheel(tag, {"SHOTpy_bin/SHOT": "native executable", library: "solver"})
                archive = PACKAGER.package(wheel, platform, self.root / platform)
                checksum = archive.with_name(archive.name + ".sha256").read_text().split()[0]
                self.assertEqual(hashlib.sha256(archive.read_bytes()).hexdigest(), checksum)
                with tarfile.open(archive) as result:
                    members = {"/".join(Path(m.name).parts[1:]): m for m in result.getmembers()}
                    self.assertIn(library, members)
                    if os.name != "nt":  # Windows chmod only controls the read-only bit.
                        self.assertEqual(members["SHOTpy_bin/SHOT"].mode, 0o755)
                        self.assertEqual(members["SHOT"].mode, 0o755)
                    self.assertIn("licenses/shotpy-2.0.0rc1.dist-info/licenses/solvers/CBC/COPYING", members)
                    self.assertNotIn("SHOTpy.cpython-311.so", members)
                    self.assertNotIn("SHOTpy_bin/__init__.py", members)

    @unittest.skipIf(os.name == "nt", "Requires a POSIX shell")
    def test_unix_launcher_after_relocation_with_spaces(self):
        wheel = self.wheel("macosx_14_0_arm64", {"SHOTpy_bin/SHOT": '#!/bin/sh\nprintf "%s\\n" "$1"\n'})
        archive = PACKAGER.package(wheel, "macos-arm64", self.root / "output")
        destination = self.root / "relocated with spaces"
        shutil.unpack_archive(archive, destination)
        executable, = destination.glob("*/SHOT")
        result = subprocess.run([str(executable), "argument with spaces"], cwd=self.root,
                                capture_output=True, text=True, check=True)
        self.assertEqual(result.stdout, "argument with spaces\n")

    def test_windows_dlls_beside_executable(self):
        wheel = self.wheel("win_amd64", {"SHOT.exe": "exe", "SHOTSolver.dll": "solver",
                                          "shotpy.libs/libipopt-hash.dll": "ipopt",
                                          "SHOTpy.cp311-win_amd64.pyd": "Python module"})
        runtime = self.root / "msvc-runtime"
        runtime.mkdir()
        for name in ("vcruntime140.dll", "vcruntime140_1.dll"):
            (runtime / name).write_bytes(b"redistributable")
        archive = PACKAGER.package(wheel, "windows-x86_64", self.root / "output", runtime)
        with zipfile.ZipFile(archive) as result:
            names = {"/".join(Path(name).parts[1:]) for name in result.namelist()}
            self.assertIn("SHOT.exe", names)
            self.assertIn("libipopt-hash.dll", names)
            self.assertIn("vcruntime140.dll", names)
            self.assertIn("vcruntime140_1.dll", names)
            self.assertNotIn("shotpy.libs/libipopt-hash.dll", names)
            self.assertNotIn("SHOTpy.cp311-win_amd64.pyd", names)

    def test_conflicting_windows_dlls_fail(self):
        wheel = self.wheel("win_amd64", {"SHOT.exe": "exe", "ipopt.dll": "one",
                                          "shotpy.libs/ipopt.dll": "different"})
        with self.assertRaisesRegex(ValueError, "Conflicting libraries"):
            PACKAGER.package(wheel, "windows-x86_64", self.root / "output")

    def test_wrong_architecture_or_missing_executable_fails(self):
        wheel = self.wheel("manylinux_2_28_x86_64", {})
        with self.assertRaisesRegex(ValueError, "does not match"):
            PACKAGER.package(wheel, "linux-aarch64", self.root / "output")
        with self.assertRaisesRegex(ValueError, "has no SHOTpy_bin/SHOT"):
            PACKAGER.package(wheel, "linux-x86_64", self.root / "output")


if __name__ == "__main__":
    unittest.main()
