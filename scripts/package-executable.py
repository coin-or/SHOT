#!/usr/bin/env python3
"""Package the native executable and libraries from a repaired CPython 3.11 wheel.

Keep Unix paths intact: auditwheel/delocate have already rewritten library
references relative to this layout. On Windows, put every DLL beside SHOT.exe
so the native loader works without delvewheel's Python initialization.
"""

import argparse
import hashlib
import os
from pathlib import Path, PurePosixPath
import shutil
import tempfile
import zipfile


PLATFORMS = {
    "linux-x86_64": ("manylinux_", "x86_64"),
    "linux-aarch64": ("manylinux_", "aarch64"),
    "macos-arm64": ("macosx_", "arm64"),
    "windows-x86_64": ("win_", "amd64"),
}


def package(wheel, platform, output, msvc_runtime=None):
    _, version, *_, tags = wheel.stem.split("-")
    prefix, architecture = PLATFORMS[platform]
    if not all(tag.startswith(prefix) and tag.endswith("_" + architecture) for tag in tags.split(".")):
        raise ValueError(f"Wheel {wheel.name} does not match {platform}")
    windows = platform.startswith("windows-")
    executable = "SHOT.exe" if windows else "SHOTpy_bin/SHOT"
    name = f"SHOT-{version}-{platform}"
    output.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary) / name
        root.mkdir()
        with zipfile.ZipFile(wheel) as archive:
            members = {item.filename for item in archive.infolist()}
            if executable not in members:
                raise ValueError(f"{wheel.name} has no {executable}")
            for item in archive.infolist():
                path = PurePosixPath(item.filename)
                if path.is_absolute() or ".." in path.parts:
                    raise ValueError(f"Invalid wheel member: {path}")
                if item.is_dir():
                    continue
                # Keep wheel license files, including licenses added by repair tools.
                license_file = any(part.endswith(".dist-info") for part in path.parts) and "licenses" in path.parts
                library = (".so" in path.name or path.suffix.lower() in (".dll", ".dylib"))
                if path.name.lower().startswith("shotpy."):
                    library = False  # The Python extension is not needed by the CLI.
                if license_file:
                    relative = Path("licenses", *path.parts)
                elif item.filename == executable or library:
                    relative = Path(path.name) if windows else Path(*path.parts)
                else:
                    continue
                destination = root / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                data = archive.read(item)
                if destination.exists() and destination.read_bytes() != data:
                    raise ValueError(f"Conflicting libraries at {relative}")
                destination.write_bytes(data)
                destination.chmod(0o755 if library or item.filename == executable else 0o644)

        if not (root / "licenses").is_dir():
            raise ValueError(f"{wheel.name} contains no license directory")
        if windows and msvc_runtime is not None:
            # delvewheel excludes these because Python normally supplies them.
            # Use the redistributables matching the compiler used for the wheel.
            for runtime_name in ("vcruntime140.dll", "vcruntime140_1.dll"):
                shutil.copyfile(msvc_runtime / runtime_name, root / runtime_name)
        if not windows:
            launcher = root / "SHOT"
            launcher.write_text('#!/bin/sh\nexec "$(dirname "$0")/SHOTpy_bin/SHOT" "$@"\n')
            launcher.chmod(0o755)
        command = "SHOT.exe" if windows else "./SHOT"
        (root / "README.txt").write_text(
            f"SHOT {version} ({platform})\n\n"
            f"Run {command} problem.osil (or an AMPL .nl file).\n"
            "Keep the entire extracted directory together; its libraries are required.\n"
            "Python and pip are not required. Includes HiGHS, Cbc, and Ipopt.\n"
            "Commercial solvers and GAMS are not included.\n"
            "Linux requires glibc 2.28 or newer; macOS requires 15 or newer.\n\n"
            "Documentation: https://shotsolver.dev/shot/\n"
            "Source and dependency versions: https://github.com/coin-or/SHOT\n"
            "Licenses are provided in licenses/.\n"
        )
        archive_format = "zip" if windows else "gztar"
        result = Path(shutil.make_archive(str(output / name), archive_format, temporary, name))
    checksum = hashlib.sha256(result.read_bytes()).hexdigest()
    result.with_name(result.name + ".sha256").write_text(f"{checksum}  {result.name}\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wheel-dir", type=Path, default=Path("wheelhouse"))
    parser.add_argument("--platform", choices=PLATFORMS, required=True)
    parser.add_argument("--output", type=Path, default=Path("executables"))
    args = parser.parse_args()
    # The executable has no CPython ABI dependency; one wheel per platform suffices.
    wheels = sorted(args.wheel_dir.glob("*-cp311-cp311-*.whl"))
    if len(wheels) != 1:
        parser.error(f"Expected one repaired CPython 3.11 wheel, found {len(wheels)}")
    msvc_runtime = None
    if args.platform.startswith("windows-"):
        redist = os.environ.get("VCToolsRedistDir")
        if not redist:
            parser.error("Run in an MSVC developer shell (VCToolsRedistDir is required)")
        directories = list((Path(redist) / "x64").glob("Microsoft.VC*.CRT"))
        if len(directories) != 1:
            parser.error(f"Expected one MSVC x64 runtime directory in {redist}")
        msvc_runtime = directories[0]
    print(package(wheels[0], args.platform, args.output.resolve(), msvc_runtime))


if __name__ == "__main__":
    main()
