#!/usr/bin/env python3
"""Apply or check the pinned formatters on tracked, project-owned source files."""

import argparse
from importlib import metadata
import os
from pathlib import Path
import subprocess
import sys
import sysconfig
import venv


ROOT = Path(__file__).resolve().parents[1]


def environment_python():
    directory = ROOT / "build" / "format-venv"

    if os.name == "nt":
        return directory / "Scripts" / "python.exe"

    return directory / "bin" / "python"


def setup_environment():
    python = environment_python()

    if not python.is_file():
        print("Creating formatter environment in build/format-venv", flush=True)
        venv.EnvBuilder(with_pip=True).create(python.parent.parent)

    return subprocess.run(
        [str(python), "-m", "pip", "install", "--disable-pip-version-check", "-r", "scripts/requirements-format.txt"],
        cwd=ROOT,
    ).returncode


def source_files():
    tracked = (
        subprocess.check_output(
            ["git", "ls-files", "-z", "--", "CMakeLists.txt", "src", "test", "cmake", "scripts", "docs"], cwd=ROOT
        )
        .decode("utf-8")
        .split("\0")
    )
    groups = {"C++": [], "CMake": [], "Python": []}
    for filename in sorted(set(tracked)):
        path = Path(filename)
        # The GAMS instruction header is copied from the GAMS SDK. Test data
        # stays untouched; its manifest generator is project-owned Python.
        if not filename or filename == "src/GAMS/GamsNLinstr.h":
            continue

        if filename.startswith("test/data/") and filename != "test/data/instances/generate_manifest.py":
            continue

        if (ROOT / path).is_symlink() or not (ROOT / path).is_file():
            continue

        if path.suffix in {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx"} or filename.endswith(".h.in"):
            groups["C++"].append(filename)
        elif path.name == "CMakeLists.txt" or path.suffix == ".cmake" or filename.endswith(".cmake.in"):
            groups["CMake"].append(filename)
        elif path.suffix == ".py":
            groups["Python"].append(filename)

    return groups


def check_versions():
    failures = []
    for line in (ROOT / "scripts/requirements-format.txt").read_text().splitlines():
        if not line or line.startswith("#"):
            continue

        package, expected = line.split("==")
        try:
            installed = metadata.version(package)
        except metadata.PackageNotFoundError:
            installed = "not installed"

        if installed != expected:
            failures.append(f"{package}: expected {expected}, found {installed}")

    if failures:
        print("\n".join(failures), file=sys.stderr)
        print("Install the tools with: python scripts/format.py --setup", file=sys.stderr)

    return not failures


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--setup", action="store_true", help="create the formatter environment and install pinned tools")
    mode.add_argument("--apply", action="store_true", help="format source files in place")
    mode.add_argument("--check", action="store_true", help="report formatting differences without changing files")
    args = parser.parse_args()

    if args.setup:
        return setup_environment()

    python = environment_python()

    if Path(sys.prefix).resolve() != python.parent.parent.resolve():
        if not python.is_file():
            print("Set up the tools first: python scripts/format.py --setup", file=sys.stderr)
            return 2

        return subprocess.run([str(python), str(Path(__file__).resolve()), *sys.argv[1:]], cwd=ROOT).returncode

    if not check_versions():
        return 2

    groups = source_files()
    commands = {
        "C++": ["clang-format", "--style=file", *(["-i"] if args.apply else ["--dry-run", "--Werror"])],
        "CMake": ["cmake-format", "--config-files", ".cmake-format", "-i" if args.apply else "--check"],
        "Python": ["ruff", "format", "--no-cache", "--config", ".ruff.toml", *([] if args.apply else ["--check"])],
    }
    scripts_dir = Path(sysconfig.get_path("scripts"))
    failed = False
    for language, files in groups.items():
        command = commands[language]
        # Use executables from this Python environment, never an unpinned
        # formatter found elsewhere on PATH (including editor installations).
        command[0] = str(scripts_dir / (command[0] + (".exe" if os.name == "nt" else "")))
        print(f"{language}: {'formatting' if args.apply else 'checking'} {len(files)} files", flush=True)
        for start in range(0, len(files), 100):
            result = subprocess.run([*command, *files[start : start + 100]], cwd=ROOT)
            failed |= result.returncode != 0

    return int(failed)


if __name__ == "__main__":
    sys.exit(main())
