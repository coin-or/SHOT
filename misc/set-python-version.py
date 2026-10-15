#!/usr/bin/env python3
"""Check or set the SHOTpy package version for a CI build.

The version is project(SHOT VERSION ...) followed by SHOT_VERSION_SUFFIX in
CMakeLists.txt, for example 2.0.0rc0. A build of a tag v<version>[-<suffix>]
checks that the tag matches it (v2.0-rc0 and v2.0.0rc0 both match 2.0.0rc0).
Any other build is a development build <version>.dev<run number>; the .dev
part is appended to the 'result' of the regex version provider in
pyproject.toml.
"""
import os
from pathlib import Path
import re
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    cmake = (root / "CMakeLists.txt").read_text()
    number = re.search(r"project\(SHOT VERSION ([0-9.]+)", cmake)
    suffix = re.search(r'set\(SHOT_VERSION_SUFFIX "([^"]*)"\)', cmake)
    if number is None or suffix is None:
        print("No project(SHOT VERSION ...) or SHOT_VERSION_SUFFIX in CMakeLists.txt", file=sys.stderr)
        return 1
    number, suffix = number.group(1), suffix.group(1)
    if not re.fullmatch(r"((a|b|rc)[0-9]+)?", suffix):
        print(f"SHOT_VERSION_SUFFIX {suffix} is not empty, a<n>, b<n> or rc<n>", file=sys.stderr)
        return 1
    version = number + suffix

    if os.environ.get("GITHUB_REF_TYPE") == "tag":
        tag = os.environ["GITHUB_REF_NAME"]
        match = re.fullmatch(r"v([0-9]+(?:\.[0-9]+)*)(?:[-_.]?((?:a|b|rc)[0-9]+))?", tag)
        if match is None:
            print(f"Tag {tag} is not v<version> or v<version>-<a|b|rc><n>", file=sys.stderr)
            return 1
        release = [int(part) for part in match.group(1).split(".")]
        cmakeRelease = [int(part) for part in number.split(".")]
        release += [0] * (len(cmakeRelease) - len(release))
        if release != cmakeRelease or (match.group(2) or "") != suffix:
            print(f"Tag {tag} does not match the version {version} in CMakeLists.txt", file=sys.stderr)
            return 1
    else:
        dev = f".dev{os.environ['GITHUB_RUN_NUMBER']}"
        pyproject = root / "pyproject.toml"
        text, count = re.subn(r'^result = "\{value\}\{suffix\}"$', f'result = "{{value}}{{suffix}}{dev}"',
            pyproject.read_text(), count=1, flags=re.MULTILINE)
        if count != 1:
            print('No result = "{value}{suffix}" in pyproject.toml', file=sys.stderr)
            return 1
        pyproject.write_text(text)
        version += dev

    print(f"SHOTpy version {version}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
