#!/usr/bin/env python3
"""Check submodule pins and vendored file contents against versions.json."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def tree_sha256(directory):
    digest = hashlib.sha256()
    for path in sorted(p for p in directory.rglob("*") if p.is_file()):
        digest.update(path.relative_to(directory).as_posix().encode() + b"\0")
        digest.update(hashlib.sha256(path.read_bytes()).digest())
    return digest.hexdigest()


def main():
    root = Path(__file__).resolve().parents[1]
    manifest = json.loads((root / "ThirdParty/versions.json").read_text())
    failures = []
    for name, dependency in manifest["dependencies"].items():
        directory = root / "ThirdParty" / name
        if dependency["kind"] == "submodule":
            # Pin verification requires a Git checkout; source distributions
            # have no submodule metadata.
            if (directory / ".git").exists():
                commit = subprocess.check_output(
                    ["git", "-C", str(directory), "rev-parse", "HEAD"], text=True).strip()
                if commit != dependency["commit"]:
                    failures.append(f"{name}: submodule commit differs from manifest")
            continue
        expected = dependency.get("tree_sha256")
        if expected is None or tree_sha256(directory) != expected:
            failures.append(f"{name}: vendored tree differs from manifest")
    if failures:
        print("\n".join(failures), file=sys.stderr)
        return 1
    print("Dependency pins and vendored contents match ThirdParty/versions.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
