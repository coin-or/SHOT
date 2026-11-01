#!/usr/bin/env python3
"""Smoke-test an extracted release archive without installing SHOTpy or solvers."""

import argparse
import hashlib
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    args = parser.parse_args()
    archive = args.archive.resolve()
    expected = archive.with_name(archive.name + ".sha256").read_text().split()[0]
    if hashlib.sha256(archive.read_bytes()).hexdigest() != expected:
        raise ValueError("Archive checksum mismatch")
    data = Path(__file__).resolve().parents[1] / "test/data"
    # Remove paths that might expose build dependencies or a Python installation.
    env = {key: value for key, value in os.environ.items()
           if key.upper() not in {"PATH", "LD_LIBRARY_PATH", "LD_PRELOAD", "DYLD_LIBRARY_PATH",
                                  "DYLD_FALLBACK_LIBRARY_PATH", "PYTHONPATH", "PYTHONHOME"}}
    env["PATH"] = str(Path(os.environ["SystemRoot"]) / "System32") if os.name == "nt" else "/usr/bin:/bin"
    with tempfile.TemporaryDirectory(prefix="shot release test ") as temporary:
        root = Path(temporary)
        shutil.unpack_archive(archive, root)
        bundle, = root.iterdir()
        executable = bundle / ("SHOT.exe" if os.name == "nt" else "SHOT")

        def run(arguments, expected_code=0):
            result = subprocess.run([str(executable), *arguments], cwd=root, env=env,
                                    capture_output=True, text=True, encoding="utf-8",
                                    errors="replace", timeout=180)
            print(result.stdout)
            print(result.stderr)
            if result.returncode != expected_code:
                raise RuntimeError(f"SHOT exited with {result.returncode}, expected {expected_code}")
            return result.stdout

        if "Usage: SHOT PROBLEMFILE" not in run(["--help"]):
            raise RuntimeError("Missing command-line help")
        run([str(root / "missing.osil")], expected_code=2)
        for solver in ("highs", "cbc"):
            result_file = root / f"{solver}.osrl"
            run([str(data / "alan.osil"), f"--mip={solver}", "--nlp=ipopt",
                 "Termination.TimeLimit=60", "--osrl", str(result_file)])
            result = ET.parse(result_file)
            objective = result.find(".//{*}objectives/{*}values/{*}obj")
            if objective is None or not math.isclose(float(objective.text), 2.925, abs_tol=1e-3):
                raise RuntimeError(f"Unexpected objective with {solver}")
        shutil.copy(data / "tls2.nl", root)
        run(["tls2", "-AMPL", "Termination.TimeLimit=60"])
        if not (root / "tls2.sol").is_file():
            raise RuntimeError("Missing AMPL solution file")


if __name__ == "__main__":
    main()
