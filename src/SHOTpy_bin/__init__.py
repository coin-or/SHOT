"""The SHOT executable installed with SHOTpy, run by the console script SHOT.

A wheel can only put scripts, and not the libraries they need, into the environment's bin directory, so the executable
is installed next to the SHOT and solver libraries, and SHOT is a launcher that runs it.
"""

import os
from pathlib import Path
import subprocess
import sys


def executable():
    """Return the path of the SHOT executable, e.g., to give it to a modeling system that calls SHOT through ASL."""
    if os.name == "nt":
        # Windows has no rpath, so the executable is in site-packages next to the SHOTpy module and the DLLs
        return str(Path(__file__).resolve().parent.parent / "SHOT.exe")

    # The executable finds the libraries one level up through its rpath
    return str(Path(__file__).resolve().parent / "SHOT")


def main():
    path = executable()
    arguments = [path, *sys.argv[1:]]

    if os.name != "nt":
        # Replacing the process passes signals and the exit code on as if SHOT was called directly
        os.execv(path, arguments)

    process = subprocess.Popen(arguments)

    while True:
        try:
            return process.wait()
        except KeyboardInterrupt:
            # Ctrl+C is also sent to SHOT, so wait for it to stop and return its exit code
            pass


if __name__ == "__main__":
    sys.exit(main())
