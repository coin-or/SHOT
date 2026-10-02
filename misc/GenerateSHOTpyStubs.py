"""
Generates the type stubs SHOTpy.pyi for the SHOTpy module with pybind11-stubgen.

Usage: python GenerateSHOTpyStubs.py <directory>, with SHOTpy importable, e.g., through PYTHONPATH. The stubs are written
to the directory, and the script fails if they cannot be generated or are not valid Python, e.g., if an enum has a
value named None, which pybind11-stubgen writes as an attribute None.
"""

import ast
import subprocess
import sys
from pathlib import Path


def main():
    directory = Path(sys.argv[1])

    subprocess.run([sys.executable, "-m", "pybind11_stubgen", "SHOTpy", "--exit-code", "-o", str(directory)],
                   check=True)

    stubFile = directory / "SHOTpy.pyi"
    ast.parse(stubFile.read_text(encoding="utf-8"), filename=str(stubFile))


if __name__ == "__main__":
    main()
