"""
Generates the type stubs SHOTpy.pyi for the SHOTpy module with pybind11-stubgen.

Usage: python GenerateSHOTpyStubs.py <directory>, with SHOTpy importable, e.g., through PYTHONPATH. The stubs are written
to the directory, and the script fails if they cannot be generated or are not valid Python.

Several enums have a value named None, e.g., MIPSolver.None, which is reached with getattr(SHOTpy.MIPSolver, "None").
pybind11-stubgen writes it as an attribute None, which is not valid Python, so it is left out of the stubs.
"""

import ast
import re
import subprocess
import sys
from pathlib import Path


def main():
    directory = Path(sys.argv[1])

    subprocess.run([sys.executable, "-m", "pybind11_stubgen", "SHOTpy", "--exit-code", "-o", str(directory)],
                   check=True)

    stubFile = directory / "SHOTpy.pyi"
    stubs = stubFile.read_text(encoding="utf-8")
    stubs = re.sub(r"(?m)^    None: typing\.ClassVar\[\w+\]  # value = .*\n|^    None: typing\.ClassVar\[\w+\]\n", "", stubs)

    ast.parse(stubs, filename=str(stubFile))
    stubFile.write_text(stubs, encoding="utf-8")


if __name__ == "__main__":
    main()
