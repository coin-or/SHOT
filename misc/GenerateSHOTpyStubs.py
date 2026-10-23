"""
Generates the type stubs SHOTpy.pyi for the SHOTpy module with pybind11-stubgen.

Usage: python GenerateSHOTpyStubs.py <directory>, containing the built SHOTpy module. The stubs are written
to the directory, and the script fails if they cannot be generated or are not valid Python, e.g., if an enum has a
value named None, which pybind11-stubgen writes as an attribute None.
"""

import ast
import runpy
import sys
from pathlib import Path


def main():
    directory = Path(sys.argv[1]).resolve()

    # Use this process's import path, including pip's isolated build dependencies.
    sys.path.insert(0, str(directory))
    sys.argv = ["pybind11_stubgen", "SHOTpy", "--exit-code", "-o", str(directory)]
    runpy.run_module("pybind11_stubgen", run_name="__main__")

    stubFile = directory / "SHOTpy.pyi"
    ast.parse(stubFile.read_text(encoding="utf-8"), filename=str(stubFile))


if __name__ == "__main__":
    main()
