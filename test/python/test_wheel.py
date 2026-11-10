"""Checks specific to the public, open-source wheels built by cibuildwheel."""

import ast
from importlib.metadata import distribution
import os
from pathlib import Path

import pytest
import SHOTpy


pytestmark = pytest.mark.skipif(
    os.environ.get("SHOTPY_TEST_INSTALLED") != "1",
    reason="Only checks the installed public wheel",
)


def test_installed_module_and_generated_stubs():
    package = distribution("SHOTpy")
    module = Path(SHOTpy.__file__).resolve()
    installed_files = {Path(package.locate_file(f)).resolve() for f in package.files}
    assert module in installed_files, "Tests loaded a build-tree module instead of the installed wheel"

    stub = module.with_name("SHOTpy.pyi")
    assert stub in installed_files
    tree = ast.parse(stub.read_text(encoding="utf-8"))
    declarations = {node.name for node in tree.body if isinstance(node, (ast.ClassDef, ast.FunctionDef))}
    assert {"Solver", "Problem", "CallbackLocation", "SolutionStatistics", "min", "max"} <= declarations


def test_public_wheel_solver_support():
    # CMake can disable a missing optional dependency; do not publish a silently reduced wheel.
    for feature in ("HAS_CBC", "HAS_IPOPT", "HAS_HIGHS", "HAS_AMPL", "HAS_OSIL"):
        assert getattr(SHOTpy, feature), f"Public wheel is missing {feature}"
    for feature in ("HAS_CPLEX", "HAS_GUROBI", "HAS_GAMS"):
        assert not getattr(SHOTpy, feature), f"Public wheel unexpectedly includes {feature}"
