"""
Pytest configuration for SHOT Python API tests.

This file is automatically loaded by pytest and provides common fixtures.
"""

import importlib.util
import os
import sys
from pathlib import Path
import pytest

def _has_shotpy(directory):
    """Whether the directory contains a built SHOTpy module."""
    return directory.is_dir() and (any(directory.glob("SHOTpy*.so")) or any(directory.glob("SHOTpy*.pyd")))


def _shotpy_modification_time(directory):
    """The modification time of the newest SHOTpy module in the directory."""
    modules = list(directory.glob("SHOTpy*.so")) + list(directory.glob("SHOTpy*.pyd"))
    return max(module.stat().st_mtime for module in modules)


# Add build directory to path so we can import SHOTpy
def _setup_path():
    """Find and add the SHOTpy module to sys.path.

    The module is taken from, in order:
      1. the directory in the environment variable SHOTPY_BUILD_DIR,
      2. the first directory in PYTHONPATH that contains it (ctest sets PYTHONPATH to the build directory),
      3. the current working directory,
      4. the build directories of the repository; if several contain it, the most recently built one, so that an
         old build directory does not hide the current one,
      5. an installed SHOTpy, e.g., a wheel, which is what the tests of a wheel build use.
    """
    explicit = os.environ.get("SHOTPY_BUILD_DIR")
    if explicit:
        if not _has_shotpy(Path(explicit)):
            raise ImportError(f"SHOTPY_BUILD_DIR={explicit} does not contain a SHOTpy module.")
        candidates = [Path(explicit)]
    else:
        python_path = [Path(p) for p in os.environ.get("PYTHONPATH", "").split(os.pathsep) if p]
        candidates = [d for d in python_path + [Path.cwd()] if _has_shotpy(d)][:1]

    if not candidates:
        repo_root = Path(__file__).parent.parent.parent
        builds = [repo_root / "build" / "debug", repo_root / "build" / "release", repo_root / "build"]
        candidates = sorted((d for d in builds if _has_shotpy(d)), key=_shotpy_modification_time, reverse=True)

    if not candidates:
        if importlib.util.find_spec("SHOTpy") is not None:
            return None

        raise ImportError(
            "Could not find a SHOTpy module. Build SHOT with -DHAS_PYTHON=on, or install the package."
        )

    build_dir = str(candidates[0].resolve())
    sys.path.insert(0, build_dir)
    return build_dir

BUILD_DIR = _setup_path()

import SHOTpy


class SHOTContext:
    """Container to hold solver, env, and problem together to manage lifetimes."""
    def __init__(self):
        self.solver = SHOTpy.Solver()
        self.env = self.solver.getEnvironment()
        self.problem = SHOTpy.Problem(self.env)


def set_default_objective(problem, variables=None):
    """
    Set a default linear objective function on the problem.
        
    Args:
        problem: The Problem object
        variables: Optional list of variables to include in the objective.
                  If None, creates a zero objective (minimize 0).
    """
    obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
    if variables:
        for var in variables:
            obj.add(SHOTpy.LinearTerm(1.0, var))
    problem.setObjective(obj)


@pytest.fixture
def shot_context():
    """Create a context with solver, env, and problem that stays alive."""
    return SHOTContext()


@pytest.fixture
def solver(shot_context):
    """Get the Solver from context."""
    return shot_context.solver


@pytest.fixture
def env(shot_context):
    """Get the environment from context."""
    return shot_context.env


@pytest.fixture
def problem(shot_context):
    """Get a fresh Problem from context."""
    return shot_context.problem


@pytest.fixture
def data_dir():
    """Return the path to the test data directory."""
    return Path(__file__).parent.parent / "data"


# Re-export SHOTpy for convenience
@pytest.fixture
def SHOTpy_module():
    """Provide the SHOTpy module."""
    return SHOTpy

