"""
Pytest configuration for SHOT Python API tests.

This file is automatically loaded by pytest and provides common fixtures.
"""

import os
import sys
from pathlib import Path
import pytest

# Add build directory to path so we can import SHOTpy
def _setup_path():
    """Find and add the SHOTpy module to sys.path."""
    test_dir = Path(__file__).parent
    repo_root = test_dir.parent.parent
    
    # Try different possible build locations
    possible_builds = [
        repo_root / "build" / "debug",
        repo_root / "build" / "release", 
        repo_root / "build",
    ]
    
    for build_dir in possible_builds:
        if build_dir.exists():
            # Check if SHOTpy module exists
            SHOTpy_files = list(build_dir.glob("SHOTpy*.so")) + list(build_dir.glob("SHOTpy*.pyd"))
            if SHOTpy_files:
                sys.path.insert(0, str(build_dir))
                return str(build_dir)
    
    # If not found, try the current working directory
    cwd = Path.cwd()
    SHOTpy_files = list(cwd.glob("SHOTpy*.so")) + list(cwd.glob("SHOTpy*.pyd"))
    if SHOTpy_files:
        sys.path.insert(0, str(cwd))
        return str(cwd)
    
    raise ImportError("Could not find SHOTpy module. Make sure SHOT is built.")

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

