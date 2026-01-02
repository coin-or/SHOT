"""
Pytest configuration for SHOT Python API tests.

This file is automatically loaded by pytest and provides common fixtures.
"""

import os
import sys
from pathlib import Path
import pytest

# Add build directory to path so we can import shotpy
def _setup_path():
    """Find and add the shotpy module to sys.path."""
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
            # Check if shotpy module exists
            shotpy_files = list(build_dir.glob("shotpy*.so")) + list(build_dir.glob("shotpy*.pyd"))
            if shotpy_files:
                sys.path.insert(0, str(build_dir))
                return str(build_dir)
    
    # If not found, try the current working directory
    cwd = Path.cwd()
    shotpy_files = list(cwd.glob("shotpy*.so")) + list(cwd.glob("shotpy*.pyd"))
    if shotpy_files:
        sys.path.insert(0, str(cwd))
        return str(cwd)
    
    raise ImportError("Could not find shotpy module. Make sure SHOT is built.")

BUILD_DIR = _setup_path()

import shotpy


class SHOTContext:
    """Container to hold solver, env, and problem together to manage lifetimes."""
    def __init__(self):
        self.solver = shotpy.Solver()
        self.env = self.solver.getEnvironment()
        self.problem = shotpy.Problem(self.env)


def set_default_objective(problem, variables=None):
    """
    Set a default linear objective function on the problem.
        
    Args:
        problem: The Problem object
        variables: Optional list of variables to include in the objective.
                  If None, creates a zero objective (minimize 0).
    """
    obj = shotpy.LinearObjectiveFunction(shotpy.ObjectiveDirection.Minimize)
    if variables:
        for var in variables:
            obj.add(shotpy.LinearTerm(1.0, var))
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


# Re-export shotpy for convenience
@pytest.fixture
def shotpy_module():
    """Provide the shotpy module."""
    return shotpy

