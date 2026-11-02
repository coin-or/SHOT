"""
Tests for the SHOT executable in the Python package, run by the console script SHOT.

The executable is only part of an installed SHOTpy, e.g., a wheel, so the tests are skipped when SHOTpy is used from a
build directory.
"""

import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import sysconfig

import pytest

SHOTpy_bin = pytest.importorskip("SHOTpy_bin", reason="The SHOT executable is only in an installed SHOTpy package")

DATA_DIR = Path(__file__).parent.parent / "data"


def shot_script():
    """The console script SHOT installed with the package."""
    name = "SHOT.exe" if os.name == "nt" else "SHOT"
    return str(Path(sysconfig.get_path("scripts")) / name)


def run(command, directory):
    return subprocess.run(command, cwd=directory, capture_output=True, text=True, timeout=300)


def test_executable_exists():
    assert Path(SHOTpy_bin.executable()).is_file()


def test_console_script_help(tmp_path):
    result = run([shot_script(), "--help"], tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Usage: SHOT PROBLEMFILE" in result.stdout


def test_module_help(tmp_path):
    result = run([sys.executable, "-m", "SHOTpy_bin", "--help"], tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr


def test_exit_code_is_passed_on(tmp_path):
    result = run([shot_script(), str(tmp_path / "missing.osil")], tmp_path)
    assert result.returncode == 2  # E_ExitCode::InputError


def test_solve_osil(tmp_path):
    result = run([shot_script(), str(DATA_DIR / "alan.osil"), "--osrl", "alan.osrl"], tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr

    objective = re.search(r'<obj idx="-1">([^<]+)</obj>', (tmp_path / "alan.osrl").read_text())
    assert objective is not None
    assert float(objective.group(1)) == pytest.approx(2.925, abs=1e-3)


def test_solve_nl_as_ampl_solver(tmp_path):
    # AMPL and Pyomo call the solver with the stub of the .nl file and -AMPL, and read the .sol file written next to it
    shutil.copy(DATA_DIR / "tls2.nl", tmp_path)
    result = run([shot_script(), "tls2", "-AMPL", "Termination.TimeLimit=60"], tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    assert (tmp_path / "tls2.sol").is_file()
