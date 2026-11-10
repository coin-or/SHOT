"""
Runs the code cells of the SHOTpy tutorial notebooks in docs/tutorial.

The cells of a notebook are run in order in one namespace, as Jupyter runs them, since later cells use what earlier
ones define. The examples assert their own results, e.g., the known optimum, so a cell fails both when it raises and
when it gives a wrong result.
"""

import contextlib
import importlib.util
import io
import json
import os
import re
from pathlib import Path

import pytest

# The figures of the tutorial are drawn without a window, which would otherwise wait for it to be closed
os.environ.setdefault("MPLBACKEND", "Agg")
pytestmark = pytest.mark.filterwarnings("ignore:FigureCanvasAgg is non-interactive")

# The solves limited by time stop with the best solution found; the tests use a shorter limit
TIME_LIMIT = re.compile(r'updateSetting\("Termination\.TimeLimit", ([0-9.]+)\)')
TEST_TIME_LIMIT = 10.0


def tutorialFolder():
    # CMake copies the tests to the build directory and gives the folder in the source directory
    if "SHOT_TUTORIAL_FOLDER" in os.environ:
        return Path(os.environ["SHOT_TUTORIAL_FOLDER"])

    return Path(__file__).resolve().parent.parent.parent / "docs" / "tutorial"


def codeCells():
    cells = []

    for path in sorted(tutorialFolder().glob("*.ipynb")):
        notebook = json.loads(path.read_text(encoding="utf-8"))
        cells += [
            (path.stem, index, "".join(cell["source"]))
            for index, cell in enumerate(notebook["cells"])
            if cell["cell_type"] == "code"
        ]

    return cells


CELLS = codeCells()


@pytest.fixture(scope="module")
def namespaces():
    """The namespace shared by the cells of each notebook, as in a Jupyter kernel."""
    return {}


@pytest.fixture(scope="module")
def missingPackages():
    """The packages that each notebook installs with %pip but that are not installed here."""
    return {}


@pytest.mark.skipif(not CELLS, reason="The tutorial notebooks were not found")
@pytest.mark.parametrize("notebook,index,source", CELLS, ids=[f"{name}-cell{index}" for name, index, _ in CELLS])
def test_tutorial_cell(notebook, index, source, namespaces, missingPackages, monkeypatch):
    if source.lstrip().startswith("%pip install"):
        # The notebook is skipped if a package that it installs is missing, as it is not installed by the tests
        packages = [name for name in source.split()[2:] if name != "SHOTpy"]
        missingPackages[notebook] = [name for name in packages if importlib.util.find_spec(name) is None]

    if missingPackages.get(notebook):
        pytest.skip(f"{notebook} needs {', '.join(missingPackages[notebook])}")

    if source.lstrip().startswith("%"):
        return

    source = TIME_LIMIT.sub(
        lambda match: f'updateSetting("Termination.TimeLimit", {min(float(match[1]), TEST_TIME_LIMIT)})', source
    )

    # The notebooks read their data files from their folder
    monkeypatch.chdir(tutorialFolder())

    output = io.StringIO()
    namespace = namespaces.setdefault(notebook, {"__name__": "__main__"})

    try:
        with contextlib.redirect_stdout(output):
            exec(compile(source, f"<{notebook} cell {index}>", "exec"), namespace)
    except Exception:
        # The output of the cell tells what it got to before failing
        print(output.getvalue()[-5000:])
        raise
