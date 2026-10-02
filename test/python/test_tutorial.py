"""
Runs the code cells of the SHOTpy tutorial notebook, docs/SHOTpy_Tutorial.ipynb.

The cells are run in order in one namespace, as Jupyter runs them, since later cells use what earlier ones define.
The examples assert their own results, e.g., the known optimum, so a cell fails both when it raises and when it
gives a wrong result.
"""

import contextlib
import io
import json
import os
from pathlib import Path

import pytest


def notebookPath():
    # CMake copies the tests to the build directory and gives the path in the source directory
    if "SHOT_TUTORIAL_NOTEBOOK" in os.environ:
        return Path(os.environ["SHOT_TUTORIAL_NOTEBOOK"])

    return Path(__file__).resolve().parent.parent.parent / "docs" / "SHOTpy_Tutorial.ipynb"


def codeCells():
    path = notebookPath()

    if not path.exists():
        return []

    notebook = json.loads(path.read_text(encoding="utf-8"))

    return [(index, "".join(cell["source"])) for index, cell in enumerate(notebook["cells"])
            if cell["cell_type"] == "code"]


CELLS = codeCells()


@pytest.fixture(scope="module")
def namespace():
    """The namespace shared by the cells, as in a Jupyter kernel."""
    return {"__name__": "__main__"}


@pytest.mark.skipif(not CELLS, reason="The tutorial notebook was not found")
@pytest.mark.parametrize("index,source", CELLS, ids=[f"cell{index}" for index, _ in CELLS])
def test_tutorial_cell(index, source, namespace):
    output = io.StringIO()

    try:
        with contextlib.redirect_stdout(output):
            exec(compile(source, f"<tutorial cell {index}>", "exec"), namespace)
    except Exception:
        # The output of the cell tells what it got to before failing
        print(output.getvalue()[-5000:])
        raise
