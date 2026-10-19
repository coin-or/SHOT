"""
Runs the code cells of the SHOTpy example notebooks in examples/SHOTpy.

The cells of a notebook are run in order in one namespace, as Jupyter runs them. The examples assert their own
results, so a notebook fails both when a cell raises and when it gives a wrong result.
"""

import contextlib
import io
import json
import os
from pathlib import Path

import pytest

# The figures of the examples are drawn without a window, which would otherwise wait for it to be closed
os.environ.setdefault("MPLBACKEND", "Agg")
pytestmark = pytest.mark.filterwarnings("ignore:FigureCanvasAgg is non-interactive")


def examplesPath():
    # CMake copies the tests to the build directory and gives the path in the source directory
    if "SHOT_EXAMPLES_DIR" in os.environ:
        return Path(os.environ["SHOT_EXAMPLES_DIR"])

    return Path(__file__).resolve().parent.parent.parent / "examples" / "SHOTpy"


NOTEBOOKS = sorted(examplesPath().glob("*.ipynb"))


@pytest.mark.skipif(not NOTEBOOKS, reason="No example notebooks were found")
@pytest.mark.parametrize("path", NOTEBOOKS, ids=[path.stem for path in NOTEBOOKS])
def test_example_notebook(path):
    notebook = json.loads(path.read_text(encoding="utf-8"))
    namespace = {"__name__": "__main__"}

    for index, cell in enumerate(notebook["cells"]):
        if cell["cell_type"] != "code":
            continue

        output = io.StringIO()

        try:
            with contextlib.redirect_stdout(output):
                exec(compile("".join(cell["source"]), f"<{path.name} cell {index}>", "exec"), namespace)
        except Exception:
            # The output of the cell tells what it got to before failing
            print(output.getvalue()[-5000:])
            raise
