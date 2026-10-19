# SHOTpy examples

Standalone Jupyter notebooks that show what SHOT can do through its Python interface, SHOTpy. Each notebook builds
its problem with SHOTpy, runs from top to bottom in a fresh kernel, and checks its own results. For an introduction to
SHOTpy, start with the [tutorial](../../docs/SHOTpy_Tutorial.ipynb).

| Notebook | Shows | Tested with | Optional |
| --- | --- | --- | --- |
| [LazyConstraints.ipynb](LazyConstraints.ipynb) | Nonlinear constraints known only to a black-box Python function, enforced by rejecting candidates in `PrimalCandidateCheck` and adding the black box's cuts in `HyperplaneSelection` (MINLPLib `ex1223b`) | HiGHS (also Cbc), Ipopt | Matplotlib |

To run a notebook against a local build, put the directory with the SHOTpy module on the Python path:

```bash
PYTHONPATH=build jupyter lab examples/SHOTpy/LazyConstraints.ipynb
```

The notebooks are stored without outputs. `test/python/test_examples.py` runs every notebook in this directory with
the Python tests, so an example that no longer gives its expected results fails the tests.
