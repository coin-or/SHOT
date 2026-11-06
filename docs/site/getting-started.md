# Your first model

Install SHOTpy in a Python 3.11 or newer environment:

```bash
python -m pip install SHOTpy
```

The public package includes open-source solver backends. For a development checkout, or to enable proprietary solver integrations, follow the [compilation instructions](CompilationInstructions.md). Check that the installed package version matches the version of this documentation.

## Build and solve a model

```python
import SHOTpy

solver = SHOTpy.Solver()
problem = SHOTpy.Problem(solver)
x = problem.addVariable("x", SHOTpy.VariableType.Real, 0.0, 10.0)
y = problem.addVariable("y", SHOTpy.VariableType.Integer, 0.0, 10.0)
problem.setObjective(SHOTpy.exp(x) + (y - 2.5)**2)
problem.addConstraint(x + y >= 3, "demand")
problem.finalize()
solver.setProblem(problem)
solver.solveProblem()
print(solver.getModelReturnStatus())
if solver.hasPrimalSolution():
    print(solver.getPrimalSolution().point)
```

The model minimizes an exponential term plus a quadratic penalty, subject to a demand constraint. The second variable must take an integer value. Always inspect the returned model status and check that a feasible solution exists before reading it: a time limit or infeasible model need not produce a solution.

SHOT prints preparation statistics, an iteration table, and a final solution
report. Follow [Understanding SHOT output](UnderstandingOutput.md) to decode
the columns and row labels. See [solver results](concepts.md) for the distinction
between a feasible solution and a proven optimum, and
[configuration](Configuration.md) for limits, tolerances, and output controls.

To start from a known point, call `solver.setStartingPoint([1.0, 2.0])` between `setProblem()` and `solveProblem()`, with a value for each variable in the order they were added. A feasible point is the first primal solution and the MIP start of the dual problems.

Continue with the [Python tutorial](SHOTpy_Tutorial.md) for nonlinear expressions, settings, file input, and callbacks. For file-based workflows, see [command-line usage](CommandLineUsage.md). The command-line program is also available without Python as a download for Linux, macOS and Windows from the [GitHub releases](https://github.com/coin-or/SHOT/releases).
