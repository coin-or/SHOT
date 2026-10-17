# Your first model

Install SHOTpy in a Python 3.11 or newer environment:

```bash
python -m pip install SHOTpy
```

The public package includes open-source solver backends. For a development checkout, or to enable proprietary solver integrations, follow the [compilation instructions](CompilationInstructions.md). The installed package version may differ from this development documentation.

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

Continue with the [Python tutorial](SHOTpy_Tutorial.md) for nonlinear expressions, settings, file input, and callbacks. For file-based workflows, see [command-line usage](CommandLineUsage.md).
