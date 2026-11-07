# Understanding solver results

## Feasible solutions and bounds

A primal solution assigns values to the variables while satisfying the constraints and integrality requirements within tolerances. For minimization, its objective gives an upper bound on the optimum. A valid dual bound is a lower bound. For maximization, the directions reverse.

The gap measures the distance between these bounds. A small gap is useful only together with the feasibility and optimality tolerances and the solver's reported status. Reaching a time or iteration limit does not establish optimality.

For primal bound `p` and globally valid dual bound `d`, the absolute gap is
`abs(d - p)` and the relative gap is `abs(d - p) / (1e-10 + abs(p))`. Relative
gaps are fractions: `0.001` is 0.1%. Either objective-gap tolerance can establish
gap convergence. An unavailable bound gives an infinite gap.

`getCurrentDualBound()` describes the current dual problem. For nonconvex
problems it can cease to be globally valid after cuts or repairs. Use
`getGlobalDualBound()` when reporting a valid bound. The public
`getAbsoluteObjectiveGap()` and `getRelativeObjectiveGap()` use the global bound.
The [output guide](UnderstandingOutput.md#bounds-gaps-and-the-asterisk) explains
the corresponding `*` marker and why iteration and final gaps can differ.

## Model status and termination

`getModelReturnStatus()` describes what was established about the model;
`getTerminationReason()` describes why the algorithm stopped. Inspect both.

| Model status | Interpretation |
| --- | --- |
| `OptimalGlobal` | The incumbent and global bound meet an objective-gap tolerance. The guarantee depends on any user-specified convexity assumptions. |
| `FeasibleSolution` | A primal solution exists, but the global gap was not closed. This can follow a limit, stagnation, a user stop, or an error. Inspect the termination reason. |
| `InfeasibleGlobal` | Infeasibility was established globally. |
| `InfeasibleLocal` | Infeasibility was reported without a global proof for the nonconvex model. |
| `NoSolutionReturned` | No feasible solution was returned; this alone is not proof of infeasibility. |
| `Unbounded`, `UnboundedNoSolution` | Unbounded status, with or without an available primal solution respectively. |
| `ErrorUnknown`, `ErrorNoSolution` | Error status; inspect solution availability and diagnostics. |
| `NotSet` | No model status has been established. |

Termination reasons include `AbsoluteGap`, `RelativeGap`, `ConstraintTolerance`,
`TimeLimit`, `IterationLimit`, `ObjectiveStagnation`, `NoDualCutsAdded`,
`InfeasibleProblem`, `UnboundedProblem`, `NumericIssues`, `Error`, and
`UserAbort`. Reaching a constraint tolerance or stopping for stagnation is not
the same as closing the global objective gap.

A successful `solveProblem()` call says the solve completed; it does not
establish optimality. In Python, a solver failure can raise an exception.
Check `hasPrimalSolution()` before requesting the incumbent. This example is a
complete solve with status and bound reporting:

```python
import SHOTpy

solver = SHOTpy.Solver()
solver.updateSetting("Termination.TimeLimit", 60.0)
problem = SHOTpy.Problem(solver)
x = problem.addVariable("x", SHOTpy.VariableType.Integer, 0.0, 5.0)
problem.setObjective((x - 2.3)**2)
problem.finalize()
assert solver.setProblem(problem)
assert solver.solveProblem()

print("Model status:", solver.getModelReturnStatus())
print("Termination reason:", solver.getTerminationReason())
print("Global dual bound:", solver.getGlobalDualBound())
if solver.hasPrimalSolution():
    solution = solver.getPrimalSolution()
    print("Feasible objective:", solution.objValue)
    print("Original variable values:", solution.point)
    print("Global relative gap:", solver.getRelativeObjectiveGap())
else:
    print("No feasible solution returned")
```

C++ exposes the same methods and uses `E_ModelReturnStatus` and
`E_TerminationReason`; see the [integration example](cpp.md). In file-based
workflows, inspect the OSrL/modeling-system status. The executable's exit code
`0` only says that the run completed. Pyomo's mapping of AMPL statuses also
requires care; see [command-line usage](CommandLineUsage.md#using-shot-from-pyomo).

## Outer approximation

SHOT constructs linear approximations of nonlinear expressions and solves mixed-integer subproblems. Primal searches seek feasible solutions to the original nonlinear problem. Convexity determines when linearizations provide globally valid bounds; nonconvex models require additional care and do not generally have the same guarantees. See [modeling and reformulation](Modeling.md) for domain restrictions, bounds, and how the original model is transformed.

## Convexity specified by the user

SHOT detects the convexity of the objective function and of each constraint function, and regards a function as nonconvex when it cannot prove otherwise. When you know better, you can specify the convexity with `setFunctionConvexity()` on a quadratic or nonlinear constraint or objective function, in C++ and SHOTpy, or with the `functionConvexity` argument of `Problem.addConstraint()` and `Problem.setObjective()` in SHOTpy. The value is `Convex`, `Concave` or `Nonconvex`. For a constraint L <= f(x) <= U, it is the convexity of f(x), and the convexity of the constraint follows from it and the bounds: f(x) <= U is convex for a convex function, and L <= f(x) for a concave one. A minimized convex or maximized concave objective function gives a convex problem. Such a function is not reformulated, e.g., its sums are not partitioned, since the convexity of its parts is not known.

`Model.Convexity.AssumeConvex` (`--convex`) instead assumes the whole problem to be convex. In both cases, the global optimality of a solution depends on the assumption, which the solution report, the `UserSpecifiedConvexity` entry of the OSrL results and `getUserSpecifiedConvexityDescription()` tell.

## Solver backends

The MIP/NLP backends compiled into SHOT depend on the build configuration. Solving
with a proprietary backend also requires a suitable license. Select a backend
available in your build and consult the [settings reference](reference/settings.md).
See the [compilation guide](CompilationInstructions.md) for dependencies and the
[debugging guide](DebuggingSHOT.md) for intermediate problems and diagnostic logs.

### Reading MIP solver capabilities

`SHOTpy.getSupportedMIPSolvers()` lists the MIP backends compiled into your build.
Use `SHOTpy.getMIPSolverCapabilities()` to inspect what SHOT's adapter supports
before choosing one. The lookup constructs no solver and needs no solver license.
It returns `None` for a backend that was not compiled in, or for `MIPSolver.NotUsed`.

```python
import SHOTpy

capabilities = SHOTpy.getMIPSolverCapabilities(SHOTpy.MIPSolver.Gurobi)

if capabilities is None:
    print("Gurobi is not compiled into this build")
else:
    print("SOS1:", capabilities.SOS1)
    print("Nonconvex quadratic constraints:", capabilities.nonconvexQuadraticConstraints)
    print("Semicontinuous domains:", capabilities.semicontinuous)
```

The returned `MIPSolverCapabilities` object has read-only properties for convex
and nonconvex quadratic objectives and constraints, SOS1/SOS2, semicontinuous and
semi-integer variable domains, SHOT's single-tree integration, and finite
variable-bound defaults. Assigning or deleting a property raises `AttributeError`.
Semi-variable support is `SemiVariableSupport.None_`, `NonnegativeInterval`, or
`SignedInterval`; the last also permits a negative nonzero branch.

C++ provides the same query through `getMIPSolverCapabilities()`, returning a
`std::optional<MIPSolverCapabilities>`. The header does not require a vendor SDK:

```cpp
#include "MIPSolver/MIPSolverCapabilities.h"
#include <iostream>

int main()
{
    const auto capabilities = SHOT::getMIPSolverCapabilities(SHOT::ES_MIPSolver::Gurobi);

    if(capabilities)
        std::cout << "SOS1: " << std::boolalpha << capabilities->SOS1 << '\n';
}
```

Capabilities describe support through SHOT's adapter; they do not enable a
reformulation or establish that a license is available. Support for an objective
and for constraints separately does not guarantee support for their combination.
For example, CPLEX supports a nonconvex quadratic objective with linear
constraints, but cannot generally combine that objective with quadratic
constraints. Its `nonconvexObjectiveExcludesQuadraticConstraints` flag records
this restriction.

See [MIP solver capabilities](MIPSolverCapabilities.md) for the complete field
definitions and semi-variable domains, or the [SHOTpy](reference/python.rst) and
{ref}`C++ <cpp-mip-solver-capabilities>` API references.
