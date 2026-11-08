# Modeling and reformulation

SHOT solves the model you supply within numerical tolerances. Its preparation
steps can substantially change the representation handed to the subsolvers.
This guide explains useful bounds, expression domains, and how to interpret
those transformations. For API syntax, follow the
[SHOTpy tutorial](SHOTpy_Tutorial.md) or [C++ integration guide](cpp.md).

## Give useful variable bounds

Finite, mathematically justified bounds help bound tightening, linearization,
and reformulation. For example, a product of a bounded continuous variable and
a binary variable can be represented using linear constraints and an auxiliary
variable. Loose bounds weaken those constraints and the resulting relaxation.

Bounds should follow from the model. Choosing arbitrary small bounds can exclude
the intended optimum. SHOT also has limits on otherwise unbounded variables,
controlled by `Model.Variables.Continuous.MinimumLowerBound`,
`Model.Variables.Continuous.MaximumUpperBound`, and the corresponding integer
settings. A warning that the solution is at maximum variable bounds is a reason
to inspect those limits and the intended model.

## Keep nonlinear expressions inside their domains

Bounds should make expressions well-defined wherever the solver may evaluate
them. In particular, logarithms require positive arguments, denominators must
stay away from zero, and square roots require nonnegative arguments. Derivatives
of a square root are singular at zero, so a positive lower bound is useful when
it is justified by the model. Fractional powers need the same domain care.

For example, this small convex model keeps a reciprocal away from its singularity:

```python
import SHOTpy

solver = SHOTpy.Solver()
problem = SHOTpy.Problem(solver)
x = problem.addVariable("x", SHOTpy.VariableType.Real, 0.1, 10.0)
problem.setObjective(x + 1 / x)
problem.finalize()
assert solver.setProblem(problem)
assert solver.solveProblem()
assert solver.hasPrimalSolution()
solution = solver.getPrimalSolution()
assert abs(solution.point[0] - 1.0) < 0.01
```

An objective or constraint being well-defined at a supplied starting point does
not ensure that all later trial points are in its domain. Express domain
restrictions through bounds and constraints as appropriate. Rescale variables,
constraint coefficients, and objective units when their magnitudes differ
greatly; overly tight numerical tolerances cannot fix poor scaling.

## Understand detected convexity

A convex minimized objective (or concave maximized objective) and convex feasible
set support SHOT's global outer-approximation guarantees. A nonlinear equality
or two-sided nonlinear constraint is not generally convex even if its function
is convex. For example, `x**2 <= 4` is convex, while `x**2 >= 1` over an interval
containing negative and positive values is not.

SHOT classifies expressions conservatively. A model can be treated as nonconvex
when its convexity cannot be established from its representation. When providing
convexity information, distinguish the curvature of a function from the
convexity of its bounded constraint. See
[user-specified convexity](concepts.md#convexity-specified-by-the-user).
Use `--convex` only when the complete model is known to be convex; it changes
the assumptions behind the reported guarantee.

## Why the reformulated model looks different

The **Problem instance** report compares the original model with SHOT's
reformulation. Common changes include:

| Transformation | Effect |
| --- | --- |
| Expression partitioning | Introduces auxiliary variables and linking constraints for parts of nonlinear expressions in the objective or constraints, so that separate parts can be approximated more effectively. A nonlinear objective can become a linear expression in these auxiliary variables. |
| Product reformulation | Introduces auxiliary variables and constraints for eligible bilinear terms; some products involving discrete variables can be reformulated exactly. |
| Quadratic decomposition | Rewrites supported convex quadratic expressions using lifting variables and eigenvalue or LDL decomposition. |
| Objective epigraph | Replaces a nonlinear objective with an auxiliary variable and a constraint linking it to the original expression. A minimized convex objective can become a linear objective plus a convex nonlinear constraint. |
| Bound tightening | Narrows variable intervals using model constraints and, when enabled, optimization subproblems. |

These changes can increase both variable and constraint counts while preserving
the intended solution. A nonconvex original problem can sometimes become convex
after reformulation. Conversely, some nonconvex terms remain and are handled
heuristically. Read the classification of **both** models and the final status.

Public primal solution vectors use the original variable order. Debug dumps
and NLP rows marked `-R` can contain auxiliary variables and refer to
reformulated constraint indices. Compare `originalproblem.txt` with
`reformulatedproblem.txt` to trace a transformation; see
[debugging SHOT](DebuggingSHOT.md#whole-problem-dumps-written-once-per-run).

Configure reformulation settings before `finalize()` and `setProblem()`.
Changing a setting after preparation does not rebuild the model. See
[configuration](Configuration.md) and the generated
[settings reference](reference/settings.md) for controls.

## Build large models efficiently

Use bulk variable, constraint, and term containers instead of repeatedly merging
one term into a large expression. The tutorial's
[large-model chapter](tutorial/building-large-problems.ipynb) gives the APIs and
explains the cost of repeated insertion. Keep original variable indices/order
when creating starting points, and check status and feasibility before consuming
the returned solution.
