# Understanding solver results

## Feasible solutions and bounds

A primal solution assigns values to the variables while satisfying the constraints and integrality requirements within tolerances. For minimization, its objective gives an upper bound on the optimum. A valid dual bound is a lower bound. For maximization, the directions reverse.

The gap measures the distance between these bounds. A small gap is useful only together with the feasibility and optimality tolerances and the solver's reported status. Reaching a time or iteration limit does not establish optimality.

## Outer approximation

SHOT constructs linear approximations of nonlinear expressions and solves mixed-integer subproblems. Primal searches seek feasible solutions to the original nonlinear problem. Convexity determines when linearizations provide globally valid bounds; nonconvex models require additional care and do not generally have the same guarantees.

## Solver backends

The available MIP/NLP backends depend on compilation options and installed licenses. Select a backend available in your build and consult the [settings reference](reference/settings.md). See the [compilation guide](CompilationInstructions.md) for dependencies and the [debugging guide](DebuggingSHOT.md) for intermediate problems and diagnostic logs.
