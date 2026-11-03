# Understanding solver results

## Feasible solutions and bounds

A primal solution assigns values to the variables while satisfying the constraints and integrality requirements within tolerances. For minimization, its objective gives an upper bound on the optimum. A valid dual bound is a lower bound. For maximization, the directions reverse.

The gap measures the distance between these bounds. A small gap is useful only together with the feasibility and optimality tolerances and the solver's reported status. Reaching a time or iteration limit does not establish optimality.

## Outer approximation

SHOT constructs linear approximations of nonlinear expressions and solves mixed-integer subproblems. Primal searches seek feasible solutions to the original nonlinear problem. Convexity determines when linearizations provide globally valid bounds; nonconvex models require additional care and do not generally have the same guarantees.

## Convexity specified by the user

SHOT detects the convexity of the objective function and of each constraint function, and regards a function as nonconvex when it cannot prove otherwise. When you know better, you can specify the convexity with `setFunctionConvexity()` on a quadratic or nonlinear constraint or objective function, in C++ and SHOTpy, or with the `functionConvexity` argument of `Problem.addConstraint()` and `Problem.setObjective()` in SHOTpy. The value is `Convex`, `Concave` or `Nonconvex`. For a constraint L <= f(x) <= U, it is the convexity of f(x), and the convexity of the constraint follows from it and the bounds: f(x) <= U is convex for a convex function, and L <= f(x) for a concave one. A minimized convex or maximized concave objective function gives a convex problem. Such a function is not reformulated, e.g., its sums are not partitioned, since the convexity of its parts is not known.

`Model.Convexity.AssumeConvex` (`--convex`) instead assumes the whole problem to be convex. In both cases, the global optimality of a solution depends on the assumption, which the solution report, the `UserSpecifiedConvexity` entry of the OSrL results and `getUserSpecifiedConvexityDescription()` tell.

## Solver backends

The available MIP/NLP backends depend on compilation options and installed licenses. Select a backend available in your build and consult the [settings reference](reference/settings.md). See the [compilation guide](CompilationInstructions.md) for dependencies and the [debugging guide](DebuggingSHOT.md) for intermediate problems and diagnostic logs.
