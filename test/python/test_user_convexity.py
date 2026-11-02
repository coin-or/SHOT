"""
Tests that the convexity of the function of a constraint or of the objective function can be specified by the user, in
which case it is used instead of detected, and that the solution status tells that the global optimality of a solution
depends on it.

The problem has the constraints exp(y1)/y2 + exp(y2)/y1 <= 8 and -10 <= -exp(y2)/y1 - y1^2, which share variables.
Both are convex, the second one since its function is concave and it only has a lower bound, but their convexity is
not detected. The other problem maximizes the concave function y1 + 2*y2 - exp(y1)/y2 - exp(y2)/y1, whose concavity is
not detected either.
"""

import pytest


def make_solver(settings=None):
    import SHOTpy

    solver = SHOTpy.Solver()
    solver.updateSetting("Output.Console.LogLevel", 6)
    solver.updateSetting("Dual.MIP.NumberOfThreads", 1)

    # Partitioning the sums would introduce auxiliary constraints whose convexity is not known
    solver.updateSetting("Model.Reformulation.Constraint.PartitionNonlinearTerms", 0)

    for name, value in (settings or {}).items():
        solver.updateSetting(name, value)
    return solver


def build_problem(solver, specify_convexity):
    import SHOTpy

    problem = SHOTpy.Problem(solver.getEnvironment())

    y1 = SHOTpy.Variable("y1", SHOTpy.VariableType.Real, 0.5, 2.0)
    y2 = SHOTpy.Variable("y2", SHOTpy.VariableType.Integer, 1.0, 3.0)

    for variable in (y1, y2):
        problem.addVariable(variable)

    problem.setObjective(-y1 - 2 * y2)

    convex = SHOTpy.Convexity.Convex if specify_convexity else SHOTpy.Convexity.NotSet
    concave = SHOTpy.Convexity.Concave if specify_convexity else SHOTpy.Convexity.NotSet

    problem.addConstraint(SHOTpy.exp(y1) / y2 + SHOTpy.exp(y2) / y1 <= 8, "c1", functionConvexity=convex)
    problem.addConstraint(-SHOTpy.exp(y2) / y1 - y1 * y1 >= -10, "c2", functionConvexity=concave)
    problem.finalize()

    return problem


def solve(specify_convexity, settings=None):
    solver = make_solver(settings)
    assert solver.setProblem(build_problem(solver, specify_convexity))
    solver.solveProblem()
    return solver


def test_detected_convexity():
    import SHOTpy

    solver = solve(False)

    # Otherwise the tests do not show that the specified convexity is used
    assert solver.getOriginalProblem().properties.convexity == SHOTpy.ProblemConvexity.Nonconvex
    assert len(solver.getReformulatedProblem().allVariables) > len(solver.getOriginalProblem().allVariables)
    assert solver.getUserSpecifiedConvexityDescription() == ""


def test_specified_convexity():
    import SHOTpy

    solver = solve(True)
    original = solver.getOriginalProblem()

    for constraint in original.numericConstraints:
        assert constraint.properties.convexity == SHOTpy.Convexity.Convex, constraint.name

    # The lower bound of c2 has been moved to the right-hand side by negating its function
    c2 = original.getConstraint("c2")
    assert c2.properties.userFunctionConvexity == SHOTpy.Convexity.Convex
    assert c2.properties.functionConvexity == SHOTpy.Convexity.Convex

    assert original.properties.convexity == SHOTpy.ProblemConvexity.Convex
    assert original.properties.numberOfUserSpecifiedConvexConstraints == 2

    # The constraints are not partitioned, since the convexity of the parts of their functions is not known
    reformulated = solver.getReformulatedProblem()
    assert reformulated.properties.convexity == SHOTpy.ProblemConvexity.Convex
    assert len(reformulated.allVariables) == len(original.allVariables)

    assert solver.getModelReturnStatus() == SHOTpy.ModelReturnStatus.OptimalGlobal
    assert solver.getPrimalBound() == pytest.approx(solve(False).getPrimalBound(), abs=1e-4)

    description = solver.getUserSpecifiedConvexityDescription()
    assert "2 constraints" in description
    assert "UserSpecifiedConvexity" in solver.getResultsOSrL()


def test_assume_convex():
    solver = solve(False, {"Model.Convexity.AssumeConvex": True})
    assert "whole problem" in solver.getUserSpecifiedConvexityDescription()


def test_set_function_convexity():
    import SHOTpy

    constraint = SHOTpy.NonlinearConstraint("c", -1e300, 1.0)
    constraint.setFunctionConvexity(SHOTpy.Convexity.Concave)
    assert constraint.properties.userFunctionConvexity == SHOTpy.Convexity.Concave

    constraint.setFunctionConvexity(SHOTpy.Convexity.NotSet)
    assert constraint.properties.userFunctionConvexity == SHOTpy.Convexity.NotSet

    with pytest.raises(ValueError):
        constraint.setFunctionConvexity(SHOTpy.Convexity.Linear)


def solve_objective_problem(specify_convexity, epigraph_strategy):
    import SHOTpy

    solver = make_solver(
        {
            "Model.Reformulation.ObjectiveFunction.PartitionNonlinearTerms": 0,
            "Model.Reformulation.ObjectiveFunction.EpigraphStrategy": epigraph_strategy,
        }
    )
    problem = SHOTpy.Problem(solver.getEnvironment())

    y1 = SHOTpy.Variable("y1", SHOTpy.VariableType.Real, 0.5, 2.0)
    y2 = SHOTpy.Variable("y2", SHOTpy.VariableType.Integer, 1.0, 3.0)

    for variable in (y1, y2):
        problem.addVariable(variable)

    problem.setObjective(
        y1 + 2 * y2 - SHOTpy.exp(y1) / y2 - SHOTpy.exp(y2) / y1,
        SHOTpy.ObjectiveDirection.Maximize,
        functionConvexity=SHOTpy.Convexity.Concave if specify_convexity else SHOTpy.Convexity.NotSet,
    )
    problem.addConstraint(y1 + y2 <= 4, "c1")
    problem.finalize()

    assert solver.setProblem(problem)
    solver.solveProblem()
    return solver


# Unchanged and EpigraphConstraint, where the objective variable of the epigraph constraint is an auxiliary variable
@pytest.mark.parametrize("epigraph_strategy, number_of_auxiliary_variables", [(0, 0), (2, 1)])
def test_specified_objective_convexity(epigraph_strategy, number_of_auxiliary_variables):
    import SHOTpy

    detected = solve_objective_problem(False, epigraph_strategy)
    detected_original = detected.getOriginalProblem()

    # Otherwise the test does not show that the specified convexity is used
    assert detected_original.properties.convexity == SHOTpy.ProblemConvexity.Nonconvex
    assert len(detected.getReformulatedProblem().allVariables) > (
        len(detected_original.allVariables) + number_of_auxiliary_variables
    )

    solver = solve_objective_problem(True, epigraph_strategy)
    original = solver.getOriginalProblem()

    assert original.objectiveFunction.properties.userFunctionConvexity == SHOTpy.Convexity.Concave
    assert original.objectiveFunction.properties.convexity == SHOTpy.Convexity.Concave
    assert original.properties.convexity == SHOTpy.ProblemConvexity.Convex
    assert original.properties.isObjectiveConvexityUserSpecified

    # The objective function is not partitioned
    reformulated = solver.getReformulatedProblem()
    assert reformulated.properties.convexity == SHOTpy.ProblemConvexity.Convex
    assert len(reformulated.allVariables) == len(original.allVariables) + number_of_auxiliary_variables

    assert solver.getModelReturnStatus() == SHOTpy.ModelReturnStatus.OptimalGlobal
    assert solver.getPrimalBound() == pytest.approx(detected.getPrimalBound(), abs=1e-4)
    assert "the objective function" in solver.getUserSpecifiedConvexityDescription()


def test_set_objective_function_convexity():
    import SHOTpy

    objective = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
    objective.setFunctionConvexity(SHOTpy.Convexity.Convex)
    assert objective.properties.userFunctionConvexity == SHOTpy.Convexity.Convex

    with pytest.raises(ValueError):
        objective.setFunctionConvexity(SHOTpy.Convexity.Unknown)
