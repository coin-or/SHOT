"""
Tests for writing constraints as comparisons, e.g., problem.addConstraint(x1 * x2 <= 5), and objectives as
expressions, e.g., problem.setObjective(SHOTpy.exp(x) + x * y).

The tests of a ConstraintExpression check its exact bounds before it is added. finalize() decides the class of a
constraint and can negate it or split it into <name> and <name>_rf, so after finalize() the tests read the
constraints back with problem.getConstraint(name) and check that they describe the same set: points on both sides of
each bound, including constants and both sides of equalities and ranges.
"""

import math

import pytest


def make_problem(y_type=None, y_bounds=(0.1, 10.0)):
    """A solver, a problem and the variables x in [0.1, 10] (real) and y (real unless given)."""
    import SHOTpy

    solver = SHOTpy.Solver()
    solver.updateSetting("Output.Console.LogLevel", 6)
    problem = SHOTpy.Problem(solver.getEnvironment())

    x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.1, 10.0)
    y = SHOTpy.Variable("y", y_type or SHOTpy.VariableType.Real, y_bounds[0], y_bounds[1])
    problem.addVariable(x)
    problem.addVariable(y)

    return solver, problem, x, y


def finalize(problem):
    """Finalize the problem, with a zero objective if it has none."""
    if problem.objectiveFunction is None:
        problem.setObjective(0.0)

    problem.finalize()


def fulfilled(problem, name, point):
    """Whether the constraint, and its other half if finalize() has split it, is fulfilled at the point."""
    parts = [c for c in problem.numericConstraints if c.name in (name, name + "_rf")]
    assert parts, f"The problem has no constraint {name}"
    return all(part.isFulfilled(point) for part in parts)


class TestConstraintExpression:
    """The exact bounds of the ConstraintExpression created by each comparison, before it is added."""

    @pytest.mark.parametrize("comparison, lower, upper", [
        (lambda x, y: x <= 3, None, 3.0),
        (lambda x, y: x >= 3, 3.0, None),
        (lambda x, y: x == 3, 3.0, 3.0),
        (lambda x, y: 2 <= x, 2.0, None),
        (lambda x, y: 2 >= x, None, 2.0),
        (lambda x, y: 2 == x, 2.0, 2.0),
        (lambda x, y: x * y <= 5, None, 5.0),
        (lambda x, y: x <= y, None, 0.0),
        (lambda x, y: x >= y, 0.0, None),
        (lambda x, y: x == y, 0.0, 0.0),
        (lambda x, y: x * y <= x + 1, None, 0.0),
        (lambda x, y: x * y >= y, 0.0, None),
        (lambda x, y: x + 1 == x * y, 0.0, 0.0),
    ])
    def test_bounds(self, comparison, lower, upper):
        import SHOTpy

        _, _, x, y = make_problem()
        constraint = comparison(x, y)

        assert isinstance(constraint, SHOTpy.ConstraintExpression)
        assert constraint.lowerBound == (lower if lower is not None else SHOTpy.SHOT_DBL_MIN)
        assert constraint.upperBound == (upper if upper is not None else SHOTpy.SHOT_DBL_MAX)
        assert isinstance(constraint.expression, SHOTpy.Expression)

    def test_inequality(self):
        import SHOTpy

        _, _, x, y = make_problem()
        assert (SHOTpy.inequality(1, x * y, 5).lowerBound, SHOTpy.inequality(1, x * y, 5).upperBound) == (1.0, 5.0)
        assert (SHOTpy.inequality(1, x, 5).lowerBound, SHOTpy.inequality(1, x, 5).upperBound) == (1.0, 5.0)

    def test_infinite_bounds_are_missing_bounds(self):
        """SHOT marks a missing bound with SHOT_DBL_MIN / SHOT_DBL_MAX, so infinite bounds are given those values."""
        import SHOTpy

        _, _, x, y = make_problem()
        assert (x <= math.inf).upperBound == SHOTpy.SHOT_DBL_MAX
        assert (x >= -math.inf).lowerBound == SHOTpy.SHOT_DBL_MIN

        constraint = SHOTpy.inequality(-math.inf, x * y, math.inf)
        assert (constraint.lowerBound, constraint.upperBound) == (SHOTpy.SHOT_DBL_MIN, SHOTpy.SHOT_DBL_MAX)

    @pytest.mark.parametrize("comparison", [
        lambda x, y: x <= math.nan,
        lambda x, y: x >= math.nan,
        lambda x, y: x == math.nan,
        lambda x, y: x == math.inf,
        lambda x, y: x * y == -math.inf,
        lambda x, y: x <= -math.inf,
        lambda x, y: x >= math.inf,
    ])
    def test_invalid_bounds(self, comparison):
        _, _, x, y = make_problem()
        with pytest.raises(ValueError):
            comparison(x, y)

    @pytest.mark.parametrize("lower, upper", [(5, 1), (math.nan, 1), (1, math.nan), (math.inf, math.inf)])
    def test_inequality_with_invalid_bounds(self, lower, upper):
        import SHOTpy

        _, _, x, y = make_problem()
        with pytest.raises(ValueError):
            SHOTpy.inequality(lower, x * y, upper)

    def test_repr(self):
        _, _, x, y = make_problem()
        assert repr(x * y <= 5).startswith("<ConstraintExpression: -inf <= ")


class TestAddConstraint:
    """After finalize(), a constraint added as a comparison describes the same set of points."""

    def test_bounds_with_a_number(self):
        _, problem, x, y = make_problem()
        problem.addConstraint(x * y <= 5, "upper")
        problem.addConstraint(x + y >= 3, "lower")
        problem.addConstraint(x * y == 4, "equal")
        finalize(problem)

        assert fulfilled(problem, "upper", [2.0, 2.5]) and not fulfilled(problem, "upper", [2.0, 2.6])
        assert fulfilled(problem, "lower", [1.0, 2.0]) and not fulfilled(problem, "lower", [1.0, 1.9])
        assert fulfilled(problem, "equal", [2.0, 2.0])
        assert not fulfilled(problem, "equal", [2.0, 2.1]) and not fulfilled(problem, "equal", [2.0, 1.9])

    def test_constant_offsets(self):
        """Constants on either side are kept, wherever finalize() moves them."""
        _, problem, x, y = make_problem()
        problem.addConstraint(x + 1 <= 5, "offset")
        problem.addConstraint(x * y + 2 >= x + 3, "both_sides")
        problem.addConstraint(3 - x == y - 1, "equal")
        finalize(problem)

        assert fulfilled(problem, "offset", [4.0, 1.0]) and not fulfilled(problem, "offset", [4.1, 1.0])
        # x*y - x >= 1
        assert fulfilled(problem, "both_sides", [2.0, 1.5]) and not fulfilled(problem, "both_sides", [2.0, 1.4])
        # x + y == 4
        assert fulfilled(problem, "equal", [1.0, 3.0])
        assert not fulfilled(problem, "equal", [1.0, 3.1]) and not fulfilled(problem, "equal", [1.0, 2.9])

    def test_comparisons_of_two_expressions(self):
        _, problem, x, y = make_problem()
        problem.addConstraint(x * y <= x + 1, "upper")
        problem.addConstraint(2 * x >= y, "lower")
        problem.addConstraint(x == y, "equal")
        finalize(problem)

        # x*y - x <= 1
        assert fulfilled(problem, "upper", [1.0, 2.0]) and not fulfilled(problem, "upper", [1.0, 2.1])
        assert fulfilled(problem, "lower", [2.0, 4.0]) and not fulfilled(problem, "lower", [2.0, 4.1])
        assert fulfilled(problem, "equal", [2.0, 2.0])
        assert not fulfilled(problem, "equal", [2.0, 2.1]) and not fulfilled(problem, "equal", [2.0, 1.9])

    @pytest.mark.parametrize("expression, inside, below, above", [
        (lambda x, y: x * y, [1.0, 3.0], [1.0, 0.9], [1.0, 5.1]),
        (lambda x, y: x + 2 * y, [1.0, 1.0], [0.5, 0.2], [3.0, 1.1]),
        (lambda x, y: x, [3.0, 1.0], [0.9, 1.0], [5.1, 1.0]),
    ])
    def test_range_both_sides(self, expression, inside, below, above):
        """inequality(1, f, 5) excludes points below 1 and above 5, also after finalize() splits it."""
        import SHOTpy

        _, problem, x, y = make_problem()
        problem.addConstraint(SHOTpy.inequality(1, expression(x, y), 5), "range")
        finalize(problem)

        assert fulfilled(problem, "range", inside)
        assert not fulfilled(problem, "range", below)
        assert not fulfilled(problem, "range", above)

    def test_class_is_chosen_by_finalize(self):
        """The classes and the split, with the settings that decide them set explicitly."""
        import SHOTpy

        solver, problem, x, y = make_problem()
        solver.updateSetting("Model.Reformulation.Quadratics.ExtractStrategy", 1)  # extract to the same constraint
        solver.updateSetting("Model.Reformulation.Quadratics.Strategy", 0)  # quadratics as nonlinear: split ranges

        problem.addConstraint(2 * x * y + 3 * x <= 5, "quadratic")
        problem.addConstraint(x + 2 * y >= 1, "linear")
        problem.addConstraint(SHOTpy.exp(x) + y <= 3, "nonlinear")
        problem.addConstraint(SHOTpy.inequality(1, x * y, 5), "split")
        finalize(problem)

        assert type(problem.getConstraint("quadratic")) is SHOTpy.QuadraticConstraint
        assert type(problem.getConstraint("linear")) is SHOTpy.LinearConstraint
        assert type(problem.getConstraint("nonlinear")) is SHOTpy.NonlinearConstraint
        assert problem.getConstraint("split") is not None
        assert problem.getConstraint("split_rf") is not None

    def test_linear_comparison_is_a_linear_constraint(self):
        """A linear comparison is added as a LinearConstraint, with its constant, as if created from the class."""
        import SHOTpy

        _, problem, x, y = make_problem()
        problem.addConstraint(2 * (x - 3 * y) / 4 + 1 - x <= 5, "linear")

        constraint = problem.getConstraint("linear")
        assert type(constraint) is SHOTpy.LinearConstraint
        # -0.5*x - 1.5*y + 1
        assert abs(constraint.calculateFunctionValue([2.0, 1.0]) - (-1.0 - 1.5 + 1.0)) < 1e-12

    def test_linear_range_is_not_split(self):
        """finalize() splits a two-sided nonlinear constraint but not a linear one, also when given as a comparison."""
        import SHOTpy

        _, problem, x, y = make_problem()
        problem.addConstraint(SHOTpy.inequality(1, x + 2 * y, 8), "range")
        finalize(problem)

        assert [c.name for c in problem.numericConstraints] == ["range"]
        assert type(problem.getConstraint("range")) is SHOTpy.LinearConstraint
        assert fulfilled(problem, "range", [1.0, 1.0])
        assert not fulfilled(problem, "range", [0.5, 0.2]) and not fulfilled(problem, "range", [4.0, 2.1])

    def test_linear_terms_of_the_same_variable_are_combined(self):
        import SHOTpy

        _, problem, x, y = make_problem()
        problem.addConstraint(x + y - x + 2 * x <= 4, "combined")
        constraint = problem.getConstraint("combined")

        assert type(constraint) is SHOTpy.LinearConstraint
        assert len(constraint.linearTerms) == 2

    def test_quadratics_left_as_expressions(self):
        """With quadratic extraction off, a quadratic comparison stays a nonlinear constraint."""
        import SHOTpy

        solver, problem, x, y = make_problem()
        solver.updateSetting("Model.Reformulation.Quadratics.ExtractStrategy", 0)  # do not extract
        problem.addConstraint(x * y <= 5, "c")
        finalize(problem)

        assert type(problem.getConstraint("c")) is SHOTpy.NonlinearConstraint
        assert fulfilled(problem, "c", [2.0, 2.5]) and not fulfilled(problem, "c", [2.0, 2.6])

    def test_mixed_with_the_class_based_api(self):
        """Constraints and objectives created from the classes can still be added alongside comparisons."""
        import SHOTpy

        _, problem, x, y = make_problem()
        problem.addConstraint(x + y <= 4, "comparison")
        problem.addConstraint(SHOTpy.LinearConstraint("linear", SHOTpy.LinearTerms([SHOTpy.LinearTerm(1.0, x)]),
                                                      SHOTpy.SHOT_DBL_MIN, 3.0))
        problem.addConstraint(SHOTpy.NonlinearConstraint("nonlinear", SHOTpy.exp(y), SHOTpy.SHOT_DBL_MIN, 5.0))
        objective = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        objective.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(objective)
        finalize(problem)

        assert {c.name for c in problem.numericConstraints} == {"comparison", "linear", "nonlinear"}
        assert fulfilled(problem, "linear", [3.0, 1.0]) and not fulfilled(problem, "linear", [3.1, 1.0])


class TestNames:
    """Names of constraints added as comparisons, and reading them back by name."""

    def test_default_names(self):
        _, problem, x, y = make_problem()
        problem.addConstraint(x <= 3)
        problem.addConstraint(y <= 4, "named")
        problem.addConstraint(x + y <= 5)
        finalize(problem)

        assert {c.name for c in problem.numericConstraints} == {"constraint_0", "named", "constraint_2"}
        assert problem.getConstraint("constraint_2").name == "constraint_2"

    def test_missing_name(self):
        _, problem, x, y = make_problem()
        problem.addConstraint(x <= 3, "c")
        finalize(problem)

        with pytest.raises(KeyError):
            problem.getConstraint("d")

    def test_duplicate_names(self):
        """Names are not checked when constraints are added; reading back an ambiguous name raises."""
        _, problem, x, y = make_problem()
        problem.addConstraint(x <= 3, "c")
        problem.addConstraint(y <= 4, "c")
        finalize(problem)

        with pytest.raises(ValueError):
            problem.getConstraint("c")


class TestSetObjective:
    """Objectives set as expressions, checked after finalize()."""

    def test_nonlinear_objective(self):
        import SHOTpy

        _, problem, x, y = make_problem()
        problem.setObjective(SHOTpy.exp(x) + x * y)
        problem.finalize()

        objective = problem.objectiveFunction
        assert type(objective) is SHOTpy.NonlinearObjectiveFunction
        assert objective.direction == SHOTpy.ObjectiveDirection.Minimize
        assert abs(objective.calculateValue([1.0, 2.0]) - (math.e + 2.0)) < 1e-10

    def test_quadratic_objective(self):
        import SHOTpy

        _, problem, x, y = make_problem()
        problem.setObjective(x * y + x**2 - 1)
        problem.finalize()

        objective = problem.objectiveFunction
        assert type(objective) is SHOTpy.QuadraticObjectiveFunction
        assert abs(objective.calculateValue([2.0, 3.0]) - 9.0) < 1e-10

    def test_linear_objective_and_direction(self):
        import SHOTpy

        _, problem, x, y = make_problem()
        problem.setObjective(x + 2 * y + 3, SHOTpy.ObjectiveDirection.Maximize)
        problem.finalize()

        objective = problem.objectiveFunction
        assert type(objective) is SHOTpy.LinearObjectiveFunction
        assert objective.direction == SHOTpy.ObjectiveDirection.Maximize
        assert abs(objective.calculateValue([1.0, 2.0]) - 8.0) < 1e-10

    def test_variable_objective(self):
        import SHOTpy

        _, problem, x, y = make_problem()
        problem.setObjective(y, direction=SHOTpy.ObjectiveDirection.Maximize)
        problem.finalize()

        objective = problem.objectiveFunction
        assert type(objective) is SHOTpy.LinearObjectiveFunction
        assert abs(objective.calculateValue([1.0, 2.0]) - 2.0) < 1e-10

    def test_constant_objective(self):
        import SHOTpy

        _, problem, x, y = make_problem()
        with pytest.raises(ValueError):
            problem.setObjective(math.nan)

        problem.setObjective(3.0)
        problem.finalize()

        assert abs(problem.objectiveFunction.calculateValue([1.0, 2.0]) - 3.0) < 1e-10


class TestSharedExpressions:
    """finalize() simplifies expressions in place, so an expression used in several places must give the same
    function everywhere."""

    def test_expression_in_two_constraints(self):
        _, problem, x, y = make_problem()
        f = -(2 * x + 1)
        problem.addConstraint(f <= 100, "a")
        problem.addConstraint(f <= 100, "b")
        finalize(problem)

        a = problem.getConstraint("a").calculateFunctionValue([2.0, 1.0])
        b = problem.getConstraint("b").calculateFunctionValue([2.0, 1.0])
        assert a == b == -5.0

    def test_same_constraint_expression_added_twice(self):
        _, problem, x, y = make_problem()
        constraint = -(2 * x + 1) >= -6
        problem.addConstraint(constraint, "a")
        problem.addConstraint(constraint, "b")
        finalize(problem)

        for name in ("a", "b"):
            assert fulfilled(problem, name, [2.5, 1.0]) and not fulfilled(problem, name, [2.6, 1.0])

    def test_expression_in_a_constraint_and_the_objective(self):
        _, problem, x, y = make_problem()
        f = -(2 * x + 1)
        problem.addConstraint(f <= 100, "a")
        problem.setObjective(f)
        problem.finalize()

        assert problem.getConstraint("a").calculateFunctionValue([2.0, 1.0]) == -5.0
        assert problem.objectiveFunction.calculateValue([2.0, 1.0]) == -5.0

    def test_subexpression_used_twice_in_one_expression(self):
        import SHOTpy

        _, problem, x, y = make_problem()
        e = -(2 * x + 1)
        problem.addConstraint(e + SHOTpy.exp(e) <= 100, "a")
        finalize(problem)

        assert abs(problem.getConstraint("a").calculateFunctionValue([2.0, 1.0]) - (-5.0 + math.exp(-5.0))) < 1e-12

    def test_class_based_constraints_sharing_an_expression(self):
        import SHOTpy

        _, problem, x, y = make_problem()
        f = -(2 * x + 1)
        problem.addConstraint(SHOTpy.NonlinearConstraint("a", f, SHOTpy.SHOT_DBL_MIN, 100.0))
        problem.addConstraint(SHOTpy.NonlinearConstraint("b", f, SHOTpy.SHOT_DBL_MIN, 100.0))
        finalize(problem)

        assert problem.getConstraint("a").calculateFunctionValue([2.0, 1.0]) == -5.0
        assert problem.getConstraint("b").calculateFunctionValue([2.0, 1.0]) == -5.0

    def test_shared_arc_functions(self):
        """The copy of a shared expression also covers asin, acos and atan."""
        import SHOTpy

        _, problem, x, y = make_problem()
        f = SHOTpy.atan(x) + SHOTpy.asin(x / 10) + SHOTpy.acos(y / 10)
        problem.addConstraint(f <= 100, "a")
        problem.addConstraint(f <= 100, "b")
        finalize(problem)

        expected = math.atan(2.0) + math.asin(0.2) + math.acos(0.1)
        for name in ("a", "b"):
            assert abs(problem.getConstraint(name).calculateFunctionValue([2.0, 1.0]) - expected) < 1e-12

    def test_user_expression_is_not_changed(self):
        """Simplifying a constraint does not change the expression object the user keeps."""
        _, problem, x, y = make_problem()
        f = -(2 * x + 1)
        before = repr(f)
        problem.addConstraint(f <= 100, "a")
        finalize(problem)

        assert repr(f) == before


class TestSolve:
    """A convex model written with comparisons is solved to its known optimum, as the same model written with classes.

    minimize (x - 3)^2 - x + 2*(y - 2.5)^2
    s.t.     x^2 + y^2 <= 20,  2 <= x + y <= 8,  exp(x) + 2/y <= 10,  x in [0.1, 10], y in {1, ..., 10}

    Without the last constraint x would be 3.5, so exp(x) + 2/y <= 10 is active: x = log(10 - 2/y). Of the integers
    closest to 2.5, y = 3 gives the smaller objective, so the optimum is x = log(28/3), y = 3.
    """

    X = math.log(28.0 / 3.0)
    OPTIMUM = (X - 3.0) ** 2 - X + 2.0 * 0.5**2

    def _solve(self, build):
        import SHOTpy

        # y >= 1 keeps the denominator of 2 / y away from zero
        solver, problem, x, y = make_problem(SHOTpy.VariableType.Integer, (1.0, 10.0))
        solver.updateSetting("Termination.ObjectiveGap.Relative", 1e-6)
        solver.updateSetting("Termination.ObjectiveGap.Absolute", 1e-6)
        solver.updateSetting("Termination.TimeLimit", 60.0)
        build(problem, x, y)
        problem.finalize()
        solver.setProblem(problem)

        assert solver.solveProblem()
        # The model is convex, so SHOT closes the gap
        assert solver.getTerminationReason() in (SHOTpy.TerminationReason.RelativeGap,
                                                  SHOTpy.TerminationReason.AbsoluteGap)
        x_value, y_value = solver.getPrimalSolution().point
        return solver.getPrimalBound(), x_value, y_value

    def test_known_optimum_with_both_formulations(self):
        import SHOTpy

        def with_comparisons(problem, x, y):
            problem.addConstraint(x**2 + y**2 <= 20, "c1")
            problem.addConstraint(SHOTpy.inequality(2, x + y, 8), "c2")
            problem.addConstraint(SHOTpy.exp(x) + 2 / y <= 10, "c3")
            problem.setObjective((x - 3)**2 - x + 2 * (y - 2.5)**2)

        def with_classes(problem, x, y):
            problem.addConstraint(SHOTpy.NonlinearConstraint("c1", x**2 + y**2, SHOTpy.SHOT_DBL_MIN, 20.0))
            problem.addConstraint(SHOTpy.LinearConstraint(
                "c2", SHOTpy.LinearTerms([SHOTpy.LinearTerm(1.0, x), SHOTpy.LinearTerm(1.0, y)]), 2.0, 8.0))
            problem.addConstraint(SHOTpy.NonlinearConstraint("c3", SHOTpy.exp(x) + 2 / y, SHOTpy.SHOT_DBL_MIN, 10.0))
            problem.setObjective(SHOTpy.NonlinearObjectiveFunction(
                SHOTpy.ObjectiveDirection.Minimize, (x - 3)**2 - x + 2 * (y - 2.5)**2, 0.0))

        for build in (with_comparisons, with_classes):
            objective, x_value, y_value = self._solve(build)
            assert abs(objective - self.OPTIMUM) < 1e-5
            assert y_value == 3.0
            assert abs(x_value - self.X) < 1e-5


class TestErrors:
    """Comparisons that would silently lose a bound or compare objects raise."""

    @pytest.mark.parametrize("action", [
        lambda x, y: 1 <= x <= 5,
        lambda x, y: 1 <= x * y <= 5,
        lambda x, y: bool(x == y),
        lambda x, y: x != y,
        lambda x, y: x * y != 2,
        lambda x, y: y in [x],
        lambda x, y: (1 <= x) <= 5,
    ])
    def test_type_error(self, action):
        _, _, x, y = make_problem()
        with pytest.raises(TypeError):
            action(x, y)

    def test_none(self):
        import SHOTpy

        _, problem, x, y = make_problem()
        assert (x == None) is False  # noqa: E711
        assert (x != None) is True  # noqa: E711
        with pytest.raises(TypeError):
            SHOTpy.inequality(1, None, 5)
        with pytest.raises(TypeError):
            problem.setObjective(None)
        with pytest.raises(TypeError):
            problem.addConstraint(None)


class TestHashing:
    """Variables and expressions can still be used in sets and as dict keys."""

    def test_variables(self):
        _, _, x, y = make_problem()

        assert {x: 1, y: 2}[x] == 1
        assert x in {x, y}
        assert x in [x, y]
        assert x is not y
        assert hash(x) == hash(x)

    def test_variable_read_back_from_the_problem(self):
        """A variable read back from the problem is the same object, so it finds the entries of the original."""
        _, problem, x, y = make_problem()
        values = {x: 1, y: 2}

        assert problem.getVariable(0) is x
        assert values[problem.getVariable(0)] == 1
        assert values[problem.allVariables[1]] == 2

    def test_expressions(self):
        _, _, x, y = make_problem()
        expression = x * y

        assert {expression: 1}[expression] == 1
