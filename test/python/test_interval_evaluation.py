"""
Tests for the Interval class and the interval evaluation of the functions in the Python API.

These tests verify that:
1. Intervals can be created from bounds, numbers and tuples, and invalid bounds raise ValueError
2. The interval arithmetic and the conversions work, and a division by zero raises IntervalError
3. The bounds of constraints, objective functions and expressions over a box contain their values in the box
4. A box that is a point gives the value at the point
5. Points and boxes are told apart, and boxes of the wrong size raise ValueError
"""

import itertools
import math

import pytest


def _build_problem(problem):
    """Builds a problem with a nonlinear constraint and objective function of several kinds of terms."""
    import SHOTpy

    x = problem.addVariable("x", SHOTpy.VariableType.Real, -5.0, 5.0)
    y = problem.addVariable("y", SHOTpy.VariableType.Real, 0.1, 5.0)
    z = problem.addVariable("z", SHOTpy.VariableType.Real, 0.1, 5.0)

    def function():
        return (2 * x - y + x * y + 3 * y * z + 2 * x * y * z - x * z * z + 1.5 * z**0.5 * y
                + SHOTpy.exp(x) * y + SHOTpy.log(z) + SHOTpy.sin(x * z))

    problem.addConstraint(function() <= 10, "nl")
    problem.addConstraint(x + 2 * y <= 10, "lin")
    problem.addConstraint(SHOTpy.exp(y) / x <= 10, "divide")
    problem.setObjective(function(), SHOTpy.ObjectiveDirection.Minimize)
    problem.finalize()

    constraints = {c.name: c for c in problem.numericConstraints}
    return constraints, (x, y, z), function


def _grid(box, steps=4):
    """The points of a grid in the box."""
    axes = [[lower + i * (upper - lower) / steps for i in range(steps + 1)] for lower, upper in box]
    return [list(point) for point in itertools.product(*axes)]


def _assert_contains(bounds, value):
    tolerance = 1e-9 * (1.0 + abs(value))
    assert bounds.lower - tolerance <= value <= bounds.upper + tolerance, f"{value} is not in {bounds}"


def _assert_unbounded(bounds):
    import SHOTpy

    assert bounds.lower == SHOTpy.SHOT_DBL_MIN
    assert bounds.upper == SHOTpy.SHOT_DBL_MAX


class TestIntervalClass:
    def test_construction(self):
        import SHOTpy

        interval = SHOTpy.Interval(-1.0, 2.5)
        assert (interval.lower, interval.upper) == (-1.0, 2.5)
        assert interval.width == 3.5
        assert interval.midpoint == 0.75

        assert SHOTpy.Interval(3) == SHOTpy.Interval(3.0, 3.0)
        assert SHOTpy.Interval((1, 2)) == SHOTpy.Interval(1.0, 2.0)
        assert repr(SHOTpy.Interval(1, 2.5)) == "Interval(1.0, 2.5)"

    def test_invalid_bounds_raise(self):
        import SHOTpy

        with pytest.raises(ValueError):
            SHOTpy.Interval(2.0, 1.0)
        with pytest.raises(ValueError):
            SHOTpy.Interval(math.nan, 1.0)
        with pytest.raises(ValueError):
            SHOTpy.Interval((1.0, 2.0, 3.0))

    def test_conversions(self):
        import SHOTpy

        interval = SHOTpy.Interval(1.0, 2.0)
        lower, upper = interval
        assert (lower, upper) == (1.0, 2.0)
        assert tuple(interval) == (1.0, 2.0)

        assert interval == (1, 2)
        assert interval != SHOTpy.Interval(1.0, 3.0)
        assert interval != "not an interval"
        assert len({interval, SHOTpy.Interval(1.0, 2.0)}) == 1

        assert 1.5 in interval and interval.contains(2.0)
        assert 2.5 not in interval

    def test_arithmetic(self):
        import SHOTpy

        a = SHOTpy.Interval(1.0, 2.0)
        b = SHOTpy.Interval(-1.0, 3.0)

        assert a + b == (0.0, 5.0)
        assert a - b == (-2.0, 3.0)
        assert a * b == (-2.0, 6.0)
        assert a / SHOTpy.Interval(2.0, 4.0) == (0.25, 1.0)
        assert -a == (-2.0, -1.0)

        assert a + 1 == (2.0, 3.0)
        assert 1 - a == (-1.0, 0.0)
        assert 2 * a == (2.0, 4.0)
        assert 2 / a == (1.0, 2.0)

    def test_division_by_zero_raises(self):
        import SHOTpy

        with pytest.raises(SHOTpy.IntervalError):
            SHOTpy.Interval(1.0, 2.0) / SHOTpy.Interval(-1.0, 1.0)

        assert issubclass(SHOTpy.IntervalError, ArithmeticError)


class TestFunctionBounds:
    BOX = [(-1.0, 2.0), (1.0, 3.0), (0.5, 2.0)]

    def test_bounds_contain_the_values(self, problem):
        constraints, _, _ = _build_problem(problem)
        objective = problem.objectiveFunction

        constraint_bounds = constraints["nl"].calculateFunctionValue(self.BOX)
        objective_bounds = objective.calculateValue(self.BOX)

        for point in _grid(self.BOX):
            _assert_contains(constraint_bounds, constraints["nl"].calculateFunctionValue(point))
            _assert_contains(objective_bounds, objective.calculateValue(point))

    def test_box_of_a_point_gives_the_value(self, problem):
        import SHOTpy

        constraints, _, _ = _build_problem(problem)
        point = [0.7, 2.2, 1.3]
        box = [SHOTpy.Interval(value) for value in point]

        for bounds, value in (
            (constraints["nl"].calculateFunctionValue(box), constraints["nl"].calculateFunctionValue(point)),
            (problem.objectiveFunction.calculateValue(box), problem.objectiveFunction.calculateValue(point)),
        ):
            assert abs(bounds.lower - value) < 1e-9 * (1.0 + abs(value))
            assert abs(bounds.upper - value) < 1e-9 * (1.0 + abs(value))

    def test_linear_constraint(self, problem):
        constraints, _, _ = _build_problem(problem)

        # x + 2y with x in [-1, 2] and y in [1, 3]
        assert constraints["lin"].calculateFunctionValue(self.BOX) == (1.0, 8.0)

    def test_bounds_over_the_variable_bounds(self, problem):
        constraints, (x, _, _), _ = _build_problem(problem)
        bounds = problem.getVariableBounds()

        assert bounds[0] == (-5.0, 5.0) and x.getBound() == (-5.0, 5.0)
        assert constraints["nl"].getConstraintFunctionBounds() == constraints["nl"].calculateFunctionValue(bounds)
        assert tuple(constraints["lin"].getConstraintFunctionBounds()) == pytest.approx((-4.8, 15.0))
        assert problem.objectiveFunction.getBounds() == problem.objectiveFunction.calculateValue(bounds)

    def test_bounds_that_cannot_be_calculated(self, problem):
        """log(z) is not defined for negative z, and exp(y) / x has a denominator that can be zero."""
        constraints, _, _ = _build_problem(problem)
        outside_domain = [(-1.0, 2.0), (1.0, 3.0), (-2.0, -1.0)]

        _assert_unbounded(constraints["nl"].calculateFunctionValue(outside_domain))
        _assert_unbounded(problem.objectiveFunction.calculateValue(outside_domain))

        bounds = constraints["divide"].calculateFunctionValue(self.BOX)
        assert bounds.lower < -1e100 and bounds.upper > 1e100

        bounds = constraints["divide"].calculateFunctionValue([(1.0, 2.0), (1.0, 3.0), (0.5, 2.0)])
        _assert_contains(bounds, math.exp(1.0) / 2.0)
        _assert_contains(bounds, math.exp(3.0))


class TestExpressionBounds:
    def test_bounds_contain_the_values(self, problem):
        _, _, function = _build_problem(problem)
        expression = function()
        box = [(-1.0, 2.0), (1.0, 3.0), (0.5, 2.0)]

        bounds = expression.calculate(box)

        for point in _grid(box):
            _assert_contains(bounds, expression.calculate(point))

    def test_bounds_over_the_variable_bounds(self, problem):
        import SHOTpy

        _, (x, y, _), _ = _build_problem(problem)

        assert (x * y).getBounds() == (-25.0, 25.0)
        assert (x * y).getBounds() == (x * y).calculate(problem.getVariableBounds())
        assert tuple(SHOTpy.exp(y).getBounds()) == pytest.approx((math.exp(0.1), math.exp(5.0)))

    def test_expression_not_defined_in_the_box_raises(self, problem):
        import SHOTpy

        _, (x, _, z), _ = _build_problem(problem)

        with pytest.raises(SHOTpy.IntervalError):
            SHOTpy.log(z).calculate([(0.0, 1.0), (1.0, 2.0), (-2.0, -1.0)])

        with pytest.raises(SHOTpy.IntervalError):
            (SHOTpy.exp(z) / x).calculate([(-1.0, 1.0), (1.0, 2.0), (1.0, 2.0)])

        # Only the part of the box where the function is defined is used
        bounds = SHOTpy.log(z).calculate([(0.0, 1.0), (1.0, 2.0), (-2.0, 1.0)])
        assert bounds.upper == 0.0


class TestPointsAndBoxes:
    def test_points_and_boxes_are_told_apart(self, problem):
        import SHOTpy

        constraints, _, _ = _build_problem(problem)
        constraint = constraints["lin"]

        # Numbers, also integers, give a point and a float
        assert constraint.calculateFunctionValue([1, 2, 3]) == 5.0
        assert constraint.calculateFunctionValue([1.0, 2.0, 3.0]) == 5.0

        # Tuples and Intervals, also mixed with numbers, give a box and an Interval
        assert constraint.calculateFunctionValue([(1, 2), (2, 3), (3, 4)]) == (5.0, 8.0)
        assert constraint.calculateFunctionValue([SHOTpy.Interval(1, 2), 2, 3.0]) == (5.0, 6.0)
        assert constraint.calculateFunctionValue([(1, 2), 2, 3]) == (5.0, 6.0)

    def test_box_of_the_wrong_size_raises(self, problem):
        import SHOTpy

        constraints, (x, y, _), _ = _build_problem(problem)
        short = [(0.0, 1.0)]

        for evaluation in (
            lambda: constraints["nl"].calculateFunctionValue(short),
            lambda: problem.objectiveFunction.calculateValue(short),
            lambda: (x * y).calculate(short),
        ):
            with pytest.raises(ValueError):
                evaluation()

    def test_function_not_in_a_problem_raises(self):
        import SHOTpy

        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 1.0)
        constraint = SHOTpy.LinearConstraint("c", 0.0, 1.0)
        constraint.add(SHOTpy.LinearTerm(1.0, x))
        objective = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        objective.add(SHOTpy.LinearTerm(1.0, x))

        for evaluation in (
            lambda: constraint.calculateFunctionValue([(0.0, 1.0)]),
            lambda: constraint.getConstraintFunctionBounds(),
            lambda: objective.calculateValue([(0.0, 1.0)]),
            lambda: objective.getBounds(),
        ):
            with pytest.raises(ValueError):
                evaluation()


class TestCallback:
    def test_bounds_in_a_callback(self, solver, problem):
        """The functions of the problems in a callback context can be bounded over a box."""
        import SHOTpy

        x = problem.addVariable("x", SHOTpy.VariableType.Real, 0.0, 4.0)
        y = problem.addVariable("y", SHOTpy.VariableType.Integer, 0.0, 4.0)
        problem.addConstraint(x**2 + y**2 <= 9, "circle")
        problem.setObjective(-x - y, SHOTpy.ObjectiveDirection.Minimize)
        problem.finalize()

        results = []

        def callback(context):
            original = context.originalProblem
            circle = next(c for c in original.numericConstraints if c.name == "circle")
            results.append(circle.calculateFunctionValue([(0.0, 1.0), (1.0, 2.0)]))

        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.registerCallback(SHOTpy.CallbackLocation.NewPrimalSolution, callback)
        assert solver.setProblem(problem)
        solver.solveProblem()

        assert results
        assert all(bounds == (1.0, 5.0) for bounds in results)
