"""
Tests for the interval derivatives of the functions in the Python API.

These tests verify that:
1. calculateIntervalDerivatives returns the bounds of the value, the gradient and the Hessian, and the status
2. The bounds contain the derivatives at the points of the box
3. The status tells whether the function is defined and differentiable on the box
4. Invalid orders, boxes of the wrong size and functions that are not in a problem raise ValueError
5. A numerical convexity check with the Gershgorin bounds of the interval Hessian, as in the tutorial, works
"""

import itertools
import math
import random

import pytest


def check_convexity(derivatives, number_of_variables):
    """Whether the function passes the numerical convexity check on the box: True if it passes, False if it does not,
    which is inconclusive, and None if the bounds cannot be used, e.g., if the function is not twice continuously
    differentiable on the box. Each diagonal element of the Hessian must be at least the sum of the absolute values of
    the other elements of its row (Gershgorin's theorem)."""
    if not (derivatives.isDefinedOnBox and derivatives.differentiableOrder == 2 and derivatives.isBounded):
        return None

    hessian = derivatives.hessian

    for i in range(number_of_variables):
        diagonal = hessian[(i, i)].lower if (i, i) in hessian else 0.0
        others = sum(max(abs(h.lower), abs(h.upper)) for (j, k), h in hessian.items() if (j == i) != (k == i))

        if diagonal - others < 0.0:
            return False

    return True


def _build_problem(problem):
    import SHOTpy

    x = problem.addVariable("x", SHOTpy.VariableType.Real, -5.0, 5.0)
    y = problem.addVariable("y", SHOTpy.VariableType.Real, 0.1, 5.0)
    z = problem.addVariable("z", SHOTpy.VariableType.Real, 0.1, 5.0)

    def function():
        return (2 * x - y + x * y + 3 * y * z + 2 * x * y * z - x * z * z + 1.5 * z**0.5 * y
                + SHOTpy.exp(x) * y + SHOTpy.log(z) + SHOTpy.sin(x * z))

    problem.addConstraint(function() <= 10, "nl")
    problem.addConstraint(x + 2 * y <= 10, "lin")
    problem.setObjective(function(), SHOTpy.ObjectiveDirection.Minimize)
    problem.finalize()

    constraints = {c.name: c for c in problem.numericConstraints}
    return constraints, (x, y, z)


def _assert_contains(bounds, value):
    tolerance = 1e-8 * (1.0 + abs(value))
    assert bounds.lower - tolerance <= value <= bounds.upper + tolerance, f"{value} is not in {bounds}"


class TestResult:
    BOX = [(-1.0, 2.0), (1.0, 3.0), (0.5, 2.0)]

    def test_fields(self, problem):
        import SHOTpy

        constraints, _ = _build_problem(problem)
        derivatives = constraints["nl"].calculateIntervalDerivatives(self.BOX)

        assert isinstance(derivatives.value, SHOTpy.Interval)
        assert set(derivatives.gradient) == {0, 1, 2}
        assert all(isinstance(g, SHOTpy.Interval) for g in derivatives.gradient.values())
        assert all(i <= j for i, j in derivatives.hessian)
        assert derivatives.isDefinedOnBox and derivatives.differentiableOrder == 2 and derivatives.isBounded
        assert "IntervalDerivatives" in repr(derivatives)

        # The dicts are converted once, not for each access
        assert derivatives.gradient is derivatives.gradient
        assert derivatives.hessian is derivatives.hessian

    def test_value_equals_the_interval_evaluation(self, problem):
        constraints, _ = _build_problem(problem)

        for constraint in constraints.values():
            derivatives = constraint.calculateIntervalDerivatives(self.BOX)
            bounds = constraint.calculateFunctionValue(self.BOX)
            assert bounds.lower <= derivatives.value.upper and derivatives.value.lower <= bounds.upper

    def test_order(self, problem):
        constraints, _ = _build_problem(problem)
        constraint = constraints["nl"]

        assert not constraint.calculateIntervalDerivatives(self.BOX, 0).gradient
        assert not constraint.calculateIntervalDerivatives(self.BOX, 1).hessian
        assert constraint.calculateIntervalDerivatives(self.BOX, 1).gradient
        assert constraint.calculateIntervalDerivatives(self.BOX, order=2).hessian

        for order in (-1, 3):
            with pytest.raises(ValueError):
                constraint.calculateIntervalDerivatives(self.BOX, order)

    def test_linear_constraint(self, problem):
        constraints, _ = _build_problem(problem)
        derivatives = constraints["lin"].calculateIntervalDerivatives(self.BOX)

        assert derivatives.value == (1.0, 8.0)
        assert derivatives.gradient == {0: (1.0, 1.0), 1: (2.0, 2.0)}
        assert derivatives.hessian == {}


class TestEnclosure:
    def test_bounds_contain_the_derivatives(self, problem):
        """The derivatives at grid and random points in the box are in the bounds."""
        constraints, _ = _build_problem(problem)
        box = [(-1.0, 2.0), (1.0, 3.0), (0.5, 2.0)]

        generator = random.Random(20261014)
        points = [list(p) for p in itertools.product(*[(lo, (lo + hi) / 2, hi) for lo, hi in box])]
        points += [[generator.uniform(lo, hi) for lo, hi in box] for _ in range(20)]

        for function in (constraints["nl"], problem.objectiveFunction):
            derivatives = function.calculateIntervalDerivatives(box)

            for point in points:
                value = (function.calculateFunctionValue(point) if hasattr(function, "calculateFunctionValue")
                         else function.calculateValue(point))
                _assert_contains(derivatives.value, value)

                for index, g in function.calculateGradient(point).items():
                    _assert_contains(derivatives.gradient[index], g)

                for key, h in function.calculateHessian(point).items():
                    _assert_contains(derivatives.hessian[key], h)

    def test_expression(self, problem):
        """sin(xy) at (0.5, 2), where the box is a point."""
        import SHOTpy

        _, (x, y, _) = _build_problem(problem)
        derivatives = SHOTpy.sin(x * y).calculateIntervalDerivatives([0.5, 2.0, 1.0])

        expected_gradient = {0: 2.0 * math.cos(1.0), 1: 0.5 * math.cos(1.0)}
        expected_hessian = {
            (0, 0): -4.0 * math.sin(1.0),
            (0, 1): math.cos(1.0) - math.sin(1.0),
            (1, 1): -0.25 * math.sin(1.0),
        }

        assert set(derivatives.gradient) == set(expected_gradient)
        assert set(derivatives.hessian) == set(expected_hessian)

        for index, value in expected_gradient.items():
            assert tuple(derivatives.gradient[index]) == pytest.approx((value, value))

        for key, value in expected_hessian.items():
            assert tuple(derivatives.hessian[key]) == pytest.approx((value, value))


class TestProducts:
    def test_product_of_several_variables(self, problem):
        """The derivatives of x0 x1 x2 x3 x4, built as nested products, at a point are the products of the others."""
        import SHOTpy

        variables = [problem.addVariable(f"x{i}", SHOTpy.VariableType.Real, -5.0, 5.0) for i in range(5)]
        point = [1.5, -2.0, 0.5, 3.0, -1.25]

        expression = variables[0]
        for variable in variables[1:]:
            expression = expression * variable

        derivatives = expression.calculateIntervalDerivatives(point)

        def product_except(*indexes):
            return math.prod(value for i, value in enumerate(point) if i not in indexes)

        assert tuple(derivatives.value) == pytest.approx((math.prod(point),) * 2)

        for i in range(5):
            assert tuple(derivatives.gradient[i]) == pytest.approx((product_except(i),) * 2)

            for j in range(i + 1, 5):
                assert tuple(derivatives.hessian[(i, j)]) == pytest.approx((product_except(i, j),) * 2)

        assert all(i != j for i, j in derivatives.hessian)


    def test_shared_subexpressions(self, problem):
        """e = xy is used twice in (e e) z and three times in e + e + sin(e), while Python also holds e."""
        import SHOTpy

        x = problem.addVariable("x", SHOTpy.VariableType.Real, -5.0, 5.0)
        y = problem.addVariable("y", SHOTpy.VariableType.Real, -5.0, 5.0)
        z = problem.addVariable("z", SHOTpy.VariableType.Real, -5.0, 5.0)
        e = x * y
        point = [1.5, -2.0, 0.5]
        xv, yv, zv = point

        # (xy)^2 z
        derivatives = (e * e * z).calculateIntervalDerivatives(point)
        expected_gradient = {0: 2 * xv * yv**2 * zv, 1: 2 * xv**2 * yv * zv, 2: xv**2 * yv**2}
        expected_hessian = {
            (0, 0): 2 * yv**2 * zv,
            (0, 1): 4 * xv * yv * zv,
            (0, 2): 2 * xv * yv**2,
            (1, 1): 2 * xv**2 * zv,
            (1, 2): 2 * xv**2 * yv,
        }

        for index, value in expected_gradient.items():
            assert tuple(derivatives.gradient[index]) == pytest.approx((value, value))

        for key, value in expected_hessian.items():
            assert tuple(derivatives.hessian[key]) == pytest.approx((value, value))

        # 2xy + sin(xy)
        derivatives = (e + e + SHOTpy.sin(e)).calculateIntervalDerivatives(point)
        c = math.cos(xv * yv)
        assert tuple(derivatives.gradient[0]) == pytest.approx(((2 + c) * yv,) * 2)
        assert tuple(derivatives.gradient[1]) == pytest.approx(((2 + c) * xv,) * 2)
        assert tuple(derivatives.hessian[(0, 1)]) == pytest.approx(((2 + c) - math.sin(xv * yv) * xv * yv,) * 2)


class TestStatus:
    def test_not_differentiable(self, problem):
        import SHOTpy

        _, (x, y, _) = _build_problem(problem)
        derivatives = (SHOTpy.abs(x) + y**2).calculateIntervalDerivatives([(-1.0, 1.0), (0.0, 1.0), (1.0, 1.0)])

        assert derivatives.isDefinedOnBox
        assert derivatives.differentiableOrder == 0
        assert not derivatives.isBounded
        assert derivatives.gradient[0] == (-1.0, 1.0)

    def test_not_defined(self, problem):
        import SHOTpy

        _, (x, _, _) = _build_problem(problem)
        derivatives = SHOTpy.log(x).calculateIntervalDerivatives([(-1.0, 2.0), (1.0, 1.0), (1.0, 1.0)])

        assert not derivatives.isDefinedOnBox
        assert derivatives.differentiableOrder == 0
        assert derivatives.gradient[0] == (SHOTpy.SHOT_DBL_MIN, SHOTpy.SHOT_DBL_MAX)

    def test_unbounded_variable(self, problem):
        """The derivatives over the variable bounds of the problem, where x is not bounded."""
        import SHOTpy

        x = problem.addVariable("x", SHOTpy.VariableType.Real, -math.inf, math.inf)
        problem.addConstraint(SHOTpy.exp(x) <= 10, "c")
        problem.setObjective(x, SHOTpy.ObjectiveDirection.Minimize)
        problem.finalize()

        derivatives = problem.numericConstraints[0].calculateIntervalDerivatives(problem.getVariableBounds())

        assert derivatives.isDefinedOnBox and not derivatives.isBounded


class TestInvalidInput:
    def test_box_of_the_wrong_size_raises(self, problem):
        constraints, (x, y, _) = _build_problem(problem)
        short = [(0.0, 1.0)]

        for evaluation in (
            lambda: constraints["nl"].calculateIntervalDerivatives(short),
            lambda: problem.objectiveFunction.calculateIntervalDerivatives(short),
            lambda: (x * y).calculateIntervalDerivatives(short),
        ):
            with pytest.raises(ValueError):
                evaluation()

    def test_function_not_in_a_problem_raises(self):
        import SHOTpy

        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 1.0)
        constraint = SHOTpy.LinearConstraint("c", 0.0, 1.0)
        constraint.add(SHOTpy.LinearTerm(1.0, x))

        with pytest.raises(ValueError):
            constraint.calculateIntervalDerivatives([(0.0, 1.0)])

        with pytest.raises(ValueError):
            (x * x).calculateIntervalDerivatives([(0.0, 1.0)])


class TestConvexityCheck:
    def test_examples_of_the_tutorial(self, problem):
        import SHOTpy

        x = problem.addVariable("x", SHOTpy.VariableType.Real, -10.0, 10.0)
        y = problem.addVariable("y", SHOTpy.VariableType.Real, -10.0, 10.0)

        # Convex on [1, 2]^2, although x*y is not convex
        function = SHOTpy.exp(x) + SHOTpy.exp(y) + x * y
        assert check_convexity(function.calculateIntervalDerivatives([(1, 2), (1, 2)]), 2) is True
        assert check_convexity(function.calculateIntervalDerivatives([(-5, -4), (-5, -4)]), 2) is False

        # Convex where x >= 0
        function = x**3 + y**2
        assert check_convexity(function.calculateIntervalDerivatives([(0, 2), (-1, 1)]), 2) is True
        assert check_convexity(function.calculateIntervalDerivatives([(-1, 2), (-1, 1)]), 2) is False

        # Convex, but not twice differentiable at x = 0
        function = SHOTpy.abs(x) + y**2
        assert check_convexity(function.calculateIntervalDerivatives([(-1, 1), (-1, 1)]), 2) is None
        assert check_convexity(function.calculateIntervalDerivatives([(0.5, 1), (-1, 1)]), 2) is True

        # Not convex: the Hessian of x*y has the eigenvalues -1 and 1
        assert check_convexity((x * y).calculateIntervalDerivatives([(-1, 1), (-1, 1)]), 2) is False


class TestCallback:
    def test_derivatives_in_a_callback(self, solver, problem):
        import SHOTpy

        x = problem.addVariable("x", SHOTpy.VariableType.Real, 0.0, 4.0)
        y = problem.addVariable("y", SHOTpy.VariableType.Integer, 0.0, 4.0)
        problem.addConstraint(x**2 + y**2 <= 9, "circle")
        problem.setObjective(-x - y, SHOTpy.ObjectiveDirection.Minimize)
        problem.finalize()

        results = []

        def callback(context):
            circle = next(c for c in context.originalProblem.numericConstraints if c.name == "circle")
            results.append(circle.calculateIntervalDerivatives([(0.0, 1.0), (1.0, 2.0)]))

        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.registerCallback(SHOTpy.CallbackLocation.NewPrimalSolution, callback)
        assert solver.setProblem(problem)
        solver.solveProblem()

        assert results
        for derivatives in results:
            assert derivatives.gradient == {0: (0.0, 2.0), 1: (2.0, 4.0)}
            assert derivatives.hessian == {(0, 0): (2.0, 2.0), (1, 1): (2.0, 2.0)}
