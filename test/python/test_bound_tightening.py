"""
Tests of the feasibility-based bound tightening SHOT does before solving a problem.
"""

import pytest


def make_solver():
    import SHOTpy

    solver = SHOTpy.Solver()
    solver.updateSetting("Output.Console.LogLevel", 6)
    solver.updateSetting("Dual.MIP.NumberOfThreads", 1)
    return solver


class TestDiscreteVariableBounds:
    """The bounds of discrete variables are rounded to integers after they have been tightened."""

    def make_problem(self, solver):
        """b1 and b2 can be zero, but the interval arithmetic through b^2/(c - x + d*b) gives them tiny positive lower
        bounds, as in the constraints of routingdelay_proj in MINLPLib."""
        import SHOTpy

        problem = SHOTpy.Problem(solver)
        x = problem.addVariable("x", SHOTpy.VariableType.Real, 0.0, 30.0)
        b1 = problem.addVariable("b1", SHOTpy.VariableType.Binary)
        b2 = problem.addVariable("b2", SHOTpy.VariableType.Binary)

        # b1 is in both constraints, and x in all terms
        problem.setObjective(x + b1 + b2)
        problem.addConstraint(SHOTpy.square(b1) / (33.7026 - x + 5.2974 * b1) - 0.122521 * b1 <= 0, "c1")
        problem.addConstraint(SHOTpy.square(b2) / (34.4395 - x + 4.5605 * b2)
                              + SHOTpy.square(b1) / (65.4828 - x + 4.7172 * b1) - 0.15 * b2 <= 0, "c2")
        problem.finalize()
        return problem

    def test_rounding_errors_do_not_fix_binaries(self):
        """A lower bound of about 1e-8 from rounding errors was rounded up to one, which removed the optimum."""
        solver = make_solver()
        problem = self.make_problem(solver)

        assert solver.setProblem(problem)
        assert problem.getVariableLowerBound(1) == 0.0
        assert problem.getVariableLowerBound(2) == 0.0

        assert solver.solveProblem()

        # All variables zero fulfill the constraints, since b^2 / (c - x + d*b) is zero for b = 0
        assert solver.getPrimalBound() == pytest.approx(0.0, abs=1e-6)
        point = list(solver.getPrimalSolution().point)
        for name in ("c1", "c2"):
            assert problem.getConstraint(name).calculateNumericValue(point).error <= 1e-6

    def test_integer_bounds_are_still_tightened(self):
        """A bound that differs from an integer by more than the tolerance is still rounded."""
        import SHOTpy

        solver = make_solver()
        problem = SHOTpy.Problem(solver)
        i1 = problem.addVariable("i1", SHOTpy.VariableType.Integer, -10.0, 10.0)
        i2 = problem.addVariable("i2", SHOTpy.VariableType.Integer, -10.0, 10.0)
        x = problem.addVariable("x", SHOTpy.VariableType.Real, 0.1, 5.0)

        # 2*i1 >= 3 gives i1 >= 1.5, i.e., i1 >= 2, and i1 + 3*i2 <= 4.5 then gives i2 <= 0.8333, i.e., i2 <= 0; the
        # nonlinear term is there since the bounds of a linear problem are not tightened
        problem.setObjective(i1 - i2 + SHOTpy.exp(x))
        problem.addConstraint(2 * i1 >= 3, "c1")
        problem.addConstraint(i1 + 3 * i2 <= 4.5, "c2")
        problem.finalize()

        assert solver.setProblem(problem)
        assert problem.getVariableLowerBound(0) == 2.0
        assert problem.getVariableUpperBound(1) == 0.0


class TestFixedVariablesInObjective:
    """A variable fixed by the bound tightening is removed from the objective function as a constant."""

    def make_problem(self, solver, quadratic, epigraph=False):
        """ex1222 in MINLPLib: x2 <= -1 - 1.1*b3 and x2 >= -exp(x1 - 0.2) fix b3 to one through x1 <= 1.2*b3."""
        import SHOTpy

        problem = SHOTpy.Problem(solver)
        x1 = problem.addVariable("x1", SHOTpy.VariableType.Real, 0.0, 10.0)
        x2 = problem.addVariable("x2", SHOTpy.VariableType.Real, -2.22554, -1.0)
        b3 = problem.addVariable("b3", SHOTpy.VariableType.Binary)
        b4 = problem.addVariable("b4", SHOTpy.VariableType.Binary)

        # b4 is fixed to one by c4, so that both a negative and a positive coefficient of a fixed variable are in
        # the objective function
        objective = (5 * x1 * x1 - 5 * x1 if quadratic else x1) - 0.7 * b3 + 0.4 * b4 + 2.05

        if epigraph:
            # The objective as an epigraph constraint, which the reformulation turns into an objective function
            t = problem.addVariable("t", SHOTpy.VariableType.Real, -100.0, 100.0)
            problem.setObjective(t)
            problem.addConstraint(objective - t <= 0, "epigraph")
        else:
            problem.setObjective(objective)

        problem.addConstraint(-x2 - SHOTpy.exp(x1 - 0.2) <= 0, "e1")
        problem.addConstraint(x2 + 1.1 * b3 <= -1, "e2")
        problem.addConstraint(x1 - 1.2 * b3 <= 0, "e3")
        problem.addConstraint(b4 >= 0.5, "c4")
        problem.finalize()
        return problem

    @pytest.mark.parametrize("epigraph", [False, True], ids=["objective", "epigraph"])
    @pytest.mark.parametrize("quadratic", [True, False], ids=["quadratic", "linear"])
    def test_optimal_value(self, quadratic, epigraph):
        """The constant of the fixed variables was lost, so that, e.g., ex1222 was solved to 1.35 instead of 1.0765,
        which was reported as globally optimal."""
        import math

        solver = make_solver()
        if epigraph:
            solver.updateSetting("Model.Reformulation.ObjectiveFunction.EpigraphStrategy", 1)
        problem = self.make_problem(solver, quadratic, epigraph)

        assert solver.setProblem(problem)
        assert solver.solveProblem()

        # b3 = b4 = 1 and x1 >= 0.2 + ln(2.1), where both objective functions are increasing in x1
        x1 = 0.2 + math.log(2.1)
        expected = (5 * x1 * x1 - 5 * x1 if quadratic else x1) - 0.7 + 0.4 + 2.05

        assert solver.getPrimalBound() == pytest.approx(expected, abs=1e-4)
        assert solver.getCurrentDualBound() <= expected + 1e-4
        assert solver.getCurrentDualBound() >= expected - 1e-2
