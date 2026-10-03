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
