"""
Tests for Solver.setStartingPoint and Solver.getStartingPoint.

A starting point is checked when it is set and is the first primal solution when the problem is solved, if it is
feasible. setProblem() replaces it with the starting point in the problem file, e.g., the initial values in an AMPL
.nl file, or removes it.
"""

import math

import pytest

from test_callbacks import build_ex1223b


def solve_and_get_first_primal_solution(solver):
    """Solve and return (point, objectiveValue, source) of the first primal solution found."""
    import SHOTpy

    solutions = []
    solver.registerCallback(
        SHOTpy.CallbackLocation.NewPrimalSolution,
        lambda ctx: solutions.append((ctx.point, ctx.objectiveValue, ctx.source)),
    )
    solver.solveProblem()
    return solutions[0] if solutions else None


class TestStartingPoint:
    def test_rejected_before_set_problem(self, solver):
        solver.updateSetting("Output.Console.LogLevel", 6)
        assert solver.setStartingPoint([0.0] * 7) is False

    def test_rejected_with_wrong_size(self, solver, env):
        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.setProblem(build_ex1223b(env))
        assert solver.setStartingPoint([0.0] * 6) is False
        assert solver.getStartingPoint() == []

    def test_rejected_with_nan(self, solver, env):
        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.setProblem(build_ex1223b(env))
        assert solver.setStartingPoint([0.0, 0.0, math.nan, 0.0, 0.0, 0.0, 0.0]) is False

    def test_empty_point_removes_it(self, solver, env):
        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.setProblem(build_ex1223b(env))
        assert solver.setStartingPoint([0.0] * 7)
        assert solver.setStartingPoint([])
        assert solver.getStartingPoint() == []

    def test_is_first_primal_solution(self, solver, env):
        """The zero point is feasible with objective 20, worse than the optimum 4.58, so it is only the first primal
        solution if it is given to the solver."""
        import SHOTpy

        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.setProblem(build_ex1223b(env))
        assert solver.setStartingPoint([0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        assert solver.setStartingPoint([0.0] * 7)
        assert solver.getStartingPoint() == [0.0] * 7

        first = solve_and_get_first_primal_solution(solver)

        assert first is not None
        point, objective_value, source = first
        assert source == SHOTpy.PrimalSolutionSource.ExternalPrimalSolution
        assert point == [0.0] * 7
        assert objective_value == pytest.approx(20.0)
        assert solver.getPrimalBound() == pytest.approx(4.5796, abs=1e-3)

    def test_ampl_initial_values(self, solver, data_dir):
        """ncvx_min_div.nl gives variable 2 the initial value 1.0; variables 0 and 1 get their lower bounds."""
        import SHOTpy

        solver.updateSetting("Output.Console.LogLevel", 6)
        if not solver.setProblem(str(data_dir / "ncvx_min_div.nl")):
            pytest.skip("SHOTpy is built without AMPL support")

        assert solver.getStartingPoint() == pytest.approx([0.5, 0.1, 1.0])

        first = solve_and_get_first_primal_solution(solver)

        assert first is not None
        point, _, source = first
        assert source == SHOTpy.PrimalSolutionSource.ExternalPrimalSolution
        assert point == pytest.approx([0.5, 0.1, 1.0])

    def test_set_problem_replaces_it(self, solver, data_dir):
        solver.updateSetting("Output.Console.LogLevel", 6)
        if not solver.setProblem(str(data_dir / "ncvx_min_div.nl")):
            pytest.skip("SHOTpy is built without AMPL support")

        assert solver.setStartingPoint([0.6, 0.2, 0.5])
        assert solver.setProblem(str(data_dir / "ncvx_min_div.nl"))
        assert solver.getStartingPoint() == pytest.approx([0.5, 0.1, 1.0])
