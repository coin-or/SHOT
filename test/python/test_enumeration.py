"""
Tests for the exhaustive search in the fixed-integer primal strategy, where an NLP problem is solved for each
combination of the values of the discrete variables before the dual problem is solved. The search is described in
docs/ExhaustiveFixedIntegerSearchPlan.md, and tested in more detail in test/EnumerationTest.cpp.
"""

import pytest


def solve_synthes1(solver, data_dir, settings=None, callback=None):
    """Solves synthes1, a convex problem with three binary variables, and returns the solution statistics."""
    import SHOTpy

    solver.updateSetting("Output.Console.LogLevel", 6)
    solver.updateSetting("Dual.MIP.NumberOfThreads", 1)

    for name, value in (settings or {}).items():
        solver.updateSetting(name, value)

    assert solver.setProblem(str(data_dir / "synthes1.osil"))

    if callback is not None:
        solver.registerCallback(SHOTpy.CallbackLocation.NewPrimalSolution, callback)

    solver.solveProblem()

    return solver.getSolutionStatistics()


def number_of_combinations_solved(statistics):
    return (statistics.numberOfFixedIntegerEnumerationCombinationsFeasible
            + statistics.numberOfFixedIntegerEnumerationCombinationsInfeasible
            + statistics.numberOfFixedIntegerEnumerationCombinationsUnresolved)


class TestExhaustiveFixedIntegerSearch:

    def test_not_used_by_default_for_convex_problem(self, solver, data_dir):
        statistics = solve_synthes1(solver, data_dir)

        assert not statistics.hasFixedIntegerEnumerationBeenRun
        assert statistics.numberOfFixedIntegerEnumerationCombinations == 0

    def test_all_combinations_are_solved(self, solver, data_dir):
        statistics = solve_synthes1(solver, data_dir, {"Primal.FixedInteger.Enumeration.UseInitially": True})

        assert statistics.hasFixedIntegerEnumerationBeenRun
        assert statistics.numberOfFixedIntegerEnumerationCombinations == 8
        assert number_of_combinations_solved(statistics) == 8
        assert statistics.numberOfFixedIntegerEnumerationCombinationsUnresolved == 0
        assert statistics.numberOfFixedIntegerEnumerationCombinationsSkipped == 0
        assert solver.getPrimalBound() == pytest.approx(6.00976, abs=1e-3)

    def test_callback_terminates_search(self, solver, data_dir):
        """A termination requested when a solution is found in the search ends it and the solution process."""
        import SHOTpy

        objective_values = []

        def terminate(ctx):
            objective_values.append(ctx.objectiveValue)
            ctx.terminate()

        statistics = solve_synthes1(solver, data_dir, {"Primal.FixedInteger.Enumeration.UseInitially": True}, terminate)

        assert len(objective_values) == 1
        assert statistics.hasFixedIntegerEnumerationBeenRun
        assert statistics.numberOfFixedIntegerEnumerationCombinationsFeasible == 1
        assert number_of_combinations_solved(statistics) < 8
        assert solver.getTerminationReason() == SHOTpy.TerminationReason.UserAbort

        # The dual problem has not been solved
        assert statistics.numberOfIterations <= 1
