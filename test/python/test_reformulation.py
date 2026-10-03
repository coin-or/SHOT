"""
Tests of the reformulations SHOT does before solving a problem.

Every model contains several terms of the reformulated kind, sharing variables and spread over more than one
constraint, since a single term hides errors in how the auxiliary variables are created and reused.
"""

import itertools

import pytest


def make_solver(settings):
    import SHOTpy

    solver = SHOTpy.Solver()
    solver.updateSetting("Output.Console.LogLevel", 6)
    solver.updateSetting("Dual.MIP.NumberOfThreads", 1)
    for name, value in settings.items():
        solver.updateSetting(name, value)
    return solver


def auxiliary_variable_names(solver):
    return [V.name for V in solver.getReformulatedProblem().allVariables if V.properties.isAuxiliary]


MONOMIAL_FORMULATIONS = {"Simple": 1, "CostaLiberti": 2}
PARTITIONING_STRATEGIES = {"Always": 0, "IfConvex": 1, "Never": 2}


class TestBinaryMonomials:
    """Binary monomials are linearized, while the continuous monomials in the same constraints remain."""

    # (coefficient, binary indexes) and (coefficient, continuous indexes) for the two constraints, and their bounds;
    # the continuous variables are in [0, 1], so their monomials are nonnegative
    UNIT_COEFFICIENTS = (
        ([(1.0, (0, 1, 2))], [(1.0, (0, 1, 2)), (1.0, (3, 4, 5))], 0.5),
        ([(1.0, (1, 2, 3))], [(1.0, (1, 2, 3)), (2.0, (0, 4, 5))], 0.5),
    )

    MIXED_COEFFICIENTS = (
        ([(2.0, (0, 1, 2))], [(1.0, (0, 1, 2)), (1.0, (3, 4, 5))], 1.5),
        ([(1.5, (1, 2, 3)), (-0.5, (0, 1, 3))], [(2.0, (0, 4, 5)), (1.0, (1, 2, 3))], 1.2),
    )

    def make_problem(self, solver, constraints):
        import SHOTpy

        problem = SHOTpy.Problem(solver)
        b = [problem.addVariable(f"b{i}", SHOTpy.VariableType.Binary) for i in range(1, 5)]
        x = [problem.addVariable(f"x{i}", SHOTpy.VariableType.Real, 0.0, 1.0) for i in range(1, 7)]

        problem.setObjective(-(b[0] + b[1] + b[2] + b[3]))

        for number, (binaryMonomials, continuousMonomials, bound) in enumerate(constraints, start=1):
            expression = 0.0
            for coefficient, indexes in binaryMonomials:
                expression = expression + coefficient * b[indexes[0]] * b[indexes[1]] * b[indexes[2]]
            for coefficient, indexes in continuousMonomials:
                expression = expression + coefficient * x[indexes[0]] * x[indexes[1]] * x[indexes[2]]
            problem.addConstraint(expression <= bound, f"c{number}")

        problem.finalize()
        return problem

    def optimal_value(self, constraints):
        """The continuous monomials are nonnegative, so they are zero in an optimal solution."""
        best = 0.0
        for binaries in itertools.product((0, 1), repeat=4):
            feasible = all(
                sum(c * binaries[i] * binaries[j] * binaries[k] for c, (i, j, k) in binaryMonomials) <= bound
                for binaryMonomials, _, bound in constraints
            )
            if feasible:
                best = min(best, -sum(binaries))
        return best

    @pytest.mark.parametrize("formulation", MONOMIAL_FORMULATIONS.values(), ids=MONOMIAL_FORMULATIONS.keys())
    @pytest.mark.parametrize("partitioning", PARTITIONING_STRATEGIES.values(), ids=PARTITIONING_STRATEGIES.keys())
    @pytest.mark.parametrize("constraints", [UNIT_COEFFICIENTS, MIXED_COEFFICIENTS], ids=["unit", "mixed"])
    def test_optimal_value(self, formulation, partitioning, constraints):
        """With partitioning always, the binary monomials of a constraint with several continuous ones were lost."""
        solver = make_solver({"Model.Reformulation.Monomials.Formulation": formulation,
                              "Model.Reformulation.Constraint.PartitionNonlinearTerms": partitioning})
        problem = self.make_problem(solver, constraints)

        assert solver.setProblem(problem)
        assert solver.solveProblem()

        expected = self.optimal_value(constraints)
        assert solver.getPrimalBound() == pytest.approx(expected, abs=1e-6)
        assert solver.getCurrentDualBound() >= expected - 1e-6

        point = list(solver.getPrimalSolution().point)
        for name in ("c1", "c2"):
            assert problem.getConstraint(name).calculateNumericValue(point).error <= 1e-6

    def test_all_binaries_one_is_infeasible(self):
        """b1*b2*b3 + x1*x2*x3 + x4*x5*x6 <= 0.5 cannot hold with all binaries one, whatever the continuous ones."""
        solver = make_solver({"Model.Reformulation.Constraint.PartitionNonlinearTerms": 0})
        problem = self.make_problem(solver, self.UNIT_COEFFICIENTS)
        point = [1.0] * 4 + [0.0] * 6

        assert problem.getConstraint("c1").calculateNumericValue(point).error > 0.0

        assert solver.setProblem(problem)
        assert solver.solveProblem()
        assert solver.getCurrentDualBound() > -4.0 + 1e-6

    @pytest.mark.parametrize("partitioning", PARTITIONING_STRATEGIES.values(), ids=PARTITIONING_STRATEGIES.keys())
    def test_auxiliary_variables(self, partitioning):
        """Each binary monomial gets one product variable, or one product variable and a weight for each vertex."""
        for formulation, expected in ((1, {"s_monb": 3, "s_monw": 0, "s_monlam": 0}),
                                      (2, {"s_monb": 0, "s_monw": 3, "s_monlam": 3 * 8})):
            solver = make_solver({"Model.Reformulation.Monomials.Formulation": formulation,
                                  "Model.Reformulation.Constraint.PartitionNonlinearTerms": partitioning})
            problem = self.make_problem(solver, self.MIXED_COEFFICIENTS)
            assert solver.setProblem(problem)

            names = auxiliary_variable_names(solver)
            for prefix, count in expected.items():
                assert sum(1 for name in names if name.startswith(prefix)) == count, (formulation, prefix, names)

