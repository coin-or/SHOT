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

class TestBinaryProducts:
    """Products of a binary variable and a bounded variable are linearized exactly."""

    # Bounds with L < -U, L > 0 and U < 0, so that both bounds of each factor matter
    INTEGER_BOUNDS = {"i1": (-2, 3), "i2": (-3, -1), "i3": (1, 4)}
    CONTINUOUS_BOUNDS = {"x1": (-10.0, 1.0), "x2": (2.0, 5.0), "x3": (-4.0, -1.0)}

    SETTINGS = {"Model.Reformulation.Quadratics.Strategy": 0,
                "Model.Reformulation.Bilinear.IntegerFormulation": 2}

    @staticmethod
    def objective(b1, b2, i1, i2, i3, x1, x2, x3):
        return (x1 - 3 * b1 * x1 + 2 * b2 * x1 - b1 * x2 + 1.5 * b2 * x3 + x3
                + i1 * b1 - 2 * b2 * i1 + b1 * i2 + 0.5 * i1 * i3)

    @staticmethod
    def constraints(b1, b2, i1, i2, i3):
        return {"c1": b1 * i1 + b1 * i2 + b2 * i1 >= -3,
                "c2": i1 * b2 - b2 * i3 + i1 * i3 <= 4}

    def make_problem(self, solver):
        import SHOTpy

        problem = SHOTpy.Problem(solver)
        b1 = problem.addVariable("b1", SHOTpy.VariableType.Binary)
        b2 = problem.addVariable("b2", SHOTpy.VariableType.Binary)
        i1, i2, i3 = (problem.addVariable(name, SHOTpy.VariableType.Integer, *bounds)
                      for name, bounds in self.INTEGER_BOUNDS.items())
        x1, x2, x3 = (problem.addVariable(name, SHOTpy.VariableType.Real, *bounds)
                      for name, bounds in self.CONTINUOUS_BOUNDS.items())

        # The same products appear in both operand orders, e.g., i1*b2 and b2*i1, and i1 is in several products
        problem.setObjective(self.objective(b1, b2, i1, i2, i3, x1, x2, x3))
        for name, constraint in self.constraints(b1, b2, i1, i2, i3).items():
            problem.addConstraint(constraint, name)

        problem.finalize()
        return problem

    def optimal_value(self):
        """The continuous variables are only in the objective, which is linear in them when the others are fixed."""
        integerRanges = [range(lower, upper + 1) for lower, upper in self.INTEGER_BOUNDS.values()]
        best = float("inf")

        for b1, b2 in itertools.product((0, 1), repeat=2):
            for i1, i2, i3 in itertools.product(*integerRanges):
                if not all(self.constraints(b1, b2, i1, i2, i3).values()):
                    continue
                for x in itertools.product(*self.CONTINUOUS_BOUNDS.values()):
                    best = min(best, self.objective(b1, b2, i1, i2, i3, *x))

        return best

    def test_optimal_value(self):
        """With L < -U, a constraint of the linearization of b*x cut off x < -U when b = 0."""
        solver = make_solver(self.SETTINGS)
        problem = self.make_problem(solver)

        assert solver.setProblem(problem)
        assert solver.solveProblem()

        expected = self.optimal_value()
        assert solver.getPrimalBound() == pytest.approx(expected, abs=1e-6)
        assert solver.getCurrentDualBound() >= expected - 1e-6

        point = list(solver.getPrimalSolution().point)
        for name in ("c1", "c2"):
            assert problem.getConstraint(name).calculateNumericValue(point).error <= 1e-6

    def test_auxiliary_variables(self):
        """A binary times an integer needs no binaries encoding the integer, or the binary itself."""
        solver = make_solver(self.SETTINGS)
        problem = self.make_problem(solver)
        assert solver.setProblem(problem)

        names = auxiliary_variable_names(solver)

        # b1*x1, b2*x1, b1*x2, b2*x3, b1*i1, b2*i1, b1*i2, b2*i3 and i1*i3, with each product once
        assert sum(1 for name in names if name.startswith("s_bl_")) == 9, names

        # Only i1*i3 is discretized, using the variable with the smaller domain, i3 in [1, 4]
        assert sum(1 for name in names if name.startswith("s_bli")) == 4, names

class TestFixedVariablesInQuadraticTerms:
    """Quadratic terms with a fixed variable are linear or constant, and get no auxiliary variables."""

    # The quadratic terms are handled as nonlinear, and either partitioned with the bilinear terms extracted, or the
    # convex ones decomposed
    VARIANTS = {
        "Partitioned": {"Model.Reformulation.Quadratics.Strategy": 0,
                        "Model.Reformulation.Constraint.PartitionQuadraticTerms": 0,
                        "Model.Reformulation.ObjectiveFunction.PartitionQuadraticTerms": 0,
                        "Model.Reformulation.Quadratics.ExtractStrategy": 2},
        "EigenValueDecomposition": {"Model.Reformulation.Quadratics.Strategy": 0,
                                    "Model.Reformulation.Quadratics.Decomposition.Method": 1},
        "LDLDecomposition": {"Model.Reformulation.Quadratics.Strategy": 0,
                             "Model.Reformulation.Quadratics.Decomposition.Method": 2},
    }

    def make_problem(self, solver, fixed, maximize):
        """The variable x is fixed to 2 by a constraint, or replaced by the constant 2.

        A variable with equal bounds is replaced by its value already when the model is created, so here x is only
        fixed by the bound tightening done before the reformulation.
        """
        import SHOTpy

        problem = SHOTpy.Problem(solver)
        x = problem.addVariable("x", SHOTpy.VariableType.Real, 0.0, 5.0) if fixed else 2.0
        y = problem.addVariable("y", SHOTpy.VariableType.Real, -1.0, 3.0)
        z = problem.addVariable("z", SHOTpy.VariableType.Real, 0.0, 2.0)
        b = problem.addVariable("b", SHOTpy.VariableType.Binary)

        direction = SHOTpy.ObjectiveDirection.Maximize if maximize else SHOTpy.ObjectiveDirection.Minimize
        sign = -1.0 if maximize else 1.0
        problem.setObjective(sign * (0.5 * x * x + x * y - 2 * x * z + y * z + y * y) + b, direction)

        # The terms with x share y and z with the terms without x, and the last constraint is convex, so that it is
        # decomposed when a decomposition is used
        problem.addConstraint(x * x + x * y + y * z + b <= 6, "c1")
        problem.addConstraint(x * z - y * y + y * z >= -3 + b, "c2")
        problem.addConstraint(x * x + y * y + z * z + x * y + y * z <= 10, "c3")
        if fixed:
            problem.addConstraint(x == 2, "fix")
        problem.finalize()
        return problem

    def solve(self, fixed, maximize, settings):
        solver = make_solver(settings)
        problem = self.make_problem(solver, fixed, maximize)
        assert solver.setProblem(problem)
        names = auxiliary_variable_names(solver)
        assert solver.solveProblem()
        return solver, problem, names

    @pytest.mark.parametrize("settings", VARIANTS.values(), ids=VARIANTS.keys())
    @pytest.mark.parametrize("maximize", [False, True], ids=["min", "max"])
    def test_same_as_constant(self, maximize, settings):
        """A fixed x gave auxiliary variables for x^2 and x*y, which the same model with the constant 2 has not."""
        solver, problem, names = self.solve(True, maximize, settings)
        constantSolver, _, constantNames = self.solve(False, maximize, settings)

        # The terms without x still get their auxiliary variables
        assert len(constantNames) > 0

        assert not any(name == "s_sq_x" or name.startswith("s_bl_x_") or name.endswith("_x") for name in names), names
        assert len(names) == len(constantNames), (names, constantNames)
        assert sorted(name for name in names if not name[-1].isdigit()) \
            == sorted(name for name in constantNames if not name[-1].isdigit())

        assert solver.getPrimalBound() == pytest.approx(constantSolver.getPrimalBound(), abs=1e-5)

        point = list(solver.getPrimalSolution().point)
        assert point[0] == 2.0
        for name in ("c1", "c2", "c3"):
            assert problem.getConstraint(name).calculateNumericValue(point).error <= 1e-6
        assert problem.objectiveFunction.calculateValue(point) == pytest.approx(solver.getPrimalBound(), abs=1e-6)

class TestAbsoluteValues:
    """An absolute value |f(x)| gets an auxiliary variable w with f(x) <= w and -f(x) <= w."""

    def make_problem(self, solver, shift):
        import SHOTpy

        problem = SHOTpy.Problem(solver)
        x = problem.addVariable("x", SHOTpy.VariableType.Real, -3.0, 3.0)
        y = problem.addVariable("y", SHOTpy.VariableType.Real, -3.0, 3.0)
        z = problem.addVariable("z", SHOTpy.VariableType.Real, -6.0, 3.0)

        # |x - 1| is in two constraints, and |y - 1 - shift| differs from |y - 1| only by the small shift
        problem.setObjective(x + y + z)
        problem.addConstraint(SHOTpy.abs(x - 1) + SHOTpy.abs(y - 1 - shift) <= 2, "c1")
        problem.addConstraint(SHOTpy.abs(x - 1) - y <= 1, "c2")
        problem.addConstraint(SHOTpy.abs(z + 2) + SHOTpy.abs(y - 1) <= 3, "c3")
        problem.finalize()
        return problem

    def test_constants(self):
        """The constant inside the absolute value was lost if negative, and given the wrong sign if positive."""
        solver = make_solver({})
        problem = self.make_problem(solver, 0.0)

        assert solver.setProblem(problem)
        assert solver.solveProblem()

        # c1 gives x + y >= 0 and c3 gives z >= -5 + |y - 1|, so the optimum is at x = -1, y = 1, z = -5, where c2
        # holds with equality
        assert solver.getPrimalBound() == pytest.approx(-5.0, abs=1e-5)
        assert solver.getCurrentDualBound() >= -5.0 - 1e-5

        point = list(solver.getPrimalSolution().point)
        for name in ("c1", "c2", "c3"):
            assert problem.getConstraint(name).calculateNumericValue(point).error <= 1e-6

    def test_auxiliary_variables(self):
        """Equal absolute values share their auxiliary variable, but constants differing in the 7th digit do not."""
        for shift, expected in ((0.0, 3), (1e-7, 4)):
            solver = make_solver({})
            problem = self.make_problem(solver, shift)
            assert solver.setProblem(problem)

            names = auxiliary_variable_names(solver)
            assert sum(1 for name in names if name.startswith("s_abs_")) == expected, (shift, names)

