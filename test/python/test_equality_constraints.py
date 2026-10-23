"""
Tests that constraints with both a lower and an upper bound, L <= f(x) <= U, e.g., equality constraints, are kept as
they are in the original problem, whichever way the problem is created.

The reformulated problem, which the dual strategy works with, has the nonlinear ones as the two constraints f(x) <= U
(with the name of the constraint) and -f(x) <= -L (with the name followed by _rf).
"""

import pytest

SHOT_DBL_MAX = 1e300


def make_solver(settings=None):
    import SHOTpy

    solver = SHOTpy.Solver()
    solver.updateSetting("Output.Console.LogLevel", 6)
    solver.updateSetting("Dual.MIP.NumberOfThreads", 1)
    for name, value in (settings or {}).items():
        solver.updateSetting(name, value)
    return solver


def has_lower_bound(constraint):
    return constraint.valueLHS > -SHOT_DBL_MAX


def has_upper_bound(constraint):
    return constraint.valueRHS < SHOT_DBL_MAX


def is_nonlinear_with_both_bounds(constraint):
    import SHOTpy

    return (
        type(constraint) is not SHOTpy.LinearConstraint and has_lower_bound(constraint) and has_upper_bound(constraint)
    )


def check_problems(solver, expected_rows, expected_with_both_bounds):
    """The original problem has the rows it was given, and the reformulated one only nonlinear rows f(x) <= U."""
    import SHOTpy

    original = solver.getOriginalProblem()
    reformulated = solver.getReformulatedProblem()

    constraints = list(original.numericConstraints)
    names = [c.name for c in constraints]

    assert len(constraints) == expected_rows, names
    assert not any(name.endswith("_rf") for name in names), names

    with_both_bounds = [c.name for c in constraints if is_nonlinear_with_both_bounds(c)]
    assert len(with_both_bounds) == expected_with_both_bounds, with_both_bounds

    reformulated_names = [c.name for c in reformulated.numericConstraints]

    for constraint in reformulated.numericConstraints:
        if type(constraint) is SHOTpy.NonlinearConstraint:
            assert not has_lower_bound(constraint), constraint.name

    # Each of the constraints has its lower side in the reformulated problem, unless the constraint has become linear
    for name in with_both_bounds:
        assert name in reformulated_names, (name, reformulated_names)

    return with_both_bounds


def max_error(problem, point):
    number_of_variables = problem.properties.numberOfVariables
    return max(c.calculateNumericValue(list(point)[:number_of_variables]).error for c in problem.numericConstraints)


def build_problem(solver):
    """
    Two equality constraints and a range sharing the terms exp(x) and exp(y), and constraints with one bound.

    minimize    x + 2 y + z + 3 b
    subject to  exp(x) + exp(y) = 4 + b
                exp(x) + log(z) = 2
                1 <= exp(y) + z^2 <= 6
                x * z >= 0.2
                x + y + z <= 4
    """
    import SHOTpy

    problem = SHOTpy.Problem(solver.getEnvironment())

    x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, -2.0, 2.0)
    y = SHOTpy.Variable("y", SHOTpy.VariableType.Real, -2.0, 2.0)
    z = SHOTpy.Variable("z", SHOTpy.VariableType.Real, 0.5, 3.0)
    b = SHOTpy.Variable("b", SHOTpy.VariableType.Binary, 0.0, 1.0)

    for variable in (x, y, z, b):
        problem.addVariable(variable)

    problem.setObjective(x + 2 * y + z + 3 * b)
    problem.addConstraint(SHOTpy.exp(x) + SHOTpy.exp(y) - b == 4, "exp_eq")
    problem.addConstraint(SHOTpy.exp(x) + SHOTpy.log(z) == 2, "log_eq")
    problem.addConstraint(SHOTpy.inequality(1, SHOTpy.exp(y) + z * z, 6), "range")
    problem.addConstraint(x * z >= 0.2, "lower")
    problem.addConstraint(x + y + z <= 4, "upper")
    problem.finalize()

    return problem


class TestOriginalProblem:
    def test_problem_created_in_python(self):
        import SHOTpy

        solver = make_solver()
        problem = build_problem(solver)

        def check(problem):
            assert [c.name for c in problem.numericConstraints] == ["exp_eq", "log_eq", "range", "lower", "upper"]

            exp_eq = problem.getConstraint("exp_eq")
            assert (exp_eq.valueLHS, exp_eq.valueRHS) == (4.0, 4.0)

            log_eq = problem.getConstraint("log_eq")
            assert (log_eq.valueLHS, log_eq.valueRHS) == (2.0, 2.0)

            constraint_range = problem.getConstraint("range")
            assert (constraint_range.valueLHS, constraint_range.valueRHS) == (1.0, 6.0)

            # The functions of the equality constraints are convex, but the constraints are not
            assert exp_eq.properties.functionConvexity == SHOTpy.Convexity.Convex
            assert exp_eq.properties.convexity == SHOTpy.Convexity.Nonconvex

            # A constraint with only a lower bound is still rewritten as -f(x) <= -L
            lower = problem.getConstraint("lower")
            assert not has_lower_bound(lower)
            assert lower.valueRHS == -0.2
            assert lower.calculateFunctionValue([1.0, 0.0, 2.0, 0.0]) == pytest.approx(-2.0)

            # exp(0) + exp(0) - b is 2 for b = 0, which fulfills exp(x) + exp(y) - b <= 4 but not the equality
            assert exp_eq.calculateNumericValue([0.0, 0.0, 1.0, 0.0]).error == pytest.approx(2.0)
            assert not exp_eq.isFulfilled([0.0, 0.0, 1.0, 0.0])

            # Below, in and above the range
            assert not constraint_range.isFulfilled([0.0, -2.0, 0.5, 0.0])
            assert constraint_range.isFulfilled([0.0, 0.0, 1.0, 0.0])
            assert not constraint_range.isFulfilled([0.0, 2.0, 1.0, 0.0])

        check(problem)

        assert solver.setProblem(problem)

        check(solver.getOriginalProblem())
        with_both_bounds = check_problems(solver, 5, 3)
        assert with_both_bounds == ["exp_eq", "log_eq", "range"]

        reformulated_names = [c.name for c in solver.getReformulatedProblem().numericConstraints]

        for name in ("exp_eq_rf", "log_eq_rf", "range_rf"):
            assert name in reformulated_names, reformulated_names

        for name in ("lower_rf", "upper_rf"):
            assert name not in reformulated_names, reformulated_names

    def test_osil(self, data_dir):
        path = data_dir / "instances" / "MINLP-nonconvex" / "ex1221.osil"

        if not path.exists():
            pytest.skip(f"{path} not found")

        solver = make_solver()
        assert solver.setProblem(str(path))

        # The file has 5 constraints, of which two are nonlinear equality constraints
        check_problems(solver, 5, 2)

    def test_ampl(self, data_dir):
        import SHOTpy

        if not SHOTpy.HAS_AMPL:
            pytest.skip("AMPL support not available in this build")

        path = data_dir / "instances" / "minlp_tests_jl" / "nlp_002_010.jl.nl"

        if not path.exists():
            pytest.skip(f"{path} not found")

        solver = make_solver()
        assert solver.setProblem(str(path))

        # The file has two nonlinear equality constraints
        check_problems(solver, 2, 2)

    def test_gams(self, tmp_path):
        import SHOTpy

        if not SHOTpy.HAS_GAMS:
            pytest.skip("GAMS support not available in this build")

        path = tmp_path / "equalities.gms"
        path.write_text(
            """
Variables x, y, z, objvar;
Binary Variable b;
Equations eobj, e1, e2, e3, e4;

eobj.. objvar =E= x + 2*y + z + 3*b;
e1..   exp(x) + exp(y) - b =E= 4;
e2..   exp(x) + log(z) =E= 2;
e3..   exp(y) + sqr(z) =L= 6;
e4..   x*z =G= 0.2;

x.lo = -2; x.up = 2;
y.lo = -2; y.up = 2;
z.lo = 0.5; z.up = 3;

Model m / all /;
Solve m using MINLP minimizing objvar;
"""
        )

        solver = make_solver()
        assert solver.setProblem(str(path))

        original = solver.getOriginalProblem()
        both_bounds = [c for c in original.numericConstraints if is_nonlinear_with_both_bounds(c)]

        # e1 and e2 are the nonlinear equality constraints; the objective equation is linear or the objective itself
        assert len(both_bounds) == 2, [c.name for c in original.numericConstraints]
        assert sorted(c.valueRHS for c in both_bounds) == [2.0, 4.0]
        assert all(c.valueLHS == c.valueRHS for c in both_bounds)

        check_problems(solver, len(list(original.numericConstraints)), 2)


PARTITIONING_STRATEGIES = {"Always": 0, "IfConvex": 1, "Never": 2}
NLP_SOURCES = {"OriginalProblem": 0, "ReformulatedProblem": 1, "Both": 2}


class TestSolve:
    @pytest.mark.parametrize("partitioning", PARTITIONING_STRATEGIES)
    @pytest.mark.parametrize("source", NLP_SOURCES)
    def test_solutions_fulfill_equality_constraints(self, partitioning, source):
        solver = make_solver(
            {
                "Model.Reformulation.Constraint.PartitionNonlinearTerms": PARTITIONING_STRATEGIES[partitioning],
                "Model.Reformulation.Constraint.PartitionQuadraticTerms": PARTITIONING_STRATEGIES[partitioning],
                "Primal.FixedInteger.SourceProblem": NLP_SOURCES[source],
                "Termination.TimeLimit": 30.0,
            }
        )

        assert solver.setProblem(build_problem(solver))
        check_problems(solver, 5, 3)

        solver.solveProblem()

        # Solving does not change the constraints of the original problem either
        check_problems(solver, 5, 3)

        assert solver.hasPrimalSolution()

        original = solver.getOriginalProblem()

        for solution in solver.getPrimalSolutions():
            assert max_error(original, solution.point) < 1e-5

        point = solver.getPrimalSolution().point

        # The solution fulfills the equality constraints, and not only exp(x) + exp(y) - b <= 4
        assert original.getConstraint("exp_eq").calculateFunctionValue(list(point)[:4]) == pytest.approx(4.0, abs=1e-5)
        assert original.getConstraint("log_eq").calculateFunctionValue(list(point)[:4]) == pytest.approx(2.0, abs=1e-5)

        assert solver.getCurrentDualBound() <= solver.getPrimalBound() + 1e-6
