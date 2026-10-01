"""
Tests that invalid use of the Python API raises an exception instead of crashing the interpreter or giving wrong
values, and that changes to a model reach the data calculated from it.
"""

import math

import pytest


def make_problem():
    """A solver, a problem and the variables x and y in [0, 10]."""
    import SHOTpy

    solver = SHOTpy.Solver()
    solver.updateSetting("Output.Console.LogLevel", 6)
    problem = SHOTpy.Problem(solver)

    x = problem.addVariable("x", SHOTpy.VariableType.Real, 0.0, 10.0)
    y = problem.addVariable("y", SHOTpy.VariableType.Real, 0.0, 10.0)

    return solver, problem, x, y


class TestCollections:
    """Indexing and iteration of the containers of the model."""

    def test_index_outside_an_empty_container_raises(self):
        import SHOTpy

        for container in (SHOTpy.Variables(), SHOTpy.LinearTerms(), SHOTpy.QuadraticTerms(),
                          SHOTpy.SignomialTerms(), SHOTpy.MonomialTerms()):
            with pytest.raises(IndexError):
                container[0]
            with pytest.raises(IndexError):
                container[-1]
            assert list(container) == []

    def test_negative_index_and_iteration(self):
        import SHOTpy

        _, _, x, y = make_problem()
        terms = SHOTpy.LinearTerms([SHOTpy.LinearTerm(1.0, x), SHOTpy.LinearTerm(2.0, y)])

        assert terms[-1].coefficient == 2.0
        assert terms[-2].coefficient == 1.0
        assert [T.coefficient for T in terms] == [1.0, 2.0]
        with pytest.raises(IndexError):
            terms[2]
        with pytest.raises(IndexError):
            terms[-3]

    def test_iterate_variables_of_problem(self):
        _, problem, x, y = make_problem()

        assert [V.name for V in problem.allVariables] == ["x", "y"]
        assert problem.allVariables[-1] is y


def test_signomial_elements_are_a_list():
    import SHOTpy

    _, _, x, y = make_problem()
    term = SHOTpy.SignomialTerm(2.0, [SHOTpy.SignomialElement(x, 0.5), SHOTpy.SignomialElement(y, -1.0)])

    assert isinstance(term.elements, list)
    assert [E.power for E in term.elements] == [0.5, -1.0]


class TestNone:
    """None is not accepted where SHOT expects an object of the model."""

    def test_none_arguments_raise(self):
        import SHOTpy

        solver, problem, x, _ = make_problem()

        calls = [
            lambda: solver.setProblem(None),
            lambda: problem.addVariables([None]),
            lambda: problem.addConstraints([None]),
            lambda: SHOTpy.exp(None),
            lambda: SHOTpy.LinearTerm(1.0, None),
            lambda: SHOTpy.QuadraticTerm(1.0, x, None),
            lambda: SHOTpy.LinearTerms([None]),
            lambda: SHOTpy.Problem(None),
        ]

        for call in calls:
            with pytest.raises(TypeError):
                call()

    def test_arithmetic_with_none_raises(self):
        _, _, x, _ = make_problem()

        with pytest.raises(TypeError):
            x + None
        with pytest.raises(TypeError):
            (x * x) * None


class TestEvaluation:
    def test_function_not_in_a_problem_cannot_be_evaluated(self):
        import SHOTpy

        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 1.0)
        constraint = SHOTpy.LinearConstraint("c", 0.0, 1.0)
        constraint.add(SHOTpy.LinearTerm(1.0, x))

        with pytest.raises(ValueError):
            constraint.calculateFunctionValue([0.5])

        objective = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        objective.add(SHOTpy.LinearTerm(1.0, x))

        with pytest.raises(ValueError):
            objective.calculateValue([0.5])


class TestModelChanges:
    def test_bound_change_after_finalize_updates_the_bound_vectors(self):
        import SHOTpy

        _, problem, x, y = make_problem()
        problem.setObjective(x + y, SHOTpy.ObjectiveDirection.Minimize)
        problem.finalize()

        x.upperBound = 3.0
        y.lowerBound = 1.0

        assert list(problem.getVariableUpperBounds()) == [3.0, 10.0]
        assert list(problem.getVariableLowerBounds()) == [0.0, 1.0]

    def test_term_fields_are_read_only(self):
        import SHOTpy

        _, _, x, y = make_problem()
        linear = SHOTpy.LinearTerm(1.0, x)
        quadratic = SHOTpy.QuadraticTerm(1.0, x, y)

        with pytest.raises(AttributeError):
            linear.coefficient = 3.0
        with pytest.raises(AttributeError):
            linear.variable = y
        with pytest.raises(AttributeError):
            quadratic.coefficient = 3.0


class TestLifecycle:
    def test_nothing_can_be_added_after_finalize(self):
        import SHOTpy

        _, problem, x, y = make_problem()
        problem.setObjective(x + y, SHOTpy.ObjectiveDirection.Minimize)
        assert not problem.isFinalized
        problem.finalize()
        assert problem.isFinalized

        calls = [
            lambda: problem.addVariable("z", SHOTpy.VariableType.Real, 0.0, 1.0),
            lambda: problem.addVariable(SHOTpy.Variable("z", SHOTpy.VariableType.Real, 0.0, 1.0)),
            lambda: problem.addConstraint(x <= 5, "c"),
            lambda: problem.addConstraint(SHOTpy.LinearConstraint("c", 0.0, 1.0)),
            lambda: problem.addConstraints([x <= 5]),
            lambda: problem.setObjective(x),
            lambda: problem.setObjective(1.0),
        ]

        for call in calls:
            with pytest.raises(RuntimeError):
                call()

        assert len(problem.allVariables) == 2
        assert len(problem.numericConstraints) == 0

    def test_variable_cannot_be_added_twice(self):
        import SHOTpy

        _, problem, x, _ = make_problem()

        with pytest.raises(ValueError):
            problem.addVariable(x)

        _, otherProblem, _, _ = make_problem()
        with pytest.raises(ValueError):
            otherProblem.addVariable(x)

        assert [V.index for V in problem.allVariables] == [0, 1]
        assert x.index == 0

    def test_list_with_a_repeated_variable_leaves_the_problem_unchanged(self):
        import SHOTpy

        _, problem, _, _ = make_problem()
        z = SHOTpy.Variable("z", SHOTpy.VariableType.Real, 0.0, 1.0)

        with pytest.raises(ValueError):
            problem.addVariables([z, z])

        assert len(problem.allVariables) == 2
        assert z.index == -1

    def test_constraint_cannot_be_added_twice(self):
        import SHOTpy

        _, problem, x, _ = make_problem()
        constraint = SHOTpy.LinearConstraint("c", 0.0, 1.0)
        constraint.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(constraint)

        with pytest.raises(ValueError):
            problem.addConstraint(constraint)
        with pytest.raises(ValueError):
            problem.addConstraints([constraint])

        assert len(problem.numericConstraints) == 1

    def test_objective_cannot_be_used_by_two_problems(self):
        import SHOTpy

        _, problem, x, _ = make_problem()
        objective = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        objective.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(objective)

        _, otherProblem, _, _ = make_problem()
        with pytest.raises(ValueError):
            otherProblem.setObjective(objective)


class TestAddConstraints:
    def test_comparisons(self):
        _, problem, x, y = make_problem()
        problem.addConstraints([x <= 1, x + y >= 2])

        assert [C.name for C in problem.numericConstraints] == ["constraint_0", "constraint_1"]

    def test_comparisons_with_names(self):
        _, problem, x, y = make_problem()
        problem.addConstraints([x <= 1, x * y >= 2], ["a", "b"])

        assert [C.name for C in problem.numericConstraints] == ["a", "b"]
        assert problem.getConstraint("b").calculateFunctionValue([1.0, 3.0]) == pytest.approx(3.0)

    def test_number_of_names_must_match(self):
        _, problem, x, y = make_problem()

        with pytest.raises(ValueError):
            problem.addConstraints([x <= 1, y <= 1], ["a"])

        assert len(problem.numericConstraints) == 0


class TestSettings:
    def test_integer_value_of_a_double_setting(self, solver):
        solver.updateSetting("Termination.TimeLimit", 10)
        assert solver.getDoubleSetting("Termination.TimeLimit") == 10.0

    def test_unknown_setting(self, solver):
        for value in (5, 5.0, True, "a"):
            with pytest.raises(RuntimeError, match="not found"):
                solver.updateSetting("Foo.Bar", value)

    def test_value_of_the_wrong_type(self, solver):
        with pytest.raises(RuntimeError, match="wrong type"):
            solver.updateSetting("Output.Console.LogLevel", 2.5)
        with pytest.raises(RuntimeError, match="wrong type"):
            solver.updateSetting("Model.Reformulation.Monomials.Extract", 1)
        with pytest.raises(RuntimeError, match="wrong type"):
            solver.updateSetting("Termination.TimeLimit", "10")


class TestNames:
    def test_solution_statistics_qp(self):
        import SHOTpy

        assert hasattr(SHOTpy.SolutionStatistics, "numberOfProblemsQP")

    def test_variable_keywords(self):
        import SHOTpy

        x = SHOTpy.Variable(name="x", type=SHOTpy.VariableType.Real, lowerBound=-1.0, upperBound=2.0)
        assert (x.lowerBound, x.upperBound) == (-1.0, 2.0)
