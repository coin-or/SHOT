"""
Tests for Problem.finalize() validation in the Python API.

Verifies that finalize() raises RuntimeError when:
- No objective function has been set
- No variables have been added
And that calling finalize() twice warns but does not raise.
"""

import pytest
import SHOTpy
from conftest import SHOTContext


class TestFinalizeNoObjective:
    """finalize() must raise RuntimeError when no objective function is set."""

    def test_raises_runtime_error(self):
        ctx = SHOTContext()
        var_x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 100.0)
        ctx.problem.addVariable(var_x)

        # No objective set
        with pytest.raises(RuntimeError, match="objective"):
            ctx.problem.finalize()

    def test_no_raise_when_objective_present(self):
        ctx = SHOTContext()
        var_x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 100.0)
        ctx.problem.addVariable(var_x)

        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, var_x))
        ctx.problem.setObjective(obj)

        # Should not raise
        ctx.problem.finalize()


class TestFinalizeNoVariables:
    """finalize() must raise RuntimeError when no variables have been added."""

    def test_raises_runtime_error(self):
        ctx = SHOTContext()

        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        ctx.problem.setObjective(obj)

        # No variables added
        with pytest.raises(RuntimeError, match="variables"):
            ctx.problem.finalize()

    def test_no_raise_when_variables_present(self):
        ctx = SHOTContext()
        var_x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 100.0)
        ctx.problem.addVariable(var_x)

        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, var_x))
        ctx.problem.setObjective(obj)

        # Should not raise
        ctx.problem.finalize()


class TestFinalizeCalledTwice:
    """Calling finalize() a second time should warn but not raise."""

    def test_second_call_does_not_raise(self):
        ctx = SHOTContext()
        var_x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 100.0)
        ctx.problem.addVariable(var_x)

        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, var_x))
        ctx.problem.setObjective(obj)

        ctx.problem.finalize()

        # Second call must not raise
        ctx.problem.finalize()

    def test_properties_unchanged_after_second_call(self):
        ctx = SHOTContext()
        var_x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 100.0)
        var_y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Integer, 0.0, 1.0)
        ctx.problem.addVariable(var_x)
        ctx.problem.addVariable(var_y)

        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, var_x))
        obj.add(SHOTpy.LinearTerm(1.0, var_y))
        ctx.problem.setObjective(obj)

        ctx.problem.finalize()
        num_vars_first = ctx.problem.properties.numberOfVariables

        ctx.problem.finalize()
        num_vars_second = ctx.problem.properties.numberOfVariables

        assert num_vars_first == num_vars_second
