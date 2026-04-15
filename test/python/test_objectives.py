"""
Tests for Objective Function creation in the Python API.
"""

import pytest


class TestLinearObjective:
    """Tests for linear objective functions."""

    def test_create_linear_minimize(self, problem):
        """Test creating a linear minimization objective."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        obj.add(SHOTpy.LinearTerm(2.0, y))
        problem.setObjective(obj)
        
        problem.finalize()
        problem_str = problem.toString()
        assert "minimize" in problem_str.lower()
        assert "x" in problem_str
        assert "y" in problem_str

    def test_create_linear_maximize(self, problem):
        """Test creating a linear maximization objective."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Maximize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        problem_str = problem.toString()
        assert "maximize" in problem_str.lower()


class TestQuadraticObjective:
    """Tests for quadratic objective functions."""

    def test_create_quadratic_objective(self, problem):
        """Test creating a quadratic objective."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        obj = SHOTpy.QuadraticObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.QuadraticTerm(1.0, x, x))
        obj.add(SHOTpy.QuadraticTerm(1.0, y, y))
        problem.setObjective(obj)
        
        problem.finalize()
        problem_str = problem.toString()
        assert "x" in problem_str
        assert "y" in problem_str

    def test_quadratic_with_constant(self, problem):
        """Test quadratic objective with constant term.
        
        Note: QuadraticObjectiveFunction only accepts QuadraticTerms via add().
        Use the constant property for constant terms.
        """
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        # x^2 + 4 (pure quadratic with constant)
        obj = SHOTpy.QuadraticObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.QuadraticTerm(1.0, x, x))
        obj.constant = 4.0
        problem.setObjective(obj)
        
        problem.finalize()
        problem_str = problem.toString()
        assert "x" in problem_str


class TestNonlinearObjective:
    """Tests for nonlinear objective functions."""

    def test_create_nonlinear_objective(self, problem):
        """Test creating a nonlinear objective."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 1.0, 10.0)
        problem.addVariable(x)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.log(x))
        problem.setObjective(obj)
        
        problem.finalize()
        problem_str = problem.toString()
        assert "log" in problem_str.lower()

    def test_nonlinear_objective_with_expression(self, problem):
        """Test nonlinear objective built with expression operators."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        expr = (x - 1)**2 + (y - 2)**2
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        problem_str = problem.toString()
        assert "x" in problem_str
        assert "y" in problem_str

    def test_objective_with_log_term(self, problem):
        """Test objective with logarithm term."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        b = SHOTpy.Variable("b", 1, SHOTpy.VariableType.Binary, 0.0, 1.0)
        problem.addVariable(x)
        problem.addVariable(b)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        expr = x - SHOTpy.log(1 + b)
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        problem_str = problem.toString()
        assert "x" in problem_str
        assert "b" in problem_str
        assert "log" in problem_str.lower()

    def test_complex_nonlinear_objective(self, problem):
        """Test a complex nonlinear objective like ex1223b."""
        import SHOTpy
        
        # Create variables
        x1 = SHOTpy.Variable("x1", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        x2 = SHOTpy.Variable("x2", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        b1 = SHOTpy.Variable("b1", 2, SHOTpy.VariableType.Binary, 0.0, 1.0)
        b2 = SHOTpy.Variable("b2", 3, SHOTpy.VariableType.Binary, 0.0, 1.0)
        
        problem.addVariable(x1)
        problem.addVariable(x2)
        problem.addVariable(b1)
        problem.addVariable(b2)
        
        # minimize (b1-1)^2 + (b2-2)^2 - log(1+b1) + (x1-1)^2 + (x2-2)^2
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        expr = (b1 - 1)**2 + (b2 - 2)**2 - SHOTpy.log(1 + b1) + (x1 - 1)**2 + (x2 - 2)**2
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        problem_str = problem.toString()
        assert "x1" in problem_str
        assert "x2" in problem_str
        assert "b1" in problem_str
        assert "b2" in problem_str
        assert "log" in problem_str.lower()
