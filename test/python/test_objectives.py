"""
Tests for Objective Function creation in the Python API.
"""

import pytest


class TestLinearObjective:
    """Tests for linear objective functions."""

    def test_create_linear_minimize(self, problem):
        """Test creating a linear minimization objective."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        y = shotpy.Variable("y", 1, shotpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        obj = shotpy.LinearObjectiveFunction(shotpy.ObjectiveDirection.Minimize)
        obj.add(shotpy.LinearTerm(1.0, x))
        obj.add(shotpy.LinearTerm(2.0, y))
        problem.setObjective(obj)
        
        problem.finalize()
        problem_str = problem.toString()
        assert "minimize" in problem_str.lower()
        assert "x" in problem_str
        assert "y" in problem_str

    def test_create_linear_maximize(self, problem):
        """Test creating a linear maximization objective."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        obj = shotpy.LinearObjectiveFunction(shotpy.ObjectiveDirection.Maximize)
        obj.add(shotpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        problem_str = problem.toString()
        assert "maximize" in problem_str.lower()


class TestQuadraticObjective:
    """Tests for quadratic objective functions."""

    def test_create_quadratic_objective(self, problem):
        """Test creating a quadratic objective."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        y = shotpy.Variable("y", 1, shotpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        obj = shotpy.QuadraticObjectiveFunction(shotpy.ObjectiveDirection.Minimize)
        obj.add(shotpy.QuadraticTerm(1.0, x, x))
        obj.add(shotpy.QuadraticTerm(1.0, y, y))
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
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        # x^2 + 4 (pure quadratic with constant)
        obj = shotpy.QuadraticObjectiveFunction(shotpy.ObjectiveDirection.Minimize)
        obj.add(shotpy.QuadraticTerm(1.0, x, x))
        obj.constant = 4.0
        problem.setObjective(obj)
        
        problem.finalize()
        problem_str = problem.toString()
        assert "x" in problem_str


class TestNonlinearObjective:
    """Tests for nonlinear objective functions."""

    def test_create_nonlinear_objective(self, problem):
        """Test creating a nonlinear objective."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 1.0, 10.0)
        problem.addVariable(x)
        
        obj = shotpy.NonlinearObjectiveFunction(shotpy.ObjectiveDirection.Minimize)
        obj.add(shotpy.log(x))
        problem.setObjective(obj)
        
        problem.finalize()
        problem_str = problem.toString()
        assert "log" in problem_str.lower()

    def test_nonlinear_objective_with_expression(self, problem):
        """Test nonlinear objective built with expression operators."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        y = shotpy.Variable("y", 1, shotpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        obj = shotpy.NonlinearObjectiveFunction(shotpy.ObjectiveDirection.Minimize)
        expr = (x - 1)**2 + (y - 2)**2
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        problem_str = problem.toString()
        assert "x" in problem_str
        assert "y" in problem_str

    def test_objective_with_log_term(self, problem):
        """Test objective with logarithm term."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        b = shotpy.Variable("b", 1, shotpy.VariableType.Binary, 0.0, 1.0)
        problem.addVariable(x)
        problem.addVariable(b)
        
        obj = shotpy.NonlinearObjectiveFunction(shotpy.ObjectiveDirection.Minimize)
        expr = x - shotpy.log(1 + b)
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        problem_str = problem.toString()
        assert "x" in problem_str
        assert "b" in problem_str
        assert "log" in problem_str.lower()

    def test_complex_nonlinear_objective(self, problem):
        """Test a complex nonlinear objective like ex1223b."""
        import shotpy
        
        # Create variables
        x1 = shotpy.Variable("x1", 0, shotpy.VariableType.Real, 0.0, 10.0)
        x2 = shotpy.Variable("x2", 1, shotpy.VariableType.Real, 0.0, 10.0)
        b1 = shotpy.Variable("b1", 2, shotpy.VariableType.Binary, 0.0, 1.0)
        b2 = shotpy.Variable("b2", 3, shotpy.VariableType.Binary, 0.0, 1.0)
        
        problem.addVariable(x1)
        problem.addVariable(x2)
        problem.addVariable(b1)
        problem.addVariable(b2)
        
        # minimize (b1-1)^2 + (b2-2)^2 - log(1+b1) + (x1-1)^2 + (x2-2)^2
        obj = shotpy.NonlinearObjectiveFunction(shotpy.ObjectiveDirection.Minimize)
        expr = (b1 - 1)**2 + (b2 - 2)**2 - shotpy.log(1 + b1) + (x1 - 1)**2 + (x2 - 2)**2
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        problem_str = problem.toString()
        assert "x1" in problem_str
        assert "x2" in problem_str
        assert "b1" in problem_str
        assert "b2" in problem_str
        assert "log" in problem_str.lower()
