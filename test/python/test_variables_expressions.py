"""
Tests for Variable and Expression creation in the Python API.
"""

import pytest
import math


class TestVariableCreation:
    """Tests for creating variables."""

    def test_create_real_variable(self, problem):
        """Test creating a real (continuous) variable."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        assert x.name == "x"
        assert x.index == 0
        assert x.lowerBound == 0.0
        assert x.upperBound == 10.0

    def test_create_binary_variable(self, problem):
        """Test creating a binary variable."""
        import shotpy
        
        b = shotpy.Variable("b", 0, shotpy.VariableType.Binary, 0.0, 1.0)
        problem.addVariable(b)
        
        assert b.name == "b"
        assert b.index == 0
        # Binary variables have bounds [0, 1]
        assert b.lowerBound == 0.0
        assert b.upperBound == 1.0

    def test_create_integer_variable(self, problem):
        """Test creating an integer variable."""
        import shotpy
        
        i = shotpy.Variable("i", 0, shotpy.VariableType.Integer, -5.0, 5.0)
        problem.addVariable(i)
        
        assert i.name == "i"
        assert i.lowerBound == -5.0
        assert i.upperBound == 5.0

    def test_multiple_variables(self, problem):
        """Test creating multiple variables with correct indices."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        y = shotpy.Variable("y", 1, shotpy.VariableType.Real, 0.0, 10.0)
        z = shotpy.Variable("z", 2, shotpy.VariableType.Real, 0.0, 10.0)
        
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        # Check we can retrieve them
        assert problem.getVariable(0).name == "x"
        assert problem.getVariable(1).name == "y"
        assert problem.getVariable(2).name == "z"

    def test_variable_identity(self, problem):
        """Test that added variables maintain identity."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        retrieved = problem.getVariable(0)
        assert x is retrieved


class TestExpressionBuilding:
    """Tests for building expressions using operator overloading."""

    def test_variable_addition(self, problem):
        """Test adding two variables."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        y = shotpy.Variable("y", 1, shotpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        expr = x + y
        assert "x" in str(expr)
        assert "y" in str(expr)

    def test_variable_plus_constant(self, problem):
        """Test adding a constant to a variable."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        expr = x + 5
        assert "x" in str(expr)
        assert "5" in str(expr)

    def test_constant_plus_variable(self, problem):
        """Test adding a variable to a constant (reverse add)."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        expr = 5 + x
        assert "x" in str(expr)
        assert "5" in str(expr)

    def test_variable_subtraction(self, problem):
        """Test subtracting variables."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        expr = x - 1
        # The expression should contain x and -1
        expr_str = str(expr)
        assert "x" in expr_str

    def test_variable_multiplication(self, problem):
        """Test multiplying variables."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        y = shotpy.Variable("y", 1, shotpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        expr = x * y
        assert "x" in str(expr)
        assert "y" in str(expr)

    def test_variable_power(self, problem):
        """Test variable raised to a power."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        expr = x ** 2
        expr_str = str(expr)
        assert "x" in expr_str
        assert "^2" in expr_str or "**2" in expr_str or "2" in expr_str

    def test_squared_expression(self, problem):
        """Test squaring an expression like (x-1)^2."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        expr = (x - 1) ** 2
        expr_str = str(expr)
        assert "x" in expr_str

    def test_log_expression(self, problem):
        """Test log function."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 1.0, 10.0)
        problem.addVariable(x)
        
        expr = shotpy.log(x)
        assert "log" in str(expr).lower()

    def test_exp_expression(self, problem):
        """Test exp function."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        expr = shotpy.exp(x)
        assert "exp" in str(expr).lower()

    def test_sqrt_expression(self, problem):
        """Test sqrt function."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        expr = shotpy.sqrt(x)
        assert "sqrt" in str(expr).lower()

    def test_sin_expression(self, problem):
        """Test sin function."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        expr = shotpy.sin(x)
        assert "sin" in str(expr).lower()

    def test_cos_expression(self, problem):
        """Test cos function."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        expr = shotpy.cos(x)
        assert "cos" in str(expr).lower()

    def test_complex_expression(self, problem):
        """Test building a complex expression."""
        import shotpy
        
        x = shotpy.Variable("x", 0, shotpy.VariableType.Real, 0.0, 10.0)
        y = shotpy.Variable("y", 1, shotpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # (x-1)^2 + (y-2)^2 + log(x+y)
        expr = (x - 1)**2 + (y - 2)**2 + shotpy.log(x + y + 1)
        expr_str = str(expr)
        assert "x" in expr_str
        assert "y" in expr_str
        assert "log" in expr_str.lower()
