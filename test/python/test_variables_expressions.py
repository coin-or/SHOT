"""
Tests for Variable and Expression creation in the Python API.
"""

import pytest
import math


class TestVariableCreation:
    """Tests for creating variables."""

    def test_create_real_variable(self, problem):
        """Test creating a real (continuous) variable."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        assert x.name == "x"
        assert x.index == 0
        assert x.lowerBound == 0.0
        assert x.upperBound == 10.0

    def test_create_binary_variable(self, problem):
        """Test creating a binary variable."""
        import SHOTpy
        
        b = SHOTpy.Variable("b", SHOTpy.VariableType.Binary, 0.0, 1.0)
        problem.addVariable(b)
        
        assert b.name == "b"
        assert b.index == 0
        # Binary variables have bounds [0, 1]
        assert b.lowerBound == 0.0
        assert b.upperBound == 1.0

    def test_create_integer_variable(self, problem):
        """Test creating an integer variable."""
        import SHOTpy
        
        i = SHOTpy.Variable("i", SHOTpy.VariableType.Integer, -5.0, 5.0)
        problem.addVariable(i)
        
        assert i.name == "i"
        assert i.lowerBound == -5.0
        assert i.upperBound == 5.0

    def test_multiple_variables(self, problem):
        """Test creating multiple variables with correct indices."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", SHOTpy.VariableType.Real, 0.0, 10.0)
        z = SHOTpy.Variable("z", SHOTpy.VariableType.Real, 0.0, 10.0)
        
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        # Check we can retrieve them
        assert problem.getVariable(0).name == "x"
        assert problem.getVariable(1).name == "y"
        assert problem.getVariable(2).name == "z"

    def test_variable_identity(self, problem):
        """Test that added variables maintain identity."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        retrieved = problem.getVariable(0)
        assert x is retrieved


class TestExpressionBuilding:
    """Tests for building expressions using operator overloading."""

    def test_variable_addition(self, problem):
        """Test adding two variables."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        expr = x + y
        assert "x" in str(expr)
        assert "y" in str(expr)

    def test_variable_plus_constant(self, problem):
        """Test adding a constant to a variable."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        expr = x + 5
        assert "x" in str(expr)
        assert "5" in str(expr)

    def test_constant_plus_variable(self, problem):
        """Test adding a variable to a constant (reverse add)."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        expr = 5 + x
        assert "x" in str(expr)
        assert "5" in str(expr)

    def test_variable_subtraction(self, problem):
        """Test subtracting variables."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        expr = x - 1
        # The expression should contain x and -1
        expr_str = str(expr)
        assert "x" in expr_str

    def test_variable_multiplication(self, problem):
        """Test multiplying variables."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        expr = x * y
        assert "x" in str(expr)
        assert "y" in str(expr)

    def test_variable_power(self, problem):
        """Test variable raised to a power."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        expr = x ** 2
        expr_str = str(expr)
        assert "x" in expr_str
        assert "^2" in expr_str or "**2" in expr_str or "2" in expr_str

    def test_squared_expression(self, problem):
        """Test squaring an expression like (x-1)^2."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        expr = (x - 1) ** 2
        expr_str = str(expr)
        assert "x" in expr_str

    def test_log_expression(self, problem):
        """Test log function."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 1.0, 10.0)
        problem.addVariable(x)
        
        expr = SHOTpy.log(x)
        assert "log" in str(expr).lower()

    def test_exp_expression(self, problem):
        """Test exp function."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        expr = SHOTpy.exp(x)
        assert "exp" in str(expr).lower()

    def test_sqrt_expression(self, problem):
        """Test sqrt function."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        expr = SHOTpy.sqrt(x)
        assert "sqrt" in str(expr).lower()

    def test_sin_expression(self, problem):
        """Test sin function."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        expr = SHOTpy.sin(x)
        assert "sin" in str(expr).lower()

    def test_cos_expression(self, problem):
        """Test cos function."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        expr = SHOTpy.cos(x)
        assert "cos" in str(expr).lower()

    def test_complex_expression(self, problem):
        """Test building a complex expression."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # (x-1)^2 + (y-2)^2 + log(x+y)
        expr = (x - 1)**2 + (y - 2)**2 + SHOTpy.log(x + y + 1)
        expr_str = str(expr)
        assert "x" in expr_str
        assert "y" in expr_str
        assert "log" in expr_str.lower()


class TestAddVariable:
    """Tests for problem.addVariable(name, type, lowerBound, upperBound), which creates the variable and returns it."""

    def test_returns_the_added_variable(self, problem):
        import SHOTpy

        x = problem.addVariable("x", SHOTpy.VariableType.Real, 0.0, 10.0)
        y = problem.addVariable("y", SHOTpy.VariableType.Integer, -5, 5)

        assert isinstance(x, SHOTpy.Variable)
        assert (x.name, x.index, x.lowerBound, x.upperBound) == ("x", 0, 0.0, 10.0)
        assert (y.name, y.index, y.lowerBound, y.upperBound) == ("y", 1, -5.0, 5.0)
        assert problem.getVariable(0) is x and problem.getVariable(1) is y

    def test_default_bounds(self, problem):
        import SHOTpy

        x = problem.addVariable("x")
        b = problem.addVariable("b", SHOTpy.VariableType.Binary)
        i = problem.addVariable("i", SHOTpy.VariableType.Integer, lowerBound=0)

        assert (x.lowerBound, x.upperBound) == (SHOTpy.SHOT_DBL_MIN, SHOTpy.SHOT_DBL_MAX)
        assert (b.lowerBound, b.upperBound) == (0.0, 1.0)
        assert (i.lowerBound, i.upperBound) == (0.0, SHOTpy.SHOT_DBL_MAX)

    def test_infinite_bounds_are_missing_bounds(self, problem):
        """SHOT marks a missing bound with SHOT_DBL_MIN / SHOT_DBL_MAX, so infinite bounds are given those values."""
        import SHOTpy

        x = problem.addVariable("x", lowerBound=-math.inf, upperBound=math.inf)
        assert (x.lowerBound, x.upperBound) == (SHOTpy.SHOT_DBL_MIN, SHOTpy.SHOT_DBL_MAX)

    @pytest.mark.parametrize("bounds", [
        {"lowerBound": math.nan},
        {"upperBound": math.nan},
        {"lowerBound": math.inf},
        {"upperBound": -math.inf},
        {"lowerBound": 5, "upperBound": 1},
    ])
    def test_invalid_bounds(self, problem, bounds):
        with pytest.raises(ValueError):
            problem.addVariable("x", **bounds)

    def test_default_names(self, problem):
        x = problem.addVariable()
        y = problem.addVariable("y")
        z = problem.addVariable()

        assert (x.name, y.name, z.name) == ("variable_0", "y", "variable_2")

    def test_semicontinuous_variable(self, problem):
        import SHOTpy

        s = problem.addVariable("s", SHOTpy.VariableType.Semicontinuous, 0.0, 10.0, semiBound=2.0)
        assert s.semiBound == 2.0

    def test_add_existing_variable(self, problem):
        """Adding a Variable created separately still works, and None is rejected."""
        import SHOTpy

        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 1.0)
        problem.addVariable(x)
        assert problem.getVariable(0) is x

        with pytest.raises(TypeError):
            problem.addVariable(None)

    def test_solve_a_model_built_with_added_variables(self):
        """A model whose variables are created with addVariable solves to its optimum: maximize x + y with
        x^2 + y^2 <= 2 gives x = y = 1."""
        import SHOTpy

        solver = SHOTpy.Solver()
        solver.updateSetting("Output.Console.LogLevel", 6)
        problem = SHOTpy.Problem(solver.getEnvironment())

        x = problem.addVariable("x", lowerBound=-10, upperBound=10)
        y = problem.addVariable("y", lowerBound=-10, upperBound=10)
        problem.addConstraint(x**2 + y**2 <= 2, "circle")
        problem.setObjective(x + y, SHOTpy.ObjectiveDirection.Maximize)
        problem.finalize()
        solver.setProblem(problem)

        assert solver.solveProblem()
        assert abs(solver.getPrimalBound() - 2.0) < 1e-4


class TestVariableBounds:
    """Tests for the bounds given to a Variable and set on it later."""

    def test_infinite_bounds_in_the_constructor(self):
        import SHOTpy

        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, -math.inf, math.inf)
        assert (x.lowerBound, x.upperBound) == (SHOTpy.SHOT_DBL_MIN, SHOTpy.SHOT_DBL_MAX)

    def test_infinite_bounds_set_later(self):
        import SHOTpy

        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 1.0)
        x.lowerBound = -math.inf
        x.upperBound = math.inf
        assert (x.lowerBound, x.upperBound) == (SHOTpy.SHOT_DBL_MIN, SHOTpy.SHOT_DBL_MAX)

        x.upperBound = 5.0
        assert x.upperBound == 5.0

    def test_nan_bounds(self):
        import SHOTpy

        with pytest.raises(ValueError):
            SHOTpy.Variable("x", SHOTpy.VariableType.Real, math.nan, 1.0)

        x = SHOTpy.Variable("x", SHOTpy.VariableType.Real, 0.0, 1.0)
        with pytest.raises(ValueError):
            x.upperBound = math.nan
