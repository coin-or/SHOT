"""
Tests for Constraint creation in the Python API.
"""

import pytest
from conftest import set_default_objective


class TestLinearConstraints:
    """Tests for linear constraints."""

    def test_create_linear_constraint(self, problem):
        """Test creating a basic linear constraint."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        # x <= 5
        c = SHOTpy.LinearConstraint(0, "c1", -SHOTpy.SHOT_DBL_MAX, 5.0)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        set_default_objective(problem, [x])
        problem.finalize()
        problem_str = problem.toString()
        assert "c1" in problem_str
        assert "x" in problem_str

    def test_create_equality_constraint(self, problem):
        """Test creating an equality constraint."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # x + y = 5
        c = SHOTpy.LinearConstraint(0, "eq1", 5.0, 5.0)
        c.add(SHOTpy.LinearTerm(1.0, x))
        c.add(SHOTpy.LinearTerm(1.0, y))
        problem.addConstraint(c)
        
        set_default_objective(problem, [x, y])
        problem.finalize()
        problem_str = problem.toString()
        assert "eq1" in problem_str

    def test_multiple_linear_terms(self, problem):
        """Test constraint with multiple linear terms."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        # 2x + 3y - z <= 10
        c = SHOTpy.LinearConstraint(0, "c1", -SHOTpy.SHOT_DBL_MAX, 10.0)
        c.add(SHOTpy.LinearTerm(2.0, x))
        c.add(SHOTpy.LinearTerm(3.0, y))
        c.add(SHOTpy.LinearTerm(-1.0, z))
        problem.addConstraint(c)
        
        set_default_objective(problem, [x, y, z])
        problem.finalize()
        problem_str = problem.toString()
        assert "x" in problem_str
        assert "y" in problem_str
        assert "z" in problem_str


class TestQuadraticConstraints:
    """Tests for quadratic constraints."""

    def test_create_quadratic_constraint(self, problem):
        """Test creating a basic quadratic constraint."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        # x^2 <= 25
        c = SHOTpy.QuadraticConstraint(0, "q1", -SHOTpy.SHOT_DBL_MAX, 25.0)
        c.add(SHOTpy.QuadraticTerm(1.0, x, x))
        problem.addConstraint(c)
        
        set_default_objective(problem, [x])
        problem.finalize()
        problem_str = problem.toString()
        assert "q1" in problem_str
        assert "x^2" in problem_str or "x*x" in problem_str

    def test_quadratic_with_cross_term(self, problem):
        """Test quadratic constraint with cross term."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # x^2 + 2xy + y^2 <= 100 (i.e., (x+y)^2 <= 100)
        c = SHOTpy.QuadraticConstraint(0, "q1", -SHOTpy.SHOT_DBL_MAX, 100.0)
        c.add(SHOTpy.QuadraticTerm(1.0, x, x))
        c.add(SHOTpy.QuadraticTerm(2.0, x, y))
        c.add(SHOTpy.QuadraticTerm(1.0, y, y))
        problem.addConstraint(c)
        
        set_default_objective(problem, [x, y])
        problem.finalize()
        problem_str = problem.toString()
        assert "x" in problem_str
        assert "y" in problem_str

    def test_quadratic_with_linear_terms(self, problem):
        """Test quadratic constraint with both linear and quadratic terms.
        
        Note: QuadraticConstraint only accepts QuadraticTerms via add().
        To have linear terms in a quadratic constraint, they need to be
        in the linear terms collection - but this API doesn't expose that directly.
        This test verifies that quadratic constraints work correctly.
        """
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # x^2 + y^2 <= 10 (pure quadratic)
        c = SHOTpy.QuadraticConstraint(0, "q1", -SHOTpy.SHOT_DBL_MAX, 10.0)
        c.add(SHOTpy.QuadraticTerm(1.0, x, x))
        c.add(SHOTpy.QuadraticTerm(1.0, y, y))
        problem.addConstraint(c)
        
        set_default_objective(problem, [x, y])
        problem.finalize()
        problem_str = problem.toString()
        assert "x" in problem_str
        assert "y" in problem_str


class TestNonlinearConstraints:
    """Tests for nonlinear constraints."""

    def test_create_nonlinear_constraint(self, problem):
        """Test creating a basic nonlinear constraint."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 1.0, 10.0)
        problem.addVariable(x)
        
        # log(x) <= 2
        c = SHOTpy.NonlinearConstraint(0, "nl1", -SHOTpy.SHOT_DBL_MAX, 2.0)
        c.add(SHOTpy.log(x))
        problem.addConstraint(c)
        
        set_default_objective(problem, [x])
        problem.finalize()
        problem_str = problem.toString()
        assert "nl1" in problem_str
        assert "log" in problem_str.lower()

    def test_nonlinear_with_multiple_variables(self, problem):
        """Test nonlinear constraint with multiple variables."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # exp(x) + exp(y) <= 100
        c = SHOTpy.NonlinearConstraint(0, "nl1", -SHOTpy.SHOT_DBL_MAX, 100.0)
        c.add(SHOTpy.exp(x) + SHOTpy.exp(y))
        problem.addConstraint(c)
        
        set_default_objective(problem, [x, y])
        problem.finalize()
        problem_str = problem.toString()
        assert "exp" in problem_str.lower()


class TestMixedConstraints:
    """Tests for problems with mixed constraint types."""

    def test_linear_and_quadratic(self, problem):
        """Test problem with both linear and quadratic constraints."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # Linear: x + y <= 5
        c1 = SHOTpy.LinearConstraint(0, "lin1", -SHOTpy.SHOT_DBL_MAX, 5.0)
        c1.add(SHOTpy.LinearTerm(1.0, x))
        c1.add(SHOTpy.LinearTerm(1.0, y))
        problem.addConstraint(c1)
        
        # Quadratic: x^2 + y^2 <= 10
        c2 = SHOTpy.QuadraticConstraint(1, "quad1", -SHOTpy.SHOT_DBL_MAX, 10.0)
        c2.add(SHOTpy.QuadraticTerm(1.0, x, x))
        c2.add(SHOTpy.QuadraticTerm(1.0, y, y))
        problem.addConstraint(c2)
        
        set_default_objective(problem, [x, y])
        problem.finalize()
        problem_str = problem.toString()
        assert "lin1" in problem_str
        assert "quad1" in problem_str

    def test_all_constraint_types(self, problem):
        """Test problem with linear, quadratic, and nonlinear constraints."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 1.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 1.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # Linear objective
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        obj.add(SHOTpy.LinearTerm(1.0, y))
        problem.setObjective(obj)
        
        # Linear: x + y <= 10
        c1 = SHOTpy.LinearConstraint(0, "lin", -SHOTpy.SHOT_DBL_MAX, 10.0)
        c1.add(SHOTpy.LinearTerm(1.0, x))
        c1.add(SHOTpy.LinearTerm(1.0, y))
        problem.addConstraint(c1)
        
        # Quadratic: x^2 + y^2 <= 50
        c2 = SHOTpy.QuadraticConstraint(1, "quad", -SHOTpy.SHOT_DBL_MAX, 50.0)
        c2.add(SHOTpy.QuadraticTerm(1.0, x, x))
        c2.add(SHOTpy.QuadraticTerm(1.0, y, y))
        problem.addConstraint(c2)
        
        # Nonlinear: log(x*y) >= 0
        c3 = SHOTpy.NonlinearConstraint(2, "nonlin", 0.0, SHOTpy.SHOT_DBL_MAX)
        c3.add(SHOTpy.log(x * y))
        problem.addConstraint(c3)
        
        problem.finalize()
        problem_str = problem.toString()
        assert "lin" in problem_str
        assert "quad" in problem_str
        assert "nonlin" in problem_str
