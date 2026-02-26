"""
Tests for gradient and Hessian calculations in the Python API.

These tests verify that:
1. Jacobian sparsity patterns are correct for constraints
2. Hessian sparsity patterns are correct for constraints and objectives
3. Gradient values are computed correctly at given points
4. Hessian values are computed correctly at given points
"""

import pytest
import math


class TestJacobianSparsityPattern:
    """Tests for Jacobian sparsity pattern computation."""

    def test_linear_constraint_jacobian(self, problem):
        """Test Jacobian sparsity for a linear constraint: 2*x + 3*y <= 10."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        linear_terms = SHOTpy.LinearTerms()
        linear_terms.add(SHOTpy.LinearTerm(2.0, x))
        linear_terms.add(SHOTpy.LinearTerm(3.0, y))
        
        constr = SHOTpy.LinearConstraint(0, "lin_constr", linear_terms, -1e20, 10.0)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        jacobian_pattern = problem.getConstraintsJacobianSparsityPattern()
        
        # Should have one constraint with two variables
        assert len(jacobian_pattern) == 1
        constr_idx, var_indices = jacobian_pattern[0]
        assert constr_idx == 0
        assert set(var_indices) == {0, 1}

    def test_quadratic_constraint_jacobian(self, problem):
        """Test Jacobian sparsity for a quadratic constraint: x^2 + x*y <= 10."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        quad_terms = SHOTpy.QuadraticTerms()
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, x, x))  # x^2
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, x, y))  # x*y
        
        constr = SHOTpy.QuadraticConstraint(0, "quad_constr", -1e20, 10.0)
        constr.add(quad_terms)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        jacobian_pattern = problem.getConstraintsJacobianSparsityPattern()
        
        # Find our constraint (index 0)
        our_constr_pattern = [p for p in jacobian_pattern if p[0] == 0]
        assert len(our_constr_pattern) >= 1
        _, var_indices = our_constr_pattern[0]
        # Both x and y appear in the gradient (from x^2 and x*y)
        assert set(var_indices) == {0, 1}

    def test_nonlinear_constraint_jacobian(self, problem):
        """Test Jacobian sparsity for a nonlinear constraint: y^3 <= 20."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        expr_power = y ** 3  # y^3
        
        constr = SHOTpy.NonlinearConstraint(0, "nl_constr", expr_power, -1e20, 20.0)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        jacobian_pattern = problem.getConstraintsJacobianSparsityPattern()
        
        # Find our constraint (index 0)
        our_constr_pattern = [p for p in jacobian_pattern if p[0] == 0]
        assert len(our_constr_pattern) >= 1
        _, var_indices = our_constr_pattern[0]
        # Only y appears in the gradient
        assert set(var_indices) == {1}  # Only y


class TestHessianSparsityPattern:
    """Tests for Hessian sparsity pattern computation."""

    def test_linear_objective_hessian(self, problem):
        """Test that linear objective has empty Hessian."""
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
        
        hessian_pattern = problem.getLagrangianHessianSparsityPattern()
        
        # Linear objective has no second derivatives
        assert len(hessian_pattern) == 0

    def test_quadratic_objective_diagonal_hessian(self, problem):
        """Test Hessian for x^2 + y^2 - should have diagonal elements."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        quad_terms = SHOTpy.QuadraticTerms()
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, x, x))  # x^2
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, y, y))  # y^2
        
        obj = SHOTpy.QuadraticObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(quad_terms)
        problem.setObjective(obj)
        
        problem.finalize()
        
        hessian_pattern = problem.getLagrangianHessianSparsityPattern()
        
        # Should have (0,0) and (1,1) for x^2 and y^2
        assert len(hessian_pattern) == 2
        pattern_set = set(hessian_pattern)
        assert (0, 0) in pattern_set
        assert (1, 1) in pattern_set

    def test_quadratic_objective_bilinear_hessian(self, problem):
        """Test Hessian for x + y + x*y - should include off-diagonal element."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 100.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Binary, 0.0, 1.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # Create objective: x + y + x*y using operator overloading
        expr_product = x * y  # x*y
        
        linear_terms = SHOTpy.LinearTerms()
        linear_terms.add(SHOTpy.LinearTerm(1.0, x))
        linear_terms.add(SHOTpy.LinearTerm(1.0, y))
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(linear_terms)
        obj.add(expr_product)
        problem.setObjective(obj)
        
        problem.finalize()
        
        hessian_pattern = problem.getLagrangianHessianSparsityPattern()
        
        # Should have (0,1) for x*y bilinear term
        pattern_set = set(hessian_pattern)
        assert (0, 1) in pattern_set, f"Expected (0,1) in pattern but got {pattern_set}"

    def test_quadratic_objective_full_hessian(self, problem):
        """Test Hessian for x^2 + y^2 + x*y - should have all three elements."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        quad_terms = SHOTpy.QuadraticTerms()
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, x, x))  # x^2
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, y, y))  # y^2
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, x, y))  # x*y
        
        obj = SHOTpy.QuadraticObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(quad_terms)
        problem.setObjective(obj)
        
        problem.finalize()
        
        hessian_pattern = problem.getLagrangianHessianSparsityPattern()
        
        # Should have (0,0), (0,1), and (1,1)
        pattern_set = set(hessian_pattern)
        assert (0, 0) in pattern_set
        assert (0, 1) in pattern_set
        assert (1, 1) in pattern_set

    def test_constraints_hessian_separate_from_objective(self, problem):
        """Test that constraint Hessian is computed separately from objective."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # Constraint: x^2 <= 10
        quad_terms = SHOTpy.QuadraticTerms()
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, x, x))  # x^2
        constr = SHOTpy.QuadraticConstraint(0, "quad_constr", -1e20, 10.0)
        constr.add(quad_terms)
        problem.addConstraint(constr)
        
        # Objective: y (linear)
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, y))
        problem.setObjective(obj)
        
        problem.finalize()
        
        # Constraint Hessian should have (0,0) from x^2
        constr_hessian = problem.getConstraintsHessianSparsityPattern()
        assert len(constr_hessian) == 1
        assert (0, 0) in set(constr_hessian)
        
        # Lagrangian Hessian should also have (0,0)
        lagrangian_hessian = problem.getLagrangianHessianSparsityPattern()
        assert (0, 0) in set(lagrangian_hessian)


class TestGradientValues:
    """Tests for gradient value computation."""

    def test_linear_constraint_gradient(self, problem):
        """Test gradient of 2*x + 3*y at any point is [2, 3]."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        linear_terms = SHOTpy.LinearTerms()
        linear_terms.add(SHOTpy.LinearTerm(2.0, x))
        linear_terms.add(SHOTpy.LinearTerm(3.0, y))
        
        constr = SHOTpy.LinearConstraint(0, "lin_constr", linear_terms, -1e20, 10.0)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        # Gradient is constant for linear function
        gradient = constr.calculateGradient([1.0, 2.0])
        assert abs(gradient[0] - 2.0) < 1e-10
        assert abs(gradient[1] - 3.0) < 1e-10

    def test_quadratic_constraint_gradient(self, problem):
        """Test gradient of x^2 + 2*x*y at point (2, 3) is [2*2 + 2*3, 2*2] = [10, 4]."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        quad_terms = SHOTpy.QuadraticTerms()
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, x, x))  # x^2
        quad_terms.add(SHOTpy.QuadraticTerm(2.0, x, y))  # 2*x*y
        
        constr = SHOTpy.QuadraticConstraint(0, "quad_constr", -1e20, 10.0)
        constr.add(quad_terms)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        # At (2, 3): d/dx = 2*x + 2*y = 4 + 6 = 10, d/dy = 2*x = 4
        gradient = constr.calculateGradient([2.0, 3.0])
        assert abs(gradient[0] - 10.0) < 1e-10
        assert abs(gradient[1] - 4.0) < 1e-10

    def test_objective_gradient(self, problem):
        """Test gradient of objective x^2 + 3*y at point (2, 1) is [4, 3]."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        quad_terms = SHOTpy.QuadraticTerms()
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, x, x))  # x^2
        
        linear_terms = SHOTpy.LinearTerms()
        linear_terms.add(SHOTpy.LinearTerm(3.0, y))  # 3*y
        
        obj = SHOTpy.QuadraticObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(quad_terms)
        obj.add(linear_terms)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # At (2, 1): d/dx = 2*x = 4, d/dy = 3
        gradient = problem.objectiveFunction.calculateGradient([2.0, 1.0])
        assert abs(gradient[0] - 4.0) < 1e-10
        assert abs(gradient[1] - 3.0) < 1e-10


class TestHessianValues:
    """Tests for Hessian value computation."""

    def test_linear_constraint_hessian(self, problem):
        """Test that linear constraint has empty Hessian."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        linear_terms = SHOTpy.LinearTerms()
        linear_terms.add(SHOTpy.LinearTerm(2.0, x))
        linear_terms.add(SHOTpy.LinearTerm(3.0, y))
        
        constr = SHOTpy.LinearConstraint(0, "lin_constr", linear_terms, -1e20, 10.0)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        hessian = constr.calculateHessian([1.0, 2.0])
        assert len(hessian) == 0

    def test_quadratic_constraint_hessian_diagonal(self, problem):
        """Test Hessian of x^2 + y^2 is [[2, 0], [0, 2]]."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        quad_terms = SHOTpy.QuadraticTerms()
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, x, x))  # x^2
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, y, y))  # y^2
        
        constr = SHOTpy.QuadraticConstraint(0, "quad_constr", -1e20, 10.0)
        constr.add(quad_terms)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        hessian = constr.calculateHessian([1.0, 2.0])
        # d^2/dx^2 of x^2 = 2, d^2/dy^2 of y^2 = 2
        assert abs(hessian[(0, 0)] - 2.0) < 1e-10
        assert abs(hessian[(1, 1)] - 2.0) < 1e-10

    def test_quadratic_constraint_hessian_bilinear(self, problem):
        """Test Hessian of x*y is [[0, 1], [1, 0]]."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        quad_terms = SHOTpy.QuadraticTerms()
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, x, y))  # x*y
        
        constr = SHOTpy.QuadraticConstraint(0, "quad_constr", -1e20, 10.0)
        constr.add(quad_terms)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        hessian = constr.calculateHessian([1.0, 2.0])
        # d^2/dxdy of x*y = 1 (stored in upper triangle)
        assert (0, 1) in hessian
        assert abs(hessian[(0, 1)] - 1.0) < 1e-10

    def test_objective_hessian_full(self, problem):
        """Test Hessian of 2*x^2 + 3*y^2 + 4*x*y is [[4, 4], [4, 6]]."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        quad_terms = SHOTpy.QuadraticTerms()
        quad_terms.add(SHOTpy.QuadraticTerm(2.0, x, x))  # 2*x^2
        quad_terms.add(SHOTpy.QuadraticTerm(3.0, y, y))  # 3*y^2
        quad_terms.add(SHOTpy.QuadraticTerm(4.0, x, y))  # 4*x*y
        
        obj = SHOTpy.QuadraticObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(quad_terms)
        problem.setObjective(obj)
        
        problem.finalize()
        
        hessian = problem.objectiveFunction.calculateHessian([1.0, 2.0])
        # d^2/dx^2 of 2*x^2 = 4, d^2/dy^2 of 3*y^2 = 6, d^2/dxdy of 4*x*y = 4
        assert abs(hessian[(0, 0)] - 4.0) < 1e-10
        assert abs(hessian[(1, 1)] - 6.0) < 1e-10
        assert abs(hessian[(0, 1)] - 4.0) < 1e-10


class TestThreeVariableHessian:
    """Tests for Hessian with three variables to verify ordering is correct."""

    def test_bilinear_terms_three_variables(self, problem):
        """Test Hessian sparsity for x*y + y*z + x*z with three variables."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        quad_terms = SHOTpy.QuadraticTerms()
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, x, y))  # x*y
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, y, z))  # y*z
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, x, z))  # x*z
        
        obj = SHOTpy.QuadraticObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(quad_terms)
        problem.setObjective(obj)
        
        problem.finalize()
        
        hessian_pattern = problem.getLagrangianHessianSparsityPattern()
        pattern_set = set(hessian_pattern)
        
        # Should have (0,1), (0,2), (1,2) - all in upper triangle (smaller index first)
        assert (0, 1) in pattern_set, f"Expected (0,1) in pattern but got {pattern_set}"
        assert (0, 2) in pattern_set, f"Expected (0,2) in pattern but got {pattern_set}"
        assert (1, 2) in pattern_set, f"Expected (1,2) in pattern but got {pattern_set}"

    def test_bilinear_terms_reverse_order(self, problem):
        """Test that y*x is stored as (0,1) not (1,0)."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # Add y*x (second var first)
        quad_terms = SHOTpy.QuadraticTerms()
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, y, x))  # y*x
        
        obj = SHOTpy.QuadraticObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(quad_terms)
        problem.setObjective(obj)
        
        problem.finalize()
        
        hessian_pattern = problem.getLagrangianHessianSparsityPattern()
        pattern_set = set(hessian_pattern)
        
        # Should be (0,1) in upper triangle form
        assert (0, 1) in pattern_set, f"Expected (0,1) in pattern but got {pattern_set}"
        assert (1, 0) not in pattern_set, f"Did not expect (1,0) in pattern"

    def test_mixed_diagonal_and_bilinear(self, problem):
        """Test Hessian for x^2 + y^2 + z^2 + x*y + y*z."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        quad_terms = SHOTpy.QuadraticTerms()
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, x, x))  # x^2
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, y, y))  # y^2
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, z, z))  # z^2
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, x, y))  # x*y
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, y, z))  # y*z
        
        obj = SHOTpy.QuadraticObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(quad_terms)
        problem.setObjective(obj)
        
        problem.finalize()
        
        hessian_pattern = problem.getLagrangianHessianSparsityPattern()
        pattern_set = set(hessian_pattern)
        
        # Diagonal elements
        assert (0, 0) in pattern_set
        assert (1, 1) in pattern_set
        assert (2, 2) in pattern_set
        # Off-diagonal elements
        assert (0, 1) in pattern_set
        assert (1, 2) in pattern_set
        # x*z not present
        assert (0, 2) not in pattern_set

    def test_hessian_values_three_variables(self, problem):
        """Test Hessian values for 2*x*y + 3*y*z + 4*x*z."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        quad_terms = SHOTpy.QuadraticTerms()
        quad_terms.add(SHOTpy.QuadraticTerm(2.0, x, y))  # 2*x*y
        quad_terms.add(SHOTpy.QuadraticTerm(3.0, y, z))  # 3*y*z
        quad_terms.add(SHOTpy.QuadraticTerm(4.0, x, z))  # 4*x*z
        
        obj = SHOTpy.QuadraticObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(quad_terms)
        problem.setObjective(obj)
        
        problem.finalize()
        
        hessian = problem.objectiveFunction.calculateHessian([1.0, 2.0, 3.0])
        
        # d^2/dxdy of 2*x*y = 2
        assert abs(hessian[(0, 1)] - 2.0) < 1e-10
        # d^2/dydz of 3*y*z = 3
        assert abs(hessian[(1, 2)] - 3.0) < 1e-10
        # d^2/dxdz of 4*x*z = 4
        assert abs(hessian[(0, 2)] - 4.0) < 1e-10


class TestExpressionSimplificationHessian:
    """Test that expressions created via operators are correctly simplified and have proper Hessians."""

    def test_expression_product_hessian(self, problem):
        """Test that x*y created via operator has correct Hessian."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        expr_product = x * y  # x*y using operator overloading
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr_product)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # After simplification, x*y should be a quadratic term
        hessian_pattern = problem.getLagrangianHessianSparsityPattern()
        pattern_set = set(hessian_pattern)
        assert (0, 1) in pattern_set
        
        hessian = problem.objectiveFunction.calculateHessian([1.0, 2.0])
        assert (0, 1) in hessian
        assert abs(hessian[(0, 1)] - 1.0) < 1e-10

    def test_expression_square_hessian(self, problem):
        """Test that x^2 created via operator has correct Hessian."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        expr_square = x ** 2  # x^2 using operator overloading
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr_square)
        problem.setObjective(obj)
        
        problem.finalize()
        
        hessian_pattern = problem.getLagrangianHessianSparsityPattern()
        pattern_set = set(hessian_pattern)
        assert (0, 0) in pattern_set
        
        hessian = problem.objectiveFunction.calculateHessian([2.0])
        assert (0, 0) in hessian
        assert abs(hessian[(0, 0)] - 2.0) < 1e-10

    def test_complex_expression_hessian(self, problem):
        """Test Hessian for x^2 + y^2 + 2*x*y created via expressions."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # x^2 + y^2 + 2*x*y = (x+y)^2 using operator overloading
        expr_x_sq = x ** 2
        expr_y_sq = y ** 2
        expr_2xy = 2 * x * y
        
        expr_sum = expr_x_sq + expr_y_sq + expr_2xy
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr_sum)
        problem.setObjective(obj)
        
        problem.finalize()
        
        hessian_pattern = problem.getLagrangianHessianSparsityPattern()
        pattern_set = set(hessian_pattern)
        assert (0, 0) in pattern_set
        assert (0, 1) in pattern_set
        assert (1, 1) in pattern_set
        
        hessian = problem.objectiveFunction.calculateHessian([1.0, 2.0])
        assert abs(hessian[(0, 0)] - 2.0) < 1e-10  # d^2/dx^2 of x^2 = 2
        assert abs(hessian[(1, 1)] - 2.0) < 1e-10  # d^2/dy^2 of y^2 = 2
        assert abs(hessian[(0, 1)] - 2.0) < 1e-10  # d^2/dxdy of 2*x*y = 2


class TestMonomialTerms:
    """Tests for monomial terms (products of variables with integer powers)."""

    def test_monomial_objective_xyz(self, problem):
        """Test monomial term x*y*z in objective."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        # Create monomial term: 2*x*y*z
        monomial = SHOTpy.MonomialTerm(2.0, [x, y, z])
        monomial_terms = SHOTpy.MonomialTerms()
        monomial_terms.add(monomial)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(monomial_terms)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # Hessian of x*y*z has entries at (0,1), (0,2), (1,2)
        hessian_pattern = problem.getLagrangianHessianSparsityPattern()
        pattern_set = set(hessian_pattern)
        
        # All pairs should be present
        assert (0, 1) in pattern_set
        assert (0, 2) in pattern_set
        assert (1, 2) in pattern_set

    def test_monomial_constraint_x2y(self, problem):
        """Test x^2*y in constraint using SignomialTerm for repeated variable powers."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # Use SignomialTerm for x^2*y (explicit powers work correctly)
        signomial = SHOTpy.SignomialTerm(1.0, [(x, 2.0), (y, 1.0)])
        signomial_terms = SHOTpy.SignomialTerms()
        signomial_terms.add(signomial)
        
        constr = SHOTpy.NonlinearConstraint(0, "sig_constr", -1e20, 10.0)
        constr.add(signomial_terms)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        # Gradient at (2, 3): d/dx = 2*x*y = 12, d/dy = x^2 = 4
        constr_ref = problem.getConstraint(0)
        gradient = constr_ref.calculateGradient([2.0, 3.0])
        assert abs(gradient[0] - 12.0) < 1e-10
        assert abs(gradient[1] - 4.0) < 1e-10

    def test_monomial_hessian_values(self, problem):
        """Test Hessian values for monomial x*y*z at point (2, 3, 4)."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        # Monomial: x*y*z
        monomial = SHOTpy.MonomialTerm(1.0, [x, y, z])
        monomial_terms = SHOTpy.MonomialTerms()
        monomial_terms.add(monomial)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(monomial_terms)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # Hessian at (2,3,4):
        # d^2/dxdy = z = 4
        # d^2/dxdz = y = 3
        # d^2/dydz = x = 2
        hessian = problem.objectiveFunction.calculateHessian([2.0, 3.0, 4.0])
        assert abs(hessian[(0, 1)] - 4.0) < 1e-10
        assert abs(hessian[(0, 2)] - 3.0) < 1e-10
        assert abs(hessian[(1, 2)] - 2.0) < 1e-10


class TestSignomialTerms:
    """Tests for signomial terms (products of variables with real powers)."""

    def test_signomial_objective_sqrt(self, problem):
        """Test signomial term x^0.5 (square root) in objective."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        
        # Create signomial: x^0.5 using (variable, power) tuple syntax
        signomial = SHOTpy.SignomialTerm(1.0, [(x, 0.5)])
        signomial_terms = SHOTpy.SignomialTerms()
        signomial_terms.add(signomial)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(signomial_terms)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # Gradient of x^0.5 at x=4: 0.5 * x^(-0.5) = 0.5 * 0.5 = 0.25
        gradient = problem.objectiveFunction.calculateGradient([4.0])
        assert abs(gradient[0] - 0.25) < 1e-10

    def test_signomial_constraint_inverse(self, problem):
        """Test signomial term 1/x = x^(-1) in constraint."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        
        # Create signomial: x^(-1) using (variable, power) tuple syntax
        signomial = SHOTpy.SignomialTerm(1.0, [(x, -1.0)])
        signomial_terms = SHOTpy.SignomialTerms()
        signomial_terms.add(signomial)
        
        constr = SHOTpy.NonlinearConstraint(0, "sig_constr", -1e20, 10.0)
        constr.add(signomial_terms)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        # Gradient of x^(-1) at x=2: -x^(-2) = -0.25
        constr_ref = problem.getConstraint(0)
        gradient = constr_ref.calculateGradient([2.0])
        assert abs(gradient[0] - (-0.25)) < 1e-10

    def test_signomial_two_variables(self, problem):
        """Test signomial x^0.5 * y^1.5 with two variables."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # Create signomial term: x^0.5 * y^1.5
        # Using list of multiple (variable, power) tuples
        signomial = SHOTpy.SignomialTerm(1.0, [(x, 0.5), (y, 1.5)])
        signomial_terms = SHOTpy.SignomialTerms()
        signomial_terms.add(signomial)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(signomial_terms)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # At (4, 1): x^0.5 * y^1.5 = 2 * 1 = 2
        # d/dx = 0.5 * x^(-0.5) * y^1.5 = 0.5 * 0.5 * 1 = 0.25
        # d/dy = 1.5 * x^0.5 * y^0.5 = 1.5 * 2 * 1 = 3.0
        gradient = problem.objectiveFunction.calculateGradient([4.0, 1.0])
        assert abs(gradient[0] - 0.25) < 1e-10
        assert abs(gradient[1] - 3.0) < 1e-10


class TestNonlinearExpressions:
    """Tests for general nonlinear expressions (sin, cos, exp, log, etc.)."""

    def test_exponential_objective(self, problem):
        """Test exponential exp(x) in objective."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        problem.addVariable(x)
        
        # Create exp(x)
        import math
        expr = SHOTpy.exp(x)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # Gradient of exp(x) at x=1 is exp(1)
        gradient = problem.objectiveFunction.calculateGradient([1.0])
        assert abs(gradient[0] - math.exp(1.0)) < 1e-10
        
        # Hessian of exp(x) at x=1 is exp(1)
        hessian = problem.objectiveFunction.calculateHessian([1.0])
        assert (0, 0) in hessian
        assert abs(hessian[(0, 0)] - math.exp(1.0)) < 1e-10

    def test_logarithm_constraint(self, problem):
        """Test logarithm log(x) in constraint."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        
        # Create log(x)
        import math
        expr = SHOTpy.log(x)
        
        constr = SHOTpy.NonlinearConstraint(0, "log_constr", expr, -1e20, 10.0)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        # Gradient of log(x) at x=2 is 1/x = 0.5
        constr_ref = problem.getConstraint(0)
        gradient = constr_ref.calculateGradient([2.0])
        assert abs(gradient[0] - 0.5) < 1e-10
        
        # Hessian of log(x) at x=2 is -1/x^2 = -0.25
        hessian = constr_ref.calculateHessian([2.0])
        assert (0, 0) in hessian
        assert abs(hessian[(0, 0)] - (-0.25)) < 1e-10

    def test_sin_cos_objective(self, problem):
        """Test sin(x) + cos(y) in objective."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, -10.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # Create sin(x) + cos(y)
        import math
        expr = SHOTpy.sin(x) + SHOTpy.cos(y)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # Gradient at (0, 0): d/dx(sin(x)) = cos(0) = 1, d/dy(cos(y)) = -sin(0) = 0
        gradient = problem.objectiveFunction.calculateGradient([0.0, 0.0])
        assert abs(gradient[0] - 1.0) < 1e-10
        assert abs(gradient.get(1, 0.0) - 0.0) < 1e-10

    def test_power_expression(self, problem):
        """Test x^3 created as power expression."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        
        # Create x^3
        expr = x ** 3
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # Gradient of x^3 at x=2: 3*x^2 = 12
        gradient = problem.objectiveFunction.calculateGradient([2.0])
        assert abs(gradient[0] - 12.0) < 1e-10
        
        # Hessian of x^3 at x=2: 6*x = 12
        hessian = problem.objectiveFunction.calculateHessian([2.0])
        assert (0, 0) in hessian
        assert abs(hessian[(0, 0)] - 12.0) < 1e-10

    def test_composite_expression(self, problem):
        """Test composite expression exp(x) * y^2."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -5.0, 5.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # Create exp(x) * y^2
        import math
        expr = SHOTpy.exp(x) * (y ** 2)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # At (0, 2): exp(0) * 4 = 4
        # d/dx = exp(x) * y^2 = 1 * 4 = 4
        # d/dy = exp(x) * 2*y = 1 * 4 = 4
        gradient = problem.objectiveFunction.calculateGradient([0.0, 2.0])
        assert abs(gradient[0] - 4.0) < 1e-10
        assert abs(gradient[1] - 4.0) < 1e-10


class TestMixedTermTypes:
    """Tests for problems with multiple term types (linear, quadratic, monomial, signomial, nonlinear)."""

    def test_all_term_types_objective(self, problem):
        """Test objective with linear + quadratic + monomial terms."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        # Objective: 2*x + 3*y^2 + x*y*z
        linear_terms = SHOTpy.LinearTerms()
        linear_terms.add(SHOTpy.LinearTerm(2.0, x))
        
        quad_terms = SHOTpy.QuadraticTerms()
        quad_terms.add(SHOTpy.QuadraticTerm(3.0, y, y))
        
        monomial = SHOTpy.MonomialTerm(1.0, [x, y, z])
        monomial_terms = SHOTpy.MonomialTerms()
        monomial_terms.add(monomial)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(linear_terms)
        obj.add(quad_terms)
        obj.add(monomial_terms)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # At (1, 2, 3):
        # Value = 2*1 + 3*4 + 1*2*3 = 2 + 12 + 6 = 20
        # Gradient:
        # d/dx = 2 + y*z = 2 + 6 = 8
        # d/dy = 6*y + x*z = 12 + 3 = 15
        # d/dz = x*y = 2
        gradient = problem.objectiveFunction.calculateGradient([1.0, 2.0, 3.0])
        assert abs(gradient[0] - 8.0) < 1e-10
        assert abs(gradient[1] - 15.0) < 1e-10
        assert abs(gradient[2] - 2.0) < 1e-10

    def test_all_term_types_constraint(self, problem):
        """Test constraint with linear + quadratic + signomial terms."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # Constraint: x + y^2 + x^0.5 <= 20
        linear_terms = SHOTpy.LinearTerms()
        linear_terms.add(SHOTpy.LinearTerm(1.0, x))
        
        quad_terms = SHOTpy.QuadraticTerms()
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, y, y))
        
        # Use the alternative SignomialTerm constructor with (variable, power) pairs
        signomial = SHOTpy.SignomialTerm(1.0, [(x, 0.5)])
        signomial_terms = SHOTpy.SignomialTerms()
        signomial_terms.add(signomial)
        
        constr = SHOTpy.NonlinearConstraint(0, "mixed_constr", -1e20, 20.0)
        constr.add(linear_terms)
        constr.add(quad_terms)
        constr.add(signomial_terms)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        # At (4, 2):
        # Value = 4 + 4 + 2 = 10
        # Gradient:
        # d/dx = 1 + 0.5*x^(-0.5) = 1 + 0.25 = 1.25
        # d/dy = 2*y = 4
        constr_ref = problem.getConstraint(0)
        gradient = constr_ref.calculateGradient([4.0, 2.0])
        assert abs(gradient[0] - 1.25) < 1e-10
        assert abs(gradient[1] - 4.0) < 1e-10

    def test_multiple_constraints_different_types(self, problem):
        """Test problem with linear, quadratic, and nonlinear constraints."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # Linear constraint: 2*x + y <= 10
        lin_terms = SHOTpy.LinearTerms()
        lin_terms.add(SHOTpy.LinearTerm(2.0, x))
        lin_terms.add(SHOTpy.LinearTerm(1.0, y))
        lin_constr = SHOTpy.LinearConstraint(0, "lin_constr", lin_terms, -1e20, 10.0)
        problem.addConstraint(lin_constr)
        
        # Quadratic constraint: x^2 + y^2 <= 25
        quad_terms = SHOTpy.QuadraticTerms()
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, x, x))
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, y, y))
        quad_constr = SHOTpy.QuadraticConstraint(1, "quad_constr", -1e20, 25.0)
        quad_constr.add(quad_terms)
        problem.addConstraint(quad_constr)
        
        # Nonlinear constraint: x^3 <= 27
        expr = x ** 3
        nl_constr = SHOTpy.NonlinearConstraint(2, "nl_constr", expr, -1e20, 27.0)
        problem.addConstraint(nl_constr)
        
        # Linear objective
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        obj.add(SHOTpy.LinearTerm(1.0, y))
        problem.setObjective(obj)
        
        problem.finalize()
        
        # Check Jacobian pattern has all three original constraints
        jacobian_pattern = problem.getConstraintsJacobianSparsityPattern()
        constraint_indices = set(p[0] for p in jacobian_pattern)
        assert 0 in constraint_indices  # Linear
        assert 1 in constraint_indices  # Quadratic
        assert 2 in constraint_indices  # Nonlinear
        
        # Check Hessian pattern (only quad and nonlinear have second derivatives)
        hessian_pattern = problem.getConstraintsHessianSparsityPattern()
        pattern_set = set(hessian_pattern)
        # Quadratic: (0,0), (1,1)
        assert (0, 0) in pattern_set
        assert (1, 1) in pattern_set

    def test_nonlinear_expression_and_terms(self, problem):
        """Test objective with both nonlinear expression and extracted terms."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # Objective: x^2 + exp(y)
        # x^2 will be extracted as quadratic term
        # exp(y) stays as nonlinear expression
        import math
        expr = (x ** 2) + SHOTpy.exp(y)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # Hessian should have (0,0) from x^2 and (1,1) from exp(y)
        hessian_pattern = problem.getLagrangianHessianSparsityPattern()
        pattern_set = set(hessian_pattern)
        assert (0, 0) in pattern_set
        assert (1, 1) in pattern_set
        
        # Gradient at (2, 0):
        # d/dx = 2*x = 4
        # d/dy = exp(y) = 1
        gradient = problem.objectiveFunction.calculateGradient([2.0, 0.0])
        assert abs(gradient[0] - 4.0) < 1e-10
        assert abs(gradient[1] - 1.0) < 1e-10
        
        # Hessian at (2, 0):
        # d^2/dx^2 = 2
        # d^2/dy^2 = exp(0) = 1
        hessian = problem.objectiveFunction.calculateHessian([2.0, 0.0])
        assert abs(hessian[(0, 0)] - 2.0) < 1e-10
        assert abs(hessian[(1, 1)] - 1.0) < 1e-10
