"""
Tests comparing nonlinear expression gradients/Hessians when in objective vs constraint.

This test module verifies that the gradient and Hessian computation works identically
whether a nonlinear expression (like exp(x)) is in the objective function or in a constraint.
"""

import pytest
import math


class TestExpObjectiveVsConstraint:
    """Compare exp(x) behavior in objective vs constraint."""

    def test_exp_in_objective_gradient(self, problem):
        """Test gradient of exp(x) when in objective function."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        problem.addVariable(x)
        
        # Create exp(x) as objective
        expr = SHOTpy.exp(x)
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # Test at multiple points
        test_points = [0.0, 1.0, 2.0, -1.0]
        for pt in test_points:
            gradient = problem.objectiveFunction.calculateGradient([pt])
            expected = math.exp(pt)
            assert abs(gradient.get(0, 0.0) - expected) < 1e-8, \
                f"At x={pt}: expected gradient {expected}, got {gradient.get(0, 0.0)}"

    def test_exp_in_constraint_gradient(self, problem):
        """Test gradient of exp(x) when in constraint."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        problem.addVariable(x)
        
        # Create exp(x) <= 100 as constraint
        expr = SHOTpy.exp(x)
        constr = SHOTpy.NonlinearConstraint(0, "exp_constr", expr, -1e20, 100.0)
        problem.addConstraint(constr)
        
        # Need an objective
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        # Test at multiple points
        test_points = [0.0, 1.0, 2.0, -1.0]
        constr_ref = problem.getConstraint(0)
        for pt in test_points:
            gradient = constr_ref.calculateGradient([pt])
            expected = math.exp(pt)
            assert abs(gradient.get(0, 0.0) - expected) < 1e-8, \
                f"At x={pt}: expected gradient {expected}, got {gradient.get(0, 0.0)}"

    def test_exp_objective_vs_constraint_same_gradient(self, shot_context):
        """Compare gradient values - objective and constraint should give same result."""
        import SHOTpy
        
        # Create problem with exp(x) in objective
        problem_obj = shot_context.problem
        x_obj = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        problem_obj.addVariable(x_obj)
        
        expr_obj = SHOTpy.exp(x_obj)
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr_obj)
        problem_obj.setObjective(obj)
        problem_obj.finalize()
        
        # Create a separate problem with exp(x) in constraint
        ctx2 = SHOTpy.Solver()
        env2 = ctx2.getEnvironment()
        problem_constr = SHOTpy.Problem(env2)
        x_constr = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        problem_constr.addVariable(x_constr)
        
        expr_constr = SHOTpy.exp(x_constr)
        constr = SHOTpy.NonlinearConstraint(0, "exp_constr", expr_constr, -1e20, 100.0)
        problem_constr.addConstraint(constr)
        
        obj_linear = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj_linear.add(SHOTpy.LinearTerm(1.0, x_constr))
        problem_constr.setObjective(obj_linear)
        problem_constr.finalize()
        
        # Compare gradients at multiple points
        test_points = [0.0, 1.0, 2.0, -1.0, 0.5]
        for pt in test_points:
            grad_obj = problem_obj.objectiveFunction.calculateGradient([pt])
            constr_ref = problem_constr.getConstraint(0)
            grad_constr = constr_ref.calculateGradient([pt])
            
            val_obj = grad_obj.get(0, 0.0)
            val_constr = grad_constr.get(0, 0.0)
            
            assert abs(val_obj - val_constr) < 1e-10, \
                f"At x={pt}: objective gradient {val_obj} != constraint gradient {val_constr}"

    def test_exp_in_objective_hessian(self, problem):
        """Test Hessian of exp(x) when in objective function."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        problem.addVariable(x)
        
        expr = SHOTpy.exp(x)
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # Test at multiple points
        test_points = [0.0, 1.0, 2.0, -1.0]
        for pt in test_points:
            hessian = problem.objectiveFunction.calculateHessian([pt])
            expected = math.exp(pt)
            assert (0, 0) in hessian, f"At x={pt}: Hessian missing (0,0) entry"
            assert abs(hessian[(0, 0)] - expected) < 1e-8, \
                f"At x={pt}: expected Hessian {expected}, got {hessian[(0, 0)]}"

    def test_exp_in_constraint_hessian(self, problem):
        """Test Hessian of exp(x) when in constraint."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        problem.addVariable(x)
        
        expr = SHOTpy.exp(x)
        constr = SHOTpy.NonlinearConstraint(0, "exp_constr", expr, -1e20, 100.0)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        test_points = [0.0, 1.0, 2.0, -1.0]
        constr_ref = problem.getConstraint(0)
        for pt in test_points:
            hessian = constr_ref.calculateHessian([pt])
            expected = math.exp(pt)
            assert (0, 0) in hessian, f"At x={pt}: Hessian missing (0,0) entry"
            assert abs(hessian[(0, 0)] - expected) < 1e-8, \
                f"At x={pt}: expected Hessian {expected}, got {hessian[(0, 0)]}"

    def test_exp_objective_vs_constraint_same_hessian(self, shot_context):
        """Compare Hessian values - objective and constraint should give same result."""
        import SHOTpy
        
        # Create problem with exp(x) in objective
        problem_obj = shot_context.problem
        x_obj = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        problem_obj.addVariable(x_obj)
        
        expr_obj = SHOTpy.exp(x_obj)
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr_obj)
        problem_obj.setObjective(obj)
        problem_obj.finalize()
        
        # Create a separate problem with exp(x) in constraint
        ctx2 = SHOTpy.Solver()
        env2 = ctx2.getEnvironment()
        problem_constr = SHOTpy.Problem(env2)
        x_constr = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        problem_constr.addVariable(x_constr)
        
        expr_constr = SHOTpy.exp(x_constr)
        constr = SHOTpy.NonlinearConstraint(0, "exp_constr", expr_constr, -1e20, 100.0)
        problem_constr.addConstraint(constr)
        
        obj_linear = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj_linear.add(SHOTpy.LinearTerm(1.0, x_constr))
        problem_constr.setObjective(obj_linear)
        problem_constr.finalize()
        
        # Compare Hessians at multiple points
        test_points = [0.0, 1.0, 2.0, -1.0, 0.5]
        for pt in test_points:
            hess_obj = problem_obj.objectiveFunction.calculateHessian([pt])
            constr_ref = problem_constr.getConstraint(0)
            hess_constr = constr_ref.calculateHessian([pt])
            
            val_obj = hess_obj.get((0, 0), 0.0)
            val_constr = hess_constr.get((0, 0), 0.0)
            
            assert abs(val_obj - val_constr) < 1e-10, \
                f"At x={pt}: objective Hessian {val_obj} != constraint Hessian {val_constr}"


class TestSinCosObjectiveVsConstraint:
    """Compare sin/cos expressions in objective vs constraint."""

    def test_sin_in_objective_gradient(self, problem):
        """Test gradient of sin(x) when in objective function."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        problem.addVariable(x)
        
        expr = SHOTpy.sin(x)
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # d/dx sin(x) = cos(x)
        test_points = [0.0, math.pi/4, math.pi/2, math.pi]
        for pt in test_points:
            gradient = problem.objectiveFunction.calculateGradient([pt])
            expected = math.cos(pt)
            assert abs(gradient.get(0, 0.0) - expected) < 1e-8, \
                f"At x={pt}: expected gradient {expected}, got {gradient.get(0, 0.0)}"

    def test_sin_in_constraint_gradient(self, problem):
        """Test gradient of sin(x) when in constraint."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        problem.addVariable(x)
        
        expr = SHOTpy.sin(x)
        constr = SHOTpy.NonlinearConstraint(0, "sin_constr", expr, -1e20, 1.0)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        test_points = [0.0, math.pi/4, math.pi/2, math.pi]
        constr_ref = problem.getConstraint(0)
        for pt in test_points:
            gradient = constr_ref.calculateGradient([pt])
            expected = math.cos(pt)
            assert abs(gradient.get(0, 0.0) - expected) < 1e-8, \
                f"At x={pt}: expected gradient {expected}, got {gradient.get(0, 0.0)}"

    def test_sin_in_objective_hessian(self, problem):
        """Test Hessian of sin(x) when in objective - should be -sin(x)."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        problem.addVariable(x)
        
        expr = SHOTpy.sin(x)
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # d^2/dx^2 sin(x) = -sin(x)
        test_points = [math.pi/4, math.pi/2, math.pi]  # Skip 0 where Hessian is 0
        for pt in test_points:
            hessian = problem.objectiveFunction.calculateHessian([pt])
            expected = -math.sin(pt)
            if abs(expected) > 1e-10:  # Only check if expected is non-zero
                assert (0, 0) in hessian, f"At x={pt}: Hessian missing (0,0) entry"
                assert abs(hessian[(0, 0)] - expected) < 1e-8, \
                    f"At x={pt}: expected Hessian {expected}, got {hessian.get((0, 0), 0.0)}"

    def test_sin_in_constraint_hessian(self, problem):
        """Test Hessian of sin(x) when in constraint - should be -sin(x)."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        problem.addVariable(x)
        
        expr = SHOTpy.sin(x)
        constr = SHOTpy.NonlinearConstraint(0, "sin_constr", expr, -1e20, 1.0)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        test_points = [math.pi/4, math.pi/2, math.pi]
        constr_ref = problem.getConstraint(0)
        for pt in test_points:
            hessian = constr_ref.calculateHessian([pt])
            expected = -math.sin(pt)
            if abs(expected) > 1e-10:
                assert (0, 0) in hessian, f"At x={pt}: Hessian missing (0,0) entry"
                assert abs(hessian[(0, 0)] - expected) < 1e-8, \
                    f"At x={pt}: expected Hessian {expected}, got {hessian.get((0, 0), 0.0)}"


class TestLogObjectiveVsConstraint:
    """Compare log(x) in objective vs constraint."""

    def test_log_in_objective_gradient(self, problem):
        """Test gradient of log(x) when in objective function."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        
        expr = SHOTpy.log(x)
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # d/dx log(x) = 1/x
        test_points = [0.5, 1.0, 2.0, 5.0]
        for pt in test_points:
            gradient = problem.objectiveFunction.calculateGradient([pt])
            expected = 1.0 / pt
            assert abs(gradient.get(0, 0.0) - expected) < 1e-8, \
                f"At x={pt}: expected gradient {expected}, got {gradient.get(0, 0.0)}"

    def test_log_in_constraint_gradient(self, problem):
        """Test gradient of log(x) when in constraint."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        
        expr = SHOTpy.log(x)
        constr = SHOTpy.NonlinearConstraint(0, "log_constr", expr, -1e20, 10.0)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        test_points = [0.5, 1.0, 2.0, 5.0]
        constr_ref = problem.getConstraint(0)
        for pt in test_points:
            gradient = constr_ref.calculateGradient([pt])
            expected = 1.0 / pt
            assert abs(gradient.get(0, 0.0) - expected) < 1e-8, \
                f"At x={pt}: expected gradient {expected}, got {gradient.get(0, 0.0)}"

    def test_log_in_objective_hessian(self, problem):
        """Test Hessian of log(x) when in objective - should be -1/x^2."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        
        expr = SHOTpy.log(x)
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # d^2/dx^2 log(x) = -1/x^2
        test_points = [0.5, 1.0, 2.0, 5.0]
        for pt in test_points:
            hessian = problem.objectiveFunction.calculateHessian([pt])
            expected = -1.0 / (pt * pt)
            assert (0, 0) in hessian, f"At x={pt}: Hessian missing (0,0) entry"
            assert abs(hessian[(0, 0)] - expected) < 1e-8, \
                f"At x={pt}: expected Hessian {expected}, got {hessian[(0, 0)]}"

    def test_log_in_constraint_hessian(self, problem):
        """Test Hessian of log(x) when in constraint - should be -1/x^2."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        
        expr = SHOTpy.log(x)
        constr = SHOTpy.NonlinearConstraint(0, "log_constr", expr, -1e20, 10.0)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        test_points = [0.5, 1.0, 2.0, 5.0]
        constr_ref = problem.getConstraint(0)
        for pt in test_points:
            hessian = constr_ref.calculateHessian([pt])
            expected = -1.0 / (pt * pt)
            assert (0, 0) in hessian, f"At x={pt}: Hessian missing (0,0) entry"
            assert abs(hessian[(0, 0)] - expected) < 1e-8, \
                f"At x={pt}: expected Hessian {expected}, got {hessian[(0, 0)]}"


class TestMultiVariableNonlinear:
    """Test nonlinear expressions with multiple variables."""

    def test_exp_x_times_y_objective_gradient(self, problem):
        """Test gradient of exp(x) * y in objective."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -5.0, 5.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # exp(x) * y
        expr = SHOTpy.exp(x) * y
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # At (1, 2): f = e * 2
        # d/dx = exp(x) * y = e * 2
        # d/dy = exp(x) = e
        pt = [1.0, 2.0]
        gradient = problem.objectiveFunction.calculateGradient(pt)
        
        expected_dx = math.exp(1.0) * 2.0
        expected_dy = math.exp(1.0)
        
        assert abs(gradient.get(0, 0.0) - expected_dx) < 1e-8, \
            f"Expected d/dx = {expected_dx}, got {gradient.get(0, 0.0)}"
        assert abs(gradient.get(1, 0.0) - expected_dy) < 1e-8, \
            f"Expected d/dy = {expected_dy}, got {gradient.get(1, 0.0)}"

    def test_exp_x_times_y_constraint_gradient(self, problem):
        """Test gradient of exp(x) * y in constraint."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -5.0, 5.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        expr = SHOTpy.exp(x) * y
        constr = SHOTpy.NonlinearConstraint(0, "nl_constr", expr, -1e20, 100.0)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        pt = [1.0, 2.0]
        constr_ref = problem.getConstraint(0)
        gradient = constr_ref.calculateGradient(pt)
        
        expected_dx = math.exp(1.0) * 2.0
        expected_dy = math.exp(1.0)
        
        assert abs(gradient.get(0, 0.0) - expected_dx) < 1e-8, \
            f"Expected d/dx = {expected_dx}, got {gradient.get(0, 0.0)}"
        assert abs(gradient.get(1, 0.0) - expected_dy) < 1e-8, \
            f"Expected d/dy = {expected_dy}, got {gradient.get(1, 0.0)}"

    def test_exp_x_times_y_objective_hessian(self, problem):
        """Test Hessian of exp(x) * y in objective."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -5.0, 5.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        expr = SHOTpy.exp(x) * y
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        
        # f = exp(x) * y
        # d^2/dx^2 = exp(x) * y
        # d^2/dxdy = exp(x)
        # d^2/dy^2 = 0
        pt = [1.0, 2.0]
        hessian = problem.objectiveFunction.calculateHessian(pt)
        
        expected_dxdx = math.exp(1.0) * 2.0
        expected_dxdy = math.exp(1.0)
        
        assert (0, 0) in hessian, "Hessian missing (0,0) entry"
        assert abs(hessian[(0, 0)] - expected_dxdx) < 1e-8, \
            f"Expected d^2/dx^2 = {expected_dxdx}, got {hessian[(0, 0)]}"
        
        assert (0, 1) in hessian, "Hessian missing (0,1) entry"
        assert abs(hessian[(0, 1)] - expected_dxdy) < 1e-8, \
            f"Expected d^2/dxdy = {expected_dxdy}, got {hessian[(0, 1)]}"

    def test_exp_x_times_y_constraint_hessian(self, problem):
        """Test Hessian of exp(x) * y in constraint."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -5.0, 5.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        expr = SHOTpy.exp(x) * y
        constr = SHOTpy.NonlinearConstraint(0, "nl_constr", expr, -1e20, 100.0)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        pt = [1.0, 2.0]
        constr_ref = problem.getConstraint(0)
        hessian = constr_ref.calculateHessian(pt)
        
        expected_dxdx = math.exp(1.0) * 2.0
        expected_dxdy = math.exp(1.0)
        
        assert (0, 0) in hessian, "Hessian missing (0,0) entry"
        assert abs(hessian[(0, 0)] - expected_dxdx) < 1e-8, \
            f"Expected d^2/dx^2 = {expected_dxdx}, got {hessian[(0, 0)]}"
        
        assert (0, 1) in hessian, "Hessian missing (0,1) entry"
        assert abs(hessian[(0, 1)] - expected_dxdy) < 1e-8, \
            f"Expected d^2/dxdy = {expected_dxdy}, got {hessian[(0, 1)]}"


class TestSparsityPatterns:
    """Test that sparsity patterns are identical for objective vs constraint."""

    def test_exp_gradient_sparsity_objective(self, problem):
        """Test gradient sparsity pattern for exp(x) in objective."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, -10.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # exp(x) - only depends on x
        expr = SHOTpy.exp(x)
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        problem.finalize()
        
        gradient = problem.objectiveFunction.calculateGradient([1.0, 1.0])
        # Should only have entry for x (index 0), not y (index 1)
        assert 0 in gradient
        assert gradient.get(1, 0.0) == 0.0  # y should have zero gradient

    def test_exp_gradient_sparsity_constraint(self, problem):
        """Test gradient sparsity pattern for exp(x) in constraint."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, -10.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        expr = SHOTpy.exp(x)
        constr = SHOTpy.NonlinearConstraint(0, "exp_constr", expr, -1e20, 100.0)
        problem.addConstraint(constr)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        problem.finalize()
        
        constr_ref = problem.getConstraint(0)
        gradient = constr_ref.calculateGradient([1.0, 1.0])
        # Should only have entry for x (index 0), not y (index 1)
        assert 0 in gradient
        assert gradient.get(1, 0.0) == 0.0

    def test_hessian_sparsity_lagrangian(self, problem):
        """Test Lagrangian Hessian sparsity includes both objective and constraint."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # Objective: exp(x)
        expr_obj = SHOTpy.exp(x)
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr_obj)
        problem.setObjective(obj)
        
        # Constraint: log(y) <= 10
        expr_constr = SHOTpy.log(y)
        constr = SHOTpy.NonlinearConstraint(0, "log_constr", expr_constr, -1e20, 10.0)
        problem.addConstraint(constr)
        
        problem.finalize()
        
        # Lagrangian Hessian should have (0,0) from exp(x) and (1,1) from log(y)
        lagrangian_hess = problem.getLagrangianHessianSparsityPattern()
        pattern_set = set(lagrangian_hess)
        
        assert (0, 0) in pattern_set, "Expected (0,0) in Lagrangian Hessian from exp(x)"
        assert (1, 1) in pattern_set, "Expected (1,1) in Lagrangian Hessian from log(y)"
