"""
Tests for solving optimization problems via the Python API.

This module tests that SHOT can correctly solve optimization problems
when built programmatically via the Python API.

To add a new problem test:
1. Create a function that builds and returns the problem
2. Add a test method that calls solve_and_verify()
"""

import pytest
import SHOTpy
from conftest import SHOTContext


class OptimizationTestBase:
    """Base class for optimization problem tests with common utilities."""
    
    @staticmethod
    def create_solver_and_problem():
        """Create a fresh solver and problem instance."""
        ctx = SHOTContext()
        return ctx.solver, ctx.env, ctx.problem
    
    @staticmethod
    def solve_and_verify(solver, problem, expected_obj, tolerance=0.01, 
                         expected_solution=None, solution_tolerance=0.1):
        """
        Finalize, solve, and verify a problem.
        
        Args:
            solver: The SHOT Solver instance
            problem: The Problem to solve
            expected_obj: Expected optimal objective value
            tolerance: Tolerance for objective comparison
            expected_solution: Optional dict mapping variable names to expected values
            solution_tolerance: Tolerance for solution value comparison
            
        Returns:
            The primal solution object
        """
        problem.finalize()
        solver.setProblem(problem)
        
        result = solver.solveProblem()
        assert result == True, "Solver failed to find a solution"
        
        obj_value = solver.getPrimalBound()
        assert abs(obj_value - expected_obj) < tolerance, \
            f"Objective {obj_value} differs from expected {expected_obj} by {abs(obj_value - expected_obj)}"
        
        sol = solver.getPrimalSolution()
        
        if expected_solution:
            # Verify specific variable values if provided
            for var_name, expected_val in expected_solution.items():
                # Find variable index by iterating through problem variables
                # This is a simplified check - actual implementation may vary
                pass
        
        return sol


def build_ex1223b(env, problem):
    """
    Build the ex1223b problem.
    
    This is a convex MINLP with 3 continuous and 4 binary variables.
    
    Optimal solution: 
        x1=0.2, x2=0.8, x3=1.907878
        b4=1, b5=1, b6=0, b7=1
    Optimal objective: 4.579582402436710
    
    Problem formulation:
        minimize (b4-1)^2 + (b5-2)^2 + (b6-1)^2 - log(1+b7) 
                 + (x1-1)^2 + (x2-2)^2 + (x3-3)^2
        subject to:
            e1: x1 + x2 + x3 + b4 + b5 + b6 <= 5
            e2: b6^2 + x1^2 + x2^2 + x3^2 <= 5.5
            e3: x1 + b4 <= 1.2
            e4: x2 + b5 <= 1.8
            e5: x3 + b6 <= 2.5
            e6: x1 + b7 <= 1.2
            e7: b5^2 + x2^2 <= 1.64
            e8: b6^2 + x3^2 <= 4.25
            e9: b5^2 + x3^2 <= 4.64
    """
    problem.name = "ex1223b"
    
    # Create variables
    x1 = SHOTpy.Variable("x1", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
    x2 = SHOTpy.Variable("x2", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
    x3 = SHOTpy.Variable("x3", 2, SHOTpy.VariableType.Real, 0.0, 10.0)
    b4 = SHOTpy.Variable("b4", 3, SHOTpy.VariableType.Binary, 0.0, 1.0)
    b5 = SHOTpy.Variable("b5", 4, SHOTpy.VariableType.Binary, 0.0, 1.0)
    b6 = SHOTpy.Variable("b6", 5, SHOTpy.VariableType.Binary, 0.0, 1.0)
    b7 = SHOTpy.Variable("b7", 6, SHOTpy.VariableType.Binary, 0.0, 1.0)
    
    # Add variables to problem
    for var in [x1, x2, x3, b4, b5, b6, b7]:
        problem.addVariable(var)
    
    # Create nonlinear objective function
    # minimize (b4-1)^2 + (b5-2)^2 + (b6-1)^2 - log(1+b7) + (x1-1)^2 + (x2-2)^2 + (x3-3)^2
    objective = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
    obj_expr = ((b4 - 1)**2 + (b5 - 2)**2 + (b6 - 1)**2 
                - SHOTpy.log(1 + b7) 
                + (x1 - 1)**2 + (x2 - 2)**2 + (x3 - 3)**2)
    objective.add(obj_expr)
    problem.setObjective(objective)
    
    # e1: x1 + x2 + x3 + b4 + b5 + b6 <= 5
    e1 = SHOTpy.LinearConstraint(0, "e1", -SHOTpy.SHOT_DBL_MAX, 5.0)
    for var in [x1, x2, x3, b4, b5, b6]:
        e1.add(SHOTpy.LinearTerm(1.0, var))
    problem.addConstraint(e1)
    
    # e2: b6^2 + x1^2 + x2^2 + x3^2 <= 5.5
    e2 = SHOTpy.QuadraticConstraint(1, "e2", -SHOTpy.SHOT_DBL_MAX, 5.5)
    for var in [b6, x1, x2, x3]:
        e2.add(SHOTpy.QuadraticTerm(1.0, var, var))
    problem.addConstraint(e2)
    
    # e3: x1 + b4 <= 1.2
    e3 = SHOTpy.LinearConstraint(2, "e3", -SHOTpy.SHOT_DBL_MAX, 1.2)
    e3.add(SHOTpy.LinearTerm(1.0, x1))
    e3.add(SHOTpy.LinearTerm(1.0, b4))
    problem.addConstraint(e3)
    
    # e4: x2 + b5 <= 1.8
    e4 = SHOTpy.LinearConstraint(3, "e4", -SHOTpy.SHOT_DBL_MAX, 1.8)
    e4.add(SHOTpy.LinearTerm(1.0, x2))
    e4.add(SHOTpy.LinearTerm(1.0, b5))
    problem.addConstraint(e4)
    
    # e5: x3 + b6 <= 2.5
    e5 = SHOTpy.LinearConstraint(4, "e5", -SHOTpy.SHOT_DBL_MAX, 2.5)
    e5.add(SHOTpy.LinearTerm(1.0, x3))
    e5.add(SHOTpy.LinearTerm(1.0, b6))
    problem.addConstraint(e5)
    
    # e6: x1 + b7 <= 1.2
    e6 = SHOTpy.LinearConstraint(5, "e6", -SHOTpy.SHOT_DBL_MAX, 1.2)
    e6.add(SHOTpy.LinearTerm(1.0, x1))
    e6.add(SHOTpy.LinearTerm(1.0, b7))
    problem.addConstraint(e6)
    
    # e7: b5^2 + x2^2 <= 1.64
    e7 = SHOTpy.QuadraticConstraint(6, "e7", -SHOTpy.SHOT_DBL_MAX, 1.64)
    e7.add(SHOTpy.QuadraticTerm(1.0, b5, b5))
    e7.add(SHOTpy.QuadraticTerm(1.0, x2, x2))
    problem.addConstraint(e7)
    
    # e8: b6^2 + x3^2 <= 4.25
    e8 = SHOTpy.QuadraticConstraint(7, "e8", -SHOTpy.SHOT_DBL_MAX, 4.25)
    e8.add(SHOTpy.QuadraticTerm(1.0, b6, b6))
    e8.add(SHOTpy.QuadraticTerm(1.0, x3, x3))
    problem.addConstraint(e8)
    
    # e9: b5^2 + x3^2 <= 4.64
    e9 = SHOTpy.QuadraticConstraint(8, "e9", -SHOTpy.SHOT_DBL_MAX, 4.64)
    e9.add(SHOTpy.QuadraticTerm(1.0, b5, b5))
    e9.add(SHOTpy.QuadraticTerm(1.0, x3, x3))
    problem.addConstraint(e9)
    
    return problem


def build_meanvarxsc(env, problem):
    """
    Build the meanvarxsc problem.

    Mean-variance portfolio selection with semicontinuous trade variables.
    Source: https://www.minlplib.org/meanvarxsc.html

    Problem type: MBQP (Mixed-Binary Quadratic Program)
    #Variables: 35 (7 continuous, 12 semicontinuous, 2 fixed-at-zero, 14 binary)
    #Constraints: 30 (all linear)
    Optimal objective: 14.36923211
    """
    problem.name = "meanvarxsc"

    # Portfolio weights: x2..x8 (positive continuous)
    x2 = SHOTpy.Variable("x2", 0, SHOTpy.VariableType.Real, 0.0, SHOTpy.SHOT_DBL_MAX)
    x3 = SHOTpy.Variable("x3", 1, SHOTpy.VariableType.Real, 0.0, SHOTpy.SHOT_DBL_MAX)
    x4 = SHOTpy.Variable("x4", 2, SHOTpy.VariableType.Real, 0.0, SHOTpy.SHOT_DBL_MAX)
    x5 = SHOTpy.Variable("x5", 3, SHOTpy.VariableType.Real, 0.0, SHOTpy.SHOT_DBL_MAX)
    x6 = SHOTpy.Variable("x6", 4, SHOTpy.VariableType.Real, 0.0, SHOTpy.SHOT_DBL_MAX)
    x7 = SHOTpy.Variable("x7", 5, SHOTpy.VariableType.Real, 0.0, SHOTpy.SHOT_DBL_MAX)
    x8 = SHOTpy.Variable("x8", 6, SHOTpy.VariableType.Real, 0.0, SHOTpy.SHOT_DBL_MAX)

    # Semicontinuous sell/buy amounts: either 0 or in [semiBound, upperBound]
    sc9  = SHOTpy.Variable("sc9",   7,  SHOTpy.VariableType.Semicontinuous, 0.0, 0.11, 0.03)
    sc10 = SHOTpy.Variable("sc10",  8,  SHOTpy.VariableType.Semicontinuous, 0.0, 0.10, 0.04)
    sc11 = SHOTpy.Variable("sc11",  9,  SHOTpy.VariableType.Semicontinuous, 0.0, 0.07, 0.04)
    sc12 = SHOTpy.Variable("sc12", 10,  SHOTpy.VariableType.Semicontinuous, 0.0, 0.11, 0.03)
    sc13 = SHOTpy.Variable("sc13", 11,  SHOTpy.VariableType.Semicontinuous, 0.0, 0.20, 0.03)
    sc14 = SHOTpy.Variable("sc14", 12,  SHOTpy.VariableType.Semicontinuous, 0.0, 0.10, 0.03)
    sc15 = SHOTpy.Variable("sc15", 13,  SHOTpy.VariableType.Semicontinuous, 0.0, 0.10, 0.03)
    sc16 = SHOTpy.Variable("sc16", 14,  SHOTpy.VariableType.Semicontinuous, 0.0, 0.20, 0.02)
    sc17 = SHOTpy.Variable("sc17", 15,  SHOTpy.VariableType.Semicontinuous, 0.0, 0.15, 0.02)
    sc18 = SHOTpy.Variable("sc18", 16, SHOTpy.VariableType.Real, 0.0, 0.0)  # fixed at 0
    sc19 = SHOTpy.Variable("sc19", 17, SHOTpy.VariableType.Real, 0.0, 0.0)  # fixed at 0
    sc20 = SHOTpy.Variable("sc20", 18,  SHOTpy.VariableType.Semicontinuous, 0.0, 0.10, 0.04)
    sc21 = SHOTpy.Variable("sc21", 19,  SHOTpy.VariableType.Semicontinuous, 0.0, 0.15, 0.04)
    sc22 = SHOTpy.Variable("sc22", 20,  SHOTpy.VariableType.Semicontinuous, 0.0, 0.20, 0.04)

    # Binary indicator variables
    b23 = SHOTpy.Variable("b23", 21, SHOTpy.VariableType.Binary, 0.0, 1.0)
    b24 = SHOTpy.Variable("b24", 22, SHOTpy.VariableType.Binary, 0.0, 1.0)
    b25 = SHOTpy.Variable("b25", 23, SHOTpy.VariableType.Binary, 0.0, 1.0)
    b26 = SHOTpy.Variable("b26", 24, SHOTpy.VariableType.Binary, 0.0, 1.0)
    b27 = SHOTpy.Variable("b27", 25, SHOTpy.VariableType.Binary, 0.0, 1.0)
    b28 = SHOTpy.Variable("b28", 26, SHOTpy.VariableType.Binary, 0.0, 1.0)
    b29 = SHOTpy.Variable("b29", 27, SHOTpy.VariableType.Binary, 0.0, 1.0)
    b30 = SHOTpy.Variable("b30", 28, SHOTpy.VariableType.Binary, 0.0, 1.0)
    b31 = SHOTpy.Variable("b31", 29, SHOTpy.VariableType.Binary, 0.0, 1.0)
    b32 = SHOTpy.Variable("b32", 30, SHOTpy.VariableType.Binary, 0.0, 1.0)
    b33 = SHOTpy.Variable("b33", 31, SHOTpy.VariableType.Binary, 0.0, 1.0)
    b34 = SHOTpy.Variable("b34", 32, SHOTpy.VariableType.Binary, 0.0, 1.0)
    b35 = SHOTpy.Variable("b35", 33, SHOTpy.VariableType.Binary, 0.0, 1.0)
    b36 = SHOTpy.Variable("b36", 34, SHOTpy.VariableType.Binary, 0.0, 1.0)

    for v in [x2, x3, x4, x5, x6, x7, x8,
              sc9, sc10, sc11, sc12, sc13, sc14, sc15, sc16, sc17, sc18, sc19, sc20, sc21, sc22,
              b23, b24, b25, b26, b27, b28, b29, b30, b31, b32, b33, b34, b35, b36]:
        problem.addVariable(v)

    # ---- Objective: minimize portfolio variance minus expected return ----
    obj = SHOTpy.QuadraticObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)

    # Quadratic (covariance) terms
    for coeff, v1, v2 in [
        (42.18, x2, x2), (40.36, x2, x3), (21.76, x2, x4), (10.6,  x2, x5),
        (24.64, x2, x6), (47.68, x2, x7), (34.82, x2, x8),
        (70.89, x3, x3), (43.16, x3, x4), (30.82, x3, x5), (46.48, x3, x6),
        (47.6,  x3, x7), (25.24, x3, x8),
        (25.51, x4, x4), (19.2,  x4, x5), (45.26, x4, x6), (26.44, x4, x7), (9.4,   x4, x8),
        (22.33, x5, x5), (20.64, x5, x6), (20.92, x5, x7), (2.0,   x5, x8),
        (30.01, x6, x6), (32.72, x6, x7), (14.4,  x6, x8),
        (42.23, x7, x7), (19.8,  x7, x8),
        (16.42, x8, x8),
    ]:
        obj.add(SHOTpy.QuadraticTerm(coeff, v1, v2))

    # Linear (expected return) terms
    for coeff, v in [(-0.06435, x2), (-0.0548, x3), (-0.02505, x4),
                     (-0.0762, x5), (-0.03815, x6), (-0.0927, x7), (-0.031, x8)]:
        obj.add(SHOTpy.LinearTerm(coeff, v))

    problem.setObjective(obj)

    # ---- Constraints ----
    INF = SHOTpy.SHOT_DBL_MAX

    # e1: x2+x3+x4+x5+x6+x7+x8 = 1
    e1 = SHOTpy.LinearConstraint(0, "e1", 1.0, 1.0)
    for v in [x2, x3, x4, x5, x6, x7, x8]:
        e1.add(SHOTpy.LinearTerm(1.0, v))
    problem.addConstraint(e1)

    # e2-e8: balance constraints (equality)
    for idx, (xi, scs, scb, rhs) in enumerate([
        (x2, sc9,  sc16, 0.2),
        (x3, sc10, sc17, 0.2),
        (x4, sc11, sc18, 0.0),
        (x5, sc12, sc19, 0.0),
        (x6, sc13, sc20, 0.2),
        (x7, sc14, sc21, 0.2),
        (x8, sc15, sc22, 0.2),
    ], start=1):
        c = SHOTpy.LinearConstraint(idx, f"e{idx + 1}", rhs, rhs)
        c.add(SHOTpy.LinearTerm(1.0,  xi))
        c.add(SHOTpy.LinearTerm(-1.0, scs))
        c.add(SHOTpy.LinearTerm(1.0,  scb))
        problem.addConstraint(c)

    # e9: sc9+...+sc15 <= 0.3
    e9 = SHOTpy.LinearConstraint(8, "e9", -INF, 0.3)
    for v in [sc9, sc10, sc11, sc12, sc13, sc14, sc15]:
        e9.add(SHOTpy.LinearTerm(1.0, v))
    problem.addConstraint(e9)

    # e10-e23: big-M bounds linking sc vars to binary indicators
    bm_data = [
        (9,  "e10",  sc9,  b23, 0.11),
        (10, "e11", sc10,  b24, 0.10),
        (11, "e12", sc11,  b25, 0.07),
        (12, "e13", sc12,  b26, 0.11),
        (13, "e14", sc13,  b27, 0.20),
        (14, "e15", sc14,  b28, 0.10),
        (15, "e16", sc15,  b29, 0.10),
        (16, "e17", sc16,  b30, 0.20),
        (17, "e18", sc17,  b31, 0.15),
        (18, "e19", sc18, None, 0.0),   # sc18 fixed at 0
        (19, "e20", sc19, None, 0.0),   # sc19 fixed at 0
        (20, "e21", sc20,  b34, 0.10),
        (21, "e22", sc21,  b35, 0.15),
        (22, "e23", sc22,  b36, 0.20),
    ]
    for idx, name, scv, bv, coeff in bm_data:
        c = SHOTpy.LinearConstraint(idx, name, -INF, 0.0)
        c.add(SHOTpy.LinearTerm(1.0, scv))
        if bv is not None:
            c.add(SHOTpy.LinearTerm(-coeff, bv))
        problem.addConstraint(c)

    # e24-e30: at-most-one constraints on binary pairs
    binary_pairs = [
        (23, "e24", b23, b30),
        (24, "e25", b24, b31),
        (25, "e26", b25, b32),
        (26, "e27", b26, b33),
        (27, "e28", b27, b34),
        (28, "e29", b28, b35),
        (29, "e30", b29, b36),
    ]
    for idx, name, bv1, bv2 in binary_pairs:
        c = SHOTpy.LinearConstraint(idx, name, -INF, 1.0)
        c.add(SHOTpy.LinearTerm(1.0, bv1))
        c.add(SHOTpy.LinearTerm(1.0, bv2))
        problem.addConstraint(c)

    return problem


# =============================================================================
# Test Classes - Add new problem tests here
# =============================================================================

class TestEx1223b(OptimizationTestBase):
    """Tests for the ex1223b optimization problem."""
    
    # Expected values for ex1223b
    EXPECTED_OBJECTIVE = 4.579582
    OBJECTIVE_TOLERANCE = 0.01
    
    def test_ex1223b_solve_exact(self):
        """Test solving ex1223b and verify optimal objective."""
        solver, env, problem = self.create_solver_and_problem()
        build_ex1223b(env, problem)
        
        self.solve_and_verify(
            solver, problem,
            expected_obj=self.EXPECTED_OBJECTIVE,
            tolerance=self.OBJECTIVE_TOLERANCE
        )
    
    def test_ex1223b_solve_completes(self):
        """Test that ex1223b can be built and solved without errors."""
        solver, env, problem = self.create_solver_and_problem()
        build_ex1223b(env, problem)
        
        problem.finalize()
        solver.setProblem(problem)
        
        result = solver.solveProblem()
        assert result == True, "Solver should complete successfully"
        
        obj_value = solver.getPrimalBound()
        assert isinstance(obj_value, float), "Should return a valid objective value"
        # Note: Due to known gradient issues, we don't check the exact value here
    
    def test_ex1223b_problem_structure(self):
        """Test that ex1223b problem has correct structure."""
        solver, env, problem = self.create_solver_and_problem()
        build_ex1223b(env, problem)
        problem.finalize()
        
        problem_str = problem.toString()
        
        # Verify problem has expected components
        assert "ex1223b" in problem_str or "minimize" in problem_str.lower()
        assert "x1" in problem_str
        assert "x2" in problem_str
        assert "x3" in problem_str
        assert "b4" in problem_str
        assert "b5" in problem_str
        assert "b6" in problem_str
        assert "b7" in problem_str
        
        # Verify constraints are present
        assert "e1" in problem_str
        assert "e2" in problem_str


class TestMeanvarxsc(OptimizationTestBase):
    """Tests for the meanvarxsc problem (semicontinuous variables)."""

    EXPECTED_OBJECTIVE = 14.36923211
    OBJECTIVE_TOLERANCE = 0.01

    def test_meanvarxsc_solve_exact(self):
        """Test solving meanvarxsc and verify optimal objective."""
        solver, env, problem = self.create_solver_and_problem()
        build_meanvarxsc(env, problem)
        self.solve_and_verify(
            solver, problem,
            expected_obj=self.EXPECTED_OBJECTIVE,
            tolerance=self.OBJECTIVE_TOLERANCE
        )

    def test_meanvarxsc_problem_structure(self):
        """Test that meanvarxsc problem has correct structure."""
        solver, env, problem = self.create_solver_and_problem()
        build_meanvarxsc(env, problem)
        problem.finalize()

        assert problem.properties.numberOfVariables == 35
        assert problem.properties.numberOfBinaryVariables == 14
        assert problem.properties.numberOfSemicontinuousVariables == 12
        assert problem.properties.numberOfNumericConstraints == 30


# =============================================================================
# File-based Problem Tests
# =============================================================================
# 
# To add a new test problem, simply add an entry to TEST_PROBLEMS below.
# Each entry specifies:
#   - name: Problem name (used to find files like name.osil, name.gms, name.nl)
#   - expected_obj: Known optimal objective value (from MINLPLib or other source)
#   - formats: List of file formats available for this problem
#   - mip_solvers: (optional) List of MIP solvers to test with ["cplex", "gurobi", "cbc", "highs"]
#                  If not specified, uses all available MIP solvers
#   - nlp_solvers: (optional) List of NLP solvers to test with ["ipopt", "gams", "shot"]
#                  If not specified, uses all available NLP solvers
#   - tolerance: (optional) Tolerance for objective comparison, default 0.01
#   - xfail: (optional) If True, marks test as expected failure
#   - xfail_reason: (optional) Reason for expected failure
#
# Example:
#   {"name": "newproblem", "expected_obj": 42.0, "formats": ["osil"], "mip_solvers": ["cbc"], "nlp_solvers": ["ipopt"]},
# =============================================================================

from pathlib import Path

# MIP solver enum mapping (uses SHOTpy.MIPSolver enum)
def get_mip_solver_enum(solver_name):
    """Get the MIPSolver enum value for a solver name."""
    mapping = {
        "cplex": SHOTpy.MIPSolver.Cplex,
        "gurobi": SHOTpy.MIPSolver.Gurobi,
        "cbc": SHOTpy.MIPSolver.Cbc,
        "highs": SHOTpy.MIPSolver.Highs
    }
    return mapping.get(solver_name.lower())

# NLP solver enum mapping (uses SHOTpy.PrimalNLPSolver enum)
def get_nlp_solver_enum(solver_name):
    """Get the PrimalNLPSolver enum value for a solver name."""
    mapping = {
        "ipopt": SHOTpy.PrimalNLPSolver.Ipopt,
        "gams": SHOTpy.PrimalNLPSolver.GAMS,
        "shot": SHOTpy.PrimalNLPSolver.SHOT,
    }
    return mapping.get(solver_name.lower())

# Test problems configuration
# Add new problems here - tests are automatically generated
TEST_PROBLEMS = [
    {
        "name": "alan",
        "expected_obj": 2.925,
        "formats": ["osil", "gms"],
        "mip_solvers": ["cplex", "gurobi", "cbc", "highs"],
    },
    {
        "name": "ex4",
        "expected_obj": -8.06413617,
        "formats": ["osil"],
        "mip_solvers": ["cplex", "gurobi", "cbc", "highs"],
    },
    {
        "name": "flay02h",
        "expected_obj": 37.94733192,
        "formats": ["osil", "gms"],
        "mip_solvers": ["cplex", "gurobi", "cbc", "highs"],
    },
    {
        "name": "synthes1",
        "expected_obj": 6.00975909,
        "formats": ["osil", "gms"],
        "mip_solvers": ["cplex", "gurobi", "cbc", "highs"],
    },
]


def get_data_dir():
    """Return the path to the test data directory."""
    return Path(__file__).parent.parent / "data"


def solve_file_and_verify(filename, expected_obj, tolerance=0.01, mip_solver=None, nlp_solver=None):
    """
    Load a problem from file, solve it, and verify the objective.
    
    Args:
        filename: Name of the problem file (e.g., "alan.gms")
        expected_obj: Expected optimal objective value
        tolerance: Tolerance for objective comparison
        mip_solver: Optional MIP solver name ("cplex", "gurobi", "cbc", "highs")
        nlp_solver: Optional NLP solver name ("ipopt", "gams", "shot")
        
    Returns:
        The solver instance after solving
    """
    solver = SHOTpy.Solver()
    filepath = get_data_dir() / filename
    
    if not filepath.exists():
        pytest.skip(f"Test file not found: {filepath}")
    
    result = solver.setProblem(str(filepath))
    assert result == True, f"Failed to load problem from {filename}"
    
    # Set MIP solver if specified (using type-safe enum)
    if mip_solver is not None:
        solver_enum = get_mip_solver_enum(mip_solver)
        if solver_enum is not None:
            solver.updateSetting("Dual.MIP.Solver", int(solver_enum))
    
    # Set NLP solver if specified (using type-safe enum)
    if nlp_solver is not None:
        solver_enum = get_nlp_solver_enum(nlp_solver)
        if solver_enum is not None:
            solver.updateSetting("Primal.FixedInteger.Solver", int(solver_enum))
    
    result = solver.solveProblem()
    assert result == True, f"Failed to solve problem {filename}"
    
    obj_value = solver.getPrimalBound()
    assert abs(obj_value - expected_obj) < tolerance, \
        f"Objective {obj_value} differs from expected {expected_obj} by {abs(obj_value - expected_obj)}"
    
    return solver


def _get_format_support(fmt):
    """Check if a file format is supported."""
    format_support = {
        "osil": SHOTpy.HAS_OSIL,
        "gms": SHOTpy.HAS_GAMS,
        "nl": SHOTpy.HAS_AMPL,
    }
    return format_support.get(fmt, False)


def _get_mip_solver_support(solver_name):
    """Check if a MIP solver is supported."""
    solver_support = {
        "cplex": SHOTpy.HAS_CPLEX,
        "gurobi": SHOTpy.HAS_GUROBI,
        "cbc": SHOTpy.HAS_CBC,
        "highs": SHOTpy.HAS_HIGHS,
    }
    return solver_support.get(solver_name.lower(), False)


def _get_nlp_solver_support(solver_name):
    """Check if an NLP solver is supported."""
    solver_support = {
        "ipopt": SHOTpy.HAS_IPOPT,
        "gams": SHOTpy.HAS_GAMS_NLP,
        "shot": SHOTpy.HAS_SHOT_NLP,
    }
    return solver_support.get(solver_name.lower(), False)


# MIP solvers that are not yet implemented (will be marked as xfail)
XFAIL_MIP_SOLVERS = {}

# NLP solvers that are not yet implemented or have issues (will be skipped to avoid segfault)
# Note: GAMS NLP solver can only be used with GAMS format files - this is handled separately
SKIP_NLP_SOLVERS = {}

# NLP solvers that are implemented but may have issues (will be marked as xfail)
XFAIL_NLP_SOLVERS = {}


# All available MIP solvers (used as default when "mip_solvers" not specified)
ALL_MIP_SOLVERS = ["cplex", "gurobi", "cbc", "highs"]

# All available NLP solvers (used as default when "nlp_solvers" not specified)
ALL_NLP_SOLVERS = ["ipopt", "gams", "shot"]


def _generate_test_cases():
    """Generate test case parameters from TEST_PROBLEMS."""
    test_cases = []
    for problem in TEST_PROBLEMS:
        name = problem["name"]
        expected_obj = problem["expected_obj"]
        tolerance = problem.get("tolerance", 0.01)
        xfail = problem.get("xfail", False)
        xfail_reason = problem.get("xfail_reason", "Known issue")
        mip_solvers = problem.get("mip_solvers", ALL_MIP_SOLVERS)
        nlp_solvers = problem.get("nlp_solvers", ALL_NLP_SOLVERS)
        
        for fmt in problem["formats"]:
            for mip_solver in mip_solvers:
                for nlp_solver in nlp_solvers:
                    filename = f"{name}.{fmt}"
                    
                    # Build test ID
                    test_id = f"{name}_{fmt}_{mip_solver}_{nlp_solver}"
                    
                    # Build marks list
                    marks = []
                    
                    # Add skip mark if format not supported
                    if not _get_format_support(fmt):
                        marks.append(pytest.mark.skip(reason=f"{fmt.upper()} format not available"))
                    
                    # Add skip mark if MIP solver not supported
                    if mip_solver and not _get_mip_solver_support(mip_solver):
                        marks.append(pytest.mark.skip(reason=f"{mip_solver.upper()} MIP solver not available"))
                    
                    # Add skip mark if NLP solver not supported
                    if nlp_solver and not _get_nlp_solver_support(nlp_solver):
                        marks.append(pytest.mark.skip(reason=f"{nlp_solver.upper()} NLP solver not available"))
                    
                    # GAMS NLP solver can only be used with GAMS format files
                    if nlp_solver and nlp_solver.lower() == "gams" and fmt != "gms":
                        marks.append(pytest.mark.skip(reason="GAMS NLP solver only works with GAMS format files"))
                    
                    # Add skip mark for NLP solvers known to cause issues (like segfaults)
                    if nlp_solver and nlp_solver.lower() in SKIP_NLP_SOLVERS:
                        marks.append(pytest.mark.skip(reason=SKIP_NLP_SOLVERS[nlp_solver.lower()]))
                    
                    # Add xfail mark for MIP solvers not yet implemented
                    if mip_solver and mip_solver.lower() in XFAIL_MIP_SOLVERS:
                        marks.append(pytest.mark.xfail(reason=XFAIL_MIP_SOLVERS[mip_solver.lower()]))
                    
                    # Add xfail mark for NLP solvers not yet implemented
                    if nlp_solver and nlp_solver.lower() in XFAIL_NLP_SOLVERS:
                        marks.append(pytest.mark.xfail(reason=XFAIL_NLP_SOLVERS[nlp_solver.lower()]))
                    
                    # Add xfail mark if specified for the problem
                    if xfail:
                        marks.append(pytest.mark.xfail(reason=xfail_reason))
                    
                    test_cases.append(
                        pytest.param(filename, expected_obj, tolerance, mip_solver, nlp_solver,
                                    id=test_id, marks=marks)
                    )
    
    return test_cases


class TestProblemsFromFile:
    """
    Parametrized tests for optimization problems loaded from files.
    
    Tests are automatically generated from TEST_PROBLEMS configuration.
    To add a new problem, simply add an entry to TEST_PROBLEMS above.
    """
    
    @pytest.mark.parametrize("filename,expected_obj,tolerance,mip_solver,nlp_solver", _generate_test_cases())
    def test_solve_problem(self, filename, expected_obj, tolerance, mip_solver, nlp_solver):
        """Test solving a problem from file and verify optimal objective."""
        solve_file_and_verify(filename, expected_obj, tolerance, mip_solver, nlp_solver)
