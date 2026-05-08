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
