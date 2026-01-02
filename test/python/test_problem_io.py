"""
Tests for problem I/O in the Python API.
"""

import pytest
from pathlib import Path


class TestOSiLInput:
    """Tests for reading OSiL problem files."""

    def test_read_osil_file(self, solver, data_dir):
        """Test reading an OSiL file."""
        osil_file = data_dir / "tls2.osil"
        
        if not osil_file.exists():
            pytest.skip(f"Test file not found: {osil_file}")
        
        result = solver.setProblem(str(osil_file))
        assert result == True

    def test_read_nonexistent_file(self, solver):
        """Test that reading a non-existent file returns False."""
        result = solver.setProblem("/nonexistent/path/to/file.osil")
        assert result == False


class TestResultsOutput:
    """Tests for getting results after solving."""

    def test_get_primal_bound(self, solver, data_dir):
        """Test getting the primal bound after solving."""
        osil_file = data_dir / "tls2.osil"
        
        if not osil_file.exists():
            pytest.skip(f"Test file not found: {osil_file}")
        
        solver.setProblem(str(osil_file))
        solver.solveProblem()
        
        # Should be able to get primal bound
        primal_bound = solver.getPrimalBound()
        assert isinstance(primal_bound, float)

    def test_get_dual_bound(self, solver, data_dir):
        """Test getting the dual bound after solving."""
        osil_file = data_dir / "tls2.osil"
        
        if not osil_file.exists():
            pytest.skip(f"Test file not found: {osil_file}")
        
        solver.setProblem(str(osil_file))
        solver.solveProblem()
        
        dual_bound = solver.getCurrentDualBound()
        assert isinstance(dual_bound, float)

    def test_get_primal_solutions(self, solver, data_dir):
        """Test getting primal solutions."""
        osil_file = data_dir / "tls2.osil"
        
        if not osil_file.exists():
            pytest.skip(f"Test file not found: {osil_file}")
        
        solver.setProblem(str(osil_file))
        solver.solveProblem()
        
        solutions = solver.getPrimalSolutions()
        assert isinstance(solutions, list)
        
        if len(solutions) > 0:
            sol = solutions[0]
            assert hasattr(sol, 'objValue')
            assert hasattr(sol, 'point')

    def test_get_osrl_output(self, solver, data_dir):
        """Test getting OSrL output."""
        osil_file = data_dir / "tls2.osil"
        
        if not osil_file.exists():
            pytest.skip(f"Test file not found: {osil_file}")
        
        solver.setProblem(str(osil_file))
        solver.solveProblem()
        
        osrl = solver.getResultsOSrL()
        assert isinstance(osrl, str)
        # OSrL should contain XML
        assert "<?xml" in osrl or "<osrl" in osrl.lower()


class TestProblemInfo:
    """Tests for getting problem information."""

    def test_get_problem_name(self, problem):
        """Test setting and getting problem name."""
        import SHOTpy
        
        problem.name = "test_problem"
        assert problem.name == "test_problem"

    def test_problem_to_string(self, problem):
        """Test converting problem to string representation."""
        import SHOTpy
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "c1", 0.0, 5.0)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        
        problem_str = problem.toString()
        assert isinstance(problem_str, str)
        assert len(problem_str) > 0
        assert "x" in problem_str
