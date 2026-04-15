"""
Tests for modeling system support (GAMS, AMPL, OSiL).

These tests verify that SHOT can read and solve problems from different
modeling system formats. Tests are automatically skipped if the corresponding
modeling system support is not compiled in.
"""

import pytest
import SHOTpy
from pathlib import Path


# Skip decorators for conditional tests
skip_if_no_gams = pytest.mark.skipif(
    not SHOTpy.HAS_GAMS,
    reason="GAMS support not available in this build"
)

skip_if_no_ampl = pytest.mark.skipif(
    not SHOTpy.HAS_AMPL,
    reason="AMPL support not available in this build"
)


class TestModelingSystemAvailability:
    """Tests for checking modeling system availability."""

    def test_has_osil_always_true(self):
        """OSiL support should always be available."""
        assert SHOTpy.HAS_OSIL == True

    def test_has_gams_is_bool(self):
        """HAS_GAMS should be a boolean."""
        assert isinstance(SHOTpy.HAS_GAMS, bool)

    def test_has_ampl_is_bool(self):
        """HAS_AMPL should be a boolean."""
        assert isinstance(SHOTpy.HAS_AMPL, bool)

    def test_get_supported_modeling_systems(self):
        """Test getSupportedModelingSystems returns valid list of enum values."""
        systems = SHOTpy.getSupportedModelingSystems()
        
        assert isinstance(systems, list)
        assert SHOTpy.ModelingSystem.OSiL in systems
        
        # All items should be ModelingSystem enum values
        for system in systems:
            assert isinstance(system, SHOTpy.ModelingSystem)
        
        # Verify consistency with individual flags
        if SHOTpy.HAS_GAMS:
            assert SHOTpy.ModelingSystem.GAMS in systems
        else:
            assert SHOTpy.ModelingSystem.GAMS not in systems
            
        if SHOTpy.HAS_AMPL:
            assert SHOTpy.ModelingSystem.AMPL in systems
        else:
            assert SHOTpy.ModelingSystem.AMPL not in systems


class TestOSiL:
    """Tests for OSiL format support."""

    @pytest.fixture
    def solver(self):
        """Create a fresh solver instance."""
        return SHOTpy.Solver()

    @pytest.fixture
    def data_dir(self):
        """Return the path to the test data directory."""
        return Path(__file__).parent.parent / "data"

    def test_read_osil_file(self, solver, data_dir):
        """Test reading an OSiL file."""
        osil_file = data_dir / "tls2.osil"
        
        if not osil_file.exists():
            pytest.skip(f"Test file not found: {osil_file}")
        
        result = solver.setProblem(str(osil_file))
        assert result == True

    def test_solve_osil_problem(self, solver, data_dir):
        """Test solving a problem from OSiL file."""
        osil_file = data_dir / "tls2.osil"
        
        if not osil_file.exists():
            pytest.skip(f"Test file not found: {osil_file}")
        
        solver.setProblem(str(osil_file))
        result = solver.solveProblem()
        assert result == True
        
        obj_value = solver.getPrimalBound()
        assert isinstance(obj_value, float)


@skip_if_no_gams
class TestGAMS:
    """Tests for GAMS format support."""

    @pytest.fixture
    def solver(self):
        """Create a fresh solver instance."""
        return SHOTpy.Solver()

    @pytest.fixture
    def data_dir(self):
        """Return the path to the test data directory."""
        return Path(__file__).parent.parent / "data"

    def test_read_gams_file(self, solver, data_dir):
        """Test reading a GAMS file."""
        gms_file = data_dir / "tls2.gms"
        
        if not gms_file.exists():
            pytest.skip(f"Test file not found: {gms_file}")
        
        result = solver.setProblem(str(gms_file))
        assert result == True

    def test_solve_gams_problem(self, solver, data_dir):
        """Test solving a problem from GAMS file."""
        gms_file = data_dir / "tls2.gms"
        
        if not gms_file.exists():
            pytest.skip(f"Test file not found: {gms_file}")
        
        solver.setProblem(str(gms_file))
        result = solver.solveProblem()
        assert result == True
        
        obj_value = solver.getPrimalBound()
        assert isinstance(obj_value, float)


@skip_if_no_ampl
class TestAMPL:
    """Tests for AMPL (.nl) format support."""

    @pytest.fixture
    def solver(self):
        """Create a fresh solver instance."""
        return SHOTpy.Solver()

    @pytest.fixture
    def data_dir(self):
        """Return the path to the test data directory."""
        return Path(__file__).parent.parent / "data"

    def test_read_nl_file(self, solver, data_dir):
        """Test reading an AMPL .nl file."""
        nl_file = data_dir / "tls2.nl"
        
        if not nl_file.exists():
            pytest.skip(f"Test file not found: {nl_file}")
        
        result = solver.setProblem(str(nl_file))
        assert result == True

    def test_solve_ampl_problem(self, solver, data_dir):
        """Test solving a problem from AMPL .nl file."""
        nl_file = data_dir / "tls2.nl"
        
        if not nl_file.exists():
            pytest.skip(f"Test file not found: {nl_file}")
        
        solver.setProblem(str(nl_file))
        result = solver.solveProblem()
        assert result == True
        
        obj_value = solver.getPrimalBound()
        assert isinstance(obj_value, float)
