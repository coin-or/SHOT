"""
Tests for module-level constants and enums in the Python API.
"""

import pytest


class TestConstants:
    """Tests for SHOT constants."""

    def test_shot_dbl_max(self):
        """Test that SHOT_DBL_MAX is defined and is a large number."""
        import shotpy
        
        assert hasattr(shotpy, 'SHOT_DBL_MAX')
        assert shotpy.SHOT_DBL_MAX > 1e100

    def test_shot_dbl_min(self):
        """Test that SHOT_DBL_MIN is defined and is a very negative number."""
        import shotpy
        
        assert hasattr(shotpy, 'SHOT_DBL_MIN')
        assert shotpy.SHOT_DBL_MIN < -1e100

    def test_has_osil_constant(self):
        """Test that HAS_OSIL is defined and is True (always available)."""
        import shotpy
        
        assert hasattr(shotpy, 'HAS_OSIL')
        assert shotpy.HAS_OSIL == True

    def test_has_gams_constant(self):
        """Test that HAS_GAMS is defined and is a boolean."""
        import shotpy
        
        assert hasattr(shotpy, 'HAS_GAMS')
        assert isinstance(shotpy.HAS_GAMS, bool)

    def test_has_ampl_constant(self):
        """Test that HAS_AMPL is defined and is a boolean."""
        import shotpy
        
        assert hasattr(shotpy, 'HAS_AMPL')
        assert isinstance(shotpy.HAS_AMPL, bool)

    def test_get_supported_modeling_systems(self):
        """Test that getSupportedModelingSystems function exists and returns a list."""
        import shotpy
        
        assert hasattr(shotpy, 'getSupportedModelingSystems')
        systems = shotpy.getSupportedModelingSystems()
        assert isinstance(systems, list)
        assert 'OSiL' in systems


class TestEnums:
    """Tests for SHOT enumerations."""

    def test_variable_type_enum(self):
        """Test VariableType enumeration."""
        import shotpy
        
        assert hasattr(shotpy, 'VariableType')
        assert hasattr(shotpy.VariableType, 'Real')
        assert hasattr(shotpy.VariableType, 'Binary')
        assert hasattr(shotpy.VariableType, 'Integer')
        assert hasattr(shotpy.VariableType, 'Semicontinuous')
        assert hasattr(shotpy.VariableType, 'Semiinteger')

    def test_objective_direction_enum(self):
        """Test ObjectiveDirection enumeration."""
        import shotpy
        
        assert hasattr(shotpy, 'ObjectiveDirection')
        assert hasattr(shotpy.ObjectiveDirection, 'Minimize')
        assert hasattr(shotpy.ObjectiveDirection, 'Maximize')

    def test_convexity_enum(self):
        """Test Convexity enumeration."""
        import shotpy
        
        assert hasattr(shotpy, 'Convexity')
        assert hasattr(shotpy.Convexity, 'Linear')
        assert hasattr(shotpy.Convexity, 'Convex')
        assert hasattr(shotpy.Convexity, 'Concave')
        assert hasattr(shotpy.Convexity, 'Nonconvex')

    def test_problem_convexity_enum(self):
        """Test ProblemConvexity enumeration."""
        import shotpy
        
        assert hasattr(shotpy, 'ProblemConvexity')
        assert hasattr(shotpy.ProblemConvexity, 'Convex')
        assert hasattr(shotpy.ProblemConvexity, 'Nonconvex')


class TestClassesExist:
    """Tests that expected classes are available."""

    def test_solver_class(self):
        """Test Solver class exists."""
        import shotpy
        assert hasattr(shotpy, 'Solver')

    def test_problem_class(self):
        """Test Problem class exists."""
        import shotpy
        assert hasattr(shotpy, 'Problem')

    def test_variable_class(self):
        """Test Variable class exists."""
        import shotpy
        assert hasattr(shotpy, 'Variable')

    def test_linear_term_class(self):
        """Test LinearTerm class exists."""
        import shotpy
        assert hasattr(shotpy, 'LinearTerm')

    def test_quadratic_term_class(self):
        """Test QuadraticTerm class exists."""
        import shotpy
        assert hasattr(shotpy, 'QuadraticTerm')

    def test_constraint_classes(self):
        """Test constraint classes exist."""
        import shotpy
        assert hasattr(shotpy, 'LinearConstraint')
        assert hasattr(shotpy, 'QuadraticConstraint')
        assert hasattr(shotpy, 'NonlinearConstraint')

    def test_objective_classes(self):
        """Test objective function classes exist."""
        import shotpy
        assert hasattr(shotpy, 'LinearObjectiveFunction')
        assert hasattr(shotpy, 'QuadraticObjectiveFunction')
        assert hasattr(shotpy, 'NonlinearObjectiveFunction')


class TestMathFunctions:
    """Tests for mathematical functions."""

    def test_log_function(self):
        """Test log function exists."""
        import shotpy
        assert hasattr(shotpy, 'log')

    def test_exp_function(self):
        """Test exp function exists."""
        import shotpy
        assert hasattr(shotpy, 'exp')

    def test_sqrt_function(self):
        """Test sqrt function exists."""
        import shotpy
        assert hasattr(shotpy, 'sqrt')

    def test_sin_function(self):
        """Test sin function exists."""
        import shotpy
        assert hasattr(shotpy, 'sin')

    def test_cos_function(self):
        """Test cos function exists."""
        import shotpy
        assert hasattr(shotpy, 'cos')

    def test_abs_function(self):
        """Test abs function exists."""
        import shotpy
        assert hasattr(shotpy, 'abs')

    def test_square_function(self):
        """Test square function exists."""
        import shotpy
        assert hasattr(shotpy, 'square')
