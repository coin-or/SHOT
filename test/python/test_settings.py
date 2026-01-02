"""
Tests for Solver settings in the Python API.
"""

import pytest


class TestSettingsAccess:
    """Tests for getting and setting solver settings."""

    def test_get_int_setting(self, solver):
        """Test getting an integer setting."""
        # Console.LogLevel is an integer setting
        log_level = solver.getIntSetting("Console.LogLevel", "Output")
        assert isinstance(log_level, int)

    def test_update_int_setting(self, solver):
        """Test updating an integer setting."""
        # Set log level to 2 (Info)
        solver.updateSetting("Console.LogLevel", "Output", 2)
        
        log_level = solver.getIntSetting("Console.LogLevel", "Output")
        assert log_level == 2

    def test_get_double_setting(self, solver):
        """Test getting a double setting."""
        import shotpy
        
        # TimeLimit is a double setting
        time_limit = solver.getDoubleSetting("TimeLimit", "Termination")
        assert isinstance(time_limit, float)

    def test_update_double_setting(self, solver):
        """Test updating a double setting."""
        solver.updateSetting("TimeLimit", "Termination", 100.0)
        
        time_limit = solver.getDoubleSetting("TimeLimit", "Termination")
        assert time_limit == 100.0

    def test_get_bool_setting(self, solver):
        """Test getting a boolean setting."""
        # Check if we can get a boolean setting
        # Reformulation.Monomials.Extract is a boolean
        extract = solver.getBoolSetting("Reformulation.Monomials.Extract", "Model")
        assert isinstance(extract, bool)

    def test_get_string_setting(self, solver):
        """Test getting a string setting."""
        # Debug.Path is a string setting in the Output category
        debug_path = solver.getStringSetting("Debug.Path", "Output")
        assert isinstance(debug_path, str)


class TestSettingsCategories:
    """Tests for different setting categories."""

    def test_output_settings(self, solver):
        """Test accessing Output category settings."""
        # Should be able to access Console.LogLevel
        log_level = solver.getIntSetting("Console.LogLevel", "Output")
        assert log_level >= 0

    def test_termination_settings(self, solver):
        """Test accessing Termination category settings."""
        time_limit = solver.getDoubleSetting("TimeLimit", "Termination")
        assert time_limit > 0

    def test_model_settings(self, solver):
        """Test accessing Model category settings."""
        extract_monomials = solver.getBoolSetting("Reformulation.Monomials.Extract", "Model")
        assert isinstance(extract_monomials, bool)


class TestSettingsValidation:
    """Tests for settings validation."""

    def test_setting_out_of_range(self, solver):
        """Test that setting an invalid value raises an error."""
        import shotpy
        
        # LogLevel should be in a valid range (0-6 typically)
        # Setting an extremely large value should fail
        with pytest.raises(Exception):
            solver.updateSetting("Console.LogLevel", "Output", 1000)

    def test_invalid_setting_name(self, solver):
        """Test that an invalid setting name raises an error."""
        import shotpy
        
        with pytest.raises(Exception):
            solver.getIntSetting("NonExistent.Setting", "Output")

    def test_invalid_category(self, solver):
        """Test that an invalid category raises an error."""
        import shotpy
        
        with pytest.raises(Exception):
            solver.getIntSetting("Console.LogLevel", "InvalidCategory")


class TestSettingsWithEnums:
    """Tests for using enum types with settings."""

    def test_set_mip_solver_with_enum(self, solver):
        """Test setting MIP solver using the MIPSolver enum."""
        import shotpy
        
        # Set MIP solver using enum (convert to int for the setting)
        solver.updateSetting("MIP.Solver", "Dual", int(shotpy.MIPSolver.Cbc))
        
        current_solver = solver.getIntSetting("MIP.Solver", "Dual")
        assert current_solver == int(shotpy.MIPSolver.Cbc)

    def test_set_nlp_solver_with_enum(self, solver):
        """Test setting NLP solver using the PrimalNLPSolver enum."""
        import shotpy
        
        # Only test if Ipopt is available
        if not shotpy.HAS_IPOPT:
            pytest.skip("Ipopt not available")
        
        solver.updateSetting("FixedInteger.Solver", "Primal", int(shotpy.PrimalNLPSolver.Ipopt))
        
        current_solver = solver.getIntSetting("FixedInteger.Solver", "Primal")
        assert current_solver == int(shotpy.PrimalNLPSolver.Ipopt)

    def test_set_tree_strategy_with_enum(self, solver):
        """Test setting tree strategy using the TreeStrategy enum."""
        import shotpy
        
        solver.updateSetting("TreeStrategy", "Dual", int(shotpy.TreeStrategy.MultiTree))
        
        current_strategy = solver.getIntSetting("TreeStrategy", "Dual")
        assert current_strategy == int(shotpy.TreeStrategy.MultiTree)
        
        solver.updateSetting("TreeStrategy", "Dual", int(shotpy.TreeStrategy.SingleTree))
        
        current_strategy = solver.getIntSetting("TreeStrategy", "Dual")
        assert current_strategy == int(shotpy.TreeStrategy.SingleTree)

    def test_set_nlp_strategy_with_enum(self, solver):
        """Test setting NLP call strategy using the PrimalNLPStrategy enum."""
        import shotpy
        
        solver.updateSetting("FixedInteger.CallStrategy", "Primal", 
                            int(shotpy.PrimalNLPStrategy.AlwaysUse))
        
        current_strategy = solver.getIntSetting("FixedInteger.CallStrategy", "Primal")
        assert current_strategy == int(shotpy.PrimalNLPStrategy.AlwaysUse)

    def test_set_quadratic_strategy_with_enum(self, solver):
        """Test setting quadratic problem strategy using the QuadraticProblemStrategy enum."""
        import shotpy
        
        solver.updateSetting("Reformulation.Quadratics.Strategy", "Model",
                            int(shotpy.QuadraticProblemStrategy.Nonlinear))
        
        current_strategy = solver.getIntSetting("Reformulation.Quadratics.Strategy", "Model")
        assert current_strategy == int(shotpy.QuadraticProblemStrategy.Nonlinear)

    def test_set_iteration_detail_with_enum(self, solver):
        """Test setting iteration output detail using the IterationOutputDetail enum."""
        import shotpy
        
        solver.updateSetting("Console.Iteration.Detail", "Output",
                            int(shotpy.IterationOutputDetail.Full))
        
        current_detail = solver.getIntSetting("Console.Iteration.Detail", "Output")
        assert current_detail == int(shotpy.IterationOutputDetail.Full)

    def test_enum_values_are_correct(self):
        """Test that enum values match expected C++ values."""
        import shotpy
        
        # MIPSolver values
        assert int(shotpy.MIPSolver.Cplex) == 0
        assert int(shotpy.MIPSolver.Gurobi) == 1
        assert int(shotpy.MIPSolver.Cbc) == 2
        # 'None' is a Python keyword, so use getattr to access it
        assert int(getattr(shotpy.MIPSolver, 'None')) == 3
        
        # PrimalNLPSolver values
        assert int(shotpy.PrimalNLPSolver.Ipopt) == 0
        assert int(shotpy.PrimalNLPSolver.GAMS) == 1
        assert int(shotpy.PrimalNLPSolver.SHOT) == 2
        assert int(getattr(shotpy.PrimalNLPSolver, 'None')) == 3
        
        # TreeStrategy values
        assert int(shotpy.TreeStrategy.MultiTree) == 0
        assert int(shotpy.TreeStrategy.SingleTree) == 1
        
        # HyperplaneCutStrategy values
        assert int(shotpy.HyperplaneCutStrategy.ESH) == 0
        assert int(shotpy.HyperplaneCutStrategy.ECP) == 1
