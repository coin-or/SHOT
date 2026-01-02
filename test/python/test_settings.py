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
