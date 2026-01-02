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

    def test_has_cplex_constant(self):
        """Test that HAS_CPLEX is defined and is a boolean."""
        import shotpy
        
        assert hasattr(shotpy, 'HAS_CPLEX')
        assert isinstance(shotpy.HAS_CPLEX, bool)

    def test_has_gurobi_constant(self):
        """Test that HAS_GUROBI is defined and is a boolean."""
        import shotpy
        
        assert hasattr(shotpy, 'HAS_GUROBI')
        assert isinstance(shotpy.HAS_GUROBI, bool)

    def test_has_cbc_constant(self):
        """Test that HAS_CBC is defined and is a boolean."""
        import shotpy
        
        assert hasattr(shotpy, 'HAS_CBC')
        assert isinstance(shotpy.HAS_CBC, bool)

    def test_has_highs_constant(self):
        """Test that HAS_HIGHS is defined and is a boolean."""
        import shotpy
        
        assert hasattr(shotpy, 'HAS_HIGHS')
        assert isinstance(shotpy.HAS_HIGHS, bool)

    @pytest.mark.xfail(reason="HiGHS support not yet implemented")
    def test_highs_available(self):
        """Test that HiGHS solver is available."""
        import shotpy
        
        assert shotpy.HAS_HIGHS == True

    def test_get_supported_mip_solvers(self):
        """Test that getSupportedMIPSolvers function exists and returns a list."""
        import shotpy
        
        assert hasattr(shotpy, 'getSupportedMIPSolvers')
        solvers = shotpy.getSupportedMIPSolvers()
        assert isinstance(solvers, list)
        # At least one MIP solver must be available (required by SHOT)
        assert len(solvers) >= 1

    def test_mip_solver_constants_match_function(self):
        """Test that MIP solver constants are consistent with getSupportedMIPSolvers."""
        import shotpy
        
        solvers = shotpy.getSupportedMIPSolvers()
        
        if shotpy.HAS_CPLEX:
            assert 'CPLEX' in solvers
        if shotpy.HAS_GUROBI:
            assert 'Gurobi' in solvers
        if shotpy.HAS_CBC:
            assert 'Cbc' in solvers
        if shotpy.HAS_HIGHS:
            assert 'HiGHS' in solvers

    def test_has_ipopt_constant(self):
        """Test that HAS_IPOPT is defined and is a boolean."""
        import shotpy
        
        assert hasattr(shotpy, 'HAS_IPOPT')
        assert isinstance(shotpy.HAS_IPOPT, bool)

    def test_has_shot_nlp_constant(self):
        """Test that HAS_SHOT_NLP is defined and is True (always available)."""
        import shotpy
        
        assert hasattr(shotpy, 'HAS_SHOT_NLP')
        assert shotpy.HAS_SHOT_NLP == True

    def test_has_gams_nlp_constant(self):
        """Test that HAS_GAMS_NLP is defined and is a boolean."""
        import shotpy
        
        assert hasattr(shotpy, 'HAS_GAMS_NLP')
        assert isinstance(shotpy.HAS_GAMS_NLP, bool)

    def test_get_supported_nlp_solvers(self):
        """Test that getSupportedNLPSolvers function exists and returns a list."""
        import shotpy
        
        assert hasattr(shotpy, 'getSupportedNLPSolvers')
        solvers = shotpy.getSupportedNLPSolvers()
        assert isinstance(solvers, list)
        # SHOT NLP solver is always available
        assert 'SHOT' in solvers

    def test_nlp_solver_constants_match_function(self):
        """Test that NLP solver constants are consistent with getSupportedNLPSolvers."""
        import shotpy
        
        solvers = shotpy.getSupportedNLPSolvers()
        
        # SHOT is always available
        assert 'SHOT' in solvers
        
        if shotpy.HAS_IPOPT:
            assert 'Ipopt' in solvers
        if shotpy.HAS_GAMS_NLP:
            assert 'GAMS' in solvers


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

    def test_hyperplane_cut_strategy_enum(self):
        """Test HyperplaneCutStrategy enumeration."""
        import shotpy
        
        assert hasattr(shotpy, 'HyperplaneCutStrategy')
        assert hasattr(shotpy.HyperplaneCutStrategy, 'ESH')
        assert hasattr(shotpy.HyperplaneCutStrategy, 'ECP')

    def test_iteration_output_detail_enum(self):
        """Test IterationOutputDetail enumeration."""
        import shotpy
        
        assert hasattr(shotpy, 'IterationOutputDetail')
        assert hasattr(shotpy.IterationOutputDetail, 'Full')
        assert hasattr(shotpy.IterationOutputDetail, 'ObjectiveGapUpdates')
        assert hasattr(shotpy.IterationOutputDetail, 'ObjectiveGapUpdatesAndNLPCalls')

    def test_mip_solver_enum(self):
        """Test MIPSolver enumeration."""
        import shotpy
        
        assert hasattr(shotpy, 'MIPSolver')
        assert hasattr(shotpy.MIPSolver, 'Cplex')
        assert hasattr(shotpy.MIPSolver, 'Gurobi')
        assert hasattr(shotpy.MIPSolver, 'Cbc')
        assert hasattr(shotpy.MIPSolver, 'None')

    def test_primal_nlp_fixed_point_enum(self):
        """Test PrimalNLPFixedPoint enumeration."""
        import shotpy
        
        assert hasattr(shotpy, 'PrimalNLPFixedPoint')
        assert hasattr(shotpy.PrimalNLPFixedPoint, 'AllSolutions')
        assert hasattr(shotpy.PrimalNLPFixedPoint, 'FirstSolution')
        assert hasattr(shotpy.PrimalNLPFixedPoint, 'AllFeasibleSolutions')
        assert hasattr(shotpy.PrimalNLPFixedPoint, 'FirstAndFeasibleSolutions')
        assert hasattr(shotpy.PrimalNLPFixedPoint, 'SmallestDeviationSolution')

    def test_primal_nlp_problem_source_enum(self):
        """Test PrimalNLPProblemSource enumeration."""
        import shotpy
        
        assert hasattr(shotpy, 'PrimalNLPProblemSource')
        assert hasattr(shotpy.PrimalNLPProblemSource, 'OriginalProblem')
        assert hasattr(shotpy.PrimalNLPProblemSource, 'ReformulatedProblem')
        assert hasattr(shotpy.PrimalNLPProblemSource, 'Both')

    def test_primal_nlp_solver_enum(self):
        """Test PrimalNLPSolver enumeration."""
        import shotpy
        
        assert hasattr(shotpy, 'PrimalNLPSolver')
        assert hasattr(shotpy.PrimalNLPSolver, 'Ipopt')
        assert hasattr(shotpy.PrimalNLPSolver, 'GAMS')
        assert hasattr(shotpy.PrimalNLPSolver, 'SHOT')
        assert hasattr(shotpy.PrimalNLPSolver, 'None')

    def test_primal_nlp_strategy_enum(self):
        """Test PrimalNLPStrategy enumeration."""
        import shotpy
        
        assert hasattr(shotpy, 'PrimalNLPStrategy')
        assert hasattr(shotpy.PrimalNLPStrategy, 'AlwaysUse')
        assert hasattr(shotpy.PrimalNLPStrategy, 'IterationOrTime')
        assert hasattr(shotpy.PrimalNLPStrategy, 'IterationOrTimeAndAllFeasibleSolutions')

    def test_quadratic_problem_strategy_enum(self):
        """Test QuadraticProblemStrategy enumeration."""
        import shotpy
        
        assert hasattr(shotpy, 'QuadraticProblemStrategy')
        assert hasattr(shotpy.QuadraticProblemStrategy, 'Nonlinear')
        assert hasattr(shotpy.QuadraticProblemStrategy, 'QuadraticObjective')
        assert hasattr(shotpy.QuadraticProblemStrategy, 'ConvexQuadraticallyConstrained')
        assert hasattr(shotpy.QuadraticProblemStrategy, 'NonconvexQuadraticallyConstrained')

    def test_tree_strategy_enum(self):
        """Test TreeStrategy enumeration."""
        import shotpy
        
        assert hasattr(shotpy, 'TreeStrategy')
        assert hasattr(shotpy.TreeStrategy, 'MultiTree')
        assert hasattr(shotpy.TreeStrategy, 'SingleTree')


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
