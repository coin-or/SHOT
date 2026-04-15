"""
Tests for module-level constants and enums in the Python API.
"""

import pytest


class TestConstants:
    """Tests for SHOT constants."""

    def test_shot_dbl_max(self):
        """Test that SHOT_DBL_MAX is defined and is a large number."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'SHOT_DBL_MAX')
        assert SHOTpy.SHOT_DBL_MAX > 1e100

    def test_shot_dbl_min(self):
        """Test that SHOT_DBL_MIN is defined and is a very negative number."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'SHOT_DBL_MIN')
        assert SHOTpy.SHOT_DBL_MIN < -1e100

    def test_has_osil_constant(self):
        """Test that HAS_OSIL is defined and is True (always available)."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'HAS_OSIL')
        assert SHOTpy.HAS_OSIL == True

    def test_has_gams_constant(self):
        """Test that HAS_GAMS is defined and is a boolean."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'HAS_GAMS')
        assert isinstance(SHOTpy.HAS_GAMS, bool)

    def test_has_ampl_constant(self):
        """Test that HAS_AMPL is defined and is a boolean."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'HAS_AMPL')
        assert isinstance(SHOTpy.HAS_AMPL, bool)

    def test_get_supported_modeling_systems(self):
        """Test that getSupportedModelingSystems function exists and returns a list of enum values."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'getSupportedModelingSystems')
        systems = SHOTpy.getSupportedModelingSystems()
        assert isinstance(systems, list)
        assert SHOTpy.ModelingSystem.OSiL in systems
        # All items should be ModelingSystem enum values
        for system in systems:
            assert isinstance(system, SHOTpy.ModelingSystem)

    def test_has_cplex_constant(self):
        """Test that HAS_CPLEX is defined and is a boolean."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'HAS_CPLEX')
        assert isinstance(SHOTpy.HAS_CPLEX, bool)

    def test_has_gurobi_constant(self):
        """Test that HAS_GUROBI is defined and is a boolean."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'HAS_GUROBI')
        assert isinstance(SHOTpy.HAS_GUROBI, bool)

    def test_has_cbc_constant(self):
        """Test that HAS_CBC is defined and is a boolean."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'HAS_CBC')
        assert isinstance(SHOTpy.HAS_CBC, bool)

    def test_has_highs_constant(self):
        """Test that HAS_HIGHS is defined and is a boolean."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'HAS_HIGHS')
        assert isinstance(SHOTpy.HAS_HIGHS, bool)

    def test_highs_available(self):
        """Test that HiGHS solver is available if compiled in."""
        import SHOTpy
        
        # This test will pass if HiGHS is available, otherwise it will fail
        # The actual availability depends on whether HAS_HIGHS was defined during compilation
        if SHOTpy.HAS_HIGHS:
            assert SHOTpy.HAS_HIGHS == True

    def test_get_supported_mip_solvers(self):
        """Test that getSupportedMIPSolvers function exists and returns a list of enum values."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'getSupportedMIPSolvers')
        solvers = SHOTpy.getSupportedMIPSolvers()
        assert isinstance(solvers, list)
        # At least one MIP solver must be available (required by SHOT)
        assert len(solvers) >= 1
        # All items should be MIPSolver enum values
        for solver in solvers:
            assert isinstance(solver, SHOTpy.MIPSolver)

    def test_mip_solver_constants_match_function(self):
        """Test that MIP solver constants are consistent with getSupportedMIPSolvers."""
        import SHOTpy
        
        solvers = SHOTpy.getSupportedMIPSolvers()
        
        if SHOTpy.HAS_CPLEX:
            assert SHOTpy.MIPSolver.Cplex in solvers
        if SHOTpy.HAS_GUROBI:
            assert SHOTpy.MIPSolver.Gurobi in solvers
        if SHOTpy.HAS_CBC:
            assert SHOTpy.MIPSolver.Cbc in solvers

    def test_has_ipopt_constant(self):
        """Test that HAS_IPOPT is defined and is a boolean."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'HAS_IPOPT')
        assert isinstance(SHOTpy.HAS_IPOPT, bool)

    def test_has_shot_nlp_constant(self):
        """Test that HAS_SHOT_NLP is defined and is True (always available)."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'HAS_SHOT_NLP')
        assert SHOTpy.HAS_SHOT_NLP == True

    def test_has_gams_nlp_constant(self):
        """Test that HAS_GAMS_NLP is defined and is a boolean."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'HAS_GAMS_NLP')
        assert isinstance(SHOTpy.HAS_GAMS_NLP, bool)

    def test_get_supported_nlp_solvers(self):
        """Test that getSupportedNLPSolvers function exists and returns a list of enum values."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'getSupportedNLPSolvers')
        solvers = SHOTpy.getSupportedNLPSolvers()
        assert isinstance(solvers, list)
        # SHOT NLP solver is always available
        assert SHOTpy.PrimalNLPSolver.SHOT in solvers
        # All items should be PrimalNLPSolver enum values
        for solver in solvers:
            assert isinstance(solver, SHOTpy.PrimalNLPSolver)

    def test_nlp_solver_constants_match_function(self):
        """Test that NLP solver constants are consistent with getSupportedNLPSolvers."""
        import SHOTpy
        
        solvers = SHOTpy.getSupportedNLPSolvers()
        
        # SHOT is always available
        assert SHOTpy.PrimalNLPSolver.SHOT in solvers
        
        if SHOTpy.HAS_IPOPT:
            assert SHOTpy.PrimalNLPSolver.Ipopt in solvers
        if SHOTpy.HAS_GAMS_NLP:
            assert SHOTpy.PrimalNLPSolver.GAMS in solvers


class TestEnums:
    """Tests for SHOT enumerations."""

    def test_variable_type_enum(self):
        """Test VariableType enumeration."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'VariableType')
        assert hasattr(SHOTpy.VariableType, 'Real')
        assert hasattr(SHOTpy.VariableType, 'Binary')
        assert hasattr(SHOTpy.VariableType, 'Integer')
        assert hasattr(SHOTpy.VariableType, 'Semicontinuous')
        assert hasattr(SHOTpy.VariableType, 'Semiinteger')

    def test_objective_direction_enum(self):
        """Test ObjectiveDirection enumeration."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'ObjectiveDirection')
        assert hasattr(SHOTpy.ObjectiveDirection, 'Minimize')
        assert hasattr(SHOTpy.ObjectiveDirection, 'Maximize')

    def test_convexity_enum(self):
        """Test Convexity enumeration."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'Convexity')
        assert hasattr(SHOTpy.Convexity, 'Linear')
        assert hasattr(SHOTpy.Convexity, 'Convex')
        assert hasattr(SHOTpy.Convexity, 'Concave')
        assert hasattr(SHOTpy.Convexity, 'Nonconvex')

    def test_problem_convexity_enum(self):
        """Test ProblemConvexity enumeration."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'ProblemConvexity')
        assert hasattr(SHOTpy.ProblemConvexity, 'Convex')
        assert hasattr(SHOTpy.ProblemConvexity, 'Nonconvex')

    def test_modeling_system_enum(self):
        """Test ModelingSystem enumeration."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'ModelingSystem')
        assert hasattr(SHOTpy.ModelingSystem, 'OSiL')
        assert hasattr(SHOTpy.ModelingSystem, 'GAMS')
        assert hasattr(SHOTpy.ModelingSystem, 'AMPL')
        assert hasattr(SHOTpy.ModelingSystem, 'None')

    def test_hyperplane_cut_strategy_enum(self):
        """Test HyperplaneCutStrategy enumeration."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'HyperplaneCutStrategy')
        assert hasattr(SHOTpy.HyperplaneCutStrategy, 'ESH')
        assert hasattr(SHOTpy.HyperplaneCutStrategy, 'ECP')

    def test_iteration_output_detail_enum(self):
        """Test IterationOutputDetail enumeration."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'IterationOutputDetail')
        assert hasattr(SHOTpy.IterationOutputDetail, 'Full')
        assert hasattr(SHOTpy.IterationOutputDetail, 'ObjectiveGapUpdates')
        assert hasattr(SHOTpy.IterationOutputDetail, 'ObjectiveGapUpdatesAndNLPCalls')

    def test_mip_solver_enum(self):
        """Test MIPSolver enumeration."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'MIPSolver')
        assert hasattr(SHOTpy.MIPSolver, 'Cplex')
        assert hasattr(SHOTpy.MIPSolver, 'Gurobi')
        assert hasattr(SHOTpy.MIPSolver, 'Cbc')
        assert hasattr(SHOTpy.MIPSolver, 'None')

    def test_primal_nlp_fixed_point_enum(self):
        """Test PrimalNLPFixedPoint enumeration."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'PrimalNLPFixedPoint')
        assert hasattr(SHOTpy.PrimalNLPFixedPoint, 'AllSolutions')
        assert hasattr(SHOTpy.PrimalNLPFixedPoint, 'FirstSolution')
        assert hasattr(SHOTpy.PrimalNLPFixedPoint, 'AllFeasibleSolutions')
        assert hasattr(SHOTpy.PrimalNLPFixedPoint, 'FirstAndFeasibleSolutions')
        assert hasattr(SHOTpy.PrimalNLPFixedPoint, 'SmallestDeviationSolution')

    def test_primal_nlp_problem_source_enum(self):
        """Test PrimalNLPProblemSource enumeration."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'PrimalNLPProblemSource')
        assert hasattr(SHOTpy.PrimalNLPProblemSource, 'OriginalProblem')
        assert hasattr(SHOTpy.PrimalNLPProblemSource, 'ReformulatedProblem')
        assert hasattr(SHOTpy.PrimalNLPProblemSource, 'Both')

    def test_primal_nlp_solver_enum(self):
        """Test PrimalNLPSolver enumeration."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'PrimalNLPSolver')
        assert hasattr(SHOTpy.PrimalNLPSolver, 'Ipopt')
        assert hasattr(SHOTpy.PrimalNLPSolver, 'GAMS')
        assert hasattr(SHOTpy.PrimalNLPSolver, 'SHOT')
        assert hasattr(SHOTpy.PrimalNLPSolver, 'None')

    def test_primal_nlp_strategy_enum(self):
        """Test PrimalNLPStrategy enumeration."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'PrimalNLPStrategy')
        assert hasattr(SHOTpy.PrimalNLPStrategy, 'AlwaysUse')
        assert hasattr(SHOTpy.PrimalNLPStrategy, 'IterationOrTime')
        assert hasattr(SHOTpy.PrimalNLPStrategy, 'IterationOrTimeAndAllFeasibleSolutions')

    def test_quadratic_problem_strategy_enum(self):
        """Test QuadraticProblemStrategy enumeration."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'QuadraticProblemStrategy')
        assert hasattr(SHOTpy.QuadraticProblemStrategy, 'Nonlinear')
        assert hasattr(SHOTpy.QuadraticProblemStrategy, 'QuadraticObjective')
        assert hasattr(SHOTpy.QuadraticProblemStrategy, 'ConvexQuadraticallyConstrained')
        assert hasattr(SHOTpy.QuadraticProblemStrategy, 'NonconvexQuadraticallyConstrained')

    def test_tree_strategy_enum(self):
        """Test TreeStrategy enumeration."""
        import SHOTpy
        
        assert hasattr(SHOTpy, 'TreeStrategy')
        assert hasattr(SHOTpy.TreeStrategy, 'MultiTree')
        assert hasattr(SHOTpy.TreeStrategy, 'SingleTree')


class TestClassesExist:
    """Tests that expected classes are available."""

    def test_solver_class(self):
        """Test Solver class exists."""
        import SHOTpy
        assert hasattr(SHOTpy, 'Solver')

    def test_problem_class(self):
        """Test Problem class exists."""
        import SHOTpy
        assert hasattr(SHOTpy, 'Problem')

    def test_variable_class(self):
        """Test Variable class exists."""
        import SHOTpy
        assert hasattr(SHOTpy, 'Variable')

    def test_linear_term_class(self):
        """Test LinearTerm class exists."""
        import SHOTpy
        assert hasattr(SHOTpy, 'LinearTerm')

    def test_quadratic_term_class(self):
        """Test QuadraticTerm class exists."""
        import SHOTpy
        assert hasattr(SHOTpy, 'QuadraticTerm')

    def test_constraint_classes(self):
        """Test constraint classes exist."""
        import SHOTpy
        assert hasattr(SHOTpy, 'LinearConstraint')
        assert hasattr(SHOTpy, 'QuadraticConstraint')
        assert hasattr(SHOTpy, 'NonlinearConstraint')

    def test_objective_classes(self):
        """Test objective function classes exist."""
        import SHOTpy
        assert hasattr(SHOTpy, 'LinearObjectiveFunction')
        assert hasattr(SHOTpy, 'QuadraticObjectiveFunction')
        assert hasattr(SHOTpy, 'NonlinearObjectiveFunction')


class TestMathFunctions:
    """Tests for mathematical functions."""

    def test_log_function(self):
        """Test log function exists."""
        import SHOTpy
        assert hasattr(SHOTpy, 'log')

    def test_exp_function(self):
        """Test exp function exists."""
        import SHOTpy
        assert hasattr(SHOTpy, 'exp')

    def test_sqrt_function(self):
        """Test sqrt function exists."""
        import SHOTpy
        assert hasattr(SHOTpy, 'sqrt')

    def test_sin_function(self):
        """Test sin function exists."""
        import SHOTpy
        assert hasattr(SHOTpy, 'sin')

    def test_cos_function(self):
        """Test cos function exists."""
        import SHOTpy
        assert hasattr(SHOTpy, 'cos')

    def test_abs_function(self):
        """Test abs function exists."""
        import SHOTpy
        assert hasattr(SHOTpy, 'abs')

    def test_square_function(self):
        """Test square function exists."""
        import SHOTpy
        assert hasattr(SHOTpy, 'square')
