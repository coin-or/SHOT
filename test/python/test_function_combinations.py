"""
Tests for creating problems with different term types and combinations.

This module tests:
- Signomial terms (variables with fractional/negative powers)
- Monomial terms (products of variables)
- Combinations of linear, quadratic, monomial, signomial terms and nonlinear expressions
"""

import pytest
import SHOTpy


class TestSignomialTerms:
    """Tests for signomial term creation and usage."""

    def test_signomial_element_creation(self):
        """Test creating a signomial element with variable and power."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        
        # Create element with fractional power: x^0.5
        elem = SHOTpy.SignomialElement(x, 0.5)
        
        assert elem.variable.name == "x"
        assert elem.power == 0.5

    def test_signomial_element_negative_power(self):
        """Test signomial element with negative power: x^(-1)."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        
        elem = SHOTpy.SignomialElement(x, -1.0)
        
        assert elem.power == -1.0

    def test_signomial_term_single_element(self):
        """Test creating signomial term: 2 * x^0.5."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        elem = SHOTpy.SignomialElement(x, 0.5)
        
        term = SHOTpy.SignomialTerm(2.0, [elem])
        
        assert term.coefficient == 2.0
        assert len(term.elements) == 1

    def test_signomial_term_multiple_elements(self):
        """Test creating signomial term: 3 * x^0.5 * y^(-0.3)."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        
        elem_x = SHOTpy.SignomialElement(x, 0.5)
        elem_y = SHOTpy.SignomialElement(y, -0.3)
        
        term = SHOTpy.SignomialTerm(3.0, [elem_x, elem_y])
        
        assert term.coefficient == 3.0
        assert len(term.elements) == 2

    def test_signomial_term_from_tuples(self):
        """Test creating signomial term from (variable, power) tuples."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        
        # Create term: 5 * x^2.5 * y^(-1)
        term = SHOTpy.SignomialTerm(5.0, [(x, 2.5), (y, -1.0)])
        
        assert term.coefficient == 5.0
        assert len(term.elements) == 2

    def test_signomial_terms_collection(self):
        """Test SignomialTerms collection."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        
        term1 = SHOTpy.SignomialTerm(2.0, [(x, 0.5)])
        term2 = SHOTpy.SignomialTerm(3.0, [(x, -0.5)])
        
        terms = SHOTpy.SignomialTerms()
        terms.add(term1)
        terms.add(term2)
        
        assert len(terms) == 2

    def test_signomial_repr(self):
        """Test string representation of signomial terms."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        term = SHOTpy.SignomialTerm(2.0, [(x, 0.5)])
        
        repr_str = repr(term)
        assert "SignomialTerm" in repr_str
        assert "x" in repr_str


class TestMonomialTerms:
    """Tests for monomial term creation and usage."""

    def test_monomial_term_creation(self):
        """Test creating a monomial term: 2*x*y*z."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, -10.0, 10.0)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, -10.0, 10.0)
        
        term = SHOTpy.MonomialTerm(2.0, [x, y, z])
        
        assert term.coefficient == 2.0
        assert len(term.variables) == 3

    def test_monomial_term_binary(self):
        """Test monomial term with binary variables."""
        b1 = SHOTpy.Variable("b1", 0, SHOTpy.VariableType.Binary, 0.0, 1.0)
        b2 = SHOTpy.Variable("b2", 1, SHOTpy.VariableType.Binary, 0.0, 1.0)
        b3 = SHOTpy.Variable("b3", 2, SHOTpy.VariableType.Binary, 0.0, 1.0)
        
        term = SHOTpy.MonomialTerm(1.0, [b1, b2, b3])
        
        assert term.isBinary == True

    def test_monomial_terms_collection(self):
        """Test MonomialTerms collection."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, -10.0, 10.0)
        
        term1 = SHOTpy.MonomialTerm(2.0, [x, y])
        term2 = SHOTpy.MonomialTerm(3.0, [x, x, y])
        
        terms = SHOTpy.MonomialTerms()
        terms.add(term1)
        terms.add(term2)
        
        assert len(terms) == 2

    def test_monomial_repr(self):
        """Test string representation of monomial terms."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, -10.0, 10.0)
        term = SHOTpy.MonomialTerm(2.0, [x, y])
        
        repr_str = repr(term)
        assert "MonomialTerm" in repr_str
        assert "x" in repr_str
        assert "y" in repr_str


class TestConstraintWithSignomialTerms:
    """Tests for constraints with signomial terms."""

    def test_nonlinear_constraint_add_signomial(self):
        """Test adding signomial terms to a nonlinear constraint."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        
        constraint = SHOTpy.NonlinearConstraint(0, "c1", -SHOTpy.SHOT_DBL_MAX, 10.0)
        
        # Add signomial term: x^0.5 <= 10
        term = SHOTpy.SignomialTerm(1.0, [(x, 0.5)])
        constraint.add(SHOTpy.SignomialTerms())
        constraint.signomialTerms.add(term)
        
        assert len(constraint.signomialTerms) == 1

    def test_nonlinear_constraint_signomial_terms_direct(self):
        """Test adding signomial terms collection to constraint."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        
        constraint = SHOTpy.NonlinearConstraint(0, "c1", -SHOTpy.SHOT_DBL_MAX, 5.0)
        
        terms = SHOTpy.SignomialTerms()
        terms.add(SHOTpy.SignomialTerm(1.0, [(x, 0.5)]))
        terms.add(SHOTpy.SignomialTerm(2.0, [(y, -0.5)]))
        
        constraint.add(terms)
        
        assert len(constraint.signomialTerms) == 2


class TestConstraintWithMonomialTerms:
    """Tests for constraints with monomial terms."""

    def test_nonlinear_constraint_add_monomial(self):
        """Test adding monomial terms to a nonlinear constraint."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, -10.0, 10.0)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, -10.0, 10.0)
        
        constraint = SHOTpy.NonlinearConstraint(0, "c1", -SHOTpy.SHOT_DBL_MAX, 100.0)
        
        # Add monomial term: x*y*z <= 100
        term = SHOTpy.MonomialTerm(1.0, [x, y, z])
        constraint.monomialTerms.add(term)
        
        assert len(constraint.monomialTerms) == 1


class TestCombinedTermTypes:
    """Tests for constraints combining different term types."""

    def test_linear_and_quadratic_terms(self):
        """Test constraint with linear and quadratic terms."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, -10.0, 10.0)
        
        # Quadratic constraint inherits from Linear, so can have both
        constraint = SHOTpy.QuadraticConstraint(0, "c1", -SHOTpy.SHOT_DBL_MAX, 10.0)
        
        # Add linear term: 2*x
        constraint.add(SHOTpy.LinearTerm(2.0, x))
        
        # Add quadratic term: x*y
        constraint.add(SHOTpy.QuadraticTerm(1.0, x, y))
        
        assert len(constraint.linearTerms) == 1
        assert len(constraint.quadraticTerms) == 1

    def test_linear_quadratic_and_nonlinear(self):
        """Test nonlinear constraint with linear, quadratic, and nonlinear parts."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        
        # Create with linear and quadratic terms, then add nonlinear expression
        linear_terms = SHOTpy.LinearTerms()
        linear_terms.add(SHOTpy.LinearTerm(3.0, x))
        
        quad_terms = SHOTpy.QuadraticTerms()
        quad_terms.add(SHOTpy.QuadraticTerm(2.0, x, x))  # 2*x^2
        
        # Nonlinear expression: log(y)
        expr = SHOTpy.log(y)
        
        constraint = SHOTpy.NonlinearConstraint(0, "c1", linear_terms, quad_terms, expr, 
                                                 -SHOTpy.SHOT_DBL_MAX, 20.0)
        
        assert len(constraint.linearTerms) == 1
        assert len(constraint.quadraticTerms) == 1
        assert constraint.nonlinearExpression is not None

    def test_all_term_types_combined(self):
        """Test constraint with all term types: linear, quadratic, monomial, signomial, nonlinear."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 0.1, 10.0)
        
        # Start with empty nonlinear constraint
        constraint = SHOTpy.NonlinearConstraint(0, "c1", -SHOTpy.SHOT_DBL_MAX, 100.0)
        
        # Add linear term: 2*x
        constraint.add(SHOTpy.LinearTerm(2.0, x))
        
        # Add quadratic term: x*y
        constraint.add(SHOTpy.QuadraticTerm(1.0, x, y))
        
        # Add monomial term: x*y*z
        constraint.add(SHOTpy.MonomialTerm(0.5, [x, y, z]))
        
        # Add signomial term: x^0.5 * y^(-0.5)
        constraint.add(SHOTpy.SignomialTerm(1.0, [(x, 0.5), (y, -0.5)]))
        
        # Add nonlinear expression: log(z)
        constraint.add(SHOTpy.log(z))
        
        assert len(constraint.linearTerms) == 1
        assert len(constraint.quadraticTerms) == 1
        assert len(constraint.monomialTerms) == 1
        assert len(constraint.signomialTerms) == 1
        assert constraint.nonlinearExpression is not None

    def test_multiple_terms_each_type(self):
        """Test constraint with multiple terms of each type."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 0.1, 10.0)
        
        constraint = SHOTpy.NonlinearConstraint(0, "c1", 0.0, 50.0)
        
        # Multiple linear terms
        constraint.add(SHOTpy.LinearTerm(1.0, x))
        constraint.add(SHOTpy.LinearTerm(2.0, y))
        constraint.add(SHOTpy.LinearTerm(3.0, z))
        
        # Multiple quadratic terms
        constraint.add(SHOTpy.QuadraticTerm(1.0, x, x))
        constraint.add(SHOTpy.QuadraticTerm(2.0, y, y))
        constraint.add(SHOTpy.QuadraticTerm(-1.0, x, y))
        
        # Multiple signomial terms
        constraint.add(SHOTpy.SignomialTerm(0.5, [(x, 0.5)]))
        constraint.add(SHOTpy.SignomialTerm(0.3, [(y, -0.5)]))
        
        assert len(constraint.linearTerms) == 3
        assert len(constraint.quadraticTerms) == 3
        assert len(constraint.signomialTerms) == 2


class TestObjectiveWithTermTypes:
    """Tests for objectives with different term types."""

    def test_linear_objective(self):
        """Test linear objective function."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, -10.0, 10.0)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        obj.add(SHOTpy.LinearTerm(2.0, y))
        
        assert len(obj.linearTerms) == 2

    def test_quadratic_objective(self):
        """Test quadratic objective function."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, -10.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, -10.0, 10.0)
        
        obj = SHOTpy.QuadraticObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        obj.add(SHOTpy.QuadraticTerm(1.0, x, x))
        obj.add(SHOTpy.QuadraticTerm(2.0, x, y))
        
        assert len(obj.linearTerms) == 1
        assert len(obj.quadraticTerms) == 2

    def test_nonlinear_objective_with_signomial(self):
        """Test nonlinear objective with signomial terms."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        
        # Add linear part
        obj.add(SHOTpy.LinearTerm(1.0, x))
        
        # Add signomial: x^0.5 * y^(-1)
        obj.add(SHOTpy.SignomialTerm(2.0, [(x, 0.5), (y, -1.0)]))
        
        assert len(obj.linearTerms) == 1
        assert len(obj.signomialTerms) == 1

    def test_nonlinear_objective_all_term_types(self):
        """Test nonlinear objective with all term types."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 0.1, 10.0)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        
        # Linear: x + 2y
        obj.add(SHOTpy.LinearTerm(1.0, x))
        obj.add(SHOTpy.LinearTerm(2.0, y))
        
        # Quadratic: x^2
        obj.add(SHOTpy.QuadraticTerm(1.0, x, x))
        
        # Signomial: x^0.5
        obj.add(SHOTpy.SignomialTerm(1.0, [(x, 0.5)]))
        
        # Monomial: x*y*z
        obj.add(SHOTpy.MonomialTerm(0.1, [x, y, z]))
        
        # Nonlinear: log(z)
        obj.add(SHOTpy.log(z))
        
        assert len(obj.linearTerms) == 2
        assert len(obj.quadraticTerms) == 1
        assert len(obj.signomialTerms) == 1
        assert len(obj.monomialTerms) == 1
        assert obj.nonlinearExpression is not None


class TestProblemWithMixedTerms:
    """Tests for complete problems with mixed term types."""

    def test_problem_with_signomial_constraint(self, solver, env):
        """Test creating and solving a problem with signomial constraints."""
        import SHOTpy
        
        # Create problem with environment
        problem = SHOTpy.Problem(env)
        problem.name = "signomial_test"
        
        # Variables with positive bounds (required for signomials with fractional powers)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # Linear objective: minimize x + y
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        obj.add(SHOTpy.LinearTerm(1.0, y))
        problem.setObjective(obj)
        
        # Signomial constraint: x^0.5 + y^0.5 >= 2
        constraint = SHOTpy.NonlinearConstraint(0, "c1", 2.0, SHOTpy.SHOT_DBL_MAX)
        constraint.add(SHOTpy.SignomialTerm(1.0, [(x, 0.5)]))
        constraint.add(SHOTpy.SignomialTerm(1.0, [(y, 0.5)]))
        problem.addConstraint(constraint)
        
        problem.finalize()
        
        assert problem.properties.isNonlinear

    def test_problem_with_mixed_constraints(self, solver, env):
        """Test problem with linear, quadratic, and nonlinear constraints."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        problem.name = "mixed_constraints"
        
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Integer, 0, 5)
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        # Objective: minimize x + y + z
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        obj.add(SHOTpy.LinearTerm(1.0, y))
        obj.add(SHOTpy.LinearTerm(1.0, z))
        problem.setObjective(obj)
        
        # Linear constraint: x + y >= 1
        c1 = SHOTpy.LinearConstraint(0, "linear", 1.0, SHOTpy.SHOT_DBL_MAX)
        c1.add(SHOTpy.LinearTerm(1.0, x))
        c1.add(SHOTpy.LinearTerm(1.0, y))
        problem.addConstraint(c1)
        
        # Quadratic constraint: x^2 + y^2 <= 10
        c2 = SHOTpy.QuadraticConstraint(1, "quadratic", -SHOTpy.SHOT_DBL_MAX, 10.0)
        c2.add(SHOTpy.QuadraticTerm(1.0, x, x))
        c2.add(SHOTpy.QuadraticTerm(1.0, y, y))
        problem.addConstraint(c2)
        
        # Nonlinear constraint with signomial: x^0.5 * y^0.5 >= 0.5
        c3 = SHOTpy.NonlinearConstraint(2, "signomial", 0.5, SHOTpy.SHOT_DBL_MAX)
        c3.add(SHOTpy.SignomialTerm(1.0, [(x, 0.5), (y, 0.5)]))
        problem.addConstraint(c3)
        
        problem.finalize()
        
        # Should be MINLP due to integer variable and nonlinear constraints
        assert problem.properties.numberOfLinearConstraints >= 1
        assert problem.properties.numberOfQuadraticConstraints >= 1
        assert problem.properties.numberOfNonlinearConstraints >= 1
        assert problem.properties.isDiscrete

    def test_geometric_programming_style(self, solver, env):
        """Test a geometric programming style problem with signomials."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        problem.name = "geometric_prog"
        
        # Variables (must be positive for GP)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.01, 100.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.01, 100.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # Objective: minimize x^2 * y^(-1)
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.SignomialTerm(1.0, [(x, 2.0), (y, -1.0)]))
        problem.setObjective(obj)
        
        # Constraint: x^(-1) + y^(-1) <= 1
        c1 = SHOTpy.NonlinearConstraint(0, "c1", -SHOTpy.SHOT_DBL_MAX, 1.0)
        c1.add(SHOTpy.SignomialTerm(1.0, [(x, -1.0)]))
        c1.add(SHOTpy.SignomialTerm(1.0, [(y, -1.0)]))
        problem.addConstraint(c1)
        
        # Constraint: x * y >= 1
        c2 = SHOTpy.NonlinearConstraint(1, "c2", 1.0, SHOTpy.SHOT_DBL_MAX)
        c2.add(SHOTpy.SignomialTerm(1.0, [(x, 1.0), (y, 1.0)]))
        problem.addConstraint(c2)
        
        problem.finalize()
        
        assert problem.properties.isNonlinear


class TestTermsWithExpressions:
    """Tests combining terms with nonlinear expressions."""

    def test_expression_plus_linear_terms(self):
        """Test nonlinear expression combined with linear terms in constraint."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        
        # Create: log(x) + 2*y <= 5
        linear_terms = SHOTpy.LinearTerms()
        linear_terms.add(SHOTpy.LinearTerm(2.0, y))
        
        expr = SHOTpy.log(x)
        
        constraint = SHOTpy.NonlinearConstraint(0, "c1", linear_terms, expr, 
                                                 -SHOTpy.SHOT_DBL_MAX, 5.0)
        
        assert len(constraint.linearTerms) == 1
        assert constraint.nonlinearExpression is not None

    def test_expression_plus_quadratic_terms(self):
        """Test nonlinear expression combined with quadratic terms."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        
        linear_terms = SHOTpy.LinearTerms()
        linear_terms.add(SHOTpy.LinearTerm(1.0, x))
        
        quad_terms = SHOTpy.QuadraticTerms()
        quad_terms.add(SHOTpy.QuadraticTerm(1.0, y, y))
        
        expr = SHOTpy.exp(x)
        
        constraint = SHOTpy.NonlinearConstraint(0, "c1", linear_terms, quad_terms, expr,
                                                 -SHOTpy.SHOT_DBL_MAX, 100.0)
        
        assert len(constraint.linearTerms) == 1
        assert len(constraint.quadraticTerms) == 1
        assert constraint.nonlinearExpression is not None

    def test_complex_expression_with_all_terms(self):
        """Test complex constraint with expression and all term types."""
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 0.1, 10.0)
        
        constraint = SHOTpy.NonlinearConstraint(0, "complex", 0.0, 100.0)
        
        # Linear: x
        constraint.add(SHOTpy.LinearTerm(1.0, x))
        
        # Quadratic: y^2
        constraint.add(SHOTpy.QuadraticTerm(1.0, y, y))
        
        # Signomial: x^0.5 * z^(-0.5)
        constraint.add(SHOTpy.SignomialTerm(1.0, [(x, 0.5), (z, -0.5)]))
        
        # Monomial: x*y*z
        constraint.add(SHOTpy.MonomialTerm(0.1, [x, y, z]))
        
        # Nonlinear expression: sin(x) + cos(y)
        constraint.add(SHOTpy.sin(x) + SHOTpy.cos(y))
        
        assert len(constraint.linearTerms) == 1
        assert len(constraint.quadraticTerms) == 1
        assert len(constraint.signomialTerms) == 1
        assert len(constraint.monomialTerms) == 1
        assert constraint.nonlinearExpression is not None


class TestTermTypeIndicatorsInOutput:
    """Tests that verify term types are correctly registered by checking problem output.
    
    SHOT's problem output uses indicators to show term types:
    - [L    ] = Linear terms only
    - [ Q   ] = Quadratic terms
    - [  M  ] = Monomial terms  
    - [   S ] = Signomial terms
    - [    E] = Nonlinear expression (general)
    """

    def test_linear_constraint_indicator(self, solver, env):
        """Test that linear constraints show 'L' indicator."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "linear_test", 1.0, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        # Find the line containing our constraint name
        for line in output.split('\n'):
            if 'linear_test' in line:
                # Should have [L    ] indicator (linear only)
                assert '[L    ]' in line, f"Expected linear indicator in: {line}"
                break
        else:
            assert False, "Constraint 'linear_test' not found in output"

    def test_quadratic_constraint_indicator(self, solver, env):
        """Test that quadratic constraints show 'Q' indicator."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        c = SHOTpy.QuadraticConstraint(0, "quadratic_test", -SHOTpy.SHOT_DBL_MAX, 10.0)
        c.add(SHOTpy.QuadraticTerm(1.0, x, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        for line in output.split('\n'):
            if 'quadratic_test' in line:
                # Should have Q indicator
                assert 'Q' in line and '[' in line, f"Expected quadratic indicator in: {line}"
                # Should NOT have S or E indicators
                assert 'S' not in line.split(']')[0].split('[')[-1], f"Unexpected signomial in: {line}"
                break
        else:
            assert False, "Constraint 'quadratic_test' not found in output"

    def test_signomial_constraint_indicator(self, solver, env):
        """Test that signomial constraints show 'S' indicator."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        c = SHOTpy.NonlinearConstraint(0, "signomial_test", 0.5, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.SignomialTerm(1.0, [(x, 0.5)]))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        for line in output.split('\n'):
            if 'signomial_test' in line:
                # Should have S indicator
                assert 'S' in line, f"Expected signomial indicator 'S' in: {line}"
                break
        else:
            assert False, "Constraint 'signomial_test' not found in output"

    def test_monomial_constraint_indicator(self, solver, env):
        """Test that monomial constraints show 'M' indicator."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        # Monomial: x * y^2
        c = SHOTpy.NonlinearConstraint(0, "monomial_test", -SHOTpy.SHOT_DBL_MAX, 10.0)
        c.add(SHOTpy.MonomialTerm(1.0, [x, y, y]))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        for line in output.split('\n'):
            if 'monomial_test' in line:
                # Should have M indicator [  M  ]
                assert 'M' in line, f"Expected monomial indicator 'M' in: {line}"
                break
        else:
            assert False, "Constraint 'monomial_test' not found in output"

    def test_nonlinear_expression_constraint_indicator(self, solver, env):
        """Test that nonlinear expression constraints show 'E' indicator."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        c = SHOTpy.NonlinearConstraint(0, "expression_test", -SHOTpy.SHOT_DBL_MAX, 5.0)
        c.add(SHOTpy.log(x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        for line in output.split('\n'):
            if 'expression_test' in line:
                # Should have E indicator for nonlinear expression
                assert 'E' in line, f"Expected expression indicator 'E' in: {line}"
                break
        else:
            assert False, "Constraint 'expression_test' not found in output"

    def test_quadratic_objective_indicator(self, solver, env):
        """Test that quadratic objective shows 'Q' indicator."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        obj = SHOTpy.QuadraticObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.QuadraticTerm(1.0, x, x))
        problem.setObjective(obj)
        
        # Need at least one constraint
        c = SHOTpy.LinearConstraint(0, "dummy", 0.0, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        # Find the objective line (after "minimize:" or "maximize:")
        lines = output.split('\n')
        for i, line in enumerate(lines):
            if 'minimize:' in line.lower() or 'maximize:' in line.lower():
                # The objective is on the next line
                obj_line = lines[i + 1] if i + 1 < len(lines) else ""
                assert 'Q' in obj_line, f"Expected quadratic indicator 'Q' in objective: {obj_line}"
                break

    def test_signomial_objective_indicator(self, solver, env):
        """Test that signomial objective shows 'S' indicator."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.SignomialTerm(1.0, [(x, 0.5)]))
        problem.setObjective(obj)
        
        # Need at least one constraint
        c = SHOTpy.LinearConstraint(0, "dummy", 0.1, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        lines = output.split('\n')
        for i, line in enumerate(lines):
            if 'minimize:' in line.lower() or 'maximize:' in line.lower():
                obj_line = lines[i + 1] if i + 1 < len(lines) else ""
                assert 'S' in obj_line, f"Expected signomial indicator 'S' in objective: {obj_line}"
                break

    def test_linear_objective_indicator(self, solver, env):
        """Test that linear objective shows 'L' indicator."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "dummy", 0.0, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        lines = output.split('\n')
        for i, line in enumerate(lines):
            if 'minimize:' in line.lower() or 'maximize:' in line.lower():
                obj_line = lines[i + 1] if i + 1 < len(lines) else ""
                assert '[L    ]' in obj_line, f"Expected linear indicator '[L    ]' in objective: {obj_line}"
                break

    def test_nonlinear_expression_objective_indicator(self, solver, env):
        """Test that nonlinear expression objective shows 'E' indicator."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.log(x))
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "dummy", 0.1, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        lines = output.split('\n')
        for i, line in enumerate(lines):
            if 'minimize:' in line.lower() or 'maximize:' in line.lower():
                obj_line = lines[i + 1] if i + 1 < len(lines) else ""
                assert 'E' in obj_line, f"Expected expression indicator 'E' in objective: {obj_line}"
                break

    def test_monomial_objective_indicator(self, solver, env):
        """Test that monomial objective shows 'M' indicator."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # Monomial objective: x * y^2
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.MonomialTerm(1.0, [x, y, y]))
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "dummy", 0.1, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        lines = output.split('\n')
        for i, line in enumerate(lines):
            if 'minimize:' in line.lower() or 'maximize:' in line.lower():
                obj_line = lines[i + 1] if i + 1 < len(lines) else ""
                assert 'M' in obj_line, f"Expected monomial indicator 'M' in objective: {obj_line}"
                break

    def test_mixed_objective_shows_all_indicators(self, solver, env):
        """Test that an objective with multiple term types shows all relevant indicators."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # Objective with linear + quadratic + signomial terms
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))           # Linear
        obj.add(SHOTpy.QuadraticTerm(1.0, y, y))     # Quadratic
        obj.add(SHOTpy.SignomialTerm(1.0, [(x, 0.5)])) # Signomial
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "dummy", 0.1, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        lines = output.split('\n')
        for i, line in enumerate(lines):
            if 'minimize:' in line.lower() or 'maximize:' in line.lower():
                obj_line = lines[i + 1] if i + 1 < len(lines) else ""
                assert 'L' in obj_line, f"Expected linear indicator 'L' in objective: {obj_line}"
                assert 'Q' in obj_line, f"Expected quadratic indicator 'Q' in objective: {obj_line}"
                assert 'S' in obj_line, f"Expected signomial indicator 'S' in objective: {obj_line}"
                break

    def test_all_objective_types(self, solver, env):
        """Test creating problems with each objective type and verify indicators."""
        import SHOTpy
        
        # Test cases: (objective_factory, expected_indicator, description, needs_two_vars)
        test_cases = [
            ('linear', 'L', 'Linear objective', False),
            ('quadratic', 'Q', 'Quadratic objective', False),
            ('monomial', 'M', 'Monomial objective', True),
            ('signomial', 'S', 'Signomial objective', False),
            ('expression', 'E', 'Expression objective', False),
        ]
        
        for obj_type, expected_indicator, description, needs_two_vars in test_cases:
            problem = SHOTpy.Problem(env)
            x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
            problem.addVariable(x)
            
            if needs_two_vars:
                y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
                problem.addVariable(y)
            
            if obj_type == 'linear':
                obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
                obj.add(SHOTpy.LinearTerm(1.0, x))
            elif obj_type == 'quadratic':
                obj = SHOTpy.QuadraticObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
                obj.add(SHOTpy.QuadraticTerm(1.0, x, x))
            elif obj_type == 'monomial':
                obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
                obj.add(SHOTpy.MonomialTerm(1.0, [x, y, y]))  # x * y^2
            elif obj_type == 'signomial':
                obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
                obj.add(SHOTpy.SignomialTerm(1.0, [(x, 0.5)]))
            else:  # expression
                obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
                obj.add(SHOTpy.log(x))
            
            problem.setObjective(obj)
            
            c = SHOTpy.LinearConstraint(0, "dummy", 0.1, SHOTpy.SHOT_DBL_MAX)
            c.add(SHOTpy.LinearTerm(1.0, x))
            problem.addConstraint(c)
            
            problem.finalize()
            output = problem.toString()
            
            lines = output.split('\n')
            for i, line in enumerate(lines):
                if 'minimize:' in line.lower() or 'maximize:' in line.lower():
                    obj_line = lines[i + 1] if i + 1 < len(lines) else ""
                    assert expected_indicator in obj_line, \
                        f"{description}: Expected '{expected_indicator}' indicator in: {obj_line}"
                    break

    def test_mixed_constraint_shows_all_indicators(self, solver, env):
        """Test that a constraint with multiple term types shows all relevant indicators."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        # Constraint with linear + quadratic + signomial terms
        c = SHOTpy.NonlinearConstraint(0, "mixed_test", -SHOTpy.SHOT_DBL_MAX, 100.0)
        c.add(SHOTpy.LinearTerm(1.0, x))           # Linear
        c.add(SHOTpy.QuadraticTerm(1.0, y, y))     # Quadratic
        c.add(SHOTpy.SignomialTerm(1.0, [(x, 0.5)])) # Signomial
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        for line in output.split('\n'):
            if 'mixed_test' in line:
                # Should have L, Q, and S indicators
                # The indicator section is typically [LQMSE] format
                assert 'L' in line, f"Expected linear indicator 'L' in: {line}"
                assert 'Q' in line, f"Expected quadratic indicator 'Q' in: {line}"
                assert 'S' in line, f"Expected signomial indicator 'S' in: {line}"
                break
        else:
            assert False, "Constraint 'mixed_test' not found in output"

    def test_all_constraint_types_in_one_problem(self, solver, env):
        """Test a problem with all constraint types shows correct indicators."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        # Linear constraint
        c1 = SHOTpy.LinearConstraint(0, "c_linear", 0.1, SHOTpy.SHOT_DBL_MAX)
        c1.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c1)
        
        # Quadratic constraint
        c2 = SHOTpy.QuadraticConstraint(1, "c_quadratic", -SHOTpy.SHOT_DBL_MAX, 50.0)
        c2.add(SHOTpy.QuadraticTerm(1.0, x, x))
        problem.addConstraint(c2)
        
        # Monomial constraint
        c3 = SHOTpy.NonlinearConstraint(2, "c_monomial", -SHOTpy.SHOT_DBL_MAX, 50.0)
        c3.add(SHOTpy.MonomialTerm(1.0, [x, y, y]))  # x * y^2
        problem.addConstraint(c3)
        
        # Signomial constraint
        c4 = SHOTpy.NonlinearConstraint(3, "c_signomial", 0.1, SHOTpy.SHOT_DBL_MAX)
        c4.add(SHOTpy.SignomialTerm(1.0, [(x, 0.5)]))
        problem.addConstraint(c4)
        
        # Nonlinear expression constraint
        c5 = SHOTpy.NonlinearConstraint(4, "c_expression", -SHOTpy.SHOT_DBL_MAX, 3.0)
        c5.add(SHOTpy.log(x))
        problem.addConstraint(c5)
        
        problem.finalize()
        output = problem.toString()
        
        indicators_found = {'L': False, 'Q': False, 'M': False, 'S': False, 'E': False}
        
        for line in output.split('\n'):
            if 'c_linear' in line:
                assert '[L    ]' in line, f"Linear constraint should have [L    ]: {line}"
                indicators_found['L'] = True
            elif 'c_quadratic' in line:
                assert 'Q' in line, f"Quadratic constraint should have Q: {line}"
                indicators_found['Q'] = True
            elif 'c_monomial' in line:
                assert 'M' in line, f"Monomial constraint should have M: {line}"
                indicators_found['M'] = True
            elif 'c_signomial' in line:
                assert 'S' in line, f"Signomial constraint should have S: {line}"
                indicators_found['S'] = True
            elif 'c_expression' in line:
                assert 'E' in line, f"Expression constraint should have E: {line}"
                indicators_found['E'] = True
        
        # Verify all constraint types were found
        assert all(indicators_found.values()), f"Not all indicators found: {indicators_found}"


class TestExpressionSimplification:
    """Tests for algebraic simplification of expressions during finalize()."""

    def test_exp_log_simplifies_to_identity(self, solver, env):
        """Test that exp(log(x)) simplifies to x for x > 0."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        # x must be > 0 for log(x) to be valid
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        
        # exp(log(x)) should simplify to x
        expr = SHOTpy.exp(SHOTpy.log(x))
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 0.1, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        # After simplification: exp(log(x)) = x
        # The output should show just x (linear), not exp or log
        assert 'exp' not in output.lower(), f"exp should be simplified away: {output}"
        assert 'log' not in output.lower() and 'ln' not in output.lower(), f"log should be simplified away: {output}"
        # Should be linear now (just x)
        assert '[L' in output, f"exp(log(x)) should simplify to linear x: {output}"

    def test_log_exp_simplifies_to_identity(self, solver, env):
        """Test that log(exp(y)) simplifies to y for all y."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        # y can be any real value
        y = SHOTpy.Variable("y", 0, SHOTpy.VariableType.Real, -5.0, 5.0)
        problem.addVariable(y)
        
        # log(exp(y)) should simplify to y
        expr = SHOTpy.log(SHOTpy.exp(y))
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", -5.0, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, y))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        # After simplification: log(exp(y)) = y
        # The output should show just y (linear), not exp or log
        assert 'exp' not in output.lower(), f"exp should be simplified away: {output}"
        assert 'log' not in output.lower() and 'ln' not in output.lower(), f"log should be simplified away: {output}"
        # Should be linear now (just y)
        assert '[L' in output, f"log(exp(y)) should simplify to linear y: {output}"

    def test_negative_plus_positive_cancels_to_zero(self, solver, env):
        """Test that -x + x simplifies to 0 (linear term cancellation)."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        # -x + x should cancel to 0
        expr = -x + x
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 0.0, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        # After simplification: -x + x = 0
        # The objective should have empty term type indicators [     ] (no L, Q, M, S, E)
        for line in output.split('\n'):
            if 'minimize' in line.lower() or 'maximize' in line.lower():
                continue
            if line.strip().startswith('[') and 'bound' not in line:
                # This is the objective line - check that term indicators are empty
                # [     ] means no terms (everything cancelled)
                assert '[     ]' in line, f"-x + x should cancel to 0 (empty terms): {line}"
                break

    def test_positive_minus_same_cancels_to_zero(self, solver, env):
        """Test that x - x simplifies to 0."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        # x - x should cancel to 0
        expr = x - x
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 0.0, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        # After simplification: x - x = 0
        # The objective should have empty term type indicators [     ] (no L, Q, M, S, E)
        for line in output.split('\n'):
            if 'minimize' in line.lower() or 'maximize' in line.lower():
                continue
            if line.strip().startswith('[') and 'bound' not in line:
                # This is the objective line - check that term indicators are empty
                assert '[     ]' in line, f"x - x should cancel to 0 (empty terms): {line}"
                break

    def test_linear_term_combination(self, solver, env):
        """Test that 2*x + 3*x simplifies to 5*x."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        # 2*x + 3*x should combine to 5*x
        expr = 2.0 * x + 3.0 * x
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 0.0, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        # Should be linear with combined coefficient
        assert '[L' in output, f"2*x + 3*x should be linear: {output}"
        # Check that 5*x appears (or +5*x)
        assert '5' in output and 'x' in output, f"Should have coefficient 5: {output}"

    def test_exp_log_nested_in_expression(self, solver, env):
        """Test exp(log(x)) + y simplifies to x + y."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # exp(log(x)) + y should simplify to x + y
        expr = SHOTpy.exp(SHOTpy.log(x)) + y
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 0.1, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        # After simplification should be linear (x + y)
        assert 'exp' not in output.lower(), f"exp should be simplified away: {output}"
        assert 'log' not in output.lower() and 'ln' not in output.lower(), f"log should be simplified away: {output}"
        assert '[L' in output, f"exp(log(x)) + y should simplify to linear: {output}"


class TestDeepNonlinearExpressions:
    """Tests for deeply nested nonlinear expressions with multiple levels and variables."""

    def test_three_level_nested_expression(self, solver, env):
        """Test expression with 3 levels of nesting: log(exp(sin(x))) simplifies to sin(x)."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 3.0)
        problem.addVariable(x)
        
        # 3 levels: log(exp(sin(x))) - this simplifies to sin(x) because log(exp(y)) = y
        expr = SHOTpy.log(SHOTpy.exp(SHOTpy.sin(x)))
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 0.1, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        # After simplification: log(exp(sin(x))) = sin(x)
        # The simplifier correctly reduces log(exp(y)) to y
        assert 'sin' in output.lower(), f"Expected sin(x) after simplification: {output}"

    def test_four_level_nested_expression(self, solver, env):
        """Test expression with 4 levels of nesting: sqrt(abs(cos(log(x))))."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 1.0, 10.0)
        problem.addVariable(x)
        
        # 4 levels: sqrt(abs(cos(log(x))))
        expr = SHOTpy.sqrt(SHOTpy.abs(SHOTpy.cos(SHOTpy.log(x))))
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 1.0, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        # Verify the expression is recognized as nonlinear
        lines = output.split('\n')
        for i, line in enumerate(lines):
            if 'minimize:' in line.lower() or 'maximize:' in line.lower():
                obj_line = lines[i + 1] if i + 1 < len(lines) else ""
                assert 'E' in obj_line, f"Deep nested expression should have 'E' indicator: {obj_line}"
                break

    def test_expression_with_three_variables(self, solver, env):
        """Test expression combining three variables: x * sin(y) + log(z)."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 3.14)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 1.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        # x * sin(y) + log(z)
        expr = x * SHOTpy.sin(y) + SHOTpy.log(z)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 0.1, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        # All three variables should appear in the problem
        assert 'x' in output
        assert 'y' in output
        assert 'z' in output

    def test_expression_with_four_variables(self, solver, env):
        """Test expression with four variables: exp(a) * cos(b) + sin(c) * log(d)."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        a = SHOTpy.Variable("a", 0, SHOTpy.VariableType.Real, 0.1, 2.0)
        b = SHOTpy.Variable("b", 1, SHOTpy.VariableType.Real, 0.1, 3.14)
        c = SHOTpy.Variable("c", 2, SHOTpy.VariableType.Real, 0.1, 3.14)
        d = SHOTpy.Variable("d", 3, SHOTpy.VariableType.Real, 1.0, 10.0)
        problem.addVariable(a)
        problem.addVariable(b)
        problem.addVariable(c)
        problem.addVariable(d)
        
        # exp(a) * cos(b) + sin(c) * log(d)
        expr = SHOTpy.exp(a) * SHOTpy.cos(b) + SHOTpy.sin(c) * SHOTpy.log(d)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        constraint = SHOTpy.LinearConstraint(0, "bound", 0.1, SHOTpy.SHOT_DBL_MAX)
        constraint.add(SHOTpy.LinearTerm(1.0, a))
        problem.addConstraint(constraint)
        
        problem.finalize()
        output = problem.toString()
        
        # All four variables should appear
        for var_name in ['a', 'b', 'c', 'd']:
            assert var_name in output, f"Variable {var_name} should appear in problem"

    def test_deeply_nested_with_arithmetic(self, solver, env):
        """Test deep nesting combined with arithmetic: (log(x+1) * exp(y-1)) / (sin(z)+2)."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 2.0)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 0.1, 3.14)
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        # (log(x+1) * exp(y-1)) / (sin(z)+2)
        expr = (SHOTpy.log(x + 1.0) * SHOTpy.exp(y - 1.0)) / (SHOTpy.sin(z) + 2.0)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 0.1, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        # Should be recognized as expression type
        lines = output.split('\n')
        for i, line in enumerate(lines):
            if 'minimize:' in line.lower() or 'maximize:' in line.lower():
                obj_line = lines[i + 1] if i + 1 < len(lines) else ""
                assert 'E' in obj_line, f"Complex expression should have 'E' indicator: {obj_line}"
                break

    def test_power_expressions_nested(self, solver, env):
        """Test nested power expressions: (x^2 + y^2)^0.5 (Euclidean norm)."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        # Euclidean norm: sqrt(x^2 + y^2)
        expr = SHOTpy.sqrt(x**2 + y**2)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 0.1, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        assert 'x' in output
        assert 'y' in output

    def test_constraint_with_deep_expression(self, solver, env):
        """Test constraint with deeply nested expression."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        # Constraint: log(x * exp(y) + sin(z)) <= 5
        expr = SHOTpy.log(x * SHOTpy.exp(y) + SHOTpy.sin(z))
        c = SHOTpy.NonlinearConstraint(0, "deep_constraint", -SHOTpy.SHOT_DBL_MAX, 5.0)
        c.add(expr)
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        for line in output.split('\n'):
            if 'deep_constraint' in line:
                assert 'E' in line, f"Deep constraint should have 'E' indicator: {line}"
                break
        else:
            assert False, "Constraint 'deep_constraint' not found in output"

    def test_multiple_deep_constraints(self, solver, env):
        """Test multiple constraints with different deep expressions."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 0.1, 3.14)
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        obj.add(SHOTpy.LinearTerm(1.0, y))
        obj.add(SHOTpy.LinearTerm(1.0, z))
        problem.setObjective(obj)
        
        # Constraint 1: exp(log(x) + log(y)) <= 50
        c1 = SHOTpy.NonlinearConstraint(0, "c1_deep", -SHOTpy.SHOT_DBL_MAX, 50.0)
        c1.add(SHOTpy.exp(SHOTpy.log(x) + SHOTpy.log(y)))
        problem.addConstraint(c1)
        
        # Constraint 2: sin(cos(z)) + x >= 0.5
        c2 = SHOTpy.NonlinearConstraint(1, "c2_deep", 0.5, SHOTpy.SHOT_DBL_MAX)
        c2.add(SHOTpy.sin(SHOTpy.cos(z)))
        c2.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c2)
        
        # Constraint 3: exp(x) + log(y) + sin(z) <= 15
        c3 = SHOTpy.NonlinearConstraint(2, "c3_deep", -SHOTpy.SHOT_DBL_MAX, 15.0)
        c3.add(SHOTpy.exp(x) + SHOTpy.log(y) + SHOTpy.sin(z))
        problem.addConstraint(c3)
        
        problem.finalize()
        output = problem.toString()
        
        constraints_found = {'c1_deep': False, 'c2_deep': False, 'c3_deep': False}
        for line in output.split('\n'):
            for cname in constraints_found:
                if cname in line:
                    assert 'E' in line, f"Constraint {cname} should have 'E' indicator: {line}"
                    constraints_found[cname] = True
        
        assert all(constraints_found.values()), f"Not all constraints found: {constraints_found}"

    def test_five_variables_complex_expression(self, solver, env):
        """Test complex expression with five variables."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        v1 = SHOTpy.Variable("v1", 0, SHOTpy.VariableType.Real, 0.1, 5.0)
        v2 = SHOTpy.Variable("v2", 1, SHOTpy.VariableType.Real, 0.1, 5.0)
        v3 = SHOTpy.Variable("v3", 2, SHOTpy.VariableType.Real, 0.1, 3.14)
        v4 = SHOTpy.Variable("v4", 3, SHOTpy.VariableType.Real, 1.0, 10.0)
        v5 = SHOTpy.Variable("v5", 4, SHOTpy.VariableType.Real, 0.1, 5.0)
        problem.addVariable(v1)
        problem.addVariable(v2)
        problem.addVariable(v3)
        problem.addVariable(v4)
        problem.addVariable(v5)
        
        # Complex: exp(v1) * sin(v2 + v3) + log(v4) * cos(v5)
        expr = SHOTpy.exp(v1) * SHOTpy.sin(v2 + v3) + SHOTpy.log(v4) * SHOTpy.cos(v5)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 0.1, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, v1))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        # All five variables should appear
        for var_name in ['v1', 'v2', 'v3', 'v4', 'v5']:
            assert var_name in output, f"Variable {var_name} should appear in problem"

    def test_expression_chain_all_trig_functions(self, solver, env):
        """Test chaining all trigonometric functions."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 1.0)
        problem.addVariable(x)
        
        # Chain: sin(cos(tan(x))) - requires tan if available, else use sin(cos(sin(x)))
        # Using what's available: sin(cos(sin(x)))
        try:
            expr = SHOTpy.sin(SHOTpy.cos(SHOTpy.tan(x)))
        except AttributeError:
            # tan might not be available, use sin instead
            expr = SHOTpy.sin(SHOTpy.cos(SHOTpy.sin(x)))
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 0.1, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        # Should have expression indicator
        lines = output.split('\n')
        for i, line in enumerate(lines):
            if 'minimize:' in line.lower() or 'maximize:' in line.lower():
                obj_line = lines[i + 1] if i + 1 < len(lines) else ""
                assert 'E' in obj_line, f"Trig chain should have 'E' indicator: {obj_line}"
                break

    def test_expression_with_subtraction_and_division(self, solver, env):
        """Test expressions with subtraction and division operations."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 1.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 1.0, 10.0)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 1.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        # (log(x) - sin(y)) / (exp(z) - 1)
        expr = (SHOTpy.log(x) - SHOTpy.sin(y)) / (SHOTpy.exp(z) - 1.0)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 1.0, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        assert 'x' in output
        assert 'y' in output
        assert 'z' in output


class TestSumsOfNonlinearExpressions:
    """Tests for sums of multiple nonlinear expressions."""

    def test_sum_of_two_expressions(self, solver, env):
        """Test sum of two nonlinear expressions: log(x) + exp(y)."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 1.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 5.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        expr = SHOTpy.log(x) + SHOTpy.exp(y)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 1.0, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        # Verify expression indicator
        lines = output.split('\n')
        for i, line in enumerate(lines):
            if 'minimize:' in line.lower() or 'maximize:' in line.lower():
                obj_line = lines[i + 1] if i + 1 < len(lines) else ""
                assert 'E' in obj_line, f"Sum of expressions should have 'E' indicator: {obj_line}"
                break

    def test_sum_of_three_expressions(self, solver, env):
        """Test sum of three nonlinear expressions: sin(x) + cos(y) + log(z)."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 3.14)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 3.14)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 1.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        expr = SHOTpy.sin(x) + SHOTpy.cos(y) + SHOTpy.log(z)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 0.1, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        assert 'x' in output
        assert 'y' in output
        assert 'z' in output

    def test_sum_of_four_expressions(self, solver, env):
        """Test sum of four nonlinear expressions: exp(a) + sqrt(b) + sin(c) + log(d)."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        a = SHOTpy.Variable("a", 0, SHOTpy.VariableType.Real, 0.1, 2.0)
        b = SHOTpy.Variable("b", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        c = SHOTpy.Variable("c", 2, SHOTpy.VariableType.Real, 0.1, 3.14)
        d = SHOTpy.Variable("d", 3, SHOTpy.VariableType.Real, 1.0, 10.0)
        problem.addVariable(a)
        problem.addVariable(b)
        problem.addVariable(c)
        problem.addVariable(d)
        
        expr = SHOTpy.exp(a) + SHOTpy.sqrt(b) + SHOTpy.sin(c) + SHOTpy.log(d)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        constraint = SHOTpy.LinearConstraint(0, "bound", 0.1, SHOTpy.SHOT_DBL_MAX)
        constraint.add(SHOTpy.LinearTerm(1.0, a))
        problem.addConstraint(constraint)
        
        problem.finalize()
        output = problem.toString()
        
        for var_name in ['a', 'b', 'c', 'd']:
            assert var_name in output, f"Variable {var_name} should appear in problem"

    def test_sum_of_nested_expressions(self, solver, env):
        """Test sum of nested expressions: log(exp(x)) + sin(cos(y)) + sqrt(z^2)."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 5.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 3.14)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        expr = SHOTpy.log(SHOTpy.exp(x)) + SHOTpy.sin(SHOTpy.cos(y)) + SHOTpy.sqrt(z**2)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 0.1, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        lines = output.split('\n')
        for i, line in enumerate(lines):
            if 'minimize:' in line.lower() or 'maximize:' in line.lower():
                obj_line = lines[i + 1] if i + 1 < len(lines) else ""
                assert 'E' in obj_line, f"Nested sum should have 'E' indicator: {obj_line}"
                break

    def test_multiple_add_calls_objective(self, solver, env):
        """Test adding multiple expressions via multiple add() calls on objective."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 1.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 3.14)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 0.1, 5.0)
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.log(x))      # First expression
        obj.add(SHOTpy.sin(y))      # Second expression  
        obj.add(SHOTpy.exp(z))      # Third expression
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 1.0, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        # All three variables should appear
        assert 'x' in output
        assert 'y' in output
        assert 'z' in output

    def test_multiple_add_calls_constraint(self, solver, env):
        """Test adding multiple expressions via multiple add() calls on constraint."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 1.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 3.14)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 0.1, 5.0)
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        c = SHOTpy.NonlinearConstraint(0, "sum_constraint", -SHOTpy.SHOT_DBL_MAX, 10.0)
        c.add(SHOTpy.log(x))        # First expression
        c.add(SHOTpy.sin(y))        # Second expression
        c.add(SHOTpy.exp(z))        # Third expression
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        for line in output.split('\n'):
            if 'sum_constraint' in line:
                assert 'E' in line, f"Sum constraint should have 'E' indicator: {line}"
                break
        else:
            assert False, "Constraint 'sum_constraint' not found in output"

    def test_sum_of_five_function_types(self, solver, env):
        """Test sum of 5 different function types."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        v1 = SHOTpy.Variable("v1", 0, SHOTpy.VariableType.Real, 1.0, 10.0)
        v2 = SHOTpy.Variable("v2", 1, SHOTpy.VariableType.Real, 0.1, 3.14)
        v3 = SHOTpy.Variable("v3", 2, SHOTpy.VariableType.Real, 0.1, 3.14)
        v4 = SHOTpy.Variable("v4", 3, SHOTpy.VariableType.Real, 0.1, 10.0)
        v5 = SHOTpy.Variable("v5", 4, SHOTpy.VariableType.Real, 0.1, 5.0)
        problem.addVariable(v1)
        problem.addVariable(v2)
        problem.addVariable(v3)
        problem.addVariable(v4)
        problem.addVariable(v5)
        
        # Sum of 5 different functions
        expr = SHOTpy.log(v1) + SHOTpy.sin(v2) + SHOTpy.cos(v3) + SHOTpy.sqrt(v4) + SHOTpy.exp(v5)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(expr)
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 1.0, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, v1))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        for var_name in ['v1', 'v2', 'v3', 'v4', 'v5']:
            assert var_name in output, f"Variable {var_name} should appear in problem"

    def test_mixed_linear_quadratic_nonlinear(self, solver, env):
        """Test mixed sum: linear + quadratic + nonlinear expressions."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 1.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        z = SHOTpy.Variable("z", 2, SHOTpy.VariableType.Real, 0.1, 3.14)
        problem.addVariable(x)
        problem.addVariable(y)
        problem.addVariable(z)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(2.0, x))         # Linear term
        obj.add(SHOTpy.QuadraticTerm(1.0, y, y))   # Quadratic term
        obj.add(SHOTpy.log(x))                      # Nonlinear expression
        obj.add(SHOTpy.sin(z))                      # Another nonlinear expression
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 1.0, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        # Should have L, Q, and E indicators
        lines = output.split('\n')
        for i, line in enumerate(lines):
            if 'minimize:' in line.lower() or 'maximize:' in line.lower():
                obj_line = lines[i + 1] if i + 1 < len(lines) else ""
                assert 'L' in obj_line, f"Should have linear indicator 'L': {obj_line}"
                assert 'Q' in obj_line, f"Should have quadratic indicator 'Q': {obj_line}"
                assert 'E' in obj_line, f"Should have expression indicator 'E': {obj_line}"
                break


class TestExpressionVsExplicitTerms:
    """Tests verifying behavior of expressions vs explicit term types.
    
    When terms are added as expressions (via operators like 2*x or x**2),
    they remain as nonlinear expressions. Only explicitly constructed terms
    (LinearTerm, QuadraticTerm, etc.) get their specific type indicators.
    """

    def test_explicit_linear_term_shows_L(self, solver, env):
        """Explicit LinearTerm shows L indicator in constraint."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        # Explicit LinearTerm
        c = SHOTpy.LinearConstraint(0, "explicit_linear", -SHOTpy.SHOT_DBL_MAX, 10.0)
        c.add(SHOTpy.LinearTerm(2.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        for line in output.split('\n'):
            if 'explicit_linear' in line:
                assert '[L    ]' in line, f"Explicit LinearTerm should show [L    ]: {line}"
                break
        else:
            assert False, "Constraint 'explicit_linear' not found"

    def test_linear_expression_extracted_to_L(self, solver, env):
        """Linear expression via operators is extracted to L indicator (linear term)."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        # Linear expression via operators (not explicit term)
        c = SHOTpy.NonlinearConstraint(0, "expr_linear", -SHOTpy.SHOT_DBL_MAX, 10.0)
        c.add(2.0 * x)  # Expression, gets extracted to LinearTerm in finalize()
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        for line in output.split('\n'):
            if 'expr_linear' in line:
                # Expression is extracted to linear term, showing L indicator
                assert '[L    ]' in line, f"Linear expression should be extracted to L: {line}"
                break
        else:
            assert False, "Constraint 'expr_linear' not found"

    def test_explicit_quadratic_term_shows_Q(self, solver, env):
        """Explicit QuadraticTerm shows Q indicator in constraint."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        # Explicit QuadraticTerm
        c = SHOTpy.QuadraticConstraint(0, "explicit_quadratic", -SHOTpy.SHOT_DBL_MAX, 10.0)
        c.add(SHOTpy.QuadraticTerm(1.0, x, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        for line in output.split('\n'):
            if 'explicit_quadratic' in line:
                assert 'Q' in line, f"Explicit QuadraticTerm should show Q: {line}"
                break
        else:
            assert False, "Constraint 'explicit_quadratic' not found"

    def test_quadratic_expression_extracted_to_Q(self, solver, env):
        """Quadratic expression via operators is extracted to Q indicator (quadratic term)."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        # Quadratic expression via operators (not explicit term)
        c = SHOTpy.NonlinearConstraint(0, "expr_quadratic", -SHOTpy.SHOT_DBL_MAX, 10.0)
        c.add(x ** 2)  # Expression, gets extracted to QuadraticTerm in finalize()
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        for line in output.split('\n'):
            if 'expr_quadratic' in line:
                # Expression is extracted to quadratic term, showing Q indicator
                assert '[ Q' in line, f"Quadratic expression should be extracted to Q: {line}"
                break
        else:
            assert False, "Constraint 'expr_quadratic' not found"

    def test_explicit_signomial_term_shows_S(self, solver, env):
        """Explicit SignomialTerm shows S indicator in constraint."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        # Explicit SignomialTerm
        c = SHOTpy.NonlinearConstraint(0, "explicit_signomial", 0.1, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.SignomialTerm(1.0, [(x, 0.5)]))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        for line in output.split('\n'):
            if 'explicit_signomial' in line:
                assert 'S' in line, f"Explicit SignomialTerm should show S: {line}"
                break
        else:
            assert False, "Constraint 'explicit_signomial' not found"

    def test_explicit_monomial_term_shows_M(self, solver, env):
        """Explicit MonomialTerm shows M indicator in constraint."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        # Explicit MonomialTerm
        c = SHOTpy.NonlinearConstraint(0, "explicit_monomial", -SHOTpy.SHOT_DBL_MAX, 10.0)
        c.add(SHOTpy.MonomialTerm(1.0, [x, y, y]))  # x * y^2
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        for line in output.split('\n'):
            if 'explicit_monomial' in line:
                assert 'M' in line, f"Explicit MonomialTerm should show M: {line}"
                break
        else:
            assert False, "Constraint 'explicit_monomial' not found"

    def test_bilinear_expression_extracted_to_Q(self, solver, env):
        """Bilinear expression x*y via operators is extracted to Q indicator (quadratic term)."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.0, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.0, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        # Bilinear expression via operators
        c = SHOTpy.NonlinearConstraint(0, "expr_bilinear", -SHOTpy.SHOT_DBL_MAX, 10.0)
        c.add(x * y)  # Expression, gets extracted to QuadraticTerm in finalize()
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        for line in output.split('\n'):
            if 'expr_bilinear' in line:
                # Bilinear expression is extracted to quadratic term, showing Q indicator
                assert '[ Q' in line, f"Bilinear expression should be extracted to Q: {line}"
                break
        else:
            assert False, "Constraint 'expr_bilinear' not found"

    def test_objective_explicit_vs_expression(self, solver, env):
        """Test objective with both explicit terms and expressions."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))         # Explicit linear -> L
        obj.add(SHOTpy.QuadraticTerm(1.0, y, y))   # Explicit quadratic -> Q
        obj.add(SHOTpy.log(x))                     # Expression -> E
        problem.setObjective(obj)
        
        c = SHOTpy.LinearConstraint(0, "bound", 0.1, SHOTpy.SHOT_DBL_MAX)
        c.add(SHOTpy.LinearTerm(1.0, x))
        problem.addConstraint(c)
        
        problem.finalize()
        output = problem.toString()
        
        lines = output.split('\n')
        for i, line in enumerate(lines):
            if 'minimize:' in line.lower() or 'maximize:' in line.lower():
                obj_line = lines[i + 1] if i + 1 < len(lines) else ""
                assert 'L' in obj_line, f"Should have L for explicit LinearTerm: {obj_line}"
                assert 'Q' in obj_line, f"Should have Q for explicit QuadraticTerm: {obj_line}"
                assert 'E' in obj_line, f"Should have E for expression: {obj_line}"
                break

    def test_constraint_properties_explicit_terms(self, solver, env):
        """Verify constraint properties are correct for explicit terms."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        y = SHOTpy.Variable("y", 1, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        problem.addVariable(y)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        # Constraint with explicit terms of each type
        c = SHOTpy.NonlinearConstraint(0, "multi_term", -SHOTpy.SHOT_DBL_MAX, 100.0)
        c.add(SHOTpy.LinearTerm(1.0, x))
        c.add(SHOTpy.QuadraticTerm(1.0, y, y))
        c.add(SHOTpy.SignomialTerm(1.0, [(x, 0.5)]))
        c.add(SHOTpy.MonomialTerm(1.0, [x, y]))
        problem.addConstraint(c)
        
        problem.finalize()
        
        # Check properties
        assert c.properties.hasLinearTerms, "Should have linear terms"
        assert c.properties.hasQuadraticTerms, "Should have quadratic terms"
        assert c.properties.hasSignomialTerms, "Should have signomial terms"
        assert c.properties.hasMonomialTerms, "Should have monomial terms"

    def test_constraint_properties_expression_only(self, solver, env):
        """Verify constraint properties when using only expressions."""
        import SHOTpy
        
        problem = SHOTpy.Problem(env)
        x = SHOTpy.Variable("x", 0, SHOTpy.VariableType.Real, 0.1, 10.0)
        problem.addVariable(x)
        
        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(1.0, x))
        problem.setObjective(obj)
        
        # Constraint with expression only
        c = SHOTpy.NonlinearConstraint(0, "expr_only", -SHOTpy.SHOT_DBL_MAX, 10.0)
        c.add(2.0 * x + x**2)  # Expressions only
        problem.addConstraint(c)
        
        problem.finalize()
        
        # Expressions don't set the specific term properties
        assert not c.properties.hasLinearTerms, "Expression doesn't set hasLinearTerms"
        assert not c.properties.hasQuadraticTerms, "Expression doesn't set hasQuadraticTerms"
        assert c.properties.hasNonlinearExpression, "Should have nonlinear expression"
