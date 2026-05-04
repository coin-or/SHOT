"""
Tests for the SHOT callback system exposed via the Python API.

Covers all six E_EventType values:
  - EventType.NewPrimalSolution
  - EventType.PrimalSolutionCandidateSelection
  - EventType.UserTerminationCheck
  - EventType.ExternalDualBound
  - EventType.ExternalHyperplaneSelection
  - EventType.ExternalPrimalSolution
"""

import pytest
import math


# ---------------------------------------------------------------------------
# Helper: build the ex1223b MINLP problem (same as C++ SolverTest case 5)
# ---------------------------------------------------------------------------

def build_ex1223b(env):
    """Return a finalised ex1223b Problem object (uses Python operator API)."""
    import SHOTpy

    problem = SHOTpy.Problem(env)
    problem.name = "ex1223b"

    x1 = SHOTpy.Variable("x1", 0, SHOTpy.VariableType.Real,   0.0, 10.0)
    x2 = SHOTpy.Variable("x2", 1, SHOTpy.VariableType.Real,   0.0, 10.0)
    x3 = SHOTpy.Variable("x3", 2, SHOTpy.VariableType.Real,   0.0, 10.0)
    b4 = SHOTpy.Variable("b4", 3, SHOTpy.VariableType.Binary, 0.0, 1.0)
    b5 = SHOTpy.Variable("b5", 4, SHOTpy.VariableType.Binary, 0.0, 1.0)
    b6 = SHOTpy.Variable("b6", 5, SHOTpy.VariableType.Binary, 0.0, 1.0)
    b7 = SHOTpy.Variable("b7", 6, SHOTpy.VariableType.Binary, 0.0, 1.0)
    for v in [x1, x2, x3, b4, b5, b6, b7]:
        problem.addVariable(v)

    # minimize (-1+b4)^2 + (-2+b5)^2 + (-1+b6)^2 - log(1+b7)
    #        + (-1+x1)^2 + (-2+x2)^2 + (-3+x3)^2
    obj_expr = ((b4 - 1.0)**2 + (b5 - 2.0)**2 + (b6 - 1.0)**2
                - SHOTpy.log(1.0 + b7)
                + (x1 - 1.0)**2 + (x2 - 2.0)**2 + (x3 - 3.0)**2)
    obj = SHOTpy.NonlinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize, obj_expr, 0.0)
    problem.setObjective(obj)

    # e1: x1+x2+x3+b4+b5+b6 <= 5  (linear)
    lt1 = SHOTpy.LinearTerms()
    for v in [x1, x2, x3, b4, b5, b6]:
        lt1.add(SHOTpy.LinearTerm(1.0, v))
    e1 = SHOTpy.LinearConstraint(0, "e1", lt1, SHOTpy.SHOT_DBL_MIN, 5.0)
    problem.addConstraint(e1)

    # e2: b6^2+x1^2+x2^2+x3^2 <= 5.5
    e2 = SHOTpy.NonlinearConstraint(1, "e2", b6**2 + x1**2 + x2**2 + x3**2,
                                     SHOTpy.SHOT_DBL_MIN, 5.5)
    problem.addConstraint(e2)

    # e3: x1+b4 <= 1.2
    lt3 = SHOTpy.LinearTerms()
    lt3.add(SHOTpy.LinearTerm(1.0, x1)); lt3.add(SHOTpy.LinearTerm(1.0, b4))
    e3 = SHOTpy.LinearConstraint(2, "e3", lt3, SHOTpy.SHOT_DBL_MIN, 1.2)
    problem.addConstraint(e3)

    # e4: x2+b5 <= 1.8
    lt4 = SHOTpy.LinearTerms()
    lt4.add(SHOTpy.LinearTerm(1.0, x2)); lt4.add(SHOTpy.LinearTerm(1.0, b5))
    e4 = SHOTpy.LinearConstraint(3, "e4", lt4, SHOTpy.SHOT_DBL_MIN, 1.8)
    problem.addConstraint(e4)

    # e5: x3+b6 <= 2.5
    lt5 = SHOTpy.LinearTerms()
    lt5.add(SHOTpy.LinearTerm(1.0, x3)); lt5.add(SHOTpy.LinearTerm(1.0, b6))
    e5 = SHOTpy.LinearConstraint(4, "e5", lt5, SHOTpy.SHOT_DBL_MIN, 2.5)
    problem.addConstraint(e5)

    # e6: x1+b7 <= 1.2
    lt6 = SHOTpy.LinearTerms()
    lt6.add(SHOTpy.LinearTerm(1.0, x1)); lt6.add(SHOTpy.LinearTerm(1.0, b7))
    e6 = SHOTpy.LinearConstraint(5, "e6", lt6, SHOTpy.SHOT_DBL_MIN, 1.2)
    problem.addConstraint(e6)

    # e7: b5^2+x2^2 <= 1.64
    e7 = SHOTpy.NonlinearConstraint(6, "e7", b5**2 + x2**2, SHOTpy.SHOT_DBL_MIN, 1.64)
    problem.addConstraint(e7)

    # e8: b6^2+x3^2 <= 4.25
    e8 = SHOTpy.NonlinearConstraint(7, "e8", b6**2 + x3**2, SHOTpy.SHOT_DBL_MIN, 4.25)
    problem.addConstraint(e8)

    # e9: b5^2+x3^2 <= 4.64
    e9 = SHOTpy.NonlinearConstraint(8, "e9", b5**2 + x3**2, SHOTpy.SHOT_DBL_MIN, 4.64)
    problem.addConstraint(e9)

    problem.finalize()
    return problem


# ---------------------------------------------------------------------------
# Helper: build a small convex MINLP (SolverTest case 8 equivalent)
#   minimize -x1 - 2*x2
#   s.t.  x1^2 + x2 + 0.1*e^x2  <= 10    (nonlinear)
#         e^x1 / x2               <= 3    (nonlinear)
#   x1 in {0,1,2,3} (integer), x2 in {1,2,3} (integer)
#   Optimal: x1=2, x2=3, obj=-8
# ---------------------------------------------------------------------------

def build_small_convex(env):
    """Return a finalised small convex MINLP for external-hyperplane tests."""
    import SHOTpy

    problem = SHOTpy.Problem(env)
    problem.name = "small_convex"

    x1 = SHOTpy.Variable("x1", 0, SHOTpy.VariableType.Integer, 0.0, 3.0)
    x2 = SHOTpy.Variable("x2", 1, SHOTpy.VariableType.Integer, 1.0, 3.0)
    for v in [x1, x2]:
        problem.addVariable(v)

    # minimize -x1 - 2*x2  (linear objective)
    lt = SHOTpy.LinearTerms()
    lt.add(SHOTpy.LinearTerm(-1.0, x1))
    lt.add(SHOTpy.LinearTerm(-2.0, x2))
    obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize, lt, 0.0)
    problem.setObjective(obj)

    # e1: x1^2 + x2 + 0.1*exp(x2) <= 10
    e1_expr = x1**2 + x2 + 0.1 * SHOTpy.exp(x2)
    e1 = SHOTpy.NonlinearConstraint(0, "e1", e1_expr, SHOTpy.SHOT_DBL_MIN, 10.0)
    problem.addConstraint(e1)

    # e2: exp(x1) / x2 <= 3
    e2_expr = SHOTpy.exp(x1) / x2
    e2 = SHOTpy.NonlinearConstraint(1, "e2", e2_expr, SHOTpy.SHOT_DBL_MIN, 3.0)
    problem.addConstraint(e2)

    problem.finalize()
    return problem, [e1, e2]


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestEventTypeEnum:
    """Verify E_EventType is exposed correctly."""

    def test_event_type_values(self):
        import SHOTpy
        assert hasattr(SHOTpy, "EventType")
        for name in [
            "ExternalDualBound",
            "ExternalHyperplaneSelection",
            "ExternalPrimalSolution",
            "NewPrimalSolution",
            "PrimalSolutionCandidateSelection",
            "UserTerminationCheck",
        ]:
            assert hasattr(SHOTpy.EventType, name), f"Missing EventType.{name}"


class TestCallbackDataStructures:
    """Verify callback data structures are exposed correctly."""

    def test_dual_bound_callback_data_attrs(self):
        import SHOTpy
        for attr in ["isMinimization", "currentDualBound", "currentPrimalBound",
                     "relativeGap", "absoluteGap", "iterationNumber", "solutionStatistics"]:
            assert hasattr(SHOTpy.DualBoundCallbackData, attr)

    def test_termination_callback_data_attrs(self):
        import SHOTpy
        for attr in ["iterationNumber", "currentDualBound", "currentPrimalBound",
                     "relativeGap", "absoluteGap", "timeElapsed", "solutionStatistics"]:
            assert hasattr(SHOTpy.TerminationCallbackData, attr)

    def test_primal_solution_callback_data_attrs(self):
        import SHOTpy
        for attr in ["isMinimization", "solution", "objectiveValue", "currentDualBound",
                     "relativeGap", "absoluteGap", "iterationNumber", "sourceType",
                     "solutionStatistics"]:
            assert hasattr(SHOTpy.PrimalSolutionCallbackData, attr)

    def test_external_hyperplane_selection_callback_data_attrs(self):
        import SHOTpy
        for attr in ["iterationNumber", "currentDualBound", "currentPrimalBound",
                     "solutionPoints", "originalProblem", "reformulatedProblem",
                     "isObjectiveNonlinear", "solutionStatistics"]:
            assert hasattr(SHOTpy.ExternalHyperplaneSelectionCallbackData, attr)

    def test_solution_point_attrs(self):
        import SHOTpy
        sp = SHOTpy.SolutionPoint()
        assert hasattr(sp, "point")
        assert hasattr(sp, "objectiveValue")
        assert hasattr(sp, "iterFound")
        assert hasattr(sp, "maxDeviation")
        assert hasattr(sp, "isRelaxedPoint")
        assert hasattr(sp, "hashValue")

    def test_external_hyperplane_attrs(self):
        import SHOTpy
        hp = SHOTpy.ExternalHyperplane()
        assert hasattr(hp, "variableIndexes")
        assert hasattr(hp, "variableCoefficients")
        assert hasattr(hp, "description")
        assert hasattr(hp, "rhsValue")
        assert hasattr(hp, "isGlobal")
        assert hasattr(hp, "source")

    def test_external_hyperplane_create_and_set(self):
        import SHOTpy
        hp = SHOTpy.ExternalHyperplane()
        hp.variableIndexes = [0, 1]
        hp.variableCoefficients = [2.0, -1.0]
        hp.rhsValue = 5.0
        hp.isGlobal = True
        hp.description = "my_cut"
        hp.source = SHOTpy.HyperplaneSource.External
        assert hp.variableIndexes == [0, 1]
        assert hp.variableCoefficients == [2.0, -1.0]
        assert hp.rhsValue == 5.0
        assert hp.isGlobal is True
        assert hp.description == "my_cut"
        assert hp.source == SHOTpy.HyperplaneSource.External


class TestNewPrimalSolutionCallback:
    """EventType.NewPrimalSolution – notification, no return value."""

    def test_callback_is_called(self, solver, env):
        """Callback is invoked at least once when a primal solution is found."""
        import SHOTpy

        call_log = []

        def on_new_primal(data):
            call_log.append({
                "objValue": data.objectiveValue,
                "iteration": data.iterationNumber,
                "gap": data.relativeGap,
            })

        solver.updateSetting("Console.LogLevel", "Output", 6)  # Off
        problem = build_ex1223b(env)
        solver.setProblem(problem)
        solver.registerCallback(SHOTpy.EventType.NewPrimalSolution, on_new_primal)
        solver.solveProblem()

        assert len(call_log) >= 1, "Callback was never called"
        # Objective values in the callback should be non-trivially large (problem has known opt ~4.58)
        best = min(e["objValue"] for e in call_log)
        assert best < 10.0

    def test_callback_receives_correct_types(self, solver, env):
        """Callback data fields have the right Python types."""
        import SHOTpy

        received = []

        def on_new_primal(data):
            received.append(data)

        solver.updateSetting("Console.LogLevel", "Output", 6)
        problem = build_ex1223b(env)
        solver.setProblem(problem)
        solver.registerCallback(SHOTpy.EventType.NewPrimalSolution, on_new_primal)
        solver.solveProblem()

        assert received, "No callback data received"
        d = received[0]
        assert isinstance(d.objectiveValue, float)
        assert isinstance(d.iterationNumber, int)
        assert isinstance(d.isMinimization, bool)
        assert isinstance(d.solution, list)
        assert all(isinstance(v, float) for v in d.solution)


class TestUserTerminationCheckCallback:
    """EventType.UserTerminationCheck – data provider returning bool."""

    def test_callback_stops_solver(self, solver, env):
        """Returning True from the callback terminates the solver early."""
        import SHOTpy

        termination_calls = []

        def should_terminate(data):
            termination_calls.append(data.iterationNumber)
            # Ask to stop after the 3rd check
            return len(termination_calls) >= 3

        solver.updateSetting("Console.LogLevel", "Output", 6)
        solver.updateSetting("IterationLimit", "Termination", 1000)
        problem = build_ex1223b(env)
        solver.setProblem(problem)
        solver.registerCallback(SHOTpy.EventType.UserTerminationCheck, should_terminate)
        solver.solveProblem()

        assert len(termination_calls) >= 3, "Termination callback was not called enough times"
        # If termination worked, total iterations should be small
        stats = solver.getSolutionStatistics()
        # The solver must have been told to terminate – iterations should be low
        assert stats.numberOfIterations < 100, (
            f"Expected solver to stop early, but ran {stats.numberOfIterations} iterations"
        )

    def test_returning_none_continues(self, solver, env):
        """Returning None (instead of bool) from the callback should not crash."""
        import SHOTpy

        call_count = [0]

        def never_terminate(data):
            call_count[0] += 1
            return None  # Python None → C++ treats as False → solver continues

        solver.updateSetting("Console.LogLevel", "Output", 6)
        solver.updateSetting("IterationLimit", "Termination", 5)
        problem = build_ex1223b(env)
        solver.setProblem(problem)
        solver.registerCallback(SHOTpy.EventType.UserTerminationCheck, never_terminate)
        solver.solveProblem()  # Must not crash

    def test_callback_receives_structured_data(self, solver, env):
        """TerminationCallbackData fields are populated correctly."""
        import SHOTpy

        received = []

        def collect(data):
            received.append(data)
            return len(received) >= 2

        solver.updateSetting("Console.LogLevel", "Output", 6)
        problem = build_ex1223b(env)
        solver.setProblem(problem)
        solver.registerCallback(SHOTpy.EventType.UserTerminationCheck, collect)
        solver.solveProblem()

        assert received
        d = received[0]
        assert isinstance(d.iterationNumber, int)
        assert isinstance(d.currentDualBound, float)
        assert isinstance(d.currentPrimalBound, float)
        assert isinstance(d.timeElapsed, float)
        assert d.timeElapsed >= 0.0


class TestExternalHyperplaneSelectionCallback:
    """EventType.ExternalHyperplaneSelection – data provider returning list[ExternalHyperplane]."""

    def test_callback_is_called_and_hyperplanes_accepted(self, solver, env):
        """External hyperplane callback is invoked with populated data."""
        import SHOTpy

        hyperplane_calls = []

        def provide_hyperplanes(data):
            hyperplane_calls.append({
                "iteration": data.iterationNumber,
                "n_points": len(data.solutionPoints),
            })
            # Returning an empty list is valid; SHOT continues with its own cuts
            return []

        solver.updateSetting("Console.LogLevel", "Output", 6)
        solver.updateSetting("CutStrategy", "Dual", 1)   # OnlyExternal
        solver.updateSetting("TreeStrategy", "Dual", 1)  # MultiTree

        problem, _ = build_small_convex(env)
        solver.setProblem(problem, problem)
        solver.registerCallback(SHOTpy.EventType.ExternalHyperplaneSelection, provide_hyperplanes)
        solver.solveProblem()

        assert len(hyperplane_calls) >= 1, "External hyperplane callback was never called"

    def test_returning_none_is_safe(self, solver, env):
        """Returning None from the hyperplane callback must not crash."""
        import SHOTpy

        solver.updateSetting("Console.LogLevel", "Output", 6)
        solver.updateSetting("CutStrategy", "Dual", 1)
        solver.updateSetting("TreeStrategy", "Dual", 1)

        problem, _ = build_small_convex(env)
        solver.setProblem(problem, problem)
        solver.registerCallback(
            SHOTpy.EventType.ExternalHyperplaneSelection, lambda data: None
        )
        solver.solveProblem()  # Must not crash

    def test_returning_empty_list_is_safe(self, solver, env):
        """Returning an empty list from the hyperplane callback must not crash."""
        import SHOTpy

        solver.updateSetting("Console.LogLevel", "Output", 6)
        solver.updateSetting("CutStrategy", "Dual", 1)
        solver.updateSetting("TreeStrategy", "Dual", 1)

        problem, _ = build_small_convex(env)
        solver.setProblem(problem, problem)
        solver.registerCallback(
            SHOTpy.EventType.ExternalHyperplaneSelection, lambda data: []
        )
        solver.solveProblem()  # Must not crash


class TestExternalHyperplaneGradientCuts:
    """End-to-end test: solve shot_ex_jogo using *only* external gradient cuts.

    This exercises the full loop:
      - CutStrategy = OnlyExternal (2) — no built-in ESH/ECP cuts
      - Relaxation.Use = False — skip LP relaxation phase
      - The callback looks up the violated constraint by .index (global constraint
        index) rather than positional index, which is the correct approach when
        the problem has a mix of linear and nonlinear constraints.
    """

    def _build_shot_ex_jogo(self, env):
        """Return (solver, problem) for the shot_ex_jogo MINLP.

        min  -x1 - x2
        s.t. 2*x1 - 3*x2          <= 2   (linear,    index 0)
             0.15*(x1-8)^2 + ...  <= 5   (nonlinear, index 1)
             1/x1 + 1/x2 - ...    <= -4  (nonlinear, index 2)
        x1 in [1,20] (real), x2 in [1,20] (integer)
        Known optimum: obj ≈ -20.9036 (primal -20.903615, dual -20.903647)
        """
        import SHOTpy

        problem = SHOTpy.Problem(env)
        problem.name = "shot_ex_jogo"

        x1 = SHOTpy.Variable("x1", 0, SHOTpy.VariableType.Real,    1.0, 20.0)
        x2 = SHOTpy.Variable("x2", 1, SHOTpy.VariableType.Integer, 1.0, 20.0)
        problem.addVariable(x1)
        problem.addVariable(x2)

        obj = SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize)
        obj.add(SHOTpy.LinearTerm(-1.0, x1))
        obj.add(SHOTpy.LinearTerm(-1.0, x2))
        problem.setObjective(obj)

        # Constraint l (index 0): 2*x1 - 3*x2 <= 2
        lt_l = SHOTpy.LinearTerms()
        lt_l.add(SHOTpy.LinearTerm( 2.0, x1))
        lt_l.add(SHOTpy.LinearTerm(-3.0, x2))
        problem.addConstraint(SHOTpy.LinearConstraint(0, "l", lt_l, SHOTpy.SHOT_DBL_MIN, 2.0))

        # Constraint c1 (index 1): 0.15*(x1-8)^2 + 0.1*(x2-6)^2 + 0.025*exp(x1)/x2^2 <= 5
        c1 = SHOTpy.NonlinearConstraint(1, "c1", SHOTpy.SHOT_DBL_MIN, 5.0)
        c1.add(0.15 * (x1 - 8.0)**2 + 0.1 * (x2 - 6.0)**2 + 0.025 * SHOTpy.exp(x1) / x2**2)
        problem.addConstraint(c1)

        # Constraint c2 (index 2): 1/x1 + 1/x2 - sqrt(x1)*sqrt(x2) <= -4
        c2 = SHOTpy.NonlinearConstraint(2, "c2", SHOTpy.SHOT_DBL_MIN, -4.0)
        c2.add(SHOTpy.SignomialTerm( 1.0, [(x1, -1.0)]))
        c2.add(SHOTpy.SignomialTerm( 1.0, [(x2, -1.0)]))
        c2.add(SHOTpy.SignomialTerm(-1.0, [(x1, 0.5), (x2, 0.5)]))
        problem.addConstraint(c2)

        problem.finalize()
        return problem

    def _make_solver_with_only_external_cuts(self):
        """Return a Solver configured to accept only external hyperplane cuts."""
        import SHOTpy
        solver = SHOTpy.Solver()
        solver.updateSetting("Console.LogLevel",                             "Output", 2)
        solver.updateSetting("Convexity.AssumeConvex",                       "Model",  True)
        solver.updateSetting("Reformulation.Constraint.PartitionQuadraticTerms", "Model", 2) # Never
        solver.updateSetting("Relaxation.Use",                               "Dual",   True)
        solver.updateSetting("CutStrategy",                                  "Dual",   2)    # OnlyExternal
        solver.updateSetting("TreeStrategy",                                 "Dual",   0)    # MultiTree
        return solver

    def test_solver_finds_solution_with_gradient_cuts(self):
        """Solver reaches a feasible solution when all cuts come from the callback."""
        import SHOTpy

        solver = self._make_solver_with_only_external_cuts()
        env = solver.getEnvironment()
        problem = self._build_shot_ex_jogo(env)
        solver.setProblem(problem, problem)

        cut_count = [0]

        def generate_hyperplanes(data):
            hyperplanes = []
            if not data.solutionPoints or data.iterationNumber == 0:
                return hyperplanes

            reform_problem = data.reformulatedProblem
            for sol_point in data.solutionPoints:
                dev_idx   = sol_point.maxDeviation.index
                violation = sol_point.maxDeviation.value
                if violation <= 0.0:
                    continue

                # Look up by global .index, not positional index
                constraint = next(
                    (nlc for nlc in reform_problem.nonlinearConstraints if nlc.index == dev_idx),
                    None
                )
                if constraint is None:
                    continue

                gradient = constraint.calculateGradient(sol_point.point)
                if not gradient:
                    continue

                var_indices = list(gradient.keys())
                rhs = sum(gradient[i] * sol_point.point[i] for i in var_indices) - violation

                hp = SHOTpy.ExternalHyperplane()
                hp.variableIndexes      = var_indices
                hp.variableCoefficients = list(gradient.values())
                hp.rhsValue             = rhs
                hp.isGlobal             = True
                hp.source               = SHOTpy.HyperplaneSource.External
                hyperplanes.append(hp)
                cut_count[0] += 1
                break

            return hyperplanes

        solver.registerCallback(SHOTpy.EventType.ExternalHyperplaneSelection, generate_hyperplanes)
        solver.solveProblem()

        assert cut_count[0] > 0, "Callback never generated any hyperplane cuts"
        assert solver.getPrimalSolutions(), "No primal solution found"
        obj = solver.getPrimalSolution().objValue
        # Known optimum: primal ≈ -20.9036, dual ≈ -20.9036
        # Accept any solution within 1% relative of the known primal bound.
        assert obj <= -20.903615014500517 * 0.99, (
            f"Objective {obj:.6f} is far from the known optimum ~-20.9036"
        )

    def test_constraint_lookup_by_index_not_position(self):
        """maxDeviation.index is a global constraint index, not a position in
        nonlinearConstraints.  This test verifies the lookup is correct by
        checking that the constraint name returned matches the expected constraint."""
        import SHOTpy

        solver = self._make_solver_with_only_external_cuts()
        # This test never returns cuts, so cap iterations to avoid running forever
        solver.updateSetting("IterationLimit", "Termination", 20)
        env = solver.getEnvironment()
        problem = self._build_shot_ex_jogo(env)
        solver.setProblem(problem, problem)

        name_log = []

        def generate_hyperplanes(data):
            if not data.solutionPoints or data.iterationNumber == 0:
                return []

            reform_problem = data.reformulatedProblem
            for sol_point in data.solutionPoints:
                dev_idx   = sol_point.maxDeviation.index
                violation = sol_point.maxDeviation.value
                if violation <= 0.0:
                    continue

                # Correct lookup: by .index attribute
                constraint = next(
                    (nlc for nlc in reform_problem.nonlinearConstraints if nlc.index == dev_idx),
                    None
                )
                if constraint is not None:
                    name_log.append(constraint.name)
                    # The name must be one of the nonlinear constraints, not the linear one
                    assert constraint.name in ("c1", "c2"), (
                        f"Lookup returned wrong constraint '{constraint.name}' "
                        f"for global index {dev_idx}"
                    )
                break
            return []

        solver.registerCallback(SHOTpy.EventType.ExternalHyperplaneSelection, generate_hyperplanes)
        solver.solveProblem()

        assert name_log, "Callback never found a violated nonlinear constraint"


class TestRegisterCallbackAPI:
    """API-level checks for Solver.registerCallback."""

    def test_register_callback_method_exists(self, solver):
        import SHOTpy
        assert hasattr(solver, "registerCallback")

    def test_register_with_lambda(self, solver, env):
        """registerCallback accepts a lambda."""
        import SHOTpy
        solver.updateSetting("Console.LogLevel", "Output", 6)
        solver.updateSetting("IterationLimit", "Termination", 2)
        problem = build_ex1223b(env)
        solver.setProblem(problem)
        solver.registerCallback(
            SHOTpy.EventType.NewPrimalSolution, lambda d: None
        )
        solver.solveProblem()  # Must not raise

    def test_register_with_callable_object(self, solver, env):
        """registerCallback accepts any callable (not just lambdas)."""
        import SHOTpy

        class Counter:
            def __init__(self):
                self.count = 0
            def __call__(self, data):
                self.count += 1

        counter = Counter()
        solver.updateSetting("Console.LogLevel", "Output", 6)
        problem = build_ex1223b(env)
        solver.setProblem(problem)
        solver.registerCallback(SHOTpy.EventType.NewPrimalSolution, counter)
        solver.solveProblem()
        assert counter.count >= 1
