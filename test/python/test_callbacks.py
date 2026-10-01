"""
Tests for the SHOT callback system exposed via the Python API.

A callback is registered for one or several locations (SHOTpy.CallbackLocation) and is called with a context of the
class for the location, which has the state of the solver and the actions available there:
  - CallbackLocation.InteriorPointSearch    InteriorPointSearchContext
  - CallbackLocation.DualBoundUpdate        DualBoundUpdateContext
  - CallbackLocation.PrimalCandidateSearch  PrimalCandidateSearchContext
  - CallbackLocation.PrimalCandidateCheck   PrimalCandidateCheckContext
  - CallbackLocation.NewPrimalSolution      NewPrimalSolutionContext
  - CallbackLocation.TerminationCheck       TerminationCheckContext
  - CallbackLocation.HyperplaneSelection    HyperplaneSelectionContext
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

    x1 = SHOTpy.Variable("x1", SHOTpy.VariableType.Real,   0.0, 10.0)
    x2 = SHOTpy.Variable("x2", SHOTpy.VariableType.Real,   0.0, 10.0)
    x3 = SHOTpy.Variable("x3", SHOTpy.VariableType.Real,   0.0, 10.0)
    b4 = SHOTpy.Variable("b4", SHOTpy.VariableType.Binary, 0.0, 1.0)
    b5 = SHOTpy.Variable("b5", SHOTpy.VariableType.Binary, 0.0, 1.0)
    b6 = SHOTpy.Variable("b6", SHOTpy.VariableType.Binary, 0.0, 1.0)
    b7 = SHOTpy.Variable("b7", SHOTpy.VariableType.Binary, 0.0, 1.0)
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
    e1 = SHOTpy.LinearConstraint("e1", lt1, SHOTpy.SHOT_DBL_MIN, 5.0)
    problem.addConstraint(e1)

    # e2: b6^2+x1^2+x2^2+x3^2 <= 5.5
    e2 = SHOTpy.NonlinearConstraint("e2", b6**2 + x1**2 + x2**2 + x3**2,
                                     SHOTpy.SHOT_DBL_MIN, 5.5)
    problem.addConstraint(e2)

    # e3: x1+b4 <= 1.2
    lt3 = SHOTpy.LinearTerms()
    lt3.add(SHOTpy.LinearTerm(1.0, x1)); lt3.add(SHOTpy.LinearTerm(1.0, b4))
    e3 = SHOTpy.LinearConstraint("e3", lt3, SHOTpy.SHOT_DBL_MIN, 1.2)
    problem.addConstraint(e3)

    # e4: x2+b5 <= 1.8
    lt4 = SHOTpy.LinearTerms()
    lt4.add(SHOTpy.LinearTerm(1.0, x2)); lt4.add(SHOTpy.LinearTerm(1.0, b5))
    e4 = SHOTpy.LinearConstraint("e4", lt4, SHOTpy.SHOT_DBL_MIN, 1.8)
    problem.addConstraint(e4)

    # e5: x3+b6 <= 2.5
    lt5 = SHOTpy.LinearTerms()
    lt5.add(SHOTpy.LinearTerm(1.0, x3)); lt5.add(SHOTpy.LinearTerm(1.0, b6))
    e5 = SHOTpy.LinearConstraint("e5", lt5, SHOTpy.SHOT_DBL_MIN, 2.5)
    problem.addConstraint(e5)

    # e6: x1+b7 <= 1.2
    lt6 = SHOTpy.LinearTerms()
    lt6.add(SHOTpy.LinearTerm(1.0, x1)); lt6.add(SHOTpy.LinearTerm(1.0, b7))
    e6 = SHOTpy.LinearConstraint("e6", lt6, SHOTpy.SHOT_DBL_MIN, 1.2)
    problem.addConstraint(e6)

    # e7: b5^2+x2^2 <= 1.64
    e7 = SHOTpy.NonlinearConstraint("e7", b5**2 + x2**2, SHOTpy.SHOT_DBL_MIN, 1.64)
    problem.addConstraint(e7)

    # e8: b6^2+x3^2 <= 4.25
    e8 = SHOTpy.NonlinearConstraint("e8", b6**2 + x3**2, SHOTpy.SHOT_DBL_MIN, 4.25)
    problem.addConstraint(e8)

    # e9: b5^2+x3^2 <= 4.64
    e9 = SHOTpy.NonlinearConstraint("e9", b5**2 + x3**2, SHOTpy.SHOT_DBL_MIN, 4.64)
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

    x1 = SHOTpy.Variable("x1", SHOTpy.VariableType.Integer, 0.0, 3.0)
    x2 = SHOTpy.Variable("x2", SHOTpy.VariableType.Integer, 1.0, 3.0)
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
    e1 = SHOTpy.NonlinearConstraint("e1", e1_expr, SHOTpy.SHOT_DBL_MIN, 10.0)
    problem.addConstraint(e1)

    # e2: exp(x1) / x2 <= 3
    e2_expr = SHOTpy.exp(x1) / x2
    e2 = SHOTpy.NonlinearConstraint("e2", e2_expr, SHOTpy.SHOT_DBL_MIN, 3.0)
    problem.addConstraint(e2)

    problem.finalize()
    return problem, [e1, e2]


def build_shot_ex_jogo(env):
    """Return the shot_ex_jogo MINLP.

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

    x1 = SHOTpy.Variable("x1", SHOTpy.VariableType.Real,    1.0, 20.0)
    x2 = SHOTpy.Variable("x2", SHOTpy.VariableType.Integer, 1.0, 20.0)
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
    problem.addConstraint(SHOTpy.LinearConstraint("l", lt_l, SHOTpy.SHOT_DBL_MIN, 2.0))

    # Constraint c1 (index 1): 0.15*(x1-8)^2 + 0.1*(x2-6)^2 + 0.025*exp(x1)/x2^2 <= 5
    c1 = SHOTpy.NonlinearConstraint("c1", SHOTpy.SHOT_DBL_MIN, 5.0)
    c1.add(0.15 * (x1 - 8.0)**2 + 0.1 * (x2 - 6.0)**2 + 0.025 * SHOTpy.exp(x1) / x2**2)
    problem.addConstraint(c1)

    # Constraint c2 (index 2): 1/x1 + 1/x2 - sqrt(x1)*sqrt(x2) <= -4
    c2 = SHOTpy.NonlinearConstraint("c2", SHOTpy.SHOT_DBL_MIN, -4.0)
    c2.add(SHOTpy.SignomialTerm( 1.0, [(x1, -1.0)]))
    c2.add(SHOTpy.SignomialTerm( 1.0, [(x2, -1.0)]))
    c2.add(SHOTpy.SignomialTerm(-1.0, [(x1, 0.5), (x2, 0.5)]))
    problem.addConstraint(c2)

    problem.finalize()
    return problem


# Known solution of shot_ex_jogo
JOGO_KNOWN_OBJ   = -20.903615014500517   # primal bound
JOGO_KNOWN_DUAL  = -20.903647306104258   # dual bound
JOGO_KNOWN_POINT = [8.903615014500554, 12.0]


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestCallbackLocationEnum:
    """CallbackLocation is exposed and its values can be combined into masks."""

    LOCATIONS = [
        "InteriorPointSearch",
        "DualBoundUpdate",
        "PrimalCandidateSearch",
        "PrimalCandidateCheck",
        "NewPrimalSolution",
        "TerminationCheck",
        "HyperplaneSelection",
    ]

    def test_location_values(self):
        import SHOTpy
        assert hasattr(SHOTpy, "CallbackLocation")
        for name in self.LOCATIONS:
            assert hasattr(SHOTpy.CallbackLocation, name), f"Missing CallbackLocation.{name}"

    def test_locations_are_distinct_bits(self):
        import SHOTpy
        values = [int(getattr(SHOTpy.CallbackLocation, name)) for name in self.LOCATIONS]
        assert len(set(values)) == len(values)
        for value in values:
            assert value > 0 and value & (value - 1) == 0, f"{value} is not a single bit"

    def test_or_of_two_locations(self):
        import SHOTpy
        L = SHOTpy.CallbackLocation
        mask = L.PrimalCandidateCheck | L.TerminationCheck
        assert mask == int(L.PrimalCandidateCheck) + int(L.TerminationCheck)

    def test_or_of_three_locations(self):
        import SHOTpy
        L = SHOTpy.CallbackLocation
        mask = L.PrimalCandidateCheck | L.TerminationCheck | L.NewPrimalSolution
        assert mask == int(L.PrimalCandidateCheck) + int(L.TerminationCheck) + int(L.NewPrimalSolution)

    def test_and_of_mask_and_location(self):
        import SHOTpy
        L = SHOTpy.CallbackLocation
        mask = L.PrimalCandidateCheck | L.TerminationCheck
        assert mask & L.TerminationCheck == int(L.TerminationCheck)
        assert L.NewPrimalSolution & mask == 0


class TestContextClasses:
    """The context classes are exposed with their properties and actions."""

    def test_common_properties(self):
        import SHOTpy
        for attr in ["location", "isValid", "isMinimization", "iterationNumber", "elapsedTime", "dualBound",
                     "globalDualBound", "primalBound", "relativeGap", "absoluteGap", "solutionStatistics",
                     "originalProblem", "reformulatedProblem", "hasPrimalSolution", "primalSolution",
                     "isTerminationRequested", "isTerminationPending", "isFinalizing", "terminate"]:
            assert hasattr(SHOTpy.CallbackContext, attr), f"CallbackContext is missing {attr}"

    def test_location_classes(self):
        import SHOTpy
        expected = {
            "PrimalCandidateCheckContext": ["point", "objectiveValue", "source", "rejectCandidate",
                                            "isCandidateRejected"],
            "NewPrimalSolutionContext": ["point", "objectiveValue", "source", "isIncumbent"],
            "DualBoundUpdateContext": ["setDualBound", "proposedDualBound"],
            "PrimalCandidateSearchContext": ["addPrimalSolution", "addedPrimalSolutions"],
            "HyperplaneSelectionContext": ["solutionPoints", "addHyperplane", "addedHyperplanes"],
            "InteriorPointSearchContext": ["interiorPoints", "setInteriorPoints", "replacementInteriorPoints"],
            "TerminationCheckContext": [],
        }
        for name, attrs in expected.items():
            cls = getattr(SHOTpy, name)
            assert issubclass(cls, SHOTpy.CallbackContext)
            for attr in attrs:
                assert hasattr(cls, attr), f"{name} is missing {attr}"

    def test_context_expired_exception(self):
        import SHOTpy
        assert issubclass(SHOTpy.CallbackContextExpired, RuntimeError)

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


class TestRegisterCallbackAPI:
    """API-level checks for Solver.registerCallback and Solver.removeCallback."""

    def test_register_callback_method_exists(self, solver):
        assert hasattr(solver, "registerCallback")
        assert hasattr(solver, "removeCallback")

    def test_register_with_lambda(self, solver, env):
        """registerCallback accepts a lambda."""
        import SHOTpy
        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.updateSetting("Termination.IterationLimit", 2)
        solver.setProblem(build_ex1223b(env))
        solver.registerCallback(SHOTpy.CallbackLocation.NewPrimalSolution, lambda ctx: None)
        solver.solveProblem()  # Must not raise

    def test_register_with_callable_object(self, solver, env):
        """registerCallback accepts any callable (not just lambdas)."""
        import SHOTpy

        class Counter:
            def __init__(self):
                self.count = 0
            def __call__(self, ctx):
                self.count += 1

        counter = Counter()
        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.setProblem(build_ex1223b(env))
        solver.registerCallback(SHOTpy.CallbackLocation.NewPrimalSolution, counter)
        solver.solveProblem()
        assert counter.count >= 1

    def test_register_with_list_and_integer_mask(self, solver, env):
        """The locations can be given as a list or as an integer mask."""
        import SHOTpy
        L = SHOTpy.CallbackLocation

        from_list = set()
        from_mask = set()

        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.setProblem(build_ex1223b(env))
        solver.registerCallback([L.NewPrimalSolution, L.TerminationCheck],
                                lambda ctx: from_list.add(ctx.location))
        solver.registerCallback(int(L.NewPrimalSolution) | int(L.TerminationCheck),
                                lambda ctx: from_mask.add(ctx.location))
        solver.solveProblem()

        assert from_list == {L.NewPrimalSolution, L.TerminationCheck}
        assert from_mask == from_list

    @pytest.mark.parametrize("locations", [0, 1 << 20, -1, []])
    def test_invalid_mask_raises_value_error(self, solver, locations):
        with pytest.raises(ValueError):
            solver.registerCallback(locations, lambda ctx: None)

    @pytest.mark.parametrize("locations", ["TerminationCheck", 1.5, True, [1, 2]])
    def test_invalid_location_type_raises_type_error(self, solver, locations):
        with pytest.raises(TypeError):
            solver.registerCallback(locations, lambda ctx: None)

    def test_register_before_set_problem(self, solver, env):
        """A callback can be registered before the problem is set."""
        import SHOTpy

        calls = [0]
        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.registerCallback(SHOTpy.CallbackLocation.NewPrimalSolution,
                                lambda ctx: calls.__setitem__(0, calls[0] + 1))
        solver.setProblem(build_ex1223b(env))
        solver.solveProblem()

        assert calls[0] >= 1

    def test_remove_callback(self, solver, env):
        """A removed callback is not called, the others still are."""
        import SHOTpy
        L = SHOTpy.CallbackLocation

        removed_calls = [0]
        kept_calls = [0]

        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.setProblem(build_ex1223b(env))
        handle = solver.registerCallback(L.NewPrimalSolution,
                                         lambda ctx: removed_calls.__setitem__(0, removed_calls[0] + 1))
        solver.registerCallback(L.NewPrimalSolution, lambda ctx: kept_calls.__setitem__(0, kept_calls[0] + 1))

        assert solver.removeCallback(handle) is True
        assert solver.removeCallback(handle) is False
        solver.solveProblem()

        assert removed_calls[0] == 0
        assert kept_calls[0] >= 1

    def test_register_and_remove_during_solve_raise(self, solver, env):
        """Callbacks cannot be registered or removed while the problem is solved."""
        import SHOTpy
        L = SHOTpy.CallbackLocation

        errors = []

        def callback(ctx):
            if errors:
                return
            for action in (lambda: solver.registerCallback(L.NewPrimalSolution, lambda c: None),
                           lambda: solver.removeCallback(handle)):
                try:
                    action()
                    errors.append(None)
                except RuntimeError as e:
                    errors.append(e)

        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.setProblem(build_ex1223b(env))
        handle = solver.registerCallback(L.TerminationCheck, callback)
        solver.solveProblem()

        assert len(errors) == 2
        assert all(isinstance(e, RuntimeError) for e in errors), errors


class TestDispatch:
    """A callback is called at exactly the locations in its mask, with the class of the location."""

    def test_mask_selects_locations_and_classes(self, solver, env):
        import SHOTpy
        L = SHOTpy.CallbackLocation

        seen = []

        def callback(ctx):
            seen.append((ctx.location, type(ctx).__name__))

        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.setProblem(build_ex1223b(env))
        solver.registerCallback(L.PrimalCandidateCheck | L.TerminationCheck, callback)
        solver.solveProblem()

        assert {location for location, _ in seen} == {L.PrimalCandidateCheck, L.TerminationCheck}
        for location, class_name in seen:
            expected = ("PrimalCandidateCheckContext" if location == L.PrimalCandidateCheck
                        else "TerminationCheckContext")
            assert class_name == expected

    def test_common_values_have_python_types(self, solver, env):
        """The common values are Python numbers, and unavailable bounds are infinite."""
        import SHOTpy

        received = []

        def callback(ctx):
            received.append({
                "isMinimization": ctx.isMinimization,
                "iterationNumber": ctx.iterationNumber,
                "elapsedTime": ctx.elapsedTime,
                "dualBound": ctx.dualBound,
                "primalBound": ctx.primalBound,
                "relativeGap": ctx.relativeGap,
                "hasPrimalSolution": ctx.hasPrimalSolution,
                "primalSolution": ctx.primalSolution,
                "statistics": ctx.solutionStatistics.numberOfIterations,
            })

        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.setProblem(build_ex1223b(env))
        solver.registerCallback(SHOTpy.CallbackLocation.TerminationCheck, callback)
        solver.solveProblem()

        assert received
        for d in received:
            assert isinstance(d["isMinimization"], bool) and d["isMinimization"]
            assert isinstance(d["iterationNumber"], int)
            assert isinstance(d["elapsedTime"], float) and d["elapsedTime"] >= 0.0
            assert isinstance(d["dualBound"], float)
            assert isinstance(d["primalBound"], float)
            if d["hasPrimalSolution"]:
                assert isinstance(d["primalSolution"], list) and len(d["primalSolution"]) == 7
                assert math.isfinite(d["primalBound"])
            else:
                assert d["primalSolution"] is None
                assert d["primalBound"] == math.inf
                assert d["relativeGap"] == math.inf


class TestContextLifetime:
    """A context kept after its callback has returned raises instead of reading freed memory."""

    def test_retained_context_raises_after_callback(self, solver, env):
        import SHOTpy

        saved = []
        points = []

        def callback(ctx):
            assert ctx.isValid
            saved.append(ctx)
            points.append(ctx.point)

        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.setProblem(build_ex1223b(env))
        solver.registerCallback(SHOTpy.CallbackLocation.NewPrimalSolution, callback)
        solver.solveProblem()

        assert saved, "Callback was never called"
        ctx = saved[0]
        assert not ctx.isValid
        assert ctx.location == SHOTpy.CallbackLocation.NewPrimalSolution

        for access in (lambda: ctx.dualBound, lambda: ctx.point, lambda: ctx.originalProblem, ctx.terminate):
            with pytest.raises(SHOTpy.CallbackContextExpired):
                access()

        # What was copied out of the context is still usable
        assert len(points[0]) == 7
        assert all(isinstance(v, float) for v in points[0])


class TestNewPrimalSolutionCallback:
    """CallbackLocation.NewPrimalSolution: called when a primal solution has been stored."""

    def test_callback_is_called(self, solver, env):
        """Callback is invoked at least once when a primal solution is found."""
        import SHOTpy

        call_log = []

        def on_new_primal(ctx):
            call_log.append({
                "objValue": ctx.objectiveValue,
                "iteration": ctx.iterationNumber,
                "gap": ctx.relativeGap,
            })

        solver.updateSetting("Output.Console.LogLevel", 6)  # Off
        solver.setProblem(build_ex1223b(env))
        solver.registerCallback(SHOTpy.CallbackLocation.NewPrimalSolution, on_new_primal)
        solver.solveProblem()

        assert len(call_log) >= 1, "Callback was never called"
        # Objective values in the callback should be non-trivially large (problem has known opt ~4.58)
        best = min(e["objValue"] for e in call_log)
        assert best < 10.0

    def test_callback_receives_correct_types(self, solver, env):
        """The values of the context have the right Python types."""
        import SHOTpy

        received = []

        def on_new_primal(ctx):
            received.append((ctx.objectiveValue, ctx.iterationNumber, ctx.isMinimization, ctx.point, ctx.source,
                             ctx.isIncumbent))

        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.setProblem(build_ex1223b(env))
        solver.registerCallback(SHOTpy.CallbackLocation.NewPrimalSolution, on_new_primal)
        solver.solveProblem()

        assert received, "No callback data received"
        objective, iteration, is_minimization, point, source, is_incumbent = received[0]
        assert isinstance(objective, float)
        assert isinstance(iteration, int)
        assert isinstance(is_minimization, bool)
        assert isinstance(point, list)
        assert all(isinstance(v, float) for v in point)
        assert isinstance(source, SHOTpy.PrimalSolutionSource)
        assert is_incumbent is True, "The first solution is always better than the (missing) previous one"

    def test_is_incumbent_is_relative_to_the_best_solution(self):
        """isIncumbent compares with the best stored solution, not the worst one in the pool.

        The known optimum and two worse feasible points are posted in the order optimum, (7, 12) with objective
        -19, and (8, 12) with objective -20. With a pool of three solutions, (8, 12) is better than the worst
        stored solution but worse than the best one, so it is not an incumbent.
        """
        import SHOTpy
        L = SHOTpy.CallbackLocation

        solver = SHOTpy.Solver()
        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.updateSetting("Output.SaveNumberOfSolutions", 3)
        solver.updateSetting("Model.Convexity.AssumeConvex", True)
        solver.updateSetting("Dual.Relaxation.Use", False)
        solver.updateSetting("Dual.TreeStrategy", 0)  # MultiTree
        env = solver.getEnvironment()
        solver.setProblem(build_shot_ex_jogo(env))

        posted = [False]
        events = []

        def callback(ctx):
            if ctx.location == L.PrimalCandidateSearch and not posted[0]:
                posted[0] = True
                for point in (JOGO_KNOWN_POINT, [7.0, 12.0], [8.0, 12.0]):
                    ctx.addPrimalSolution(point)
            elif ctx.location == L.NewPrimalSolution:
                events.append((ctx.point, ctx.objectiveValue, ctx.isIncumbent))

        solver.registerCallback(L.PrimalCandidateSearch | L.NewPrimalSolution, callback)
        solver.solveProblem()

        assert posted[0], "The solutions were never posted"

        # Each solution is an incumbent exactly when it is better than all solutions stored before it
        best = math.inf
        for point, objective, is_incumbent in events:
            assert is_incumbent == (objective < best), (point, objective, is_incumbent, best)
            best = min(best, objective)

        by_point = {tuple(round(v, 6) for v in point): is_incumbent for point, _, is_incumbent in events}
        assert by_point.get((7.0, 12.0)) is False, events
        assert by_point.get((8.0, 12.0)) is False, events


class TestPrimalCandidateCheckCallback:
    """CallbackLocation.PrimalCandidateCheck: rejectCandidate() skips the feasibility check."""

    def test_callback_is_called(self, solver, env):
        """Callback fires at least once during a solve."""
        import SHOTpy

        call_log = []

        solver.updateSetting("Output.Console.LogLevel", 2)
        solver.setProblem(build_ex1223b(env))
        solver.registerCallback(SHOTpy.CallbackLocation.PrimalCandidateCheck,
                                lambda ctx: call_log.append(ctx.objectiveValue))
        solver.solveProblem()

        assert len(call_log) >= 1, "PrimalCandidateCheck callback was never called"

    def test_not_rejecting_accepts(self, solver, env):
        """A candidate that is not rejected is checked as usual, and a return value is ignored."""
        import SHOTpy

        solver.updateSetting("Output.Console.LogLevel", 2)
        solver.setProblem(build_ex1223b(env))
        solver.registerCallback(SHOTpy.CallbackLocation.PrimalCandidateCheck, lambda ctx: False)
        solver.solveProblem()

        assert solver.getPrimalSolutions(), "No primal solution found when no candidate is rejected"

    def test_reject_all_prevents_primal_solutions(self, solver, env):
        """Rejecting every candidate prevents any primal solution from being accepted."""
        import SHOTpy

        rejected = [0]

        def reject_all(ctx):
            rejected[0] += 1
            ctx.rejectCandidate()
            assert ctx.isCandidateRejected

        solver.updateSetting("Output.Console.LogLevel", 2)
        # Cap iterations so the test does not run forever with no primal bound
        solver.updateSetting("Termination.IterationLimit", 10)
        solver.setProblem(build_ex1223b(env))
        solver.registerCallback(SHOTpy.CallbackLocation.PrimalCandidateCheck, reject_all)
        solver.solveProblem()

        assert rejected[0] >= 1, "Reject callback was never called"
        # With all candidates rejected no primal solution should have been recorded
        assert not solver.getPrimalSolutions(), (
            "Expected no primal solutions when every candidate is rejected"
        )

    def test_selective_rejection_filters_accepted_candidates(self):
        """Rejected candidates never become incumbents, and the filter does not change the optimum."""
        import SHOTpy
        L = SHOTpy.CallbackLocation

        # ── Run 1: accept everything ──────────────────────────────────────────
        solver1 = SHOTpy.Solver()
        solver1.updateSetting("Output.Console.LogLevel", 2)
        env1 = solver1.getEnvironment()
        solver1.setProblem(build_ex1223b(env1))

        accepted_log1 = []
        solver1.registerCallback(L.NewPrimalSolution, lambda ctx: accepted_log1.append(ctx.objectiveValue))
        solver1.solveProblem()

        # ── Run 2: reject candidates with obj > 5 (sub-optimal ones) ─────────
        solver2 = SHOTpy.Solver()
        solver2.updateSetting("Output.Console.LogLevel", 2)
        env2 = solver2.getEnvironment()
        solver2.setProblem(build_ex1223b(env2))

        def reject_worse_than_five(ctx):
            if ctx.objectiveValue > 5.0:
                ctx.rejectCandidate()

        accepted_log2 = []
        solver2.registerCallback(L.PrimalCandidateCheck, reject_worse_than_five)
        solver2.registerCallback(L.NewPrimalSolution, lambda ctx: accepted_log2.append(ctx.objectiveValue))
        solver2.solveProblem()

        print(f"\n  accepted (all): {len(accepted_log1)}  accepted (filtered): {len(accepted_log2)}")

        # The number of incumbents is not a meaningful invariant: it depends on the order in which the
        # search happens to encounter improving solutions, and rejecting a candidate changes that order.
        # With bound tightening the unfiltered run reaches the optimum as its very first incumbent, so
        # there is no headroom for the filtered run to accept fewer. What the callback does guarantee is
        # that a rejected candidate never becomes an incumbent, and that filtering out only candidates
        # worse than the optimum still leaves the optimum reachable.
        assert accepted_log2, "The filter rejected every candidate, so it cannot be verified"
        assert all(value <= 5.0 + 1e-6 for value in accepted_log2), (
            f"A candidate rejected by the callback became an incumbent: {accepted_log2}"
        )
        assert abs(solver1.getPrimalBound() - solver2.getPrimalBound()) < 1e-4, (
            "Rejecting only candidates worse than the optimum must not change the optimum found: "
            f"{solver1.getPrimalBound()} vs {solver2.getPrimalBound()}"
        )


class TestTerminationCheckCallback:
    """CallbackLocation.TerminationCheck: terminate() stops the solver."""

    def test_callback_stops_solver(self, solver, env):
        """Calling terminate() in the callback terminates the solver early."""
        import SHOTpy

        termination_calls = []

        def should_terminate(ctx):
            termination_calls.append(ctx.iterationNumber)
            # Ask to stop after the 3rd check
            if len(termination_calls) >= 3:
                ctx.terminate()

        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.updateSetting("Termination.IterationLimit", 1000)
        solver.setProblem(build_ex1223b(env))
        solver.registerCallback(SHOTpy.CallbackLocation.TerminationCheck, should_terminate)
        solver.solveProblem()

        assert len(termination_calls) >= 3, "Termination callback was not called enough times"
        # The solver must have been told to terminate – iterations should be low
        stats = solver.getSolutionStatistics()
        assert stats.numberOfIterations < 100, (
            f"Expected solver to stop early, but ran {stats.numberOfIterations} iterations"
        )

    def test_not_terminating_continues(self, solver, env):
        """A callback that does not call terminate() lets the solver continue, and a return value is ignored."""
        import SHOTpy

        call_count = [0]

        def never_terminate(ctx):
            call_count[0] += 1
            return True  # Ignored: only terminate() stops the solver

        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.updateSetting("Termination.IterationLimit", 5)
        solver.setProblem(build_ex1223b(env))
        solver.registerCallback(SHOTpy.CallbackLocation.TerminationCheck, never_terminate)
        solver.solveProblem()

        assert call_count[0] >= 1
        assert solver.getTerminationReason() != SHOTpy.TerminationReason.UserAbort

    def test_terminate_from_primal_candidate_check(self, solver, env):
        """terminate() works at any location; the solver stops at its next termination check."""
        import SHOTpy

        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.updateSetting("Termination.IterationLimit", 1000)
        solver.setProblem(build_ex1223b(env))
        solver.registerCallback(SHOTpy.CallbackLocation.PrimalCandidateCheck, lambda ctx: ctx.terminate())
        solver.solveProblem()

        assert solver.getTerminationReason() == SHOTpy.TerminationReason.UserAbort
        assert solver.getSolutionStatistics().numberOfIterations < 100

    def test_terminate_from_interior_point_search_stops_before_the_first_dual_solve(self):
        """terminate() before the first dual problem is solved stops SHOT before it is solved."""
        import SHOTpy

        solver = SHOTpy.Solver()
        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.updateSetting("Model.Reformulation.Quadratics.Strategy", 0)  # Nonlinear -> ESH
        env = solver.getEnvironment()
        solver.setProblem(build_ex1223b(env))

        called = [False]

        def terminate(ctx):
            called[0] = True
            ctx.terminate()

        solver.registerCallback(SHOTpy.CallbackLocation.InteriorPointSearch, terminate)
        solver.solveProblem()

        stats = solver.getSolutionStatistics()
        assert called[0], "The interior point callback was not called"
        assert solver.getTerminationReason() == SHOTpy.TerminationReason.UserAbort
        assert stats.numberOfProblemsFeasibleMILP + stats.numberOfProblemsOptimalMILP + stats.numberOfProblemsLP == 0, (
            "A dual problem was solved after termination was requested"
        )

    def test_gap_reason_has_precedence_over_termination(self, solver, env):
        """A termination requested when a solution also closes the gap is reported as the gap reason."""
        import SHOTpy

        requested = [0]

        def terminate(ctx):
            requested[0] += 1
            ctx.terminate()

        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.setProblem(build_ex1223b(env))
        solver.registerCallback(SHOTpy.CallbackLocation.NewPrimalSolution, terminate)
        solver.solveProblem()

        reason = solver.getTerminationReason()
        assert requested[0] >= 1
        assert reason in (SHOTpy.TerminationReason.UserAbort, SHOTpy.TerminationReason.RelativeGap,
                          SHOTpy.TerminationReason.AbsoluteGap)

        gap_closed = (solver.getRelativeObjectiveGap() <= solver.getDoubleSetting("Termination.ObjectiveGap.Relative")
                      or solver.getAbsoluteObjectiveGap() <= solver.getDoubleSetting("Termination.ObjectiveGap.Absolute"))
        if gap_closed:
            assert reason != SHOTpy.TerminationReason.UserAbort

    def test_termination_pending_after_terminate(self):
        """After terminate() at one location, callbacks at other locations see isTerminationPending until SHOT stops."""
        import SHOTpy
        L = SHOTpy.CallbackLocation

        solver = SHOTpy.Solver()
        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.updateSetting("Model.Reformulation.Quadratics.Strategy", 0)  # Nonlinear -> multi-tree
        env = solver.getEnvironment()
        solver.setProblem(build_ex1223b(env))

        requested = [False]
        calls = []

        def callback(ctx):
            calls.append((ctx.location, requested[0], ctx.isTerminationPending, ctx.isFinalizing))
            if ctx.location == L.PrimalCandidateCheck and not requested[0]:
                requested[0] = True
                ctx.terminate()
                assert ctx.isTerminationRequested
                assert not ctx.isTerminationPending, "A request in this context is not pending yet"

        solver.registerCallback(L.PrimalCandidateCheck | L.NewPrimalSolution | L.PrimalCandidateSearch, callback)
        solver.solveProblem()

        assert requested[0]
        assert solver.getTerminationReason() == SHOTpy.TerminationReason.UserAbort
        before = [c for c in calls if not c[1]]
        after = [c for c in calls if c[1]][1:]  # the call that requested termination is the first one
        assert before and all(not pending for _, _, pending, _ in before)
        assert after, "No callback was called after termination was requested"
        assert all(pending for _, _, pending, _ in after), calls
        assert any(finalizing for _, _, _, finalizing in after), "The finalization was not reached"

    def test_is_finalizing(self):
        """isFinalizing is false in the main loop and true while the solution is finalized."""
        import SHOTpy
        L = SHOTpy.CallbackLocation

        solver = SHOTpy.Solver()
        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.updateSetting("Model.Reformulation.Quadratics.Strategy", 0)  # Nonlinear -> multi-tree
        env = solver.getEnvironment()
        solver.setProblem(build_ex1223b(env))

        calls = []
        solver.registerCallback(L.PrimalCandidateSearch | L.TerminationCheck,
                                lambda ctx: calls.append((ctx.location, ctx.isFinalizing, ctx.isTerminationPending)))
        solver.solveProblem()

        assert solver.getTerminationReason() != SHOTpy.TerminationReason.UserAbort
        assert all(not pending for _, _, pending in calls)
        assert all(not finalizing for location, finalizing, _ in calls if location == L.TerminationCheck)

        search = [finalizing for location, finalizing, _ in calls if location == L.PrimalCandidateSearch]
        assert len(search) >= 2
        assert search[0] is False, "The first primal search is in the main loop"
        assert search[-1] is True, "The primal search during finalization was not reported as finalizing"

    def test_callback_receives_structured_data(self, solver, env):
        """The values of the context are populated correctly."""
        import SHOTpy

        received = []

        def collect(ctx):
            received.append((ctx.iterationNumber, ctx.dualBound, ctx.primalBound, ctx.elapsedTime))
            if len(received) >= 2:
                ctx.terminate()

        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.setProblem(build_ex1223b(env))
        solver.registerCallback(SHOTpy.CallbackLocation.TerminationCheck, collect)
        solver.solveProblem()

        assert received
        iteration, dual_bound, primal_bound, elapsed = received[0]
        assert isinstance(iteration, int)
        assert isinstance(dual_bound, float)
        assert isinstance(primal_bound, float)
        assert isinstance(elapsed, float)
        assert elapsed >= 0.0


class TestHyperplaneSelectionCallback:
    """CallbackLocation.HyperplaneSelection: addHyperplane() adds a hyperplane."""

    def _make_solver(self, env_solver):
        solver = env_solver
        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.updateSetting("Dual.CutStrategy", 1)   # ECP
        solver.updateSetting("Dual.TreeStrategy", 1)  # SingleTree
        return solver

    def test_callback_is_called(self, solver, env):
        """The hyperplane callback is invoked with populated data."""
        import SHOTpy

        hyperplane_calls = []

        def provide_hyperplanes(ctx):
            hyperplane_calls.append({
                "iteration": ctx.iterationNumber,
                "n_points": len(ctx.solutionPoints),
            })
            # Adding no hyperplanes is valid; SHOT continues with its own cuts

        self._make_solver(solver)
        problem, _ = build_small_convex(env)
        solver.setProblem(problem, problem)
        solver.registerCallback(SHOTpy.CallbackLocation.HyperplaneSelection, provide_hyperplanes)
        solver.solveProblem()

        assert len(hyperplane_calls) >= 1, "Hyperplane callback was never called"

    @pytest.mark.parametrize("returned", [None, []])
    def test_return_value_is_ignored(self, solver, env, returned):
        """A value returned by the callback is ignored."""
        import SHOTpy

        self._make_solver(solver)
        problem, _ = build_small_convex(env)
        solver.setProblem(problem, problem)
        solver.registerCallback(SHOTpy.CallbackLocation.HyperplaneSelection, lambda ctx: returned)
        solver.solveProblem()

        assert solver.getPrimalSolutions()


class TestHyperplaneGradientCuts:
    """End-to-end test: solve shot_ex_jogo using *only* external gradient cuts.

    This exercises the full loop:
      - CutStrategy = OnlyExternal (2) — no built-in ESH/ECP cuts
      - The callback looks up the violated constraint by .index (global constraint
        index) rather than positional index, which is the correct approach when
        the problem has a mix of linear and nonlinear constraints.
    """

    def _make_solver_with_only_external_cuts(self):
        """Return a Solver configured to accept only external hyperplane cuts."""
        import SHOTpy
        solver = SHOTpy.Solver()
        solver.updateSetting("Output.Console.LogLevel", 2)
        solver.updateSetting("Model.Convexity.AssumeConvex", True)
        solver.updateSetting("Model.Reformulation.Constraint.PartitionQuadraticTerms", 2) # Never
        solver.updateSetting("Dual.Relaxation.Use", True)
        solver.updateSetting("Dual.CutStrategy", 2)    # OnlyExternal
        solver.updateSetting("Dual.TreeStrategy", 0)    # MultiTree
        return solver

    def test_solver_finds_solution_with_gradient_cuts(self):
        """Solver reaches a feasible solution when all cuts come from the callback."""
        import SHOTpy

        solver = self._make_solver_with_only_external_cuts()
        env = solver.getEnvironment()
        problem = build_shot_ex_jogo(env)
        solver.setProblem(problem, problem)

        cut_count = [0]

        def generate_hyperplanes(ctx):
            if not ctx.solutionPoints or ctx.iterationNumber == 0:
                return

            reform_problem = ctx.reformulatedProblem
            for sol_point in ctx.solutionPoints:
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
                ctx.addHyperplane(hp)
                cut_count[0] += 1
                break

        solver.registerCallback(SHOTpy.CallbackLocation.HyperplaneSelection, generate_hyperplanes)
        solver.solveProblem()

        assert cut_count[0] > 0, "Callback never generated any hyperplane cuts"
        assert solver.getPrimalSolutions(), "No primal solution found"
        obj = solver.getPrimalSolution().objValue
        # Known optimum: primal ≈ -20.9036, dual ≈ -20.9036
        # Accept any solution within 1% relative of the known primal bound.
        assert obj <= JOGO_KNOWN_OBJ * 0.99, (
            f"Objective {obj:.6f} is far from the known optimum ~-20.9036"
        )

    def test_constraint_lookup_by_index_not_position(self):
        """maxDeviation.index is a global constraint index, not a position in
        nonlinearConstraints.  This test verifies the lookup is correct by
        checking that the constraint name returned matches the expected constraint."""
        import SHOTpy

        solver = self._make_solver_with_only_external_cuts()
        # This test never adds cuts, so cap iterations to avoid running forever
        solver.updateSetting("Termination.IterationLimit", 20)
        env = solver.getEnvironment()
        problem = build_shot_ex_jogo(env)
        solver.setProblem(problem, problem)

        name_log = []

        def inspect_constraints(ctx):
            if not ctx.solutionPoints or ctx.iterationNumber == 0:
                return

            reform_problem = ctx.reformulatedProblem
            for sol_point in ctx.solutionPoints:
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
                break

        solver.registerCallback(SHOTpy.CallbackLocation.HyperplaneSelection, inspect_constraints)
        solver.solveProblem()

        assert name_log, "Callback never found a violated nonlinear constraint"
        # The names must be those of the nonlinear constraints, not the linear one
        assert set(name_log) <= {"c1", "c2"}, f"Lookup returned wrong constraints: {set(name_log)}"


class TestDualBoundAndPrimalSearchCallbacks:
    """CallbackLocation.DualBoundUpdate and CallbackLocation.PrimalCandidateSearch on shot_ex_jogo.

    Mirrors the C++ GurobiExternalDualBoundCallbackTest / CbcExternalDualBoundCallbackTest
    pattern, adapted to the Python API:
      - DualBoundUpdate:       ctx.setDualBound(value)
      - PrimalCandidateSearch: ctx.addPrimalSolution(point)

    Known solution for shot_ex_jogo:
      x1 ≈ 8.9036 (real), x2 = 12 (integer), obj ≈ -20.9036
    """

    def test_dual_bound_callback_is_called(self):
        """The DualBoundUpdate callback is invoked and can tighten the dual bound.

        The callback proposes the known optimal dual bound (-20.9036) once, while the
        solver's own dual bound is worse (smaller for minimization).
        """
        import SHOTpy

        solver = SHOTpy.Solver()
        solver.updateSetting("Output.Console.LogLevel", 2)
        solver.updateSetting("Dual.TreeStrategy", 0)  # MultiTree
        env = solver.getEnvironment()
        solver.setProblem(build_shot_ex_jogo(env))

        call_log = []
        injected = [False]

        def provide_dual_bound(ctx):
            call_log.append(ctx.dualBound)
            if not injected[0] and ctx.dualBound < JOGO_KNOWN_DUAL:
                injected[0] = True
                ctx.setDualBound(JOGO_KNOWN_DUAL)

        solver.registerCallback(SHOTpy.CallbackLocation.DualBoundUpdate, provide_dual_bound)
        solver.solveProblem()

        assert len(call_log) >= 1, "DualBoundUpdate callback was never called"
        assert call_log[0] == -math.inf, "The dual bound before the first dual solve must be -inf"
        assert injected[0], "Known dual bound was never injected"
        # Solver must still find a feasible solution
        assert solver.getPrimalSolutions(), "No primal solution found after dual bound injection"

    def test_dual_bound_callback_without_bound_is_safe(self):
        """A DualBoundUpdate callback that proposes nothing must not crash."""
        import SHOTpy

        solver = SHOTpy.Solver()
        solver.updateSetting("Output.Console.LogLevel", 2)
        solver.updateSetting("Dual.TreeStrategy", 0)  # MultiTree
        env = solver.getEnvironment()
        solver.setProblem(build_shot_ex_jogo(env))

        solver.registerCallback(SHOTpy.CallbackLocation.DualBoundUpdate, lambda ctx: None)
        solver.solveProblem()

        assert solver.getPrimalSolutions()

    def test_tightest_proposed_dual_bound_is_used(self):
        """With several callbacks, the tightest proposed bound is used, and none sees the others' proposals
        in the committed dual bound."""
        import SHOTpy
        L = SHOTpy.CallbackLocation

        solver = SHOTpy.Solver()
        solver.updateSetting("Output.Console.LogLevel", 2)
        solver.updateSetting("Dual.TreeStrategy", 0)  # MultiTree
        env = solver.getEnvironment()
        solver.setProblem(build_shot_ex_jogo(env))

        first_call = []

        def propose_minus_30(ctx):
            ctx.setDualBound(-30.0)

        def propose_minus_25(ctx):
            ctx.setDualBound(-25.0)

        def observe(ctx):
            if not first_call:
                first_call.append((ctx.dualBound, ctx.proposedDualBound))
            else:
                first_call.append((ctx.dualBound, None))

        solver.registerCallback(L.DualBoundUpdate, propose_minus_30)
        solver.registerCallback(L.DualBoundUpdate, propose_minus_25)
        solver.registerCallback(L.DualBoundUpdate, observe)
        solver.solveProblem()

        assert len(first_call) >= 2
        committed, proposed = first_call[0]
        assert committed == -math.inf, "A proposal must not be visible in the committed dual bound"
        assert proposed == -25.0, "The tightest proposal must be kept"
        assert first_call[1][0] >= -25.0, "The tightest proposal must have been used as the dual bound"

    def test_primal_candidate_search_callback_is_called(self):
        """Injecting a pre-verified optimal solution in the first iteration leads to
        early termination (mirrors the C++ GurobiExternalDualBoundCallbackTest pattern).

        Phase 1: solve shot_ex_jogo and take the optimal primal solution that SHOT
                 has already verified as feasible (no rounding issues).
        Phase 2: post that solution through PrimalCandidateSearch at iteration 1.
                 With the primal bound immediately tight, the solver should
                 terminate in no more iterations than in Phase 1.
        """
        import SHOTpy
        L = SHOTpy.CallbackLocation

        # ── Phase 1: the verified optimal primal solution ─────────────
        solver1 = SHOTpy.Solver()
        solver1.updateSetting("Output.Console.LogLevel", 6)
        solver1.updateSetting("Model.Convexity.AssumeConvex", True)
        env1 = solver1.getEnvironment()
        solver1.setProblem(build_shot_ex_jogo(env1), build_shot_ex_jogo(env1))

        collected_solutions = []
        solver1.registerCallback(L.NewPrimalSolution, lambda ctx: collected_solutions.append(ctx.point))
        solver1.solveProblem()
        iters_phase1 = solver1.getSolutionStatistics().numberOfIterations
        print(f"\n  Phase 1 finished: {iters_phase1} iterations, "
              f"{len(collected_solutions)} primal solutions collected")
        assert collected_solutions, "No primal solutions collected in phase 1"

        best_solution = list(solver1.getPrimalSolution().point)
        best_obj      = solver1.getPrimalSolution().objValue
        print(f"  Best solution: {best_solution}  obj={best_obj:.6f}")

        # ── Phase 2: inject in the first iteration ────────────────────────────
        solver2 = SHOTpy.Solver()
        solver2.updateSetting("Output.Console.LogLevel", 2)
        solver2.updateSetting("Model.Convexity.AssumeConvex", True)
        solver2.updateSetting("Dual.Relaxation.Use", False)
        solver2.updateSetting("Dual.TreeStrategy", 0)    # MultiTree
        env2 = solver2.getEnvironment()
        solver2.setProblem(build_shot_ex_jogo(env2), build_shot_ex_jogo(env2))

        provided = [False]
        call_log  = []

        def on_new_primal(ctx):
            print(f"  [NewPrimalSolution]      iter={ctx.iterationNumber}  obj={ctx.objectiveValue:.6f}")

        def provide_primal_solution(ctx):
            call_log.append(ctx.iterationNumber)
            print(f"  [PrimalCandidateSearch] iter={ctx.iterationNumber}  "
                  f"dual={ctx.dualBound:.4f}  primal={ctx.primalBound:.4f}")
            if not provided[0]:
                provided[0] = True
                print(f"    -> injecting phase-1 optimal solution: {best_solution}")
                ctx.addPrimalSolution(best_solution)

        solver2.registerCallback(L.NewPrimalSolution,     on_new_primal)
        solver2.registerCallback(L.PrimalCandidateSearch, provide_primal_solution)
        solver2.solveProblem()

        iters_phase2 = solver2.getSolutionStatistics().numberOfIterations
        obj2         = solver2.getPrimalSolution().objValue
        print(f"\n  Phase 2 finished: {iters_phase2} iterations  obj={obj2:.6f}")

        assert call_log, "PrimalCandidateSearch callback never called"
        assert solver2.getPrimalSolutions(), "No primal solution found in phase 2"
        assert obj2 <= JOGO_KNOWN_OBJ * 0.99, f"Objective {obj2:.6f} far from optimum"
        # Injecting the optimal solution up-front should mean phase 2 needs no more
        # iterations than phase 1 (and typically fewer)
        assert iters_phase2 <= iters_phase1, (
            f"Expected phase 2 ({iters_phase2} iter) to be no worse than "
            f"phase 1 ({iters_phase1} iter)"
        )

    def test_several_primal_solutions_can_be_added(self):
        """addPrimalSolution can be called several times in one callback."""
        import SHOTpy
        L = SHOTpy.CallbackLocation

        solver = SHOTpy.Solver()
        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.updateSetting("Output.SaveNumberOfSolutions", 3)
        solver.updateSetting("Model.Convexity.AssumeConvex", True)
        solver.updateSetting("Dual.Relaxation.Use", False)
        solver.updateSetting("Dual.TreeStrategy", 0)  # MultiTree
        env = solver.getEnvironment()
        solver.setProblem(build_shot_ex_jogo(env))

        posted = [False]
        added = []
        external = []

        def callback(ctx):
            if ctx.location == L.PrimalCandidateSearch:
                if not posted[0]:
                    posted[0] = True
                    ctx.addPrimalSolution([7.0, 12.0])
                    ctx.addPrimalSolution([8.0, 11.0])
                    added.extend(ctx.addedPrimalSolutions)
            elif ctx.source == SHOTpy.PrimalSolutionSource.ExternalPrimalSolution:
                external.append(ctx.point)

        solver.registerCallback(L.PrimalCandidateSearch | L.NewPrimalSolution, callback)
        solver.solveProblem()

        assert added == [[7.0, 12.0], [8.0, 11.0]]
        assert len(external) == 2, f"Both posted solutions should have been stored: {external}"

    def test_primal_candidate_search_without_solutions_is_safe(self):
        """A PrimalCandidateSearch callback that adds nothing must not crash."""
        import SHOTpy

        solver = SHOTpy.Solver()
        solver.updateSetting("Output.Console.LogLevel", 2)
        env = solver.getEnvironment()
        solver.setProblem(build_shot_ex_jogo(env))

        solver.registerCallback(SHOTpy.CallbackLocation.PrimalCandidateSearch, lambda ctx: None)
        solver.solveProblem()

        assert solver.getPrimalSolutions()

    def test_dual_and_primal_callbacks_together(self):
        """DualBoundUpdate and PrimalCandidateSearch both inject values that the solver accepts.

        - DualBoundUpdate proposes the known optimal dual bound on the first call where
          the solver's own bound is still worse.
        - PrimalCandidateSearch posts the known optimal point on the first call.
          A NewPrimalSolution call near the known objective confirms the point was
          accepted (not silently discarded).
        """
        import SHOTpy
        L = SHOTpy.CallbackLocation

        solver = SHOTpy.Solver()
        solver.updateSetting("Output.Console.LogLevel", 2)
        solver.updateSetting("Model.Convexity.AssumeConvex", True)
        solver.updateSetting("Dual.Relaxation.Use", False)
        solver.updateSetting("Dual.TreeStrategy", 0)   # MultiTree
        env = solver.getEnvironment()
        problem = build_shot_ex_jogo(env)
        solver.setProblem(problem, problem)

        dual_calls      = [0]
        primal_calls    = [0]
        dual_injected   = [False]
        primal_injected = [False]
        primal_accepted = [False]

        def on_new_primal(ctx):
            print(f"  [NewPrimalSolution]        iter={ctx.iterationNumber:3d}  "
                  f"obj={ctx.objectiveValue:.6f}  point={[round(v,4) for v in ctx.point]}")
            # Confirm the injected primal solution was accepted by the solver
            if primal_injected[0] and abs(ctx.objectiveValue - JOGO_KNOWN_OBJ) < 0.01:
                primal_accepted[0] = True

        def provide_dual(ctx):
            dual_calls[0] += 1
            # Propose the known dual bound once while the solver's bound is still worse
            if not dual_injected[0] and ctx.dualBound < JOGO_KNOWN_DUAL:
                dual_injected[0] = True
                ctx.setDualBound(JOGO_KNOWN_DUAL)

        def provide_primal(ctx):
            primal_calls[0] += 1
            if not primal_injected[0]:
                primal_injected[0] = True
                ctx.addPrimalSolution(JOGO_KNOWN_POINT)

        solver.registerCallback(L.NewPrimalSolution,     on_new_primal)
        solver.registerCallback(L.DualBoundUpdate,       provide_dual)
        solver.registerCallback(L.PrimalCandidateSearch, provide_primal)
        solver.solveProblem()

        print(f"\n  Summary: dual_calls={dual_calls[0]}  primal_calls={primal_calls[0]}  "
              f"dual_injected={dual_injected[0]}  primal_injected={primal_injected[0]}  "
              f"primal_accepted={primal_accepted[0]}")
        assert dual_calls[0]   >= 1, "DualBoundUpdate callback never called"
        assert primal_calls[0] >= 1, "PrimalCandidateSearch callback never called"
        assert dual_injected[0],   "Known dual bound was never injected (condition never met)"
        assert primal_injected[0], "Known primal point was never injected"
        assert primal_accepted[0], (
            f"Injected primal point was not accepted: NewPrimalSolution never called "
            f"near obj={JOGO_KNOWN_OBJ:.6f}"
        )
        assert solver.getPrimalSolutions(), "No primal solution found"


class TestInvalidActions:
    """Actions with malformed input raise ValueError inside the callback."""

    def test_invalid_primal_solutions_and_dual_bounds(self):
        import SHOTpy
        L = SHOTpy.CallbackLocation

        solver = SHOTpy.Solver()
        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.updateSetting("Dual.TreeStrategy", 0)  # MultiTree
        env = solver.getEnvironment()
        solver.setProblem(build_shot_ex_jogo(env))

        errors = {}

        def record(name, action):
            if name in errors:
                return
            try:
                action()
                errors[name] = None
            except ValueError as e:
                errors[name] = e

        def callback(ctx):
            if ctx.location == L.PrimalCandidateSearch:
                record("too short", lambda: ctx.addPrimalSolution([1.0]))
                record("nan", lambda: ctx.addPrimalSolution([math.nan, 12.0]))
                record("inf", lambda: ctx.addPrimalSolution([8.0, math.inf]))
                assert ctx.addedPrimalSolutions == []
            elif ctx.location == L.DualBoundUpdate:
                record("nan bound", lambda: ctx.setDualBound(math.nan))
                record("inf bound", lambda: ctx.setDualBound(-math.inf))
                assert ctx.proposedDualBound is None

        solver.registerCallback(L.PrimalCandidateSearch | L.DualBoundUpdate, callback)
        solver.solveProblem()

        assert set(errors) == {"too short", "nan", "inf", "nan bound", "inf bound"}
        for name, error in errors.items():
            assert isinstance(error, ValueError), f"{name}: {error!r}"

        # The solve is not affected by the rejected actions
        assert abs(solver.getPrimalBound() - JOGO_KNOWN_OBJ) < 1e-3

    def test_invalid_hyperplanes(self):
        import SHOTpy

        solver = SHOTpy.Solver()
        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.updateSetting("Dual.TreeStrategy", 0)  # MultiTree
        env = solver.getEnvironment()
        problem = build_shot_ex_jogo(env)
        solver.setProblem(problem, problem)

        def hyperplane(indexes, coefficients, rhs=0.0):
            hp = SHOTpy.ExternalHyperplane()
            hp.variableIndexes = indexes
            hp.variableCoefficients = coefficients
            hp.rhsValue = rhs
            hp.source = SHOTpy.HyperplaneSource.External
            return hp

        errors = {}

        def callback(ctx):
            if errors:
                return
            cases = {
                "index out of range": hyperplane([0, 100], [1.0, 1.0]),
                "negative index": hyperplane([-1], [1.0]),
                "mismatched lengths": hyperplane([0, 1], [1.0]),
                "no variables": hyperplane([], []),
                "nan coefficient": hyperplane([0], [math.nan]),
                "inf rhs": hyperplane([0], [1.0], math.inf),
            }
            for name, hp in cases.items():
                try:
                    ctx.addHyperplane(hp)
                    errors[name] = None
                except ValueError as e:
                    errors[name] = e
            assert ctx.addedHyperplanes == []

        solver.registerCallback(SHOTpy.CallbackLocation.HyperplaneSelection, callback)
        solver.solveProblem()

        assert len(errors) == 6
        for name, error in errors.items():
            assert isinstance(error, ValueError), f"{name}: {error!r}"

    def test_invalid_interior_points(self):
        import SHOTpy

        solver = SHOTpy.Solver()
        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.updateSetting("Model.Reformulation.Quadratics.Strategy", 0)  # Nonlinear -> ESH
        env = solver.getEnvironment()
        solver.setProblem(build_ex1223b(env))

        errors = {}

        def callback(ctx):
            for name, points in {"empty": [], "too short": [[1.0]], "nan": [[math.nan] * 7]}.items():
                try:
                    ctx.setInteriorPoints(points)
                    errors[name] = None
                except ValueError as e:
                    errors[name] = e
            assert ctx.replacementInteriorPoints is None

        solver.registerCallback(SHOTpy.CallbackLocation.InteriorPointSearch, callback)
        solver.solveProblem()

        assert set(errors) == {"empty", "too short", "nan"}
        for name, error in errors.items():
            assert isinstance(error, ValueError), f"{name}: {error!r}"
        assert solver.getPrimalSolutions()


class TestCallbackFailure:
    """An exception in a callback stops SHOT and is raised by solveProblem()."""

    def test_exception_is_raised_by_solve_problem(self):
        import SHOTpy
        L = SHOTpy.CallbackLocation

        solver = SHOTpy.Solver()
        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.updateSetting("Dual.TreeStrategy", 0)  # MultiTree
        env = solver.getEnvironment()
        solver.setProblem(build_shot_ex_jogo(env))

        failed = [False]
        calls_after_failure = []
        later_callback_calls = [0]

        class CallbackError(Exception):
            pass

        def failing(ctx):
            if ctx.location == L.PrimalCandidateSearch and not failed[0]:
                # The solution is queued, but must be discarded when the callback fails
                ctx.addPrimalSolution(JOGO_KNOWN_POINT)
                failed[0] = True
                raise CallbackError("the callback failed")
            if failed[0]:
                calls_after_failure.append(ctx.location)

        def later(ctx):
            if ctx.location == L.PrimalCandidateSearch:
                later_callback_calls[0] += 1
            if failed[0]:
                calls_after_failure.append(ctx.location)

        solver.registerCallback(L.PrimalCandidateSearch | L.NewPrimalSolution | L.TerminationCheck, failing)
        solver.registerCallback(L.PrimalCandidateSearch | L.NewPrimalSolution | L.TerminationCheck, later)

        with pytest.raises(CallbackError, match="the callback failed") as info:
            solver.solveProblem()

        # The traceback of the Python exception is kept
        assert any(entry.name == "failing" for entry in info.traceback)

        assert failed[0]
        assert later_callback_calls[0] == 0, "A callback after the failing one was called"
        assert calls_after_failure == [], f"Callbacks were called after the failure: {calls_after_failure}"
        assert solver.getTerminationReason() == SHOTpy.TerminationReason.Error

        for solution in solver.getPrimalSolutions():
            assert solution.sourceType != SHOTpy.PrimalSolutionSource.ExternalPrimalSolution, (
                "The solution queued by the failing callback was added"
            )

    def test_invalid_action_fails_the_solve(self, solver, env):
        """An invalid action that is not caught in the callback fails the solve with a ValueError."""
        import SHOTpy

        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.setProblem(build_ex1223b(env))
        solver.registerCallback(SHOTpy.CallbackLocation.PrimalCandidateSearch,
                                lambda ctx: ctx.addPrimalSolution([1.0]))

        with pytest.raises(ValueError):
            solver.solveProblem()

        assert solver.getTerminationReason() == SHOTpy.TerminationReason.Error


class TestThreads:
    """Python callbacks called from the threads of a multithreaded MIP solver do not deadlock."""

    SCRIPT = """
import sys
sys.path.insert(0, {build_dir!r})
import SHOTpy

L = SHOTpy.CallbackLocation
solver = SHOTpy.Solver()
solver.updateSetting("Output.Console.LogLevel", 6)
solver.updateSetting("Dual.MIP.Solver", 0)             # Cplex
solver.updateSetting("Dual.TreeStrategy", 1)           # SingleTree
solver.updateSetting("Dual.MIP.NumberOfThreads", 4)
solver.updateSetting("Termination.TimeLimit", 60.0)

if not solver.setProblem({filename!r}):
    sys.exit(3)

if solver.getIntSetting("Dual.MIP.Solver") != 0:
    sys.exit(2)

import threading

threads = dict()

def callback(ctx):
    threads.setdefault(ctx.location.name, set()).add(threading.get_ident())
    _ = (ctx.dualBound, ctx.primalBound, ctx.primalSolution)

solver.registerCallback(L.TerminationCheck | L.DualBoundUpdate | L.PrimalCandidateSearch, callback)
solver.solveProblem()
print(sorted((name, len(idents)) for name, idents in threads.items()))
"""

    def test_cplex_single_tree_with_threads(self, data_dir):
        import subprocess
        import sys
        from conftest import BUILD_DIR

        # CPLEX calls the single-tree callback of this instance from all its threads
        script = self.SCRIPT.format(build_dir=BUILD_DIR, filename=str(data_dir / "tls2.osil"))

        try:
            result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, timeout=60)
        except subprocess.TimeoutExpired:
            pytest.fail("The solve with Python callbacks from CPLEX threads did not finish (deadlock?)")

        if result.returncode == 2:
            pytest.skip("SHOT is not built with CPLEX")

        assert result.returncode == 0, result.stdout + result.stderr
        threads = dict(eval(result.stdout.strip().splitlines()[-1]))
        assert threads.get("TerminationCheck", 0) >= 1, threads
        assert max(threads.values()) > 1, f"The callbacks were not called from several threads: {threads}"


class TestMultipleCallbacksForInteriorPoints:
    """With several callbacks, the last setInteriorPoints() is used, and none sees the others' replacements
    in the committed interior points."""

    def test_last_replacement_wins(self):
        import SHOTpy
        L = SHOTpy.CallbackLocation

        solver = SHOTpy.Solver()
        solver.updateSetting("Output.Console.LogLevel", 6)
        solver.updateSetting("Model.Reformulation.Quadratics.Strategy", 0)  # Nonlinear -> ESH
        env = solver.getEnvironment()
        solver.setProblem(build_ex1223b(env))

        observed = {}

        def first(ctx):
            observed["internal"] = ctx.interiorPoints
            ctx.setInteriorPoints([ctx.interiorPoints[0]])

        def second(ctx):
            observed["seen by second"] = ctx.interiorPoints
            observed["pending before second"] = ctx.replacementInteriorPoints
            replacement = [list(ctx.interiorPoints[0])]
            ctx.setInteriorPoints(replacement)
            observed["pending after second"] = ctx.replacementInteriorPoints
            observed["second replacement"] = replacement

        solver.registerCallback(L.InteriorPointSearch, first)
        solver.registerCallback(L.InteriorPointSearch, second)
        solver.solveProblem()

        assert observed["internal"], "The internal search found no interior points"
        assert observed["seen by second"] == observed["internal"]
        assert observed["pending before second"] == [observed["internal"][0]]
        assert observed["pending after second"] == observed["second replacement"]
        assert solver.getPrimalSolutions()


# ---------------------------------------------------------------------------
# ESH interior point callback tests
# ---------------------------------------------------------------------------

class TestCallbackESHInteriorPoint:
    """Tests for the InteriorPointSearch callback.

    Uses ex1223b as the test problem (built with build_ex1223b helper).
    """

    def test_callback_fires_during_normal_esh(self):
        """Callback fires during a normal ESH solve and receives interior point(s)."""
        import SHOTpy

        solver = SHOTpy.Solver()
        solver.updateSetting("Output.Console.LogLevel", 6)  # Off
        solver.updateSetting("Model.Reformulation.Quadratics.Strategy", 0)  # Nonlinear -> multi-tree/ESH
        env = solver.getEnvironment()
        solver.setProblem(build_ex1223b(env))

        callback_fired = [False]
        points_received = [0]

        def on_interior_points(ctx):
            callback_fired[0] = True
            points_received[0] = len(ctx.interiorPoints)
            print(f"  [InteriorPointSearch] callback fired, {points_received[0]} current point(s)")
            # No points are set, so the current points are kept

        solver.registerCallback(SHOTpy.CallbackLocation.InteriorPointSearch, on_interior_points)
        solver.solveProblem()

        assert callback_fired[0], "ESH interior point callback was not fired"
        assert points_received[0] > 0, (
            "Callback fired but received no interior points from the internal strategy"
        )

    def test_callback_only_external_strategy(self):
        """Two-phase test: extract interior point, then inject via OnlyExternal strategy.

        Phase 1: Solve ex1223b with the default ESH strategy; record the interior point
                 found by the internal cutting-plane minimax solver and the optimal primal
                 objective value.

        Phase 2: Solve ex1223b again with ESH.InteriorPoint.Strategy = OnlyExternal.
                 Provide the Phase 1 interior point through the callback.  The solver must
                 reach the same optimal objective (within 1e-4 relative tolerance).
        """
        import SHOTpy

        tol = 1e-4

        # ── Phase 1 ──────────────────────────────────────────────────────────
        solver1 = SHOTpy.Solver()
        solver1.updateSetting("Output.Console.LogLevel", 6)
        solver1.updateSetting("Model.Reformulation.Quadratics.Strategy", 0)  # Nonlinear -> multi-tree/ESH
        env1 = solver1.getEnvironment()
        solver1.setProblem(build_ex1223b(env1))

        captured_points = []

        def capture_interior_points(ctx):
            # Store a copy of whatever the internal solver found
            captured_points.extend(ctx.interiorPoints)

        solver1.registerCallback(SHOTpy.CallbackLocation.InteriorPointSearch, capture_interior_points)
        solver1.solveProblem()

        assert solver1.getPrimalSolutions(), "Phase 1: no primal solution found"
        phase1_obj = solver1.getPrimalSolution().objValue
        print(f"\n  Phase 1 objective: {phase1_obj:.6f}")

        assert captured_points, "Phase 1: no interior point captured from internal strategy"
        print(f"  Captured {len(captured_points)} interior point(s) from Phase 1")

        # ── Phase 2 ──────────────────────────────────────────────────────────
        solver2 = SHOTpy.Solver()
        solver2.updateSetting("Output.Console.LogLevel", 6)
        solver2.updateSetting("Model.Reformulation.Quadratics.Strategy", 0)  # Nonlinear -> multi-tree/ESH
        solver2.updateSetting("Dual.ESH.InteriorPoint.Strategy", 1)  # OnlyExternal
        env2 = solver2.getEnvironment()
        solver2.setProblem(build_ex1223b(env2))

        callback_fired = [False]
        points_given = [None]

        def provide_interior_points(ctx):
            callback_fired[0] = True
            points_given[0] = len(ctx.interiorPoints)
            print(f"  [OnlyExternal] callback fired, injecting {len(captured_points)} point(s)")
            ctx.setInteriorPoints(captured_points)  # inject the Phase-1 interior points

        solver2.registerCallback(SHOTpy.CallbackLocation.InteriorPointSearch, provide_interior_points)
        solver2.solveProblem()

        assert callback_fired[0], "Phase 2: ESH interior point callback was not fired"
        assert points_given[0] == 0, (
            "OnlyExternal: callback received interior points (internal strategy should not have run)"
        )
        assert solver2.getPrimalSolutions(), "Phase 2: no primal solution found"

        phase2_obj = solver2.getPrimalSolution().objValue
        print(f"  Phase 2 objective: {phase2_obj:.6f}")

        assert abs(phase2_obj - phase1_obj) <= tol + tol * abs(phase1_obj), (
            f"Phase 2 objective {phase2_obj:.6f} differs from Phase 1 ({phase1_obj:.6f}) "
            f"by more than tolerance {tol}"
        )

    def test_aux_problem_interior_point_injection(self):
        """Auxiliary-problem approach: find an interior point by minimising a slack variable.

        Phase 1: Solve an auxiliary version of ex1223b where:
          - binary variables b4-b7 are relaxed to Real [0, 1],
          - an auxiliary variable mu (Real [-100, 100]) is added,
          - mu is subtracted from each quadratic/nonlinear constraint,
          - the objective is ``minimize mu``.
          Optimal mu < 0 guarantees a strictly interior point.

        Phase 2: Use the auxiliary-problem point as the only interior point (via the
          OnlyExternal strategy) and verify that ex1223b is still solved to optimality.
        """
        import SHOTpy

        # ── Phase 1: auxiliary minimize-mu problem ────────────────────────────
        solver_aux = SHOTpy.Solver()
        solver_aux.updateSetting("Output.Console.LogLevel", 6)
        solver_aux.updateSetting("Model.Reformulation.Quadratics.Strategy", 0)  # Nonlinear
        env_aux = solver_aux.getEnvironment()

        problem_aux = SHOTpy.Problem(env_aux)
        problem_aux.name = "ex1223b_interior"

        # Original 7 variables, b4-b7 relaxed to Real [0, 1]
        x1 = SHOTpy.Variable("x1", SHOTpy.VariableType.Real, 0.0, 10.0)
        x2 = SHOTpy.Variable("x2", SHOTpy.VariableType.Real, 0.0, 10.0)
        x3 = SHOTpy.Variable("x3", SHOTpy.VariableType.Real, 0.0, 10.0)
        b4 = SHOTpy.Variable("b4", SHOTpy.VariableType.Real, 0.0, 1.0)
        b5 = SHOTpy.Variable("b5", SHOTpy.VariableType.Real, 0.0, 1.0)
        b6 = SHOTpy.Variable("b6", SHOTpy.VariableType.Real, 0.0, 1.0)
        b7 = SHOTpy.Variable("b7", SHOTpy.VariableType.Real, 0.0, 1.0)
        mu = SHOTpy.Variable("mu", SHOTpy.VariableType.Real, -100.0, 100.0)
        for v in [x1, x2, x3, b4, b5, b6, b7, mu]:
            problem_aux.addVariable(v)

        # Objective: minimize mu
        lt_obj = SHOTpy.LinearTerms()
        lt_obj.add(SHOTpy.LinearTerm(1.0, mu))
        problem_aux.setObjective(
            SHOTpy.LinearObjectiveFunction(SHOTpy.ObjectiveDirection.Minimize, lt_obj, 0.0)
        )

        # e1: x1+x2+x3+b4+b5+b6 <= 5  (linear, unchanged)
        lt1 = SHOTpy.LinearTerms()
        for v in [x1, x2, x3, b4, b5, b6]:
            lt1.add(SHOTpy.LinearTerm(1.0, v))
        problem_aux.addConstraint(
            SHOTpy.LinearConstraint("e1", lt1, SHOTpy.SHOT_DBL_MIN, 5.0)
        )

        # e2: b6^2+x1^2+x2^2+x3^2 - mu <= 5.5  (quadratic - mu)
        problem_aux.addConstraint(SHOTpy.NonlinearConstraint("e2", b6**2 + x1**2 + x2**2 + x3**2 - mu, SHOTpy.SHOT_DBL_MIN, 5.5
        ))

        # e3-e6: linear constraints, unchanged
        for idx, (va, vb, rhs, nm) in enumerate(
                [(x1, b4, 1.2, "e3"), (x2, b5, 1.8, "e4"),
                 (x3, b6, 2.5, "e5"), (x1, b7, 1.2, "e6")], start=2):
            lt = SHOTpy.LinearTerms()
            lt.add(SHOTpy.LinearTerm(1.0, va))
            lt.add(SHOTpy.LinearTerm(1.0, vb))
            problem_aux.addConstraint(
                SHOTpy.LinearConstraint(nm, lt, SHOTpy.SHOT_DBL_MIN, rhs)
            )

        # e7: b5^2+x2^2 - mu <= 1.64
        problem_aux.addConstraint(SHOTpy.NonlinearConstraint("e7", b5**2 + x2**2 - mu, SHOTpy.SHOT_DBL_MIN, 1.64
        ))
        # e8: b6^2+x3^2 - mu <= 4.25
        problem_aux.addConstraint(SHOTpy.NonlinearConstraint("e8", b6**2 + x3**2 - mu, SHOTpy.SHOT_DBL_MIN, 4.25
        ))
        # e9: b5^2+x3^2 - mu <= 4.64
        problem_aux.addConstraint(SHOTpy.NonlinearConstraint("e9", b5**2 + x3**2 - mu, SHOTpy.SHOT_DBL_MIN, 4.64
        ))

        problem_aux.finalize()
        solver_aux.setProblem(problem_aux)
        solver_aux.solveProblem()

        assert solver_aux.getPrimalSolutions(), "Auxiliary problem: no solution found"
        aux_sol = solver_aux.getPrimalSolution()
        optimal_mu = aux_sol.objValue
        print(f"\n  Auxiliary problem: optimal mu = {optimal_mu:.6f}")
        if optimal_mu < 0.0:
            print(f"  mu < 0: point is strictly interior to all quadratic constraints")
        else:
            print(f"  Warning: mu >= 0; point may not be strictly interior")

        # The aux problem has 8 variables (x1..b7, mu).  Drop mu (index 7).
        interior_point = list(aux_sol.point[:7])
        print(f"  Interior point (mu dropped): {[round(v, 4) for v in interior_point]}")

        # ── Phase 2: inject aux-problem interior point via OnlyExternal ───────
        solver2 = SHOTpy.Solver()
        solver2.updateSetting("Output.Console.LogLevel", 6)
        solver2.updateSetting("Model.Reformulation.Quadratics.Strategy", 0)  # Nonlinear
        solver2.updateSetting("Dual.ESH.InteriorPoint.Strategy", 1)  # OnlyExternal
        env2 = solver2.getEnvironment()
        solver2.setProblem(build_ex1223b(env2))

        callback_fired = [False]

        def inject_aux_point(ctx):
            callback_fired[0] = True
            print(f"  [OnlyExternal] callback fired, injecting aux-problem interior point")
            ctx.setInteriorPoints([interior_point])

        solver2.registerCallback(SHOTpy.CallbackLocation.InteriorPointSearch, inject_aux_point)
        solver2.solveProblem()

        assert callback_fired[0], "Phase 2: ESH interior point callback was not fired"
        assert solver2.getPrimalSolutions(), (
            "Phase 2: no primal solution found when using aux-problem interior point"
        )
        phase2_obj = solver2.getPrimalSolution().objValue
        print(f"  Phase 2 objective: {phase2_obj:.6f}  (expected ≈ 4.5796)")
        assert abs(phase2_obj - 4.579582) < 0.01, (
            f"Phase 2 objective {phase2_obj:.6f} differs from known optimum 4.5796"
        )
