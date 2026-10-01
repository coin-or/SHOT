# Callbacks

SHOT calls user callbacks at a number of locations in the solution process. A callback is a single function
`void(CallbackContext&)` in C++, or `fn(ctx)` in Python, registered for one or several locations. The context tells
the callback where it is called, holds the state of the solver, and has the actions available at that location.

## Registering a callback

C++:

```cpp
#include "Solver.h"

auto solver = std::make_unique<SHOT::Solver>();

// One callback for two locations
int handle = solver->registerCallback(
    E_CallbackLocation::PrimalCandidateCheck | E_CallbackLocation::TerminationCheck,
    [](CallbackContext& context)
    {
        if(auto candidate = context.as<PrimalCandidateCheckContext>(); candidate && candidate->getObjectiveValue() > 5.0)
            candidate->rejectCandidate();

        if(context.getElapsedTime() > 60.0)
            context.terminate();
    });

// A callback for the location of a context class
solver->registerCallback<NewPrimalSolutionContext>(
    [](NewPrimalSolutionContext& solution) { std::cout << solution.getObjectiveValue() << '\n'; });

solver->removeCallback(handle);
```

Python:

```python
L = SHOTpy.CallbackLocation

def callback(ctx):
    if isinstance(ctx, SHOTpy.PrimalCandidateCheckContext) and ctx.objectiveValue > 5.0:
        ctx.rejectCandidate()
    if ctx.elapsedTime > 60.0:
        ctx.terminate()

handle = solver.registerCallback(L.PrimalCandidateCheck | L.TerminationCheck, callback)
solver.removeCallback(handle)
```

In Python, the locations can also be given as a list, e.g., `[L.PrimalCandidateCheck, L.TerminationCheck]`. The
callback receives the context class of the location, and its return value is ignored.

- Callbacks can be registered before or after `setProblem()`, but not while `solveProblem()` runs: registering or
  removing a callback then throws (`RuntimeError` in Python).
- Several callbacks can be registered for the same location. They are called in the order they were registered.

## Locations

| Location | Context class | Where it is called | Strategies | Points | Actions |
|---|---|---|---|---|---|
| `InteriorPointSearch` | `InteriorPointSearchContext` | `TaskFindInteriorPoint`, after the internal interior point search, or instead of it with `Dual.ESH.InteriorPoint.Strategy` = OnlyExternal | Multi-tree, single-tree, NLP; ESH cut strategy and nonlinear constraints only | `getInteriorPoints()`: reformulated | `setInteriorPoints(points)` |
| `DualBoundUpdate` | `DualBoundUpdateContext` | `TaskUpdateExternalDualBound`, before the first dual problem and once per iteration; the CPLEX and Gurobi single-tree callbacks | Multi-tree, single-tree, NLP | – | `setDualBound(value)` |
| `PrimalCandidateSearch` | `PrimalCandidateSearchContext` | `TaskSelectPrimalCandidatesFromExternalSource`, once per iteration and when the solution is finalized; the CPLEX and Gurobi single-tree callbacks | Multi-tree, single-tree, NLP | – | `addPrimalSolution(point)` |
| `PrimalCandidateCheck` | `PrimalCandidateCheckContext` | `PrimalSolver::checkPrimalSolutionCandidates`, for each candidate before it is checked | All | `getPoint()`: original | `rejectCandidate()` |
| `NewPrimalSolution` | `NewPrimalSolutionContext` | `Results::addPrimalSolution`, after a solution has been stored | All | `getPoint()`: original | – |
| `TerminationCheck` | `TerminationCheckContext` | `TaskCheckUserTermination`, once per iteration; the termination checks inside the MIP solvers (Cbc, CPLEX, Gurobi, HiGHS) and the single-tree callbacks | All | – | – |
| `HyperplaneSelection` | `HyperplaneSelectionContext` | `TaskSelectHyperplanesExternal`, after SHOT has selected its own hyperplanes; the single-tree callbacks | Multi-tree, single-tree, NLP | `getSolutionPoints()`: reformulated | `addHyperplane(hyperplane)` |

The points are in the variables of the original problem ("original") or of the reformulated problem
("reformulated"). The reformulated problem has the variables of the original problem first, followed by the auxiliary
variables of the reformulations. Use `getOriginalProblem()` and `getReformulatedProblem()` to evaluate functions in
them. The problems must not be modified.

## Information available everywhere

All contexts derive from `CallbackContext`, which has:

- `getLocation()`, `isMinimization()`, `getIterationNumber()`, `getElapsedTime()`
- `getDualBound()`: the current dual bound, which the termination criteria use
- `getGlobalDualBound()`: the dual bound valid for the whole problem
- `getPrimalBound()`, `getRelativeGap()`, `getAbsoluteGap()` (the gaps use the current dual bound)
- `getSolutionStatistics()`: a copy of the statistics
- `getOriginalProblem()`, `getReformulatedProblem()`
- `hasPrimalSolution()`, `getPrimalSolution()`: the best solution, in the variables of the original problem
- `isTerminationPending()`: termination was requested before SHOT reached the location, e.g., by a callback at
  another location, so SHOT is stopping; it stays true while the solution is finalized
- `isFinalizing()`: SHOT is finalizing the solution, for any termination reason; the primal strategies still run, so,
  e.g., `PrimalCandidateSearch`, `PrimalCandidateCheck` and `NewPrimalSolution` are still reached
- `isTerminationRequested()`: `terminate()` has been called in this context, by this or an earlier callback at the
  same location

In Python these are properties without the `get`, e.g., `ctx.dualBound`, and `ctx.primalSolution` is `None` when
there is no primal solution.

The values are a snapshot taken when SHOT reaches the location, so every callback at the same location sees the same
values. Values that are not available yet are infinite:

- The dual bound is `-inf` (minimization) or `+inf` (maximization) before the first dual problem has been solved.
- The primal bound is `+inf` (minimization) or `-inf` (maximization) before a primal solution has been found.
- The gaps are `+inf` while either bound is missing.
- The iteration number is 0 before the first iteration.

## Actions

The actions are recorded on the context and applied by SHOT after all callbacks at the location have returned. A
callback never sees what another callback did through the state of the solver, e.g., the dual bound after
`setDualBound()`.

| Action | Input | With several callbacks |
|---|---|---|
| `terminate()` (all contexts) | – | any |
| `rejectCandidate()` | – | any |
| `setDualBound(value)` | finite | the tightest value is proposed; SHOT uses it only if it is better than its own dual bound |
| `addPrimalSolution(point)` | the original or reformulated number of variables, all values finite | appended |
| `addHyperplane(hyperplane)` | as many indexes as coefficients, at least one; indexes of variables in the reformulated problem, or of the auxiliary objective variable of the dual problem; finite coefficients and right-hand side | appended |
| `setInteriorPoints(points)` | at least one point, each with the original or reformulated number of variables and finite values | the last call is used; not calling it keeps the current points |

An action with invalid input throws `std::invalid_argument` (`ValueError` in Python). If the callback does not catch
it, it is a callback failure (see below).

Actions and getters that don't belong to a location don't exist on its context class. In C++, `context.as<T>()`
returns `nullptr` if the callback is called at another location.

## Termination

`terminate()` requests SHOT to terminate. It takes effect at the next termination check point:

- `TaskCheckUserTermination` in the iteration of every strategy
- a check before the first dual problem is solved, for termination requested at `InteriorPointSearch` or at the first
  `DualBoundUpdate`
- the termination checks inside the MIP solvers, which interrupt them

The other actions of the callbacks that request termination are still applied. So, for example, adding a primal
solution and terminating in the same call works.

Until SHOT stops, callbacks at other locations are still called, e.g., `NewPrimalSolution` for the remaining candidates
of the iteration, the callbacks of other threads of the MIP solver, and the callbacks during finalization. They see
`isTerminationPending()` as true and can skip expensive work. `TerminationCheck` is not called once termination has
been requested.

A termination criterion that is checked earlier in the iteration takes precedence. If a gap, iteration limit or time
limit is met in the same iteration, that is the reported termination reason. `UserAbort` is reported only when the
request is what stops SHOT.

## Callback failures

If a callback throws, SHOT does the following:

1. It stores the exception and does not call the remaining callbacks at the location.
2. It discards the actions of all callbacks at the location, including a termination request.
3. It calls no callback again during the solve.
4. It terminates with reason `Error`.

`Solver::solveProblem()` then rethrows the exception after the solution has been finalized. In Python, the original
exception is raised by `solveProblem()`, with its traceback. The GAMS interface reports it as a solver error.

## Context lifetime

A context can only be used while the callback it was given to runs. After that, every getter and action throws
`CallbackContextExpired` (`SHOTpy.CallbackContextExpired`, a `RuntimeError`). The exceptions are `getLocation()` and
`isValid()`. Copy what you want to keep: the Python properties return copies.

## Threads

The callbacks are called one at a time, also when the MIP solver reaches a location from several threads (CPLEX
single-tree). In Python, `solveProblem()` releases the GIL, and the callback takes it, so a callback can be called from
a thread of the MIP solver. A callback must not call methods of the `Solver` that is solving.

## Adding a location

1. Add a bit to `E_CallbackLocation` in `src/Callback.h`, and extend `AllCallbackLocations` and
   `getCallbackLocationName()`.
2. Add a context class that derives from `CallbackContext`, with `static constexpr E_CallbackLocation Location`, getters
   for the data of the location, and actions that call `checkValid()` and validate their input. Override
   `discardActions()` and `releaseData()` if the class records actions or holds data.
3. Call it where SHOT reaches the location. Build the context only when `env->callbacks->isActive(location)` is true:
   ```cpp
   auto context = std::make_shared<XContext>(env, data);
   env->callbacks->invoke(*context);
   // apply the actions of the context
   context->invalidate();
   ```
4. Bind the enum value and the class in `src/SHOTpy.cpp`: `py::class_<XContext, CallbackContext, std::shared_ptr<XContext>>`,
   with getters that return copies.
5. Add a row to the table above, and tests in `test/SolverTest.cpp` and `test/python/test_callbacks.py`.

If the location is reached during `setProblem()`, release the GIL in the Python binding of `setProblem()` as well.
