# Changelog

User-visible changes to SHOT and SHOTpy. See [CONTRIBUTING.md](CONTRIBUTING.md)
for how to add entries. Changes in versions before 2.0 are described in the
[GitHub releases](https://github.com/coin-or/SHOT/releases).

## Unreleased

Changes since SHOT 1.1.

### Breaking changes

- macOS ARM64 wheels and standalone packages require macOS 15 or newer.
  The previous deployment target, macOS 14, did not match the bundled
  Fortran runtimes ([#308]).
- Gurobi builds require version 10.0 or newer ([#299]).
- Settings are updated with `updateSetting("Category.Name", value)` instead
  of `updateSetting(name, category, value)`, and a priority decides which
  source wins when a setting is given more than once ([#187]).
- Several settings were renamed or given new types or bounds, so options
  files using the old names or values give an error ([#204], [#229]); the
  removed settings are listed under Removed. Renamed settings:
  - `Model.Reformulation.ObjectiveFunction.Epigraph.Use` is replaced by
    `Model.Reformulation.ObjectiveFunction.EpigraphStrategy`.
  - `Model.Reformulation.Quadratics.EigenValueDecomposition.Use` and `.Method`
    are replaced by `Model.Reformulation.Quadratics.Decomposition.Method`, and
    `.Formulation` by `Model.Reformulation.Quadratics.Decomposition.Formulation`
    ([#183]).
  - `Dual.ESH.Rootsearch.UniqueConstraints` is now
    `Dual.HyperplaneCuts.UniqueConstraints`.
- The per-event callbacks are replaced by one generic callback, registered
  for a set of locations in the solution process; see
  [docs/Callbacks.md](docs/Callbacks.md) ([#206], [#224]).
- The C++ headers are installed in `include/SHOT` instead of directly in
  `include`, so that headers such as `Solver.h` and `Settings.h` no longer
  clash with other packages. Projects using the CMake target
  `SHOT::SHOTSolver` need no change; include headers as `<SHOT/Solver.h>` or,
  as before, `"Solver.h"`. Projects that do not use the CMake package must add
  `<prefix>/include/SHOT` to their include path ([#241], [#250]).
- C++17 is required ([#217]).
- The results and problems of an environment are only available while its
  solver exists: a solver frees them when it is destroyed, so that the
  environment is freed. A problem used afterwards keeps the environment with
  its settings and output. The environment was never freed, since its
  components refer back to it, so creating many solvers, e.g., in a loop in
  SHOTpy, used about 0.3 MB more memory per solver ([#258]).

### Added

- Standalone executable archives for Linux x86_64 and ARM64, macOS ARM64,
  and Windows x64, with HiGHS, Cbc, Ipopt, solver licenses and SHA-256
  checksums. Python and pip are not required. The archives are extracted
  from repaired wheels, tested on fresh runners, and attached to tagged
  GitHub releases; manual builds provide them as Actions artifacts ([#308]).
- `Primal.FixedInteger.MaxStartsPerCall` limits the number of dual-pool
  starting points used per NLP call to five by default; zero allows all starts
  ([#306]).
- MIP backend capability queries for convex/nonconvex quadratic objectives and
  constraints, SOS1/SOS2, semicontinuous and semi-integer variables, SHOT's
  single-tree integration, and finite variable-bound defaults, available
  without constructing a solver. SHOTpy exposes the records through
  `getMIPSolverCapabilities()` with read-only properties ([#299]).
- A [MIP solver capabilities guide](docs/MIPSolverCapabilities.md) and
  [C++ and SHOTpy query examples](docs/site/concepts.md#solver-backends) in the
  public documentation, with generated API references ([#299]).
- Guides to interpreting terminal output, configuring solves, and understanding
  modeling and reformulation, with expanded result handling and C++ examples
  ([#295]).
- HiGHS as MIP solver ([#182], [#191], [#194], [4f096597]).
- Uno ([#200]) and CONOPT ([#225]) as NLP solvers for the fixed-integer
  primal heuristic.
- SHOTpy, a Python interface for building, solving and inspecting problems,
  including SOS and semicontinuous variables; it requires Python 3.11
  ([#185], [#192], [#208]).
- Wheels for Linux (x86_64 and arm64), macOS and Windows with Cbc, Ipopt and
  HiGHS, which also install the `SHOT` executable for use from AMPL, Pyomo
  and JuMP ([#221], [#228]).
- Evaluation of gradients, Hessians, interval bounds and interval
  derivatives from C++ and Python ([#216]).
- Native Windows builds with MSVC ([#221]) and an installable CMake package
  with the target `SHOT::SHOTSolver` ([#214]).
- LDLᵀ reformulation of convex quadratic expressions ([#183], [f4a5daaa]).
- Reformulation of `a*x1*x2 >= c` with positive variables as convex
  constraints (`Model.Reformulation.Bilinear.InverseFormulation`) ([#226]).
- Supporting hyperplanes in feasible fixed-integer NLP solutions
  (`Primal.FixedInteger.CreateSupportingHyperplanes`, on by default) ([#234]).
- Fixed-integer NLP problems are also solved during the search for nonconvex
  continuous problems ([#235]).
- Final polishing of the solution with NLP solves
  (`Primal.PolishSolution.NumberOfPoints`, on by default) ([#204]).
- Exit codes in the SHOT executable ([#207]).
- Starting points from GAMS ([#190]) and AMPL ([1b054e3e]), the
  `shot_options` environment variable in AMPL mode ([#227]), and the `tan`
  function in GAMS ([#189]).
- `Solver::setStartingPoint()` and `getStartingPoint()` in C++ and SHOTpy to
  give a starting point after `setProblem()`, as AMPL and GAMS can. A feasible
  point is the first primal solution and the MIP start of the dual problems
  ([#246]). With `Primal.FixedInteger.UseStartingPoint` (on by default), an
  NLP problem with the discrete variables fixed is solved from a starting
  point that is not feasible, which may repair it ([#261], [#262]).
- A `BoundTightening` callback location, called in `setProblem()` after the
  bound tightening of the original and of the reformulated problem, where
  bounds from other techniques can tighten the variable bounds; with
  `Model.BoundTightening.FeasibilityBased.Use` off, only these are used.
  `Model.BoundTightening.FeasibilityBased.UseAfterCallback` propagates them
  with another bound tightening pass ([#247]).
- `setFunctionConvexity()` on quadratic and nonlinear constraints and
  objective functions in C++ and SHOTpy, and the `functionConvexity`
  argument of SHOTpy's `Problem.addConstraint()` and `Problem.setObjective()`,
  specify the convexity of a function, which is then used instead of
  detected. The solution report, the OSrL results and
  `getUserSpecifiedConvexityDescription()` tell when global optimality
  depends on such functions or on `Model.Convexity.AssumeConvex` ([#256]).
- The AMPL solve message gives the SHOT version, the problem types and the
  subsolvers; the status text is unchanged ([d9db1155]).
- The options report shows the number of MIP solver threads, and a warning
  is given when several threads are requested but Cbc has no thread support
  ([66efbd6d]).
- A warning when a primal bound passes a global dual bound ([#236]).
- Clear errors for problems without variables or objective ([5a82942f]).
- MPS files in the debug output for Cbc, CPLEX, Gurobi and HiGHS
  ([64b10f44]).
- Many new tests: C++ tests of the model, settings, dual bounds and
  determinism, a test for each MIP and NLP solver, instance tests including
  the MINLPTests.jl problems, and Python tests of SHOTpy ([#192], [#197],
  [#204]).
- Documentation of the command-line usage, compilation, callbacks and
  debugging of SHOT, a SHOTpy tutorial, and a C++ and Python API reference
  on the website ([06f753cd], [#206], [#220], [#240]).
- Contribution guidelines (`CONTRIBUTING.md`) and this changelog ([#240]),
  and `AGENTS.md` with instructions for AI coding agents ([06f753cd]).
- The settings reference lists the settings of all MIP and NLP solvers and of
  GAMS with the solver each requires, also when SHOT is built without them,
  and the recommended values for nonconvex problems ([#242]).
- In the multi-tree strategy, the discrete values of the previous MIP
  solution are given to the MIP solver as a partial start, which it completes
  by solving the LP with these values fixed. This is on for convex problems
  (setting `Dual.MIP.UseDualSolutionStart`), where it mainly helps HiGHS and
  Cbc find primal solutions ([#248]).

### Changed

- Nonconvex continuous problems use the best dual solution as the default NLP
  starting point, rather than every point in the solution pool. Explicit
  `Primal.FixedInteger.Source` selections still take precedence ([#306]).
- NLP progress rows show the outer SHOT iteration, with separate NLP solve
  and batch-start counters. `NLPPOOL` replaces `NLPUNFEA` to distinguish a
  starting point that violates nonlinear constraints from an infeasible NLP
  ([#306]).
- Equality constraints and ranges are kept as they are in the original
  problem ([#210]).
- Much faster reading, reformulating and solving of large problems, and
  monomial and signomial terms with the same factors are merged ([#204],
  [#212], [#237]).
- Repeated runs with the same settings give the same results, except when
  limited by time ([#204]).
- The cutoff is given to HiGHS and Cbc as a solver parameter, as for CPLEX
  and Gurobi ([ec55766b]).
- Dual stagnation is again checked as a termination criterion ([cf408fbd]).
- The MIP start includes the values of the reformulation variables
  ([8c4a2bb8]).
- Improved primal reduction cut heuristic ([#178]).
- Updated bundled third-party libraries ([#213]).
- MC++ is pinned to 5.1.0 and the optional Uno compatibility target is
  2.9.3 ([#304]).
- New repository layout, and the documentation is on the website ([#220],
  [#240]).
- Settings of a MIP or NLP solver or of GAMS that SHOT is not compiled with
  are accepted instead of giving an error, so that the same options files
  work with all builds; a warning tells that they are ignored if they are
  changed from their defaults ([#242]).
- The C++ API documents, as the Python API, that
  `Solver::getCurrentDualBound()` is not a valid bound for a nonconvex
  problem once cuts have been added to nonconvex functions, and that
  `Solver::getGlobalDualBound()` is.
- `Model.Reformulation.Bilinear.IntegerFormulation.MaxDomain` is 250 instead
  of 100, so that integer variables with domains such as 0..200 are
  discretized in bilinear terms. On the MINLPLib instances where this
  matters (nvs01–nvs24, primary), HiGHS, Cbc and CPLEX solve more of them,
  e.g., nvs02 and nvs14 within seconds instead of not at all in 60 s. The
  limit now also applies to bilinear terms that are extracted to auxiliary
  variables, e.g., with `Model.Reformulation.Quadratics.ExtractStrategy=2`,
  whose integer variables were discretized regardless of their domain
  ([#282]).
- SHOT tells how many bilinear terms with integer variables are not
  discretized because of their bounds or the limit, and warns when bilinear
  terms get nonconvex auxiliary constraints although
  `Model.Convexity.AssumeConvex` is true, since the solution may then not be
  optimal ([#282]).

### Fixed

- Python wheels preserve external solver license notices on Linux and
  macOS as well as Windows ([#308]).
- SHOT builds with `CMAKE_BUILD_TYPE=RelWithDebInfo`; configuring the bundled
  CppAD failed for this build type ([#307]).
- Floating-point checks explicitly use `std::abs`. Depending on the included
  headers, GCC could use integer `abs` and treat, e.g., a power 1.5 as linear
  and 2.5 as quadratic ([#307]).
- The convexity of the tangent of an interval containing a pole is unknown;
  the interval error was not caught ([#307]).
- The Gurobi interface passes empty arrays without accessing their first
  element when adding special ordered sets or repairing infeasibility ([#307]).
- HiGHS model-loading messages no longer bypass SHOT's console output settings,
  including when creating internal feasibility and repair models ([#303]).
- HiGHS distinguishes unbounded from infeasible MILPs by checking integer
  feasibility with presolve enabled, avoiding a numerical status for unbounded
  integer domains ([#301]).
- Convex quadratic objectives are retained when supported by the MIP backend
  even when other constraints make the problem nonconvex ([#301]).
- Removing a `HyperplaneSelection` callback after `setProblem()` restores
  direct MILP solving when no cut callback remains ([#301]).
- Nonconvex quadratic constraints retain their quadratic terms when the MIP
  backend and quadratic strategy support them, including Gurobi's recommended
  strategy for binary quadratic models ([#301]).
- Supported quadratic parts are retained by default in objectives and
  constraints containing higher-degree terms when the MIP backend supports
  them. Explicit partitioning and integer-bilinear reformulation settings
  still take precedence ([#302]).
- Complete MILP reformulations are solved directly by the MIP backend to its
  optimality tolerances, without SHOT's single-tree callbacks or initial LP
  relaxation. Models with a `HyperplaneSelection` callback retain the tree
  strategy so that user cuts are applied ([#301]).
- SOS-only models with continuous columns use the discrete strategy with Cbc;
  HiGHS rejects unsupported SOS sets instead of omitting them. Unsupported
  negative semicontinuous and semi-integer branches are rejected by the CPLEX
  and HiGHS adapters before model construction ([#299]).
- Retained nonconvex quadratic objectives with linear constraints can use
  Gurobi's direct strategy under the objective-enabled reformulation modes too
  ([#299]).
- `hasMIPSolver()` reports HiGHS availability consistently with the supported
  solver list ([#299]).
- Compatibility settings for an unavailable MIP solver's fallback are applied
  before reformulation, including when the unavailable solver was explicitly
  selected. A failed MIP model construction stops initialization ([#299]).
- Convex maximization QCQP and MIQCQP models retain their quadratic constraints
  with CPLEX and use the direct quadratic strategy ([#298]).
- CPLEX uses the direct quadratic strategy for nonconvex QP and MIQP models
  with only linear constraints when quadratic objectives are enabled ([#298]).
- Nonconvex QP, MIQP, QCQP and MIQCQP models use the direct quadratic strategy
  with Gurobi when `Model.Reformulation.Quadratics.Strategy=3`,
  including when selected by recommended settings, instead of falling back to
  the NLP or multi-tree strategy ([#298]).
- Integer bilinear discretization stores and evaluates its auxiliary binary values
  in linear space and time instead of building a quadratic number of polynomial
  factors. Fractional points use bounded interpolation between adjacent values
  ([#297]).
- Repeated Ipopt NLP solves reuse initialized solver state after completed solves,
  including iteration/time limits and infeasibility, instead of always rebuilding
  it. Errors force a fresh initialization for the next solve ([#296]).
- The iteration table labels MIQCQP subproblems correctly instead of `MIQPCP`
  ([#295]).
- Ipopt NLP solves stop when iteration values become NaN or infinite, avoiding
  a numerical stall in the linear solver that could exceed the time limit.
  The failed solve is reported as an error without retaining its solution
  ([#294]).
- Antiepigraph reformulation reconstructs the objective variable correctly for
  scaled and offset objectives and normalizes maximization objectives consistently.
  Each MIP solution uses its own objective value to reconstruct the variable
  ([#293]).
- Antiepigraph reformulation preserves the defining constraint when its direction,
  ranged bounds or objective-variable domain make the reformulation unsafe,
  preventing incorrect global optimality claims and lost variable restrictions
  ([#292]).
- A GAMS NLP solver licence failure produces one warning and switches to
  GAMS IPOPT without reusing solver-specific options. Failed solvers are
  skipped for the rest of the problem; if IPOPT also has a licence failure,
  further GAMS NLP calls are skipped ([#291]).
- Single-tree integer-cut regression tests for Gurobi and CPLEX trigger
  cuts deterministically, avoiding failures caused by solver search behavior
  ([#290]).
- Binary and integer bilinear products stay in their original expressions
  when linearization would require big-M values of 1e15 or larger. A warning
  reports the skipped linearizations. This avoids rejected or poorly scaled
  MIP models without introducing nonconvex auxiliary equalities ([#289]).
- HiGHS recovery attempts share the remaining solve time, with additional
  time checks in interrupt callbacks. Solves without presolve stop with a
  numerical status when large integer domains risk overflow in HiGHS's
  reduced-cost probing, avoiding a loop that ignores time limits ([#289]).
- Gurobi, CPLEX and Cbc retry apparently infeasible dual problems with more
  robust numerical settings, avoiding false infeasibility from badly scaled cuts.
  Artificial variable bounds are removed before accepting infeasibility of
  an unrepaired global relaxation, also without an incumbent or cutoff.
  If such bounds remain, SHOT stops without claiming infeasibility ([#288]).
- Gurobi accepts nonconvex quadratic objectives when recommended settings
  are disabled, including with quadratic strategies 1 and 2, instead of
  stopping with an "Objective Q not PSD" error ([#287]).
- Large negative lower bounds and positive upper bounds of auxiliary
  variables are treated as infinite in MIP models (threshold 1e15), with
  fixed values preserved. Oversized square and product auxiliaries use
  continuous columns, and very loose interval objective bounds are not used
  as cutoffs. This prevents incorrect optimality claims from poorly scaled
  dual problems while preserving original variable bounds ([#286]).
- Integer cuts after a feasible fixed-integer NLP solve of a convex
  problem are added only when the NLP status is optimal and its point is
  accepted as a primal solution. Rejected or merely feasible points no
  longer exclude an integer combination that may contain a better solution
  and lead to a false optimality claim ([#285]).
- Primal candidates from MIP solution pools and callbacks are checked
  against the original linear constraints even when
  `Primal.Tolerance.TrustLinearConstraintValues` is enabled. Numerically
  infeasible points, e.g., after bounding an unbounded dual objective, can
  no longer become the incumbent and produce a false optimality claim
  ([#284]).
- Many fixes for wrong results, invalid dual bounds, crashes and
  non-termination, mainly on nonconvex problems, found by running the
  MINLPLib and MINLPTests instances with several MIP and NLP solvers
  ([#153], [#171], [#204], [#209], [#211], [#215], [#222], [#231], [#239]).
- Fixes in the GAMS and AMPL interfaces ([#150], [#162], [#181], [#196],
  [#198], [#227], [46404779], [801c7809]) and in the Gurobi, Cbc, HiGHS and
  Ipopt interfaces ([#151], [#155], [#170], [#172], [#218], [#232],
  [15695f16], [7f2b2af7], [bca3b691]).
- CPLEX now uses the MIP start in the multi-tree strategy; it was deleted
  whenever cuts were added to the model ([#245]).
- The times for solving relaxed and MIP problems in the time report are the
  times spent in the MIP solver; they measured the whole LP step and the time
  from the end of the LP step to the end of the run, and the time for MIP
  problems was zero without an LP step ([46affef9], [a49ecf75]).
- The bound tightening, the initial outer approximation and the creation of
  the primal NLP solvers, performed when the problem is set, stop at the time
  limit, so that the solution time runs less over the limit when reading a
  large problem uses up most of it ([#251]).
- The OSrL results name HiGHS as the dual solver, where the name was empty,
  and give the NLP solver used for the fixed-integer NLP problems as
  `FixedNLPSolver`, which gave the dual solver ([c95eba1d], [#252]). The
  trace file gives the NLP solver only if an NLP problem was solved, and the
  GAMS NLP solver called instead of the setting, e.g., `CONOPT` instead of
  `auto` ([#252]).
- With `Dual.HyperplaneCuts.UseIntegerCuts`, the default for nonconvex
  problems, an integer combination for which the fixed-integer NLP problem is
  infeasible is also excluded with an integer cut. SHOT could otherwise stop
  without a primal solution when the MIP solver returned the same combination
  again ([#257]).
- The ESH method adds a cut for a violated convex constraint also when the
  root search ends on the boundary, where the constraint value can be rounded
  to slightly below zero. Convex problems, e.g., with quadratic terms and
  Cbc or HiGHS, could otherwise stop with a gap since no additional cuts
  could be added ([#255]).
- A MIP problem that HiGHS reports as infeasible is solved again without
  presolve, since HiGHS can reject every solution of a feasible problem when
  they violate a row after the presolve is undone. The dual problem could
  then be taken as infeasible, and the solution stop without a primal
  solution ([#255]).
- With Gurobi, the ESH interior point search finds a point also when Gurobi's
  presolve reports the minimax problem as infeasible or unbounded, which it
  can do because of the large lower bound of the objective variable. The
  problem is solved again without dual reductions, as are the other problems
  solved before the main iterations; the search could otherwise end without
  an interior point ([#259]).
- A dual problem that is infeasible because of the cutoff gives the cutoff as
  dual bound also when only the solution limit was updated since the cutoff
  was set. The solution could otherwise stop with a gap, e.g., on a convex
  problem, since the dual problem was taken as infeasible ([#263]).
- A primal solution that fulfils the constraints to
  `Termination.ConstraintTolerance` is reported instead of an incumbent that
  only fulfils them to the larger primal tolerances, if its objective value is
  within the termination gap tolerances. A point from a relaxation could
  otherwise be reported with an objective value better than that of any
  feasible solution when the objective is sensitive to the constraint
  violation ([#265]).
- With Gurobi in the single-tree strategy, SHOT no longer crashes when an
  integer cut of nonbinary variables has been added, which adds variables
  to the MIP problem. The callback read the values of the new variables
  outside the variables it got before the first solve.
- The cuts of the initial polyhedral outer approximation
  (`Model.BoundTightening.InitialPOA.Use`) are scaled by their largest
  coefficient, as the other cuts. A cut generated close to a singularity,
  e.g., of `x^2/y` for `y` close to zero, could have coefficients so large
  that Gurobi returned a solution of the dual problem that was not optimal,
  and SHOT took a suboptimal solution as globally optimal.
- With `Dual.MIP.UpdateObjectiveBounds`, the bounds of the objective
  variable are updated in the sense of the dual problem, and only tightened.
  A maximization problem was reported as infeasible, since the bounds of the
  maximized objective were given to the minimized objective variable, and
  the bounds were loosened when there was no primal or dual bound yet.
- When selecting cuts for nonconvex constraints, the check whether a cut
  would cut away a primal solution read the values of auxiliary variables
  from outside the primal solution, which only has the variables of the
  original problem. It now uses the values of the auxiliary variables, so
  that cuts are not postponed, or added, because of arbitrary values.
- The largest error of the linear constraints of a primal solution of a
  problem without linear constraints was undefined, and could be reported in
  the output and the OSrL results and used as the largest constraint error of
  the solution.
- The values of the discrete variables of a fixed-integer NLP candidate are
  rounded rather than truncated when checking whether the integer combination
  has been tested before. A value just below an integer was taken as the
  integer below, so the same combination could be solved again, or a new one
  skipped; values outside the range of an `int` were undefined.
- The lazy cuts of the Gurobi and Cplex single-tree strategy are scaled by
  their largest coefficient, as the other cuts. They were only scaled when
  their RHS was larger than 1e15, so a cut generated close to a singularity,
  e.g., of `x^2/y` for `y` close to zero, could be given to the MIP solver
  with coefficients of the order 1e20.
- With Gurobi, a continuous convex problem with quadratic constraints that
  Gurobi solves to optimality gets its objective value as dual bound. Gurobi
  reports an infinite `ObjBound` for such a problem, so the dual bound was
  missing and the solution could not be proven optimal.
- When Gurobi stops with numerical trouble, the dual problem is solved again
  with `NumericFocus` 3, unless `Subsolver.Gurobi.NumericFocus` is already 3.
  SHOT otherwise stopped, e.g., on a degenerate convex problem with
  quadratic constraints.
- Numerical trouble in the relaxation of a problem with discrete variables
  (`Dual.Relaxation.Use`) ends the relaxation instead of the solution, so the
  problem with the discrete variables is still solved.
- The values of the solutions of the MIP solver are projected onto the
  variable bounds before they are used. A value slightly outside a bound,
  within the feasibility tolerance of the MIP solver, could be outside the
  domain of a nonlinear function, e.g., `y < 0` in `x^2/y` with `y >= 0`, so
  that the constraint was fulfilled by a large margin. With Cplex, an
  infeasible point was then accepted as the optimal solution, and no cut
  was generated for the constraint.
- The OSrL result file has the values of the variables, the objective and
  the constraints with all significant digits. They were written with six
  decimals, so that, e.g., a value of `1.5e-7` was written as `0.000000`.
- Fixes in the CPLEX interface ([#280]):
  - The cutoff constraint of a minimization problem with a constant in the
    objective function lost the constant when the cutoff was updated, so
    that it cut off solutions better than the cutoff and SHOT could report a
    wrong optimum.
  - The cuts are found by their position among all constraints, also when the
    problem has quadratic constraints. Their positions were counted among the
    linear constraints only, so that repairing or removing cuts changed the
    bounds of other constraints, e.g., removed a quadratic constraint.
  - In the single-tree strategy, the callback uses only the variables of the
    problem after integer cuts of nonbinary variables have added variables
    to the MIP problem, and gives CPLEX primal solutions for these variables
    only. SHOT crashed in this case.
  - When CPLEX fails, e.g., because of numerical trouble, the dual problem is
    solved again with numerical emphasis, and the solutions are used if it
    fails again. SHOT stopped with an error.
  - A dual problem that CPLEX reports as infeasible before a primal solution
    is found is solved again without presolve, since CPLEX's presolve can
    report a feasible problem with quadratic constraints as infeasible.
- SHOT crashed when reading a problem with several monomial or signomial
  terms in the objective function with
  `Model.Reformulation.ObjectiveFunction.EpigraphStrategy=2` and
  `Model.Reformulation.Constraint.PartitionNonlinearTerms=0`: the bounds of
  the terms of the epigraph constraint were calculated before the constraint
  belonged to the reformulated problem ([#281]).
- An integer variable without bounds, or with the artificial bounds from
  `Model.Variables.Integer.MinimumLowerBound` and `MaximumUpperBound`, is no
  longer discretized in a bilinear term that is extracted to an auxiliary
  variable, e.g., with `Model.Reformulation.Quadratics.ExtractStrategy=2`;
  the term is reformulated as a continuous bilinear term instead. The
  discretization has one binary variable for each value of the variable, so
  SHOT used up all memory ([#282]).
- The interval bounds of `cos`, `sin` and `tan` are correct for arguments
  with bounds larger than about 6.7e9, e.g. a variable without a lower or
  upper bound. MC++ converted the number of periods to an `int`, which
  overflowed and gave wrong intervals, so that bound tightening could cut
  off the optimum and SHOT reported a wrong optimal solution. On x86-64 this
  happened already for, e.g., a variable in [-1e20, 1], on arm64 for bounds
  such as [-1e12, -1e11] ([#283]).

### Removed

- The unused C++ `IMIPSolver::supportsQuadraticObjective()` and
  `supportsQuadraticConstraints()` methods. Use the backend capability records
  or `getMIPSolverCapabilities()` instead ([#299]).
- The legacy Cplex single-tree callback, deprecated by Cplex, and the setting
  `Subsolver.Cplex.UseGenericCallback`; the generic Cplex callback is always
  used ([#233]).
- The per-event callback API (`E_EventType`), replaced by the generic
  callback ([#206]).
- The unfinished presolve in the MIP solver interfaces, with the settings
  `Dual.MIP.Presolve.Frequency`, `Dual.MIP.Presolve.RemoveRedundantConstraints`,
  `Dual.MIP.Presolve.UpdateObtainedBounds` and
  `Primal.FixedInteger.UsePresolveBounds` ([bf3f7447]).
- The settings `Subsolver.Gurobi.PoolSolutions` (use
  `Dual.MIP.SolutionPool.Capacity`), `Subsolver.Ipopt.MaxIterations` (use
  `Primal.FixedInteger.IterationLimit`) and
  `Subsolver.Rootsearch.ActiveConstraintTolerance` ([#229]), and
  `Dual.HyperplaneCuts.SaveHyperplanePoints` ([#204]).
- The Optimization Services modeling system; reading OSiL files is not
  affected ([#204]).
- The unused C++ enums `E_ObjectiveFunctionType`, `E_ProblemType` and
  `ES_RootsearchConstraintStrategy` ([#238]).

[#150]: https://github.com/coin-or/SHOT/pull/150
[#151]: https://github.com/coin-or/SHOT/pull/151
[#153]: https://github.com/coin-or/SHOT/pull/153
[#155]: https://github.com/coin-or/SHOT/pull/155
[#162]: https://github.com/coin-or/SHOT/pull/162
[#170]: https://github.com/coin-or/SHOT/pull/170
[#171]: https://github.com/coin-or/SHOT/pull/171
[#172]: https://github.com/coin-or/SHOT/pull/172
[#178]: https://github.com/coin-or/SHOT/pull/178
[#181]: https://github.com/coin-or/SHOT/pull/181
[#182]: https://github.com/coin-or/SHOT/pull/182
[#183]: https://github.com/coin-or/SHOT/pull/183
[#185]: https://github.com/coin-or/SHOT/pull/185
[#187]: https://github.com/coin-or/SHOT/pull/187
[#189]: https://github.com/coin-or/SHOT/pull/189
[#190]: https://github.com/coin-or/SHOT/pull/190
[#191]: https://github.com/coin-or/SHOT/pull/191
[#192]: https://github.com/coin-or/SHOT/pull/192
[#194]: https://github.com/coin-or/SHOT/pull/194
[#196]: https://github.com/coin-or/SHOT/pull/196
[#197]: https://github.com/coin-or/SHOT/pull/197
[#198]: https://github.com/coin-or/SHOT/pull/198
[#200]: https://github.com/coin-or/SHOT/pull/200
[#204]: https://github.com/coin-or/SHOT/pull/204
[#206]: https://github.com/coin-or/SHOT/pull/206
[#207]: https://github.com/coin-or/SHOT/pull/207
[#208]: https://github.com/coin-or/SHOT/pull/208
[#209]: https://github.com/coin-or/SHOT/pull/209
[#210]: https://github.com/coin-or/SHOT/pull/210
[#211]: https://github.com/coin-or/SHOT/pull/211
[#212]: https://github.com/coin-or/SHOT/pull/212
[#213]: https://github.com/coin-or/SHOT/pull/213
[#214]: https://github.com/coin-or/SHOT/pull/214
[#215]: https://github.com/coin-or/SHOT/pull/215
[#216]: https://github.com/coin-or/SHOT/pull/216
[#217]: https://github.com/coin-or/SHOT/pull/217
[#218]: https://github.com/coin-or/SHOT/pull/218
[#220]: https://github.com/coin-or/SHOT/pull/220
[#221]: https://github.com/coin-or/SHOT/pull/221
[#222]: https://github.com/coin-or/SHOT/pull/222
[#224]: https://github.com/coin-or/SHOT/pull/224
[#225]: https://github.com/coin-or/SHOT/pull/225
[#226]: https://github.com/coin-or/SHOT/pull/226
[#227]: https://github.com/coin-or/SHOT/pull/227
[#228]: https://github.com/coin-or/SHOT/pull/228
[#229]: https://github.com/coin-or/SHOT/pull/229
[#231]: https://github.com/coin-or/SHOT/pull/231
[#232]: https://github.com/coin-or/SHOT/pull/232
[#233]: https://github.com/coin-or/SHOT/pull/233
[#234]: https://github.com/coin-or/SHOT/pull/234
[#235]: https://github.com/coin-or/SHOT/pull/235
[#236]: https://github.com/coin-or/SHOT/pull/236
[#237]: https://github.com/coin-or/SHOT/pull/237
[#238]: https://github.com/coin-or/SHOT/pull/238
[#239]: https://github.com/coin-or/SHOT/pull/239
[#240]: https://github.com/coin-or/SHOT/pull/240
[#241]: https://github.com/coin-or/SHOT/pull/241
[#242]: https://github.com/coin-or/SHOT/pull/242
[#245]: https://github.com/coin-or/SHOT/pull/245
[#246]: https://github.com/coin-or/SHOT/pull/246
[#247]: https://github.com/coin-or/SHOT/pull/247
[#248]: https://github.com/coin-or/SHOT/pull/248
[#250]: https://github.com/coin-or/SHOT/pull/250
[#251]: https://github.com/coin-or/SHOT/pull/251
[#252]: https://github.com/coin-or/SHOT/pull/252
[#255]: https://github.com/coin-or/SHOT/pull/255
[#256]: https://github.com/coin-or/SHOT/pull/256
[#257]: https://github.com/coin-or/SHOT/pull/257
[#258]: https://github.com/coin-or/SHOT/pull/258
[#259]: https://github.com/coin-or/SHOT/pull/259
[#261]: https://github.com/coin-or/SHOT/pull/261
[#262]: https://github.com/coin-or/SHOT/pull/262
[#263]: https://github.com/coin-or/SHOT/pull/263
[#265]: https://github.com/coin-or/SHOT/pull/265
[#280]: https://github.com/coin-or/SHOT/pull/280
[#281]: https://github.com/coin-or/SHOT/pull/281
[#282]: https://github.com/coin-or/SHOT/pull/282
[#283]: https://github.com/coin-or/SHOT/pull/283
[#284]: https://github.com/coin-or/SHOT/pull/284
[#285]: https://github.com/coin-or/SHOT/pull/285
[#286]: https://github.com/coin-or/SHOT/pull/286
[#287]: https://github.com/coin-or/SHOT/pull/287
[#288]: https://github.com/coin-or/SHOT/pull/288
[#289]: https://github.com/coin-or/SHOT/pull/289
[#290]: https://github.com/coin-or/SHOT/pull/290
[#291]: https://github.com/coin-or/SHOT/pull/291
[#292]: https://github.com/coin-or/SHOT/pull/292
[#293]: https://github.com/coin-or/SHOT/pull/293
[#294]: https://github.com/coin-or/SHOT/pull/294
[#295]: https://github.com/coin-or/SHOT/pull/295
[#296]: https://github.com/coin-or/SHOT/pull/296
[#297]: https://github.com/coin-or/SHOT/pull/297
[#298]: https://github.com/coin-or/SHOT/pull/298
[#299]: https://github.com/coin-or/SHOT/pull/299
[#301]: https://github.com/coin-or/SHOT/pull/301
[#302]: https://github.com/coin-or/SHOT/pull/302
[#304]: https://github.com/coin-or/SHOT/pull/304
[#303]: https://github.com/coin-or/SHOT/pull/303
[#306]: https://github.com/coin-or/SHOT/pull/306
[#307]: https://github.com/coin-or/SHOT/pull/307
[#308]: https://github.com/coin-or/SHOT/pull/308
[06f753cd]: https://github.com/coin-or/SHOT/commit/06f753cd
[15695f16]: https://github.com/coin-or/SHOT/commit/15695f16
[1b054e3e]: https://github.com/coin-or/SHOT/commit/1b054e3e
[46404779]: https://github.com/coin-or/SHOT/commit/46404779
[46affef9]: https://github.com/coin-or/SHOT/commit/46affef9
[4f096597]: https://github.com/coin-or/SHOT/commit/4f096597
[5a82942f]: https://github.com/coin-or/SHOT/commit/5a82942f
[64b10f44]: https://github.com/coin-or/SHOT/commit/64b10f44
[66efbd6d]: https://github.com/coin-or/SHOT/commit/66efbd6d
[7f2b2af7]: https://github.com/coin-or/SHOT/commit/7f2b2af7
[801c7809]: https://github.com/coin-or/SHOT/commit/801c7809
[8c4a2bb8]: https://github.com/coin-or/SHOT/commit/8c4a2bb8
[a49ecf75]: https://github.com/coin-or/SHOT/commit/a49ecf75
[bca3b691]: https://github.com/coin-or/SHOT/commit/bca3b691
[bf3f7447]: https://github.com/coin-or/SHOT/commit/bf3f7447
[c95eba1d]: https://github.com/coin-or/SHOT/commit/c95eba1d
[cf408fbd]: https://github.com/coin-or/SHOT/commit/cf408fbd
[d9db1155]: https://github.com/coin-or/SHOT/commit/d9db1155
[ec55766b]: https://github.com/coin-or/SHOT/commit/ec55766b
[f4a5daaa]: https://github.com/coin-or/SHOT/commit/f4a5daaa
