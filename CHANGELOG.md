# Changelog

User-visible changes to SHOT and SHOTpy. See [CONTRIBUTING.md](CONTRIBUTING.md)
for how to add entries. Changes in versions before 2.0 are described in the
[GitHub releases](https://github.com/coin-or/SHOT/releases).

## Unreleased

### Changed

- The SHOTpy tutorial is a series of four notebooks in `docs/tutorial`, from
  getting started to solver features, Python workflows and advanced modeling,
  with their data files. The documentation site shows each notebook as a page
  and offers the folder as a download.

## 2.0.0 — 2026-11-10

Changes since SHOT 1.1.

### Breaking changes

- macOS ARM64 wheels and standalone packages require macOS 15 or newer
  ([#308]).
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
  solver exists; a problem used afterwards keeps the environment with its
  settings and output. Each solver previously leaked about 0.3 MB, e.g., when
  creating solvers in a loop in SHOTpy ([#258]).

### Added

- SHOTpy, a Python interface for building, solving and inspecting problems,
  including SOS and semicontinuous variables; it requires Python 3.11
  ([#185], [#192], [#208]).
- Wheels for Linux (x86_64 and arm64), macOS and Windows with Cbc, Ipopt and
  HiGHS, which also install the `SHOT` executable for use from AMPL, Pyomo
  and JuMP, and standalone executable archives with SHA-256 checksums for
  Linux x86_64 and ARM64, macOS ARM64 and Windows x64 that need no Python
  ([#221], [#228], [#308]).
- Native Windows builds with MSVC ([#221]) and an installable CMake package
  with the target `SHOT::SHOTSolver` ([#214]).
- HiGHS as MIP solver ([#182], [#191], [#194], [4f096597]), and Uno
  ([#200]) and CONOPT ([#225]) as NLP solvers for the fixed-integer primal
  heuristic.
- Queries of what each MIP solver supports (quadratic objectives and
  constraints, SOS, semicontinuous and semi-integer variables, single-tree
  strategy, variable-bound defaults) without constructing a solver, in C++
  and through SHOTpy's `getMIPSolverCapabilities()` ([#299]).
- Evaluation of gradients, Hessians, interval bounds and interval
  derivatives from C++ and Python ([#216]).
- LDLᵀ reformulation of convex quadratic expressions ([#183], [f4a5daaa]).
- Reformulation of `a*x1*x2 >= c` with positive variables as convex
  constraints (`Model.Reformulation.Bilinear.InverseFormulation`) ([#226]).
- Supporting hyperplanes in feasible fixed-integer NLP solutions
  (`Primal.FixedInteger.CreateSupportingHyperplanes`, on by default) ([#234]).
- Fixed-integer NLP problems are also solved during the search for nonconvex
  continuous problems ([#235]), by default from the best dual solution, with
  at most `Primal.FixedInteger.MaxStartsPerCall` (5) starting points per call
  ([#306]).
- Final polishing of the solution with NLP solves
  (`Primal.PolishSolution.NumberOfPoints`, on by default), shown as
  `NLPFINAL` and skipped for problems without continuous variables ([#204],
  [#318]).
- `Solver::setStartingPoint()` and `getStartingPoint()` in C++ and SHOTpy to
  give a starting point after `setProblem()`. A feasible point is the first
  primal solution and the MIP start of the dual problems ([#246]); from a
  point that is not feasible, an NLP problem with the discrete variables
  fixed is solved, which may repair it
  (`Primal.FixedInteger.UseStartingPoint`, on by default) ([#261], [#262]).
- Starting points from GAMS ([#190]) and AMPL ([1b054e3e]), the
  `shot_options` environment variable in AMPL mode ([#227]), and the `tan`
  function in GAMS ([#189]).
- A `BoundTightening` callback location, where bounds from other techniques
  can tighten the variable bounds after SHOT's bound tightening
  (`Model.BoundTightening.FeasibilityBased.UseAfterCallback` propagates them)
  ([#247]).
- `setFunctionConvexity()` in C++ and SHOTpy, and the `functionConvexity`
  argument in SHOTpy, give the convexity of a constraint or objective
  function instead of detecting it. The solution report, the OSrL results and
  `getUserSpecifiedConvexityDescription()` tell when global optimality
  depends on such functions or on `Model.Convexity.AssumeConvex` ([#256]).
- In the multi-tree strategy for convex problems, the discrete values of the
  previous MIP solution are given to the MIP solver as a partial start
  (`Dual.MIP.UseDualSolutionStart`), which mainly helps HiGHS and Cbc find
  primal solutions ([#248]).
- Systems of equations without an objective, i.e., GAMS CNS models and OSiL
  files without an objective, are solved by minimizing zero. SHOT stopped
  with an error for CNS models and crashed for such OSiL files ([#312]).
- Exit codes in the SHOT executable ([#207]).
- More information in the output: the SHOT version, problem types and
  subsolvers in the AMPL solve message ([d9db1155]); the number of MIP solver
  threads in the options report, with a warning when Cbc has no thread
  support ([66efbd6d]); a warning when a primal bound passes a global dual
  bound ([#236]); clear errors for problems without variables or objective
  ([5a82942f]); and MPS files of the MIP problems in the debug output
  ([64b10f44]).
- Documentation on the website: command-line usage, compilation,
  configuration, modeling and reformulations, understanding the output,
  callbacks, MIP solver capabilities and debugging; a SHOTpy tutorial; a C++
  and Python API reference; and a settings reference that lists the settings
  of all MIP and NLP solvers and of GAMS, also when SHOT is built without
  them, with the recommended values for nonconvex problems ([06f753cd],
  [#206], [#220], [#240], [#242], [#295], [#299]). The C++ API documents, as
  the Python API, that `Solver::getCurrentDualBound()` is not a valid bound
  for nonconvex problems; `Solver::getGlobalDualBound()` is.
- Contribution guidelines (`CONTRIBUTING.md`) and this changelog ([#240]).

### Changed

- Equality constraints and ranges are kept as they are in the original
  problem ([#210]).
- Much faster reading, reformulating and solving of large problems, and
  monomial and signomial terms with the same factors are merged ([#204],
  [#212], [#237]).
- Repeated runs with the same settings give the same results, except when
  limited by time ([#204]).
- Quadratic objectives and constraints are passed to the MIP solver whenever
  it supports them, instead of being reformulated or solved with the NLP or
  multi-tree strategy: convex quadratic objectives also when other
  constraints are nonconvex, nonconvex quadratic objectives and constraints
  with Gurobi, nonconvex QP and MIQP and convex maximization QCQP and MIQCQP
  with CPLEX, and the supported quadratic parts of functions with
  higher-degree terms. Gurobi no longer stops with "Objective Q not PSD"
  when the recommended settings are not used. Problems that are MILPs after
  reformulation are solved directly by the MIP solver, unless a
  `HyperplaneSelection` callback is registered ([#287], [#298], [#299],
  [#301], [#302]).
- Bilinear terms with integer variables:
  `Model.Reformulation.Bilinear.IntegerFormulation.MaxDomain` is 250 instead
  of 100, and also applies to bilinear terms extracted to auxiliary
  variables, e.g., with `Model.Reformulation.Quadratics.ExtractStrategy=2`.
  SHOT tells how many such terms are not discretized, and warns when
  bilinear terms get nonconvex auxiliary constraints although
  `Model.Convexity.AssumeConvex` is true ([#282]). The discretization uses
  linear instead of quadratic memory and time ([#297]).
- Repeated Ipopt NLP solves reuse the initialized solver ([#296]).
- The cutoff is given to Cbc as a solver parameter, as for CPLEX and Gurobi
  ([ec55766b]).
- Dual stagnation is again checked as a termination criterion ([cf408fbd]).
- The MIP start includes the values of the reformulation variables
  ([8c4a2bb8]).
- Improved primal reduction cut heuristic ([#178]).
- NLP rows in the iteration table are numbered by the fixed-integer NLP
  problems solved ([#306], [#317]).
- Settings of a MIP or NLP solver or of GAMS that SHOT is not compiled with
  are accepted instead of giving an error, so that the same options files
  work with all builds; a warning tells that they are ignored if they are
  changed from their defaults ([#242]).
- Updated bundled third-party libraries ([#213]); MC++ is pinned to 5.1.0
  and the optional Uno compatibility target is 2.9.3 ([#304]).
- New repository layout ([#220], [#240]).

### Fixed

- Early stops without a primal solution on nonconvex problems, with "no
  additional dual cuts can be added": the infeasibility repair continues when
  there is no primal solution yet, cuts are added for the deviating
  constraints also when the most deviating ones have already been cut in the
  point (`Dual.HyperplaneCuts.MaxConstraintFactor` is now relative to the
  largest deviation not cut yet), and an integer combination with an
  infeasible fixed-integer NLP problem is excluded with an integer cut
  ([#257], [#312]).
- Integer cuts: with Cbc, an integer cut excludes exactly its own
  combination of values, where it could exclude the wrong one or too few
  ([#312]); for convex problems, an integer cut after a fixed-integer NLP
  problem is only added when its solution is optimal and accepted, so that a
  better solution is not excluded ([#285]); and discrete values are rounded
  instead of truncated when checking whether a combination has been tested
  ([#260]).
- Wrong solutions reported as optimal: candidates from MIP solution pools
  and callbacks are checked against the linear constraints also with
  `Primal.Tolerance.TrustLinearConstraintValues` ([#284]); MIP solutions are
  projected onto the variable bounds before nonlinear functions are
  evaluated ([#272]); and a solution that fulfils
  `Termination.ConstraintTolerance` is reported instead of one that only
  fulfils the looser primal tolerances, if it is within the gap tolerances
  ([#265]).
- Wrong dual bounds from badly scaled dual problems: the cuts of the initial
  polyhedral outer approximation and the lazy cuts of the single-tree
  strategy are scaled by their largest coefficient as the other cuts
  ([#268], [#270]), and huge bounds of auxiliary variables (from 1e15) are
  treated as infinite in the MIP problem ([#286]).
- Wrong results from interval arithmetic and convexity detection: the
  interval bounds of `cos`, `sin` and `tan` for arguments with large bounds
  ([#283]), the convexity of `tan` over a pole, and floating-point
  exponents that GCC could round with the integer `abs`, e.g., treating a
  power 1.5 as linear ([#307]).
- The antiepigraph reformulation gives correct objective values for scaled,
  offset and maximized objectives, and is not used when it is not valid for
  the constraint ([#292], [#293]).
- Dual problems wrongly taken as infeasible or failing: Gurobi, CPLEX and Cbc
  solve them again with more robust numerical settings, and artificial
  bounds are removed before infeasibility is accepted ([#288]); infeasibility
  because of the cutoff gives the cutoff as dual bound ([#263]);
  `Dual.MIP.UpdateObjectiveBounds` tightens the objective bounds in the sense
  of the dual problem, where maximization problems were reported infeasible
  ([#269]); and numerical trouble in the relaxation of a problem with
  discrete variables ends only the relaxation ([#271]).
- The ESH method adds a cut for a violated convex constraint also when the
  root search ends on the boundary ([#255]), and with Gurobi, the interior
  point search finds a point also when Gurobi's presolve reports the minimax
  problem as infeasible or unbounded ([#259]).
- Gurobi: a continuous convex problem with quadratic constraints gets a dual
  bound, numerical trouble is retried with `NumericFocus` 3 ([#271]), and
  SHOT no longer crashes in the single-tree strategy after integer cuts of
  nonbinary variables ([#267]) or reads past empty arrays when adding SOS
  or repairing infeasibility ([#307]).
- CPLEX ([#245], [#280]):
  - The MIP start is used in the multi-tree strategy; it was deleted
    whenever cuts were added.
  - The cutoff constraint keeps the objective constant when the cutoff is
    updated, where it could cut off better solutions.
  - Repairing or removing cuts no longer changes other constraints of
    problems with quadratic constraints.
  - SHOT no longer crashes in the single-tree strategy after integer cuts of
    nonbinary variables.
  - Failed and presolve-infeasible dual problems are solved again with
    numerical emphasis or without presolve instead of stopping SHOT.
- Integer variables in bilinear terms: unbounded ones, or ones with only
  artificial bounds, are no longer discretized when the term is extracted to
  an auxiliary variable, which used up all memory ([#282]), and products are
  not linearized when this would need big-M values of 1e15 or more ([#289]).
- Ipopt solves stop when iteration values become NaN or infinite, which
  could stall the linear solver past the time limit ([#294]).
- A GAMS NLP solver licence failure gives one warning and switches to GAMS
  IPOPT ([#291]).
- Time limits and times: the bound tightening, initial outer approximation
  and primal NLP setup in `setProblem()` stop at the time limit ([#251]);
  Cbc's time limit is wall-clock instead of CPU time summed over threads
  ([#311]); and the time report gives the time spent in the MIP solver for
  relaxed and MIP problems ([46affef9], [a49ecf75]).
- Results: the OSrL file has the values with all significant digits instead
  of six decimals ([4b31c007]); the OSrL results and trace file give the NLP
  solver used for the fixed-integer NLP problems ([#252]); the largest linear
  constraint error of a problem without linear constraints is no longer
  undefined, and cuts for nonconvex constraints are no longer postponed or
  added because of values read outside the primal solution ([#260]); and the
  iteration table labels MIQCQP subproblems correctly ([#295]).
- SHOT no longer crashes when reading several monomial or signomial terms in
  the objective with `Model.Reformulation.ObjectiveFunction.EpigraphStrategy=2`
  and `Model.Reformulation.Constraint.PartitionNonlinearTerms=0` ([#281]).
- Models with only SOS constraints and continuous variables use the discrete
  strategy with Cbc, CPLEX rejects negative semicontinuous and semi-integer
  variables it does not support, and the settings of the fallback for an
  unavailable MIP solver are applied before reformulation ([#299]).
- SHOT builds with `CMAKE_BUILD_TYPE=RelWithDebInfo` ([#307]).
- Many fixes for wrong results, invalid dual bounds, crashes and
  non-termination, mainly on nonconvex problems, found by running the
  MINLPLib and MINLPTests instances with several MIP and NLP solvers
  ([#153], [#171], [#204], [#209], [#211], [#215], [#222], [#231], [#239]).
- Fixes in the GAMS and AMPL interfaces ([#150], [#162], [#181], [#196],
  [#198], [#227], [46404779], [801c7809]) and in the Gurobi, Cbc, HiGHS and
  Ipopt interfaces ([#151], [#155], [#170], [#172], [#218], [#232],
  [15695f16], [7f2b2af7], [bca3b691]).

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
[#260]: https://github.com/coin-or/SHOT/pull/260
[#261]: https://github.com/coin-or/SHOT/pull/261
[#262]: https://github.com/coin-or/SHOT/pull/262
[#263]: https://github.com/coin-or/SHOT/pull/263
[#265]: https://github.com/coin-or/SHOT/pull/265
[#267]: https://github.com/coin-or/SHOT/pull/267
[#268]: https://github.com/coin-or/SHOT/pull/268
[#269]: https://github.com/coin-or/SHOT/pull/269
[#270]: https://github.com/coin-or/SHOT/pull/270
[#271]: https://github.com/coin-or/SHOT/pull/271
[#272]: https://github.com/coin-or/SHOT/pull/272
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
[#306]: https://github.com/coin-or/SHOT/pull/306
[#307]: https://github.com/coin-or/SHOT/pull/307
[#308]: https://github.com/coin-or/SHOT/pull/308
[#311]: https://github.com/coin-or/SHOT/pull/311
[#312]: https://github.com/coin-or/SHOT/pull/312
[#317]: https://github.com/coin-or/SHOT/pull/317
[#318]: https://github.com/coin-or/SHOT/pull/318
[06f753cd]: https://github.com/coin-or/SHOT/commit/06f753cd
[15695f16]: https://github.com/coin-or/SHOT/commit/15695f16
[1b054e3e]: https://github.com/coin-or/SHOT/commit/1b054e3e
[46404779]: https://github.com/coin-or/SHOT/commit/46404779
[46affef9]: https://github.com/coin-or/SHOT/commit/46affef9
[4b31c007]: https://github.com/coin-or/SHOT/commit/4b31c007
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
[cf408fbd]: https://github.com/coin-or/SHOT/commit/cf408fbd
[d9db1155]: https://github.com/coin-or/SHOT/commit/d9db1155
[ec55766b]: https://github.com/coin-or/SHOT/commit/ec55766b
[f4a5daaa]: https://github.com/coin-or/SHOT/commit/f4a5daaa
