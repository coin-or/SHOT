# Changelog

User-visible changes to SHOT and SHOTpy. See [CONTRIBUTING.md](CONTRIBUTING.md)
for how to add entries. Changes in versions before 2.0 are described in the
[GitHub releases](https://github.com/coin-or/SHOT/releases).

## Unreleased

Changes since SHOT 1.1.

### Breaking changes

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
  ([#246]).
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
- New repository layout, and the documentation is on the website ([#220],
  [#240]).
- Settings of a MIP or NLP solver or of GAMS that SHOT is not compiled with
  are accepted instead of giving an error, so that the same options files
  work with all builds; a warning tells that they are ignored if they are
  changed from their defaults ([#242]).

### Fixed

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

### Removed

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
