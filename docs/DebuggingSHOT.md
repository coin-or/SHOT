# Debugging SHOT

This guide is for anyone (human or AI agent) debugging solver behavior — wrong
bounds, missing/incorrect solutions, unexpected reformulations, slow
convergence — using SHOT's built-in debug-output mechanism. It documents the
actual files SHOT writes, derived from reading the source.

For general CLI usage and the full options reference, see
https://www.shotsolver.dev/shot/using-shot/getting-started and `./SHOT --docs`
(below). This document only covers the debug-output mechanism.

## 1. Enabling debug mode

```bash
./SHOT problemfile.osil --debug=<directory>
```

- `--debug` alone enables debug mode but leaves the output path unset, in
  which case SHOT auto-creates a temp directory (`SHOT_debug_<random-hex>`
  under the OS temp dir) and prints it to the console/log as
  `Debug directory: <path>`. **Always pass `--debug=<directory>` explicitly**
  so you know where to look without parsing log output.
- Equivalent settings (settable via an options file passed with `--opt`, or
  inline as `Key=value` arguments): `Output.Debug.Enable=true` and
  `Output.Debug.Path=<directory>`.
- SHOT copies the original input problem file into the debug directory
  verbatim (e.g. `myproblem.osil`) — that's just a copy for reference, not a
  generated artifact.
- A tight `--timelimit=<seconds>` (or a small problem) keeps the number of
  iteration files manageable when you only need a few iterations to
  reproduce an issue. `Termination.IterationLimit=<n>` (default 200000)
  caps the number of dual/MIP iterations directly and is often more useful
  than a time limit when you want a deterministic, reproducible number of
  debug files regardless of machine speed.
- Full CLI syntax and switches (including which problem file formats a
  given build supports) are documented in
  [docs/CommandLineUsage.md](CommandLineUsage.md).

## 2. Make the console output verbose while debugging

SHOT's console/log output is itself a primary debugging tool, complementary
to the debug directory — it shows what the solver is doing iteration by
iteration without needing to open individual files. When debugging, it's
usually worth turning up verbosity on top of `--debug`:

- `Output.Console.Iteration.Detail=0` — always set this to `0` (`Full`) when
  debugging. The default (`1`, "on objective gap update") only prints a
  console line when the bound improves, which hides iterations where SHOT
  is stuck. `0` prints every iteration.
- `Output.Console.DualSolver.Show=true` — also prints the underlying MIP
  solver's own output (its log lines, e.g. B&C node counts/gaps) inline in
  the console, not just SHOT's summary line. Useful when the MIP solver
  itself is slow or behaving unexpectedly. (Setting this to `true`
  automatically forces `Output.Console.Iteration.Detail` to `Full` as well,
  since SHOT does this internally.) There's an analogous
  `Output.Console.PrimalSolver.Show=true` for showing the primal/NLP
  solver's own output.
- `Output.Console.LogLevel` / `Output.File.LogLevel` — control how much is
  logged to the console vs. the `SHOT.log` file, independently. Values (as
  either the integer or the name): `0`/`Trace`, `1`/`Debug`, `2`/`Info`
  (default), `3`/`Warning`, `4`/`Error`, `5`/`Critical`, `6`/`Off`. Lower
  numbers are more verbose. Set `Output.Console.LogLevel=0` (or `=1`) to
  see SHOT's internal trace/debug log messages (e.g.
  `env->output->outputDebug(...)` calls throughout the source, such as
  "Using single-tree solution strategy.") in addition to the normal
  iteration output. Since the log file is separate from the console, you
  can keep the console at a normal level and set only
  `Output.File.LogLevel=1` to get a detailed trace in `SHOT.log` without
  flooding the terminal.

Example for a maximally verbose debugging run:

```bash
./SHOT problemfile --debug=<directory> \
  Output.Console.Iteration.Detail=0 \
  Output.Console.DualSolver.Show=true \
  Output.Console.LogLevel=1 \
  Termination.IterationLimit=50
```

## 3. Solution strategies

SHOT picks one of a small number of **solution strategies** — each a
different sequence of tasks — based on the (reformulated) problem's
detected type and settings. This matters for debugging because it
determines which debug files you should expect to see at all:

- **MIQCQP strategy** — used for convex MILP/QP/MIQP/QCQP/MIQCQP problems
  (i.e. problems the MIP solver can handle natively, quadratics included).
  Solves directly via the MIP solver plus primal heuristics; no
  outer-approximation cutting-plane loop, so no `dualiter{N}_mostdev.txt`-
  style constraint-deviation files (there's nothing nonlinear to cut
  against).
- **NLP strategy** — used for convex, purely continuous (non-discrete) NLP
  problems. No MIP solver involved at all, so no `dualiter*` files; look at
  the NLP solve output/log instead.
- **Multi-tree strategy** — the classical SHOT algorithm for general convex
  MINLP: repeatedly solves a MIP relaxation to completion, adds
  supporting-hyperplane/ECP cuts, and resolves. This is what generates the
  full `dualiter{N}_*` sequence described below.
- **Single-tree strategy** — solves one branch-and-cut MIP tree, adding
  cuts via callbacks during the search rather than resolving from scratch
  each iteration (requires solver callback support — currently only CPLEX
  and Gurobi; Cbc and HiGHS are always forced to multi-tree, regardless of
  `Dual.TreeStrategy`). Selected via the `Dual.TreeStrategy` setting
  (`0`=multi-tree, `1`=single-tree) when the problem is discrete and
  convex; still produces `dualiter{N}_*` files, but "iteration" here
  corresponds to callback invocations within the single tree rather than
  separate resolves.

Which strategy was actually used is printed to the console/log as a debug
line (e.g. "Using single-tree solution strategy.") when
`Output.Console.LogLevel`/`Output.File.LogLevel` is verbose enough (see
above) — check this first if the debug directory doesn't contain the files
you expected.

## 4. Why the output looks the way it does

SHOT's solver loop is task-based: a `TaskHandler` runs a sequence of task
objects (each a `run()` implementation) against a shared `Environment`
(`env`) that holds the problem, settings, subsolvers, and results. The dual
strategy (polyhedral outer approximation: add cuts → solve MIP → check gap →
loop) and the primal strategy (heuristics: MIP solution pool, fixed-integer
NLP solves) are separate task sequences that both run every iteration and
both write their own tagged files into the debug directory. This is why you
see two parallel numbering schemes — `dualiter{N}_*` for the outer
approximation loop and `primal*`/`primalnlp{N}_*` for primal heuristics —
rather than one single "iteration" concept. `N` is the dual/MIP iteration
counter; it does not necessarily line up 1:1 with how many primal solutions
were found.

## 5. Debug file reference

### Whole-problem dumps (written once per run)

| File | Contents | Use it to check |
|---|---|---|
| `<inputname>.osil`/`.gms`/`.nl` | Verbatim copy of the input file | You're debugging the run you think you are |
| `originalproblem.txt` | SHOT's parsed view of the **original** problem: objective, constraints (each tagged with detected curvature, e.g. `[L-convex]`, `[Q-convex]`, `[NL-convex]`), then variables with type/bounds | Whether SHOT parsed/classified the model as you intended |
| `reformulatedproblem.txt` | The problem **after** reformulation (convexification of bilinear/signomial terms, epigraph splitting, auxiliary variables like `s_sq_q_evd_*`) | What SHOT is actually solving as the dual problem — compare against `originalproblem.txt` when reformulation looks wrong |
| `sparsitypattern_jacobian.txt` / `_ref.txt` | Per-constraint list of variables in its gradient (non-`_ref` = original, `_ref` = reformulated) | Whether a variable you expect in a constraint is actually there |
| `sparsitypattern_hessianoflagrangian.txt` / `_ref.txt` | Variable-pairs present in the Lagrangian Hessian | Second-order structure / convexity investigations |
| `usedsettings.opt` | Every effective setting for this run, in `.opt` format | The run actually used the settings you think it did (defaults + overrides all resolved) |

### Dual (MIP / outer-approximation) iterations — one set per iteration `N`

| File | Contents | Use it to check |
|---|---|---|
| `dualiter{N}_problem.lp` | The polyhedral MIP relaxation solved at iteration N, in LP format | What cuts/constraints the MIP solver actually saw |
| `dualiter{N}_mipstart.txt` | Warm-start point given to the MIP solver (`varname\tvalue` lines) | Whether a bad warm start is causing solver trouble |
| `dualiter{N}_solpt_{k}.txt` | The k-th solution in the MIP solution pool at iteration N | Alternative solutions the MIP considered |
| `dualiter{N}_solinfo_{k}.txt` | Two lines: objective value and max constraint deviation `[index]: value` for pool solution k | Quickly compare pool solutions without re-parsing full points |
| `dualiter{N}_mostdev.txt` | Single line: the most-violated nonlinear constraint at this iteration's solution, `[index]: value` | **The single most useful file for slow/stalled convergence** — read it across increasing N; a constraint index that never stops appearing is where SHOT is struggling to cut off the relaxation |

Example `dualiter5_mostdev.txt`:
```bash
most dev. constraint ([index]: value)	[40]: 3.469446951953614e-18
```
(here the deviation is ~0, i.e. essentially converged at this constraint).

### Interior-point search (runs once, before the main loop, to seed ESH)

| File | Contents | Use it to check |
|---|---|---|
| `minimax{i}.lp` | LP subproblem at inner iteration i of the interior-point cutting-plane/minimax search | Interior-point search not converging |
| `minimax{i}_solpt.txt` | Solution point of that LP | Where the search is heading |
| `interiorpoint_notused_{i}.txt` | A candidate interior point that was **rejected** (max nonlinear deviation ≥ 0, i.e. not strictly interior) | Why SHOT couldn't find a valid interior point (common cause of a stuck/failed run on nonconvex or numerically hard feasible regions) |
| `interiorpoint_{i}.txt` | An **accepted** interior point (only appears if one was found) | The point used to seed ESH supporting-hyperplane cuts |

### Primal (feasible-solution) search

| File | Contents | Use it to check |
|---|---|---|
| `primalnlp{N}_warmstart_{k}.txt` | Continuous warm-start point (`varname\tvalue`) given to the fixed-integer NLP solve, candidate k, triggered around dual iteration N | What starting point the NLP heuristic was given |
| `primal_solpt{N}.txt` | The N-th primal-feasible solution SHOT found overall (source, iteration found, objective value, largest constraint errors, then the full solution point) | Tracing where/how each improving feasible solution was found |

### Conditional / backend-specific files (not always present)

These exist in the source but only fire for specific solver backends or
algorithm branches, so don't be surprised if they're missing from a given
run:

- `lp0.lp` — full MIP rebuild when the multi-tree strategy reinitializes.
- `dualiter{N}_infeasrelaxweights.txt`, `_infeasrelax.lp`, `_repaired.lp` —
  MIP-solver-backend-specific infeasibility-repair diagnostics; not every
  MIP solver backend implements this.
- `convexbounding_problem{N}.lp` — convex-only bounding MIP for nonconvex
  problems.
- `interiorpointnlp{i}.txt`, `interiorpoint_provided{i}.txt`,
  `interiorpoint_provided_notused_{i}.txt` — variants of the interior-point
  search, including user/callback-provided candidates.
- `primalnlp{N}_{k}.txt` + matching `.osrl` — the fixed-integer NLP
  subproblem itself and its solver options (only written by NLP backends
  that support dumping their problem).

## 6. A practical debugging workflow

1. Reproduce with `./SHOT problemfile --debug=<dir>`, adding the console
   verbosity settings from section 2 (at minimum
   `Output.Console.Iteration.Detail=0`) and either `--timelimit=<n>` or
   `Termination.IterationLimit=<n>` if you only need to see the first
   several iterations. Note which solution strategy was selected (section
   3) so you know which debug files to expect.
2. **Model looks wrong / unexpected reformulation** → compare
   `originalproblem.txt` against `reformulatedproblem.txt`; check the
   curvature tags and look for unexpected auxiliary variables.
3. **Dual bound not improving / stalling** → read `dualiter{N}_mostdev.txt`
   for increasing `N`; if the same constraint index keeps showing up, open
   the matching `dualiter{N}_problem.lp` to see the relaxation SHOT is
   actually cutting against.
4. **No primal (feasible) solution found** → check
   `interiorpoint_notused_*.txt` first (a bad/rejected interior point often
   means ESH cuts never got seeded properly), then look for
   `primalnlp{N}_warmstart_{k}.txt` / `primal_solpt{N}.txt` to see what the
   NLP heuristic was given and whether it ever produced a candidate.
5. **"Is this the run I think it is?"** → check `usedsettings.opt` for the
   actual effective settings.
6. For the full, always-current settings reference (not just debug-related
   settings), run `./SHOT --docs`, which writes `options.md` to the current
   directory from the live source of truth — prefer that over guessing
   setting names from this doc.

## 7. Automated tests as a debugging aid

SHOT has a C++ test suite (optional at build time: configure with
`-DCOMPILE_TESTS=on`) that can help pin down when/where a regression was
introduced, or reproduce a failure in isolation instead of debugging a full
`./SHOT` run.

- **How it's built**: `test/CMakeLists.txt` compiles every `*Test.cpp` file
  under `test/` (e.g. `ModelTest.cpp`, `SettingsTest.cpp`, `SolverTest.cpp`,
  `InstanceTest.cpp`, plus `CbcTest.cpp`/`CplexTest.cpp`/`GurobiTest.cpp`/
  `HighsTest.cpp`/`IpoptTest.cpp`/`GAMSTest.cpp` for whichever
  subsolvers were compiled in) into a single `test_runner` executable
  (CMake's `create_test_sourcelist` mechanism). Each file defines an
  `int <Name>Test(int argc, char* argv[])` entry point that dispatches to
  individual test cases by a numeric "part" (`argv[1]`); the list of parts
  per test group is in `test/CMakeLists.txt` (e.g. `Model` has parts 1–19,
  `Solver` 1–13, `Settings` 1–9).
- **Running the full suite**: `ctest --output-on-failure` from the build
  directory (this is what CI does). If you're on a build configured before
  this doc's revision that fixed `test/CMakeLists.txt`'s `add_test()` call
  (it used to reference an always-unset `TEST_PATH` variable and resolve to
  the literal, nonexistent path `/test_runner`, since generator expressions
  are silently ignored by CMake's legacy positional `add_test(<name>
  <command>)` form), re-run `cmake .` in your build directory to regenerate
  `CTestTestfile.cmake` before trying `ctest` again.
- **Running/debugging one test directly** (most useful when actually
  debugging): from the build directory,

  ```bash
  ./test/test_runner <Name>test <part>
  ```

  e.g. `./test/test_runner Modeltest 6` or `./test/test_runner Solvertest 9`
  (name casing matters: capitalized group name + lowercase `test`, no
  space). This prints that one test's full, unfiltered stdout — much
  easier to read than a `ctest` run, and a good target to run under a
  debugger or with extra print statements.
- **`InstanceTest` — the most directly useful one for solver-behavior
  regressions**: solves a batch of real problem instances (currently
  sourced from MINLPTests.jl, listed in
  `test/data/instances/<group>/instances.json` with each entry's expected
  `objective` value or `"status": "infeasible"`) and checks the result
  against a tolerance, printing `[PASS]`/`[WARN]`/`[SKIP]` per instance and
  a summary with lists of warned/failed instances. The numeric part
  selects the MIP+NLP solver combination (`1`=HiGHS+Ipopt, `2`=Gurobi+Ipopt,
  `3`=Cplex+Ipopt, `4`=Cbc+Ipopt, `5`=HiGHS+SHOT-as-NLP,
  `6`=Gurobi+SHOT-as-NLP, `7`=Cplex+SHOT-as-NLP, `8`=Cbc+SHOT-as-NLP — this
  is a fixed, hand-registered list in a `switch` statement, not every
  theoretically valid pairing; see section 9 below). Add `-v` for verbose
  per-instance solver output:

  ```bash
  ./test/test_runner Instancetest 1 -v
  ```

  If an instance that used to `[PASS]` now regresses, that's a concrete,
  minimal repro: rerun the same file directly,
  `./SHOT test/data/instances/<group>/<file> --debug=<dir>
  Output.Console.Iteration.Detail=0`, and work through the debugging
  workflow in section 6 above.
- **Python API tests** (`test/python/*.py`, run via `pytest`) only register
  if Python bindings are built (`-DHAS_PYTHON=on`, target `SHOTpy`) and
  `pytest` is importable; they cover the Python/SHOTpy binding surface
  (modeling, constraints, callbacks, gradients/Hessians) rather than the
  core solve algorithm, so they're more useful for API-layer bugs than
  solver-behavior ones.

## 8. Lessons from real debugging sessions

These are specific, non-obvious traps encountered while actually chasing bugs
in this codebase — add to this list as you find more.

### Build hygiene

- Rebuild every binary that exercises your change, not just `SHOT`.
  `test_runner` statically links `libSHOTTasks.a`, `libSHOTDualStrategy.a`,
  etc.; `make SHOT` alone does not relink it. A fix that "doesn't seem to
  work" under `test_runner` may just be stale object code — after any
  `src/` edit, run the default `make -j` target, or explicitly
  `make SHOT test_runner`, rather than building only the one target you
  think you need.

### Settings pitfalls

- An unrecognized `Key=Value` CLI argument (a typo, or a missing prefix
  like `Dual.`) is silently dropped, not rejected — SHOT falls back to
  that setting's default with no warning. Don't trust that a flag "took"
  just because the run didn't error; confirm via the console header
  (`Running HiGHS 1.15.1...`, `Dual strategy: ... solver: HiGHS`) or
  `usedsettings.opt` that the solver/setting you intended is actually the
  one in effect.

### CppAD / numerical domain errors

- A crash mentioning CppAD and "nan" (`forward.hpp`, `subgraph_reverse.hpp`)
  is usually evaluating a nonlinear expression outside its domain (division
  by a variable that is zero, the gradient of a Euclidean-norm term at the
  origin, log of a negative number, etc.) — not a logic bug in SHOT's own
  code. Before patching individual call sites, check `ADFun::check_for_nan()`:
  the assertion that throws is wrapped in `#ifndef NDEBUG`, so it is
  compiled out entirely in release builds, which already rely on NaN
  flowing through silently to the several existing downstream NaN checks
  (`NLPSolverCuttingPlaneMinimax.cpp`, `MIPSolverBase::createHyperplaneTerms`).
  Calling `ADFunctions.check_for_nan(false)` once, in `Problem.cpp` right
  after `Dependent()`, is usually the correct, general fix — not scattered
  try/catch at every call site.

### Exception-handling pitfalls specific to this codebase

- Grep for `throw new` before assuming a `catch(const std::exception&)`
  will actually catch a thrown `SHOT::Exception` subclass. Some throw sites
  throw a heap-allocated *pointer* (`throw new X(...)`) instead of the
  exception object — a pointer never matches `catch(const std::exception&)`,
  so the exception propagates uncaught no matter how much catching you add
  upstream. Fix the throw site itself (`throw X(...)`, no `new`) before
  wrapping callers in try/catch.
- A `false`/silent-failure return from a `bool`-returning setup function
  (`setProblem()`, `createProblem()`, `selectStrategy()`) is very often
  ignored by its caller several layers up (e.g. `TaskCreateMIPProblem::run()`
  never checks `createProblem()`'s return value). `Solver::selectStrategy()`
  itself has an internal try/catch that swallows initialization exceptions
  and returns `false` rather than propagating them. So when a crash or
  assert happens deep inside a *second* use of an object (e.g. `solveProblem()`
  called well after construction, on an already-"successfully constructed"
  solver), don't assume construction actually succeeded just because nothing
  threw — walk backward through the chain of `bool` returns to find where a
  failure was silently dropped.

### Task-graph state (`SolutionStrategyMultiTree` and friends)

- `TaskHandler` is a flat, mutable "next task" pointer — any task can call
  `setNextTask()`, and fields like `Results::terminationReason` are not
  exclusively a "we've decided to stop" signal: some tasks (e.g.
  `TaskCheckPrimalStagnation`) reuse it as an internal marker for their own
  retry logic while redirecting elsewhere in the loop, not to
  `FinalizeSolution`. Before gating a task on a shared/global field like
  `terminationReason` or `isTerminated()`, grep every place that sets *and*
  reads it, not just the one call site of the bug you're chasing — an
  overly broad guard can silently break a legitimate internal loop instead
  of the one you meant to fix. If a task class is reused in two different
  structural roles (e.g. main-loop vs. `FinalizeSolution`-embedded),
  consider a constructor flag to scope the fix to the role that's actually
  broken, the way `TaskAddPrimalReductionCut`'s `isFinalAttempt` does.

### Nested/recursive solves (`NLPSolverSHOT`, `TaskPerformBoundTightening`)

- Some code paths construct an entire second `Solver` internally
  (`NLPSolverSHOT` for the SHOT-as-NLP primal heuristic,
  `TaskPerformBoundTightening`'s `POASolver`). These deliberately call
  `Problem::createCopy(..., copyAuxiliary=false)` and pass the *same*
  problem object as both `problem` and `reformulatedProblem` to
  `setProblem()`, which skips `TaskReformulateProblem` — this is
  intentional, not an oversight: reformulating would construct entirely
  new `Variable` objects, breaking the later `setVariableBounds()` /
  `fixVariables()` calls these classes rely on to mutate the *same*
  variables the nested solver actually solves against. "Just let it
  reformulate" looks like the natural fix for a nested-solve bug but
  silently breaks correctness (bounds mutated on a now-disconnected copy)
  instead of crashing, which is a worse failure mode. Grep for
  `setVariableBounds`/`fixVariables` calls on the same member before
  changing how a nested solve is initialized.

### Verification discipline

- `InstanceTest`'s solver combinations are a fixed, hand-registered list in
  a `switch` (`test/InstanceTest.cpp`) — not every theoretically valid
  `Dual.MIP.Solver` × `Primal.FixedInteger.Solver` pairing is actually
  tested. "All instance tests pass" only means the *registered* combos
  pass; check the switch statement before trusting that a specific pairing
  has ever been exercised. Adding an untested combo is a cheap, high-yield
  way to find real bugs.
- A fix that stops a crash is not the same as a fix that makes the instance
  solve correctly — check both separately. `[WARN] no primal solution`, or
  an objective far from the instance's expected value in `instances.json`,
  after a crash fix usually means there's a second, often pre-existing,
  issue; don't conflate "no longer crashes" with "now works."
- Any change to shared, widely-reused numerical machinery (the
  interior-point/cutting-plane search in `NLPSolverCuttingPlaneMinimax.cpp`,
  the CppAD tape, anything touched by many call paths) needs a full
  `InstanceTest` regression sweep across *all* registered combos, not just
  the instance you're fixing — its blast radius is the whole suite. A
  targeted change that "fixes" one instance can silently break others.

### Fast, targeted crash diagnosis

- Reach for `lldb -b -s <script>` early, not after several rounds of
  `fprintf`/rebuild. A batch script with
  `breakpoint set --func-regex <ExceptionClassName>` (for an uncaught
  exception) or `breakpoint set --file X.cpp --line N` (for a specific
  `assert`), followed by `run ...` and `bt`, gets a definitive, full-frame
  stack trace in one pass — far faster than iterating on print statements
  across multiple rebuild cycles. Note macOS has no `timeout` shell
  command; use the Bash tool's own timeout parameter, `lldb -b` batch mode
  (which exits on its own), or background the process and poll instead.

## 9. If this doc is stale

File names and which task writes them are derived from the source and can
drift as SHOT evolves. If a described file is missing or looks different,
grep `src/` for `Output.Debug.Enable` to find all debug-output call sites,
or specifically for `Utilities::writeStringToFile` and
`Utilities::saveVariablePointVectorToFile` calls (the two generic writers
most debug files go through) to see current behavior.
