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
  each iteration (only available with MIP solvers that support solver
  callbacks). Selected via the `Dual.TreeStrategy` setting
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
  `6`=Gurobi+SHOT-as-NLP — only the combinations matching what's actually
  compiled in are registered). Add `-v` for verbose per-instance solver
  output:

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

## 8. If this doc is stale

File names and which task writes them are derived from the source and can
drift as SHOT evolves. If a described file is missing or looks different,
grep `src/` for `Output.Debug.Enable` to find all debug-output call sites,
or specifically for `Utilities::writeStringToFile` and
`Utilities::saveVariablePointVectorToFile` calls (the two generic writers
most debug files go through) to see current behavior.
