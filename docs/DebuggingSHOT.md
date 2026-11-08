# Debugging SHOT

Use this guide to investigate wrong bounds, missing solutions, unexpected
reformulations, and slow convergence using intermediate models and solution
points. For the normal terminal report, start with
[Understanding SHOT output](UnderstandingOutput.md).

For CLI syntax, see [command-line usage](CommandLineUsage.md). For settings,
see [configuration](Configuration.md), the generated
[reference](reference/settings.md), or `./SHOT --docs` for your executable.

## Enabling debug mode

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

## Make the console output verbose while debugging

SHOT's console/log output is itself a primary debugging tool, complementary
to the debug directory — it shows what the solver is doing iteration by
iteration without needing to open individual files. When debugging, it's
usually worth turning up verbosity on top of `--debug`:

- `Output.Console.Iteration.Detail=0` — always set this to `0` (`Full`) when
  debugging. The default (`1`, "on objective gap update") prints bound
  updates, selected exceptional events, and periodic progress after more than
  100 suppressed calls or two seconds. It can hide intervening iterations.
  `0` prints all progress rows to both console and file log.
- `Output.Console.DualSolver.Show=true` — also prints the underlying MIP
  solver's own output (its log lines, e.g. B&C node counts/gaps) inline in
  the console, not just SHOT's summary line. Useful when the MIP solver
  itself is slow or behaving unexpectedly. (Setting this to `true`
  automatically forces `Output.Console.Iteration.Detail` to `Full` as well,
  since SHOT does this internally.) There's an analogous
  `Output.Console.PrimalSolver.Show=true` for showing the primal/NLP
  solver's own output.
- `Output.Console.LogLevel` / `Output.File.LogLevel` — control how much is
  logged to the console vs. the `SHOT.log` file, independently. Values (use
  the integer on the command line): `0`/Trace, `1`/Debug, `2`/Info
  (default), `3`/Warning, `4`/Error, `5`/Critical, `6`/Off. Lower
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

## Solution strategies

SHOT picks one of a small number of **solution strategies** — each a
different sequence of tasks — based on the (reformulated) problem's
detected type and settings. This matters for debugging because it
determines which debug files you should expect to see at all:

- **Direct MILP strategy** — used for complete MILP models, including linear
  reformulations of nonlinear models. Solves the model directly with the
  selected MIP backend, without an initial LP relaxation or SHOT's single-tree
  callbacks. A registered `HyperplaneSelection` callback instead selects the
  tree strategy so that user cuts can be added.
- **MIQCQP strategy** — used for convex QP/MIQP/QCQP/MIQCQP problems supported by CPLEX or Gurobi
  and for nonconvex QP/MIQP problems with CPLEX when quadratic objectives
  are enabled. Nonconvex quadratic problems also use this strategy with
  Gurobi when
  `Model.Reformulation.Quadratics.Strategy=3` (recommended for nonconvex
  problems with Gurobi). The reformulated model must still contain quadratic
  terms and have no general nonlinear terms.
  Solves directly via the MIP solver plus primal heuristics; no
  outer-approximation cutting-plane loop, so no `dualiter{N}_mostdev.txt`-
  style constraint-deviation files (there's nothing nonlinear to cut
  against).
- **NLP strategy** — used for convex, purely continuous (non-discrete) LP/NLP
  problems. Solves continuous dual problems and can use NLP primal heuristics;
  inspect the dual problem files and any NLP solve output/log.
- **Multi-tree strategy** — the classical SHOT algorithm for general convex
  MINLP models and MILPs with user cut callbacks: repeatedly solves a MIP
  relaxation, potentially stopping at a solution limit, adds
  supporting-hyperplane/ECP cuts, and resolves. This generates the full
  `dualiter{N}_*` sequence described below.
- **Single-tree strategy** — solves one branch-and-cut MIP tree, adding
  cuts via callbacks during the search rather than resolving from scratch
  each iteration (requires solver callback support — currently only CPLEX
  and Gurobi; Cbc and HiGHS are always forced to multi-tree, regardless of
  `Dual.TreeStrategy`). Selected via the `Dual.TreeStrategy` setting
  (`0`=multi-tree, `1`=single-tree) when the problem is discrete and
  convex; still produces `dualiter{N}_*` files, but "iteration" here
  corresponds to callback invocations within the single tree rather than
  separate resolves.

The **Options** section reports the selected dual strategy at normal Info
verbosity. Check it first if the debug directory does not contain the files
you expected. Debug logging adds details about strategy selection.

## Iteration numbering

Debug filenames use dual-iteration and primal-search counters. These need not
match the number of printed rows or accepted primal solutions. For the terminal
row counters and original/reformulated constraint indices, see
[the output guide](UnderstandingOutput.md#main-iteration-columns).

## Debug file reference

Every LP model dump listed below also has an MPS counterpart with the same
basename (for example, `dualiter0_problem.lp` and `dualiter0_problem.mps`).
This applies to Cbc, CPLEX, Gurobi, and HiGHS, including interior-point,
convex-bounding, and infeasibility-repair models. Cbc may append `.gz` to
its MPS files when its COIN-OR libraries support compression.

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
- `lazycuts.txt` and `singletree_problem_with_lazycuts.lp` — single-tree
  strategy with Gurobi only. The cuts added as lazy constraints in the
  callback are not part of `dualiter{N}_problem.lp`; `lazycuts.txt` lists each
  one in LP syntax as it is added (`LAZY{i}_HP` hyperplanes, `_IC` integer
  cuts, `_DB` external dual bounds), and after the tree search the model is
  written once more with all of them as ordinary constraints. Solving that
  file with `gurobi_cl` shows whether the cuts themselves cut off a known
  solution, or whether a wrong bound only arises in the callback.

## A practical debugging workflow

1. Reproduce with `./SHOT problemfile --debug=<dir>`, adding the console
   verbosity settings from “Make the console output verbose while debugging” (at minimum
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

## Developer debugging

For reproducing regressions with the C++ and Python test suites, task-graph and
exception-handling pitfalls, and detailed investigations of bounds and
reformulations, see [Developer debugging](DeveloperDebugging.md).
