# Configuring SHOT

The [settings reference](reference/settings.md) lists names, types, ranges, and
defaults. This guide explains when to set them and gives common configurations.
For the settings registered by your particular executable, run `SHOT --docs`.

## Set options before preparing the model

Names are case-sensitive. Unknown names raise an exception in the C++ and Python
API, stop processing an options file, and produce a warning on the command line.
Enum settings take integer values, not the descriptive labels in the reference.

On the command line:

```bash
SHOT problem.osil Termination.TimeLimit=60 Termination.ObjectiveGap.Relative=0.001
```

In Python, use `updateSetting()` before creating and finalizing a problem, or
before reading it with `setProblem()`. Reformulation and bound tightening use
settings during preparation; changing them afterwards does not repeat that work.

```python
import SHOTpy

solver = SHOTpy.Solver()
solver.updateSetting("Termination.TimeLimit", 60.0)
solver.updateSetting("Termination.ObjectiveGap.Relative", 0.001)
solver.updateSetting("Dual.MIP.NumberOfThreads", 1)

# Create Problem(solver), finalize it, and pass it to setProblem(),
# or read a problem with solver.setProblem("problem.osil").
assert solver.getDoubleSetting("Termination.TimeLimit") == 60.0
assert solver.getIntSetting("Dual.MIP.NumberOfThreads") == 1
```

C++ uses the same `solver.updateSetting("Category.Name", value)` convention.
Use the type listed in the reference; an integer is also accepted for a
floating-point setting.

## Which setting value wins?

Settings have priorities, rather than simply following the order of writes.
Higher rows in this table override lower ones:

| Source | Report tag | Purpose |
| --- | --- | --- |
| Solver compatibility | `[solver (compatibility)]` | Required changes for backend capabilities or the compiled build. |
| Solver internals | `[solver]` | Values changed by SHOT during the solution process. |
| User API or command-line argument | `[user]` | Explicit choices in C++/Python or on the command line. |
| Options file | `[options file]` | Values read from `.opt` or `.osol`. |
| Recommended internal value | `[recommended]` | Conditional defaults, for example for nonconvex problems. |
| Default | `[default]` | Registered initial value. |

Within a priority, the latest write wins. Explicit user choices override
recommended values, but cannot force a backend to support an unavailable
feature. For example, selecting single-tree with HiGHS or Cbc is overridden to
multi-tree. Read the **Non-default options** and selected strategy in the
terminal report to check the effective configuration.

`Strategy.UseRecommendedSettings=false` disables conditional recommended
settings; it does not disable compatibility changes. Prefer the recommendations
unless investigating a particular algorithmic choice. In AMPL mode,
command-line options take precedence over values in `shot_options`.

## Select subsolvers and a tree strategy

```bash
SHOT problem.osil --mip=highs --nlp=ipopt --tree=multi --threads=1
```

Use only backends present in your build. `SHOT --help` lists compiled CLI
choices. Cbc and HiGHS support multi-tree; CPLEX and Gurobi also support
single-tree and native quadratic subproblems. Proprietary backends require the
appropriate installation and license. The NLP solver searches for primal
solutions; an NLP call is not required in every run.

All public backend settings appear in the reference, including those for
backends absent from a build. SHOT warns when an unavailable backend's options
are changed from their defaults. See [compilation](CompilationInstructions.md)
for enabling a backend, and [solver concepts](concepts.md) for bounds and
optimality guarantees.

## Limits and tolerances

```bash
SHOT problem.osil --timelimit=60 Termination.IterationLimit=1000 \
  --absgap=0.0001 --relgap=0.001
```

`Termination.ObjectiveGap.Relative=0.001` means 0.1%. The absolute gap is in
objective units. Either gap tolerance can terminate the solve; near a zero
objective, the absolute criterion is often easier to interpret.

Gap tolerances describe the distance between the incumbent and global dual
bound. They are separate from feasibility tolerances:

| Setting | What it controls |
| --- | --- |
| `Primal.Tolerance.Integer` | Integrality of accepted primal candidates. |
| `Primal.Tolerance.LinearConstraint` | Linear constraint feasibility when checked by SHOT. |
| `Primal.Tolerance.NonlinearConstraint` | Nonlinear feasibility of accepted primal candidates. |
| `Primal.Tolerance.TrustLinearConstraintValues` | Whether linear feasibility reported by the subsolver is trusted. |
| `Termination.ConstraintTolerance` | Constraint-deviation criterion for the current dual solution. |
| `Termination.ObjectiveConstraintTolerance` | Consistency of the nonlinear objective at the dual solution for constraint-based termination. |

Do not infer feasibility of an incumbent from only the `max.err.` value of a
dual row. See [the output guide](UnderstandingOutput.md#main-iteration-columns).
After a limit, inspect both [status and solution availability](concepts.md#model-status-and-termination).

## Output and diagnostics

For every progress row while keeping normal message severity:

```bash
SHOT problem.osil Output.Console.Iteration.Detail=0
```

For every primal NLP call as well as ordinary progress updates:

```bash
SHOT problem.osil Output.Console.Iteration.Detail=2
```

For a normal console with a detailed file log:

```bash
SHOT problem.osil --log detailed.log Output.Console.Iteration.Detail=0 \
  Output.Console.LogLevel=2 Output.File.LogLevel=1
```

Iteration filtering applies before both logging destinations. Set detail `0`
to preserve all rows in the file. To inspect the MIP or NLP solver itself, add
`Output.Console.DualSolver.Show=true` or `Output.Console.PrimalSolver.Show=true`.
The [output guide](UnderstandingOutput.md) distinguishes SHOT and subsolver logs.

To save intermediate models and points:

```bash
SHOT problem.osil --debug=shot-debug Output.Console.Iteration.Detail=0 \
  Termination.IterationLimit=50
```

See the [debug file reference](DebuggingSHOT.md#debug-file-reference) for what to
inspect. Debug mode and logging verbosity are separate controls.

## Repeatable comparisons

Use a Release build, the same solver versions, model, and settings, and run one
instance at a time. Set `Dual.MIP.NumberOfThreads=1` and, on Linux/macOS,
`OPENBLAS_NUM_THREADS=1` to avoid multithreaded BLAS in Ipopt/MUMPS. An iteration
limit is more reproducible than a time limit; time-limited runs depend on machine
load and speed. Record the version/revision and effective options with results.
