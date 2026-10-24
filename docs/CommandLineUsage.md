# Running the SHOT Console Application

This describes how to invoke the standalone `SHOT` executable and its
command-line switches, derived from the actual CLI parsing in
`src/SHOT.cpp` (the `--help` output is generated directly from that code,
so running `./SHOT --help` on your build is always the authoritative,
build-specific answer — it also lists which problem formats *this*
compiled binary supports, see “Supported problem file formats”).

For the debug-output mechanism specifically (`--debug`, intermediate
problem/solution files), see
[docs/DebuggingSHOT.md](DebuggingSHOT.md).

## Basic usage

```bash
./SHOT PROBLEMFILE [ARGUMENTS] [OPTIONS]
```

`PROBLEMFILE` is the only required positional argument. Everything else is
optional. Run `./SHOT --help` for the built-in usage summary.

The SHOTpy pip package (`pip install SHOTpy`, or `pipx install SHOTpy` /
`uv tool install SHOTpy` for only the command) contains the executable, built
with Cbc, Ipopt and HiGHS, and installs it as the command `SHOT`, which takes
the same arguments. In Python, `SHOTpy_bin.executable()` gives the path of the
executable itself.

## Supported problem file formats

SHOT always supports:

- OSiL (`.osil` or `.xml`)

Support for the following is **only present if enabled at compile time**
(CMake options `HAS_AMPL` and `HAS_GAMS`, both `ON` by default, but a given
build/package may have either disabled):

- AMPL (`.nl`) — requires `HAS_AMPL`
- GAMS (`.gms`) — requires `HAS_GAMS`

`./SHOT --help` prints exactly which formats the binary you're running
supports (a "SHOT has been compiled with support for the following problem
formats" line). If a `.nl` or `.gms` file is rejected, check that first —
it usually means the build lacks that interface, not that the file is
malformed. OSiL is the safe fallback that's always available.

When using `.nl` files through the AMPL Solver Library calling convention,
pass `--AMPL` (or `-AMPL`) in addition to the problem file (only valid for
`.nl` files; requires `HAS_AMPL`). In this mode SHOT also reads options from
the environment variable `shot_options`, which AMPL sets with
`option shot_options '...';`. The options are given as `KEY=VALUE`
separated by whitespace, and a value with spaces can be put in quotes, e.g.
`option shot_options 'Termination.TimeLimit=60 Output.Debug.Path="my dir"';`.
Options on the command line take precedence over those in `shot_options`.

## Shorthand flags

| Flag | Effect |
|---|---|
| `--convex` | Assumes the problem is convex (sets `Model.Convexity.AssumeConvex=true`) |
| `--debug` / `--debug=<directory>` | Enables debug-output mode; see [DebuggingSHOT.md](DebuggingSHOT.md) |
| `--mip=<cbc\|highs\|cplex\|gurobi>` | Selects the MIP (dual) solver — only the ones the build was compiled with are available |
| `--nlp=<ipopt\|gams\|shot\|uno\|conopt>` | Selects the primal NLP solver |
| `--tree=<single\|multi>` | Selects single-tree (branch-and-cut) vs. multi-tree dual strategy — single-tree requires solver callback support, currently only available with CPLEX or Gurobi; Cbc and HiGHS are always forced to multi-tree |
| `--threads=<n>` | Maximum number of threads (`Dual.MIP.NumberOfThreads`) |
| `--absgap=<value>` | Absolute objective gap termination tolerance (`Termination.ObjectiveGap.Absolute`) |
| `--relgap=<value>` | Relative objective gap termination tolerance (`Termination.ObjectiveGap.Relative`) |
| `--timelimit=<value>` | Time limit in seconds (`Termination.TimeLimit`) |

There is no shorthand flag for the iteration limit — set it directly as
`Termination.IterationLimit=<n>` (see “Setting any option directly”). It's often more useful
than `--timelimit` when you want a reproducible, machine-speed-independent
stopping point (e.g. while debugging).

## Setting any option directly

Beyond the shorthand flags above, **any** SHOT setting can be set directly
as a positional `KEY=VALUE` argument (case-sensitive, no spaces around the
`=`):

```bash
./SHOT problemfile Termination.TimeLimit=100.0 Termination.ObjectiveGap.Absolute=0.1
```

SHOT matches `KEY` against its known setting identifiers (checked in order:
string, boolean, integer, enum, then double settings) and parses `VALUE`
accordingly — booleans are `true`/`false` in any case, or `1`/`0` (the
same holds in options files), enums accept their integer value. Unrecognized keys are ignored: they are listed under
“Unrecognized options” in the log, but do not produce an error — double
check the name (e.g. via `--docs`, “Full/current options reference”) if a
`KEY=VALUE` argument doesn't seem to take effect.

Options can also be read from a file instead of (or in addition to) inline
arguments:

- `--opt [FILE]` — read options from `FILE` in SHOT's own (GAMS-like)
  format; if `FILE` is omitted, an existing `SHOT.opt` in the current
  directory is used, or a new template one is generated.
- `--osol [FILE]` — same idea but in OSoL (XML) format; defaults to
  `SHOT.osol`.

## Output/result files

- `--log FILE` — sets the log file name (default: `SHOT.log` in the
  current directory). See [DebuggingSHOT.md](DebuggingSHOT.md) for the
  `Output.Console.LogLevel`/`Output.File.LogLevel` settings that control
  verbosity.
- The solution result is always written in OSrL format, to
  `--osrl FILE` if given, otherwise to
  `<Output.ResultPath>/<problem name>.osrl`.
- `--trc [FILE]` — additionally writes a trace file (default:
  `<Output.ResultPath>/<problem name>.trc`).
- `--sol [FILE]` — additionally writes an AMPL `.sol` file (default:
  `<problemfile with .sol extension>`); written automatically when running
  in `--AMPL` mode.

## Exit codes

The exit code tells whether the run itself succeeded. It does not give the
model status: an infeasible or unbounded problem, or a run stopped by a
limit, still exits with 0, and the status is written to the result files.

| Code | Meaning |
|------|---------|
| 0 | The problem was solved (whatever the model status), or `--help`/`--docs` was run |
| 1 | Invalid command line, e.g. no problem file given, or `--AMPL` with a problem that is not an AMPL (`.nl`) file |
| 2 | The problem file or the options file given by `--opt`/`--osol` cannot be found or read |
| 3 | An error occurred while solving the problem, e.g. a callback failed |
| 4 | A result file (OSrL, trace, `.sol`), the options file or `options.md` could not be written |

## Using SHOT from Pyomo

[Pyomo](https://www.pyomo.org) calls SHOT through the AMPL Solver Library
interface: it writes the model to an `.nl` file, runs the `SHOT` executable
in `-AMPL` mode and reads the solution back from the `.sol` file. This needs
a `SHOT` executable built with `HAS_AMPL` (see “Supported problem file
formats”). The SHOTpy pip package contains such an executable (see “Basic
usage”); otherwise build SHOT from source as described in the
[compilation instructions](CompilationInstructions.md).

Pyomo has no built-in `shot` solver, so use its generic `asl:` interface and
give the path to the executable, unless `SHOT` is on your `PATH`. With the
pip package, `executable=SHOTpy_bin.executable()` works in any environment:

```python
import pyomo.environ as pyo
from pyomo.opt import TerminationCondition

m = pyo.ConcreteModel()
m.x = pyo.Var(domain=pyo.Integers, bounds=(0, 5))
m.y = pyo.Var(bounds=(0, 5))
m.obj = pyo.Objective(expr=(m.x - 2.3)**2 + (m.y - 1.7)**2)
m.c = pyo.Constraint(expr=m.x**2 + m.y**2 <= 9)

opt = pyo.SolverFactory('asl:shot', executable='/path/to/SHOT')
opt.options['Termination.TimeLimit'] = 60
opt.options['Dual.MIP.NumberOfThreads'] = 1
opt.options['Primal.FixedInteger.Use'] = True

results = opt.solve(m, tee=True)

if results.solver.termination_condition == TerminationCondition.optimal:
    print(pyo.value(m.obj), m.x.value, m.y.value)
```

**Options.** Each entry in `opt.options` is passed to SHOT both as a
`KEY=VALUE` argument and in the environment variable `shot_options`, so the
rules in “Setting any option directly” apply. Use the full, case-sensitive
setting names. Booleans can be given as Python's `True`/`False`, and enum
settings take their integer value. With `tee=True`, SHOT's log lists the
options it received under “Non-default options”, marked `[user]`, and any
names it does not recognize under “Unrecognized options”.

**Results.** The solution values are loaded into the model, and
`results.solver.message` holds SHOT's solve message. Its first part is the
status, e.g. `SHOT 2.0.0: Solved to global optimality`. The rest names the
problem types and the MIP and NLP solvers used. Pyomo writes `:` in the
message as `\x3a`.

Pyomo turns SHOT's AMPL status code into a termination condition by code
range, so different outcomes can map to the same condition. The exact code is
available in `results.solver.id`:

| Code | SHOT status | Pyomo termination condition / solver status |
|------|-------------|---------------------------------------------|
| 0 | Solved to global optimality | `optimal` / `ok` |
| 100 | Solved to local optimality | `optimal` / `warning` |
| 101 | Iteration limit reached, feasible solution found | `optimal` / `warning` |
| 102 | User abort, feasible solution found | `optimal` / `warning` |
| 103 | Error occurred, feasible solution found | `optimal` / `warning` |
| 104 | Feasible solution found, optimality not guaranteed (e.g. nonconvex problems) | `optimal` / `warning` |
| 200 | No solution found since dual problem is infeasible | `infeasible` / `warning` |
| 300 | No solution found since dual problem is unbounded | `unbounded` / `warning` |
| 400 | Time limit, feasible solution found; or a limit was reached with no solution | `maxIterations` / `warning` |
| 500 | No solution found since an error occurred | `internalSolverError` / `error` |
| 600 | No solution found due to user abort | `unknown` / `ok` |

So `optimal` only means a global optimum when the solver status is `ok`
(code 0). A time limit is reported as `maxIterations`. When SHOT has no
solution it returns no variable values, so the variables in the model keep
the values they had before the solve. Check the termination condition before
using the variable values.

## Full/current options reference

```bash
./SHOT --docs
```

Writes `options.md` to the current directory, generated live from the
solver's actual registered settings (name, type, default, description,
allowed enum values). This is the authoritative settings reference —
prefer it over guessing a setting name, since settings can be added,
renamed, or have their defaults changed between versions.

## Quick reference

```bash
./SHOT --help
```

prints a build-specific summary of everything above (supported formats for
this build, all shorthand flags available given how it was compiled, and
the general `KEY=VALUE` mechanism).
