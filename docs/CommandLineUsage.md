# Running the SHOT Console Application

This describes how to invoke the standalone `SHOT` executable and its
command-line switches, derived from the actual CLI parsing in
`src/SHOT.cpp` (the `--help` output is generated directly from that code,
so running `./SHOT --help` on your build is always the authoritative,
build-specific answer — it also lists which problem formats *this*
compiled binary supports, see section 2).

For the debug-output mechanism specifically (`--debug`, intermediate
problem/solution files), see
[docs/DebuggingSHOT.md](DebuggingSHOT.md).

## 1. Basic usage

```bash
./SHOT PROBLEMFILE [ARGUMENTS] [OPTIONS]
```

`PROBLEMFILE` is the only required positional argument. Everything else is
optional. Run `./SHOT --help` for the built-in usage summary.

## 2. Supported problem file formats

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
`.nl` files; requires `HAS_AMPL`).

## 3. Shorthand flags

| Flag | Effect |
|---|---|
| `--convex` | Assumes the problem is convex (sets `Model.Convexity.AssumeConvex=true`) |
| `--debug` / `--debug=<directory>` | Enables debug-output mode; see [DebuggingSHOT.md](DebuggingSHOT.md) |
| `--mip=<cbc\|highs\|cplex\|gurobi>` | Selects the MIP (dual) solver — only the ones the build was compiled with are available |
| `--nlp=<ipopt\|gams\|shot>` | Selects the primal NLP solver |
| `--tree=<single\|multi>` | Selects single-tree (branch-and-cut) vs. multi-tree dual strategy — only meaningful for MIP solvers that support the required callbacks (currently Cplex/Gurobi builds) |
| `--threads=<n>` | Maximum number of threads (`Dual.MIP.NumberOfThreads`) |
| `--absgap=<value>` | Absolute objective gap termination tolerance (`Termination.ObjectiveGap.Absolute`) |
| `--relgap=<value>` | Relative objective gap termination tolerance (`Termination.ObjectiveGap.Relative`) |
| `--timelimit=<value>` | Time limit in seconds (`Termination.TimeLimit`) |

There is no shorthand flag for the iteration limit — set it directly as
`Termination.IterationLimit=<n>` (see section 4). It's often more useful
than `--timelimit` when you want a reproducible, machine-speed-independent
stopping point (e.g. while debugging).

## 4. Setting any option directly

Beyond the shorthand flags above, **any** SHOT setting can be set directly
as a positional `KEY=VALUE` argument (case-sensitive, no spaces around the
`=`):

```bash
./SHOT problemfile Termination.TimeLimit=100.0 Termination.ObjectiveGap.Absolute=0.1
```

SHOT matches `KEY` against its known setting identifiers (checked in order:
string, boolean, integer, enum, then double settings) and parses `VALUE`
accordingly — booleans must be literally `true`/`false`, enums accept
their integer value. Unrecognized keys are silently ignored, so typos in a
setting name will not produce an error — double check the name (e.g. via
`--docs`, section 6) if a `KEY=VALUE` argument doesn't seem to take effect.

Options can also be read from a file instead of (or in addition to) inline
arguments:

- `--opt [FILE]` — read options from `FILE` in SHOT's own (GAMS-like)
  format; if `FILE` is omitted, an existing `SHOT.opt` in the current
  directory is used, or a new template one is generated.
- `--osol [FILE]` — same idea but in OSoL (XML) format; defaults to
  `SHOT.osol`.

## 5. Output/result files

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

## 6. Full/current options reference

```bash
./SHOT --docs
```

Writes `options.md` to the current directory, generated live from the
solver's actual registered settings (name, type, default, description,
allowed enum values). This is the authoritative settings reference —
prefer it over guessing a setting name, since settings can be added,
renamed, or have their defaults changed between versions.

## 7. Quick reference

```bash
./SHOT --help
```

prints a build-specific summary of everything above (supported formats for
this build, all shorthand flags available given how it was compiled, and
the general `KEY=VALUE` mechanism).
