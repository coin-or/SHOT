# SHOT

SHOT is an open-source MINLP solver written in C++ (COIN-OR project). See
`README.md` and `docs/` for additional documentation.

## Building SHOT

SHOT compiles on Linux and macOS, and natively on Windows x64 with MSVC.
WSL remains an alternative on Windows. For cloning/submodules, dependency installation
(Cbc/Ipopt/HiGHS/GAMS/CPLEX/Gurobi), CMake options, and building, see
[docs/CompilationInstructions.md](docs/CompilationInstructions.md).

## Running SHOT

```bash
./SHOT problemfile [ARGUMENTS] [OPTIONS]
```

For the full set of command-line switches (including which problem file
formats a given build supports), see
[docs/CommandLineUsage.md](docs/CommandLineUsage.md).

## Callbacks

SHOT calls user callbacks (C++ and Python) at locations in the solution
process given by `E_CallbackLocation`. Each location has its own context class
in `src/Callback.h`. For the locations, the rules for actions, termination and
failures, and the checklist for adding a location, see
[docs/Callbacks.md](docs/Callbacks.md).

## Debugging SHOT

To debug solver behavior (wrong bounds, missing solutions, unexpected
reformulations, stalled convergence), run with the built-in debug-output
mechanism and verbose console logging:

```bash
./SHOT problemfile --debug=<directory> Output.Console.Iteration.Detail=0
```

This writes per-iteration intermediate problems and solution points to
`<directory>`. For the full file reference and a step-by-step debugging
workflow — including how to use the automated test suite (`test_runner`,
`InstanceTest`) to reproduce/isolate a regression — see
[docs/DebuggingSHOT.md](docs/DebuggingSHOT.md).

## Code and tests

- Do not name test instances (MINLPLib, QPLIB, ...) in code comments;
  describe the case in general terms, e.g., "a square of a variable with a
  negative lower bound". Instance names belong in commit messages, PRs,
  docs and tests.
- Settings names are case-sensitive. An unknown setting is an exception
  in the C++ and Python API, an error that stops SHOT in an options file,
  and a warning on the command line. Keep this rather than matching names
  case-insensitively or having the API return a status.
- When testing a reformulation, use at least two terms of the reformulated
  kind that share variables, so that merging terms and sharing auxiliary
  variables are tested too.

## Benchmarking

Compare performance on a Release build with `Dual.MIP.NumberOfThreads=1`
and `OPENBLAS_NUM_THREADS=1` (Ipopt's and MUMPS's OpenBLAS otherwise uses
all cores), and run the instances one at a time. Runs limited by time are
not reproducible; compare with the same limits, solver versions and
hardware.

## Contributing

Follow [CONTRIBUTING.md](CONTRIBUTING.md) for branch names, PR titles and
validation. Ask for explicit approval before each commit or amend. An
instruction to implement a change is not approval to merge, push tags or
publish packages.
