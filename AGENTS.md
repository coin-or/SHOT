# SHOT

SHOT is an open-source MINLP solver written in C++ (COIN-OR project). See
`README.md` and `docs/` for additional documentation.

## Building SHOT

SHOT compiles on Linux and macOS. Windows is supported through Windows Subsystem 
for Linux (WSL). For cloning/submodules, dependency installation 
(Cbc/Ipopt/HiGHS/GAMS/CPLEX/Gurobi), CMake options, and building, see
[docs/CompilationInstructions.md](docs/CompilationInstructions.md).

## Running SHOT

```bash
./SHOT problemfile [ARGUMENTS] [OPTIONS]
```

For the full set of command-line switches (including which problem file
formats a given build supports), see
[docs/CommandLineUsage.md](docs/CommandLineUsage.md).

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
