[![Build and Test](https://github.com/coin-or/SHOT/actions/workflows/build-workflow.yml/badge.svg)](https://github.com/coin-or/SHOT/actions/workflows/build-workflow.yml)
[![Documentation](https://github.com/coin-or/SHOT/actions/workflows/docs.yml/badge.svg)](https://shotsolver.dev/docs/)
[![PyPI](https://img.shields.io/pypi/v/SHOTpy)](https://pypi.org/project/SHOTpy/)
[![GitHub release](https://img.shields.io/github/v/release/coin-or/SHOT)](https://github.com/coin-or/SHOT/releases)
[![License](https://img.shields.io/badge/License-EPL%202.0-red.svg)](https://opensource.org/licenses/EPL-2.0)


# The Supporting Hyperplane Optimization Toolkit

<picture><source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/coin-or/SHOT/master/docs/site/_static/shot-logo-dark.png"><img align="right" width="200" alt="SHOT logo" src="https://raw.githubusercontent.com/coin-or/SHOT/master/docs/site/_static/shot-logo.png"></picture>*SHOT is a software for solving mathematical optimization problems of the mixed-integer nonlinear programming (MINLP) class. In addition to MINLP problems, SHOT can also be used for subclasses such as NLP and MI(QC)QP.*

SHOT is global for convex MINLP, and mainly a heuristic for nonconvex MINLP, where the bounds on the objective value are still guaranteed. A reformulation engine rewrites each problem into a form that suits the method, and some nonconvex problem types are reformulated into convex ones and solved to global optimality.

## Documentation

The project website [shotsolver.dev](https://shotsolver.dev) contains:

- [Getting started](https://shotsolver.dev/getting-started/): installation and examples for Python, GAMS, Pyomo, JuMP, AMPL, the command line and C++
- [Documentation](https://shotsolver.dev/docs/): the guides and references below
  - [SHOTpy tutorial](https://shotsolver.dev/docs/dev/SHOTpy_Tutorial.html): modeling, settings, file input and callbacks in Python
  - [Command-line usage](https://shotsolver.dev/docs/dev/CommandLineUsage.html): problem formats, options and use as an AMPL solver
  - [C++ integration](https://shotsolver.dev/docs/dev/cpp.html) and [callbacks](https://shotsolver.dev/docs/dev/Callbacks.html)
  - [Solver settings](https://shotsolver.dev/docs/dev/reference/settings.html): all settings with their defaults
  - [Installation](https://shotsolver.dev/docs/dev/CompilationInstructions.html): building from source and with proprietary solvers
  - [Debugging](https://shotsolver.dev/docs/dev/DebuggingSHOT.html): investigating solver behavior
- [Publications](https://shotsolver.dev/publications/) and [news](https://shotsolver.dev/news/)

## Installation

Install SHOTpy, built with the open-source solvers Cbc, Ipopt and HiGHS, from PyPI:

```bash
pip install SHOTpy
```

```python
import SHOTpy

solver = SHOTpy.Solver()
problem = SHOTpy.Problem(solver)

x = problem.addVariable("x", SHOTpy.VariableType.Real, 0.0, 10.0)
y = problem.addVariable("y", SHOTpy.VariableType.Integer, 0.0, 10.0)

problem.setObjective(SHOTpy.exp(x) + (y - 2.5)**2)
problem.addConstraint(x + y >= 3, "c")
problem.finalize()

solver.setProblem(problem)
solver.solveProblem()
print(solver.getModelReturnStatus(), solver.getPrimalSolution().point)
```

The package also installs the SHOT executable as the command `SHOT`, which
solves problem files and can be used as an AMPL solver, e.g., from Pyomo,
JuMP, or AMPL. To get only the command, install it as a tool with
`pipx install SHOTpy` or `uv tool install SHOTpy`:

```bash
SHOT problem.nl -AMPL
SHOT problem.osil Termination.TimeLimit=60
```

The same executable, with the same solvers, can also be downloaded without
Python from the [GitHub releases](https://github.com/coin-or/SHOT/releases) as an archive for
Linux x86_64 and ARM64, macOS ARM64 and Windows x64. Extract the whole
archive and run `./SHOT` (or `SHOT.exe` on Windows) in its directory.

The public wheels do not include CPLEX, Gurobi, GAMS or CONOPT. To use them,
build SHOTpy from source as described in
[SHOTpy with proprietary solvers](https://shotsolver.dev/docs/dev/CompilationInstructions.html#shotpy-with-proprietary-solvers).
SHOT builds on Linux, macOS, and natively on Windows x64 with MSVC.

## Using SHOT

**SHOT can be used**
- as a console application ([command-line usage](https://shotsolver.dev/docs/dev/CommandLineUsage.html)),
- from [GAMS](https://www.gams.com) — the easiest way to use SHOT is through [GAMSPy](https://gamspy.readthedocs.io),
- on the [NEOS Server](https://neos-server.org/neos/solvers/minco:SHOT/GAMS.html),
- from any modeling system that can use ASL, such as
  - [Pyomo/Python](https://www.pyomo.org) (see [Using SHOT from Pyomo](https://shotsolver.dev/docs/dev/CommandLineUsage.html#using-shot-from-pyomo)),
  - [JuMP/Julia](https://github.com/jump-dev/AmplNLWriter.jl), and
  - [AMPL](https://www.ampl.com),
- through its own Python API ([SHOTpy](https://shotsolver.dev/docs/dev/SHOTpy_Tutorial.html)), or
- using its API implemented in C++ ([C++ integration](https://shotsolver.dev/docs/dev/cpp.html)), both of which support [callbacks](https://shotsolver.dev/docs/dev/Callbacks.html) for customizing SHOT's behavior (e.g. injecting primal solutions, warm-starting, or adding custom cuts).

SHOT requires a MILP solver: [Cplex](https://www.ibm.com/products/ilog-cplex-optimization-studio), [Gurobi](https://www.gurobi.com), [Cbc](https://www.github.com/coin-or/Cbc) or [HiGHS](https://highs.dev). NLP subproblems can use [Ipopt](https://www.github.com/coin-or/Ipopt), [Uno](https://github.com/cvanaret/Uno), the optional [CONOPT](https://conopt.gams.com) C++ API, or SHOT's built-in NLP solver. If SHOT is interfaced with GAMS, any licensed NLP solver can be used.

SHOT is a [COIN-OR project](https://www.coin-or.org), and won the [COIN-OR Cup 2018](https://web.archive.org/web/20191206234504/https://www.coin-or.org/2018-coin-or-cup-winner/). Project manager is Andreas Lundell. A full list of contributors is available on the [project website](https://shotsolver.dev/team/).

## Dual bound through polyhedral (outer) approximation

SHOT is based on iteratively creating a tighter polyhedral approximation of the nonlinear feasible set by generating supporting hyperplanes or cutting planes. These linearized problems are then solved with a mixed-integer linear programming (MILP) solver such as CPLEX, Gurobi, Cbc or HiGHS. If CPLEX or Gurobi is used, the subproblems can also include quadratic and bilinear nonlinearities directly; then MIQP or MIQCQP subproblems are solved. The dual strategy can be run either as a multi-tree search (repeatedly re-solving the MIP relaxation) or, with CPLEX or Gurobi, as a single-tree (branch-and-cut) search that adds cuts via callbacks during one search tree; Cbc and HiGHS currently only support the multi-tree strategy.

The problem types that can be solved to global optimality depend on which MIP solver is used:

| Problem type | CPLEX | Gurobi | Cbc | HiGHS |
| --- | --- | --- | --- | --- |
| Convex MIQP | ✓* | ✓* | ✓ | ✓ |
| Convex MIQCQP | ✓* | ✓* | ✓ | ✓ |
| Convex MINLP | ✓ | ✓ | ✓ | ✓ |
| Nonconvex MIQP | ✓* | ✓* | ? | ? |
| Nonconvex MIQCQP | ? | ✓* | ? | ? |
| Nonconvex MINLP | ? | ? | ? | ? |

✓ = solved to global optimality, ? = no guarantee that a solution is found, * = the problem can be handled directly by the subsolver (native quadratic/bilinear support).

## Primal bound using heuristics

The solution to the outer approximation problem provides a lower (dual) bound (when solving a minimization problem) to the original problem if the problem is convex. If the problem is nonconvex, convergence to the global optimal solution cannot be guaranteed (but might be achieved for certain classes of problems, cf. [this paper](https://optimization-online.org/2020/03/7691/)).

To get an upper (primal) bound (when solving a minimization problem) on the optimal solution SHOT utilizes the following heuristics:
- Solving nonlinear programming (NLP) problems where the integer variables have been fixed to valid values. This is done by calling an external NLP solver (e.g. Ipopt).
- By checking solutions from the MIP solver's solution pool for points that fulfill also the nonlinearities in the original MINLP problem.
- By performing root searches. 

## Termination

When the relative or absolute difference (objective gap) between the primal and dual bounds is less than a user-specified value, SHOT terminates with the current primal solution. If the original problem is convex, this is a global solution to the problem. If it is nonconvex, there is normally no guarantee that such a solution can be found, however SHOT will always in addition to the primal solution give a valid lower bound on the solution. 

## Contributing

Bug reports and pull requests are welcome on [GitHub](https://github.com/coin-or/SHOT/issues). See [CONTRIBUTING.md](https://github.com/coin-or/SHOT/blob/master/CONTRIBUTING.md) for the conventions and [CHANGELOG.md](https://github.com/coin-or/SHOT/blob/master/CHANGELOG.md) for recent changes.

## Publications

**SHOT is best described in the paper:**

Lundell, A. Kronqvist, J. and Westerlund, T. *The supporting hyperplane optimization toolkit for convex MINLP*. Journal of Global Optimization (2022). https://link.springer.com/article/10.1007/s10898-022-01128-0

**The features for solving nonconvex MINLP problems are described in the papers:**

Lundell, A. and Kronqvist, J., *Polyhedral approximation strategies for nonconvex mixed-integer nonlinear programming in SHOT*. Journal of Global Optimization (2021). https://doi.org/10.1007/s10898-021-01006-1

Lundell, A. and Kronqvist, J. *On Solving Nonconvex MINLP Problems with SHOT* (2019). In: Le Thi H., Le H., Pham Dinh T. (editors) Optimization of Complex Systems: Theory, Models, Algorithms and Applications. WCGO 2019. Advances in Intelligent Systems and Computing, vol 991. Springer, Cham.

**Reformulation of nonseparable convex quadratic expressions (preprint):**

Blomqvist, J., Tamm, E., Kronqvist, J. and Lundell, A. *Decomposition-Based Reformulation of Nonseparable Quadratic Expressions in Convex MINLP*. Optimization Online (2026). https://optimization-online.org/?p=34367

All publications are listed on the [project website](https://shotsolver.dev/publications/).
