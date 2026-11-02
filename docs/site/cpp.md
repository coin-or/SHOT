# Integrating SHOT with C++

Build SHOT and its dependencies using the [compilation instructions](CompilationInstructions.md). Compile your application against the same headers, libraries, and solver configuration as that build.

## Solve a problem file

```cpp
#include "Solver.h"
#include <iostream>

int main(int argc, char** argv)
{
    if(argc != 2)
        return 1;
    SHOT::Solver solver;
    if(!solver.setProblem(argv[1]))
        return 2;
    if(!solver.solveProblem())
        return 3;
    std::cout << "Solve finished; inspect model status and results.\n";
}
```

The input formats available depend on your build. A successful solver call does not by itself establish global optimality; inspect the model status, available primal solutions, and bounds.

## Give a starting point

Call `setStartingPoint()` after `setProblem()` with a value for each variable of the original problem, in its order. If the point is feasible, it is the first primal solution, which gives the cutoff and the MIP start of the dual problems. With `Primal.FixedInteger.UseStartingPoint=true`, an NLP problem with the discrete variables fixed is also solved from it before the first dual problem. `setProblem()` replaces the starting point with the one in the problem file (the initial values in an AMPL `.nl` file or the levels of the variables in GAMS), so a point given afterwards takes precedence. `getStartingPoint()` returns the point that will be used.

```cpp
solver.setProblem(problem);
solver.setStartingPoint(SHOT::VectorDouble { 0.0, 1.5, 1.0 });
solver.solveProblem();
```

Use [callbacks](Callbacks.md) to monitor progress, request termination, or supply information at supported locations. Contexts are only valid during the callback; copy values needed after it returns. Register and remove callbacks outside an active solve.

See the [C++ reference](reference/cpp.rst) for the solver, settings, results, and callback interfaces.
