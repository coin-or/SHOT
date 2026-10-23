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

Use [callbacks](Callbacks.md) to monitor progress, request termination, or supply information at supported locations. Contexts are only valid during the callback; copy values needed after it returns. Register and remove callbacks outside an active solve.

See the [C++ reference](reference/cpp.rst) for the solver, settings, results, and callback interfaces.
