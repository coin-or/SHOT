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
    std::cout << "Model status: " << static_cast<int>(solver.getModelReturnStatus()) << '\n';
    std::cout << "Termination reason: " << static_cast<int>(solver.getTerminationReason()) << '\n';
    std::cout << "Global dual bound: " << solver.getGlobalDualBound() << '\n';
    if(solver.hasPrimalSolution())
    {
        auto solution = solver.getPrimalSolution();
        std::cout << "Feasible objective: " << solution.objValue << '\n';
        for(double value : solution.point)
            std::cout << value << ' ';
        std::cout << '\n';
    }
}
```

The input formats available depend on your build. A successful solver call does not by itself establish global optimality; inspect the model status, available primal solutions, and bounds.

The statuses use `E_ModelReturnStatus` and `E_TerminationReason`. Compare with
`E_ModelReturnStatus::OptimalGlobal` if a proof within objective-gap tolerances
is required. See [solver results](concepts.md) for the status meanings and
[Understanding SHOT output](UnderstandingOutput.md) for the terminal report.

## Build a model in C++

This complete alternative `main.cpp` minimizes `x*x + y*y` with `x + y >= 3`,
where `x` is real and `y` is integer. The optimum is `5`, at `(1, 2)` or `(2, 1)`.

```cpp
#include "Solver.h"
#include "Model/Problem.h"
#include "Model/Variables.h"
#include "Model/Terms.h"
#include "Model/ObjectiveFunction.h"
#include "Model/Constraints.h"
#include <iostream>
#include <memory>

int main()
{
    SHOT::Solver solver;
    solver.updateSetting("Termination.TimeLimit", 60.0);
    auto problem = std::make_shared<SHOT::Problem>(solver.getEnvironment());
    auto x = std::make_shared<SHOT::Variable>("x", SHOT::E_VariableType::Real, 0.0, 10.0);
    auto y = std::make_shared<SHOT::Variable>("y", SHOT::E_VariableType::Integer, 0.0, 10.0);
    problem->add(x);
    problem->add(y);

    auto objective = std::make_shared<SHOT::QuadraticObjectiveFunction>(
        SHOT::E_ObjectiveFunctionDirection::Minimize);
    objective->add(std::make_shared<SHOT::QuadraticTerm>(1.0, x, x));
    objective->add(std::make_shared<SHOT::QuadraticTerm>(1.0, y, y));
    problem->add(objective);

    auto demand = std::make_shared<SHOT::LinearConstraint>("demand", 3.0, SHOT::SHOT_DBL_MAX);
    demand->add(std::make_shared<SHOT::LinearTerm>(1.0, x));
    demand->add(std::make_shared<SHOT::LinearTerm>(1.0, y));
    problem->add(demand);
    problem->finalize();

    if(!solver.setProblem(problem) || !solver.solveProblem())
        return 1;
    if(!solver.hasPrimalSolution())
        return 2;
    auto solution = solver.getPrimalSolution();
    std::cout << "Feasible objective: " << solution.objValue << '\n';
    std::cout << "Global dual bound: " << solver.getGlobalDualBound() << '\n';
    return solver.getModelReturnStatus() == SHOT::E_ModelReturnStatus::OptimalGlobal ? 0 : 3;
}
```

Set options before constructing/finalizing the model and calling `setProblem()`;
see [configuration](Configuration.md). Public solution points follow the
original variable order, here `x`, then `y`.

## Compile an application with CMake

Install a built SHOT checkout, including development files, into a prefix you
control:

```bash
cmake --install build --prefix /absolute/path/to/shot-install
```

In a separate application directory, save either complete example above as
`main.cpp` and create `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(shot_example LANGUAGES CXX)
find_package(SHOT CONFIG REQUIRED)
add_executable(shot_example main.cpp)
target_link_libraries(shot_example PRIVATE SHOT::SHOTSolver)
```

Then configure and build the application:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/absolute/path/to/shot-install
cmake --build build --config Release
```

Run `./build/shot_example problem.osil` for the file-input example, or
`./build/shot_example` for the model-building example. Multi-configuration
generators put the executable under the selected configuration, for example
`build/Release/shot_example.exe` on Windows.

The imported target supplies include directories and C++17. Headers are installed
under `include/SHOT`; both `"Solver.h"` and `<SHOT/Solver.h>` work through the
target. The SHOT shared library and its backend runtime dependencies must be
available when running the application. In particular, Windows needs the DLL
directory on `PATH` or the required DLLs beside the executable. Use the same
toolchain and build configuration as the installed SHOT package.

An external backend's shared libraries may need their own runtime search path,
even when the application links successfully. On Linux, add that backend's
library directory to `LD_LIBRARY_PATH`; on macOS, use `DYLD_LIBRARY_PATH` when
needed. For example, an optional Uno installation outside the system library
paths may require its `lib` directory. A loader error naming a missing library
occurs before SHOT can start; changing the selected solver does not resolve it.

## Give a starting point

Call `setStartingPoint()` after `setProblem()` with a value for each variable of the original problem, in its order. If the point is feasible, it is the first primal solution, which gives the cutoff and the MIP start of the dual problems. If it is not feasible, an NLP problem with the discrete variables fixed is solved from it before the first dual problem instead, which may repair it (`Primal.FixedInteger.UseStartingPoint`, on by default). `setProblem()` replaces the starting point with the one in the problem file (the initial values in an AMPL `.nl` file or the levels of the variables in GAMS), so a point given afterwards takes precedence. `getStartingPoint()` returns the point that will be used.

```cpp
solver.setProblem(problem);
solver.setStartingPoint(SHOT::VectorDouble { 0.0, 1.5, 1.0 });
solver.solveProblem();
```

Use [callbacks](Callbacks.md) to monitor progress, request termination, or supply information at supported locations. Contexts are only valid during the callback; copy values needed after it returns. Register and remove callbacks outside an active solve.

See the [C++ reference](reference/cpp.rst) for the solver, settings, results, and callback interfaces.
