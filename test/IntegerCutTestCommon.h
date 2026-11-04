/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "../src/DualSolver.h"
#include "../src/Enums.h"
#include "../src/Environment.h"
#include "../src/Solver.h"
#include "../src/Structs.h"
#include "../src/Model/ObjectiveFunction.h"
#include "../src/Model/Problem.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

// Solves a problem whose fixed-integer NLP problems are infeasible, so that integer cuts are added, and checks that
// cuts were added and that the bounds are on the correct side of the optimal objective value. The problems may be
// nonconvex and are stopped by the time limit, so the optimum itself is not required.
static bool solveWithIntegerCuts(SHOT::ES_MIPSolver mipSolver, const std::string& filename, double optimalObjective)
{
    auto solver = std::make_unique<SHOT::Solver>();

    // The integer cuts are added in the first seconds; the time limit stops the problems that do not converge
    solver->updateSetting("Termination.TimeLimit", 10.0);
    // One thread, so that the solution path does not depend on the other tests that are run at the same time
    solver->updateSetting("Dual.MIP.NumberOfThreads", 1);
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(mipSolver));

    try
    {
        if(!solver->setProblem(filename))
            return false;
    }
    catch(SHOT::Exception& e)
    {
        std::cout << "Error: " << e.what() << '\n';
        return false;
    }

    solver->solveProblem();

    auto env = solver->getEnvironment();
    bool passed = true;

    if(env->dualSolver->generatedIntegerCuts.empty())
    {
        std::cout << filename << ": no integer cuts were added\n";
        passed = false;
    }

    // Without a primal solution, the primal bound is infinite and on the correct side
    double primalBound = solver->getPrimalBound();
    double dualBound = solver->getGlobalDualBound();
    double tolerance = 1e-4 * std::max(1.0, std::abs(optimalObjective));
    bool isMinimize = env->problem->objectiveFunction->properties.isMinimize;

    if(isMinimize ? primalBound < optimalObjective - tolerance : primalBound > optimalObjective + tolerance)
    {
        std::cout << filename << ": the primal bound " << primalBound << " is better than the optimum "
                  << optimalObjective << '\n';
        passed = false;
    }

    if(isMinimize ? dualBound > optimalObjective + tolerance : dualBound < optimalObjective - tolerance)
    {
        std::cout << filename << ": the dual bound " << dualBound << " is past the optimum " << optimalObjective
                  << '\n';
        passed = false;
    }

    return passed;
}
