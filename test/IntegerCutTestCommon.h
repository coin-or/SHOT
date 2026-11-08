/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once

#include "../src/DualSolver.h"
#include "../src/Callback.h"
#include "../src/Enums.h"
#include "../src/Environment.h"
#include "../src/Solver.h"
#include "../src/Structs.h"
#include "../src/Iteration.h"
#include "../src/Results.h"
#include "../src/MIPSolver/IMIPSolver.h"
#include "../src/Tasks/TaskAddIntegerCuts.h"
#include "../src/Model/ObjectiveFunction.h"
#include "../src/Model/Problem.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

// Exercise single-tree callbacks after nonbinary integer cuts add variables between solves, without depending on
// which fixed-integer NLP points a solver happens to try. The excluded interior values are known to be nonoptimal.
inline bool integerCutsBetweenSingleTreeSolves(SHOT::ES_MIPSolver mipSolver)
{
    using namespace SHOT;
    auto solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(mipSolver));
    solver->updateSetting("Dual.MIP.NumberOfThreads", 1);
    solver->updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::SingleTree));
    solver->updateSetting("Dual.HyperplaneCuts.UseIntegerCuts", true);
    solver->updateSetting("Dual.HyperplaneCuts.Delay", false);
    solver->updateSetting("Primal.FixedInteger.Use", false);
    solver->updateSetting("Model.BoundTightening.FeasibilityBased.Use", false);
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));

    // An external-cut callback selects the single-tree integration for this otherwise complete MILP.
    solver->registerCallback<HyperplaneSelectionContext>([](HyperplaneSelectionContext&) { });

    // min x - z, x >= z/2, x in [0,3], integer z in [0,4]: optimum -2 at (2,4).
    auto problem = std::make_shared<Problem>(env);
    auto x = std::make_shared<Variable>("x", E_VariableType::Real, 0.0, 3.0);
    auto z = std::make_shared<Variable>("z", E_VariableType::Integer, 0.0, 4.0);
    problem->add({ x, z });
    auto constraint = std::make_shared<LinearConstraint>("coupling", 0.0, SHOT_DBL_MAX);
    constraint->add(std::make_shared<LinearTerm>(1.0, x));
    constraint->add(std::make_shared<LinearTerm>(-0.5, z));
    problem->add(constraint);
    auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    objective->add(std::make_shared<LinearTerm>(1.0, x));
    objective->add(std::make_shared<LinearTerm>(-1.0, z));
    problem->add(objective);
    problem->finalize();
    if(!solver->setProblem(problem))
        return false;

    int callbackCalls = 0;
    solver->registerCallback<PrimalCandidateSearchContext>([&](PrimalCandidateSearchContext&) { ++callbackCalls; });
    // Keep SHOT's gap open so each native solve reaches its candidate callbacks. Native incumbents are unaffected.
    solver->registerCallback<PrimalCandidateCheckContext>(
        [](PrimalCandidateCheckContext& candidate) { candidate.rejectCandidate(); });

    auto backend = env->dualSolver->MIPSolver;
    env->results->createIteration();
    for(int phase = 0; phase < 3; ++phase)
    {
        if(phase > 0)
        {
            IntegerCut cut;
            cut.variableIndexes = { z->getIndex() };
            cut.variableValues = { phase + 1 };
            env->dualSolver->addIntegerCut(cut);
            TaskAddIntegerCuts(env).run();
        }

        if(backend->getNumberOfVariables() != 2 + 2 * phase || env->solutionStatistics.numberOfIntegerCuts != phase)
        {
            std::cout << "The nonbinary integer cut did not add two auxiliary variables\n";
            return false;
        }

        int callsBefore = callbackCalls;
        backend->setTimeLimit(5.0);
        backend->setSolutionLimit(2100000000);
        auto status = backend->solveProblem();
        if(status != E_ProblemSolutionStatus::Optimal || callbackCalls == callsBefore
            || std::abs(backend->getObjectiveValue() + 2.0) > 1e-6
            || std::abs(backend->getDualObjectiveValue() + 2.0) > 1e-6)
        {
            std::cout << "Single-tree solve " << phase << " failed after adding integer-cut variables; status "
                      << static_cast<int>(status) << ", candidate callbacks " << callbackCalls - callsBefore << '\n';
            return false;
        }

        auto point = backend->getVariableSolution(0);
        if(point.size() != static_cast<size_t>(2 + 2 * phase)
            || !env->problem->areNumericConstraintsFulfilled(point, 1e-6)
            || !env->problem->areVariableBoundsFulfilled(point, 1e-6)
            || !env->problem->areIntegralityConstraintsFulfilled(point, 1e-6)
            || std::abs(point[z->getIndex()] - 4.0) > 1e-6)
            return false;
    }
    return true;
}

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
