/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

/* Regression tests for the validity of dual bounds reported by the MIP backends
   and by the convex bounding task. A dual bound that does not come from a
   successfully solved relaxation may cut off the optimum of the original
   problem, as happened for the MINLPLib instance uselinear. */

#include "../src/Solver.h"
#include "../src/DualSolver.h"
#include "../src/Environment.h"
#include "../src/Iteration.h"
#include "../src/Results.h"
#include "../src/Settings.h"
#include "../src/Structs.h"

#include "../src/Model/Constraints.h"
#include "../src/Model/ObjectiveFunction.h"
#include "../src/Model/Problem.h"
#include "../src/Model/Terms.h"
#include "../src/Model/Variables.h"

#include "../src/MIPSolver/IMIPSolver.h"
#include "../src/Tasks/TaskCreateMIPProblem.h"
#include "../src/Tasks/TaskPerformConvexBounding.h"

#ifdef HAS_CBC
#include "../src/MIPSolver/MIPSolverCbc.h"
#endif
#ifdef HAS_CPLEX
#include "../src/MIPSolver/MIPSolverCplex.h"
#endif
#ifdef HAS_GUROBI
#include "../src/MIPSolver/MIPSolverGurobi.h"
#endif
#ifdef HAS_HIGHS
#include "../src/MIPSolver/MIPSolverHighs.h"
#endif

#include <cmath>
#include <iostream>

using namespace SHOT;

namespace
{

enum class ModelKind
{
    Bounded,
    Unbounded,
    Infeasible
};

// Builds a small pure-MIP model with a known optimum of -2.75 (minimization),
// or a variant that is unbounded or infeasible instead.
std::shared_ptr<Solver> makeSolver(ES_MIPSolver mipSolver, ModelKind kind, bool minimize)
{
    auto solver = std::make_shared<Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(mipSolver));
    solver->updateSetting("Dual.MIP.NumberOfThreads", 1);
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));

    double unbounded = (kind == ModelKind::Unbounded) ? 1e50 : 3.0;

    auto problem = std::make_shared<Problem>(env);
    auto x = std::make_shared<Variable>("x", E_VariableType::Real, -unbounded, unbounded);
    auto b = std::make_shared<Variable>("b", E_VariableType::Binary, 0.0, 1.0);
    problem->add({ x, b });

    if(kind == ModelKind::Infeasible)
    {
        // x >= 1 together with x <= -1 has no feasible point
        auto c1 = std::make_shared<LinearConstraint>("c1", 1.0, SHOT_DBL_MAX);
        c1->add(std::make_shared<LinearTerm>(1.0, x));
        auto c2 = std::make_shared<LinearConstraint>("c2", SHOT_DBL_MIN, -1.0);
        c2->add(std::make_shared<LinearTerm>(1.0, x));
        problem->add(c1);
        problem->add(c2);
    }

    auto objective = std::make_shared<LinearObjectiveFunction>(
        minimize ? E_ObjectiveFunctionDirection::Minimize : E_ObjectiveFunctionDirection::Maximize);
    objective->add(std::make_shared<LinearTerm>(minimize ? 1.0 : -1.0, x));
    objective->add(std::make_shared<LinearTerm>(minimize ? -0.75 : 0.75, b));
    objective->constant = 0.0;
    problem->add(objective);
    problem->finalize();

    if(!solver->setProblem(problem))
        return nullptr;

    env->results->usedMIPSolver = mipSolver;
    return solver;
}

MIPSolverPtr makeBackend(EnvironmentPtr env, ES_MIPSolver mipSolver)
{
    MIPSolverPtr backend;

    switch(mipSolver)
    {
#ifdef HAS_CBC
    case ES_MIPSolver::Cbc:
        backend = std::make_shared<MIPSolverCbc>(env);
        break;
#endif
#ifdef HAS_CPLEX
    case ES_MIPSolver::Cplex:
        backend = std::make_shared<MIPSolverCplex>(env);
        break;
#endif
#ifdef HAS_GUROBI
    case ES_MIPSolver::Gurobi:
        backend = std::make_shared<MIPSolverGurobi>(env);
        break;
#endif
#ifdef HAS_HIGHS
    case ES_MIPSolver::Highs:
        backend = std::make_shared<MIPSolverHighs>(env);
        break;
#endif
    default:
        break;
    }

    return backend;
}

std::string name(ES_MIPSolver mipSolver)
{
    switch(mipSolver)
    {
    case ES_MIPSolver::Cbc:
        return "Cbc";
    case ES_MIPSolver::Cplex:
        return "Cplex";
    case ES_MIPSolver::Gurobi:
        return "Gurobi";
    case ES_MIPSolver::Highs:
        return "Highs";
    default:
        return "unknown";
    }
}

// A backend must not report a dual bound for a relaxation it did not solve.
bool testUnsuccessfulSolveGivesNoBound(ES_MIPSolver mipSolver, ModelKind kind)
{
    bool passed = true;

    for(bool minimize : { true, false })
    {
        auto solver = makeSolver(mipSolver, kind, minimize);

        if(!solver)
        {
            std::cout << "Could not create problem for " << name(mipSolver) << '\n';
            return false;
        }

        auto env = solver->getEnvironment();
        auto backend = makeBackend(env, mipSolver);

        if(!backend->initializeProblem())
        {
            std::cout << "Could not initialize " << name(mipSolver) << '\n';
            return false;
        }

        TaskCreateMIPProblem(env, backend, env->reformulatedProblem).run();

        auto status = backend->solveProblem();
        double bound = backend->getDualObjectiveValue();
        double noBound = minimize ? SHOT_DBL_MIN : SHOT_DBL_MAX;

        if(status == E_ProblemSolutionStatus::Optimal || status == E_ProblemSolutionStatus::Feasible)
        {
            std::cout << name(mipSolver) << ": expected an unsuccessful solve, got status "
                      << static_cast<int>(status) << '\n';
            passed = false;
        }
        else if(bound != noBound)
        {
            std::cout << name(mipSolver) << (minimize ? " (min)" : " (max)")
                      << ": reported dual bound " << bound << " for a relaxation with status "
                      << static_cast<int>(status) << ", expected no bound\n";
            passed = false;
        }
    }

    return passed;
}

// Convex bounding must not publish a global dual bound from a bounding problem
// that the backend could not solve, or that it silently modified to make
// bounded. It must also leave the main iteration's repair state alone.
bool testConvexBoundingRejectsUnboundedModel(ES_MIPSolver mipSolver)
{
    bool passed = true;

    for(bool minimize : { true, false })
    {
        auto solver = makeSolver(mipSolver, ModelKind::Unbounded, minimize);

        if(!solver)
        {
            std::cout << "Could not create problem for " << name(mipSolver) << '\n';
            return false;
        }

        auto env = solver->getEnvironment();

        // Satisfy the preconditions of the task without running a full solve
        env->solutionStatistics.numberOfHyperplanesWithConvexSource = 1;
        env->solutionStatistics.numberOfHyperplanesWithNonconvexSource = 1;
        solver->updateSetting("Dual.ConvexBounding.IdleIterations", 0);
        env->results->createIteration();

        double dualBoundBefore = env->results->getGlobalDualBound();

        TaskPerformConvexBounding(env).run();

        double dualBoundAfter = env->results->getGlobalDualBound();

        if(dualBoundAfter != dualBoundBefore)
        {
            std::cout << name(mipSolver) << (minimize ? " (min)" : " (max)")
                      << ": convex bounding published dual bound " << dualBoundAfter
                      << " from an unbounded bounding problem (was " << dualBoundBefore << ")\n";
            passed = false;
        }

        if(env->results->getCurrentIteration()->hasInfeasibilityRepairBeenPerformed)
        {
            std::cout << name(mipSolver) << (minimize ? " (min)" : " (max)")
                      << ": convex bounding changed the main iteration's repair state\n";
            passed = false;
        }

        if(env->results->hasPrimalSolution())
        {
            std::cout << name(mipSolver) << (minimize ? " (min)" : " (max)")
                      << ": convex bounding accepted a primal solution from an unbounded bounding problem\n";
            passed = false;
        }
    }

    return passed;
}

std::vector<ES_MIPSolver> compiledSolvers()
{
    std::vector<ES_MIPSolver> solvers;
#ifdef HAS_CBC
    solvers.push_back(ES_MIPSolver::Cbc);
#endif
#ifdef HAS_CPLEX
    solvers.push_back(ES_MIPSolver::Cplex);
#endif
#ifdef HAS_GUROBI
    solvers.push_back(ES_MIPSolver::Gurobi);
#endif
#ifdef HAS_HIGHS
    solvers.push_back(ES_MIPSolver::Highs);
#endif
    return solvers;
}

}

int DualBoundTest(int argc, char* argv[])
{
    int choice = 1;

    if(argc > 1 && sscanf(argv[1], "%d", &choice) != 1)
    {
        std::cout << "Couldn't parse that input as a number\n";
        return -1;
    }

    bool passed = true;

    switch(choice)
    {
    case 1:
        std::cout << "Starting test that an infeasible dual problem gives no dual bound:\n";
        for(auto mipSolver : compiledSolvers())
            passed = testUnsuccessfulSolveGivesNoBound(mipSolver, ModelKind::Infeasible) && passed;
        std::cout << "Finished test that an infeasible dual problem gives no dual bound.\n";
        break;
    case 2:
        std::cout << "Starting test that an unbounded dual problem gives no dual bound:\n";
        for(auto mipSolver : compiledSolvers())
            passed = testUnsuccessfulSolveGivesNoBound(mipSolver, ModelKind::Unbounded) && passed;
        std::cout << "Finished test that an unbounded dual problem gives no dual bound.\n";
        break;
    case 3:
        std::cout << "Starting test that convex bounding rejects an unbounded bounding problem:\n";
        for(auto mipSolver : compiledSolvers())
            passed = testConvexBoundingRejectsUnboundedModel(mipSolver) && passed;
        std::cout << "Finished test that convex bounding rejects an unbounded bounding problem.\n";
        break;
    default:
        passed = false;
        std::cout << "Test #" << choice << " does not exist!\n";
    }

    return passed ? 0 : -1;
}
