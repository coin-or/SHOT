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
#include "../src/DualSolver.h"
#include "../src/Utilities.h"

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

// A hyperplane must be detected as already added when it is generated again in (almost) the same point for the
// same constraint, but not for another constraint or point, or for an objective cut with another objective value.
bool testDuplicateHyperplanesAreDetected(ES_MIPSolver mipSolver)
{
    // Only the two linear constraints of the model are needed, the problem is never solved
    auto solver = makeSolver(mipSolver, ModelKind::Infeasible, true);

    if(!solver)
    {
        std::cout << "Could not create problem for " << name(mipSolver) << '\n';
        return false;
    }

    auto env = solver->getEnvironment();

    // Duplicates are not checked in single-tree mode, since lazy constraints are not always added
    solver->updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::MultiTree));
    env->results->createIteration();

    auto& constraints = env->reformulatedProblem->numericConstraints;

    if(constraints.size() < 2)
    {
        std::cout << name(mipSolver) << ": expected two constraints in the reformulated problem, got "
                  << constraints.size() << '\n';
        return false;
    }

    bool passed = true;

    auto check = [&](bool condition, const std::string& description)
    {
        if(!condition)
        {
            std::cout << name(mipSolver) << ": " << description << '\n';
            passed = false;
        }
    };

    auto dualSolver = env->dualSolver;
    int firstIndex = constraints[0]->getIndex();
    int secondIndex = constraints[1]->getIndex();

    VectorDouble point(env->reformulatedProblem->properties.numberOfVariables, 1.0);
    VectorDouble otherPoint = point;
    otherPoint[0] = 2.0;

    auto createConstraintHyperplane = [&](NumericConstraintPtr constraint)
    {
        auto hyperplane = std::make_shared<ConstraintHyperplane>();
        hyperplane->source = E_HyperplaneSource::External;
        hyperplane->sourceConstraint = constraint;
        hyperplane->generatedPoint = point;
        hyperplane->isGlobal = true;
        return hyperplane;
    };

    auto createObjectiveHyperplane = [&](double objectiveValue)
    {
        auto hyperplane = std::make_shared<ObjectiveHyperplane>();
        hyperplane->source = E_HyperplaneSource::External;
        hyperplane->generatedPoint = point;
        hyperplane->objectiveFunctionValue = objectiveValue;
        hyperplane->isGlobal = true;
        return hyperplane;
    };

    double hash = Utilities::calculateHash(point);

    check(!dualSolver->hasHyperplaneBeenAdded(hash, firstIndex), "hyperplane detected before it was generated");

    dualSolver->addGeneratedHyperplane(createConstraintHyperplane(constraints[0]));

    check(dualSolver->hasHyperplaneBeenAdded(hash, firstIndex), "generated hyperplane not detected");
    check(dualSolver->hasHyperplaneBeenAdded(hash * (1.0 + 1e-12), firstIndex),
        "hyperplane in an almost identical point not detected");
    check(!dualSolver->hasHyperplaneBeenAdded(hash, secondIndex), "hyperplane detected for another constraint");
    check(!dualSolver->hasHyperplaneBeenAdded(Utilities::calculateHash(otherPoint), firstIndex),
        "hyperplane detected for another point");

    auto waitingListSize = dualSolver->hyperplaneWaitingList.size();

    dualSolver->addHyperplane(createConstraintHyperplane(constraints[0]));
    check(dualSolver->hyperplaneWaitingList.size() == waitingListSize, "duplicate hyperplane added to waiting list");

    dualSolver->addHyperplane(createConstraintHyperplane(constraints[1]));
    check(dualSolver->hyperplaneWaitingList.size() == waitingListSize + 1,
        "hyperplane for another constraint not added to waiting list");

    // Objective cuts in the same point are different cuts if their objective values differ
    dualSolver->addGeneratedHyperplane(createObjectiveHyperplane(1.0));
    waitingListSize = dualSolver->hyperplaneWaitingList.size();

    dualSolver->addHyperplane(createObjectiveHyperplane(1.0));
    check(dualSolver->hyperplaneWaitingList.size() == waitingListSize,
        "duplicate objective hyperplane added to waiting list");

    dualSolver->addHyperplane(createObjectiveHyperplane(2.0));
    check(dualSolver->hyperplaneWaitingList.size() == waitingListSize + 1,
        "objective hyperplane with another objective value not added to waiting list");

    return passed;
}

// The objective value of a solution to the dual problem is only a valid dual bound if the problem was solved to
// proven optimality. The MIP solvers also report optimality when their own gap tolerance has been met, so a loose
// gap tolerance must not result in a dual bound that has passed the known optimal value.
bool testDualBoundWithLooseGapTolerance(ES_MIPSolver mipSolver)
{
    const std::string problemFile = "data/instances/MINLP-convex-small/nvs12.osil";
    const double optimalValue = -481.2;

    auto solver = std::make_shared<Solver>();

    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(mipSolver));
    solver->updateSetting("Dual.MIP.NumberOfThreads", 1);
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
    solver->updateSetting("Termination.ObjectiveGap.Relative", 0.05);
    solver->updateSetting("Termination.TimeLimit", 60.0);

    if(!solver->setProblem(problemFile))
    {
        std::cout << name(mipSolver) << ": could not read " << problemFile << '\n';
        return false;
    }

    if(!solver->solveProblem())
    {
        std::cout << name(mipSolver) << ": could not solve " << problemFile << '\n';
        return false;
    }

    auto env = solver->getEnvironment();
    double dualBound = env->results->getGlobalDualBound();
    double tolerance = 1e-4 * std::max(1.0, std::abs(optimalValue));

    bool passed = env->problem->objectiveFunction->properties.isMinimize ? dualBound <= optimalValue + tolerance
                                                                        : dualBound >= optimalValue - tolerance;

    if(!passed)
    {
        std::cout << name(mipSolver) << ": dual bound " << dualBound << " has passed the optimal value "
                  << optimalValue << '\n';
    }

    return passed;
}

// The optimal value lies between the dual and the primal bound, so a dual bound candidate can only pass the primal
// bound by numerical error. A candidate that passes it by more is not a bound for the problem and must not be
// accepted, since that would close the objective gap by force.
bool testDualBoundCandidatePastPrimalBound(ES_MIPSolver mipSolver)
{
    bool passed = true;

    for(bool minimize : { true, false })
    {
        auto solver = makeSolver(mipSolver, ModelKind::Bounded, minimize);

        if(!solver)
        {
            std::cout << "Could not create problem for " << name(mipSolver) << '\n';
            return false;
        }

        auto env = solver->getEnvironment();
        env->results->createIteration();

        const double primalValue = minimize ? 10.0 : -10.0;

        PrimalSolution primalSolution;
        primalSolution.point = VectorDouble(env->reformulatedProblem->properties.numberOfVariables, 0.0);
        primalSolution.sourceType = E_PrimalSolutionSource::MIPSolutionPool;
        primalSolution.sourceDescription = "test";
        primalSolution.objValue = primalValue;
        primalSolution.iterFound = 0;
        primalSolution.maxIntegerToleranceError = 0.0;

        env->results->addPrimalSolution(primalSolution);

        auto addCandidate = [&](double objValue) {
            DualSolution candidate = { VectorDouble {}, E_DualSolutionSource::MIPSolverBound, objValue, 0, false };
            env->dualSolver->addDualSolutionCandidate(candidate);
            env->dualSolver->checkDualSolutionCandidates();
        };

        auto check = [&](const std::string& description, double expected) {
            double dualBound = env->results->getCurrentDualBound();

            if(std::abs(dualBound - expected) > 1e-8)
            {
                std::cout << name(mipSolver) << (minimize ? " (min)" : " (max)") << ": " << description
                          << ", dual bound is " << dualBound << " instead of " << expected << '\n';
                passed = false;
            }
        };

        // A bound on the correct side of the primal bound is used as it is
        double validBound = minimize ? primalValue - 1.0 : primalValue + 1.0;
        addCandidate(validBound);
        check("a valid dual bound was not accepted", validBound);

        // Passing the primal bound by more than numerical error means the candidate is not a valid bound. The
        // difference is kept within the relative objective gap tolerance, since that is the window in which the
        // candidate used to be accepted as the primal bound.
        addCandidate(minimize ? primalValue + 0.005 : primalValue - 0.005);
        check("a dual bound past the primal bound was accepted", validBound);

        // Passing it by numerical error only means that the primal solution is optimal
        addCandidate(minimize ? primalValue + 1e-12 : primalValue - 1e-12);
        check("a dual bound within numerical error of the primal bound was not accepted", primalValue);
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
    case 4:
        std::cout << "Starting test that duplicate hyperplanes are detected:\n";
        for(auto mipSolver : compiledSolvers())
            passed = testDuplicateHyperplanesAreDetected(mipSolver) && passed;
        std::cout << "Finished test that duplicate hyperplanes are detected.\n";
        break;
    case 5:
        std::cout << "Starting test that a loose gap tolerance does not give an invalid dual bound:\n";
        for(auto mipSolver : compiledSolvers())
            passed = testDualBoundWithLooseGapTolerance(mipSolver) && passed;
        std::cout << "Finished test that a loose gap tolerance does not give an invalid dual bound.\n";
        break;
    case 6:
        std::cout << "Starting test that a dual bound past the primal bound is not accepted:\n";
        for(auto mipSolver : compiledSolvers())
            passed = testDualBoundCandidatePastPrimalBound(mipSolver) && passed;
        std::cout << "Finished test that a dual bound past the primal bound is not accepted.\n";
        break;
    default:
        passed = false;
        std::cout << "Test #" << choice << " does not exist!\n";
    }

    return passed ? 0 : -1;
}
