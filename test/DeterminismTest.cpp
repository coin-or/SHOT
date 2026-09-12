/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

/* Regression tests for the reproducibility of a solve. Two runs of the same problem with the same settings must
   follow the same search path, which they only do if nothing in the solver depends on where objects happen to be
   allocated, or on numbers drawn from a randomly seeded engine. */

#include "../src/Solver.h"
#include "../src/Environment.h"
#include "../src/Results.h"
#include "../src/Settings.h"
#include "../src/Utilities.h"

#include "../src/Model/Constraints.h"
#include "../src/Model/NonlinearExpressions.h"
#include "../src/Model/ObjectiveFunction.h"
#include "../src/Model/Problem.h"
#include "../src/Model/Terms.h"
#include "../src/Model/Variables.h"

#include <cmath>
#include <iomanip>
#include <optional>
#include <tuple>
#include <cstdio>
#include <iostream>
#include <set>
#include <vector>

using namespace SHOT;

// The containers the gradients and Hessians are returned in must order their entries on the variable index. With
// the default comparator of std::map they are ordered on the address of the variable instead, which changes from
// run to run, and everything generated while iterating over them changes with it.
static_assert(std::is_same<SparseVariableVector::key_compare, VariableIndexComparator>::value,
    "The sparse variable vector must be ordered on the variable index");
static_assert(std::is_same<SparseVariableMatrix::key_compare, VariableIndexComparator>::value,
    "The sparse variable matrix must be ordered on the variable index");

namespace
{

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

// Solves the instance with the given seed and returns the iterations and the bounds it ended at
std::optional<std::tuple<int, double, double>> solveWithSeed(ES_MIPSolver mipSolver, int seed)
{
    const std::string problemFile = "data/instances/MINLP-convex-small/synthes1.osil";

    auto solver = std::make_shared<Solver>();

    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(mipSolver));
    solver->updateSetting("Dual.MIP.NumberOfThreads", 1);
    solver->updateSetting("Dual.MIP.RandomSeed", seed);
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
    solver->updateSetting("Termination.TimeLimit", 60.0);

    if(!solver->setProblem(problemFile))
    {
        std::cout << "  " << name(mipSolver) << ": could not read " << problemFile << '\n';
        return std::nullopt;
    }

    if(!solver->solveProblem())
    {
        std::cout << "  " << name(mipSolver) << ": could not solve " << problemFile << '\n';
        return std::nullopt;
    }

    auto env = solver->getEnvironment();

    return std::make_tuple(static_cast<int>(env->results->iterations.size()), env->results->getGlobalDualBound(),
        env->results->getPrimalBound());
}

}

bool testGradientsAreOrderedOnVariableIndex()
{
    bool passed = true;

    auto solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));

    auto problem = std::make_shared<Problem>(env);
    problem->name = "orderingtest";

    const int numberOfVariables = 12;

    // The variables are allocated first and then numbered in the opposite order, so that the variable with the
    // lowest index is the one allocated last. Iterating on the address then gives a different order than
    // iterating on the index for the usual allocator behaviour.
    std::vector<VariablePtr> allocated;

    for(int i = 0; i < numberOfVariables; i++)
        allocated.push_back(
            std::make_shared<Variable>("x" + std::to_string(i), E_VariableType::Real, 0.1, 10.0));

    for(auto it = allocated.rbegin(); it != allocated.rend(); it++)
        problem->add(*it);

    int ascendingAddresses = 0;

    for(size_t i = 1; i < problem->allVariables.size(); i++)
    {
        if(problem->allVariables.at(i - 1).get() < problem->allVariables.at(i).get())
            ascendingAddresses++;
    }

    std::cout << "  " << ascendingAddresses << " of " << problem->allVariables.size() - 1
              << " consecutive variables are at ascending addresses (the indexes are ascending by construction)\n";

    // A nonlinear constraint containing every variable, so that the gradient and the Hessian have an entry for
    // each of them
    auto sum = std::make_shared<ExpressionSum>();

    for(auto& V : problem->allVariables)
        sum->children.add(std::make_shared<ExpressionLog>(std::make_shared<ExpressionVariable>(V)));

    auto constraint = std::make_shared<NonlinearConstraint>("nlconstraint", sum, SHOT_DBL_MIN, 100.0);
    problem->add(constraint);

    auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);

    for(auto& V : problem->allVariables)
        objective->add(std::make_shared<LinearTerm>(1.0, V));

    problem->add(objective);
    problem->finalize();

    VectorDouble point(numberOfVariables, 2.0);

    auto gradient = constraint->calculateGradient(point, true);

    if((int)gradient.size() != numberOfVariables)
    {
        std::cout << "  FAILED: the gradient has " << gradient.size() << " entries instead of " << numberOfVariables
                  << ".\n";
        passed = false;
    }

    int previousIndex = -1;

    for(auto& G : gradient)
    {
        if(G.first->getIndex() <= previousIndex)
        {
            std::cout << "  FAILED: the gradient entry for variable index " << G.first->getIndex()
                      << " comes after the one for index " << previousIndex << ".\n";
            passed = false;
            break;
        }

        previousIndex = G.first->getIndex();
    }

    auto hessian = constraint->calculateHessian(point, true);

    std::pair<int, int> previousKey = { -1, -1 };

    for(auto& H : hessian)
    {
        std::pair<int, int> key = { H.first.first->getIndex(), H.first.second->getIndex() };

        if(key <= previousKey)
        {
            std::cout << "  FAILED: the Hessian entry for (" << key.first << ", " << key.second
                      << ") comes after the one for (" << previousKey.first << ", " << previousKey.second << ").\n";
            passed = false;
            break;
        }

        previousKey = key;
    }

    if(passed)
        std::cout << "  The gradient and the Hessian are both ordered on the variable index.\n";

    return passed;
}

bool testHashCoefficientsAreTheSameInEveryRun()
{
    bool passed = true;

    // A coefficient may only depend on its position and its stream, so that the hash of a point does not depend on
    // how many coefficients have been generated before it, or on which solver instance generates them
    for(size_t index = 0; index < 50; index++)
    {
        for(size_t stream = 0; stream < 2; stream++)
        {
            double first = Utilities::fixedPseudoRandomNumber(index, stream, 1.0, 101.0);
            double second = Utilities::fixedPseudoRandomNumber(index, stream, 1.0, 101.0);

            if(first != second)
            {
                std::cout << "  FAILED: the coefficient for index " << index << " in stream " << stream
                          << " changed between two calls.\n";
                passed = false;
            }

            if(first < 1.0 || first >= 101.0)
            {
                std::cout << "  FAILED: the coefficient for index " << index << " in stream " << stream << " is "
                          << first << ", which is outside the requested interval.\n";
                passed = false;
            }
        }
    }

    // Coefficients that repeat would make different points hash to the same value
    std::set<double> values;

    for(size_t index = 0; index < 1000; index++)
    {
        values.insert(Utilities::fixedPseudoRandomNumber(index, 0, 1.0, 101.0));
        values.insert(Utilities::fixedPseudoRandomNumber(index, 1, 1.0, 101.0));
    }

    if(values.size() != 2000)
    {
        std::cout << "  FAILED: only " << values.size() << " of 2000 coefficients are different.\n";
        passed = false;
    }

    // Hashing a longer point extends the coefficients that the shorter points are hashed with, which must not
    // change the hash of a point that has already been hashed
    VectorDouble shortPoint = { 1.0, -2.5, 3.25, 0.0, 17.0 };
    VectorDouble longPoint(500, 1.5);

    double before = Utilities::calculateHash(shortPoint);
    Utilities::calculateHash(longPoint);
    double after = Utilities::calculateHash(shortPoint);

    if(before != after)
    {
        std::cout << "  FAILED: the hash of a point changed from " << before << " to " << after
                  << " after a longer point had been hashed.\n";
        passed = false;
    }

    // Recorded from a run of this test. The point hash has to be the same in every process for duplicate
    // detection to behave the same way, which a randomly seeded generator cannot give. Change this value only
    // together with a deliberate change of fixedPseudoRandomNumber.
    VectorDouble recordedPoint = { 1.0, 2.0, 3.0, 4.0 };
    double recordedHash = 641.9325787265052;

    if(Utilities::calculateHash(recordedPoint) != recordedHash)
    {
        std::cout << "  FAILED: the hash of the recorded point is " << std::setprecision(17)
                  << Utilities::calculateHash(recordedPoint) << " instead of " << recordedHash << ".\n";
        passed = false;
    }

    if(passed)
        std::cout << "  The hash coefficients depend only on their position.\n";

    return passed;
}

bool testTheSameSeedGivesTheSameSolve(ES_MIPSolver mipSolver)
{
    const double optimalValue = 6.00975891;

    auto first = solveWithSeed(mipSolver, 1);
    auto second = solveWithSeed(mipSolver, 1);

    if(!first || !second)
        return false;

    if(*first != *second)
    {
        std::cout << "  FAILED: " << name(mipSolver) << " solved the same problem with the same seed in "
                  << std::get<0>(*first) << " iterations ending at [" << std::get<1>(*first) << ", "
                  << std::get<2>(*first) << "] the first time and in " << std::get<0>(*second)
                  << " iterations ending at [" << std::get<1>(*second) << ", " << std::get<2>(*second)
                  << "] the second time.\n";
        return false;
    }

    // Another seed is a different but equally valid solve, so it has to reach the same optimum
    auto other = solveWithSeed(mipSolver, 4711);

    if(!other)
        return false;

    // A seed of zero leaves the solver at its own default, which has to solve the problem just as well
    auto solverDefault = solveWithSeed(mipSolver, 0);

    if(!solverDefault)
        return false;

    if(std::abs(std::get<2>(*solverDefault) - optimalValue) > 1e-4 * std::max(1.0, std::abs(optimalValue)))
    {
        std::cout << "  FAILED: " << name(mipSolver) << " returned " << std::get<2>(*solverDefault)
                  << " instead of " << optimalValue << " when the seed was left at the default of the solver.\n";
        return false;
    }

    if(std::abs(std::get<2>(*other) - optimalValue) > 1e-4 * std::max(1.0, std::abs(optimalValue)))
    {
        std::cout << "  FAILED: " << name(mipSolver) << " returned " << std::get<2>(*other) << " instead of "
                  << optimalValue << " when the seed was changed.\n";
        return false;
    }

    std::cout << "  " << name(mipSolver) << ": both solves with seed 1 took " << std::get<0>(*first)
              << " iterations, seed 4711 took " << std::get<0>(*other) << " and the default of the solver "
              << std::get<0>(*solverDefault) << ", all reaching the same optimum.\n";

    return true;
}

int DeterminismTest(int argc, char* argv[])
{
    int choice = 1;

    if(argc > 1)
    {
        if(sscanf(argv[1], "%d", &choice) != 1)
        {
            printf("Couldn't parse that input as a number\n");
            return -1;
        }
    }

    bool passed = true;

    switch(choice)
    {
    case 1:
        std::cout << "Starting test that gradients and Hessians are ordered on the variable index:\n";
        passed = testGradientsAreOrderedOnVariableIndex();
        std::cout << "Finished test that gradients and Hessians are ordered on the variable index.\n";
        break;
    case 2:
        std::cout << "Starting test that the hash coefficients are the same in every run:\n";
        passed = testHashCoefficientsAreTheSameInEveryRun();
        std::cout << "Finished test that the hash coefficients are the same in every run.\n";
        break;
    case 3:
        std::cout << "Starting test that the same random seed gives the same solve:\n";
        for(auto mipSolver : compiledSolvers())
            passed = testTheSameSeedGivesTheSameSolve(mipSolver) && passed;
        std::cout << "Finished test that the same random seed gives the same solve.\n";
        break;
    default:
        passed = false;
        std::cout << "Test #" << choice << " does not exist!\n";
    }

    return passed ? 0 : -1;
}
