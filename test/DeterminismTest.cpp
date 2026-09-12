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
#include "../src/Settings.h"
#include "../src/Utilities.h"

#include "../src/Model/Constraints.h"
#include "../src/Model/NonlinearExpressions.h"
#include "../src/Model/ObjectiveFunction.h"
#include "../src/Model/Problem.h"
#include "../src/Model/Terms.h"
#include "../src/Model/Variables.h"

#include <cstdio>
#include <iostream>
#include <vector>

using namespace SHOT;

// The containers the gradients and Hessians are returned in must order their entries on the variable index. With
// the default comparator of std::map they are ordered on the address of the variable instead, which changes from
// run to run, and everything generated while iterating over them changes with it.
static_assert(std::is_same<SparseVariableVector::key_compare, VariableIndexComparator>::value,
    "The sparse variable vector must be ordered on the variable index");
static_assert(std::is_same<SparseVariableMatrix::key_compare, VariableIndexComparator>::value,
    "The sparse variable matrix must be ordered on the variable index");

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
    default:
        passed = false;
        std::cout << "Test #" << choice << " does not exist!\n";
    }

    return passed ? 0 : -1;
}
