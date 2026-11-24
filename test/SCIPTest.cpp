/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "../src/Callback.h"
#include "../src/DualSolver.h"
#include "../src/Environment.h"
#include "../src/Iteration.h"
#include "../src/Results.h"
#include "../src/Settings.h"
#include "../src/Solver.h"

#include "../src/MIPSolver/MIPSolverSCIP.h"

#include "../src/Model/Problem.h"
#include "IntegerCutTestCommon.h"

#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <tuple>

using namespace SHOT;

// Solves a problem with SCIP as MIP solver and checks that the optimal objective value is within the bounds
bool SCIPSolveTest(std::string filename, double correctObjectiveValue)
{
    auto solver = std::make_unique<Solver>();

    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::SCIP));

    try
    {
        if(!solver->setProblem(filename))
            return (false);
    }
    catch(Exception& e)
    {
        std::cout << "Error: " << e.what() << '\n';
        return (false);
    }

    solver->solveProblem();

    if(solver->getPrimalSolutions().empty())
    {
        std::cout << "No primal solution was found.\n";
        return (false);
    }

    double primalBound = solver->getPrimalBound();
    double dualBound = solver->getGlobalDualBound();
    bool isMinimize = solver->getOriginalProblem()->objectiveFunction->properties.isMinimize;

    std::cout << "Bounds [dual, primal]: [" << dualBound << ", " << primalBound << "], optimum "
              << correctObjectiveValue << '\n';

    if(isMinimize ? (correctObjectiveValue > primalBound + 1e-5 || correctObjectiveValue < dualBound - 1e-5)
                  : (correctObjectiveValue < primalBound - 1e-5 || correctObjectiveValue > dualBound + 1e-5))
    {
        std::cout << "The optimal objective value is not within the bounds.\n";
        return (false);
    }

    return (true);
}

bool SCIPTerminationCallbackTest(std::string filename)
{
    auto solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Termination.TimeLimit", 60.0);
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Error));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::SCIP));

    if(!solver->setProblem(filename))
    {
        std::cout << "Error while reading problem\n";
        return (false);
    }

    // Terminates in the third iteration
    solver->registerCallback<TerminationCheckContext>(
        [](TerminationCheckContext& context)
        {
            if(context.getIterationNumber() == 3)
                context.terminate();
        });

    if(!solver->solveProblem())
    {
        std::cout << "Error while solving problem\n";
        return (false);
    }

    if(env->results->getNumberOfIterations() != 3)
    {
        std::cout << "The termination callback did not stop the solver in the third iteration.\n";
        return (false);
    }

    if(env->results->terminationReason != E_TerminationReason::UserAbort)
    {
        std::cout << "The termination reason is not UserAbort.\n";
        return (false);
    }

    return (true);
}

// A MIP solver for the tests that create the problem directly, with an iteration for the statistics of the repair
std::shared_ptr<MIPSolverSCIP> createMIPSolver(EnvironmentPtr env)
{
    env->reformulatedProblem = std::make_shared<Problem>(env);
    env->results->createIteration();

    auto mip = std::make_shared<MIPSolverSCIP>(env);
    mip->initializeProblem();

    return (mip);
}

bool SCIPRepairInfeasibilityTest()
{
    for(bool isMinimization : { false, true })
    {
        Solver solver;
        auto env = solver.getEnvironment();
        auto mip = createMIPSolver(env);

        // The repair requires a generated cut to have been recorded
        env->dualSolver->generatedHyperplanes.push_back(std::make_shared<GeneratedHyperplane>());

        // Maximize an integer variable in [0, 10], or minimize a free variable
        if(isMinimization)
            mip->addVariable("x", E_VariableType::Real, -mip->getUnboundedVariableBoundValue(),
                mip->getUnboundedVariableBoundValue(), 0.0);
        else
            mip->addVariable("x", E_VariableType::Integer, 0.0, 10.0, 0.0);

        mip->addVariable("y", E_VariableType::Binary, 0.0, 1.0, 0.0);
        mip->initializeObjective();
        mip->addLinearTermToObjective(1.0, 0);
        mip->finalizeObjective(isMinimization);
        mip->finalizeProblem();
        mip->setSolutionLimit(2100000000);
        mip->setTimeLimit(10.0);

        // The repairable cuts x >= 2 and x <= 1 contradict each other
        std::map<int, double> terms = { { 0, 1.0 } };
        mip->addLinearConstraint(terms, -2.0, "lower", true, true);
        mip->addLinearConstraint(terms, 1.0, "upper", false, true);

        if(mip->solveProblem() != E_ProblemSolutionStatus::Infeasible)
        {
            std::cout << "The problem with the contradicting cuts is not infeasible.\n";
            return (false);
        }

        if(!mip->repairInfeasibility()
            || env->results->getCurrentIteration()->numberOfInfeasibilityRepairedConstraints == 0)
        {
            std::cout << "The infeasibility repair failed for a " << (isMinimization ? "minimization" : "maximization")
                      << " problem.\n";
            return (false);
        }

        if(auto status = mip->solveProblem(); status != E_ProblemSolutionStatus::Optimal)
        {
            std::cout << "The repaired problem is not solved to optimality, status " << static_cast<int>(status)
                      << ".\n";
            return (false);
        }
    }

    return (true);
}

// Maximizes x1 + 2 x2 + 3 x3 + 4 x4 with the variables in [0, 1] in a special ordered set, where at most one
// variable, or two adjacent ones, are nonzero
bool SCIPSpecialOrderedSetTest()
{
    for(auto type : { E_SOSType::One, E_SOSType::Two })
    {
        Solver solver;
        auto env = solver.getEnvironment();
        auto mip = createMIPSolver(env);

        for(int i = 0; i < 4; i++)
            mip->addVariable("x" + std::to_string(i + 1), E_VariableType::Real, 0.0, 1.0, 0.0);

        mip->initializeObjective();

        for(int i = 0; i < 4; i++)
            mip->addLinearTermToObjective(i + 1.0, i);

        mip->finalizeObjective(false);
        mip->addSpecialOrderedSet(type, { 0, 1, 2, 3 });
        mip->finalizeProblem();
        mip->setSolutionLimit(2100000000);
        mip->setTimeLimit(10.0);

        double expected = (type == E_SOSType::One) ? 4.0 : 7.0;

        if(mip->solveProblem() != E_ProblemSolutionStatus::Optimal
            || std::abs(mip->getObjectiveValue(0) - expected) > 1e-6)
        {
            std::cout << "The problem with a special ordered set of type " << (type == E_SOSType::One ? 1 : 2)
                      << " is not solved to the optimum " << expected << ".\n";
            return (false);
        }
    }

    return (true);
}

// Creates a knapsack problem, maximize the sum of v_i x_i with sum of w_i x_i <= 100 and binary variables
std::shared_ptr<MIPSolverSCIP> createKnapsackProblem(EnvironmentPtr env, int numberOfItems)
{
    auto mip = createMIPSolver(env);

    for(int i = 0; i < numberOfItems; i++)
        mip->addVariable("x" + std::to_string(i), E_VariableType::Binary, 0.0, 1.0, 0.0);

    mip->initializeObjective();

    for(int i = 0; i < numberOfItems; i++)
        mip->addLinearTermToObjective(10.0 + (i * 37) % 23, i);

    mip->finalizeObjective(false);

    mip->initializeConstraint();

    for(int i = 0; i < numberOfItems; i++)
        mip->addLinearTermToConstraint(5.0 + (i * 13) % 17, i);

    mip->finalizeConstraint("weight", SHOT_DBL_MIN, 100.0);
    mip->finalizeProblem();
    mip->setTimeLimit(10.0);

    return (mip);
}

// SCIP keeps the solutions it has found when a problem is changed, and they count towards the solution limit of the
// next solve. A solve after a cut that removes the solution must not stop at an old solution.
bool SCIPNoSolutionsFromEarlierSolvesTest()
{
    Solver solver;
    auto env = solver.getEnvironment();
    auto mip = createKnapsackProblem(env, 30);

    mip->setSolutionLimit(1);
    mip->solveProblem();

    if(mip->getNumberOfSolutions() == 0)
    {
        std::cout << "No solution was found in the first solve.\n";
        return (false);
    }

    auto firstSolution = mip->getVariableSolution(0);

    // The cut that removes only the first solution: sum(x_i, x_i = 1) - sum(x_i, x_i = 0) <= (number of ones) - 1
    std::map<int, double> terms;
    double numberOfOnes = 0.0;

    for(size_t i = 0; i < firstSolution.size(); i++)
    {
        if(firstSolution[i] > 0.5)
        {
            terms[static_cast<int>(i)] = 1.0;
            numberOfOnes += 1.0;
        }
        else
        {
            terms[static_cast<int>(i)] = -1.0;
        }
    }

    mip->addLinearConstraint(terms, -(numberOfOnes - 1.0), "cut", false, false);
    mip->solveProblem();

    for(int i = 0; i < mip->getNumberOfSolutions(); i++)
    {
        auto solution = mip->getVariableSolution(i);
        double sum = 0.0;

        for(auto& [index, coefficient] : terms)
            sum += coefficient * solution[index];

        if(sum > numberOfOnes - 1.0 + 1e-6)
        {
            std::cout << "A solution of the second solve violates the cut, so it is from the first solve.\n";
            return (false);
        }
    }

    if(mip->getNumberOfSolutions() == 0)
    {
        std::cout << "No solution was found in the second solve.\n";
        return (false);
    }

    return (true);
}

// SCIP counts a start as a found solution, so with the solution limit one the solve must still find a solution of its
// own, and not stop at the start
bool SCIPMIPStartSolutionLimitTest()
{
    Solver solver;
    auto env = solver.getEnvironment();
    auto mip = createKnapsackProblem(env, 30);

    // The start with all items out of the knapsack is feasible, with the objective value zero
    mip->addMIPStart(VectorDouble(30, 0.0));
    mip->setSolutionLimit(1);
    mip->solveProblem();

    if(mip->getNumberOfSolutions() == 0 || mip->getObjectiveValue(0) <= 0.0)
    {
        std::cout << "The solve stopped at the start instead of finding a solution of its own.\n";
        return (false);
    }

    return (true);
}

// The solution limit only applies to discrete problems. SCIP also solves a continuous problem with branch and
// bound, so it must not stop at the first solution of an LP.
bool SCIPContinuousProblemSolutionLimitTest()
{
    Solver solver;
    auto env = solver.getEnvironment();
    auto mip = createMIPSolver(env);

    // Maximize x + y with x + 2y <= 4 and 3x + y <= 6, whose optimum 2.8 is at x = 1.6 and y = 1.2
    mip->addVariable("x", E_VariableType::Real, 0.0, 10.0, 0.0);
    mip->addVariable("y", E_VariableType::Real, 0.0, 10.0, 0.0);
    mip->initializeObjective();
    mip->addLinearTermToObjective(1.0, 0);
    mip->addLinearTermToObjective(1.0, 1);
    mip->finalizeObjective(false);
    mip->addLinearConstraint({ { 0, 1.0 }, { 1, 2.0 } }, -4.0, "c1", false, false);
    mip->addLinearConstraint({ { 0, 3.0 }, { 1, 1.0 } }, -6.0, "c2", false, false);
    mip->finalizeProblem();
    mip->setTimeLimit(10.0);
    mip->setSolutionLimit(1);

    if(auto status = mip->solveProblem(); status != E_ProblemSolutionStatus::Optimal)
    {
        std::cout << "The LP is not solved to optimality with the solution limit one, status "
                  << static_cast<int>(status) << ".\n";
        return (false);
    }

    if(std::abs(mip->getObjectiveValue(0) - 2.8) > 1e-6 || std::abs(mip->getDualObjectiveValue() - 2.8) > 1e-6)
    {
        std::cout << "The objective value or the bound of the LP is not the optimum 2.8.\n";
        return (false);
    }

    return (true);
}

// Solves a problem with a quadratic objective function or constraint directly with SCIP, and checks the objective
// value and the dual bound of the global optimum
bool SCIPQuadraticTest()
{
    // Minimize and maximize x * y, which is nonconvex, with x + y <= 3 and x, y in [0, 2]: the minimum 0 is at x = 0
    // or y = 0, and the maximum 2.25 at x = y = 1.5
    for(bool isMinimization : { true, false })
    {
        Solver solver;
        auto env = solver.getEnvironment();
        auto mip = createMIPSolver(env);

        mip->addVariable("x", E_VariableType::Real, 0.0, 2.0, 0.0);
        mip->addVariable("y", E_VariableType::Real, 0.0, 2.0, 0.0);
        mip->initializeObjective();
        mip->addQuadraticTermToObjective(1.0, 0, 1);
        mip->finalizeObjective(isMinimization, 1.0);
        mip->addLinearConstraint({ { 0, 1.0 }, { 1, 1.0 } }, -3.0, "sum", false, false);
        mip->finalizeProblem();
        mip->setTimeLimit(10.0);

        double expected = isMinimization ? 1.0 : 3.25;

        if(mip->solveProblem() != E_ProblemSolutionStatus::Optimal
            || std::abs(mip->getObjectiveValue(0) - expected) > 1e-5
            || std::abs(mip->getDualObjectiveValue() - expected) > 1e-5)
        {
            std::cout << "The " << (isMinimization ? "minimum" : "maximum") << " of the quadratic objective function "
                      << "is not " << expected << ".\n";
            return (false);
        }
    }

    // Maximize x + y with the nonconvex constraint x^2 + y^2 >= 1 and the convex constraint x^2 + y^2 <= 4, where x
    // and y are integers in [0, 2]: the maximum 2 is at, e.g., x = y = 1
    Solver solver;
    auto env = solver.getEnvironment();
    auto mip = createMIPSolver(env);

    mip->addVariable("x", E_VariableType::Integer, 0.0, 2.0, 0.0);
    mip->addVariable("y", E_VariableType::Integer, 0.0, 2.0, 0.0);
    mip->initializeObjective();
    mip->addLinearTermToObjective(1.0, 0);
    mip->addLinearTermToObjective(1.0, 1);
    mip->finalizeObjective(false);

    for(auto [name, lhs, rhs] : { std::tuple("outside", 1.0, SHOT_DBL_MAX), std::tuple("inside", SHOT_DBL_MIN, 4.0) })
    {
        mip->initializeConstraint();
        mip->addQuadraticTermToConstraint(1.0, 0, 0);
        mip->addQuadraticTermToConstraint(1.0, 1, 1);
        mip->finalizeConstraint(name, lhs, rhs);
    }

    mip->finalizeProblem();
    mip->setTimeLimit(10.0);

    if(mip->solveProblem() != E_ProblemSolutionStatus::Optimal || std::abs(mip->getObjectiveValue(0) - 2.0) > 1e-5)
    {
        std::cout << "The problem with quadratic constraints is not solved to the optimum 2.\n";
        return (false);
    }

    return (true);
}

int SCIPTest(int argc, char* argv[])
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
        std::cout << "Solving a MINLP problem with SCIP.\n";
        passed = SCIPSolveTest("data/tls2.osil", 5.3);
        break;
    case 2:
        std::cout << "Testing the termination callback with SCIP.\n";
        passed = SCIPTerminationCallbackTest("data/tls2.osil");
        break;
    case 3:
        std::cout << "Solving gear.osil with integer cuts in SCIP.\n";
        passed = solveWithIntegerCuts(ES_MIPSolver::SCIP, "data/gear.osil", 0.0);
        break;
    case 4:
        std::cout << "Solving windfac.osil with integer cuts in SCIP.\n";
        passed = solveWithIntegerCuts(ES_MIPSolver::SCIP, "data/windfac.osil", 0.254487);
        break;
    case 5:
        std::cout << "Solving ex1252a.osil with integer cuts in SCIP.\n";
        passed = solveWithIntegerCuts(ES_MIPSolver::SCIP, "data/ex1252a.osil", 128893.741);
        break;
    case 6:
        std::cout << "Solving a problem with semicontinuous variables with SCIP.\n";
        passed = SCIPSolveTest("data/meanvarxsc.osil", 14.36923211);
        break;
    case 7:
        std::cout << "Solving the nonconvex maximization problem ncvx_max_div.nl with SCIP.\n";
        passed = SCIPSolveTest("data/ncvx_max_div.nl", 13.0);
        break;
    case 8:
        std::cout << "Solving the nonconvex minimization problem ncvx_min_div.nl with SCIP.\n";
        passed = SCIPSolveTest("data/ncvx_min_div.nl", -13.0);
        break;
    case 9:
        std::cout << "Solving the nonconvex maximization problem ncvx_max_ndiv.nl with SCIP.\n";
        passed = SCIPSolveTest("data/ncvx_max_ndiv.nl", 13.0);
        break;
    case 10:
        std::cout << "Solving the nonconvex minimization problem ncvx_min_ndiv.nl with SCIP.\n";
        passed = SCIPSolveTest("data/ncvx_min_ndiv.nl", -13.0);
        break;
    case 11:
        std::cout << "Testing the SCIP infeasibility repair for minimization and maximization problems.\n";
        passed = SCIPRepairInfeasibilityTest();
        break;
    case 12:
        std::cout << "Testing special ordered sets of type 1 and 2 in SCIP.\n";
        passed = SCIPSpecialOrderedSetTest();
        break;
    case 13:
        std::cout << "Testing that a SCIP solve does not use the solutions of earlier solves.\n";
        passed = SCIPNoSolutionsFromEarlierSolvesTest();
        break;
    case 14:
        std::cout << "Testing that a MIP start does not use up the solution limit in SCIP.\n";
        passed = SCIPMIPStartSolutionLimitTest();
        break;
    case 15:
        std::cout << "Testing that the solution limit does not stop a continuous problem in SCIP.\n";
        passed = SCIPContinuousProblemSolutionLimitTest();
        break;
    case 16:
        std::cout << "Testing quadratic objective functions and constraints in SCIP.\n";
        passed = SCIPQuadraticTest();
        break;
    case 17:
        std::cout << "Solving a nonconvex MIQCQP problem directly with SCIP.\n";
        passed = SCIPSolveTest("data/instances/MINLP-nonconvex/edgecross10-010.osil", 1.0);
        break;
    case 18:
        std::cout << "Solving a convex MIQCQP problem directly with SCIP.\n";
        passed = SCIPSolveTest("data/instances/MIQCQP-convex/nvs10.osil", -310.8);
        break;
    default:
        passed = false;
        std::cout << "Test #" << choice << " does not exist!\n";
    }

    return (passed ? 0 : -1);
}
