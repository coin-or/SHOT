/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

// Tests the exhaustive search in the fixed-integer primal strategy, where an NLP problem is solved for each combination
// of the values of the discrete variables. See docs/ExhaustiveFixedIntegerSearchPlan.md.

#include "../src/Solver.h"
#include "../src/DualSolver.h"
#include "../src/Environment.h"
#include "../src/PrimalSolver.h"
#include "../src/Results.h"
#include "../src/Settings.h"

#include "../src/Model/Variables.h"
#include "../src/Model/Terms.h"
#include "../src/Model/Constraints.h"
#include "../src/Model/ObjectiveFunction.h"
#include "../src/Model/Problem.h"

#include <cmath>
#include <iostream>

using namespace SHOT;

namespace
{

std::unique_ptr<Solver> createSolver(ES_MIPSolver mipSolver = ES_MIPSolver::Highs)
{
    auto solver = std::make_unique<Solver>();
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(mipSolver));
    solver->updateSetting("Dual.MIP.NumberOfThreads", 1);

    // The bounds of the integer variable are otherwise tightened, which removes combinations
    solver->updateSetting("Model.BoundTightening.FeasibilityBased.Use", false);
    solver->updateSetting("Model.BoundTightening.InitialPOA.Use", false);
    return (solver);
}

bool expect(bool condition, const std::string& description)
{
    if(!condition)
        std::cout << "  FAILED: " << description << "\n";

    return (condition);
}

int numberOfCombinationsSolved(EnvironmentPtr env)
{
    return (env->solutionStatistics.numberOfFixedIntegerEnumerationCombinationsFeasible
        + env->solutionStatistics.numberOfFixedIntegerEnumerationCombinationsInfeasible
        + env->solutionStatistics.numberOfFixedIntegerEnumerationCombinationsUnresolved);
}

void printStatistics(EnvironmentPtr env)
{
    std::cout << "  Search run: " << env->solutionStatistics.hasFixedIntegerEnumerationBeenRun
              << ", combinations: " << env->solutionStatistics.numberOfFixedIntegerEnumerationCombinations
              << ", feasible: " << env->solutionStatistics.numberOfFixedIntegerEnumerationCombinationsFeasible
              << ", infeasible: " << env->solutionStatistics.numberOfFixedIntegerEnumerationCombinationsInfeasible
              << ", unresolved: " << env->solutionStatistics.numberOfFixedIntegerEnumerationCombinationsUnresolved
              << ", skipped: " << env->solutionStatistics.numberOfFixedIntegerEnumerationCombinationsSkipped << "\n";
}

// A nonconvex problem with the six combinations of a binary variable b and an integer variable i in [1,3]. The linear
// constraints are fulfilled for (b,i) = (0,1), (1,1) and (0,2).
//
//  minimize    x*y - y*z + b + i
//  subject to  x*y + y*z - b - i <= 4
//              x + y + z - i >= 2
//              x + y + z + 2*b + 4*i <= 12
//              0.5 <= x,y,z <= 3
ProblemPtr createNonconvexProblem(EnvironmentPtr env)
{
    auto problem = std::make_shared<Problem>(env);

    auto x = std::make_shared<Variable>("x", E_VariableType::Real, 0.5, 3.0);
    auto y = std::make_shared<Variable>("y", E_VariableType::Real, 0.5, 3.0);
    auto z = std::make_shared<Variable>("z", E_VariableType::Real, 0.5, 3.0);
    auto b = std::make_shared<Variable>("b", E_VariableType::Binary, 0.0, 1.0);
    auto i = std::make_shared<Variable>("i", E_VariableType::Integer, 1.0, 3.0);
    problem->add({ x, y, z, b, i });

    auto objective = std::make_shared<QuadraticObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    objective->add(std::make_shared<QuadraticTerm>(1.0, x, y));
    objective->add(std::make_shared<QuadraticTerm>(-1.0, y, z));
    objective->add(std::make_shared<LinearTerm>(1.0, b));
    objective->add(std::make_shared<LinearTerm>(1.0, i));
    problem->add(objective);

    QuadraticTerms bilinearTerms;
    bilinearTerms.add(std::make_shared<QuadraticTerm>(1.0, x, y));
    bilinearTerms.add(std::make_shared<QuadraticTerm>(1.0, y, z));
    LinearTerms bilinearLinearTerms;
    bilinearLinearTerms.add(std::make_shared<LinearTerm>(-1.0, b));
    bilinearLinearTerms.add(std::make_shared<LinearTerm>(-1.0, i));
    problem->add(
        std::make_shared<QuadraticConstraint>("bilinear", bilinearLinearTerms, bilinearTerms, SHOT_DBL_MIN, 4.0));

    LinearTerms lowerTerms;
    lowerTerms.add(std::make_shared<LinearTerm>(1.0, x));
    lowerTerms.add(std::make_shared<LinearTerm>(1.0, y));
    lowerTerms.add(std::make_shared<LinearTerm>(1.0, z));
    lowerTerms.add(std::make_shared<LinearTerm>(-1.0, i));
    problem->add(std::make_shared<LinearConstraint>("lower", lowerTerms, 2.0, SHOT_DBL_MAX));

    LinearTerms upperTerms;
    upperTerms.add(std::make_shared<LinearTerm>(1.0, x));
    upperTerms.add(std::make_shared<LinearTerm>(1.0, y));
    upperTerms.add(std::make_shared<LinearTerm>(1.0, z));
    upperTerms.add(std::make_shared<LinearTerm>(2.0, b));
    upperTerms.add(std::make_shared<LinearTerm>(4.0, i));
    problem->add(std::make_shared<LinearConstraint>("upper", upperTerms, SHOT_DBL_MIN, 12.0));

    problem->finalize();

    return (problem);
}

// A convex problem with a quadratic objective function and the two combinations of a binary variable
//
//  minimize    x^2 + y^2 + b
//  subject to  x + y - b >= 1
//              0 <= x,y <= 4
ProblemPtr createConvexQuadraticProblem(EnvironmentPtr env)
{
    auto problem = std::make_shared<Problem>(env);

    auto x = std::make_shared<Variable>("x", E_VariableType::Real, 0.0, 4.0);
    auto y = std::make_shared<Variable>("y", E_VariableType::Real, 0.0, 4.0);
    auto b = std::make_shared<Variable>("b", E_VariableType::Binary, 0.0, 1.0);
    problem->add({ x, y, b });

    auto objective = std::make_shared<QuadraticObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    objective->add(std::make_shared<QuadraticTerm>(1.0, x, x));
    objective->add(std::make_shared<QuadraticTerm>(1.0, y, y));
    objective->add(std::make_shared<LinearTerm>(1.0, b));
    problem->add(objective);

    LinearTerms terms;
    terms.add(std::make_shared<LinearTerm>(1.0, x));
    terms.add(std::make_shared<LinearTerm>(1.0, y));
    terms.add(std::make_shared<LinearTerm>(-1.0, b));
    problem->add(std::make_shared<LinearConstraint>("linear", terms, 1.0, SHOT_DBL_MAX));

    problem->finalize();

    return (problem);
}

bool setNonconvexProblem(Solver* solver)
{
    if(!solver->setProblem(createNonconvexProblem(solver->getEnvironment())))
    {
        std::cout << "  FAILED: could not set the problem.\n";
        return (false);
    }

    return (true);
}

bool EnumerationTestNonconvexDefault()
{
    bool passed = true;

    auto solver = createSolver();
    auto env = solver->getEnvironment();

    if(!setNonconvexProblem(solver.get()))
        return (false);

    passed = expect(env->settings->getSetting<bool>("Primal.FixedInteger.Enumeration.UseInitially"),
                 "the search is not used by default for a nonconvex problem")
        && passed;

    if(!solver->solveProblem())
        return (expect(false, "could not solve the problem"));

    printStatistics(env);

    passed = expect(env->solutionStatistics.hasFixedIntegerEnumerationBeenRun, "the search was not run") && passed;
    passed = expect(env->solutionStatistics.numberOfFixedIntegerEnumerationCombinations == 6,
                 "the number of combinations is not six")
        && passed;
    passed = expect(numberOfCombinationsSolved(env) == 6, "not all combinations were solved") && passed;
    passed = expect(env->solutionStatistics.numberOfFixedIntegerEnumerationCombinationsFeasible == 3,
                 "the number of feasible combinations is not three")
        && passed;
    passed = expect(env->solutionStatistics.numberOfFixedIntegerEnumerationCombinationsSkipped == 0,
                 "combinations were skipped")
        && passed;
    passed = expect(solver->hasPrimalSolution(), "no primal solution was found") && passed;

    return (passed);
}

bool EnumerationTestNoIntegerCuts()
{
    // The solution process is terminated at the first primal solution, which is found in the search, so an integer
    // cut can only have been added there

    bool passed = true;

    auto solver = createSolver();
    auto env = solver->getEnvironment();

    if(!setNonconvexProblem(solver.get()))
        return (false);

    passed = expect(env->settings->getSetting<bool>("Dual.HyperplaneCuts.UseIntegerCuts"),
                 "integer cuts are not used for the nonconvex problem")
        && passed;

    solver->registerCallback<NewPrimalSolutionContext>([](NewPrimalSolutionContext& context) { context.terminate(); });

    solver->solveProblem();

    printStatistics(env);

    passed = expect(env->solutionStatistics.hasFixedIntegerEnumerationBeenRun, "the search was not run") && passed;
    passed = expect(env->solutionStatistics.numberOfFixedIntegerEnumerationCombinationsFeasible == 1,
                 "the search did not end at the first feasible combination")
        && passed;
    passed = expect(numberOfCombinationsSolved(env) < 6, "the search was not terminated") && passed;
    passed = expect(env->results->terminationReason == E_TerminationReason::UserAbort,
                 "the solution process was not terminated by the callback")
        && passed;
    // A new integer cut is in the waiting list until it is added to the dual problem
    passed = expect(env->dualSolver->integerCutWaitingList.empty() && env->dualSolver->generatedIntegerCuts.empty(),
                 "an integer cut was added in the search")
        && passed;
    passed
        = expect(env->primalSolver->usedPrimalNLPCandidates.size() == 1, "the combination was not added as used")
        && passed;

    return (passed);
}

bool EnumerationTestMaxCombinations()
{
    bool passed = true;

    auto solver = createSolver();
    auto env = solver->getEnvironment();
    solver->updateSetting("Primal.FixedInteger.Enumeration.MaxCombinations", 5);

    if(!setNonconvexProblem(solver.get()))
        return (false);

    if(!solver->solveProblem())
        return (expect(false, "could not solve the problem"));

    printStatistics(env);

    passed = expect(!env->solutionStatistics.hasFixedIntegerEnumerationBeenRun,
                 "the search was run with more combinations than the limit")
        && passed;
    passed = expect(solver->hasPrimalSolution(), "no primal solution was found") && passed;

    return (passed);
}

bool EnumerationTestTimeLimit()
{
    // Without time for the search, no NLP problem is solved in it

    bool passed = true;

    auto solver = createSolver();
    auto env = solver->getEnvironment();
    solver->updateSetting("Primal.FixedInteger.Enumeration.TimeLimit", 0.0);

    if(!setNonconvexProblem(solver.get()))
        return (false);

    if(!solver->solveProblem())
        return (expect(false, "could not solve the problem"));

    printStatistics(env);

    passed = expect(env->solutionStatistics.hasFixedIntegerEnumerationBeenRun, "the search was not run") && passed;
    passed = expect(numberOfCombinationsSolved(env) == 0, "combinations were solved without time for it") && passed;
    passed = expect(solver->hasPrimalSolution(), "no primal solution was found after the search") && passed;

    return (passed);
}

bool EnumerationTestConvex(ES_TreeStrategy treeStrategy, ES_PrimalNLPProblemSource sourceProblem)
{
    // synthes1 is convex and has three binary variables. The search is not used by default, and the gap is closed, so
    // the fallback is not run either.

    bool passed = true;

    for(bool useEnumeration : { false, true })
    {
        auto solver = createSolver();
        auto env = solver->getEnvironment();
        solver->updateSetting("Dual.TreeStrategy", static_cast<int>(treeStrategy));
        solver->updateSetting("Primal.FixedInteger.SourceProblem", static_cast<int>(sourceProblem));

        if(useEnumeration)
            solver->updateSetting("Primal.FixedInteger.Enumeration.UseInitially", true);

        if(!solver->setProblem("data/synthes1.osil"))
            return (expect(false, "could not read the problem"));

        if(!useEnumeration)
            passed = expect(!env->settings->getSetting<bool>("Primal.FixedInteger.Enumeration.UseInitially"),
                         "the search is used by default for a convex problem")
                && passed;

        if(!solver->solveProblem())
            return (expect(false, "could not solve the problem"));

        printStatistics(env);

        passed = expect(env->solutionStatistics.hasFixedIntegerEnumerationBeenRun == useEnumeration,
                     useEnumeration ? "the search was not run" : "the search was run")
            && passed;
        passed = expect(std::abs(solver->getPrimalBound() - 6.00976) < 1e-3, "the objective value is not correct")
            && passed;
        passed = expect(env->results->terminationReason == E_TerminationReason::AbsoluteGap
                         || env->results->terminationReason == E_TerminationReason::RelativeGap,
                     "the gap was not closed")
            && passed;

        if(useEnumeration)
        {
            passed = expect(env->solutionStatistics.numberOfFixedIntegerEnumerationCombinations == 8,
                         "the number of combinations is not eight")
                && passed;
            passed = expect(numberOfCombinationsSolved(env) == 8, "not all combinations were solved") && passed;
            passed = expect(env->solutionStatistics.numberOfFixedIntegerEnumerationCombinationsUnresolved == 0,
                         "there are unresolved combinations")
                && passed;

            // Each combination is solved once on each problem
            int numberOfProblems = (sourceProblem == ES_PrimalNLPProblemSource::Both) ? 2 : 1;
            passed = expect(env->solutionStatistics.numberOfProblemsFixedNLP >= 8 * numberOfProblems,
                         "not all NLP problems were solved")
                && passed;
        }
    }

    return (passed);
}

bool EnumerationTestFallback()
{
    // The dual stagnation limit terminates the solution process of the convex problem with an open gap

    bool passed = true;

    for(bool useFallback : { true, false })
    {
        auto solver = createSolver();
        auto env = solver->getEnvironment();
        solver->updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::MultiTree));
        solver->updateSetting("Termination.DualStagnation.IterationLimit", 0);
        solver->updateSetting("Primal.FixedInteger.Enumeration.UseAsFallback", useFallback);

        if(!solver->setProblem("data/synthes1.osil"))
            return (expect(false, "could not read the problem"));

        if(!solver->solveProblem())
            return (expect(false, "could not solve the problem"));

        printStatistics(env);

        passed = expect(env->results->terminationReason == E_TerminationReason::ObjectiveStagnation,
                     "the solution process was not terminated due to stagnation")
            && passed;
        passed = expect(env->solutionStatistics.hasFixedIntegerEnumerationBeenRun == useFallback,
                     useFallback ? "the fallback search was not run" : "the fallback search was run")
            && passed;

        if(useFallback)
        {
            passed = expect(numberOfCombinationsSolved(env)
                             + env->solutionStatistics.numberOfFixedIntegerEnumerationCombinationsSkipped
                         == 8,
                         "not all combinations were solved or skipped")
                && passed;
            passed = expect(std::abs(solver->getPrimalBound() - 6.00976) < 1e-3,
                         "the optimal solution was not found in the fallback search")
                && passed;

            // The search does not change the dual bound or the termination reason
            passed = expect(solver->getCurrentDualBound() < 6.0, "the dual bound was changed") && passed;
        }
    }

    // The fallback is also used after the search before the dual strategy, for the combinations not solved there. The
    // time limit of the search is zero the first time and changed after it
    for(bool hasTimeInitially : { false, true })
    {
        auto solver = createSolver();
        auto env = solver->getEnvironment();
        solver->updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::MultiTree));
        solver->updateSetting("Termination.DualStagnation.IterationLimit", 0);
        solver->updateSetting("Primal.FixedInteger.Enumeration.UseInitially", true);

        if(!hasTimeInitially)
            solver->updateSetting("Primal.FixedInteger.Enumeration.TimeLimit", 0.0);

        if(!solver->setProblem("data/synthes1.osil"))
            return (expect(false, "could not read the problem"));

        // The callback is also called in the interior point search, which is before the first search
        solver->registerCallback<TerminationCheckContext>(
            [&solver, &env](TerminationCheckContext&)
            {
                if(env->solutionStatistics.hasFixedIntegerEnumerationBeenRun)
                    solver->updateSetting("Primal.FixedInteger.Enumeration.TimeLimit", 30.0);
            });

        if(!solver->solveProblem())
            return (expect(false, "could not solve the problem"));

        printStatistics(env);

        passed = expect(env->solutionStatistics.hasFixedIntegerEnumerationBeenRun, "the search was not run") && passed;
        passed = expect(env->solutionStatistics.hasFixedIntegerEnumerationFallbackBeenRun == !hasTimeInitially,
                     hasTimeInitially ? "the fallback search was run although all combinations had been solved"
                                      : "the fallback search was not run after the search before the dual strategy")
            && passed;
        passed = expect(numberOfCombinationsSolved(env)
                         + env->solutionStatistics.numberOfFixedIntegerEnumerationCombinationsSkipped
                     == 8,
                     "not all combinations were solved or skipped")
            && passed;
    }

    return (passed);
}

bool EnumerationTestMIQCQP([[maybe_unused]] ES_MIPSolver mipSolver)
{
    bool passed = true;

    auto solver = createSolver(mipSolver);
    auto env = solver->getEnvironment();
    solver->updateSetting("Primal.FixedInteger.Enumeration.UseInitially", true);

    if(!solver->setProblem(createConvexQuadraticProblem(env)))
        return (expect(false, "could not set the problem"));

    if(!solver->solveProblem())
        return (expect(false, "could not solve the problem"));

    printStatistics(env);

    passed = expect(env->results->usedSolutionStrategy == E_SolutionStrategy::MIQP,
                 "the problem was not solved with the MIQCQP strategy")
        && passed;
    passed = expect(env->solutionStatistics.hasFixedIntegerEnumerationBeenRun, "the search was not run") && passed;
    passed = expect(env->solutionStatistics.numberOfFixedIntegerEnumerationCombinationsFeasible == 2,
                 "the two combinations were not feasible")
        && passed;
    passed = expect(std::abs(solver->getPrimalBound() - 0.5) < 1e-4, "the objective value is not correct") && passed;

    return (passed);
}

} // namespace

int EnumerationTest(int argc, char* argv[])
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
        std::cout << "Starting test of the exhaustive search for a nonconvex problem:\n";
        passed = EnumerationTestNonconvexDefault();
        break;
    case 2:
        std::cout << "Starting test that no integer cuts are added in the exhaustive search:\n";
        passed = EnumerationTestNoIntegerCuts();
        break;
    case 3:
        std::cout << "Starting test of the limit on the number of combinations in the exhaustive search:\n";
        passed = EnumerationTestMaxCombinations();
        break;
    case 4:
        std::cout << "Starting test of the time limit of the exhaustive search:\n";
        passed = EnumerationTestTimeLimit();
        break;
    case 5:
        std::cout << "Starting test of the exhaustive search for a convex problem with the multi-tree strategy:\n";
        passed = EnumerationTestConvex(ES_TreeStrategy::MultiTree, ES_PrimalNLPProblemSource::ReformulatedProblem);
        break;
    case 6:
        std::cout << "Starting test of the exhaustive search for a convex problem with the single-tree strategy:\n";
        passed = EnumerationTestConvex(ES_TreeStrategy::SingleTree, ES_PrimalNLPProblemSource::Both);
        break;
    case 7:
        std::cout << "Starting test of the exhaustive search as a fallback:\n";
        passed = EnumerationTestFallback();
        break;
    case 8:
        std::cout << "Starting test of the exhaustive search in the MIQCQP strategy with Gurobi:\n";
#ifdef HAS_GUROBI
        passed = EnumerationTestMIQCQP(ES_MIPSolver::Gurobi);
#endif
        break;
    case 9:
        std::cout << "Starting test of the exhaustive search in the MIQCQP strategy with Cplex:\n";
#ifdef HAS_CPLEX
        passed = EnumerationTestMIQCQP(ES_MIPSolver::Cplex);
#endif
        break;
    default:
        passed = false;
        std::cout << "Test #" << choice << " does not exist!\n";
    }

    if(passed)
        return 0;
    else
        return -1;
}
