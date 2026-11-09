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
#include "../src/Report.h"
#include "../src/Results.h"
#include "../src/Solver.h"
#include "../src/TaskHandler.h"
#include "../src/Utilities.h"

#include "../src/MIPSolver/IMIPSolver.h"
#include "../src/MIPSolver/MIPSolverHighs.h"
#include "../src/Tasks/TaskCreateMIPProblem.h"

#include "../src/Model/Problem.h"
#include "../src/Model/ObjectiveFunction.h"
#include "IntegerCutTestCommon.h"

#include <iostream>

using namespace SHOT;

namespace SHOT
{
extern DllExport HighsCallbackFunctionType highsCallback;
}

bool HighsCallbackTimeLimitTest()
{
    auto solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();
    auto backend = std::make_shared<MIPSolverHighs>(env);
    env->dualSolver->MIPSolver = backend;
    HighsMipData data;
    data.env = env;
    data.solveStarted = std::chrono::steady_clock::now() - std::chrono::seconds(2);
    data.timeLimit = 1.0;

    for(int type : { kCallbackSimplexInterrupt, kCallbackIpmInterrupt, kCallbackMipInterrupt })
    {
        HighsCallbackOutput output;
        output.simplex_iteration_count = 1;
        output.ipm_iteration_count = 1;
        HighsCallbackInput input;
        backend->interruptedByTimeLimit = false;
        highsCallback(type, "", &output, &input, &data);
        if(!input.user_interrupt || backend->getSolutionStatus() != E_ProblemSolutionStatus::TimeLimit
            || backend->interruptedByTermination || backend->interruptedBySolutionLimit)
            return false;
    }

    // A fresh budget must not interrupt the next solve or be confused with a solution/user limit.
    data.solveStarted = std::chrono::steady_clock::now();
    data.timeLimit = 10.0;
    HighsCallbackOutput output;
    output.simplex_iteration_count = 1;
    HighsCallbackInput input;
    backend->interruptedByTimeLimit = false;
    highsCallback(kCallbackSimplexInterrupt, "", &output, &input, &data);
    return !input.user_interrupt && !backend->interruptedByTimeLimit;
}

bool HighsLargeIntegerDomainWithoutPresolveTest()
{
    for(bool presolve : { false, true })
    {
        auto solver = std::make_unique<Solver>();
        auto env = solver->getEnvironment();
        solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Highs));
        solver->updateSetting("Dual.MIP.NumberOfThreads", 1);
        solver->updateSetting("Subsolver.Highs.Presolve", presolve ? 1 : 0);
        solver->updateSetting("Model.BoundTightening.FeasibilityBased.Use", false);
        solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
        auto problem = std::make_shared<Problem>(env);
        double upper = static_cast<double>(std::numeric_limits<HighsInt>::max());
        auto x = std::make_shared<Variable>("x", E_VariableType::Integer, 0.0, upper);
        problem->add(x);
        auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        objective->add(std::make_shared<LinearTerm>(1.0, x));
        problem->add(objective);
        problem->finalize();
        if(!solver->setProblem(problem))
            return false;
        auto backend = std::make_shared<MIPSolverHighs>(env);
        env->dualSolver->MIPSolver = backend;
        if(!backend->initializeProblem())
            return false;

        TaskCreateMIPProblem(env, backend, problem, MIPSolverHighs::capabilities).run();
        backend->setTimeLimit(1.0);
        backend->setSolutionLimit(2100000000);
        auto status = backend->solveProblem();
        if(presolve)
        {
            if(status != E_ProblemSolutionStatus::Optimal || std::abs(backend->getObjectiveValue()) > 1e-6)
                return false;
        }
        else if(status != E_ProblemSolutionStatus::Numeric || backend->getNumberOfSolutions() != 0
            || backend->getDualObjectiveValue() != SHOT_DBL_MIN)
            return false;
        if(backend->getCurrentVariableBounds(0).second != upper)
            return false;
    }
    return true;
}

// The infeasibility repair must work for maximization problems, and when the objective is unbounded with the
// repairable cuts relaxed
bool HighsRepairInfeasibilityTest()
{
    for(bool isMinimization : { false, true })
    {
        Solver solver;
        auto env = solver.getEnvironment();
        env->reformulatedProblem = std::make_shared<Problem>(env);
        env->results->createIteration();
        // The repair requires a generated cut to have been recorded
        env->dualSolver->generatedHyperplanes.push_back(std::make_shared<GeneratedHyperplane>());

        MIPSolverHighs mip(env);
        mip.initializeProblem();
        // Maximize an integer variable in [0, 10], or minimize a free variable
        if(isMinimization)
            mip.addVariable("x", E_VariableType::Real, -mip.getUnboundedVariableBoundValue(),
                mip.getUnboundedVariableBoundValue(), 0.0);
        else
            mip.addVariable("x", E_VariableType::Integer, 0.0, 10.0, 0.0);

        mip.addVariable("y", E_VariableType::Binary, 0.0, 1.0, 0.0);
        mip.initializeObjective();
        mip.addLinearTermToObjective(1.0, 0);
        mip.finalizeObjective(isMinimization);
        mip.finalizeProblem();
        mip.setSolutionLimit(2100000000);
        mip.setTimeLimit(10.0);

        // The repairable cuts x >= 2 and x <= 1 contradict each other
        std::map<int, double> terms = { { 0, 1.0 } };
        mip.addLinearConstraint(terms, -2.0, "lower", true, true);
        mip.addLinearConstraint(terms, 1.0, "upper", false, true);

        if(mip.solveProblem() != E_ProblemSolutionStatus::Infeasible)
        {
            std::cout << "The problem with the contradicting cuts is not infeasible.\n";
            return (false);
        }

        if(!mip.repairInfeasibility())
        {
            std::cout << "The infeasibility repair failed for a " << (isMinimization ? "minimization" : "maximization")
                      << " problem.\n";
            return (false);
        }

        if(env->results->getCurrentIteration()->numberOfInfeasibilityRepairedConstraints == 0)
        {
            std::cout << "No constraint was relaxed by the infeasibility repair.\n";
            return (false);
        }

        auto status = mip.solveProblem();

        if(status != E_ProblemSolutionStatus::Optimal)
        {
            std::cout << "The repaired problem is not solved to optimality, status " << static_cast<int>(status)
                      << ".\n";
            return (false);
        }
    }

    return (true);
}

bool HighsTest1(std::string filename, double correctObjectiveValue)
{
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Highs));

    try
    {
        if(solver->setProblem(filename))
        {
            passed = true;
        }
        else
        {
            return false;
        }
    }
    catch(Exception& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        return false;
    }

    solver->solveProblem();
    std::string osrl = solver->getResultsOSrL();
    std::string trace = solver->getResultsTrace();
    if(!Utilities::writeStringToFile("result.osrl", osrl))
    {
        std::cout << "Could not write results to OSrL file." << std::endl;
        passed = false;
    }

    if(!Utilities::writeStringToFile("trace.trc", trace))
    {
        std::cout << "Could not write results to trace file." << std::endl;
        passed = false;
    }

    if(solver->getPrimalSolutions().size() > 0)
    {
        std::cout << std::endl << "Objective value: " << solver->getPrimalSolution().objValue << std::endl;
    }
    else
    {
        passed = false;
    }

    if(solver->getOriginalProblem()->objectiveFunction->properties.isMinimize)
    {
        if(correctObjectiveValue <= solver->getPrimalBound() + 1e-5
            && correctObjectiveValue >= solver->getGlobalDualBound() - 1e-5)
        {
            std::cout << std::endl
                      << "Global objective value is within dual and primal bounds for minimization problem."
                      << std::endl;
        }
        else
        {
            std::cout << std::endl
                      << "Global objective value is not within dual and primal bounds for minimization problem."
                      << std::endl;
            passed = false;
        }
    }
    else
    {
        if(correctObjectiveValue >= solver->getPrimalBound() - 1e-5
            && correctObjectiveValue <= solver->getGlobalDualBound() + 1e-5)
        {
            std::cout << std::endl
                      << "Global objective value " << correctObjectiveValue
                      << " is within primal and dual bounds for maximization problem." << std::endl;
        }
        else
        {
            std::cout << std::endl
                      << "Global objective value is not within primal and dual bounds for maximization problem."
                      << std::endl;
            passed = false;
        }
    }

    return passed;
}

bool HighsTerminationCallbackTest(std::string filename)
{
    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Error));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Highs));
    solver->updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::MultiTree));

    std::cout << "Reading problem:  " << filename << '\n';

    if(!solver->setProblem(filename))
    {
        std::cout << "Error while reading problem";
        return (false);
    }

    // Registers a callback that terminates in the third iteration
    solver->registerCallback<TerminationCheckContext>(
        [](TerminationCheckContext& context)
        {
            std::cout << "Callback activated at iteration " << context.getIterationNumber() << ".\n";
            if(context.getIterationNumber() == 3)
            {
                std::cout << "Terminating.\n";
                context.terminate();
            }
        });

    // Solving the problem
    if(!solver->solveProblem())
    {
        std::cout << "Error while solving problem\n";
        return (false);
    }

    if(env->results->getNumberOfIterations() != 3)
    {
        std::cout << "Termination callback did not seem to work as expected\n";
        return (false);
    }

    if(env->results->terminationReason != E_TerminationReason::UserAbort)
    {
        std::cout << "Termination reason is not UserAbort as expected\n";
        return (false);
    }

    return (true);
}

bool HighsExternalDualBoundCallbackTest(std::string filename, double dualBoundToTest, ES_TreeStrategy treeStrategy)
{
    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Highs));
    solver->updateSetting("Dual.TreeStrategy", static_cast<int>(treeStrategy));

    std::cout << "Reading problem:  " << filename << '\n';

    if(!solver->setProblem(filename))
    {
        std::cout << "Error while reading problem";
        return (false);
    }

    // Vector to collect all primal solutions found during optimization
    std::vector<VectorDouble> foundSolutions;

    // Registers a callback that collects all new primal solutions
    solver->registerCallback<NewPrimalSolutionContext>(
        [&foundSolutions](NewPrimalSolutionContext& solution)
        {
            std::cout << "New primal solution found with objective value: " << solution.getObjectiveValue()
                      << " from source: " << static_cast<int>(solution.getSource()) << " (iteration "
                      << solution.getIterationNumber() << ")\n";

            foundSolutions.push_back(solution.getPoint());
        });

    if(!solver->solveProblem())
    {
        std::cout << "Error while solving problem\n";
        return (false);
    }

    std::cout << "Total solutions collected: " << foundSolutions.size() << "\n";

    // Create a new solver instance
    solver = std::make_unique<Solver>();
    env = solver->getEnvironment();

    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Highs));
    solver->updateSetting("Dual.TreeStrategy", static_cast<int>(treeStrategy));

    std::cout << "Reading problem:  " << filename << '\n';

    if(!solver->setProblem(filename))
    {
        std::cout << "Error while reading problem";
        return (false);
    }

    // Registers a callback that sets the dual bound to a fixed value
    solver->registerCallback<DualBoundUpdateContext>(
        [dualBoundToTest](DualBoundUpdateContext& context)
        {
            if(context.isMinimization() ? context.getDualBound() >= dualBoundToTest
                                        : context.getDualBound() <= dualBoundToTest)
                return;

            std::cout << "Current dual bound is " << context.getDualBound()
                      << ", new external dual bound given as = " << dualBoundToTest << "\n";
            context.setDualBound(dualBoundToTest);
        });

    // Registers a callback that provides external primal solutions from our collected solutions
    solver->registerCallback<PrimalCandidateSearchContext>(
        [&foundSolutions, &env](PrimalCandidateSearchContext& context)
        {
            if(!env->dualSolver->MIPSolver->getDiscreteVariableStatus())
            {
                std::cout
                    << "Still waiting to add primal solution candidates until the relaxation strategy is finished.\n";
                return;
            }

            std::cout << "External primal solution callback requested (iteration " << context.getIterationNumber()
                      << ", current gap: " << context.getRelativeGap() << ")\n";

            if(!foundSolutions.empty())
            {
                std::cout << "Providing " << foundSolutions.size() << " collected solutions as external candidates\n";

                for(auto& solution : foundSolutions)
                    context.addPrimalSolution(solution);

                foundSolutions.clear();
            }
            else
            {
                std::cout << "No collected solutions available to provide\n";
            }
        });

    if(!solver->solveProblem())
    {
        std::cout << "Error while solving problem\n";
        return (false);
    }

    env->report->outputSolutionReport();

    if(env->solutionStatistics.hasExternalDualBoundBeenSet)
    {
        std::cout << "External dual bound callback was executed successfully.\n";
    }
    else
    {
        std::cout << "External dual bound callback was not executed as expected.\n";
        return (false);
    }

    return (true);
}

bool HighsSchedulerChangedDuringSolveTest(std::string filename, std::string otherFilename, double correctObjectiveValue)
{
    // HiGHS has one scheduler of threads for the process, and a solve that asks for another number of threads than it
    // was created with fails. A solver whose number of threads was set when its problem was set up therefore failed
    // when another solver in the process, e.g., a nested SHOT solver for the fixed-integer NLP problems, used another
    // number of threads while it was solving: its next MIP problem was reported as a numerical issue. Here another
    // solver with two threads is run from a callback of the solver with one thread.

    auto solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Error));
    solver->updateSetting("Termination.TimeLimit", 60.0);
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Highs));
    solver->updateSetting("Dual.MIP.NumberOfThreads", 1);
    solver->updateSetting("Primal.FixedInteger.Use", false);

    if(!solver->setProblem(filename))
    {
        std::cout << "Error while reading problem " << filename << '\n';
        return (false);
    }

    bool isOtherSolved = false;

    solver->registerCallback<TerminationCheckContext>(
        [&](TerminationCheckContext& context)
        {
            if(isOtherSolved || context.getIterationNumber() < 2)
                return;

            auto otherSolver = std::make_unique<Solver>();
            otherSolver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
            otherSolver->updateSetting("Termination.TimeLimit", 10.0);
            otherSolver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Highs));
            otherSolver->updateSetting("Dual.MIP.NumberOfThreads", 2);

            isOtherSolved = otherSolver->setProblem(otherFilename) && otherSolver->solveProblem();
        });

    solver->solveProblem();

    if(!isOtherSolved)
    {
        std::cout << "The other problem " << otherFilename << " was not solved from the callback.\n";
        return (false);
    }

    if(!solver->hasPrimalSolution()
        || std::abs(solver->getPrimalSolution().objValue - correctObjectiveValue)
            > 1e-3 * std::abs(correctObjectiveValue))
    {
        std::cout << "The problem was not solved to the objective " << correctObjectiveValue
                  << ", terminated with: " << env->results->terminationReasonDescription << '\n';
        return (false);
    }

    std::cout << "Solved to the objective " << solver->getPrimalSolution().objValue << " with the problem "
              << otherFilename << " solved in between.\n";

    return (true);
}

int HighsTest(int argc, char* argv[])
{
    int defaultchoice = 1;

    int choice = defaultchoice;

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
        std::cout << "Starting test to solve a MINLP problem with Highs." << std::endl;
        passed = HighsTest1("data/tls2.osil", 5.3);
        std::cout << "Finished test to solve a MINLP problem with Highs." << std::endl;
        break;
    case 2:
        std::cout << "Starting test to check termination callback in Highs:" << std::endl;
        passed = HighsTerminationCallbackTest("data/tls2.osil");
        std::cout << "Finished test checking termination callback in Highs." << std::endl;
        break;
    case 3:
        std::cout << "Starting test to solve gear.osil with Highs." << std::endl;
        passed = solveWithIntegerCuts(ES_MIPSolver::Highs, "data/gear.osil", 0.0);
        std::cout << "Finished test to solve gear.osil with Highs." << std::endl;
        break;
    case 4:
        std::cout << "Starting test to solve windfac.osil with Highs." << std::endl;
        passed = solveWithIntegerCuts(ES_MIPSolver::Highs, "data/windfac.osil", 0.254487);
        std::cout << "Finished test to solve windfac.osil with Highs." << std::endl;
        break;
    case 5:
        std::cout << "Starting test to solve ex1252a.osil with Highs." << std::endl;
        passed = solveWithIntegerCuts(ES_MIPSolver::Highs, "data/ex1252a.osil", 128893.741);
        std::cout << "Finished test to solve ex1252a.osil with Highs." << std::endl;
        break;
    case 6:
        std::cout << "Starting test to solve problem with semicont. variables:" << std::endl;
        passed = HighsTest1("data/meanvarxsc.osil", 14.36923211);
        std::cout << "Finished test to solve problem with semicont. variables." << std::endl;
        break;
    case 7:
        std::cout << "Starting test to solve nonconvex maximization problem 'ncvx_max_div.nl':" << std::endl;
        passed = HighsTest1("data/ncvx_max_div.nl", 13.0);
        std::cout << "Finished test to solve nonconvex maximization problem 'ncvx_max_div.nl'." << std::endl;
        break;
    case 8:
        std::cout << "Starting test to solve nonconvex maximization problem 'ncvx_min_div.nl':" << std::endl;
        passed = HighsTest1("data/ncvx_min_div.nl", -13.0);
        std::cout << "Finished test to solve nonconvex maximization problem 'ncvx_min_div.nl'." << std::endl;
        break;
    case 9:
        std::cout << "Starting test to solve nonconvex maximization problem 'ncvx_max_ndiv.nl':" << std::endl;
        passed = HighsTest1("data/ncvx_max_ndiv.nl", 13.0);
        std::cout << "Finished test to solve nonconvex maximization problem 'ncvx_max_ndiv.nl'." << std::endl;
        break;
    case 10:
        std::cout << "Starting test to solve nonconvex maximization problem 'ncvx_min_ndiv.nl':" << std::endl;
        passed = HighsTest1("data/ncvx_min_ndiv.nl", -13.0);
        std::cout << "Finished test to solve nonconvex maximization problem 'ncvx_min_ndiv.nl'." << std::endl;
        break;
    case 11:
        std::cout << "Starting test for callbacks getting and setting primal solutions and dual bounds through a "
                     "callback with multi-tree strategy";
        passed = HighsExternalDualBoundCallbackTest("data/synthes1.osil", 5.0, ES_TreeStrategy::MultiTree);
        std::cout << "Finished test for callbacks getting and setting primal solutions and dual bounds through a "
                     "callback with multi-tree strategy."
                  << std::endl;
        break;
    case 12:
        std::cout << "Starting test with another number of threads in HiGHS during a solve." << std::endl;
        passed = HighsSchedulerChangedDuringSolveTest("data/synthes1.osil", "data/tls2.osil", 6.00976);
        std::cout << "Finished test with another number of threads in HiGHS during a solve." << std::endl;
        break;
    case 13:
        std::cout << "Testing HiGHS callback time limits before iteration throttling." << std::endl;
        passed = HighsCallbackTimeLimitTest();
        break;
    case 14:
        std::cout << "Testing unsafe integer domains without HiGHS presolve." << std::endl;
        passed = HighsLargeIntegerDomainWithoutPresolveTest();
        break;
    case 15:
        std::cout << "Testing the HiGHS infeasibility repair for minimization and maximization problems." << std::endl;
        passed = HighsRepairInfeasibilityTest();
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
