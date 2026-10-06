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
#include "../src/Utilities.h"
#include "../src/TaskHandler.h"

#include "../src/MIPSolver/IMIPSolver.h"
#include "../src/MIPSolver/MIPSolverCplex.h"

#include "../src/Model/Problem.h"
#include "../src/Model/ObjectiveFunction.h"
#include "../src/Tasks/TaskCreateMIPProblem.h"

#include <iostream>

using namespace SHOT;

bool CplexTest1(std::string filename, double correctObjectiveValue)
{
    bool passed = true;

    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Cplex));

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
            && correctObjectiveValue >= solver->getCurrentDualBound() - 1e-5)
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
            && correctObjectiveValue <= solver->getCurrentDualBound() + 1e-5)
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

// For nonconvex problems where finding a primal solution is not guaranteed;
// only verifies the solver terminates without crashing.
bool CplexTestNocrash(std::string filename)
{
    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Cplex));

    try
    {
        if(!solver->setProblem(filename))
            return false;
    }
    catch(SHOT::Exception& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        return false;
    }

    solver->solveProblem();

    return true;
}

bool CplexTerminationCallbackTest(std::string filename)
{
    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Error));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Cplex));
    solver->updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::MultiTree));

    std::cout << "Reading problem:  " << filename << '\n';

    if(!solver->setProblem(filename))
    {
        std::cout << "Error while reading problem";
        return (false);
    }

    // Registers a callback that terminates after the third iteration
    solver->registerCallback<TerminationCheckContext>(
        [](TerminationCheckContext& context)
        {
            std::cout << "Termination callback activated (iteration " << context.getIterationNumber() << ")\n";

            if(context.getIterationNumber() > 3)
            {
                std::cout << "Terminating after iteration " << context.getIterationNumber() << "\n";
                context.terminate();
            }
        });

    // Solving the problem
    if(!solver->solveProblem())
    {
        std::cout << "Error while solving problem\n";
        return (false);
    }

    if(env->results->terminationReason != E_TerminationReason::UserAbort)
    {
        std::cout << "Termination callback did not seem to work as expected\n";
        return (false);
    }

    return (true);
}

bool CplexTerminationCallbackSingleTreeTest(std::string filename)
{
    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Cplex));
    solver->updateSetting("Output.Console.Iteration.Detail", static_cast<int>(ES_IterationOutputDetail::Full));
    solver->updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::SingleTree));

    if(!solver->setProblem(filename))
    {
        std::cout << "Error while reading problem";
        return (false);
    }

    solver->registerCallback<TerminationCheckContext>(
        [](TerminationCheckContext& context)
        {
            if(context.getIterationNumber() > 10)
            {
                std::cout << "Terminating after iteration " << context.getIterationNumber() << "\n";
                context.terminate();
            }
        });

    if(!solver->solveProblem())
    {
        std::cout << "Error while solving problem\n";
        return (false);
    }

    if(env->results->terminationReason != E_TerminationReason::UserAbort)
    {
        std::cout << "Termination callback did not terminate the single-tree solve as expected\n";
        return (false);
    }

    return (true);
}

bool CplexExternalPrimalSolutionSingleTreeTest(std::string filename)
{
    // Phase 1: collect primal solution points
    std::vector<VectorDouble> collectedSolutions;

    {
        std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

        solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
        solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Cplex));
        solver->updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::SingleTree));

        if(!solver->setProblem(filename))
        {
            std::cout << "Error while reading problem in phase 1\n";
            return (false);
        }

        solver->registerCallback<NewPrimalSolutionContext>([&collectedSolutions](NewPrimalSolutionContext& solution)
            { collectedSolutions.push_back(solution.getPoint()); });

        if(!solver->solveProblem())
        {
            std::cout << "Error while solving problem in phase 1\n";
            return (false);
        }
    }

    if(collectedSolutions.empty())
    {
        std::cout << "No primal solutions collected in phase 1, cannot proceed\n";
        return (false);
    }

    std::cout << "Phase 1 collected " << collectedSolutions.size() << " primal solution(s)\n";

    // Phase 2: solve with CPLEX single-tree, injecting the collected solutions via callback
    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Cplex));
    solver->updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::SingleTree));

    if(!solver->setProblem(filename))
    {
        std::cout << "Error while reading problem in phase 2\n";
        return (false);
    }

    solver->registerCallback<PrimalCandidateSearchContext>(
        [&collectedSolutions, &env](PrimalCandidateSearchContext& context)
        {
            if(!env->dualSolver->MIPSolver->getDiscreteVariableStatus())
                return;

            if(collectedSolutions.empty())
                return;

            std::cout << "Injecting " << collectedSolutions.size() << " external primal solution(s)\n";

            for(auto& solution : collectedSolutions)
                context.addPrimalSolution(solution);

            collectedSolutions.clear();
        });

    if(!solver->solveProblem())
    {
        std::cout << "Error while solving problem in phase 2\n";
        return (false);
    }

    bool foundExternalSolution
        = env->results->primalSolutionSourceStatistics.count(E_PrimalSolutionSource::ExternalPrimalSolution) > 0;

    if(foundExternalSolution)
    {
        int count
            = env->results->primalSolutionSourceStatistics.at(E_PrimalSolutionSource::ExternalPrimalSolution);
        std::cout << count << " external primal solution(s) were accepted by the solver\n";
    }

    if(!foundExternalSolution)
    {
        std::cout << "No external primal solution was accepted by the solver\n";
        return (false);
    }

    return (true);
}

bool CplexExternalDualBoundLazyConstraintTest(std::string filename, double externalDualBound)
{
    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Cplex));
    solver->updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::SingleTree));

    std::cout << "Reading problem:  " << filename << '\n';

    if(!solver->setProblem(filename))
    {
        std::cout << "Error while reading problem\n";
        return (false);
    }

    // Register a callback that raises the external dual bound by 0.1 on each call,
    // starting from (externalDualBound - 0.4) up to externalDualBound, exercising the
    // incremental lazy constraint mechanism.
    double currentExternalBound = externalDualBound - 0.4;
    solver->registerCallback<DualBoundUpdateContext>(
        [externalDualBound, &currentExternalBound](DualBoundUpdateContext& context)
        {
            if(currentExternalBound < externalDualBound)
            {
                currentExternalBound = std::min(currentExternalBound + 0.1, externalDualBound);
                std::cout << "Current dual bound is " << context.getDualBound()
                          << ", providing external dual bound: " << currentExternalBound << "\n";
                context.setDualBound(currentExternalBound);
            }
        });

    if(!solver->solveProblem())
    {
        std::cout << "Error while solving problem\n";
        return (false);
    }

    env->report->outputSolutionReport();

    if(!env->solutionStatistics.hasExternalDualBoundBeenSet)
    {
        std::cout << "External dual bound was never applied.\n";
        return (false);
    }

    double finalDualBound = solver->getCurrentDualBound();
    bool isMin = solver->getOriginalProblem()->objectiveFunction->properties.isMinimize;
    bool boundRespected = isMin ? (finalDualBound >= externalDualBound - 1e-6)
                                : (finalDualBound <= externalDualBound + 1e-6);

    if(!boundRespected)
    {
        std::cout << "Final dual bound " << finalDualBound
                  << " does not respect the provided external bound " << externalDualBound << "\n";
        return (false);
    }

    std::cout << "External dual bound lazy constraint was enforced successfully. "
              << "Final dual bound: " << finalDualBound << "\n";
    return (true);
}

bool CplexExternalDualBoundCallbackTest(std::string filename, double dualBoundToTest, ES_TreeStrategy treeStrategy)
{
    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Cplex));
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

            // Add the solution to our collection
            foundSolutions.push_back(solution.getPoint());
        });

    // Solving the problem
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
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Cplex));
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
            if(context.getDualBound() >= dualBoundToTest)
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

// A nonconvex objective with a quadratic constraint makes CPLEX throw 5002, including on the retry with
// OptimalityTarget=3. The termination callback must be released before another solve or backend destruction.
bool CplexCallbackAfterSolveErrorTest(bool quadraticConstraint)
{
    auto solver = std::make_unique<Solver>();
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Cplex));
    solver->updateSetting("Dual.MIP.NumberOfThreads", 1);
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
    auto env = solver->getEnvironment();

    auto problem = std::make_shared<Problem>(env);
    auto x = std::make_shared<Variable>("x", E_VariableType::Real, -1.0, 1.0);
    auto b = std::make_shared<Variable>("b", E_VariableType::Binary, 0.0, 1.0);
    problem->add({ x, b });
    auto objective = std::make_shared<QuadraticObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    objective->add(std::make_shared<QuadraticTerm>(-1.0, x, x));
    objective->add(std::make_shared<LinearTerm>(-1.0, b));
    problem->add(objective);

    if(quadraticConstraint)
    {
        auto constraint = std::make_shared<QuadraticConstraint>("ball", SHOT_DBL_MIN, 1.0);
        constraint->add(std::make_shared<QuadraticTerm>(1.0, x, x));
        problem->add(constraint);
    }

    problem->finalize();
    if(!solver->setProblem(problem))
        return false;

    auto backend = std::make_shared<MIPSolverCplex>(env);
    if(!backend->initializeProblem())
        return false;

    // Give the original quadratic model to CPLEX so SHOT's reformulation does not remove the error trigger.
    TaskCreateMIPProblem(env, backend, problem).run();
    backend->setTimeLimit(10.0);
    backend->setSolutionLimit(2100000000);

    for(int solve = 0; solve < 2; ++solve)
    {
        auto status = backend->solveProblem();
        if(quadraticConstraint)
        {
            if(status != E_ProblemSolutionStatus::Error || backend->getDualObjectiveValue() != SHOT_DBL_MIN)
                return false;
        }
        else if(status != E_ProblemSolutionStatus::Optimal || std::abs(backend->getObjectiveValue() + 2.0) > 1e-6
            || std::abs(backend->getDualObjectiveValue() + 2.0) > 1e-6)
        {
            return false;
        }
    }

    backend.reset(); // Previously asserted in IloCplex::remove() after the unsuccessful solve.
    return true;
}

int CplexTest(int argc, char* argv[])
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
        std::cout << "Starting test to solve a MINLP problem with Cplex:" << std::endl;
        passed = CplexTest1("data/tls2.osil", 5.3);
        std::cout << "Finished test to solve a MINLP problem with Cplex." << std::endl;
        break;
    case 2:
        std::cout << "Starting test to check termination callback in Cplex:" << std::endl;
        passed = CplexTerminationCallbackTest("data/tls2.osil");
        std::cout << "Finished test checking termination callback in Cplex." << std::endl;
        break;
    case 3:
        std::cout << "Starting test to solve problem with semicont. variables with Cplex:" << std::endl;
        passed = CplexTest1("data/meanvarxsc.osil", 14.36923211);
        std::cout << "Finished test to solve problem with semicont. variables with Cplex." << std::endl;
        break;
    case 4:
        std::cout << "Starting test to solve nonconvex maximization problem 'ncvx_max_div.nl':" << std::endl;
        passed = CplexTest1("data/ncvx_max_div.nl", 13.0);
        std::cout << "Finished test to solve nonconvex maximization problem 'ncvx_max_div.nl'." << std::endl;
        break;
    case 5:
        std::cout << "Starting test to solve nonconvex maximization problem 'ncvx_min_div.nl':" << std::endl;
        passed = CplexTest1("data/ncvx_min_div.nl", -13.0);
        std::cout << "Finished test to solve nonconvex maximization problem 'ncvx_min_div.nl'." << std::endl;
        break;
    case 6:
        std::cout << "Starting test to solve nonconvex maximization problem 'ncvx_max_ndiv.nl':" << std::endl;
        passed = CplexTest1("data/ncvx_max_ndiv.nl", 13.0);
        std::cout << "Finished test to solve nonconvex maximization problem 'ncvx_max_ndiv.nl'." << std::endl;
        break;
    case 7:
        std::cout << "Starting test to solve nonconvex maximization problem 'ncvx_min_ndiv.nl':" << std::endl;
        passed = CplexTest1("data/ncvx_min_ndiv.nl", -13.0);
        std::cout << "Finished test to solve nonconvex maximization problem 'ncvx_min_ndiv.nl'." << std::endl;
        break;
    case 8:
        std::cout << "Starting test for callbacks getting and setting primal solutions and dual bounds through a "
                     "callback with multi-tree strategy";
        passed = CplexExternalDualBoundCallbackTest("data/synthes1.osil", 5.0, ES_TreeStrategy::MultiTree);
        std::cout << "Finished test for callbacks getting and setting primal solutions and dual bounds through a "
                     "callback with multi-tree strategy.";
        break;
    case 9:
        std::cout << "Starting test for callbacks getting and setting primal solutions and dual bounds through a "
                     "callback with single-tree strategy";
        passed = CplexExternalDualBoundCallbackTest("data/synthes1.osil", 5.0, ES_TreeStrategy::SingleTree);
        std::cout << "Finished test for callbacks getting and setting primal solutions and dual bounds through a "
                     "callback with single-tree strategy."
                  << std::endl;
        break;
    case 10:
        std::cout << "Starting test for external dual bound lazy constraint in single-tree strategy" << std::endl;
        passed = CplexExternalDualBoundLazyConstraintTest("data/fo7_2.osil", 17.4);
        std::cout << "Finished test for external dual bound lazy constraint in single-tree strategy.";
        break;
    case 11:
        std::cout << "Starting test for external primal solution injection in single-tree strategy" << std::endl;
        passed = CplexExternalPrimalSolutionSingleTreeTest("data/synthes1.osil");
        std::cout << "Finished test for external primal solution injection in single-tree strategy.";
        break;
    case 12:
        std::cout << "Starting test for termination callback in single-tree strategy" << std::endl;
        passed = CplexTerminationCallbackSingleTreeTest("data/synthes1.osil");
        std::cout << "Finished test for termination callback in single-tree strategy.";
        break;
    case 13:
        std::cout << "Starting test to solve gear.osil with Cplex." << std::endl;
        passed = CplexTestNocrash("data/gear.osil");
        std::cout << "Finished test to solve gear.osil with Cplex." << std::endl;
        break;
    case 14:
        std::cout << "Starting test to solve windfac.osil with Cplex." << std::endl;
        passed = CplexTestNocrash("data/windfac.osil");
        std::cout << "Finished test to solve windfac.osil with Cplex." << std::endl;
        break;
    case 15:
        std::cout << "Starting test to solve ex1252a.osil with Cplex." << std::endl;
        passed = CplexTestNocrash("data/ex1252a.osil");
        std::cout << "Finished test to solve ex1252a.osil with Cplex." << std::endl;
        break;
    case 16:
        std::cout << "Testing Cplex callback cleanup after a rejected nonconvex MIQCP.\n";
        passed = CplexCallbackAfterSolveErrorTest(true);
        break;
    case 17:
        std::cout << "Testing Cplex callback cleanup when retrying a nonconvex MIQP.\n";
        passed = CplexCallbackAfterSolveErrorTest(false);
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
