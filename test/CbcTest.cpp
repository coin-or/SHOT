/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "../src/CallbackData.h"
#include "../src/DualSolver.h"
#include "../src/Environment.h"
#include "../src/Report.h"
#include "../src/Results.h"
#include "../src/Solver.h"
#include "../src/Utilities.h"
#include "../src/TaskHandler.h"

#include "../src/MIPSolver/IMIPSolver.h"

#include "../src/Model/Problem.h"
#include "../src/Model/ObjectiveFunction.h"

#include <iostream>

using namespace SHOT;

bool CbcTest1(std::string filename, double correctObjectiveValue)
{
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Cbc));

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

bool CbcTerminationCallbackTest(std::string filename)
{
    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Error));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Cbc));
    solver->updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::MultiTree));

    std::cout << "Reading problem:  " << filename << '\n';

    if(!solver->setProblem(filename))
    {
        std::cout << "Error while reading problem";
        return (false);
    }

    // Registers a callback that terminates after the third iteration
    solver->registerCallback(E_EventType::UserTerminationCheck, [](std::any args) -> bool {
        auto data = std::any_cast<TerminationCallbackData>(args);
        std::cout << "Termination callback activated with structured data (iteration " << data.iterationNumber << ")\n";

        if(data.iterationNumber > 3)
        {
            std::cout << "Terminating after iteration " << data.iterationNumber << "\n";
            return true;
        }

        return false;
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

bool CbcExternalDualBoundCallbackTest(std::string filename, double dualBoundToTest, ES_TreeStrategy treeStrategy)
{
    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Cbc));
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
    solver->registerCallback(E_EventType::NewPrimalSolution, [&foundSolutions](std::any args) {
        try
        {
            auto data = std::any_cast<PrimalSolutionCallbackData>(args);
            std::cout << "New primal solution found with objective value: " << data.objectiveValue
                      << " from source: " << static_cast<int>(data.sourceType) << " (iteration " << data.iterationNumber
                      << ")\n";

            // Add the solution to our collection
            foundSolutions.push_back(data.solution);
        }
        catch(const std::bad_any_cast&)
        {
            std::cout << "New primal solution callback executed with no valid structured data\n";
        }
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

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Gurobi));
    solver->updateSetting("Dual.TreeStrategy", static_cast<int>(treeStrategy));

    std::cout << "Reading problem:  " << filename << '\n';

    if(!solver->setProblem(filename))
    {
        std::cout << "Error while reading problem";
        return (false);
    }

    // Registers a callback that sets the dual bound to a fixed value
    solver->registerCallback(E_EventType::ExternalDualBound, [dualBoundToTest](std::any args) {
        double newDualBound = std::numeric_limits<double>::quiet_NaN();

        try
        {
            auto data = std::any_cast<DualBoundCallbackData>(args);

            if(data.currentDualBound >= dualBoundToTest)
                return (newDualBound);

            newDualBound = dualBoundToTest;
            std::cout << "Current dual bound is " << data.currentDualBound
                      << ", new external dual bound given as = " << newDualBound << "\n";
        }
        catch(const std::bad_any_cast&)
        {
            std::cout << "External dual bound callback executed with no valid structured data\n";
            throw std::runtime_error("Invalid data type for external dual bound callback");
        }

        return (newDualBound);
    });

    // Registers a callback that provides external primal solutions from our collected solutions
    solver->registerCallback(
        E_EventType::ExternalPrimalSolution, [&foundSolutions, &env](std::any args) -> std::vector<VectorDouble> {
            if(!env->dualSolver->MIPSolver->getDiscreteVariableStatus())
            {
                std::cout
                    << "Still waiting to add primal solution candidates until the relaxation strategy is finished.\n";
                return std::vector<VectorDouble>();
            }

            try
            {
                auto data = std::any_cast<ExternalPrimalSolutionCallbackData>(args);
                std::cout << "External primal solution callback requested (iteration " << data.iterationNumber
                          << ", current gap: " << data.relativeGap << ")\n";

                if(!foundSolutions.empty())
                {
                    std::cout << "Providing " << foundSolutions.size()
                              << " collected solutions as external candidates\n";

                    // Create a copy of foundSolutions and clear the original
                    std::vector<VectorDouble> solutionsToAdd = foundSolutions;
                    foundSolutions.clear();

                    return solutionsToAdd;
                }
                else
                {
                    std::cout << "No collected solutions available to provide\n";
                    return std::vector<VectorDouble>();
                }
            }
            catch(const std::bad_any_cast&)
            {
                std::cout << "External primal solution callback executed with no valid structured data\n";
                return std::vector<VectorDouble>();
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

int CbcTest(int argc, char* argv[])
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
        std::cout << "Starting test to solve a MINLP problem with Cbc." << std::endl;
        passed = CbcTest1("data/tls2.osil", 5.3);
        std::cout << "Finished test to solve a MINLP problem with Cbc." << std::endl;
        break;
    case 2:
        std::cout << "Starting test to check termination callback in Cbc:" << std::endl;
        passed = CbcTerminationCallbackTest("data/tls2.osil");
        std::cout << "Finished test checking termination callback in Cbc." << std::endl;
        break;
    case 3:
        std::cout << "Starting test to solve problem with semicont. variables:" << std::endl;
        passed = CbcTest1("data/meanvarxsc.osil", 14.36923211);
        std::cout << "Finished test to solve problem with semicont. variables." << std::endl;
        break;
    case 4:
        std::cout << "Starting test to solve nonconvex maximization problem 'ncvx_max_div.nl':" << std::endl;
        passed = CbcTest1("data/ncvx_max_div.nl", 13.0);
        std::cout << "Finished test to solve nonconvex maximization problem 'ncvx_max_div.nl'." << std::endl;
        break;
    case 5:
        std::cout << "Starting test to solve nonconvex maximization problem 'ncvx_min_div.nl':" << std::endl;
        passed = CbcTest1("data/ncvx_min_div.nl", -13.0);
        std::cout << "Finished test to solve nonconvex maximization problem 'ncvx_min_div.nl'." << std::endl;
        break;
    case 6:
        std::cout << "Starting test to solve nonconvex maximization problem 'ncvx_max_ndiv.nl':" << std::endl;
        passed = CbcTest1("data/ncvx_max_ndiv.nl", 13.0);
        std::cout << "Finished test to solve nonconvex maximization problem 'ncvx_max_ndiv.nl'." << std::endl;
        break;
    case 7:
        std::cout << "Starting test to solve nonconvex maximization problem 'ncvx_min_ndiv.nl':" << std::endl;
        passed = CbcTest1("data/ncvx_min_ndiv.nl", -13.0);
        std::cout << "Finished test to solve nonconvex maximization problem 'ncvx_min_ndiv.nl'." << std::endl;
        break;
    case 8:
        std::cout << "Starting test for callbacks getting and setting primal solutions and dual bounds through a "
                     "callback with multi-tree strategy";
        passed = CbcExternalDualBoundCallbackTest("data/synthes1.osil", 5.0, ES_TreeStrategy::MultiTree);
        std::cout << "Finished test for callbacks getting and setting primal solutions and dual bounds through a "
                     "callback with multi-tree strategy."
                  << std::endl;
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