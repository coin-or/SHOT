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
#include "IntegerCutTestCommon.h"

#include <cmath>
#include <functional>
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

// Solves a problem in the single-tree strategy with the generic callback of Cplex, and checks that the bounds enclose
// the optimal objective value and that the gap is closed. With several threads, a bound of one thread taken as the
// global bound would not enclose it, and Cplex discarding the optimal solutions would leave a gap.
bool CplexGenericCallbackTest(std::string filename, double correctObjectiveValue, int numberOfThreads,
    int maxRelaxedLazyConstraints = 0, bool addRelaxedLazyConstraintsAsLocal = false)
{
    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();

    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Cplex));
    solver->updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::SingleTree));
    solver->updateSetting("Dual.MIP.NumberOfThreads", numberOfThreads);
    solver->updateSetting("Dual.Relaxation.MaxLazyConstraints", maxRelaxedLazyConstraints);
    solver->updateSetting("Subsolver.Cplex.AddRelaxedLazyConstraintsAsLocal", addRelaxedLazyConstraintsAsLocal);

    if(!solver->setProblem(filename))
    {
        std::cout << "Error while reading problem\n";
        return (false);
    }

    if(!solver->solveProblem())
    {
        std::cout << "Error while solving problem\n";
        return (false);
    }

    if(solver->getPrimalSolutions().empty())
    {
        std::cout << "No primal solution found\n";
        return (false);
    }

    double primalBound = solver->getPrimalBound();
    double dualBound = solver->getGlobalDualBound();

    std::cout << "Dual bound: " << dualBound << ", primal bound: " << primalBound
              << ", optimal objective value: " << correctObjectiveValue << "\n";

    bool isMinimize = solver->getOriginalProblem()->objectiveFunction->properties.isMinimize;
    double lowerBound = isMinimize ? dualBound : primalBound;
    double upperBound = isMinimize ? primalBound : dualBound;

    if(correctObjectiveValue < lowerBound - 1e-5 || correctObjectiveValue > upperBound + 1e-5)
    {
        std::cout << "The optimal objective value is not within the bounds\n";
        return (false);
    }

    if(env->results->terminationReason != E_TerminationReason::AbsoluteGap
        && env->results->terminationReason != E_TerminationReason::RelativeGap)
    {
        std::cout << "The solver did not terminate since the objective gap was closed\n";
        return (false);
    }

    return (true);
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

bool CplexRelaxationConversionTest()
{
    auto solver = std::make_unique<Solver>();
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Cplex));
    solver->updateSetting("Dual.MIP.NumberOfThreads", 1);
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
    auto env = solver->getEnvironment();

    auto problem = std::make_shared<Problem>(env);
    auto binary = std::make_shared<Variable>("binary", E_VariableType::Binary, 0.0, 1.0);
    auto integer = std::make_shared<Variable>("integer", E_VariableType::Integer, 0.0, 2.0);
    auto continuous = std::make_shared<Variable>("continuous", E_VariableType::Real, 0.0, 1.0);
    problem->add({ binary, integer, continuous });

    auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    for(auto& variable : { binary, integer, continuous })
        objective->add(std::make_shared<LinearTerm>(1.0, variable));
    problem->add(objective);

    for(auto& variable : { binary, integer, continuous })
    {
        auto constraint = std::make_shared<LinearConstraint>(variable->name + "_lower", 0.5, SHOT_DBL_MAX);
        constraint->add(std::make_shared<LinearTerm>(1.0, variable));
        problem->add(constraint);
    }

    problem->finalize();
    if(!solver->setProblem(problem))
        return false;

    auto backend = std::make_shared<MIPSolverCplex>(env);
    if(!backend->initializeProblem())
        return false;

    TaskCreateMIPProblem(env, backend, problem).run();
    backend->setTimeLimit(10.0);
    backend->setSolutionLimit(2100000000);

    for(int repeat = 0; repeat < 2; ++repeat)
    {
        backend->activateDiscreteVariables(false);
        if(backend->solveProblem() != E_ProblemSolutionStatus::Optimal
            || std::abs(backend->getObjectiveValue() - 1.5) > 1e-6)
            return false;

        backend->activateDiscreteVariables(true);
        if(backend->solveProblem() != E_ProblemSolutionStatus::Optimal
            || std::abs(backend->getObjectiveValue() - 2.5) > 1e-6)
            return false;
    }

    return true;
}

// Solves the problem with Cplex and the settings given by the function, and checks that the solution does not end with
// an error, that there is a primal solution and that the primal bound is the optimum
bool CplexSettingsTest(std::string filename, double correctObjectiveValue, std::function<void(Solver&)> setSettings,
    bool requireIntegerCuts = false)
{
    auto solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Cplex));
    solver->updateSetting("Dual.MIP.NumberOfThreads", 1);
    solver->updateSetting("Termination.TimeLimit", 60.0);
    setSettings(*solver);

    if(!solver->setProblem(filename))
        return false;

    solver->solveProblem();

    if(env->results->terminationReason == E_TerminationReason::Error
        || env->results->terminationReason == E_TerminationReason::NumericIssues)
    {
        std::cout << "The solution ended with an error: " << env->results->terminationReasonDescription << std::endl;
        return false;
    }

    if(requireIntegerCuts && env->solutionStatistics.numberOfIntegerCuts == 0)
    {
        std::cout << "No integer cuts were added." << std::endl;
        return false;
    }

    if(!env->results->hasPrimalSolution())
    {
        std::cout << "No primal solution was found." << std::endl;
        return false;
    }

    // NaN when the problem is nonconvex and only a primal solution is required
    if(!std::isnan(correctObjectiveValue)
        && std::abs(solver->getPrimalBound() - correctObjectiveValue)
            > 1e-4 * std::max(1.0, std::abs(correctObjectiveValue)))
    {
        std::cout << "The primal bound " << solver->getPrimalBound() << " is not the optimum " << correctObjectiveValue
                  << "." << std::endl;
        return false;
    }

    return true;
}

class CplexRecoveryProbe : public MIPSolverCplex
{
public:
    using MIPSolverCplex::MIPSolverCplex;

    E_ProblemSolutionStatus recoverWithExpiredBudget()
    {
        return resolveWithNumericalEmphasis(E_ProblemSolutionStatus::Infeasible,
            cplexInstance.getCplexTime() - cplexInstance.getParam(IloCplex::Param::TimeLimit) - 1.0);
    }
};

bool CplexInfeasibilityRecoverySettingsTest()
{
    for(int emphasis : { 0, 1 })
    {
        auto solver = std::make_unique<Solver>();
        solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Cplex));
        solver->updateSetting("Dual.MIP.NumberOfThreads", 1);
        solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
        solver->updateSetting("Model.BoundTightening.FeasibilityBased.Use", false);
        auto env = solver->getEnvironment();
        auto problem = std::make_shared<Problem>(env);
        auto x = std::make_shared<Variable>("x", E_VariableType::Integer, 0.0, 1.0);
        problem->add(x);
        auto constraint = std::make_shared<LinearConstraint>("infeasible", 2.0, SHOT_DBL_MAX);
        constraint->add(std::make_shared<LinearTerm>(1.0, x));
        problem->add(constraint);
        auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        objective->add(std::make_shared<LinearTerm>(1.0, x));
        problem->add(objective);
        problem->finalize();
        if(!solver->setProblem(problem))
            return false;
        auto backend = std::make_shared<CplexRecoveryProbe>(env);
        if(!backend->initializeProblem())
            return false;
        TaskCreateMIPProblem(env, backend, problem).run();
        backend->setTimeLimit(10.0);
        backend->cplexInstance.setParam(IloCplex::Param::Emphasis::Numerical, emphasis);
        backend->cplexInstance.setParam(IloCplex::Param::Advance, 2);

        auto settingsRestored = [&]()
        {
            return backend->cplexInstance.getParam(IloCplex::Param::Emphasis::Numerical) == emphasis
                && backend->cplexInstance.getParam(IloCplex::Param::Preprocessing::Presolve)
                && backend->cplexInstance.getParam(IloCplex::Param::Advance) == 2
                && backend->cplexInstance.getParam(IloCplex::Param::TimeLimit) == 10.0;
        };

        if(backend->solveProblem() != E_ProblemSolutionStatus::Infeasible || !settingsRestored()
            || backend->getDualObjectiveValue() != SHOT_DBL_MIN)
            return false;

        // Once the budget is exhausted, do not start another solve or publish a bound from the failed one.
        if(backend->recoverWithExpiredBudget() != E_ProblemSolutionStatus::TimeLimit || !settingsRestored()
            || backend->getDualObjectiveValue() != SHOT_DBL_MIN)
            return false;
    }
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
        passed = solveWithIntegerCuts(ES_MIPSolver::Cplex, "data/gear.osil", 0.0);
        std::cout << "Finished test to solve gear.osil with Cplex." << std::endl;
        break;
    case 14:
        std::cout << "Starting test to solve windfac.osil with Cplex." << std::endl;
        passed = solveWithIntegerCuts(ES_MIPSolver::Cplex, "data/windfac.osil", 0.254487);
        std::cout << "Finished test to solve windfac.osil with Cplex." << std::endl;
        break;
    case 15:
        std::cout << "Starting test to solve ex1252a.osil with Cplex." << std::endl;
        passed = solveWithIntegerCuts(ES_MIPSolver::Cplex, "data/ex1252a.osil", 128893.741);
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
    case 18:
        std::cout << "Testing Cplex LP/MIP conversion for mixed variable types.\n";
        passed = CplexRelaxationConversionTest();
        break;
    case 19:
        std::cout << "Testing the generic Cplex callback on a convex MINLP problem.\n";
        passed = CplexGenericCallbackTest("data/synthes1.osil", 6.00975891, 1);
        break;
    case 20:
        // Candidates near the boundary must not be rejected with hyperplanes that only cut them off by rounding errors
        std::cout << "Testing the generic Cplex callback on a minimization problem with a nonlinear objective.\n";
        passed = CplexGenericCallbackTest("data/constant_synthes2_min.osil", 73.0353126, 1);
        break;
    case 21:
        std::cout << "Testing the generic Cplex callback on a maximization problem with a nonlinear objective.\n";
        passed = CplexGenericCallbackTest("data/constant_synthes2_max.osil", -73.0353126, 1);
        break;
    case 22:
        std::cout << "Testing the generic Cplex callback with several threads.\n";
        passed = CplexGenericCallbackTest("data/fo7_2.osil", 17.7493, 4);
        break;
    case 23:
        std::cout << "Testing the generic Cplex callback with local hyperplanes at relaxed points.\n";
        passed = CplexGenericCallbackTest("data/synthes1.osil", 6.00975891, 1, 10, true);
        break;
    case 24:
        // The cutoff is a constraint in the LP relaxation, and the objective function has a constant
        std::cout << "Testing the Cplex cutoff constraint with a constant in the objective function.\n";
        passed = CplexSettingsTest("data/instances/minlp_tests_jl/nlp-mi_003_016.jl.nl", 6.141592653589793,
            [](Solver& solver) { solver.updateSetting("Strategy.UseRecommendedSettings", false); });
        break;
    case 25:
        // Cuts are added after the quadratic constraints and later removed when the dual problem is repaired
        std::cout << "Testing the removal of Cplex cuts in a problem with quadratic constraints.\n";
        passed = CplexSettingsTest("data/instances/minlp_tests_jl/nlp-cvx_108_012.jl.nl", 0.5927195136970647,
            [](Solver& solver)
            {
                solver.updateSetting("Model.Reformulation.ObjectiveFunction.EpigraphStrategy",
                    static_cast<int>(ES_ObjectiveEpigraphStrategy::EpigraphConstraint));
            });
        break;
    case 26:
        // Integer cuts of nonbinary variables add variables to the MIP problem in the single-tree strategy
        std::cout << "Testing the Cplex single-tree callback after integer cuts of nonbinary variables.\n";
        passed = CplexSettingsTest(
            "data/instances/MINLP-convex/cvxnonsep_psig20r.osil", 95.89738736,
            [](Solver& solver)
            {
                solver.updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::SingleTree));
                solver.updateSetting("Dual.CutStrategy", static_cast<int>(ES_HyperplaneCutStrategy::ECP));
                solver.updateSetting("Dual.HyperplaneCuts.UseIntegerCuts", true);
                solver.updateSetting("Subsolver.Cplex.NumericalEmphasis", 1);
                solver.updateSetting("Subsolver.Cplex.MIPEmphasis", 4);
            },
            true);
        break;
    case 27:
        // Cplex fails with numerical trouble (status FailFeas) in a dual problem
        std::cout << "Testing that numerical trouble in Cplex does not end the solution.\n";
        passed = CplexSettingsTest("data/instances/MINLP-nonconvex/nvs05.osil", NAN, [](Solver&) { });
        break;
    case 28:
        // Cplex's presolve reports the first dual problem, with convex quadratic constraints, as infeasible
        std::cout << "Testing a feasible problem that Cplex's presolve reports as infeasible.\n";
        for(auto treeStrategy : { ES_TreeStrategy::MultiTree, ES_TreeStrategy::SingleTree })
            passed = CplexSettingsTest("data/instances/MINLP-convex-small/nvs11.osil", -431.0,
                         [treeStrategy](Solver& solver)
                         {
                             solver.updateSetting("Dual.TreeStrategy", static_cast<int>(treeStrategy));
                             solver.updateSetting("Model.Reformulation.ObjectiveFunction.EpigraphStrategy",
                                 static_cast<int>(ES_ObjectiveEpigraphStrategy::EpigraphConstraint));
                         })
                && passed;
        break;
    case 29:
        std::cout << "Testing Cplex recovery settings and remaining time.\n";
        passed = CplexInfeasibilityRecoverySettingsTest();
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
