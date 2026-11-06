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

#include "../src/Model/Problem.h"
#include "../src/Model/ObjectiveFunction.h"
#include "IntegerCutTestCommon.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>

using namespace SHOT;

bool GurobiTest1(std::string filename, double correctObjectiveValue)
{
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Gurobi));

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
                      << "Given objective value (" << correctObjectiveValue << ") is within dual ("
                      << solver->getGlobalDualBound() << ") and primal (" << solver->getPrimalBound()
                      << ") for minimization problem." << std::endl;
        }
        else
        {
            std::cout << std::endl
                      << "Given objective value (" << correctObjectiveValue << ") is not within dual ("
                      << solver->getGlobalDualBound() << ") and primal (" << solver->getPrimalBound()
                      << ") for minimization problem." << std::endl;
            passed = false;
        }
    }
    else
    {
        if(correctObjectiveValue >= solver->getPrimalBound() - 1e-5
            && correctObjectiveValue <= solver->getGlobalDualBound() + 1e-5)
        {
            std::cout << std::endl
                      << "Given objective value (" << correctObjectiveValue << ") is within primal ("
                      << solver->getPrimalBound() << ") and dual (" << solver->getGlobalDualBound()
                      << ") for maximization problem." << std::endl;
        }
        else
        {
            std::cout << std::endl
                      << "Given objective value (" << correctObjectiveValue << ") is not within primal ("
                      << solver->getPrimalBound() << ") and dual (" << solver->getGlobalDualBound()
                      << ") for maximization problem." << std::endl;
            passed = false;
        }
    }

    return passed;
}

// A cut of x^2/y generated where y is close to zero has coefficients of the order 1/y^2, and the lazy cuts in the
// single-tree strategy are scaled as the other cuts so that the largest value is 1e9. The lazy cuts are read from the
// file they are written to in debug mode.
bool GurobiLazyCutScalingTest()
{
    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    std::string debugPath = "debug_gurobi_lazy_cut_scaling";
    std::remove((debugPath + "/lazycuts.txt").c_str());

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
    solver->updateSetting("Output.Debug.Enable", true);
    solver->updateSetting("Output.Debug.Path", debugPath);
    solver->updateSetting("Termination.TimeLimit", 60.0);
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Gurobi));
    solver->updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::SingleTree));
    // The cutting planes are generated in the solution points of the MIP problem, where y is at first at its lower
    // bound, and without the relaxation first solved without integer variables, the first cuts are lazy cuts
    solver->updateSetting("Dual.CutStrategy", static_cast<int>(ES_HyperplaneCutStrategy::ECP));
    solver->updateSetting("Dual.Relaxation.Use", false);
    solver->updateSetting("Model.BoundTightening.FeasibilityBased.Use", false);

    // min t + y + b s.t. x^2/y <= t, which has the optimum 2 in x = y = 1, b = 0
    auto problem = std::make_shared<Problem>(env);
    auto x = std::make_shared<Variable>("x", E_VariableType::Real, 1.0, 2.0);
    auto y = std::make_shared<Variable>("y", E_VariableType::Real, 1e-10, 10.0);
    auto t = std::make_shared<Variable>("t", E_VariableType::Real, 0.0, 1e3);
    auto b = std::make_shared<Variable>("b", E_VariableType::Binary, 0.0, 1.0);
    problem->add(Variables { x, y, t, b });

    auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    objective->add(std::make_shared<LinearTerm>(1.0, t));
    objective->add(std::make_shared<LinearTerm>(1.0, y));
    objective->add(std::make_shared<LinearTerm>(1.0, b));
    problem->add(objective);

    auto constraint = std::make_shared<NonlinearConstraint>("quotient", SHOT_DBL_MIN, 0.0);
    constraint->add(std::make_shared<LinearTerm>(-1.0, t));
    constraint->add(
        std::make_shared<ExpressionDivide>(std::make_shared<ExpressionSquare>(std::make_shared<ExpressionVariable>(x)),
            std::make_shared<ExpressionVariable>(y)));
    problem->add(constraint);
    problem->finalize();

    if(!solver->setProblem(problem))
        return false;

    solver->solveProblem();

    std::ifstream file(debugPath + "/lazycuts.txt");
    std::string line;
    int numberOfCuts = 0;
    double largestValue = 0.0;

    while(std::getline(file, line))
    {
        if(line.find("_HP:") == std::string::npos)
            continue;

        numberOfCuts++;

        // The line is name: coefficient variable ... sense rhs, and the values are the tokens that are numbers
        std::istringstream tokens(line.substr(line.find(':') + 1));
        std::string token;

        while(tokens >> token)
        {
            char* end;
            double value = std::strtod(token.c_str(), &end);

            if(*end == '\0')
                largestValue = std::max(largestValue, std::abs(value));
        }
    }

    std::cout << "  " << numberOfCuts << " lazy cuts with the largest value " << largestValue << std::endl;

    if(numberOfCuts == 0)
    {
        std::cout << "No lazy cuts were added." << std::endl;
        return false;
    }

    if(largestValue > 1e9 * (1 + 1e-6))
    {
        std::cout << "A lazy cut has a value larger than 1e9." << std::endl;
        return false;
    }

    if(std::abs(solver->getPrimalBound() - 2.0) > 1e-3)
    {
        std::cout << "The primal bound " << solver->getPrimalBound() << " is not the optimum 2." << std::endl;
        return false;
    }

    return true;
}

// Solves a problem with Gurobi and checks that a primal solution is found, and if requested, that the objective gap is
// closed
bool GurobiDualProblemTest(
    std::string filename, bool gapClosed, const std::function<void(Solver&)>& configure = nullptr)
{
    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
    solver->updateSetting("Termination.TimeLimit", 60.0);
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Gurobi));
    solver->updateSetting("Dual.MIP.NumberOfThreads", 1);

    if(configure)
        configure(*solver);

    if(!solver->setProblem(filename))
        return false;

    solver->solveProblem();

    std::cout << "  " << env->results->terminationReasonDescription << " Dual bound " << solver->getGlobalDualBound()
              << ", primal bound " << solver->getPrimalBound() << "." << std::endl;

    if(!env->results->hasPrimalSolution())
    {
        std::cout << "No primal solution was found." << std::endl;
        return false;
    }

    if(gapClosed && !env->results->isAbsoluteObjectiveGapToleranceMet()
        && !env->results->isRelativeObjectiveGapToleranceMet())
    {
        std::cout << "The objective gap is not closed." << std::endl;
        return false;
    }

    return true;
}

bool GurobiTerminationCallbackTest(std::string filename)
{
    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Gurobi));
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

            if(context.getSolutionStatistics().numberOfIterations > 3)
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

bool GurobiTerminationCallbackSingleTreeTest(std::string filename)
{
    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Gurobi));
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

bool GurobiExternalPrimalSolutionSingleTreeTest(std::string filename)
{
    // Phase 1: collect primal solution points from a MultiTree solve
    std::vector<VectorDouble> collectedSolutions;

    {
        std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

        solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
        solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Gurobi));
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

    // Phase 2: solve with Gurobi single-tree, injecting the collected solutions via callback
    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Gurobi));
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

    auto allSolutions [[maybe_unused]] = solver->getPrimalSolutions();
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

bool GurobiExternalDualBoundLazyConstraintTest(std::string filename, double externalDualBound)
{
    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Gurobi));
    solver->updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::SingleTree));

    std::cout << "Reading problem:  " << filename << '\n';

    if(!solver->setProblem(filename))
    {
        std::cout << "Error while reading problem\n";
        return (false);
    }

    // Register a callback that raises the external dual bound by 0.1 on each call,
    // starting from (externalDualBound - 0.1) up to externalDualBound, exercising the
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

bool GurobiExternalDualBoundCallbackTest(std::string filename, double dualBoundToTest, ES_TreeStrategy treeStrategy)
{
    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Gurobi));
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
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Gurobi));
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

// Bounding a free auxiliary objective after an unbounded LP can give a numerically infeasible point. It must not
// become the incumbent and close the objective gap at an infeasible objective value.
bool GurobiPrimalValidationAfterUnboundedLPTest(const std::string& filename)
{
    bool passed = true;

    for(auto treeStrategy : { ES_TreeStrategy::MultiTree, ES_TreeStrategy::SingleTree })
    {
        auto solver = std::make_unique<Solver>();
        auto env = solver->getEnvironment();
        solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
        solver->updateSetting("Termination.TimeLimit", 30.0);
        solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Gurobi));
        solver->updateSetting("Dual.MIP.NumberOfThreads", 1);
        solver->updateSetting("Dual.TreeStrategy", static_cast<int>(treeStrategy));
        solver->updateSetting("Dual.CutStrategy", static_cast<int>(ES_HyperplaneCutStrategy::ECP));
        solver->updateSetting(
            "Model.Reformulation.Quadratics.Strategy", static_cast<int>(ES_QuadraticProblemStrategy::Nonlinear));
        solver->updateSetting("Model.Reformulation.ObjectiveFunction.EpigraphStrategy", 2);
        solver->updateSetting("Model.BoundTightening.FeasibilityBased.Use", false);
        solver->updateSetting("Primal.Tolerance.TrustLinearConstraintValues", true);

        if(!solver->setProblem(filename) || !solver->solveProblem())
            return false;

        bool hasCorrectBounds = std::abs(solver->getPrimalBound() - 2.925) <= 1e-3
            && std::abs(solver->getGlobalDualBound() - 2.925) <= 1e-3;

        if(!solver->hasPrimalSolution() || !hasCorrectBounds
            || solver->getModelReturnStatus() != E_ModelReturnStatus::OptimalGlobal)
        {
            std::cout << "Tree strategy " << static_cast<int>(treeStrategy)
                      << ": expected optimal bounds of 2.925, got [" << solver->getGlobalDualBound() << ", "
                      << solver->getPrimalBound() << "]\n";
            passed = false;
        }

        for(auto& solution : solver->getPrimalSolutions())
        {
            auto deviation
                = env->problem->getMaxNumericConstraintValue(solution.point, env->problem->linearConstraints);
            if(deviation.error > 1e-6)
            {
                std::cout << "Accepted a solution violating " << deviation.constraint->name << " by " << deviation.error
                          << '\n';
                passed = false;
            }
        }
    }

    return passed;
}

bool GurobiQuadraticObjectiveWithoutRecommendedSettingsTest()
{
    bool passed = true;

    for(auto strategy :
        { ES_QuadraticProblemStrategy::QuadraticObjective, ES_QuadraticProblemStrategy::ConvexQuadraticallyConstrained,
            ES_QuadraticProblemStrategy::NonconvexQuadraticallyConstrained })
    {
        auto solver = std::make_unique<Solver>();
        solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
        solver->updateSetting("Strategy.UseRecommendedSettings", false);
        solver->updateSetting("Dual.MIP.Solver", static_cast<int>(ES_MIPSolver::Gurobi));
        solver->updateSetting("Dual.MIP.NumberOfThreads", 1);
        solver->updateSetting("Termination.TimeLimit", 10.0);
        solver->updateSetting("Model.Reformulation.Quadratics.Strategy", static_cast<int>(strategy));

        if(!solver->setProblem("data/instances/minlp_tests_jl/nlp-mi_003_014.jl.nl") || !solver->solveProblem())
            return false;

        // The fixed-integer fallback can find the optimum even after a MIP error. Require an error-free solve and
        // valid bounds, without requiring a global proof for this nonconvex cutting-plane problem.
        if(solver->getTerminationReason() == E_TerminationReason::Error || !solver->hasPrimalSolution()
            || std::abs(solver->getPrimalBound() - 11.0) > 1e-5 || !std::isfinite(solver->getGlobalDualBound())
            || solver->getGlobalDualBound() < 11.0 - 1e-5)
        {
            std::cout << "Quadratic strategy " << static_cast<int>(strategy)
                      << ": expected an error-free solve with primal value 11 and a valid dual bound, got ["
                      << solver->getPrimalBound() << ", " << solver->getGlobalDualBound() << "], termination reason "
                      << static_cast<int>(solver->getTerminationReason()) << '\n';
            passed = false;
        }

        if(solver->hasPrimalSolution())
        {
            auto problem = solver->getOriginalProblem();
            auto deviation
                = problem->getMaxNumericConstraintValue(solver->getPrimalSolution().point, problem->numericConstraints);
            if(!std::isfinite(deviation.error) || deviation.error > 1e-5)
            {
                std::cout << "Returned point violates the original constraints by " << deviation.error << '\n';
                passed = false;
            }
        }
    }

    return passed;
}

int GurobiTest(int argc, char* argv[])
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
        std::cout << "Starting test to solve a MINLP problem with Gurobi." << std::endl;
        passed = GurobiTest1("data/tls2.osil", 5.3);
        std::cout << "Finished test to solve a MINLP problem with Gurobi." << std::endl;
        break;
    case 2:
        std::cout << "Starting test to check termination callback in Gurobi:" << std::endl;
        passed = GurobiTerminationCallbackTest("data/tls2.osil");
        std::cout << "Finished test checking termination callback in Gurobi." << std::endl;
        break;
    case 3:
        std::cout << "Starting test to solve problem with semicont. variables with Gurobi.:" << std::endl;
        passed = GurobiTest1("data/meanvarxsc.osil", 14.36923211);
        std::cout << "Finished test to solve problem with semicont. variables with Gurobi." << std::endl;
        break;
    case 4:
        std::cout << "Starting test to solve nonconvex maximization problem 'ncvx_max_div.nl':" << std::endl;
        passed = GurobiTest1("data/ncvx_max_div.nl", 13.0);
        std::cout << "Finished test to solve nonconvex maximization problem 'ncvx_max_div.nl'." << std::endl;
        break;
    case 5:
        std::cout << "Starting test to solve nonconvex maximization problem 'ncvx_min_div.nl':" << std::endl;
        passed = GurobiTest1("data/ncvx_min_div.nl", -13.0);
        std::cout << "Finished test to solve nonconvex maximization problem 'ncvx_min_div.nl'." << std::endl;
        break;
    case 6:
        std::cout << "Starting test to solve nonconvex maximization problem 'ncvx_max_ndiv.nl':" << std::endl;
        passed = GurobiTest1("data/ncvx_max_ndiv.nl", 13.0);
        std::cout << "Finished test to solve nonconvex maximization problem 'ncvx_max_ndiv.nl'." << std::endl;
        break;
    case 7:
        std::cout << "Starting test to solve nonconvex maximization problem 'ncvx_min_ndiv.nl':" << std::endl;
        passed = GurobiTest1("data/ncvx_min_ndiv.nl", -13.0);
        std::cout << "Finished test to solve nonconvex maximization problem 'ncvx_min_ndiv.nl'." << std::endl;
        break;
    case 8:
        std::cout << "Starting test for callbacks getting and setting primal solutions and dual bounds through a "
                     "callback with multi-tree strategy"
                  << std::endl;
        passed = GurobiExternalDualBoundCallbackTest("data/synthes1.osil", 5.0, ES_TreeStrategy::MultiTree);
        std::cout << "Finished test for callbacks getting and setting primal solutions and dual bounds through a "
                     "callback with multi-tree strategy.";
        break;
    case 9:
        std::cout << "Starting test for callbacks getting and setting primal solutions and dual bounds through a "
                     "callback with single-tree strategy"
                  << std::endl;
        passed = GurobiExternalDualBoundCallbackTest("data/synthes1.osil", 5.0, ES_TreeStrategy::SingleTree);
        std::cout << "Finished test for callbacks getting and setting primal solutions and dual bounds through a "
                     "callback with single-tree strategy.";
        break;
    case 10:
        std::cout << "Starting test for external dual bound lazy constraint in single-tree strategy" << std::endl;
        passed = GurobiExternalDualBoundLazyConstraintTest("data/fo7_2.osil", 17.4);
        std::cout << "Finished test for external dual bound lazy constraint in single-tree strategy.";
        break;
    case 11:
        std::cout << "Starting test for external primal solution injection in single-tree strategy" << std::endl;
        passed = GurobiExternalPrimalSolutionSingleTreeTest("data/synthes1.osil");
        std::cout << "Finished test for external primal solution injection in single-tree strategy.";
        break;
    case 12:
        std::cout << "Starting test for termination callback in single-tree strategy" << std::endl;
        passed = GurobiTerminationCallbackSingleTreeTest("data/synthes1.osil");
        std::cout << "Finished test for termination callback in single-tree strategy.";
        break;
    case 13:
        std::cout << "Starting test to solve gear.osil with Gurobi." << std::endl;
        passed = solveWithIntegerCuts(ES_MIPSolver::Gurobi, "data/gear.osil", 0.0);
        std::cout << "Finished test to solve gear.osil with Gurobi." << std::endl;
        break;
    case 14:
        std::cout << "Starting test to solve windfac.osil with Gurobi." << std::endl;
        passed = solveWithIntegerCuts(ES_MIPSolver::Gurobi, "data/windfac.osil", 0.254487);
        std::cout << "Finished test to solve windfac.osil with Gurobi." << std::endl;
        break;
    case 15:
        std::cout << "Starting test to solve ex1252a.osil with Gurobi." << std::endl;
        passed = solveWithIntegerCuts(ES_MIPSolver::Gurobi, "data/ex1252a.osil", 128893.741);
        std::cout << "Finished test to solve ex1252a.osil with Gurobi." << std::endl;
        break;
    case 16:
        std::cout << "Starting test for integer cuts of nonbinary variables in single-tree strategy" << std::endl;
        passed = integerCutsBetweenSingleTreeSolves(ES_MIPSolver::Gurobi);
        std::cout << "Finished test for integer cuts of nonbinary variables in single-tree strategy." << std::endl;
        break;
    case 17:
        std::cout << "Starting test for scaling the lazy cuts in single-tree strategy" << std::endl;
        passed = GurobiLazyCutScalingTest();
        std::cout << "Finished test for scaling the lazy cuts in single-tree strategy." << std::endl;
        break;
    case 18:
        // Gurobi solves the convex problem with quadratic constraints directly, and its objective value is the bound
        std::cout << "Starting test for the dual bound of a continuous convex problem with quadratic constraints"
                  << std::endl;
        passed = GurobiDualProblemTest("data/instances/minlp_tests_jl/nlp-cvx_202_010.jl.nl", true);
        std::cout << "Finished test for the dual bound of a continuous convex problem with quadratic constraints."
                  << std::endl;
        break;
    case 19:
        // The barrier method has numerical trouble on the degenerate convex problem with quadratic constraints
        std::cout << "Starting test for solving the dual problem again after numerical trouble" << std::endl;
        passed = GurobiDualProblemTest("data/instances/minlp_tests_jl/nlp-cvx_202_011.jl.nl", true);
        std::cout << "Finished test for solving the dual problem again after numerical trouble." << std::endl;
        break;
    case 20:
        // The relaxation gets numerical trouble after a repair, also when Gurobi uses its most careful numerics, and
        // the primal solutions are only found in the problem with the discrete variables
        std::cout << "Starting test for ending the relaxation after numerical trouble" << std::endl;
        passed = GurobiDualProblemTest("data/instances/MINLP-nonconvex/nvs05.osil", false,
            [](Solver& solver)
            {
                solver.updateSetting("Strategy.UseRecommendedSettings", false);
                solver.updateSetting("Subsolver.Gurobi.NumericFocus", 3);
            });
        std::cout << "Finished test for ending the relaxation after numerical trouble." << std::endl;
        break;
    case 21:
        std::cout << "Starting test for primal feasibility after an unbounded LP" << std::endl;
        passed = GurobiPrimalValidationAfterUnboundedLPTest("data/instances/MIQCQP-convex/alan.osil");
        std::cout << "Finished test for primal feasibility after an unbounded LP." << std::endl;
        break;
    case 22:
        std::cout << "Starting test for quadratic objectives without recommended settings" << std::endl;
        passed = GurobiQuadraticObjectiveWithoutRecommendedSettingsTest();
        std::cout << "Finished test for quadratic objectives without recommended settings." << std::endl;
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
