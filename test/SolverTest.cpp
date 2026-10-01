/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "../src/Solver.h"
#include "../src/DualSolver.h"
#include "../src/Environment.h"
#include "../src/MIPSolver/IMIPSolver.h"
#include "../src/Results.h"
#include "../src/Settings.h"
#include "../src/Structs.h"
#include "../src/TaskHandler.h"
#include "../src/Utilities.h"
#include "../src/Callback.h"
#include "../src/Model/Simplifications.h"

#include "../src/Model/Variables.h"
#include "../src/Model/Terms.h"
#include "../src/Model/Constraints.h"
#include "../src/Model/NonlinearExpressions.h"
#include "../src/Model/Problem.h"
#include "../src/Model/ObjectiveFunction.h"

#include "../src/ModelingSystem/ModelingSystemOSiL.h"
#include "../src/ModelingSystem/ModelingSystemAMPL.h"

#include "../src/RootsearchMethod/RootsearchMethodBoost.h"

#include "../src/Tasks/TaskReformulateProblem.h"

#include <set>
#include <stdexcept>

using namespace SHOT;

bool ReadProblem(std::string filename)
{
    bool passed = true;

    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();

    if(solver->setProblem(filename))
    {
        passed = true;
    }
    else
    {
        passed = false;
    }

    return passed;
}

bool SolveProblem(std::string filename)
{
    bool passed = true;

    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();

    if(solver->setProblem(filename))
    {
        passed = true;
    }
    else
    {
        passed = false;
    }

    if(passed == false)
        return passed;

    solver->solveProblem();
    std::string osrl = solver->getResultsOSrL();
    std::string trace = solver->getResultsTrace();
    if(!SHOT::Utilities::writeStringToFile("result.osrl", osrl))
    {
        std::cout << "Could not write results to OSrL file." << std::endl;
        passed = false;
    }

    if(!SHOT::Utilities::writeStringToFile("trace.trc", trace))
    {
        std::cout << "Could not write results to trace file." << std::endl;
        passed = false;
    }

    if(solver->getPrimalSolutions().size() > 0)
    {
        std::cout << std::endl << "Objective value: " << solver->getPrimalSolution().objValue << std::endl;
        passed = true;
    }
    else
    {
        passed = false;
    }

    return passed;
}

bool TestRootsearch(const std::string& problemFile)
{
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Error));

    std::cout << "Reading problem:  " << problemFile << '\n';

    if(!solver->setProblem(problemFile))
    {
        std::cout << "Error while reading problem";
        passed = false;
    }

    std::cout << env->problem << "\n\n";

    VectorDouble interiorPoint;
    interiorPoint.push_back(7.44902);
    interiorPoint.push_back(8.53506);

    VectorDouble exteriorPoint;
    exteriorPoint.push_back(20.0);
    exteriorPoint.push_back(20.0);

    std::cout << "Interior point:\n";
    Utilities::displayVector(interiorPoint);

    std::cout << "Exterior point:\n";
    Utilities::displayVector(exteriorPoint);

    auto rootsearch = std::make_unique<RootsearchMethodBoost>(env);

    auto root = rootsearch->findZero(
        interiorPoint, exteriorPoint, 100, 10e-13, 10e-3, env->problem->nonlinearConstraints, false);

    std::cout << "Root found:\n";
    Utilities::displayVector(root.first, root.second);

    exteriorPoint.clear();
    exteriorPoint.push_back(8.47199);
    exteriorPoint.push_back(20.0);

    std::cout << "Interior point:\n";
    Utilities::displayVector(interiorPoint);

    std::cout << "Exterior point:\n";
    Utilities::displayVector(exteriorPoint);

    root = rootsearch->findZero(
        interiorPoint, exteriorPoint, 100, 10e-13, 10e-3, env->problem->nonlinearConstraints, false);

    std::cout << "Root found:\n";
    Utilities::displayVector(root.first, root.second);

    exteriorPoint.clear();
    exteriorPoint.push_back(1.0);
    exteriorPoint.push_back(10.0);

    std::cout << "Interior point:\n";
    Utilities::displayVector(interiorPoint);

    std::cout << "Exterior point:\n";
    Utilities::displayVector(exteriorPoint);

    root = rootsearch->findZero(
        interiorPoint, exteriorPoint, 100, 10e-13, 10e-3, env->problem->nonlinearConstraints, false);

    std::cout << "Root found:\n";
    Utilities::displayVector(root.first, root.second);

    return passed;
}

bool TestGradient(const std::string& problemFile)
{
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    // A time limit so that a test that does not converge cannot stop the test suite
    solver->updateSetting("Termination.TimeLimit", 60.0);

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Error));

    std::cout << "Reading problem:  " << problemFile << '\n';

    if(!solver->setProblem(problemFile))
    {
        std::cout << "Error while reading problem";
        passed = false;
    }

    VectorDouble point;

    for(auto& V : env->problem->allVariables)
    {
        point.push_back((V->upperBound - V->lowerBound) / 2.0);
    }

    std::cout << "Point to evaluate gradients in:\n";
    Utilities::displayVector(point);

    for(auto& C : env->problem->numericConstraints)
    {
        std::cout << "\nCalculating gradient for constraint:\t" << C << ":\n";

        auto gradient = C->calculateGradient(point, true);

        for(auto const& G : gradient)
        {
            std::cout << G.first->name << ":  " << G.second << '\n';
        }

        std::cout << '\n';
    }

    return passed;
}

// Forward declarations — defined later in this file
static std::pair<std::unique_ptr<SHOT::Solver>, std::shared_ptr<SHOT::Environment>> MakeEx1223bSolver(
    bool forceNonlinear = false);

bool TestCallbackESHInteriorPoint();

bool CreateAndSolveProblem()
{
    bool passed = true;

    auto solverEnv = MakeEx1223bSolver();
    auto& solver = solverEnv.first;
    auto& env = solverEnv.second;
    solver->updateSetting("Output.Debug.Enable", true);
    auto problem = env->problem;

    // Writing the problem to console
    std::cout << '\n';
    std::cout << "Problem created:\n\n";
    std::cout << env->problem << '\n';

    // Writing the reformulated problem to console
    std::cout << '\n';
    std::cout << "Reformulated problem created:\n\n";
    std::cout << env->reformulatedProblem << '\n';

    // Solving the problem
    solver->solveProblem();

    if(solver->getPrimalSolutions().size() > 0)
    {
        std::cout << std::endl << "Objective value: " << solver->getPrimalSolution().objValue << std::endl;
        passed = true;
    }
    else
    {
        passed = false;
    }

    if(!passed)
        std::cout << "Cound not solve problem!\n";

    std::cout << "Now trying to reuse the created problem while recreating the solver instance with a callback "
                 "activating everytime a new primal solution is found.\n\n";

    auto reformulatedProblem = env->reformulatedProblem; // Since this is a shared pointer it will not be deleted

    solver = nullptr;
    env = nullptr;

    // Will now do a test to see if we can add a primal solution callback to the solver
    // and reuse the problem instance without problems.

    // Need to reinitialize the SHOT solver class and update the environment
    solver = std::make_unique<SHOT::Solver>();
    env = solver->getEnvironment();
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));

    // Resetting our problem objects
    solver->setProblem(problem, reformulatedProblem);

    // Registers a callback that is activated every time a new primal solution is found
    solver->registerCallback<NewPrimalSolutionContext>(
        [&env, &passed](NewPrimalSolutionContext& solution)
        {
            std::cout << "We have a new primal solution: " << solution.getObjectiveValue()
                      << " found at iteration: " << solution.getIterationNumber() << ". In total we now have "
                      << env->solutionStatistics.numberOfFoundPrimalSolutions << " solutions.\n";

            std::cout << "Primal solution point:\n";
            Utilities::displayVector(solution.getPoint());

            if(solution.getObjectiveValue() == env->results->getPrimalBound())
            {
                std::cout << "Ok, new primal solution has been saved successfully. " << solution.getObjectiveValue()
                          << ".\n";
                passed = true;
            }
            else
            {
                std::cout << "Error: new primal solution not saved successfully!\n";
                passed = false;
            }
        });

    // Solving the problem
    solver->solveProblem();

    if(solver->getPrimalSolutions().size() > 0)
    {
        std::cout << std::endl << "Objective value: " << solver->getPrimalSolution().objValue << std::endl;
        passed = true;
    }
    else
    {
        passed = false;
    }

    if(!passed)
        std::cout << "Cound not resolve problem!\n";
    else
        std::cout << "Could reuse the problem instance without problem!\n";

    std::cout << "Now trying to have two callbacks, one that prints out the primal solution and one that is "
                 "terminating after one primal solution has been found\n\n";

    solver = nullptr;
    env = nullptr;

    // Will now do a test to see if we can add a primal solution callback to the solver
    // and reuse the problem instance without problems.

    // Need to reinitialize the SHOT solver class and update the environment
    solver = std::make_unique<SHOT::Solver>();
    env = solver->getEnvironment();
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));

    // Resetting our problem objects
    solver->setProblem(problem, reformulatedProblem);

    // Registers a callback that is activated every time a new primal solution is found
    solver->registerCallback<NewPrimalSolutionContext>(
        [&env, &passed](NewPrimalSolutionContext& solution)
        {
            std::cout << "We have a new primal solution: " << solution.getObjectiveValue()
                      << " found at iteration: " << solution.getIterationNumber() << ". In total we now have "
                      << env->solutionStatistics.numberOfFoundPrimalSolutions << " solutions.\n";

            std::cout << "Primal solution point:\n";
            Utilities::displayVector(solution.getPoint());

            if(solution.getObjectiveValue() == env->results->getPrimalBound())
            {
                std::cout << "Ok, new primal solution has been saved successfully. " << solution.getObjectiveValue()
                          << ".\n";
                passed = true;
            }
            else
            {
                std::cout << "Error: new primal solution not saved successfully!\n";
                passed = false;
            }
        });

    // Registers a callback that terminates if we have found at least one primal solution
    solver->registerCallback<TerminationCheckContext>(
        [](TerminationCheckContext& context)
        {
            std::cout << "Termination callback - iteration: " << context.getIterationNumber() << "\n";

            // If we have found one primal solution, we terminate the solver
            if(context.getIterationNumber() > 0 && context.getSolutionStatistics().numberOfFoundPrimalSolutions > 0)
            {
                std::cout << "Termination callback activated. We have found at least one solution.\n";
                context.terminate();
            }
            else
            {
                std::cout << "Termination callback activated. We have not found a primal solution yet.\n";
            }
        });

    // Solving the problem
    solver->solveProblem();

    if(solver->getPrimalSolutions().size() > 0)
    {
        std::cout << std::endl << "Objective value: " << solver->getPrimalSolution().objValue << std::endl;
        passed = true;
    }
    else
    {
        passed = false;
    }

    if(!passed)
        std::cout << "Cound not resolve problem!\n";
    else
        std::cout << "Could reuse the problem instance without problem!\n";

    return passed;
}

bool TestCallbackUserTermination()
{
    bool passed = true;

    // Initializing the SHOT solver class
    auto [solver, env] = MakeEx1223bSolver();
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Critical));
    solver->updateSetting("Output.Debug.Enable", true);

    // Writing the problem to console
    std::cout << '\n';
    std::cout << "Problem created:\n\n";
    std::cout << env->problem << '\n';

    // Writing the reformulated problem to console
    std::cout << '\n';
    std::cout << "Reformulated problem created:\n\n";
    std::cout << env->reformulatedProblem << '\n';

    // Track termination
    int iterationCount = 0;
    bool terminationRequested = false;
    bool solverWasTerminated = false;

    // Register user termination check
    solver->registerCallback<TerminationCheckContext>(
        [&iterationCount, &terminationRequested](TerminationCheckContext& context)
        {
            iterationCount++;

            std::cout << "User termination check called (call #" << iterationCount << ", solver iteration "
                      << context.getIterationNumber() << ")" << std::endl;

            // Terminate after 3 checks
            if(iterationCount >= 3)
            {
                std::cout << "User termination check requesting termination" << std::endl;
                terminationRequested = true;
                context.terminate();
            }
        });

    // Set high iteration limit so termination comes from our callback
    env->settings->updateSetting("Termination.IterationLimit", 100);

    // Solve the problem
    solver->solveProblem();

    // Verify that our termination callback was effective
    if(terminationRequested)
    {
        std::cout << "User termination was successfully requested" << std::endl;
        passed = true;
    }
    else
    {
        std::cout << "User termination was not requested (unexpected)" << std::endl;
        passed = false;
    }

    // A termination requested through the callback must be reported as a user abort
    if(passed && env->results->terminationReason != E_TerminationReason::UserAbort)
    {
        std::cout << "Termination reason is not UserAbort as expected" << std::endl;
        passed = false;
    }

    if(!passed)
        std::cout << "Could not terminate problem with callback!\n";
    else
        std::cout << "Could terminate problem with callback!\n";

    return passed;
}

bool TestCallbackExternalHyperplane()
{
    bool passed = true;

    auto solver = std::make_unique<SHOT::Solver>();

    // Contains the environment variable unique to the created solver instance
    auto env = solver->getEnvironment();
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
    solver->updateSetting("Model.Convexity.AssumeConvex", true);
    solver->updateSetting("Output.Debug.Enable", true);
    solver->updateSetting(
        "Model.Reformulation.Constraint.PartitionQuadraticTerms", static_cast<int>(ES_PartitionNonlinearSums::Never));
    solver->updateSetting("Dual.Relaxation.Use", false);
    solver->updateSetting("Dual.CutStrategy", static_cast<int>(ES_HyperplaneCutStrategy::OnlyExternal));
    solver->updateSetting("Dual.TreeStrategy", static_cast<int>(ES_TreeStrategy::MultiTree));

    // Initializing a SHOT problem class
    auto problem = std::make_shared<SHOT::Problem>(env);
    problem->name = "ex1223b";

    // Creating the variables
    auto x1 = std::make_shared<Variable>("x1", E_VariableType::Integer, 0.0, 3.0);
    auto x2 = std::make_shared<Variable>("x2", E_VariableType::Integer, 1.0, 3.0);

    // All variables are nonlinear, so need to add expression variables as well
    auto nl_x1 = std::make_shared<ExpressionVariable>(x1);
    auto nl_x2 = std::make_shared<ExpressionVariable>(x2);

    // Adding the variables to the problem
    problem->add({ x1, x2 });

    // Creating the objective function
    // minimize -x1 -2x2

    auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    problem->add(objective);

    objective->add(std::make_shared<LinearTerm>(-1.0, x1));
    objective->add(std::make_shared<LinearTerm>(-2.0, x2));

    // Creating the constraint e1: 0.1 e^x2 + x1^2 + x2 <= 10;
    auto e1 = std::make_shared<NonlinearConstraint>("e1", SHOT_DBL_MIN, 10.0);
    e1->add(std::make_shared<QuadraticTerm>(1.0, x1, x1));
    e1->add(std::make_shared<LinearTerm>(1.0, x2));

    e1->add(std::make_shared<ExpressionProduct>(
        std::make_shared<ExpressionConstant>(0.1), std::make_shared<ExpressionExp>(nl_x2)));
    problem->add(e1);

    // Creating the constraint e2: e^x1 / x2  <= 3;
    auto e2 = std::make_shared<NonlinearConstraint>("e2", SHOT_DBL_MIN, 3.0);

    e2->add(std::make_shared<ExpressionDivide>(std::make_shared<ExpressionExp>(nl_x1), nl_x2));
    problem->add(e2);

    NonlinearConstraints constraints = { e1, e2 };

    // Add constraints to a vector

    // Finalize the problem object (after this no changes should be made)
    problem->updateProperties();
    problem->finalize();
    solver->setProblem(problem, problem);

    // Writing the problem to console
    std::cout << '\n';
    std::cout << "Problem created:\n\n";
    std::cout << env->problem << '\n';

    // Writing the reformulated problem to console
    std::cout << '\n';
    std::cout << "Reformulated problem created:\n\n";
    std::cout << env->reformulatedProblem << '\n';

    // Register external hyperplane callback
    solver->registerCallback<HyperplaneSelectionContext>(
        [&env, &constraints](HyperplaneSelectionContext& context)
        {
            std::cout << "External hyperplane callback called at iteration " << context.getIterationNumber()
                      << std::endl;
            std::cout << "Current dual bound: " << context.getDualBound() << std::endl;
            std::cout << "Current primal bound: " << context.getPrimalBound() << std::endl;
            std::cout << "Number of solution points: " << context.getSolutionPoints().size() << std::endl;

            int numberOfHyperplanes = 0;

            // Example: Add a simple cutting plane if we have solution points
            if(!context.getSolutionPoints().empty() && context.getIterationNumber() > 0)
            {
                for(const auto& solPoint : context.getSolutionPoints())
                {
                    std::cout << "\nSolution point: \n";
                    Utilities::displayVector(solPoint.point);

                    // Constraint with largest error
                    auto constraint = constraints.at(solPoint.maxDeviation.index);

                    double funcValue = constraint->calculateFunctionValue(solPoint.point) - constraint->valueRHS;
                    auto gradient = constraint->calculateGradient(solPoint.point, false);

                    // This is just an example - in practice you'd generate meaningful hyperplanes
                    ExternalHyperplane hyperplane;

                    double constant = funcValue;
                    constant += (-gradient[env->reformulatedProblem->getVariable(0)]) * solPoint.point.at(0);
                    constant += (-gradient[env->reformulatedProblem->getVariable(1)]) * solPoint.point.at(1);

                    // Set hyperplane properties
                    hyperplane.variableIndexes = { 0, 1 }; // x1, x2
                    hyperplane.variableCoefficients.emplace_back() = gradient[env->reformulatedProblem->getVariable(0)];
                    hyperplane.variableCoefficients.emplace_back() = gradient[env->reformulatedProblem->getVariable(1)];
                    hyperplane.rhsValue = -constant; // RHS
                    hyperplane.isGlobal = true;
                    hyperplane.description = fmt::format("hyp_{}", context.getIterationNumber());
                    hyperplane.source = E_HyperplaneSource::External;

                    context.addHyperplane(hyperplane);
                    numberOfHyperplanes++;

                    std::cout << "Generated hyperplane variable coefficients: \n";
                    Utilities::displayVector(hyperplane.variableCoefficients);
                    std::cout << "RHS value: " << hyperplane.rhsValue << std::endl;

                    break; // Only add one hyperplane per iterations
                }
            }

            std::cout << "Added " << numberOfHyperplanes << " external hyperplanes" << std::endl;
        });

    solver->solveProblem();

    auto filename = "dualiter_problem.lp";

    env->dualSolver->MIPSolver->writeProblemToFile(filename);

    if(solver->getPrimalSolutions().size() > 0)
    {
        std::cout << "Solution found: \n";
        std::cout << std::endl << "Objective value: " << solver->getPrimalSolution().objValue << std::endl;
        std::cout << std::endl << "Solution point: \n";
        Utilities::displayVector(solver->getPrimalSolution().point);

        if(solver->getPrimalSolution().objValue == -8 && solver->getPrimalSolution().point.size() == 2
            && solver->getPrimalSolution().point[0] == 2 && solver->getPrimalSolution().point[1] == 3)
        {
            std::cout << "Ok, solution is correct!" << std::endl;
            passed = true;
        }
        else
        {
            std::cout << "Error: solution is not correct!" << std::endl;
            passed = false;
        }
    }
    else
    {
        passed = false;
    }

    return passed;
}

// Build the ex1223b MINLP instance inline and return a ready-to-use solver + environment.
// Reused by multiple tests.
static std::pair<std::unique_ptr<SHOT::Solver>, std::shared_ptr<SHOT::Environment>> MakeEx1223bSolver(
    bool forceNonlinear)
{
    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
    if(forceNonlinear)
        solver->updateSetting(
            "Model.Reformulation.Quadratics.Strategy", static_cast<int>(ES_QuadraticProblemStrategy::Nonlinear));

    auto problem = std::make_shared<SHOT::Problem>(env);
    problem->name = "ex1223b";

    auto x1 = std::make_shared<Variable>("x1", E_VariableType::Real, 0.0, 10.0);
    auto x2 = std::make_shared<Variable>("x2", E_VariableType::Real, 0.0, 10.0);
    auto x3 = std::make_shared<Variable>("x3", E_VariableType::Real, 0.0, 10.0);
    auto b4 = std::make_shared<Variable>("b4", E_VariableType::Binary);
    auto b5 = std::make_shared<Variable>("b5", E_VariableType::Binary);
    auto b6 = std::make_shared<Variable>("b6", E_VariableType::Binary);
    auto b7 = std::make_shared<Variable>("b7", E_VariableType::Binary);

    auto nl_x1 = std::make_shared<ExpressionVariable>(x1);
    auto nl_x2 = std::make_shared<ExpressionVariable>(x2);
    auto nl_x3 = std::make_shared<ExpressionVariable>(x3);
    auto nl_b4 = std::make_shared<ExpressionVariable>(b4);
    auto nl_b5 = std::make_shared<ExpressionVariable>(b5);
    auto nl_b6 = std::make_shared<ExpressionVariable>(b6);
    auto nl_b7 = std::make_shared<ExpressionVariable>(b7);

    problem->add({ x1, x2, x3, b4, b5, b6, b7 });

    auto objective = std::make_shared<NonlinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    problem->add(objective);
    objective->add(std::make_shared<ExpressionSquare>(
        std::make_shared<ExpressionSum>(std::make_shared<ExpressionConstant>(-1), nl_b4)));
    objective->add(std::make_shared<ExpressionSquare>(
        std::make_shared<ExpressionSum>(std::make_shared<ExpressionConstant>(-2), nl_b5)));
    objective->add(std::make_shared<ExpressionSquare>(
        std::make_shared<ExpressionSum>(std::make_shared<ExpressionConstant>(-1), nl_b6)));
    objective->add(std::make_shared<ExpressionNegate>(std::make_shared<ExpressionLog>(
        std::make_shared<ExpressionSum>(std::make_shared<ExpressionConstant>(1), nl_b7))));
    objective->add(std::make_shared<ExpressionSquare>(
        std::make_shared<ExpressionSum>(std::make_shared<ExpressionConstant>(-1), nl_x1)));
    objective->add(std::make_shared<ExpressionSquare>(
        std::make_shared<ExpressionSum>(std::make_shared<ExpressionConstant>(-2), nl_x2)));
    objective->add(std::make_shared<ExpressionSquare>(
        std::make_shared<ExpressionSum>(std::make_shared<ExpressionConstant>(-3), nl_x3)));

    auto e1 = std::make_shared<LinearConstraint>("e1", SHOT_DBL_MIN, 5.0);
    e1->add(std::make_shared<LinearTerm>(1.0, x1));
    e1->add(std::make_shared<LinearTerm>(1.0, x2));
    e1->add(std::make_shared<LinearTerm>(1.0, x3));
    e1->add(std::make_shared<LinearTerm>(1.0, b4));
    e1->add(std::make_shared<LinearTerm>(1.0, b5));
    e1->add(std::make_shared<LinearTerm>(1.0, b6));
    problem->add(e1);

    auto e2 = std::make_shared<QuadraticConstraint>("e2", SHOT_DBL_MIN, 5.5);
    e2->add(std::make_shared<QuadraticTerm>(1.0, b6, b6));
    e2->add(std::make_shared<QuadraticTerm>(1.0, x1, x1));
    e2->add(std::make_shared<QuadraticTerm>(1.0, x2, x2));
    e2->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
    problem->add(e2);

    auto e3 = std::make_shared<LinearConstraint>("e3", SHOT_DBL_MIN, 1.2);
    e3->add(std::make_shared<LinearTerm>(1.0, x1));
    e3->add(std::make_shared<LinearTerm>(1.0, b4));
    problem->add(e3);

    auto e4 = std::make_shared<LinearConstraint>("e4", SHOT_DBL_MIN, 1.8);
    e4->add(std::make_shared<LinearTerm>(1.0, x2));
    e4->add(std::make_shared<LinearTerm>(1.0, b5));
    problem->add(e4);

    auto e5 = std::make_shared<LinearConstraint>("e5", SHOT_DBL_MIN, 2.5);
    e5->add(std::make_shared<LinearTerm>(1.0, x3));
    e5->add(std::make_shared<LinearTerm>(1.0, b6));
    problem->add(e5);

    auto e6 = std::make_shared<LinearConstraint>("e6", SHOT_DBL_MIN, 1.2);
    e6->add(std::make_shared<LinearTerm>(1.0, x1));
    e6->add(std::make_shared<LinearTerm>(1.0, b7));
    problem->add(e6);

    auto e7 = std::make_shared<QuadraticConstraint>("e7", SHOT_DBL_MIN, 1.64);
    e7->add(std::make_shared<QuadraticTerm>(1.0, b5, b5));
    e7->add(std::make_shared<QuadraticTerm>(1.0, x2, x2));
    problem->add(e7);

    auto e8 = std::make_shared<QuadraticConstraint>("e8", SHOT_DBL_MIN, 4.25);
    e8->add(std::make_shared<QuadraticTerm>(1.0, b6, b6));
    e8->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
    problem->add(e8);

    auto e9 = std::make_shared<QuadraticConstraint>("e9", SHOT_DBL_MIN, 4.64);
    e9->add(std::make_shared<QuadraticTerm>(1.0, b5, b5));
    e9->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
    problem->add(e9);

    simplifyNonlinearExpressions(problem, true, true, true);
    problem->updateProperties();
    problem->finalize();
    solver->setProblem(problem);

    return { std::move(solver), env };
}

bool TestCallbackPrimalCandidateSelection()
{
    bool passed = true;

    // ── Sub-test 1: callback fires at least once ───────────────────────────
    std::cout << "\nSub-test 1: PrimalCandidateCheck callback fires\n";
    {
        auto [solver, env] = MakeEx1223bSolver();
        int candidateCount = 0;

        solver->registerCallback<PrimalCandidateCheckContext>(
            [&candidateCount](PrimalCandidateCheckContext& candidate)
            {
                candidateCount++;
                std::cout << "  Candidate #" << candidateCount << "  obj=" << candidate.getObjectiveValue()
                          << "  iter=" << candidate.getIterationNumber() << "\n";
                // The candidate is accepted since it is not rejected
            });

        solver->solveProblem();

        if(candidateCount >= 1)
        {
            std::cout << "  OK: callback fired " << candidateCount << " time(s)\n";
        }
        else
        {
            std::cout << "  FAIL: callback never fired\n";
            passed = false;
        }
    }

    // ── Sub-test 2: returning false prevents all primal solutions ──────────
    std::cout << "\nSub-test 2: rejecting every candidate blocks all primal solutions\n";
    {
        auto [solver, env] = MakeEx1223bSolver();
        // Cap iterations so the test terminates quickly
        solver->updateSetting("Termination.IterationLimit", 10);
        int rejectedCount = 0;

        solver->registerCallback<PrimalCandidateCheckContext>(
            [&rejectedCount](PrimalCandidateCheckContext& candidate)
            {
                rejectedCount++;
                std::cout << "  Rejecting candidate obj=" << candidate.getObjectiveValue() << "\n";
                candidate.rejectCandidate(); // reject everything
            });

        solver->solveProblem();

        if(rejectedCount >= 1 && solver->getPrimalSolutions().empty())
        {
            std::cout << "  OK: rejected " << rejectedCount << " candidate(s), no primal solution recorded\n";
        }
        else
        {
            std::cout << "  FAIL: rejectedCount=" << rejectedCount
                      << "  primalSolutions=" << solver->getPrimalSolutions().size() << "\n";
            passed = false;
        }
    }

    // ── Sub-test 3: selective rejection reduces number of incumbents ───────
    std::cout << "\nSub-test 3: selective rejection reduces accepted incumbents\n";
    {
        // Run A: accept all
        int acceptedA = 0;
        {
            auto [solver, env] = MakeEx1223bSolver();
            solver->registerCallback<PrimalCandidateCheckContext>([](PrimalCandidateCheckContext&) { });
            solver->registerCallback<NewPrimalSolutionContext>(
                [&acceptedA](NewPrimalSolutionContext&) { acceptedA++; });
            solver->solveProblem();
        }

        // Run B: reject candidates with obj > 5
        int acceptedB = 0;
        {
            auto [solver, env] = MakeEx1223bSolver();
            solver->registerCallback<PrimalCandidateCheckContext>(
                [](PrimalCandidateCheckContext& candidate)
                {
                    if(candidate.getObjectiveValue() > 5.0)
                        candidate.rejectCandidate();
                });
            solver->registerCallback<NewPrimalSolutionContext>(
                [&acceptedB](NewPrimalSolutionContext&) { acceptedB++; });
            solver->solveProblem();
        }

        std::cout << "  Accept-all incumbents: " << acceptedA << "  Selective incumbents: " << acceptedB << "\n";

        if(acceptedB <= acceptedA)
        {
            std::cout << "  OK: selective rejection did not produce more incumbents\n";
        }
        else
        {
            std::cout << "  FAIL: selective rejection produced MORE incumbents than accept-all\n";
            passed = false;
        }
    }

    if(!passed)
        std::cout << "\nTestCallbackPrimalCandidateSelection FAILED\n";
    else
        std::cout << "\nTestCallbackPrimalCandidateSelection PASSED\n";

    return passed;
}

bool TestCallbackESHInteriorPoint()
{
    bool passed = true;

    // ── Phase 1: Solve ex1223b with default ESH strategy and capture the interior point ──
    std::cout << "\nPhase 1: Solving ex1223b with default ESH strategy to obtain an interior point\n";

    VectorDouble capturedInteriorPoint;
    double phase1ObjValue = SHOT_DBL_MAX;

    {
        auto [solver, env] = MakeEx1223bSolver(true); // force nonlinear strategy so TaskFindInteriorPoint runs

        solver->solveProblem();

        if(solver->getPrimalSolutions().empty())
        {
            std::cout << "Phase 1 FAILED: could not find a primal solution\n";
            return false;
        }

        phase1ObjValue = solver->getPrimalSolution().objValue;
        std::cout << "Phase 1 primal objective: " << phase1ObjValue << "\n";

        if(env->dualSolver->interiorPts.empty())
        {
            std::cout << "Phase 1 FAILED: no interior points found by internal strategy\n";
            return false;
        }

        capturedInteriorPoint = env->dualSolver->interiorPts[0]->point;

        std::cout << "Captured interior point (first " << capturedInteriorPoint.size() << " variables):\n";
        Utilities::displayVector(capturedInteriorPoint);
    }

    // ── Phase 2a: Verify callback fires during normal ESH ──
    std::cout << "\nPhase 2a: Verify ESH interior point callback fires during normal ESH\n";
    {
        auto [solver, env] = MakeEx1223bSolver(true); // force nonlinear strategy so TaskFindInteriorPoint runs
        bool callbackFired = false;
        size_t pointsReceived = 0;

        solver->registerCallback<InteriorPointSearchContext>(
            [&](InteriorPointSearchContext& context)
            {
                callbackFired = true;
                pointsReceived = context.getInteriorPoints().size();
                std::cout << "  ESH interior point callback fired with " << pointsReceived << " current point(s)\n";
                // No points are set, so the current points are kept
            });

        solver->solveProblem();

        if(!callbackFired)
        {
            std::cout << "Phase 2a FAILED: ESH interior point callback was not fired\n";
            passed = false;
        }
        else if(pointsReceived == 0)
        {
            std::cout << "Phase 2a FAILED: callback fired but received no interior points\n";
            passed = false;
        }
        else
        {
            std::cout << "Phase 2a PASSED: callback fired with " << pointsReceived << " interior point(s)\n";
        }
    }

    // ── Phase 2b: Solve with OnlyExternal strategy + inject captured point via callback ──
    std::cout << "\nPhase 2b: Solving with ESH.InteriorPoint.Strategy=OnlyExternal and injecting captured point\n";
    {
        auto [solver, env] = MakeEx1223bSolver(true); // force nonlinear strategy so TaskFindInteriorPoint runs

        solver->updateSetting(
            "Dual.ESH.InteriorPoint.Strategy", static_cast<int>(ES_ESHInteriorPointStrategy::OnlyExternal));

        bool callbackFired = false;

        solver->registerCallback<InteriorPointSearchContext>(
            [&](InteriorPointSearchContext& context)
            {
                callbackFired = true;
                std::cout << "  ESH interior point callback fired (OnlyExternal strategy)\n";
                std::cout << "  interior points from callback: " << context.getInteriorPoints().size()
                          << " (should be 0)\n";
                // Use the captured point from Phase 1
                context.setInteriorPoints({ capturedInteriorPoint });
            });

        solver->solveProblem();

        if(!callbackFired)
        {
            std::cout << "Phase 2b FAILED: callback was not fired\n";
            passed = false;
        }
        else if(solver->getPrimalSolutions().empty())
        {
            std::cout << "Phase 2b FAILED: no primal solution found\n";
            passed = false;
        }
        else
        {
            double phase2ObjValue = solver->getPrimalSolution().objValue;
            std::cout << "Phase 2b primal objective: " << phase2ObjValue << "\n";

            const double tol = 1e-4;
            if(std::abs(phase2ObjValue - phase1ObjValue) <= tol + tol * std::abs(phase1ObjValue))
            {
                std::cout << "Phase 2b PASSED: objective matches Phase 1 within tolerance\n";
            }
            else
            {
                std::cout << "Phase 2b FAILED: objective " << phase2ObjValue << " differs from Phase 1 ("
                          << phase1ObjValue << ") by more than tolerance\n";
                passed = false;
            }
        }
    }

    if(!passed)
        std::cout << "\nTestCallbackESHInteriorPoint FAILED\n";
    else
        std::cout << "\nTestCallbackESHInteriorPoint PASSED\n";

    return passed;
}

// Builds a modified ex1223b where the objective is to minimize an auxiliary variable mu,
// which is subtracted from each quadratic constraint. The optimal solution gives a point
// that is maximally interior to those constraints (mu < 0 means strictly interior).
static std::pair<std::unique_ptr<SHOT::Solver>, std::shared_ptr<SHOT::Environment>> MakeEx1223bInteriorPointSolver()
{
    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
    // Treat quadratic constraints as nonlinear so the interior point is valid for ESH
    solver->updateSetting(
        "Model.Reformulation.Quadratics.Strategy", static_cast<int>(ES_QuadraticProblemStrategy::Nonlinear));

    auto problem = std::make_shared<SHOT::Problem>(env);
    problem->name = "ex1223b_interior";

    // Original variables — all continuous (binary variables relaxed to [0,1])
    auto x1 = std::make_shared<Variable>("x1", E_VariableType::Real, 0.0, 10.0);
    auto x2 = std::make_shared<Variable>("x2", E_VariableType::Real, 0.0, 10.0);
    auto x3 = std::make_shared<Variable>("x3", E_VariableType::Real, 0.0, 10.0);
    auto b4 = std::make_shared<Variable>("b4", E_VariableType::Real, 0.0, 1.0);
    auto b5 = std::make_shared<Variable>("b5", E_VariableType::Real, 0.0, 1.0);
    auto b6 = std::make_shared<Variable>("b6", E_VariableType::Real, 0.0, 1.0);
    auto b7 = std::make_shared<Variable>("b7", E_VariableType::Real, 0.0, 1.0);

    // Auxiliary variable mu: the objective is to minimize mu
    auto mu = std::make_shared<Variable>("mu", E_VariableType::Real, -100.0, 100.0);

    problem->add({ x1, x2, x3, b4, b5, b6, b7, mu });

    // Objective: minimize mu
    auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    objective->add(std::make_shared<LinearTerm>(1.0, mu));
    problem->add(objective);

    // Linear constraints: unchanged from ex1223b
    auto e1 = std::make_shared<LinearConstraint>("e1", SHOT_DBL_MIN, 5.0);
    e1->add(std::make_shared<LinearTerm>(1.0, x1));
    e1->add(std::make_shared<LinearTerm>(1.0, x2));
    e1->add(std::make_shared<LinearTerm>(1.0, x3));
    e1->add(std::make_shared<LinearTerm>(1.0, b4));
    e1->add(std::make_shared<LinearTerm>(1.0, b5));
    e1->add(std::make_shared<LinearTerm>(1.0, b6));
    problem->add(e1);

    // Quadratic constraints with -mu added: f(x) - mu <= rhs
    auto e2 = std::make_shared<QuadraticConstraint>("e2", SHOT_DBL_MIN, 5.5);
    e2->add(std::make_shared<QuadraticTerm>(1.0, b6, b6));
    e2->add(std::make_shared<QuadraticTerm>(1.0, x1, x1));
    e2->add(std::make_shared<QuadraticTerm>(1.0, x2, x2));
    e2->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
    e2->add(std::make_shared<LinearTerm>(-1.0, mu));
    problem->add(e2);

    auto e3 = std::make_shared<LinearConstraint>("e3", SHOT_DBL_MIN, 1.2);
    e3->add(std::make_shared<LinearTerm>(1.0, x1));
    e3->add(std::make_shared<LinearTerm>(1.0, b4));
    problem->add(e3);

    auto e4 = std::make_shared<LinearConstraint>("e4", SHOT_DBL_MIN, 1.8);
    e4->add(std::make_shared<LinearTerm>(1.0, x2));
    e4->add(std::make_shared<LinearTerm>(1.0, b5));
    problem->add(e4);

    auto e5 = std::make_shared<LinearConstraint>("e5", SHOT_DBL_MIN, 2.5);
    e5->add(std::make_shared<LinearTerm>(1.0, x3));
    e5->add(std::make_shared<LinearTerm>(1.0, b6));
    problem->add(e5);

    auto e6 = std::make_shared<LinearConstraint>("e6", SHOT_DBL_MIN, 1.2);
    e6->add(std::make_shared<LinearTerm>(1.0, x1));
    e6->add(std::make_shared<LinearTerm>(1.0, b7));
    problem->add(e6);

    auto e7 = std::make_shared<QuadraticConstraint>("e7", SHOT_DBL_MIN, 1.64);
    e7->add(std::make_shared<QuadraticTerm>(1.0, b5, b5));
    e7->add(std::make_shared<QuadraticTerm>(1.0, x2, x2));
    e7->add(std::make_shared<LinearTerm>(-1.0, mu));
    problem->add(e7);

    auto e8 = std::make_shared<QuadraticConstraint>("e8", SHOT_DBL_MIN, 4.25);
    e8->add(std::make_shared<QuadraticTerm>(1.0, b6, b6));
    e8->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
    e8->add(std::make_shared<LinearTerm>(-1.0, mu));
    problem->add(e8);

    auto e9 = std::make_shared<QuadraticConstraint>("e9", SHOT_DBL_MIN, 4.64);
    e9->add(std::make_shared<QuadraticTerm>(1.0, b5, b5));
    e9->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
    e9->add(std::make_shared<LinearTerm>(-1.0, mu));
    problem->add(e9);

    simplifyNonlinearExpressions(problem, true, true, true);
    problem->updateProperties();
    problem->finalize();
    solver->setProblem(problem);

    return { std::move(solver), env };
}

// Returns the worst (most positive) constraint error across all constraint types for a primal solution.
static double maxConstraintError(const PrimalSolution& sol)
{
    double err = SHOT_DBL_MIN;
    if(sol.maxDevatingConstraintLinear.index >= 0)
        err = std::max(err, sol.maxDevatingConstraintLinear.value);
    if(sol.maxDevatingConstraintQuadratic.index >= 0)
        err = std::max(err, sol.maxDevatingConstraintQuadratic.value);
    if(sol.maxDevatingConstraintNonlinear.index >= 0)
        err = std::max(err, sol.maxDevatingConstraintNonlinear.value);
    return err;
}

static void printPrimalSolutionConstraintErrors(const PrimalSolution& sol)
{
    if(sol.maxDevatingConstraintLinear.index >= 0)
        std::cout << "  max linear constraint error:    constraint " << sol.maxDevatingConstraintLinear.index
                  << "  error = " << sol.maxDevatingConstraintLinear.value << "\n";
    if(sol.maxDevatingConstraintQuadratic.index >= 0)
        std::cout << "  max quadratic constraint error: constraint " << sol.maxDevatingConstraintQuadratic.index
                  << "  error = " << sol.maxDevatingConstraintQuadratic.value << "\n";
    if(sol.maxDevatingConstraintNonlinear.index >= 0)
        std::cout << "  max nonlinear constraint error: constraint " << sol.maxDevatingConstraintNonlinear.index
                  << "  error = " << sol.maxDevatingConstraintNonlinear.value << "\n";
}

bool TestCallbackESHExternalInteriorPointFromAuxProblem()
{
    bool passed = true;

    // ── Phase 1: Solve modified ex1223b (minimize mu) to obtain a strictly interior point ──
    // The modified problem subtracts an auxiliary variable mu from each quadratic constraint.
    // Minimizing mu maximizes the slack, yielding a point that is as deep in the interior
    // of the nonlinear feasible region as possible. If optimal mu < 0, the point is strictly
    // interior to all quadratic constraints.
    std::cout << "\nPhase 1: Solving modified ex1223b (minimize mu) to obtain an interior point\n";

    VectorDouble interiorPoint; // original variables only (x1..b7), mu excluded

    {
        auto [solver, env] = MakeEx1223bInteriorPointSolver();

        solver->solveProblem();

        if(solver->getPrimalSolutions().empty())
        {
            std::cout << "Phase 1 FAILED: could not find a solution\n";
            return false;
        }

        const auto& sol = solver->getPrimalSolution();
        std::cout << "Phase 1 optimal mu (objective): " << sol.objValue << "\n";
        std::cout << "Phase 1 full solution point (x1..b7, mu):\n";
        Utilities::displayVector(sol.point);
        std::cout << "Phase 1 constraint errors:\n";
        printPrimalSolutionConstraintErrors(sol);

        if(sol.objValue >= 0.0)
            std::cout << "  Warning: mu >= 0 (" << sol.objValue
                      << "); the point may not be strictly interior to all quadratic constraints\n";
        else
            std::cout << "  mu = " << sol.objValue << " < 0: point is strictly interior to quadratic constraints\n";

        // The interior point problem has 8 variables (x1,x2,x3,b4,b5,b6,b7,mu).
        // Drop mu (last entry) to get the 7-variable point for the original problem.
        interiorPoint.assign(sol.point.begin(), sol.point.begin() + 7);
        std::cout << "Phase 1 interior point (original variables, mu dropped):\n";
        Utilities::displayVector(interiorPoint);
    }

    // ── Phase 2: Solve original ex1223b with OnlyExternal + inject the interior point ──
    std::cout << "\nPhase 2: Solving original ex1223b with ESH.InteriorPoint.Strategy=OnlyExternal\n"
                 "         and injecting the Phase 1 interior point\n";
    {
        auto [solver, env] = MakeEx1223bSolver(true); // force nonlinear strategy

        solver->updateSetting(
            "Dual.ESH.InteriorPoint.Strategy", static_cast<int>(ES_ESHInteriorPointStrategy::OnlyExternal));

        bool callbackFired = false;

        solver->registerCallback<InteriorPointSearchContext>(
            [&](InteriorPointSearchContext& context)
            {
                callbackFired = true;
                std::cout << "  ESH interior point callback fired (OnlyExternal strategy)\n";
                std::cout << "  Injecting Phase 1 interior point\n";
                context.setInteriorPoints({ interiorPoint });
            });

        solver->solveProblem();

        if(!callbackFired)
        {
            std::cout << "Phase 2 FAILED: callback was not fired\n";
            passed = false;
        }
        else if(solver->getPrimalSolutions().empty())
        {
            std::cout << "Phase 2 FAILED: no primal solution found\n";
            passed = false;
        }
        else
        {
            double phase2ObjValue = solver->getPrimalSolution().objValue;
            std::cout << "Phase 2 primal objective: " << phase2ObjValue << "\n";
            std::cout << "Phase 2 PASSED: solution found with injected interior point\n";
        }
    }

    if(!passed)
        std::cout << "\nTestCallbackESHExternalInteriorPointFromAuxProblem FAILED\n";
    else
        std::cout << "\nTestCallbackESHExternalInteriorPointFromAuxProblem PASSED\n";

    return passed;
}

bool TestAMPLInitialValues()
{
    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Error));

    if(!solver->setProblem("data/ncvx_min_div.nl"))
    {
        std::cout << "  Could not load data/ncvx_min_div.nl\n";
        return false;
    }

    // ncvx_min_div.nl: x1 segment gives var 2 = 1.0; vars 0,1 have no initial value so get lb (0.5, 0.1)
    // 0 constraints => initial point is always feasible and accepted as a primal solution
    auto sols = solver->getPrimalSolutions();
    if(sols.empty())
    {
        std::cout << "  No primal solution found from initial values\n";
        return false;
    }

    auto& pt = sols.back().point;
    bool passed = std::abs(pt[0] - 0.5) < 1e-10 && std::abs(pt[1] - 0.1) < 1e-10 && std::abs(pt[2] - 1.0) < 1e-10;

    std::cout << fmt::format("  Initial point: ({}, {}, {})", pt[0], pt[1], pt[2]);
    if(passed)
        std::cout << " - correct\n";
    else
        std::cout << fmt::format(" - FAILED, expected (0.5, 0.1, 1.0)\n");

    return passed;
}

bool TestAMPLInitialValuesOutOfBounds()
{
    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Error));

    if(!solver->setProblem("data/ncvx_min_div_oob.nl"))
    {
        std::cout << "  Could not load data/ncvx_min_div_oob.nl\n";
        return false;
    }

    // x3 segment: var 0=0.8 (ub=0.6), var 1=0.05 (lb=0.1), var 2=2.0 (ub=1.0)
    // all three are out of bounds and should be projected to {0.6, 0.1, 1.0}
    auto sols = solver->getPrimalSolutions();
    if(sols.empty())
    {
        std::cout << "  No primal solution found from out-of-bounds initial values\n";
        return false;
    }

    auto& sol = sols.back();
    if(!sol.boundProjectionPerformed)
    {
        std::cout << "  Expected bound projection to be performed\n";
        return false;
    }

    auto& pt = sol.point;
    bool passed = std::abs(pt[0] - 0.6) < 1e-10 && std::abs(pt[1] - 0.1) < 1e-10 && std::abs(pt[2] - 1.0) < 1e-10;

    std::cout << fmt::format("  Raw initial values: (0.8, 0.05, 2.0)\n");
    std::cout << fmt::format("  Projected point:    ({}, {}, {})", pt[0], pt[1], pt[2]);
    if(passed)
        std::cout << " - correct\n";
    else
        std::cout << fmt::format(" - FAILED, expected (0.6, 0.1, 1.0)\n");

    return passed;
}

// Verifies that an ordinary solve is never reported as a user abort. The dual solver is interrupted by SHOT itself
// whenever a termination criterion is met in a callback, and that interruption used to be indistinguishable from a
// termination requested by the user.
// The dual bound of a convex problem must be closed even when a constraint is infinite outside its domain. The
// perspective x^2/z of nlp-cvx_204_010 is infinite where z is zero, and both the interior point search and the
// generation of the hyperplanes ended up in such a point: no interior point was found, the hyperplanes generated
// in the solution point had infinite coefficients and were thrown away, and the solve stopped with the dual bound
// -2 for the optimum -1.207 and the message that no additional dual cuts can be added. Note that the instance
// tests accept that outcome, since they only require the optimum to lie between the bounds.
bool TestDualBoundOfPerspectiveConstraint()
{
    const double expectedObjective = -1.2071067837918394;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Critical));
    solver->updateSetting("Termination.TimeLimit", 30.0);

    if(!solver->setProblem("data/instances/minlp_tests_jl/nlp-cvx_204_010.jl.nl"))
    {
        std::cout << "Could not set problem!\n";
        return (false);
    }

    if(!solver->solveProblem())
    {
        std::cout << "Error while solving problem\n";
        return (false);
    }

    if(env->results->terminationReason != E_TerminationReason::AbsoluteGap
        && env->results->terminationReason != E_TerminationReason::RelativeGap)
    {
        std::cout << "The objective gap was not closed: " << env->results->terminationReasonDescription << '\n';
        return (false);
    }

    double primalBound = solver->getPrimalBound();
    double dualBound = solver->getGlobalDualBound();

    std::cout << "  Objective bounds are [" << dualBound << ", " << primalBound << "], the optimum is "
              << expectedObjective << '\n';

    if(std::abs(primalBound - expectedObjective) > 1e-2 || std::abs(dualBound - expectedObjective) > 1e-2)
    {
        std::cout << "The bounds do not agree with the optimum.\n";
        return (false);
    }

    return (true);
}

bool TestTerminationReasonOfOrdinarySolve()
{
    auto [solver, env] = MakeEx1223bSolver();
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Critical));

    if(!solver->solveProblem())
    {
        std::cout << "Error while solving problem\n";
        return (false);
    }

    if(env->results->terminationReason == E_TerminationReason::UserAbort)
    {
        std::cout << "An ordinary solve was reported as a user abort: " << env->results->terminationReasonDescription
                  << '\n';
        return (false);
    }

    if(env->results->terminationReason != E_TerminationReason::AbsoluteGap
        && env->results->terminationReason != E_TerminationReason::RelativeGap
        && env->results->terminationReason != E_TerminationReason::ConstraintTolerance)
    {
        std::cout << "Unexpected termination reason: " << env->results->terminationReasonDescription << '\n';
        return (false);
    }

    // The interrupted dual problem must still be accounted for in the statistics
    auto& stats = env->solutionStatistics;
    int solvedDiscreteDualProblems = stats.numberOfProblemsOptimalMILP + stats.numberOfProblemsFeasibleMILP
        + stats.numberOfProblemsOptimalMIQP + stats.numberOfProblemsFeasibleMIQP
        + stats.numberOfProblemsOptimalMIQCQP + stats.numberOfProblemsFeasibleMIQCQP;

    if(solvedDiscreteDualProblems == 0)
    {
        std::cout << "No discrete dual problems were counted in the solution statistics\n";
        return (false);
    }

    return (true);
}

// A constraint whose nonlinear or quadratic terms only involve fixed variables holds nothing nonlinear once those
// terms are folded into the constant, and must be classified as the linear constraint it has become. Leaving it
// among the nonlinear or quadratic constraints puts it in a class it does not belong to, where it is counted in
// none of them and, in the nl case, is carried along with no variables left in its expression.
bool TestConstraintClassesForFixedVariables(const std::string& problemFile)
{
    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Critical));

    if(!solver->setProblem(problemFile))
    {
        std::cout << "Error while reading " << problemFile << '\n';
        return (false);
    }

    bool passed = true;

    for(auto& problem : { env->problem, env->reformulatedProblem })
    {
        auto& properties = problem->properties;

        if(properties.numberOfNumericConstraints
            != properties.numberOfLinearConstraints + properties.numberOfQuadraticConstraints
                + properties.numberOfNonlinearConstraints)
        {
            std::cout << "The constraint classes do not add up to the number of constraints: "
                      << properties.numberOfNumericConstraints << " != " << properties.numberOfLinearConstraints
                      << " + " << properties.numberOfQuadraticConstraints << " + "
                      << properties.numberOfNonlinearConstraints << '\n';
            passed = false;
        }

        for(auto& C : problem->nonlinearConstraints)
        {
            if(C->properties.hasNonlinearExpression && C->variablesInNonlinearExpression.size() == 0)
            {
                std::cout << "Constraint " << C->name << " is nonlinear but its expression has no variables\n";
                passed = false;
            }
        }
    }

    if(!solver->solveProblem())
    {
        std::cout << "Error while solving " << problemFile << '\n';
        return (false);
    }

    double objectiveValue = env->results->getPrimalBound();

    if(std::abs(objectiveValue - 1.0) > 1e-6)
    {
        std::cout << "Expected an objective value of 1, got " << objectiveValue << '\n';
        passed = false;
    }

    return passed;
}

bool TestCallbackGenericContext()
{
    bool passed = true;

    // A callback registered for two locations is called at exactly those locations, with the context classes for
    // them
    auto [solver, env] = MakeEx1223bSolver();
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));

    std::set<E_CallbackLocation> locations;
    bool contextClassesMatch = true;

    solver->registerCallback(E_CallbackLocation::PrimalCandidateCheck | E_CallbackLocation::TerminationCheck,
        [&locations, &contextClassesMatch](CallbackContext& context)
        {
            locations.insert(context.getLocation());

            auto candidate = context.as<PrimalCandidateCheckContext>();
            auto termination = context.as<TerminationCheckContext>();

            // Exactly one of the classes is that of the location
            if((candidate != nullptr) == (termination != nullptr))
                contextClassesMatch = false;

            if(context.as<NewPrimalSolutionContext>() != nullptr)
                contextClassesMatch = false;

            if(candidate != nullptr && candidate->getPoint().size() != 7)
                contextClassesMatch = false;
        });

    solver->solveProblem();

    std::set<E_CallbackLocation> expectedLocations
        = { E_CallbackLocation::PrimalCandidateCheck, E_CallbackLocation::TerminationCheck };

    if(locations != expectedLocations)
    {
        std::cout << "The callback was not called at exactly the locations it was registered for\n";
        passed = false;
    }

    if(!contextClassesMatch)
    {
        std::cout << "The class of a context was not that of its location\n";
        passed = false;
    }

    return passed;
}

bool TestCallbackFailure()
{
    bool passed = true;

    // An exception in a callback discards the actions of the callbacks at that location, stops all callbacks and
    // SHOT, and is rethrown by solveProblem()
    auto [solver, env] = MakeEx1223bSolver(true); // the nonlinear strategy has the PrimalCandidateSearch location
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));

    bool failed = false;
    int callsAfterFailure = 0;
    int laterCallbackCalls = 0;

    auto locations = E_CallbackLocation::PrimalCandidateSearch | E_CallbackLocation::NewPrimalSolution
        | E_CallbackLocation::TerminationCheck;

    solver->registerCallback(locations,
        [&failed, &callsAfterFailure](CallbackContext& context)
        {
            if(failed)
            {
                callsAfterFailure++;
                return;
            }

            if(auto search = context.as<PrimalCandidateSearchContext>())
            {
                // A feasible solution is queued, but must be discarded since the callback fails
                search->addPrimalSolution(VectorDouble(7, 0.0));
                failed = true;
                throw std::runtime_error("the callback failed");
            }
        });

    solver->registerCallback(locations,
        [&failed, &callsAfterFailure, &laterCallbackCalls](CallbackContext& context)
        {
            if(context.getLocation() == E_CallbackLocation::PrimalCandidateSearch)
                laterCallbackCalls++;

            if(failed)
                callsAfterFailure++;
        });

    bool exceptionRethrown = false;

    try
    {
        solver->solveProblem();
    }
    catch(const std::runtime_error& e)
    {
        exceptionRethrown = (std::string(e.what()) == "the callback failed");
    }

    if(!failed || !exceptionRethrown)
    {
        std::cout << "The exception of the callback was not rethrown by solveProblem()\n";
        passed = false;
    }

    if(laterCallbackCalls != 0 || callsAfterFailure != 0)
    {
        std::cout << "Callbacks were called after the failure: " << laterCallbackCalls << " at the failing location, "
                  << callsAfterFailure << " in total\n";
        passed = false;
    }

    if(env->results->terminationReason != E_TerminationReason::Error)
    {
        std::cout << "The termination reason is not Error\n";
        passed = false;
    }

    for(auto& S : env->results->primalSolutions)
    {
        if(S.sourceType == E_PrimalSolutionSource::ExternalPrimalSolution)
        {
            std::cout << "The solution queued by the failing callback was added\n";
            passed = false;
        }
    }

    return passed;
}

bool TestPrimalSolutionPool()
{
    bool passed = true;

    // The solution pool keeps the best solutions, sorted with the incumbent first, and only a solution better than
    // the incumbent changes the primal bound
    auto [solver, env] = MakeEx1223bSolver();
    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
    solver->updateSetting("Output.SaveNumberOfSolutions", 3);

    auto makeSolution = [](double objectiveValue, double firstValue)
    {
        PrimalSolution solution;
        solution.point = VectorDouble(7, 0.0);
        solution.point[0] = firstValue;
        solution.objValue = objectiveValue;
        solution.sourceType = E_PrimalSolutionSource::ExternalPrimalSolution;
        return (solution);
    };

    auto check = [&passed, &env](const std::string& step, std::vector<double> expectedObjectives,
                     double expectedIncumbentFirstValue)
    {
        auto& pool = env->results->primalSolutions;

        std::vector<double> objectives;
        for(auto& S : pool)
            objectives.push_back(S.objValue);

        if(objectives != expectedObjectives)
        {
            std::cout << step << ": the objective values in the solution pool are not the expected ones\n";
            passed = false;
            return;
        }

        if(env->results->getPrimalBound() != expectedObjectives.front()
            || env->results->primalSolution.at(0) != expectedIncumbentFirstValue
            || pool.front().point.at(0) != expectedIncumbentFirstValue)
        {
            std::cout << step << ": the incumbent or the primal bound is not the expected one\n";
            passed = false;
        }
    };

    env->results->addPrimalSolution(makeSolution(10.0, 1.0));
    check("First solution", { 10.0 }, 1.0);

    env->results->addPrimalSolution(makeSolution(12.0, 2.0));
    check("Worse solution", { 10.0, 12.0 }, 1.0);

    // Better than the worst solution but worse than the incumbent: added to the pool, but not the incumbent
    env->solutionStatistics.hasReductionCutBeenAddedSincePrimalImprovement = true;
    env->results->addPrimalSolution(makeSolution(11.0, 3.0));
    check("Solution between the best and the worst", { 10.0, 11.0, 12.0 }, 1.0);

    if(!env->solutionStatistics.hasReductionCutBeenAddedSincePrimalImprovement)
    {
        std::cout << "A solution that is not an improvement was counted as one\n";
        passed = false;
    }

    // The pool is full and the solution is worse than all in it
    env->results->addPrimalSolution(makeSolution(13.0, 4.0));
    check("Worse solution with a full pool", { 10.0, 11.0, 12.0 }, 1.0);

    // A new incumbent replaces the worst solution
    env->results->addPrimalSolution(makeSolution(9.0, 5.0));
    check("Better solution with a full pool", { 9.0, 10.0, 11.0 }, 5.0);

    if(env->solutionStatistics.hasReductionCutBeenAddedSincePrimalImprovement)
    {
        std::cout << "An improvement of the primal bound was not counted as one\n";
        passed = false;
    }

    // A solution with the same objective value as the incumbent but a smaller constraint error replaces it
    auto moreAccurate = makeSolution(9.0, 6.0);
    moreAccurate.maxDevatingConstraintLinear = PairIndexValue(-1, 0.0);
    moreAccurate.maxDevatingConstraintQuadratic = PairIndexValue(-1, 0.0);
    moreAccurate.maxDevatingConstraintNonlinear = PairIndexValue(-1, 0.0);
    env->results->addPrimalSolution(moreAccurate);
    check("More accurate solution", { 9.0, 9.0, 10.0 }, 6.0);

    // The objective values are only compared up to a relative tolerance of 1e-10. In a full pool of two, the incumbent
    // has the objective value 9 and the error 5, and the other solution an almost equal objective value and the
    // error 1. A solution with the same objective value as the other one and the error 3 replaces the incumbent,
    // although it is neither better nor more accurate than the worst solution in the pool
    auto [solver2, env2] = MakeEx1223bSolver();
    solver2->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Off));
    solver2->updateSetting("Output.SaveNumberOfSolutions", 2);

    auto withError = [&makeSolution](double objectiveValue, double firstValue, double error)
    {
        auto solution = makeSolution(objectiveValue, firstValue);
        solution.maxDevatingConstraintLinear = PairIndexValue(-1, error);
        solution.maxDevatingConstraintQuadratic = PairIndexValue(-1, error);
        solution.maxDevatingConstraintNonlinear = PairIndexValue(-1, error);
        return (solution);
    };

    double almostNine = 9.0 + 1e-10;

    env2->results->addPrimalSolution(withError(almostNine, 1.0, 1.0));
    env2->results->addPrimalSolution(withError(9.0, 2.0, 5.0));
    env2->results->addPrimalSolution(withError(almostNine, 3.0, 3.0));

    auto& pool = env2->results->primalSolutions;

    if(pool.size() != 2 || pool.front().point.at(0) != 3.0 || env2->results->primalSolution.at(0) != 3.0
        || pool.back().point.at(0) != 2.0)
    {
        std::cout << "A more accurate solution with an almost equal objective value did not replace the incumbent in "
                     "a full pool\n";
        passed = false;
    }

    return passed;
}

int SolverTest(int argc, char* argv[])
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
        std::cout << "Starting test to read OSiL files:" << std::endl;
        passed = ReadProblem("data/tls2.osil");
        std::cout << "Finished test to read OSiL files." << std::endl;
        break;
    case 2:
        std::cout << "Starting test to read NL files:" << std::endl;
        passed = ReadProblem("data/tls2.nl");
        std::cout << "Finished test to read NL files." << std::endl;
        break;
    case 3:
        std::cout << "Starting test to solve a MINLP problem in OSiL syntax:" << std::endl;
        passed = SolveProblem("data/tls2.osil");
        std::cout << "Finished test to solve a MINLP problem in OSiL syntax." << std::endl;
        break;
    case 4:
        std::cout << "Starting test to evaluate gradients in OSiL file:" << std::endl;
        passed = TestGradient("data/flay02h.osil");
        std::cout << "Finished test to evaluate gradients in OSiL file." << std::endl;
        break;
    case 5:
        std::cout << "Starting test solving model using SHOT API:" << std::endl;
        passed = CreateAndSolveProblem();
        std::cout << "Finished test solving model using SHOT API." << std::endl;
        break;
    case 6:
        std::cout << "Starting test to read OSiL file with semicont. variables:" << std::endl;
        passed = ReadProblem("data/meanvarxsc.osil");
        std::cout << "Finished test to read OSiL file with semicont. variables." << std::endl;
        break;
    case 7:
        std::cout << "Starting test for callback system - user termination check:" << std::endl;
        passed = TestCallbackUserTermination();
        std::cout << "Finished test for callback system - user termination check." << std::endl;
        break;
    case 8:
        std::cout << "Starting test for callback system - external hyperplanes" << std::endl;
        passed = TestCallbackExternalHyperplane();
        std::cout << "Finished test for callback system - external hyperplanes." << std::endl;
        break;
    case 9:
        std::cout << "Starting test for callback system - primal candidate selection" << std::endl;
        passed = TestCallbackPrimalCandidateSelection();
        std::cout << "Finished test for callback system - primal candidate selection." << std::endl;
        break;
    case 10:
        std::cout << "Starting test for callback system - ESH interior point" << std::endl;
        passed = TestCallbackESHInteriorPoint();
        std::cout << "Finished test for callback system - ESH interior point." << std::endl;
        break;
    case 11:
        std::cout << "Starting test for callback system - ESH external interior point from auxiliary problem"
                  << std::endl;
        passed = TestCallbackESHExternalInteriorPointFromAuxProblem();
        std::cout << "Finished test for callback system - ESH external interior point from auxiliary problem."
                  << std::endl;
        break;
    case 12:
        std::cout << "Starting test for AMPL initial values" << std::endl;
        passed = TestAMPLInitialValues();
        std::cout << "Finished test for AMPL initial values." << std::endl;
        break;
    case 13:
        std::cout << "Starting test for AMPL initial values out of bounds" << std::endl;
        passed = TestAMPLInitialValuesOutOfBounds();
        std::cout << "Finished test for AMPL initial values out of bounds." << std::endl;
        break;
    case 14:
        std::cout << "Starting test for the termination reason of an ordinary solve" << std::endl;
        passed = TestTerminationReasonOfOrdinarySolve();
        std::cout << "Finished test for the termination reason of an ordinary solve." << std::endl;
        break;
    case 15:
        std::cout << "Starting test for constraint classes with fixed variables (nl format)" << std::endl;
        passed = TestConstraintClassesForFixedVariables("data/fixedvars.nl");
        std::cout << "Finished test for constraint classes with fixed variables (nl format)." << std::endl;
        break;
    case 16:
        std::cout << "Starting test for constraint classes with fixed variables (osil format)" << std::endl;
        passed = TestConstraintClassesForFixedVariables("data/fixedvars.osil");
        std::cout << "Finished test for constraint classes with fixed variables (osil format)." << std::endl;
        break;
    case 17:
        std::cout << "Starting test for the dual bound of a constraint that is infinite outside its domain"
                  << std::endl;
        passed = TestDualBoundOfPerspectiveConstraint();
        std::cout << "Finished test for the dual bound of a constraint that is infinite outside its domain."
                  << std::endl;
        break;
    case 18:
        std::cout << "Starting test for callback system - generic callback context" << std::endl;
        passed = TestCallbackGenericContext();
        std::cout << "Finished test for callback system - generic callback context." << std::endl;
        break;
    case 19:
        std::cout << "Starting test for callback system - failing callback" << std::endl;
        passed = TestCallbackFailure();
        std::cout << "Finished test for callback system - failing callback." << std::endl;
        break;
    case 20:
        std::cout << "Starting test for the primal solution pool" << std::endl;
        passed = TestPrimalSolutionPool();
        std::cout << "Finished test for the primal solution pool." << std::endl;
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