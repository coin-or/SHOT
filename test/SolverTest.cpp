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
#include "../src/CallbackData.h"
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

    solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(E_LogLevel::Error));

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

    solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(E_LogLevel::Error));

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
    solver->updateSetting("Debug.Enable", "Output", true);
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
    solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(E_LogLevel::Off));

    // Resetting our problem objects
    solver->setProblem(problem, reformulatedProblem);

    // Registers a callback that is activated every time a new primal solution is found
    solver->registerCallback(E_EventType::NewPrimalSolution, [&env, &passed](std::any args) {
        try
        {
            PrimalSolution solution = std::any_cast<PrimalSolution>(args);

            std::cout << "We have a new primal solution: " << solution.objValue
                      << " found at iteration: " << solution.iterFound << ". In total we now have "
                      << env->solutionStatistics.numberOfFoundPrimalSolutions << " solutions.\n";

            std::cout << "Primal solution point:\n";
            Utilities::displayVector(solution.point);

            if(solution.objValue == env->results->getPrimalBound())
            {
                std::cout << "Ok, new primal solution has been saved successfully. " << solution.objValue << ".\n";
                passed = true;
            }
            else
            {
                std::cout << "Error: new primal solution not saved successfully!\n";
                passed = false;
            }
        }
        catch(const std::bad_any_cast& e)
        {
            // If the callback argument is not a PrimalSolution
            passed = false;
            std::cout << "Failed to cast callback argument: " << e.what() << std::endl;
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
    solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(E_LogLevel::Off));

    // Resetting our problem objects
    solver->setProblem(problem, reformulatedProblem);

    // Registers a callback that is activated every time a new primal solution is found
    solver->registerCallback(E_EventType::NewPrimalSolution, [&env, &passed](std::any args) {
        try
        {
            PrimalSolution solution = std::any_cast<PrimalSolution>(args);

            std::cout << "We have a new primal solution: " << solution.objValue
                      << " found at iteration: " << solution.iterFound << ". In total we now have "
                      << env->solutionStatistics.numberOfFoundPrimalSolutions << " solutions.\n";

            std::cout << "Primal solution point:\n";
            Utilities::displayVector(solution.point);

            if(solution.objValue == env->results->getPrimalBound())
            {
                std::cout << "Ok, new primal solution has been saved successfully. " << solution.objValue << ".\n";
                passed = true;
            }
            else
            {
                std::cout << "Error: new primal solution not saved successfully!\n";
                passed = false;
            }
        }
        catch(const std::bad_any_cast& e)
        {
            // If the callback argument is not a PrimalSolution
            passed = false;
            std::cout << "Failed to cast callback argument: " << e.what() << std::endl;
        }
    });

    // Registers a callback that terminates if we have found at least one primal solution
    solver->registerCallback(E_EventType::UserTerminationCheck, [](std::any args) -> bool {
        auto data = std::any_cast<TerminationCallbackData>(args);
        std::cout << "Termination callback with structured data - iteration: " << data.iterationNumber << "\n";

        // If we have found one primal solution, we terminate the solver
        if(data.iterationNumber > 0 && data.solutionStatistics.numberOfFoundPrimalSolutions > 0)
        {
            std::cout << "Termination callback activated. We have found at least one solution.\n";
            return true;
        }
        else
        {
            std::cout << "Termination callback activated. We have not found a primal solution yet.\n";
            return false;
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
    solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(E_LogLevel::Critical));
    solver->updateSetting("Debug.Enable", "Output", true);

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

    // Register user termination check - returns bool
    solver->registerCallback(
        E_EventType::UserTerminationCheck, [&iterationCount, &terminationRequested](std::any args) -> bool {
            iterationCount++;

            auto data = std::any_cast<TerminationCallbackData>(args);
            std::cout << "User termination check called with structured data (call #" << iterationCount
                      << ", solver iteration " << data.iterationNumber << ")" << std::endl;

            // Terminate after 3 checks
            if(iterationCount >= 3)
            {
                std::cout << "User termination check requesting termination" << std::endl;
                terminationRequested = true;
                return true; // Request termination
            }

            return false; // Continue
        });

    // Set high iteration limit so termination comes from our callback
    env->settings->updateSetting("IterationLimit", "Termination", 100);

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
    solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(E_LogLevel::Off));
    solver->updateSetting("Convexity.AssumeConvex", "Model", true);
    solver->updateSetting("Debug.Enable", "Output", true);
    solver->updateSetting("Reformulation.Constraint.PartitionQuadraticTerms", "Model",
        static_cast<int>(ES_PartitionNonlinearSums::Never));
    solver->updateSetting("Relaxation.Use", "Dual", false);
    solver->updateSetting("CutStrategy", "Dual", static_cast<int>(ES_HyperplaneCutStrategy::OnlyExternal));
    solver->updateSetting("TreeStrategy", "Dual", static_cast<int>(ES_TreeStrategy::MultiTree));

    // Initializing a SHOT problem class
    auto problem = std::make_shared<SHOT::Problem>(env);
    problem->name = "ex1223b";

    // Creating the variables
    auto x1 = std::make_shared<Variable>("x1", 0, E_VariableType::Integer, 0.0, 3.0);
    auto x2 = std::make_shared<Variable>("x2", 1, E_VariableType::Integer, 1.0, 3.0);

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
    auto e1 = std::make_shared<NonlinearConstraint>(0, "e1", SHOT_DBL_MIN, 10.0);
    e1->add(std::make_shared<QuadraticTerm>(1.0, x1, x1));
    e1->add(std::make_shared<LinearTerm>(1.0, x2));

    e1->add(std::make_shared<ExpressionProduct>(
        std::make_shared<ExpressionConstant>(0.1), std::make_shared<ExpressionExp>(nl_x2)));
    problem->add(e1);

    // Creating the constraint e2: e^x1 / x2  <= 3;
    auto e2 = std::make_shared<NonlinearConstraint>(1, "e2", SHOT_DBL_MIN, 3.0);

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
    solver->registerCallback(E_EventType::ExternalHyperplaneSelection, [&env, &constraints](std::any args) -> std::any {
        auto data = std::any_cast<ExternalHyperplaneSelectionCallbackData>(args);

        std::cout << "External hyperplane callback called at iteration " << data.iterationNumber << std::endl;
        std::cout << "Current dual bound: " << data.currentDualBound << std::endl;
        std::cout << "Current primal bound: " << data.currentPrimalBound << std::endl;
        std::cout << "Number of solution points: " << data.solutionPoints.size() << std::endl;

        std::vector<ExternalHyperplane> hyperplanes;

        // Example: Add a simple cutting plane if we have solution points
        if(!data.solutionPoints.empty() && data.iterationNumber > 0)
        {
            for(const auto& solPoint : data.solutionPoints)
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
                hyperplane.description = fmt::format("hyp_{}", data.iterationNumber);
                hyperplane.source = E_HyperplaneSource::External;

                hyperplanes.push_back(hyperplane);

                std::cout << "Generated hyperplane variable coefficients: \n";
                Utilities::displayVector(hyperplane.variableCoefficients);
                std::cout << "RHS value: " << hyperplane.rhsValue << std::endl;

                break; // Only add one hyperplane per iterations
            }
        }

        std::cout << "Returning " << hyperplanes.size() << " external hyperplanes" << std::endl;
        return std::make_any<std::vector<ExternalHyperplane>>(hyperplanes);
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
    solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(E_LogLevel::Info));
    if(forceNonlinear)
        solver->updateSetting("Reformulation.Quadratics.Strategy", "Model",
            static_cast<int>(ES_QuadraticProblemStrategy::Nonlinear));

    auto problem = std::make_shared<SHOT::Problem>(env);
    problem->name = "ex1223b";

    auto x1 = std::make_shared<Variable>("x1", 0, E_VariableType::Real, 0.0, 10.0);
    auto x2 = std::make_shared<Variable>("x2", 1, E_VariableType::Real, 0.0, 10.0);
    auto x3 = std::make_shared<Variable>("x3", 2, E_VariableType::Real, 0.0, 10.0);
    auto b4 = std::make_shared<Variable>("b4", 3, E_VariableType::Binary);
    auto b5 = std::make_shared<Variable>("b5", 4, E_VariableType::Binary);
    auto b6 = std::make_shared<Variable>("b6", 5, E_VariableType::Binary);
    auto b7 = std::make_shared<Variable>("b7", 6, E_VariableType::Binary);

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

    auto e1 = std::make_shared<LinearConstraint>(0, "e1", SHOT_DBL_MIN, 5.0);
    e1->add(std::make_shared<LinearTerm>(1.0, x1)); e1->add(std::make_shared<LinearTerm>(1.0, x2));
    e1->add(std::make_shared<LinearTerm>(1.0, x3)); e1->add(std::make_shared<LinearTerm>(1.0, b4));
    e1->add(std::make_shared<LinearTerm>(1.0, b5)); e1->add(std::make_shared<LinearTerm>(1.0, b6));
    problem->add(e1);

    auto e2 = std::make_shared<QuadraticConstraint>(1, "e2", SHOT_DBL_MIN, 5.5);
    e2->add(std::make_shared<QuadraticTerm>(1.0, b6, b6)); e2->add(std::make_shared<QuadraticTerm>(1.0, x1, x1));
    e2->add(std::make_shared<QuadraticTerm>(1.0, x2, x2)); e2->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
    problem->add(e2);

    auto e3 = std::make_shared<LinearConstraint>(2, "e3", SHOT_DBL_MIN, 1.2);
    e3->add(std::make_shared<LinearTerm>(1.0, x1)); e3->add(std::make_shared<LinearTerm>(1.0, b4));
    problem->add(e3);

    auto e4 = std::make_shared<LinearConstraint>(3, "e4", SHOT_DBL_MIN, 1.8);
    e4->add(std::make_shared<LinearTerm>(1.0, x2)); e4->add(std::make_shared<LinearTerm>(1.0, b5));
    problem->add(e4);

    auto e5 = std::make_shared<LinearConstraint>(4, "e5", SHOT_DBL_MIN, 2.5);
    e5->add(std::make_shared<LinearTerm>(1.0, x3)); e5->add(std::make_shared<LinearTerm>(1.0, b6));
    problem->add(e5);

    auto e6 = std::make_shared<LinearConstraint>(5, "e6", SHOT_DBL_MIN, 1.2);
    e6->add(std::make_shared<LinearTerm>(1.0, x1)); e6->add(std::make_shared<LinearTerm>(1.0, b7));
    problem->add(e6);

    auto e7 = std::make_shared<QuadraticConstraint>(6, "e7", SHOT_DBL_MIN, 1.64);
    e7->add(std::make_shared<QuadraticTerm>(1.0, b5, b5)); e7->add(std::make_shared<QuadraticTerm>(1.0, x2, x2));
    problem->add(e7);

    auto e8 = std::make_shared<QuadraticConstraint>(7, "e8", SHOT_DBL_MIN, 4.25);
    e8->add(std::make_shared<QuadraticTerm>(1.0, b6, b6)); e8->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
    problem->add(e8);

    auto e9 = std::make_shared<QuadraticConstraint>(8, "e9", SHOT_DBL_MIN, 4.64);
    e9->add(std::make_shared<QuadraticTerm>(1.0, b5, b5)); e9->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
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
    std::cout << "\nSub-test 1: PrimalSolutionCandidateSelection callback fires\n";
    {
        auto [solver, env] = MakeEx1223bSolver();
        int candidateCount = 0;

        solver->registerCallback(
            E_EventType::PrimalSolutionCandidateSelection, [&candidateCount](std::any args) -> bool {
                auto data = std::any_cast<PrimalSolutionCallbackData>(args);
                candidateCount++;
                std::cout << "  Candidate #" << candidateCount << "  obj=" << data.objectiveValue
                          << "  iter=" << data.iterationNumber << "\n";
                return true; // accept
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
    std::cout << "\nSub-test 2: returning false blocks all primal solutions\n";
    {
        auto [solver, env] = MakeEx1223bSolver();
        // Cap iterations so the test terminates quickly
        solver->updateSetting("IterationLimit", "Termination", 10);
        int rejectedCount = 0;

        solver->registerCallback(
            E_EventType::PrimalSolutionCandidateSelection, [&rejectedCount](std::any args) -> bool {
                auto data = std::any_cast<PrimalSolutionCallbackData>(args);
                rejectedCount++;
                std::cout << "  Rejecting candidate obj=" << data.objectiveValue << "\n";
                return false; // reject everything
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
            solver->registerCallback(
                E_EventType::PrimalSolutionCandidateSelection, [](std::any) -> bool { return true; });
            solver->registerCallback(E_EventType::NewPrimalSolution,
                [&acceptedA](std::any) { acceptedA++; });
            solver->solveProblem();
        }

        // Run B: reject candidates with obj > 5
        int acceptedB = 0;
        {
            auto [solver, env] = MakeEx1223bSolver();
            solver->registerCallback(
                E_EventType::PrimalSolutionCandidateSelection, [](std::any args) -> bool {
                    auto data = std::any_cast<PrimalSolutionCallbackData>(args);
                    return data.objectiveValue <= 5.0;
                });
            solver->registerCallback(E_EventType::NewPrimalSolution,
                [&acceptedB](std::any) { acceptedB++; });
            solver->solveProblem();
        }

        std::cout << "  Accept-all incumbents: " << acceptedA
                  << "  Selective incumbents: " << acceptedB << "\n";

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

        solver->registerCallback(
            E_EventType::ExternalESHRootsearchPointsSelection, [&](std::any args) -> std::any {
                auto data = std::any_cast<ESHInteriorPointCallbackData>(args);
                callbackFired = true;
                pointsReceived = data.currentInteriorPoints.size();
                std::cout << "  ESH interior point callback fired with " << pointsReceived << " current point(s)\n";
                // Return empty to keep current points unchanged
                return std::any(std::vector<VectorDouble>{});
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

        solver->updateSetting("ESH.InteriorPoint.Strategy", "Dual",
            static_cast<int>(ES_ESHInteriorPointStrategy::OnlyExternal));

        bool callbackFired = false;

        solver->registerCallback(
            E_EventType::ExternalESHRootsearchPointsSelection, [&](std::any args) -> std::any {
                auto data = std::any_cast<ESHInteriorPointCallbackData>(args);
                callbackFired = true;
                std::cout << "  ESH interior point callback fired (OnlyExternal strategy)\n";
                std::cout << "  currentInteriorPoints size from callback: "
                          << data.currentInteriorPoints.size() << " (should be 0)\n";
                // Return the captured point from Phase 1
                return std::any(std::vector<VectorDouble>{ capturedInteriorPoint });
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
                std::cout << "Phase 2b FAILED: objective " << phase2ObjValue
                          << " differs from Phase 1 (" << phase1ObjValue << ") by more than tolerance\n";
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
static std::pair<std::unique_ptr<SHOT::Solver>, std::shared_ptr<SHOT::Environment>>
MakeEx1223bInteriorPointSolver()
{
    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();
    solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(E_LogLevel::Info));
    // Treat quadratic constraints as nonlinear so the interior point is valid for ESH
    solver->updateSetting("Reformulation.Quadratics.Strategy", "Model",
        static_cast<int>(ES_QuadraticProblemStrategy::Nonlinear));

    auto problem = std::make_shared<SHOT::Problem>(env);
    problem->name = "ex1223b_interior";

    // Original variables — all continuous (binary variables relaxed to [0,1])
    auto x1 = std::make_shared<Variable>("x1", 0, E_VariableType::Real, 0.0, 10.0);
    auto x2 = std::make_shared<Variable>("x2", 1, E_VariableType::Real, 0.0, 10.0);
    auto x3 = std::make_shared<Variable>("x3", 2, E_VariableType::Real, 0.0, 10.0);
    auto b4 = std::make_shared<Variable>("b4", 3, E_VariableType::Real, 0.0, 1.0);
    auto b5 = std::make_shared<Variable>("b5", 4, E_VariableType::Real, 0.0, 1.0);
    auto b6 = std::make_shared<Variable>("b6", 5, E_VariableType::Real, 0.0, 1.0);
    auto b7 = std::make_shared<Variable>("b7", 6, E_VariableType::Real, 0.0, 1.0);

    // Auxiliary variable mu: the objective is to minimize mu
    auto mu = std::make_shared<Variable>("mu", 7, E_VariableType::Real, -100.0, 100.0);

    problem->add({ x1, x2, x3, b4, b5, b6, b7, mu });

    // Objective: minimize mu
    auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    objective->add(std::make_shared<LinearTerm>(1.0, mu));
    problem->add(objective);

    // Linear constraints: unchanged from ex1223b
    auto e1 = std::make_shared<LinearConstraint>(0, "e1", SHOT_DBL_MIN, 5.0);
    e1->add(std::make_shared<LinearTerm>(1.0, x1)); e1->add(std::make_shared<LinearTerm>(1.0, x2));
    e1->add(std::make_shared<LinearTerm>(1.0, x3)); e1->add(std::make_shared<LinearTerm>(1.0, b4));
    e1->add(std::make_shared<LinearTerm>(1.0, b5)); e1->add(std::make_shared<LinearTerm>(1.0, b6));
    problem->add(e1);

    // Quadratic constraints with -mu added: f(x) - mu <= rhs
    auto e2 = std::make_shared<QuadraticConstraint>(1, "e2", SHOT_DBL_MIN, 5.5);
    e2->add(std::make_shared<QuadraticTerm>(1.0, b6, b6)); e2->add(std::make_shared<QuadraticTerm>(1.0, x1, x1));
    e2->add(std::make_shared<QuadraticTerm>(1.0, x2, x2)); e2->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
    e2->add(std::make_shared<LinearTerm>(-1.0, mu));
    problem->add(e2);

    auto e3 = std::make_shared<LinearConstraint>(2, "e3", SHOT_DBL_MIN, 1.2);
    e3->add(std::make_shared<LinearTerm>(1.0, x1)); e3->add(std::make_shared<LinearTerm>(1.0, b4));
    problem->add(e3);

    auto e4 = std::make_shared<LinearConstraint>(3, "e4", SHOT_DBL_MIN, 1.8);
    e4->add(std::make_shared<LinearTerm>(1.0, x2)); e4->add(std::make_shared<LinearTerm>(1.0, b5));
    problem->add(e4);

    auto e5 = std::make_shared<LinearConstraint>(4, "e5", SHOT_DBL_MIN, 2.5);
    e5->add(std::make_shared<LinearTerm>(1.0, x3)); e5->add(std::make_shared<LinearTerm>(1.0, b6));
    problem->add(e5);

    auto e6 = std::make_shared<LinearConstraint>(5, "e6", SHOT_DBL_MIN, 1.2);
    e6->add(std::make_shared<LinearTerm>(1.0, x1)); e6->add(std::make_shared<LinearTerm>(1.0, b7));
    problem->add(e6);

    auto e7 = std::make_shared<QuadraticConstraint>(6, "e7", SHOT_DBL_MIN, 1.64);
    e7->add(std::make_shared<QuadraticTerm>(1.0, b5, b5)); e7->add(std::make_shared<QuadraticTerm>(1.0, x2, x2));
    e7->add(std::make_shared<LinearTerm>(-1.0, mu));
    problem->add(e7);

    auto e8 = std::make_shared<QuadraticConstraint>(7, "e8", SHOT_DBL_MIN, 4.25);
    e8->add(std::make_shared<QuadraticTerm>(1.0, b6, b6)); e8->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
    e8->add(std::make_shared<LinearTerm>(-1.0, mu));
    problem->add(e8);

    auto e9 = std::make_shared<QuadraticConstraint>(8, "e9", SHOT_DBL_MIN, 4.64);
    e9->add(std::make_shared<QuadraticTerm>(1.0, b5, b5)); e9->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
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
        std::cout << "  max linear constraint error:    constraint "
                  << sol.maxDevatingConstraintLinear.index << "  error = "
                  << sol.maxDevatingConstraintLinear.value << "\n";
    if(sol.maxDevatingConstraintQuadratic.index >= 0)
        std::cout << "  max quadratic constraint error: constraint "
                  << sol.maxDevatingConstraintQuadratic.index << "  error = "
                  << sol.maxDevatingConstraintQuadratic.value << "\n";
    if(sol.maxDevatingConstraintNonlinear.index >= 0)
        std::cout << "  max nonlinear constraint error: constraint "
                  << sol.maxDevatingConstraintNonlinear.index << "  error = "
                  << sol.maxDevatingConstraintNonlinear.value << "\n";
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
            std::cout << "  mu = " << sol.objValue
                      << " < 0: point is strictly interior to quadratic constraints\n";

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

        solver->updateSetting("ESH.InteriorPoint.Strategy", "Dual",
            static_cast<int>(ES_ESHInteriorPointStrategy::OnlyExternal));

        bool callbackFired = false;

        solver->registerCallback(
            E_EventType::ExternalESHRootsearchPointsSelection, [&](std::any args) -> std::any {
                auto data = std::any_cast<ESHInteriorPointCallbackData>(args);
                callbackFired = true;
                std::cout << "  ESH interior point callback fired (OnlyExternal strategy)\n";
                std::cout << "  Injecting Phase 1 interior point\n";
                return std::any(std::vector<VectorDouble>{ interiorPoint });
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
        std::cout << "Starting test for callback system - ESH external interior point from auxiliary problem" << std::endl;
        passed = TestCallbackESHExternalInteriorPointFromAuxProblem();
        std::cout << "Finished test for callback system - ESH external interior point from auxiliary problem." << std::endl;
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