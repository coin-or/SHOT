/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "../src/Solver.h"
#include "../src/Environment.h"
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

bool CreateAndSolveProblem()
{
    bool passed = true;

    // Initializing the SHOT solver class
    auto solver = std::make_unique<SHOT::Solver>();

    // Contains the environment variable unique to the created solver instance
    auto env = solver->getEnvironment();
    solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(E_LogLevel::Off));
    solver->updateSetting("Debug.Enable", "Output", true);

    // Initializing a SHOT problem class
    auto problem = std::make_shared<SHOT::Problem>(env);
    problem->name = "ex1223b";

    // Creating the variables
    auto x1 = std::make_shared<Variable>("x1", 0, E_VariableType::Real, 0.0, 10.0);
    auto x2 = std::make_shared<Variable>("x2", 1, E_VariableType::Real, 0.0, 10.0);
    auto x3 = std::make_shared<Variable>("x3", 2, E_VariableType::Real, 0.0, 10.0);
    auto b4 = std::make_shared<Variable>("b4", 3, E_VariableType::Binary);
    auto b5 = std::make_shared<Variable>("b5", 4, E_VariableType::Binary);
    auto b6 = std::make_shared<Variable>("b6", 5, E_VariableType::Binary);
    auto b7 = std::make_shared<Variable>("b7", 6, E_VariableType::Binary);

    // All variables are nonlinear, so need to add expression variables as well
    auto nl_x1 = std::make_shared<ExpressionVariable>(x1);
    auto nl_x2 = std::make_shared<ExpressionVariable>(x2);
    auto nl_x3 = std::make_shared<ExpressionVariable>(x3);
    auto nl_b4 = std::make_shared<ExpressionVariable>(b4);
    auto nl_b5 = std::make_shared<ExpressionVariable>(b5);
    auto nl_b6 = std::make_shared<ExpressionVariable>(b6);
    auto nl_b7 = std::make_shared<ExpressionVariable>(b7);

    // Adding the variables to the problem
    problem->add({ x1, x2, x3, b4, b5, b6, b7 });

    // Creating the objective function
    // minimize -(sqr((-1) + b4) + sqr((-2) + b5) + sqr((-1) + b6) - log(1 + b7) + sqr((-1) + x1) + sqr((-2) + x2) +
    // sqr((-3) + x3))

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

    // Creating the constraint e1: x1 + x2 + x3 + b4 + b5 + b6 <= 5;
    auto e1 = std::make_shared<LinearConstraint>(0, "e1", SHOT_DBL_MIN, 5.0);
    e1->add(std::make_shared<LinearTerm>(1.0, x1));
    e1->add(std::make_shared<LinearTerm>(1.0, x2));
    e1->add(std::make_shared<LinearTerm>(1.0, x3));
    e1->add(std::make_shared<LinearTerm>(1.0, b4));
    e1->add(std::make_shared<LinearTerm>(1.0, b5));
    e1->add(std::make_shared<LinearTerm>(1.0, b6));
    problem->add(e1);

    // Creating the constraint e2: sqr(b6) + sqr(x1) + sqr(x2) + sqr(x3) <= 5.5;
    auto e2 = std::make_shared<QuadraticConstraint>(1, "e2", SHOT_DBL_MIN, 5.5);
    e2->add(std::make_shared<QuadraticTerm>(1.0, b6, b6));
    e2->add(std::make_shared<QuadraticTerm>(1.0, x1, x1));
    e2->add(std::make_shared<QuadraticTerm>(1.0, x2, x2));
    e2->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
    problem->add(e2);

    // Creating the constraint e3: x1 +  b4 <= 1.2;
    auto e3 = std::make_shared<LinearConstraint>(2, "e3", SHOT_DBL_MIN, 1.2);
    e3->add(std::make_shared<LinearTerm>(1.0, x1));
    e3->add(std::make_shared<LinearTerm>(1.0, b4));
    problem->add(e3);

    // Creating the constraint e4: x2 +  b5 <= 1.8;
    auto e4 = std::make_shared<LinearConstraint>(3, "e4", SHOT_DBL_MIN, 1.8);
    e4->add(std::make_shared<LinearTerm>(1.0, x2));
    e4->add(std::make_shared<LinearTerm>(1.0, b5));
    problem->add(e4);

    // Creating the constraint e5: x3 +  b6 <= 2.5;
    auto e5 = std::make_shared<LinearConstraint>(4, "e5", SHOT_DBL_MIN, 2.5);
    e5->add(std::make_shared<LinearTerm>(1.0, x3));
    e5->add(std::make_shared<LinearTerm>(1.0, b6));
    problem->add(e5);

    // Creating the constraint e6: x1 +  b7 <= 1.2;
    auto e6 = std::make_shared<LinearConstraint>(5, "e6", SHOT_DBL_MIN, 1.2);
    e6->add(std::make_shared<LinearTerm>(1.0, x1));
    e6->add(std::make_shared<LinearTerm>(1.0, b7));
    problem->add(e6);

    // Creating the constraint e7: sqr(b5) + sqr(x2) <= 1.64;
    auto e7 = std::make_shared<QuadraticConstraint>(6, "e7", SHOT_DBL_MIN, 1.64);
    e7->add(std::make_shared<QuadraticTerm>(1.0, b5, b5));
    e7->add(std::make_shared<QuadraticTerm>(1.0, x2, x2));
    problem->add(e7);

    // Creating the constraint e8: sqr(b6) + sqr(x3) <= 4.25;
    auto e8 = std::make_shared<QuadraticConstraint>(7, "e8", SHOT_DBL_MIN, 4.25);
    e8->add(std::make_shared<QuadraticTerm>(1.0, b6, b6));
    e8->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
    problem->add(e8);

    // Creating the constraint e9: sqr(b5) + sqr(x3) <= 4.64;
    auto e9 = std::make_shared<QuadraticConstraint>(8, "e9", SHOT_DBL_MIN, 4.64);
    e9->add(std::make_shared<QuadraticTerm>(1.0, b5, b5));
    e9->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
    problem->add(e9);

    // Simplify the nonlinear expressions (and extract e.g. the quadratics)
    simplifyNonlinearExpressions(problem, true, true, true);

    // Finalize the problem object (after this no changes should be made)
    problem->updateProperties();
    problem->finalize();
    solver->setProblem(problem);

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
    auto settings = env->settings;

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
    auto solver = std::make_unique<SHOT::Solver>();

    // Contains the environment variable unique to the created solver instance
    auto env = solver->getEnvironment();
    solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(E_LogLevel::Critical));
    solver->updateSetting("Debug.Enable", "Output", true);

    // Initializing a SHOT problem class
    auto problem = std::make_shared<SHOT::Problem>(env);
    problem->name = "ex1223b";

    // Creating the variables
    auto x1 = std::make_shared<Variable>("x1", 0, E_VariableType::Real, 0.0, 10.0);
    auto x2 = std::make_shared<Variable>("x2", 1, E_VariableType::Real, 0.0, 10.0);
    auto x3 = std::make_shared<Variable>("x3", 2, E_VariableType::Real, 0.0, 10.0);
    auto b4 = std::make_shared<Variable>("b4", 3, E_VariableType::Binary);
    auto b5 = std::make_shared<Variable>("b5", 4, E_VariableType::Binary);
    auto b6 = std::make_shared<Variable>("b6", 5, E_VariableType::Binary);
    auto b7 = std::make_shared<Variable>("b7", 6, E_VariableType::Binary);

    // All variables are nonlinear, so need to add expression variables as well
    auto nl_x1 = std::make_shared<ExpressionVariable>(x1);
    auto nl_x2 = std::make_shared<ExpressionVariable>(x2);
    auto nl_x3 = std::make_shared<ExpressionVariable>(x3);
    auto nl_b4 = std::make_shared<ExpressionVariable>(b4);
    auto nl_b5 = std::make_shared<ExpressionVariable>(b5);
    auto nl_b6 = std::make_shared<ExpressionVariable>(b6);
    auto nl_b7 = std::make_shared<ExpressionVariable>(b7);

    // Adding the variables to the problem
    problem->add({ x1, x2, x3, b4, b5, b6, b7 });

    // Creating the objective function
    // minimize -(sqr((-1) + b4) + sqr((-2) + b5) + sqr((-1) + b6) - log(1 + b7) + sqr((-1) + x1) + sqr((-2) + x2) +
    // sqr((-3) + x3))

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

    // Creating the constraint e1: x1 + x2 + x3 + b4 + b5 + b6 <= 5;
    auto e1 = std::make_shared<LinearConstraint>(0, "e1", SHOT_DBL_MIN, 5.0);
    e1->add(std::make_shared<LinearTerm>(1.0, x1));
    e1->add(std::make_shared<LinearTerm>(1.0, x2));
    e1->add(std::make_shared<LinearTerm>(1.0, x3));
    e1->add(std::make_shared<LinearTerm>(1.0, b4));
    e1->add(std::make_shared<LinearTerm>(1.0, b5));
    e1->add(std::make_shared<LinearTerm>(1.0, b6));
    problem->add(e1);

    // Creating the constraint e2: sqr(b6) + sqr(x1) + sqr(x2) + sqr(x3) <= 5.5;
    auto e2 = std::make_shared<QuadraticConstraint>(1, "e2", SHOT_DBL_MIN, 5.5);
    e2->add(std::make_shared<QuadraticTerm>(1.0, b6, b6));
    e2->add(std::make_shared<QuadraticTerm>(1.0, x1, x1));
    e2->add(std::make_shared<QuadraticTerm>(1.0, x2, x2));
    e2->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
    problem->add(e2);

    // Creating the constraint e3: x1 +  b4 <= 1.2;
    auto e3 = std::make_shared<LinearConstraint>(2, "e3", SHOT_DBL_MIN, 1.2);
    e3->add(std::make_shared<LinearTerm>(1.0, x1));
    e3->add(std::make_shared<LinearTerm>(1.0, b4));
    problem->add(e3);

    // Creating the constraint e4: x2 +  b5 <= 1.8;
    auto e4 = std::make_shared<LinearConstraint>(3, "e4", SHOT_DBL_MIN, 1.8);
    e4->add(std::make_shared<LinearTerm>(1.0, x2));
    e4->add(std::make_shared<LinearTerm>(1.0, b5));
    problem->add(e4);

    // Creating the constraint e5: x3 +  b6 <= 2.5;
    auto e5 = std::make_shared<LinearConstraint>(4, "e5", SHOT_DBL_MIN, 2.5);
    e5->add(std::make_shared<LinearTerm>(1.0, x3));
    e5->add(std::make_shared<LinearTerm>(1.0, b6));
    problem->add(e5);

    // Creating the constraint e6: x1 +  b7 <= 1.2;
    auto e6 = std::make_shared<LinearConstraint>(5, "e6", SHOT_DBL_MIN, 1.2);
    e6->add(std::make_shared<LinearTerm>(1.0, x1));
    e6->add(std::make_shared<LinearTerm>(1.0, b7));
    problem->add(e6);

    // Creating the constraint e7: sqr(b5) + sqr(x2) <= 1.64;
    auto e7 = std::make_shared<QuadraticConstraint>(6, "e7", SHOT_DBL_MIN, 1.64);
    e7->add(std::make_shared<QuadraticTerm>(1.0, b5, b5));
    e7->add(std::make_shared<QuadraticTerm>(1.0, x2, x2));
    problem->add(e7);

    // Creating the constraint e8: sqr(b6) + sqr(x3) <= 4.25;
    auto e8 = std::make_shared<QuadraticConstraint>(7, "e8", SHOT_DBL_MIN, 4.25);
    e8->add(std::make_shared<QuadraticTerm>(1.0, b6, b6));
    e8->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
    problem->add(e8);

    // Creating the constraint e9: sqr(b5) + sqr(x3) <= 4.64;
    auto e9 = std::make_shared<QuadraticConstraint>(8, "e9", SHOT_DBL_MIN, 4.64);
    e9->add(std::make_shared<QuadraticTerm>(1.0, b5, b5));
    e9->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
    problem->add(e9);

    // Simplify the nonlinear expressions (and extract e.g. the quadratics)
    simplifyNonlinearExpressions(problem, true, true, true);

    // Finalize the problem object (after this no changes should be made)
    problem->updateProperties();
    problem->finalize();
    solver->setProblem(problem);

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
    solver->updateSetting("Reformulation.Constraint.PartitionQuadraticTerms", "Model", 2);
    solver->updateSetting("Relaxation.Use", "Dual", false);
    solver->updateSetting("CutStrategy", "Dual", 1);

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
    default:
        passed = false;
        std::cout << "Test #" << choice << " does not exist!\n";
    }

    if(passed)
        return 0;
    else
        return -1;
}