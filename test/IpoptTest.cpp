/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "../src/Solver.h"
#include "../src/Environment.h"
#include "../src/Settings.h"
#include "../src/Timing.h"
#include "../src/Utilities.h"

#include "../src/Model/Variables.h"
#include "../src/Model/Terms.h"
#include "../src/Model/Constraints.h"
#include "../src/Model/NonlinearExpressions.h"
#include "../src/Model/Problem.h"

#include "../src/NLPSolver/NLPSolverIpoptRelaxed.h"

using namespace SHOT;

bool IpoptTest1()
{
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

    env->problem = problem;

    // Creating variables

    /*
     * min_x f(x) = -(x2-2)^2
     *  s.t.
     *       0 = x1^2 + x2 - 1
     *       -1 <= x1 <= 1
     */

    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, -1.0, 1.0);
    SHOT::ExpressionVariablePtr expressionVariable_x = std::make_shared<SHOT::ExpressionVariable>(var_x);

    auto var_y = std::make_shared<SHOT::Variable>("y", 1, SHOT::E_VariableType::Real, -10.0, 10.0);
    SHOT::ExpressionVariablePtr expressionVariable_y = std::make_shared<SHOT::ExpressionVariable>(var_y);

    SHOT::Variables variables = { var_x, var_y };
    problem->add(variables);

    SHOT::NonlinearObjectiveFunctionPtr objectiveFunction
        = std::make_shared<SHOT::NonlinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Maximize);

    SHOT::NonlinearExpressionPtr exprConstant1 = std::make_shared<SHOT::ExpressionConstant>(2);

    SHOT::NonlinearExpressionPtr exprMinus = std::make_shared<SHOT::ExpressionSum>(
        expressionVariable_y, std::make_shared<ExpressionNegate>(exprConstant1));
    SHOT::NonlinearExpressionPtr exprSquared1 = std::make_shared<SHOT::ExpressionSquare>(exprMinus);

    objectiveFunction->add(exprSquared1);
    problem->add(objectiveFunction);

    SHOT::NonlinearExpressionPtr exprSquared2 = std::make_shared<SHOT::ExpressionSquare>(expressionVariable_x);
    SHOT::NonlinearExpressionPtr exprPlus = std::make_shared<SHOT::ExpressionSum>(exprSquared2, expressionVariable_y);
    SHOT::NonlinearConstraintPtr nonlinearConstraint
        = std::make_shared<SHOT::NonlinearConstraint>(0, "nlconstr", exprPlus, 1.0, 1.0);
    problem->add(nonlinearConstraint);

    std::cout << '\n';
    std::cout << "Finalizing problem:\n";
    problem->finalize();

    std::cout << '\n';
    std::cout << "Problem created:\n\n";
    std::cout << problem << '\n';

    std::cout << '\n';
    std::cout << "Jacobian sparsity pattern:\n";
    auto jacobianSparsityPattern = problem->getConstraintsJacobianSparsityPattern();

    for(auto& E : *jacobianSparsityPattern)
    {
        for(auto& V : E.second)
            std::cout << "(" << E.first->index << "," << V->index << ")\n";
    }

    std::cout << '\n';
    std::cout << "Hessian of the objective function sparsity pattern:\n";
    auto objectiveSparsityPattern = problem->objectiveFunction->getHessianSparsityPattern();

    for(auto& E : *objectiveSparsityPattern)
    {
        std::cout << "(" << E.first->index << "," << E.second->index << ")\n";
    }

    std::cout << '\n';
    std::cout << "Hessian of the constraints sparsity pattern:\n";
    auto constraintsSparsityPattern = problem->getConstraintsHessianSparsityPattern();

    for(auto& E : *constraintsSparsityPattern)
    {
        std::cout << "(" << E.first->index << "," << E.second->index << ")\n";
    }

    auto NLPSolver = std::make_shared<NLPSolverIpoptRelaxed>(env, problem);

    std::cout << "\nCalculating objective function gradient in point (2.0,3.0):\n";

    SHOT::VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);

    auto gradient = problem->objectiveFunction->calculateGradient(point, true);

    for(auto const& G : gradient)
    {
        std::cout << G.first->name << ":  " << G.second << '\n';
    }

    std::cout << "\nCalculating constraint gradient in point (2.0,3.0):\n";

    gradient = problem->nonlinearConstraints.at(0)->calculateGradient(point, true);

    for(auto const& G : gradient)
    {
        std::cout << G.first->name << ":  " << G.second << '\n';
    }

    NLPSolver->setStartingPoint(std::vector<int>({ 0, 1 }), std::vector<double>({ 5.0, 5.0 }));

    NLPSolver->solveProblem();

    std::cout << '\n';
    std::cout << "The objective value is: " << NLPSolver->getObjectiveValue() << std::endl;

    auto solution = NLPSolver->getSolution();

    std::cout << '\n';
    std::cout << "The solution vector is:\n\n";
    Utilities::displayVector(solution);

    return (passed);
}

bool IpoptTest2()
{
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

    env->problem = problem;

    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.1, 2.0);
    SHOT::ExpressionVariablePtr expressionVariable_x = std::make_shared<SHOT::ExpressionVariable>(var_x);

    auto var_y = std::make_shared<SHOT::Variable>("y", 1, SHOT::E_VariableType::Real, 0.1, 10.0);
    SHOT::ExpressionVariablePtr expressionVariable_y = std::make_shared<SHOT::ExpressionVariable>(var_y);

    SHOT::Variables variables = { var_x, var_y };
    problem->add(variables);

    SHOT::NonlinearObjectiveFunctionPtr objectiveFunction
        = std::make_shared<SHOT::NonlinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Minimize);

    SHOT::NonlinearExpressionPtr exprTimes = std::make_shared<SHOT::ExpressionProduct>(
        std::make_shared<SHOT::ExpressionConstant>(4.0), expressionVariable_y);

    SHOT::NonlinearExpressionPtr exprMinus
        = std::make_shared<SHOT::ExpressionSum>(expressionVariable_x, std::make_shared<ExpressionNegate>(exprTimes));
    SHOT::NonlinearExpressionPtr exprConstant = std::make_shared<SHOT::ExpressionConstant>(2.0);
    SHOT::NonlinearExpressionPtr exprPower = std::make_shared<SHOT::ExpressionPower>(exprMinus, exprConstant);

    SHOT::LinearTerms linearTerms;
    linearTerms.add(std::make_shared<LinearTerm>(1.0, var_x));
    linearTerms.add(std::make_shared<LinearTerm>(1.0, var_y));
    auto linearConstraint = std::make_shared<SHOT::LinearConstraint>(0, "linconstr", linearTerms, 3.0, 3.0);

    linearConstraint->add(linearTerms);
    problem->add(linearConstraint);

    objectiveFunction->add(exprPower);
    problem->add(objectiveFunction);

    std::cout << '\n';
    std::cout << "Finalizing problem:\n";
    problem->finalize();

    std::cout << '\n';
    std::cout << "Problem created:\n\n";
    std::cout << problem << '\n';

    std::cout << '\n';
    std::cout << "Hessian of the Lagrangian sparsity pattern:\n";
    auto lagrangianSparsityPattern = problem->objectiveFunction->getHessianSparsityPattern();

    for(auto& E : *lagrangianSparsityPattern)
    {
        std::cout << "(" << E.first->index << "," << E.second->index << ")\n";
    }

    auto NLPSolver = std::make_shared<NLPSolverIpoptRelaxed>(env, problem);

    std::cout << "\nCalculating objective function gradient in point (2.0,3.0):\n";

    SHOT::VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);

    auto gradient = problem->objectiveFunction->calculateGradient(point, true);

    for(auto const& G : gradient)
    {
        std::cout << G.first->name << ":  " << G.second << '\n';
    }

    NLPSolver->setStartingPoint(std::vector<int>({ 0, 1 }), std::vector<double>({ 5.0, 5.0 }));

    NLPSolver->solveProblem();

    std::cout << '\n';
    std::cout << "The objective value is: " << NLPSolver->getObjectiveValue() << std::endl;

    auto solution = NLPSolver->getSolution();

    std::cout << '\n';
    std::cout << "The solution vector is:\n\n";
    Utilities::displayVector(solution);

    return (passed);
}

int IpoptTest(int argc, char* argv[])
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
        std::cout << "Starting test to solve 1D constrained problem using Ipopt:" << std::endl;
        passed = IpoptTest1();
        std::cout << "Finished test to solve 1D constrained problem using Ipopt." << std::endl;
        break;
    case 2:
        std::cout << "Starting test to solve 2D unconstrained problem using Ipopt:" << std::endl;
        passed = IpoptTest2();
        std::cout << "Finished test to solve 2D unconstrained problem using Ipopt." << std::endl;
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