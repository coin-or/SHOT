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
#include "../src/Results.h"

#include "../src/Model/Variables.h"
#include "../src/Model/Terms.h"
#include "../src/Model/Constraints.h"
#include "../src/Model/NonlinearExpressions.h"
#include "../src/Model/Problem.h"
#include "../src/Model/Simplifications.h"

#include "../src/Tasks/TaskReformulateProblem.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <sstream>

using namespace SHOT;

bool ModelTestVariables();
bool ModelTestTerms();
bool ModelTestNonlinearExpressions();
bool ModelTestObjective();
bool ModelTestConstraints();
bool ModelTestCreateProblem();
bool ModelTestCreateProblem2();
bool ModelTestCreateProblem3();
bool ModelTestSquareRootReformulation();
bool ModelTestConvexity();
bool ModelTestCopy();
bool ModelTestEx1223b();
bool ModelTestGradientsAndHessians();
bool ModelTestFinalizeCalledTwice();
bool ModelTestFinalizeNoObjective();
bool ModelTestFinalizeNoVariables();
bool ModelTestMeanvarxscWithSolver(ES_MIPSolver mipSolver);
bool ModelTestSemiContinuous();
bool ModelTestSOS1WithSolver(ES_MIPSolver mipSolver);
bool ModelTestSOS1();
bool ModelTestSOS2WithSolver(ES_MIPSolver mipSolver);
bool ModelTestSOS2();
bool ModelTestObjectiveEpigraphStrategy();
bool ModelTestAntiEpigraphReformulation();
bool ModelTestObjectivePartitioningStrategy();
bool ModelTestSignomialElementBounds();
bool ModelTestTermAndExpressionBounds();
bool ModelTestMixedTermBoundTightening();

bool TestReadProblem(const std::string& problemFile);
bool TestRootsearch(const std::string& problemFile);
bool TestGradient(const std::string& problemFile);
bool TestReformulateProblem(const std::string& problemFile);

int ModelTest(int argc, char* argv[])
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
        passed = ModelTestVariables();
        break;
    case 2:
        passed = ModelTestTerms();
        break;
    case 3:
        passed = ModelTestNonlinearExpressions();
        break;
    case 4:
        passed = ModelTestObjective();
        break;
    case 5:
        passed = ModelTestConstraints();
        break;
    case 6:
        passed = ModelTestCreateProblem();
        break;
    case 7:
        passed = ModelTestCreateProblem2();
        break;
    case 8:
        passed = ModelTestCreateProblem3();
        break;
    case 9:
        passed = ModelTestConvexity();
        break;
    case 10:
        passed = ModelTestCopy();
        break;
    case 11:
        passed = ModelTestEx1223b();
        break;
    case 12:
        passed = ModelTestGradientsAndHessians();
        break;
    case 13:
        passed = ModelTestSquareRootReformulation();
        break;
    case 14:
        passed = ModelTestFinalizeCalledTwice();
        break;
    case 15:
        passed = ModelTestFinalizeNoObjective();
        break;
    case 16:
        passed = ModelTestFinalizeNoVariables();
        break;
    case 17:
        passed = ModelTestSemiContinuous();
        break;
    case 18:
        passed = ModelTestSOS1();
        break;
    case 19:
        passed = ModelTestSOS2();
        break;
    case 20:
        passed = ModelTestObjectiveEpigraphStrategy();
        break;
    case 21:
        passed = ModelTestAntiEpigraphReformulation();
        break;
    case 22:
        passed = ModelTestObjectivePartitioningStrategy();
        break;
    case 23:
        passed = ModelTestSignomialElementBounds();
        break;
    case 24:
        passed = ModelTestTermAndExpressionBounds();
        break;
    case 25:
        passed = ModelTestMixedTermBoundTightening();
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

bool ModelTestVariables()
{
    bool passed = true;

    std::cout << "Creating variable:\n";
    SHOT::VariablePtr var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 100.0);
    std::cout << "Variable " << var_x << " created.\n";

    SHOT::VectorDouble point;
    point.push_back(2.0);

    double value = var_x->calculate(point);
    double realValue = point.at(0);

    std::cout << "Calculating variable value: " << value << " (should be equal to " << realValue << ").\n";

    if(value != realValue)
        passed = false;

    return passed;
}

bool ModelTestTerms()
{
    bool passed = true;

    SHOT::VariablePtr var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 100.0);
    SHOT::VariablePtr var_y = std::make_shared<SHOT::Variable>("y", 1, SHOT::E_VariableType::Integer, 0.0, 1.0);

    SHOT::VectorDouble point;
    point.push_back(3.0);

    std::cout << "Creating linear term: \n";
    SHOT::LinearTermPtr linearTerm = std::make_shared<SHOT::LinearTerm>(-1, var_x);
    std::cout << "Linear term created: " << linearTerm << "\n";

    double value = linearTerm->calculate(point);
    double realValue = linearTerm->coefficient * point.at(0);

    std::cout << "Calculating term value: " << value << " (should be equal to " << realValue << ").\n";

    if(value != realValue)
        passed = false;

    std::cout << "Creating quadratic term: \n";
    SHOT::QuadraticTermPtr quadraticTerm1 = std::make_shared<SHOT::QuadraticTerm>(1, var_x, var_y);
    std::cout << "Quadratic term created: " << quadraticTerm1 << "\n";

    SHOT::VectorDouble point2;
    point2.push_back(2.0);
    point2.push_back(3.0);

    value = quadraticTerm1->calculate(point2);
    realValue = quadraticTerm1->coefficient * point2.at(0) * point2.at(1);

    std::cout << "Calculating term value: " << value << " (should be equal to " << realValue << ").\n";

    if(value != realValue)
        passed = false;

    std::cout << "Creating quadratic term: \n";
    SHOT::QuadraticTermPtr quadraticTerm2 = std::make_shared<SHOT::QuadraticTerm>(1, var_x, var_x);
    std::cout << "Quadratic term created: " << quadraticTerm2 << "\n";

    value = quadraticTerm2->calculate(point2);
    realValue = quadraticTerm2->coefficient * point2.at(0) * point2.at(0);

    std::cout << "Calculating term value: " << value << " (should be equal to " << realValue << ").\n";

    if(value != realValue)
        passed = false;

    return passed;
}

bool ModelTestNonlinearExpressions()
{
    bool passed = true;

    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 100.0);
    SHOT::ExpressionVariablePtr expressionVariable_x = std::make_shared<SHOT::ExpressionVariable>(var_x);

    auto var_y = std::make_shared<SHOT::Variable>("y", 1, SHOT::E_VariableType::Integer, 0.0, 1.0);
    SHOT::ExpressionVariablePtr expressionVariable_y = std::make_shared<SHOT::ExpressionVariable>(var_y);

    SHOT::VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);

    std::cout << "Creating negate expression\n";
    SHOT::NonlinearExpressionPtr exprNegate = std::make_shared<SHOT::ExpressionNegate>(expressionVariable_x);
    std::cout << "Negate expression " << exprNegate << " created\n";

    auto value = exprNegate->calculate(point);
    double realValue = -var_x->calculate(point);

    std::cout << "Calculating expression value: " << value << " (should be equal to " << realValue << ").\n";

    if(value != realValue)
        passed = false;

    std::cout << "Creating plus expression\n";

    SHOT::NonlinearExpressionPtr exprPlus
        = std::make_shared<SHOT::ExpressionSum>(expressionVariable_x, expressionVariable_y);
    std::cout << "Plus expression " << exprPlus << " created\n";

    value = exprPlus->calculate(point);
    realValue = var_x->calculate(point) + var_y->calculate(point);

    std::cout << "Calculating expression value: " << value << " (should be equal to " << realValue << ").\n";

    if(value != realValue)
        passed = false;

    std::cout << "Creating power expression\n";
    SHOT::NonlinearExpressionPtr exprPower = std::make_shared<SHOT::ExpressionPower>(exprPlus, exprNegate);
    std::cout << "Power expression " << exprPower << " created \n";

    value = exprPower->calculate(point);
    realValue = pow(var_x->calculate(point) + var_y->calculate(point), -var_x->calculate(point));

    std::cout << "Calculating expression value: " << value << " (should be equal to " << realValue << ").\n";

    if(value != realValue)
        passed = false;

    return passed;
}

bool ModelTestObjective()
{
    bool passed = true;

    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 100.0);
    SHOT::ExpressionVariablePtr expressionVariable_x = std::make_shared<SHOT::ExpressionVariable>(var_x);

    auto var_y = std::make_shared<SHOT::Variable>("y", 1, SHOT::E_VariableType::Integer, 0.0, 1.0);
    SHOT::ExpressionVariablePtr expressionVariable_y = std::make_shared<SHOT::ExpressionVariable>(var_y);

    std::cout << "Creating linear terms\n";
    SHOT::LinearTermPtr linearTerm1 = std::make_shared<SHOT::LinearTerm>(-1, var_x);
    SHOT::LinearTermPtr linearTerm2 = std::make_shared<SHOT::LinearTerm>(1.2, var_y);
    SHOT::LinearTerms linearTerms;
    linearTerms.add(linearTerm1);
    linearTerms.add(linearTerm2);
    std::cout << "Linear terms " << linearTerms << " created\n";

    std::cout << "Creating quadratic terms\n";
    SHOT::QuadraticTermPtr quadraticTerm1 = std::make_shared<SHOT::QuadraticTerm>(1, var_x, var_y);
    SHOT::QuadraticTermPtr quadraticTerm2 = std::make_shared<SHOT::QuadraticTerm>(2, var_x, var_x);
    SHOT::QuadraticTerms quadraticTerms;
    quadraticTerms.add(quadraticTerm1);
    quadraticTerms.add(quadraticTerm2);
    std::cout << "Quadratic terms " << quadraticTerms << " created\n";

    std::cout << "Creating nonlinear expression:\n";
    SHOT::NonlinearExpressions expressions;
    expressions.add(expressionVariable_x);
    expressions.add(expressionVariable_x);
    expressions.add(expressionVariable_y);
    SHOT::NonlinearExpressionPtr exprProduct = std::make_shared<SHOT::ExpressionProduct>(expressions);
    std::cout << "Nonlinear expression " << exprProduct << " created\n";

    SHOT::VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);

    std::cout << "Creating objective function:\n";
    SHOT::NonlinearObjectiveFunctionPtr nonlinearObjective = std::make_shared<SHOT::NonlinearObjectiveFunction>(
        SHOT::E_ObjectiveFunctionDirection::Minimize, linearTerms, quadraticTerms, exprProduct, 10.0);

    std::cout << "Objective function " << nonlinearObjective << " created\n";
    double objectiveValue = nonlinearObjective->calculateValue(point);
    double realValue = nonlinearObjective->constant + linearTerm1->coefficient * point.at(0)
        + linearTerm2->coefficient * point.at(1) + quadraticTerm1->coefficient * point.at(0) * point.at(1)
        + +quadraticTerm2->coefficient * point.at(0) * point.at(0) + point.at(0) * point.at(0) * point.at(1);

    std::cout << "Calculating objective value: " << objectiveValue << " (should be equal to " << realValue << ").\n";

    if(objectiveValue != realValue)
        passed = false;

    return passed;
}

bool ModelTestConstraints()
{
    bool passed = true;

    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 100.0);
    SHOT::ExpressionVariablePtr expressionVariable_x = std::make_shared<SHOT::ExpressionVariable>(var_x);

    auto var_y = std::make_shared<SHOT::Variable>("y", 1, SHOT::E_VariableType::Integer, 0.0, 1.0);
    SHOT::ExpressionVariablePtr expressionVariable_y = std::make_shared<SHOT::ExpressionVariable>(var_y);

    std::cout << "Creating linear terms\n";
    SHOT::LinearTermPtr linearTerm1 = std::make_shared<SHOT::LinearTerm>(-1, var_x);
    SHOT::LinearTermPtr linearTerm2 = std::make_shared<SHOT::LinearTerm>(1.2, var_y);
    SHOT::LinearTerms linearTerms;
    linearTerms.add(linearTerm1);
    linearTerms.add(linearTerm2);
    std::cout << "Linear terms " << linearTerms << " created\n";

    std::cout << "Creating quadratic terms\n";
    SHOT::QuadraticTermPtr quadraticTerm1 = std::make_shared<SHOT::QuadraticTerm>(1, var_x, var_y);
    SHOT::QuadraticTermPtr quadraticTerm2 = std::make_shared<SHOT::QuadraticTerm>(2, var_x, var_x);
    SHOT::QuadraticTerms quadraticTerms;
    quadraticTerms.add(quadraticTerm1);
    quadraticTerms.add(quadraticTerm2);
    std::cout << "Quadratic terms " << quadraticTerms << " created\n";

    std::cout << "Creating quadratic constraint:\n";
    SHOT::QuadraticConstraintPtr quadraticConstraint
        = std::make_shared<SHOT::QuadraticConstraint>(0, "quadconstr", linearTerms, quadraticTerms, -10.0, 20.0);
    std::cout << "Quadratic constraint created\n";

    SHOT::VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);

    auto constraintValue = quadraticConstraint->calculateNumericValue(point);
    double realValue = linearTerm1->coefficient * point.at(0) + linearTerm2->coefficient * point.at(1)
        + quadraticTerm1->coefficient * point.at(0) * point.at(1)
        + +quadraticTerm2->coefficient * point.at(0) * point.at(0);

    std::cout << "Calculating constraint value: " << constraintValue.functionValue << " (should be equal to "
              << realValue << ").\n";

    if(constraintValue.functionValue != realValue)
        passed = false;

    std::cout << "Creating nonlinear expression:\n";
    SHOT::NonlinearExpressions expressions;
    expressions.add(expressionVariable_x);
    expressions.add(expressionVariable_x);
    expressions.add(expressionVariable_y);
    SHOT::NonlinearExpressionPtr exprProduct = std::make_shared<SHOT::ExpressionProduct>(expressions);
    std::cout << "Nonlinear expression " << exprProduct << " created\n";

    std::cout << "Creating nonlinear constraint:\n";
    SHOT::NonlinearConstraintPtr nonlinearConstraint = std::make_shared<SHOT::NonlinearConstraint>(
        0, "nlconstr", linearTerms, quadraticTerms, exprProduct, -10.0, 20.0);
    std::cout << "Nonlinear constraint " << nonlinearConstraint << " created\n";

    constraintValue = nonlinearConstraint->calculateNumericValue(point);
    realValue = linearTerm1->coefficient * point.at(0) + linearTerm2->coefficient * point.at(1)
        + quadraticTerm1->coefficient * point.at(0) * point.at(1)
        + quadraticTerm2->coefficient * point.at(0) * point.at(0) + point.at(0) * point.at(0) * point.at(1);

    std::cout << "Calculating nonlinear constraint value " << constraintValue.functionValue << " (should be equal to "
              << realValue << ").\n";

    if(constraintValue.functionValue != realValue)
        passed = false;

    bool isFulfilled = nonlinearConstraint->isFulfilled(point);

    std::cout << "Is nonlinear constraint valid in point (x,y)=(" << point.at(0) << "," << point.at(1) << ")? ";
    std::cout << (nonlinearConstraint->isFulfilled(point) ? "yes" : "no");
    std::cout << ". Function value: " << nonlinearConstraint->calculateFunctionValue(point);
    std::cout << "\n";

    if(isFulfilled)
        passed = false;

    point.at(0) = 1.0;
    point.at(1) = 1.0;

    isFulfilled = nonlinearConstraint->isFulfilled(point);

    std::cout << "Is nonlinear constraint valid in point (x,y)=(" << point.at(0) << "," << point.at(1) << ")? ";
    std::cout << (nonlinearConstraint->isFulfilled(point) ? "yes" : "no");
    std::cout << ". Function value: " << nonlinearConstraint->calculateFunctionValue(point);
    std::cout << "\n";

    if(!isFulfilled)
        passed = false;

    return passed;
}

bool ModelTestCreateProblem()
{
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();
    SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);
    env->problem = problem;

    // Creating variables

    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 100.0);
    SHOT::ExpressionVariablePtr expressionVariable_x = std::make_shared<SHOT::ExpressionVariable>(var_x);

    auto var_y = std::make_shared<SHOT::Variable>("y", 1, SHOT::E_VariableType::Integer, 0.0, 1.0);
    SHOT::ExpressionVariablePtr expressionVariable_y = std::make_shared<SHOT::ExpressionVariable>(var_y);

    auto var_z = std::make_shared<SHOT::Variable>("z", 2, SHOT::E_VariableType::Integer, 0.0, 2.0);
    SHOT::ExpressionVariablePtr expressionVariable_z = std::make_shared<SHOT::ExpressionVariable>(var_z);

    SHOT::Variables variables = { var_x, var_y, var_z };
    problem->add(variables);

    SHOT::NonlinearObjectiveFunctionPtr objectiveFunction
        = std::make_shared<SHOT::NonlinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Minimize);
    SHOT::LinearTermPtr objLinearTerm1 = std::make_shared<SHOT::LinearTerm>(1.0, var_x);
    SHOT::LinearTermPtr objLinearTerm2 = std::make_shared<SHOT::LinearTerm>(1.0, var_y);

    SHOT::LinearTerms objLinearTerms;
    objLinearTerms.add(objLinearTerm1);
    objLinearTerms.add(objLinearTerm2);

    objectiveFunction->add(objLinearTerms);

    SHOT::QuadraticTermPtr objQuadraticTerm1 = std::make_shared<SHOT::QuadraticTerm>(1, var_x, var_y);
    SHOT::QuadraticTermPtr objQuadraticTerm2 = std::make_shared<SHOT::QuadraticTerm>(2, var_x, var_x);
    SHOT::QuadraticTerms objQuadraticTerms;
    objQuadraticTerms.add(objQuadraticTerm1);
    objQuadraticTerms.add(objQuadraticTerm2);
    objectiveFunction->add(objQuadraticTerms);

    SHOT::NonlinearExpressions objExpressions;
    objExpressions.add(expressionVariable_x);
    objExpressions.add(expressionVariable_y);
    SHOT::NonlinearExpressionPtr objExprSum = std::make_shared<SHOT::ExpressionSum>(objExpressions);
    objectiveFunction->add(objExprSum);
    problem->add(objectiveFunction);

    std::cout << "Objective function " << objectiveFunction << "created\n";

    SHOT::LinearTermPtr linearTerm1 = std::make_shared<SHOT::LinearTerm>(-1, var_x);
    SHOT::LinearTermPtr linearTerm2 = std::make_shared<SHOT::LinearTerm>(1.2, var_y);
    SHOT::LinearTerms linearTerms;
    linearTerms.add(linearTerm1);
    linearTerms.add(linearTerm2);
    SHOT::LinearConstraintPtr linearConstraint
        = std::make_shared<SHOT::LinearConstraint>(0, "linconstr", linearTerms, -2.0, 4.0);
    problem->add(linearConstraint);

    std::cout << '\n';
    std::cout << "Linear constraint " << linearConstraint << " created\n";

    SHOT::QuadraticTermPtr quadraticTerm1 = std::make_shared<SHOT::QuadraticTerm>(1, var_x, var_y);
    SHOT::QuadraticTermPtr quadraticTerm2 = std::make_shared<SHOT::QuadraticTerm>(2, var_x, var_x);
    SHOT::QuadraticTerms quadraticTerms;
    quadraticTerms.add(quadraticTerm1);
    quadraticTerms.add(quadraticTerm2);
    SHOT::QuadraticConstraintPtr quadraticConstraint
        = std::make_shared<SHOT::QuadraticConstraint>(1, "quadconstr", quadraticTerms, -10.0, 20.0);
    problem->add(quadraticConstraint);

    std::cout << '\n';
    std::cout << "Quadratic constraint " << quadraticConstraint << " created\n";

    // Create separate linear terms for the nonlinear constraint (don't share with linearConstraint)
    SHOT::LinearTermPtr nlLinearTerm1 = std::make_shared<SHOT::LinearTerm>(-1, var_x);
    SHOT::LinearTermPtr nlLinearTerm2 = std::make_shared<SHOT::LinearTerm>(1.2, var_y);
    SHOT::LinearTerms nlLinearTerms;
    nlLinearTerms.add(nlLinearTerm1);
    nlLinearTerms.add(nlLinearTerm2);

    SHOT::NonlinearExpressionPtr exprPlus
        = std::make_shared<SHOT::ExpressionSum>(expressionVariable_z, expressionVariable_x);
    SHOT::NonlinearExpressionPtr exprConstant = std::make_shared<SHOT::ExpressionConstant>(3);
    SHOT::NonlinearExpressionPtr exprPower
        = std::make_shared<SHOT::ExpressionPower>(expressionVariable_z, exprConstant);

    SHOT::NonlinearExpressions expressions;
    expressions.add(exprPower);
    expressions.add(exprPlus);

    SHOT::NonlinearExpressionPtr exprSum = std::make_shared<SHOT::ExpressionSum>(expressions);

    SHOT::NonlinearConstraintPtr nonlinearConstraint
        = std::make_shared<SHOT::NonlinearConstraint>(2, "nlconstr", nlLinearTerms, exprSum, -10.0, 20.0);
    problem->add(nonlinearConstraint);

    std::cout << '\n';
    std::cout << "Nonlinear constraint " << nonlinearConstraint << " created\n";

    SHOT::NonlinearExpressionPtr exprConstant2 = std::make_shared<SHOT::ExpressionConstant>(40.0);
    SHOT::NonlinearExpressionPtr exprTimes
        = std::make_shared<SHOT::ExpressionProduct>(exprConstant2, expressionVariable_x);
    SHOT::NonlinearExpressionPtr exprPlus2 = std::make_shared<SHOT::ExpressionProduct>(exprTimes, expressionVariable_y);

    SHOT::NonlinearConstraintPtr nonlinearConstraint2
        = std::make_shared<SHOT::NonlinearConstraint>(3, "nlconstr2", exprPlus2, -1000.0, 0);
    problem->add(nonlinearConstraint2);

    std::cout << '\n';
    std::cout << "Nonlinear constraint " << nonlinearConstraint2 << " created\n";

    std::cout << '\n';
    std::cout << "Finalizing problem:\n";
    problem->finalize();

    // After finalize(), constraints may have been replaced with different types
    // (e.g., NonlinearConstraint -> QuadraticConstraint if the expression was bilinear).
    // We need to get fresh references from the problem.
    auto finalLinearConstraint = std::dynamic_pointer_cast<SHOT::NumericConstraint>(problem->getConstraint(0));
    auto finalQuadraticConstraint = std::dynamic_pointer_cast<SHOT::NumericConstraint>(problem->getConstraint(1));
    auto finalNonlinearConstraint = std::dynamic_pointer_cast<SHOT::NumericConstraint>(problem->getConstraint(2));
    auto finalNonlinearConstraint2 = std::dynamic_pointer_cast<SHOT::NumericConstraint>(problem->getConstraint(3));

    std::cout << '\n';
    std::cout << "Problem created:\n\n";
    std::cout << problem << '\n';

    SHOT::VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);
    point.push_back(1.0);
    std::cout << '\n';
    std::cout << "Considering point (" << point[0] << ',' << point[1] << ',' << point[2] << ')' << '\n';

    std::cout << "\nJacobian sparsity pattern:\n";
    auto jacobianSparsityPattern = problem->getConstraintsJacobianSparsityPattern();

    for(auto& E : *jacobianSparsityPattern)
    {
        for(auto& V : E.second)
            std::cout << "(" << E.first->index << "," << V->index << ")\n";
    }

    std::cout << '\n';
    std::cout << "Hessian of the Lagrangian sparsity pattern:\n";
    auto lagrangianSparsityPattern = problem->getConstraintsHessianSparsityPattern();

    for(auto& E : *lagrangianSparsityPattern)
    {
        std::cout << "(" << E.first->index << "," << E.second->index << ")\n";
    }

    std::cout << "\nCalculating gradient for function in linear constraint:\n";
    auto gradientLinear = finalLinearConstraint->calculateGradient(point, true);

    for(auto const& G : gradientLinear)
    {
        std::cout << G.first->name << ": " << G.second << '\n';
    }

    std::cout << "\nCalculating Hessian for function in linear constraint (there should be none):\n";
    auto hessianLinear = finalLinearConstraint->calculateHessian(point, true);

    if(hessianLinear.size() > 0)
    {
        std::cout << "The number of Hessian elements is: " << hessianLinear.size() << ".\n";

        for(auto const& H : hessianLinear)
        {
            std::cout << "(" + H.first.first->name << "," << H.first.second->name << "): " << H.second << '\n';
        }

        passed = false;
    }

    std::cout << "\nCalculating gradient for function in quadratic constraint:\n";
    auto gradientQuadratic = finalQuadraticConstraint->calculateGradient(point, true);

    for(auto const& G : gradientQuadratic)
    {
        std::cout << G.first->name << ": " << G.second << '\n';
    }

    std::cout << "\nCalculating hessian for function in quadratic constraint:\n";
    auto hessianQuadratic = finalQuadraticConstraint->calculateHessian(point, true);

    for(auto const& H : hessianQuadratic)
    {
        std::cout << "(" + H.first.first->name << "," << H.first.second->name << "): " << H.second << '\n';
    }

    std::cout << "\nCalculating gradient for function in first nonlinear constraint:\n";
    auto gradientNonlinear = finalNonlinearConstraint->calculateGradient(point, true);

    for(auto const& G : gradientNonlinear)
    {
        std::cout << G.first->name << ":  " << G.second << '\n';
    }

    std::cout << "\nCalculating hessian for function in first nonlinear constraint (there should be one element):\n";
    auto hessianNonlinear = finalNonlinearConstraint->calculateHessian(point, true);

    for(auto const& H : hessianNonlinear)
    {
        std::cout << "(" + H.first.first->name << "," << H.first.second->name << "): " << H.second << '\n';
    }

    std::cout << "\nCalculating gradient for function in second nonlinear constraint:\n";
    auto gradientNonlinear2 = finalNonlinearConstraint2->calculateGradient(point, true);

    for(auto const& G : gradientNonlinear2)
    {
        std::cout << G.first->name << ":  " << G.second << '\n';
    }

    std::cout << "\nCalculating hessian for function in second nonlinear constraint:\n";
    auto hessianNonlinear2 = finalNonlinearConstraint2->calculateHessian(point, true);

    for(auto const& H : hessianNonlinear2)
    {
        std::cout << "(" + H.first.first->name << "," << H.first.second->name << "): " << H.second << '\n';
    }

    SHOT::Interval X(1., 2.);
    SHOT::Interval Y(2., 3.);
    SHOT::Interval Z(3., 4.);

    SHOT::IntervalVector vector;
    vector.push_back(X);
    vector.push_back(Y);
    vector.push_back(Z);

    std::cout << "\nCalculating function interval value for interval vector\n";
    std::cout << "x = " << X << '\n';
    std::cout << "y = " << Y << '\n';
    std::cout << "z = " << Z << '\n';

    auto linearIntervalValue = linearConstraint->calculateFunctionValue(vector);
    std::cout << "\nValue for linear constraint is: ";
    std::cout << linearIntervalValue << '\n';

    if(std::abs(linearIntervalValue.l() - 0.4) > (1.0e-12) || std::abs(linearIntervalValue.u() - 2.6) > (1.0e-12))
    {
        std::cout << "Interval is not correct.\n";
        passed = false;
    }

    auto quadraticIntervalValue = quadraticConstraint->calculateFunctionValue(vector);
    std::cout << "\nValue for quadratic constraint is: ";
    std::cout << quadraticIntervalValue << '\n';

    if(std::abs(quadraticIntervalValue.l() - 4) > (1.0e-12) || std::abs(quadraticIntervalValue.u() - 14) > (1.0e-12))
    {
        std::cout << "Interval is not correct.\n";
        passed = false;
    }

    auto nonlinearIntervalValue = nonlinearConstraint->calculateFunctionValue(vector);
    std::cout << "\nValue for nonlinear constraint is: ";
    std::cout << nonlinearIntervalValue << '\n';

    if(std::abs(nonlinearIntervalValue.l() - 31.4) > (1.0e-12)
        || std::abs(nonlinearIntervalValue.u() - 72.6) > (1.0e-12))
    {
        std::cout << "Interval is not correct.\n";
        passed = false;
    }

    point.at(0) = 20.0;
    point.at(1) = 1.0;
    point.at(2) = 1.0;

    std::cout << '\n';
    std::cout << "Considering point (" << point[0] << ',' << point[1] << ',' << point[2] << ')' << '\n';

    std::cout << "\nTesting for an invalid point for all constraints:\n";
    auto mostDevConstraint = problem->getMostDeviatingNumericConstraint(point);
    if(!mostDevConstraint)
    {
        passed = false;
    }
    else
    {
        double error = mostDevConstraint.value().error;
        auto name = mostDevConstraint.value().constraint->name;
        std::cout << "The most deviating constraint is " << name << " with error " << error << "\n";

        if(error != 800.0)
            passed = false;
    }

    std::cout << '\n';

    point.at(0) = 0.0;
    point.at(1) = 1.0;

    std::cout << '\n';
    std::cout << "Considering point (" << point[0] << ',' << point[1] << ',' << point[2] << ')' << '\n';

    std::cout << "Testing to get all deviating constraint values in a valid point (there should be none)\n";

    auto deviatingConstraints = problem->getAllDeviatingNumericConstraints(point, 0.0);
    std::cout << "Number of invalid constraints in the point is " << deviatingConstraints.size() << '\n';

    if(deviatingConstraints.size() != 0)
    {
        passed = false;
    }

    std::cout << '\n';

    std::cout << "Testing to get the most deviating constraint value in a valid point (x,y) = (" << point.at(0) << ','
              << point.at(1) << "):\n";

    auto mostDevConstraint2 = problem->getMostDeviatingNumericConstraint(point);

    if(!mostDevConstraint2)
    {
        std::cout << "Constraint not found, everything ok\n";
        passed = true;
    }
    else
    {
        std::cout << "Constraint found, something went wrong\n";
        passed = false;
    }

    return passed;
}

bool ModelTestCreateProblem2()
{
    // Nonlinear constraint with only one variable (out of a total of two)
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();
    SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);
    env->problem = problem;

    // Creating variables
    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 100.0);
    auto var_y = std::make_shared<SHOT::Variable>("y", 1, SHOT::E_VariableType::Integer, 0.0, 1.0);
    SHOT::ExpressionVariablePtr expressionVariable_y = std::make_shared<SHOT::ExpressionVariable>(var_y);

    SHOT::Variables variables = { var_x, var_y };
    problem->add(variables);

    SHOT::LinearObjectiveFunctionPtr objectiveFunction
        = std::make_shared<SHOT::LinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Minimize);
    SHOT::LinearTermPtr objLinearTerm1 = std::make_shared<SHOT::LinearTerm>(1.0, var_x);
    SHOT::LinearTermPtr objLinearTerm2 = std::make_shared<SHOT::LinearTerm>(1.0, var_y);

    SHOT::LinearTerms objLinearTerms;
    objLinearTerms.add(objLinearTerm1);
    objLinearTerms.add(objLinearTerm2);

    objectiveFunction->add(objLinearTerms);
    problem->add(objectiveFunction);

    SHOT::NonlinearExpressionPtr exprConstant = std::make_shared<SHOT::ExpressionConstant>(3);
    SHOT::NonlinearExpressionPtr exprPower
        = std::make_shared<SHOT::ExpressionPower>(expressionVariable_y, exprConstant);

    SHOT::NonlinearConstraintPtr nonlinearConstraint
        = std::make_shared<SHOT::NonlinearConstraint>(0, "nlconstr", exprPower, -10.0, 20.0);
    problem->add(nonlinearConstraint);

    std::cout << '\n';
    std::cout << "Finalizing problem:\n";
    problem->finalize();

    std::cout << '\n';
    std::cout << "Problem created:\n\n";
    std::cout << problem << '\n';

    SHOT::VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);

    std::cout << '\n';
    std::cout << "Considering point (" << point[0] << ',' << point[1] << ')' << '\n';

    std::cout << '\n';
    std::cout << "Jacobian sparsity pattern:\n";
    auto jacobianSparsityPattern = problem->getConstraintsJacobianSparsityPattern();

    for(auto& E : *jacobianSparsityPattern)
    {
        for(auto& V : E.second)
            std::cout << "(" << E.first->index << "," << V->index << ")\n";
    }

    if(!(jacobianSparsityPattern->at(0).first->index == 0 && jacobianSparsityPattern->at(0).second.at(0)->index == 1))
    {
        std::cout << "The sparsity pattern is wrong!\n";
        passed = false;
    }

    std::cout << '\n';
    std::cout << "Hessian of the Lagrangian sparsity pattern:\n";
    auto lagrangianSparsityPattern = problem->getConstraintsHessianSparsityPattern();

    for(auto& E : *lagrangianSparsityPattern)
    {
        std::cout << "(" << E.first->index << "," << E.second->index << ")\n";
    }

    if(!(lagrangianSparsityPattern->at(0).first->index == 1 && lagrangianSparsityPattern->at(0).second->index == 1))
    {
        std::cout << "The sparsity pattern is wrong!\n";
        passed = false;
    }

    std::cout << "\nCalculating gradient for function in first nonlinear constraint:\n";
    auto gradientNonlinear = nonlinearConstraint->calculateGradient(point, true);

    for(auto const& G : gradientNonlinear)
    {
        std::cout << G.first->name << ":  " << G.second << '\n';
    }

    std::cout << "\nCalculating Hessian for function in first nonlinear constraint:\n";
    auto hessianNonlinear = nonlinearConstraint->calculateHessian(point, true);

    for(auto const& H : hessianNonlinear)
    {
        std::cout << "(" + H.first.first->name << "," << H.first.second->name << "): " << H.second << '\n';
    }

    return passed;
}

bool ModelTestCreateProblem3()
{
    // Two nonlinear constraint with only one variable each
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();
    SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);
    env->problem = problem;

    // Creating variables

    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 100.0);
    SHOT::ExpressionVariablePtr expressionVariable_x = std::make_shared<SHOT::ExpressionVariable>(var_x);

    auto var_y = std::make_shared<SHOT::Variable>("y", 1, SHOT::E_VariableType::Integer, 0.0, 1.0);
    SHOT::ExpressionVariablePtr expressionVariable_y = std::make_shared<SHOT::ExpressionVariable>(var_y);

    SHOT::Variables variables = { var_x, var_y };
    problem->add(variables);

    SHOT::NonlinearObjectiveFunctionPtr objectiveFunction
        = std::make_shared<SHOT::NonlinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Minimize);
    SHOT::LinearTermPtr objLinearTerm1 = std::make_shared<SHOT::LinearTerm>(1.0, var_x);
    SHOT::LinearTermPtr objLinearTerm2 = std::make_shared<SHOT::LinearTerm>(1.0, var_y);

    SHOT::LinearTerms objLinearTerms;
    objLinearTerms.add(objLinearTerm1);
    objLinearTerms.add(objLinearTerm2);

    SHOT::NonlinearExpressionPtr exprProd
        = std::make_shared<SHOT::ExpressionProduct>(expressionVariable_x, expressionVariable_y);

    objectiveFunction->add(objLinearTerms);
    objectiveFunction->add(exprProd);
    problem->add(objectiveFunction);

    SHOT::NonlinearExpressionPtr exprConstant = std::make_shared<SHOT::ExpressionConstant>(3);
    SHOT::NonlinearExpressionPtr exprPower
        = std::make_shared<SHOT::ExpressionPower>(expressionVariable_y, exprConstant);

    SHOT::NonlinearConstraintPtr nonlinearConstraint
        = std::make_shared<SHOT::NonlinearConstraint>(0, "nlconstr", exprPower, -10.0, 20.0);
    problem->add(nonlinearConstraint);

    SHOT::NonlinearExpressionPtr exprConstant2 = std::make_shared<SHOT::ExpressionConstant>(4);
    SHOT::NonlinearExpressionPtr exprPower2
        = std::make_shared<SHOT::ExpressionPower>(expressionVariable_x, exprConstant2);

    SHOT::NonlinearConstraintPtr nonlinearConstraint2
        = std::make_shared<SHOT::NonlinearConstraint>(1, "nlconstr2", exprPower2, -10.0, 20.0);
    problem->add(nonlinearConstraint2);

    std::cout << '\n';
    std::cout << "Finalizing problem:\n";
    problem->finalize();

    std::cout << '\n';
    std::cout << "Problem created:\n\n";
    std::cout << problem << '\n';

    SHOT::VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);

    std::cout << '\n';
    std::cout << "Considering point (" << point[0] << ',' << point[1] << ')' << '\n';

    std::cout << '\n';
    std::cout << "Jacobian sparsity pattern:\n";
    auto jacobianSparsityPattern = problem->getConstraintsJacobianSparsityPattern();

    for(auto& E : *jacobianSparsityPattern)
    {
        for(auto& V : E.second)
            std::cout << "(" << E.first->index << "," << V->index << ")\n";
    }

    if(!(jacobianSparsityPattern->at(0).first->index == 0 && jacobianSparsityPattern->at(0).second.at(0)->index == 1
           && jacobianSparsityPattern->at(1).first->index == 1
           && jacobianSparsityPattern->at(1).second.at(0)->index == 0))
    {
        std::cout << "The sparsity pattern is wrong!\n";
        passed = false;
    }

    std::cout << '\n';
    std::cout << "Hessian of the Lagrangian sparsity pattern (constraints only):\n";
    auto lagrangianSparsityPattern = problem->getConstraintsHessianSparsityPattern();

    for(auto& E : *lagrangianSparsityPattern)
    {
        std::cout << "(" << E.first->index << "," << E.second->index << ")\n";
    }

    if(!(lagrangianSparsityPattern->at(0).first->index == 0 && lagrangianSparsityPattern->at(0).second->index == 0
           && lagrangianSparsityPattern->at(1).first->index == 1
           && lagrangianSparsityPattern->at(1).second->index == 1))
    {
        std::cout << "The sparsity pattern is wrong!\n";
        passed = false;
    }

    std::cout << '\n';
    std::cout << "Hessian of the Lagrangian sparsity pattern (with objective function):\n";
    lagrangianSparsityPattern = problem->getLagrangianHessianSparsityPattern();

    for(auto& E : *lagrangianSparsityPattern)
    {
        std::cout << "(" << E.first->index << "," << E.second->index << ")\n";
    }

    if(!(lagrangianSparsityPattern->at(0).first->index == 0 && lagrangianSparsityPattern->at(0).second->index == 0
           && lagrangianSparsityPattern->at(1).first->index == 0 && lagrangianSparsityPattern->at(1).second->index == 1
           && lagrangianSparsityPattern->at(2).first->index == 1
           && lagrangianSparsityPattern->at(2).second->index == 1))
    {
        std::cout << "The sparsity pattern is wrong!\n";
        passed = false;
    }

    std::cout << "\nCalculating gradient for function in first nonlinear constraint:\n";
    auto gradientNonlinear = nonlinearConstraint->calculateGradient(point, true);

    for(auto const& G : gradientNonlinear)
    {
        std::cout << G.first->name << ":  " << G.second << '\n';
    }

    std::cout << "\nCalculating Hessian for function in first nonlinear constraint:\n";
    auto hessianNonlinear = nonlinearConstraint->calculateHessian(point, true);

    for(auto const& H : hessianNonlinear)
    {
        std::cout << "(" + H.first.first->name << "," << H.first.second->name << "): " << H.second << '\n';
    }

    std::cout << "\nCalculating gradient for function in second nonlinear constraint:\n";
    gradientNonlinear = nonlinearConstraint2->calculateGradient(point, true);

    for(auto const& G : gradientNonlinear)
    {
        std::cout << G.first->name << ":  " << G.second << '\n';
    }

    std::cout << "\nCalculating hessian for function in second nonlinear constraint:\n";
    hessianNonlinear = nonlinearConstraint2->calculateHessian(point, true);

    for(auto const& H : hessianNonlinear)
    {
        std::cout << "(" + H.first.first->name << "," << H.first.second->name << "): " << H.second << '\n';
    }

    return passed;
}

bool ModelTestConvexity()
{
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();
    SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);
    env->problem = problem;

    // Creating variables

    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 100.0);
    SHOT::ExpressionVariablePtr expressionVariable_x = std::make_shared<SHOT::ExpressionVariable>(var_x);

    auto var_y = std::make_shared<SHOT::Variable>("y", 1, SHOT::E_VariableType::Integer, 0.0, 1.0);
    SHOT::ExpressionVariablePtr expressionVariable_y = std::make_shared<SHOT::ExpressionVariable>(var_y);

    SHOT::Variables variables = { var_x, var_y };
    problem->add(variables);

    auto objectiveFunction
        = std::make_shared<SHOT::QuadraticObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Minimize);
    SHOT::LinearTermPtr objLinearTerm1 = std::make_shared<SHOT::LinearTerm>(1.0, var_x);
    SHOT::LinearTermPtr objLinearTerm2 = std::make_shared<SHOT::LinearTerm>(1.0, var_y);

    SHOT::LinearTerms objLinearTerms;
    objLinearTerms.add(objLinearTerm1);
    objLinearTerms.add(objLinearTerm2);

    objectiveFunction->add(objLinearTerms);

    SHOT::QuadraticTermPtr objQuadraticTerm1 = std::make_shared<SHOT::QuadraticTerm>(1, var_x, var_y);
    SHOT::QuadraticTermPtr objQuadraticTerm2 = std::make_shared<SHOT::QuadraticTerm>(2, var_x, var_x);
    SHOT::QuadraticTerms objQuadraticTerms;
    objQuadraticTerms.add(objQuadraticTerm1);
    objQuadraticTerms.add(objQuadraticTerm2);
    objectiveFunction->add(objQuadraticTerms);

    SHOT::QuadraticTermPtr quadraticTerm1 = std::make_shared<SHOT::QuadraticTerm>(1, var_x, var_x);
    SHOT::QuadraticTermPtr quadraticTerm2 = std::make_shared<SHOT::QuadraticTerm>(2, var_x, var_y);
    SHOT::QuadraticTermPtr quadraticTerm3 = std::make_shared<SHOT::QuadraticTerm>(1, var_y, var_y);
    SHOT::QuadraticTerms quadraticTerms;
    quadraticTerms.add(quadraticTerm1);
    quadraticTerms.add(quadraticTerm2);
    quadraticTerms.add(quadraticTerm3);

    SHOT::QuadraticConstraintPtr quadraticConstraint
        = std::make_shared<SHOT::QuadraticConstraint>(0, "quadconstr", quadraticTerms, -10.0, 20.0);

    problem->add(objectiveFunction);

    problem->finalize();

    std::cout << "Objective function " << problem->objectiveFunction << " created.\n";

    auto convexity = objQuadraticTerms.getConvexity();

    switch(convexity)
    {
    case(E_Convexity::Convex):
        std::cout << "Objective is convex\n";
        break;

    case(E_Convexity::Concave):
        std::cout << "Objective is concave\n";
        break;
    case(E_Convexity::Linear):
        std::cout << "Objective is linear\n";
        break;
    case(E_Convexity::Nonconvex):
        std::cout << "Objective is nonconvex\n";
        break;
    case(E_Convexity::Unknown):
        std::cout << "Convexity status of objective unknown\n";
        break;
    case(E_Convexity::NotSet):
        std::cout << "Convexity status of objective not set\n";
        break;
    default:
        break;
    }

    std::cout << "\nConstraint " << quadraticConstraint << " created.\n";

    convexity = quadraticTerms.getConvexity();

    switch(convexity)
    {
    case(E_Convexity::Convex):
        std::cout << "Constraint is convex\n";
        break;

    case(E_Convexity::Concave):
        std::cout << "Constraint is concave\n";
        break;
    case(E_Convexity::Linear):
        std::cout << "Constraint is linear\n";
        break;
    case(E_Convexity::Nonconvex):
        std::cout << "Constraint is nonconvex\n";
        break;
    case(E_Convexity::Unknown):
        std::cout << "Constraint status of objective unknown\n";
        break;
    case(E_Convexity::NotSet):
        std::cout << "Constraint status of objective not set\n";
        break;
    default:
        break;
    }

    return passed;
}

bool ModelTestCopy()
{
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();
    SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);
    env->problem = problem;

    // Creating variables

    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 100.0);
    SHOT::ExpressionVariablePtr expressionVariable_x = std::make_shared<SHOT::ExpressionVariable>(var_x);

    auto var_y = std::make_shared<SHOT::Variable>("y", 1, SHOT::E_VariableType::Integer, 0.0, 1.0);
    SHOT::ExpressionVariablePtr expressionVariable_y = std::make_shared<SHOT::ExpressionVariable>(var_y);

    auto var_z = std::make_shared<SHOT::Variable>("z", 2, SHOT::E_VariableType::Integer, 0.0, 2.0);
    SHOT::ExpressionVariablePtr expressionVariable_z = std::make_shared<SHOT::ExpressionVariable>(var_z);

    SHOT::Variables variables = { var_x, var_y, var_z };
    problem->add(variables);

    SHOT::NonlinearObjectiveFunctionPtr objectiveFunction
        = std::make_shared<SHOT::NonlinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Minimize);
    SHOT::LinearTermPtr objLinearTerm1 = std::make_shared<SHOT::LinearTerm>(1.0, var_x);
    SHOT::LinearTermPtr objLinearTerm2 = std::make_shared<SHOT::LinearTerm>(1.0, var_y);

    SHOT::LinearTerms objLinearTerms;
    objLinearTerms.add(objLinearTerm1);
    objLinearTerms.add(objLinearTerm2);

    objectiveFunction->add(objLinearTerms);

    SHOT::QuadraticTermPtr objQuadraticTerm1 = std::make_shared<SHOT::QuadraticTerm>(1, var_x, var_y);
    SHOT::QuadraticTermPtr objQuadraticTerm2 = std::make_shared<SHOT::QuadraticTerm>(2, var_x, var_x);
    SHOT::QuadraticTerms objQuadraticTerms;
    objQuadraticTerms.add(objQuadraticTerm1);
    objQuadraticTerms.add(objQuadraticTerm2);
    objectiveFunction->add(objQuadraticTerms);

    SHOT::NonlinearExpressions objExpressions;
    objExpressions.add(expressionVariable_x);
    objExpressions.add(expressionVariable_y);
    SHOT::NonlinearExpressionPtr objExprSum = std::make_shared<SHOT::ExpressionSum>(objExpressions);
    objectiveFunction->add(objExprSum);
    problem->add(objectiveFunction);

    SHOT::LinearTermPtr linearTerm1 = std::make_shared<SHOT::LinearTerm>(-1, var_x);
    SHOT::LinearTermPtr linearTerm2 = std::make_shared<SHOT::LinearTerm>(1.2, var_y);
    SHOT::LinearTerms linearTerms;
    linearTerms.add(linearTerm1);
    linearTerms.add(linearTerm2);
    SHOT::LinearConstraintPtr linearConstraint
        = std::make_shared<SHOT::LinearConstraint>(0, "linconstr", linearTerms, -2.0, 4.0);
    problem->add(linearConstraint);

    SHOT::QuadraticTermPtr quadraticTerm1 = std::make_shared<SHOT::QuadraticTerm>(1, var_x, var_y);
    SHOT::QuadraticTermPtr quadraticTerm2 = std::make_shared<SHOT::QuadraticTerm>(2, var_x, var_x);
    SHOT::QuadraticTerms quadraticTerms;
    quadraticTerms.add(quadraticTerm1);
    quadraticTerms.add(quadraticTerm2);
    SHOT::QuadraticConstraintPtr quadraticConstraint
        = std::make_shared<SHOT::QuadraticConstraint>(1, "quadconstr", quadraticTerms, -10.0, 20.0);
    problem->add(quadraticConstraint);

    SHOT::NonlinearExpressionPtr exprPlus
        = std::make_shared<SHOT::ExpressionSum>(expressionVariable_z, expressionVariable_x);
    SHOT::NonlinearExpressionPtr exprConstant = std::make_shared<SHOT::ExpressionConstant>(3);
    SHOT::NonlinearExpressionPtr exprPower
        = std::make_shared<SHOT::ExpressionPower>(expressionVariable_z, exprConstant);

    SHOT::NonlinearExpressions expressions;
    expressions.add(exprPower);
    expressions.add(exprPlus);

    SHOT::NonlinearExpressionPtr exprSum = std::make_shared<SHOT::ExpressionSum>(expressions);

    SHOT::NonlinearConstraintPtr nonlinearConstraint
        = std::make_shared<SHOT::NonlinearConstraint>(2, "nlconstr", linearTerms, exprSum, -10.0, 20.0);
    problem->add(nonlinearConstraint);

    SHOT::NonlinearExpressionPtr exprConstant2 = std::make_shared<SHOT::ExpressionConstant>(40.0);
    SHOT::NonlinearExpressionPtr exprTimes
        = std::make_shared<SHOT::ExpressionProduct>(exprConstant2, expressionVariable_x);
    SHOT::NonlinearExpressionPtr exprPlus2 = std::make_shared<SHOT::ExpressionProduct>(exprTimes, expressionVariable_y);

    SHOT::NonlinearConstraintPtr nonlinearConstraint2
        = std::make_shared<SHOT::NonlinearConstraint>(3, "nlconstr2", exprPlus2, -1000.0, 0);
    problem->add(nonlinearConstraint2);

    problem->finalize();

    std::stringstream problemText;
    problemText << problem;

    std::cout << "Problem created:\n\n";
    std::cout << problemText.str() << '\n';

    auto problemCopy = problem->createCopy(solver->getEnvironment());

    std::stringstream problemCopyText;
    problemCopyText << problemCopy;

    std::cout << "Problem copy created:\n\n";
    std::cout << problemCopyText.str() << '\n';

    if(problemCopyText.str().erase('*') != problemText.str().erase('*'))
        passed = false;

    auto problemRelaxedCopy = problem->createCopy(solver->getEnvironment(), true);
    std::cout << "Relaxed problem copy created:\n\n";
    std::cout << problemRelaxedCopy << '\n';

    return passed;
}

bool ModelTestEx1223b()
{
    // Test the ex1223b problem from MINLPLib using the C++ API
    // This is a convex MINLP with 3 continuous and 4 binary variables.
    // Optimal solution: x1=0.2, x2=0.8, x3=1.907878, b4=1, b5=1, b6=0, b7=1
    // Optimal objective: 4.579582402436710

    bool passed = true;

    std::cout << "\n=== Testing ex1223b problem creation and solving ===\n\n";

    // Initializing the SHOT solver class
    auto solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
    solver->updateSetting("Output.Debug.Enable", true);

    // Initializing a SHOT problem class
    auto problem = std::make_shared<Problem>(env);
    problem->name = "ex1223b";

    // Creating the variables
    auto x1 = std::make_shared<Variable>("x1", 0, E_VariableType::Real, 0.0, 10.0);
    auto x2 = std::make_shared<Variable>("x2", 1, E_VariableType::Real, 0.0, 10.0);
    auto x3 = std::make_shared<Variable>("x3", 2, E_VariableType::Real, 0.0, 10.0);
    auto b4 = std::make_shared<Variable>("b4", 3, E_VariableType::Binary);
    auto b5 = std::make_shared<Variable>("b5", 4, E_VariableType::Binary);
    auto b6 = std::make_shared<Variable>("b6", 5, E_VariableType::Binary);
    auto b7 = std::make_shared<Variable>("b7", 6, E_VariableType::Binary);

    // Expression variables for nonlinear terms
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
    // minimize (b4-1)^2 + (b5-2)^2 + (b6-1)^2 - log(1+b7) + (x1-1)^2 + (x2-2)^2 + (x3-3)^2
    auto objective = std::make_shared<NonlinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    problem->add(objective);

    // (b4 - 1)^2
    objective->add(std::make_shared<ExpressionSquare>(
        std::make_shared<ExpressionSum>(std::make_shared<ExpressionConstant>(-1), nl_b4)));
    // (b5 - 2)^2
    objective->add(std::make_shared<ExpressionSquare>(
        std::make_shared<ExpressionSum>(std::make_shared<ExpressionConstant>(-2), nl_b5)));
    // (b6 - 1)^2
    objective->add(std::make_shared<ExpressionSquare>(
        std::make_shared<ExpressionSum>(std::make_shared<ExpressionConstant>(-1), nl_b6)));
    // -log(1 + b7)
    objective->add(std::make_shared<ExpressionNegate>(std::make_shared<ExpressionLog>(
        std::make_shared<ExpressionSum>(std::make_shared<ExpressionConstant>(1), nl_b7))));
    // (x1 - 1)^2
    objective->add(std::make_shared<ExpressionSquare>(
        std::make_shared<ExpressionSum>(std::make_shared<ExpressionConstant>(-1), nl_x1)));
    // (x2 - 2)^2
    objective->add(std::make_shared<ExpressionSquare>(
        std::make_shared<ExpressionSum>(std::make_shared<ExpressionConstant>(-2), nl_x2)));
    // (x3 - 3)^2
    objective->add(std::make_shared<ExpressionSquare>(
        std::make_shared<ExpressionSum>(std::make_shared<ExpressionConstant>(-3), nl_x3)));

    // e1: x1 + x2 + x3 + b4 + b5 + b6 <= 5
    auto e1 = std::make_shared<LinearConstraint>(0, "e1", SHOT_DBL_MIN, 5.0);
    e1->add(std::make_shared<LinearTerm>(1.0, x1));
    e1->add(std::make_shared<LinearTerm>(1.0, x2));
    e1->add(std::make_shared<LinearTerm>(1.0, x3));
    e1->add(std::make_shared<LinearTerm>(1.0, b4));
    e1->add(std::make_shared<LinearTerm>(1.0, b5));
    e1->add(std::make_shared<LinearTerm>(1.0, b6));
    problem->add(e1);

    // e2: b6^2 + x1^2 + x2^2 + x3^2 <= 5.5
    auto e2 = std::make_shared<QuadraticConstraint>(1, "e2", SHOT_DBL_MIN, 5.5);
    e2->add(std::make_shared<QuadraticTerm>(1.0, b6, b6));
    e2->add(std::make_shared<QuadraticTerm>(1.0, x1, x1));
    e2->add(std::make_shared<QuadraticTerm>(1.0, x2, x2));
    e2->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
    problem->add(e2);

    // e3: x1 + b4 <= 1.2
    auto e3 = std::make_shared<LinearConstraint>(2, "e3", SHOT_DBL_MIN, 1.2);
    e3->add(std::make_shared<LinearTerm>(1.0, x1));
    e3->add(std::make_shared<LinearTerm>(1.0, b4));
    problem->add(e3);

    // e4: x2 + b5 <= 1.8
    auto e4 = std::make_shared<LinearConstraint>(3, "e4", SHOT_DBL_MIN, 1.8);
    e4->add(std::make_shared<LinearTerm>(1.0, x2));
    e4->add(std::make_shared<LinearTerm>(1.0, b5));
    problem->add(e4);

    // e5: x3 + b6 <= 2.5
    auto e5 = std::make_shared<LinearConstraint>(4, "e5", SHOT_DBL_MIN, 2.5);
    e5->add(std::make_shared<LinearTerm>(1.0, x3));
    e5->add(std::make_shared<LinearTerm>(1.0, b6));
    problem->add(e5);

    // e6: x1 + b7 <= 1.2
    auto e6 = std::make_shared<LinearConstraint>(5, "e6", SHOT_DBL_MIN, 1.2);
    e6->add(std::make_shared<LinearTerm>(1.0, x1));
    e6->add(std::make_shared<LinearTerm>(1.0, b7));
    problem->add(e6);

    // e7: b5^2 + x2^2 <= 1.64
    auto e7 = std::make_shared<QuadraticConstraint>(6, "e7", SHOT_DBL_MIN, 1.64);
    e7->add(std::make_shared<QuadraticTerm>(1.0, b5, b5));
    e7->add(std::make_shared<QuadraticTerm>(1.0, x2, x2));
    problem->add(e7);

    // e8: b6^2 + x3^2 <= 4.25
    auto e8 = std::make_shared<QuadraticConstraint>(7, "e8", SHOT_DBL_MIN, 4.25);
    e8->add(std::make_shared<QuadraticTerm>(1.0, b6, b6));
    e8->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
    problem->add(e8);

    // e9: b5^2 + x3^2 <= 4.64
    auto e9 = std::make_shared<QuadraticConstraint>(8, "e9", SHOT_DBL_MIN, 4.64);
    e9->add(std::make_shared<QuadraticTerm>(1.0, b5, b5));
    e9->add(std::make_shared<QuadraticTerm>(1.0, x3, x3));
    problem->add(e9);

    // Finalize the problem object (this now includes simplifyNonlinearExpressions and updateProperties)
    problem->finalize();

    std::cout << "Problem created:\n\n";
    std::cout << problem << '\n';

    // Set problem and solve
    solver->setProblem(problem);

    std::cout << "\nSolving...\n";

    if(!solver->solveProblem())
    {
        std::cout << "Failed to solve problem!\n";
        passed = false;
    }
    else
    {
        auto solutions = env->results->primalSolutions;

        if(solutions.size() == 0)
        {
            std::cout << "No solution found!\n";
            passed = false;
        }
        else
        {
            double objValue = solutions[0].objValue;
            double expectedObj = 4.579582;

            std::cout << "\nSolution found:\n";
            std::cout << "  Objective value: " << objValue << "\n";
            std::cout << "  Expected value:  " << expectedObj << "\n";

            if(solutions[0].point.size() >= 7)
            {
                std::cout << "  x1 = " << solutions[0].point[0] << "\n";
                std::cout << "  x2 = " << solutions[0].point[1] << "\n";
                std::cout << "  x3 = " << solutions[0].point[2] << "\n";
                std::cout << "  b4 = " << solutions[0].point[3] << "\n";
                std::cout << "  b5 = " << solutions[0].point[4] << "\n";
                std::cout << "  b6 = " << solutions[0].point[5] << "\n";
                std::cout << "  b7 = " << solutions[0].point[6] << "\n";
            }

            if(std::abs(objValue - expectedObj) < 0.01)
            {
                std::cout << "\n*** TEST PASSED: Objective matches expected value! ***\n";
            }
            else
            {
                std::cout << "\n*** TEST FAILED: Objective differs from expected! ***\n";
                std::cout << "  Difference: " << std::abs(objValue - expectedObj) << "\n";
                passed = false;
            }
        }
    }

    return passed;
}

bool ModelTestMeanvarxscWithSolver(ES_MIPSolver mipSolver)
{
    bool passed = true;

    std::string solverName;
    switch(mipSolver)
    {
    case ES_MIPSolver::Cbc:
        solverName = "CBC";
        break;
    case ES_MIPSolver::Highs:
        solverName = "HiGHS";
        break;
    case ES_MIPSolver::Cplex:
        solverName = "CPLEX";
        break;
    case ES_MIPSolver::Gurobi:
        solverName = "Gurobi";
        break;
    default:
        solverName = "unknown";
        break;
    }

    std::cout << "\n=== Testing meanvarxsc problem with " << solverName << " ===\n\n";

    auto solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(mipSolver));

    auto problem = std::make_shared<Problem>(env);
    problem->name = "meanvarxsc";

    auto x2 = std::make_shared<Variable>("x2", 0, E_VariableType::Real, 0.0, SHOT_DBL_MAX);
    auto x3 = std::make_shared<Variable>("x3", 1, E_VariableType::Real, 0.0, SHOT_DBL_MAX);
    auto x4 = std::make_shared<Variable>("x4", 2, E_VariableType::Real, 0.0, SHOT_DBL_MAX);
    auto x5 = std::make_shared<Variable>("x5", 3, E_VariableType::Real, 0.0, SHOT_DBL_MAX);
    auto x6 = std::make_shared<Variable>("x6", 4, E_VariableType::Real, 0.0, SHOT_DBL_MAX);
    auto x7 = std::make_shared<Variable>("x7", 5, E_VariableType::Real, 0.0, SHOT_DBL_MAX);
    auto x8 = std::make_shared<Variable>("x8", 6, E_VariableType::Real, 0.0, SHOT_DBL_MAX);

    auto sc9 = std::make_shared<Variable>("sc9", 7, E_VariableType::Semicontinuous, 0.0, 0.11, 0.03);
    auto sc10 = std::make_shared<Variable>("sc10", 8, E_VariableType::Semicontinuous, 0.0, 0.10, 0.04);
    auto sc11 = std::make_shared<Variable>("sc11", 9, E_VariableType::Semicontinuous, 0.0, 0.07, 0.04);
    auto sc12 = std::make_shared<Variable>("sc12", 10, E_VariableType::Semicontinuous, 0.0, 0.11, 0.03);
    auto sc13 = std::make_shared<Variable>("sc13", 11, E_VariableType::Semicontinuous, 0.0, 0.20, 0.03);
    auto sc14 = std::make_shared<Variable>("sc14", 12, E_VariableType::Semicontinuous, 0.0, 0.10, 0.03);
    auto sc15 = std::make_shared<Variable>("sc15", 13, E_VariableType::Semicontinuous, 0.0, 0.10, 0.03);
    auto sc16 = std::make_shared<Variable>("sc16", 14, E_VariableType::Semicontinuous, 0.0, 0.20, 0.02);
    auto sc17 = std::make_shared<Variable>("sc17", 15, E_VariableType::Semicontinuous, 0.0, 0.15, 0.02);
    auto sc18 = std::make_shared<Variable>("sc18", 16, E_VariableType::Real, 0.0, 0.0);
    auto sc19 = std::make_shared<Variable>("sc19", 17, E_VariableType::Real, 0.0, 0.0);
    auto sc20 = std::make_shared<Variable>("sc20", 18, E_VariableType::Semicontinuous, 0.0, 0.10, 0.04);
    auto sc21 = std::make_shared<Variable>("sc21", 19, E_VariableType::Semicontinuous, 0.0, 0.15, 0.04);
    auto sc22 = std::make_shared<Variable>("sc22", 20, E_VariableType::Semicontinuous, 0.0, 0.20, 0.04);

    auto b23 = std::make_shared<Variable>("b23", 21, E_VariableType::Binary);
    auto b24 = std::make_shared<Variable>("b24", 22, E_VariableType::Binary);
    auto b25 = std::make_shared<Variable>("b25", 23, E_VariableType::Binary);
    auto b26 = std::make_shared<Variable>("b26", 24, E_VariableType::Binary);
    auto b27 = std::make_shared<Variable>("b27", 25, E_VariableType::Binary);
    auto b28 = std::make_shared<Variable>("b28", 26, E_VariableType::Binary);
    auto b29 = std::make_shared<Variable>("b29", 27, E_VariableType::Binary);
    auto b30 = std::make_shared<Variable>("b30", 28, E_VariableType::Binary);
    auto b31 = std::make_shared<Variable>("b31", 29, E_VariableType::Binary);
    auto b32 = std::make_shared<Variable>("b32", 30, E_VariableType::Binary);
    auto b33 = std::make_shared<Variable>("b33", 31, E_VariableType::Binary);
    auto b34 = std::make_shared<Variable>("b34", 32, E_VariableType::Binary);
    auto b35 = std::make_shared<Variable>("b35", 33, E_VariableType::Binary);
    auto b36 = std::make_shared<Variable>("b36", 34, E_VariableType::Binary);

    problem->add({ x2, x3, x4, x5, x6, x7, x8, sc9, sc10, sc11, sc12, sc13, sc14, sc15, sc16, sc17, sc18, sc19, sc20,
        sc21, sc22, b23, b24, b25, b26, b27, b28, b29, b30, b31, b32, b33, b34, b35, b36 });

    auto objective = std::make_shared<QuadraticObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    objective->add(std::make_shared<QuadraticTerm>(42.18, x2, x2));
    objective->add(std::make_shared<QuadraticTerm>(40.36, x2, x3));
    objective->add(std::make_shared<QuadraticTerm>(21.76, x2, x4));
    objective->add(std::make_shared<QuadraticTerm>(10.6, x2, x5));
    objective->add(std::make_shared<QuadraticTerm>(24.64, x2, x6));
    objective->add(std::make_shared<QuadraticTerm>(47.68, x2, x7));
    objective->add(std::make_shared<QuadraticTerm>(34.82, x2, x8));
    objective->add(std::make_shared<QuadraticTerm>(70.89, x3, x3));
    objective->add(std::make_shared<QuadraticTerm>(43.16, x3, x4));
    objective->add(std::make_shared<QuadraticTerm>(30.82, x3, x5));
    objective->add(std::make_shared<QuadraticTerm>(46.48, x3, x6));
    objective->add(std::make_shared<QuadraticTerm>(47.6, x3, x7));
    objective->add(std::make_shared<QuadraticTerm>(25.24, x3, x8));
    objective->add(std::make_shared<QuadraticTerm>(25.51, x4, x4));
    objective->add(std::make_shared<QuadraticTerm>(19.2, x4, x5));
    objective->add(std::make_shared<QuadraticTerm>(45.26, x4, x6));
    objective->add(std::make_shared<QuadraticTerm>(26.44, x4, x7));
    objective->add(std::make_shared<QuadraticTerm>(9.4, x4, x8));
    objective->add(std::make_shared<QuadraticTerm>(22.33, x5, x5));
    objective->add(std::make_shared<QuadraticTerm>(20.64, x5, x6));
    objective->add(std::make_shared<QuadraticTerm>(20.92, x5, x7));
    objective->add(std::make_shared<QuadraticTerm>(2.0, x5, x8));
    objective->add(std::make_shared<QuadraticTerm>(30.01, x6, x6));
    objective->add(std::make_shared<QuadraticTerm>(32.72, x6, x7));
    objective->add(std::make_shared<QuadraticTerm>(14.4, x6, x8));
    objective->add(std::make_shared<QuadraticTerm>(42.23, x7, x7));
    objective->add(std::make_shared<QuadraticTerm>(19.8, x7, x8));
    objective->add(std::make_shared<QuadraticTerm>(16.42, x8, x8));
    objective->add(std::make_shared<LinearTerm>(-0.06435, x2));
    objective->add(std::make_shared<LinearTerm>(-0.0548, x3));
    objective->add(std::make_shared<LinearTerm>(-0.02505, x4));
    objective->add(std::make_shared<LinearTerm>(-0.0762, x5));
    objective->add(std::make_shared<LinearTerm>(-0.03815, x6));
    objective->add(std::make_shared<LinearTerm>(-0.0927, x7));
    objective->add(std::make_shared<LinearTerm>(-0.031, x8));
    problem->add(objective);

    auto ce1 = std::make_shared<LinearConstraint>(0, "e1", 1.0, 1.0);
    for(auto& v : { x2, x3, x4, x5, x6, x7, x8 })
        ce1->add(std::make_shared<LinearTerm>(1.0, v));
    problem->add(ce1);

    auto makeBalance
        = [&](int idx, const std::string& name, VariablePtr xi, VariablePtr scs, VariablePtr scb, double rhs)
    {
        auto c = std::make_shared<LinearConstraint>(idx, name, rhs, rhs);
        c->add(std::make_shared<LinearTerm>(1.0, xi));
        c->add(std::make_shared<LinearTerm>(-1.0, scs));
        c->add(std::make_shared<LinearTerm>(1.0, scb));
        problem->add(c);
    };
    makeBalance(1, "e2", x2, sc9, sc16, 0.2);
    makeBalance(2, "e3", x3, sc10, sc17, 0.2);
    makeBalance(3, "e4", x4, sc11, sc18, 0.0);
    makeBalance(4, "e5", x5, sc12, sc19, 0.0);
    makeBalance(5, "e6", x6, sc13, sc20, 0.2);
    makeBalance(6, "e7", x7, sc14, sc21, 0.2);
    makeBalance(7, "e8", x8, sc15, sc22, 0.2);

    auto ce9 = std::make_shared<LinearConstraint>(8, "e9", SHOT_DBL_MIN, 0.3);
    for(auto& v : { sc9, sc10, sc11, sc12, sc13, sc14, sc15 })
        ce9->add(std::make_shared<LinearTerm>(1.0, v));
    problem->add(ce9);

    struct BM
    {
        int idx;
        std::string name;
        VariablePtr scv;
        VariablePtr bv;
        double coeff;
    };
    std::vector<BM> bmData = {
        { 9, "e10", sc9, b23, 0.11 },
        { 10, "e11", sc10, b24, 0.10 },
        { 11, "e12", sc11, b25, 0.07 },
        { 12, "e13", sc12, b26, 0.11 },
        { 13, "e14", sc13, b27, 0.20 },
        { 14, "e15", sc14, b28, 0.10 },
        { 15, "e16", sc15, b29, 0.10 },
        { 16, "e17", sc16, b30, 0.20 },
        { 17, "e18", sc17, b31, 0.15 },
        { 18, "e19", sc18, nullptr, 0.0 },
        { 19, "e20", sc19, nullptr, 0.0 },
        { 20, "e21", sc20, b34, 0.10 },
        { 21, "e22", sc21, b35, 0.15 },
        { 22, "e23", sc22, b36, 0.20 },
    };
    for(auto& b : bmData)
    {
        auto c = std::make_shared<LinearConstraint>(b.idx, b.name, SHOT_DBL_MIN, 0.0);
        c->add(std::make_shared<LinearTerm>(1.0, b.scv));
        if(b.bv)
            c->add(std::make_shared<LinearTerm>(-b.coeff, b.bv));
        problem->add(c);
    }

    struct BP
    {
        int idx;
        std::string name;
        VariablePtr b1;
        VariablePtr b2;
    };
    std::vector<BP> bpData = {
        { 23, "e24", b23, b30 },
        { 24, "e25", b24, b31 },
        { 25, "e26", b25, b32 },
        { 26, "e27", b26, b33 },
        { 27, "e28", b27, b34 },
        { 28, "e29", b28, b35 },
        { 29, "e30", b29, b36 },
    };
    for(auto& p : bpData)
    {
        auto c = std::make_shared<LinearConstraint>(p.idx, p.name, SHOT_DBL_MIN, 1.0);
        c->add(std::make_shared<LinearTerm>(1.0, p.b1));
        c->add(std::make_shared<LinearTerm>(1.0, p.b2));
        problem->add(c);
    }

    problem->finalize();
    solver->setProblem(problem);

    std::cout << "\nSolving...\n";

    if(!solver->solveProblem())
    {
        std::cout << "Failed to solve problem!\n";
        passed = false;
    }
    else
    {
        auto solutions = env->results->primalSolutions;
        if(solutions.empty())
        {
            std::cout << "No solution found!\n";
            passed = false;
        }
        else
        {
            double objValue = solutions[0].objValue;
            double expectedObj = 14.36923211;
            std::cout << "\nSolution found:\n";
            std::cout << "  Objective value: " << objValue << "\n";
            std::cout << "  Expected value:  " << expectedObj << "\n";
            if(std::abs(objValue - expectedObj) < 0.01)
                std::cout << "\n*** TEST PASSED: Objective matches expected value! ***\n";
            else
            {
                std::cout << "\n*** TEST FAILED: Objective differs from expected! ***\n";
                std::cout << "  Difference: " << std::abs(objValue - expectedObj) << "\n";
                passed = false;
            }
        }
    }

    return passed;
}

bool ModelTestSemiContinuous()
{
    bool passed = true;
#ifdef HAS_CBC
    passed = ModelTestMeanvarxscWithSolver(ES_MIPSolver::Cbc) && passed;
#endif
    // TODO: HiGHS semi-continuous support needs further investigation
#ifdef HAS_HIGHS
    passed = ModelTestMeanvarxscWithSolver(ES_MIPSolver::Highs) && passed;
#endif
#ifdef HAS_CPLEX
    passed = ModelTestMeanvarxscWithSolver(ES_MIPSolver::Cplex) && passed;
#endif
#ifdef HAS_GUROBI
    passed = ModelTestMeanvarxscWithSolver(ES_MIPSolver::Gurobi) && passed;
#endif
    return passed;
}

bool ModelTestSOS1WithSolver(ES_MIPSolver mipSolver)
{
    bool passed = true;

    std::string solverName;
    switch(mipSolver)
    {
    case ES_MIPSolver::Cbc:
        solverName = "CBC";
        break;
    case ES_MIPSolver::Highs:
        solverName = "HiGHS";
        break;
    case ES_MIPSolver::Cplex:
        solverName = "CPLEX";
        break;
    case ES_MIPSolver::Gurobi:
        solverName = "Gurobi";
        break;
    default:
        solverName = "unknown";
        break;
    }

    std::cout << "\n=== Testing SOS1 (sos1a) with " << solverName << " ===\n\n";

    // Simple capacitated transportation problem:
    // maximize  0.9*x1 + 1.0*x2 + 1.1*x3
    // s.t.      x1 + x2 + x3 <= 1
    //           x1 in [0, 0.8], x2 in [0, 0.6], x3 in [0, 0.6]
    //           SOS1: {x1, x2, x3}
    // Optimal:  x1 = 0.8, obj = 0.72

    auto solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(mipSolver));

    auto problem = std::make_shared<Problem>(env);
    problem->name = "sos1a";

    auto x1 = std::make_shared<Variable>("x1", 0, E_VariableType::Real, 0.0, 0.8);
    auto x2 = std::make_shared<Variable>("x2", 1, E_VariableType::Real, 0.0, 0.6);
    auto x3 = std::make_shared<Variable>("x3", 2, E_VariableType::Real, 0.0, 0.6);
    problem->add({ x1, x2, x3 });

    auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Maximize);
    objective->add(std::make_shared<LinearTerm>(0.9, x1));
    objective->add(std::make_shared<LinearTerm>(1.0, x2));
    objective->add(std::make_shared<LinearTerm>(1.1, x3));
    problem->add(objective);

    auto c1 = std::make_shared<LinearConstraint>(0, "xsum", SHOT_DBL_MIN, 1.0);
    c1->add(std::make_shared<LinearTerm>(1.0, x1));
    c1->add(std::make_shared<LinearTerm>(1.0, x2));
    c1->add(std::make_shared<LinearTerm>(1.0, x3));
    problem->add(c1);

    auto sos1
        = std::make_shared<SpecialOrderedSet>(E_SOSType::One, Variables { x1, x2, x3 }, VectorDouble { 1.0, 2.0, 3.0 });
    problem->add(sos1);

    problem->finalize();
    solver->setProblem(problem);

    std::cout << "\nSolving...\n";

    if(!solver->solveProblem())
    {
        std::cout << "Failed to solve problem!\n";
        passed = false;
    }
    else
    {
        auto solutions = env->results->primalSolutions;
        if(solutions.empty())
        {
            std::cout << "No solution found!\n";
            passed = false;
        }
        else
        {
            double objValue = solutions[0].objValue;
            double expectedObj = 0.72;
            std::cout << "\nSolution found:\n";
            std::cout << "  Objective value: " << objValue << "\n";
            std::cout << "  Expected value:  " << expectedObj << "\n";
            if(std::abs(objValue - expectedObj) < 0.01)
                std::cout << "\n*** TEST PASSED: Objective matches expected value! ***\n";
            else
            {
                std::cout << "\n*** TEST FAILED: Objective differs from expected! ***\n";
                std::cout << "  Difference: " << std::abs(objValue - expectedObj) << "\n";
                passed = false;
            }
        }
    }

    return passed;
}

bool ModelTestSOS1()
{
    bool passed = true;
#ifdef HAS_CBC
    passed = ModelTestSOS1WithSolver(ES_MIPSolver::Cbc) && passed;
#endif
#ifdef HAS_HIGHS
    try
    {
        passed = ModelTestSOS1WithSolver(ES_MIPSolver::Highs) && passed;
    }
    catch(OperationNotImplementedException*)
    {
        std::cout << "   HiGHS does not support SOS — skipping.\n";
    }
#endif
#ifdef HAS_CPLEX
    passed = ModelTestSOS1WithSolver(ES_MIPSolver::Cplex) && passed;
#endif
#ifdef HAS_GUROBI
    passed = ModelTestSOS1WithSolver(ES_MIPSolver::Gurobi) && passed;
#endif
    return passed;
}

bool ModelTestSOS2WithSolver(ES_MIPSolver mipSolver)
{
    bool passed = true;

    std::string solverName;
    switch(mipSolver)
    {
    case ES_MIPSolver::Cbc:
        solverName = "CBC";
        break;
    case ES_MIPSolver::Highs:
        solverName = "HiGHS";
        break;
    case ES_MIPSolver::Cplex:
        solverName = "CPLEX";
        break;
    case ES_MIPSolver::Gurobi:
        solverName = "Gurobi";
        break;
    default:
        solverName = "unknown";
        break;
    }

    std::cout << "\n=== Testing SOS2 (sos2a) with " << solverName << " ===\n\n";

    // Linear interpolation problem:
    // minimize  fplus + fminus
    // s.t.      w1 + w2 + w3 = 1
    //           fplus  >= (w1 + 2*w2 + 3*w3) - 1.3   (fplus  >= fx - Fbar)
    //           fminus >= 1.3 - (w1 + 2*w2 + 3*w3)   (fminus >= Fbar - fx)
    //           w1, w2, w3 >= 0
    //           fplus, fminus >= 0
    //           SOS2: {w1, w2, w3}
    // Optimal:  w1 = 0.7, w2 = 0.3, fx = 1.3, obj = 0.0

    auto solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(mipSolver));

    auto problem = std::make_shared<Problem>(env);
    problem->name = "sos2a";

    auto w1 = std::make_shared<Variable>("w1", 0, E_VariableType::Real, 0.0, SHOT_DBL_MAX);
    auto w2 = std::make_shared<Variable>("w2", 1, E_VariableType::Real, 0.0, SHOT_DBL_MAX);
    auto w3 = std::make_shared<Variable>("w3", 2, E_VariableType::Real, 0.0, SHOT_DBL_MAX);
    auto fplus = std::make_shared<Variable>("fplus", 3, E_VariableType::Real, 0.0, SHOT_DBL_MAX);
    auto fminus = std::make_shared<Variable>("fminus", 4, E_VariableType::Real, 0.0, SHOT_DBL_MAX);
    problem->add({ w1, w2, w3, fplus, fminus });

    auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
    objective->add(std::make_shared<LinearTerm>(1.0, fplus));
    objective->add(std::make_shared<LinearTerm>(1.0, fminus));
    problem->add(objective);

    // w1 + w2 + w3 = 1
    auto cwsum = std::make_shared<LinearConstraint>(0, "wsum", 1.0, 1.0);
    cwsum->add(std::make_shared<LinearTerm>(1.0, w1));
    cwsum->add(std::make_shared<LinearTerm>(1.0, w2));
    cwsum->add(std::make_shared<LinearTerm>(1.0, w3));
    problem->add(cwsum);

    // fplus - w1 - 2*w2 - 3*w3 >= -1.3  (fplus >= fx - 1.3)
    auto cgapplus = std::make_shared<LinearConstraint>(1, "gapplus", -1.3, SHOT_DBL_MAX);
    cgapplus->add(std::make_shared<LinearTerm>(1.0, fplus));
    cgapplus->add(std::make_shared<LinearTerm>(-1.0, w1));
    cgapplus->add(std::make_shared<LinearTerm>(-2.0, w2));
    cgapplus->add(std::make_shared<LinearTerm>(-3.0, w3));
    problem->add(cgapplus);

    // fminus + w1 + 2*w2 + 3*w3 >= 1.3  (fminus >= 1.3 - fx)
    auto cgapminus = std::make_shared<LinearConstraint>(2, "gapminus", 1.3, SHOT_DBL_MAX);
    cgapminus->add(std::make_shared<LinearTerm>(1.0, fminus));
    cgapminus->add(std::make_shared<LinearTerm>(1.0, w1));
    cgapminus->add(std::make_shared<LinearTerm>(2.0, w2));
    cgapminus->add(std::make_shared<LinearTerm>(3.0, w3));
    problem->add(cgapminus);

    auto sos2
        = std::make_shared<SpecialOrderedSet>(E_SOSType::Two, Variables { w1, w2, w3 }, VectorDouble { 1.0, 2.0, 3.0 });
    problem->add(sos2);

    problem->finalize();
    solver->setProblem(problem);

    std::cout << "\nSolving...\n";

    if(!solver->solveProblem())
    {
        std::cout << "Failed to solve problem!\n";
        passed = false;
    }
    else
    {
        auto solutions = env->results->primalSolutions;
        if(solutions.empty())
        {
            std::cout << "No solution found!\n";
            passed = false;
        }
        else
        {
            double objValue = solutions[0].objValue;
            double expectedObj = 0.0;
            std::cout << "\nSolution found:\n";
            std::cout << "  Objective value: " << objValue << "\n";
            std::cout << "  Expected value:  " << expectedObj << "\n";
            if(std::abs(objValue - expectedObj) < 0.01)
                std::cout << "\n*** TEST PASSED: Objective matches expected value! ***\n";
            else
            {
                std::cout << "\n*** TEST FAILED: Objective differs from expected! ***\n";
                std::cout << "  Difference: " << std::abs(objValue - expectedObj) << "\n";
                passed = false;
            }
        }
    }

    return passed;
}

bool ModelTestSOS2()
{
    bool passed = true;
#ifdef HAS_CBC
    passed = ModelTestSOS2WithSolver(ES_MIPSolver::Cbc) && passed;
#endif
#ifdef HAS_HIGHS
    try
    {
        passed = ModelTestSOS2WithSolver(ES_MIPSolver::Highs) && passed;
    }
    catch(OperationNotImplementedException*)
    {
        std::cout << "   HiGHS does not support SOS — skipping.\n";
    }
#endif
#ifdef HAS_CPLEX
    passed = ModelTestSOS2WithSolver(ES_MIPSolver::Cplex) && passed;
#endif
#ifdef HAS_GUROBI
    passed = ModelTestSOS2WithSolver(ES_MIPSolver::Gurobi) && passed;
#endif
    return passed;
}

bool ModelTestGradientsAndHessians()
{
    // Test gradient and Hessian calculations for various expression types
    // This verifies that CppAD is working correctly for nonlinear expressions

    bool passed = true;
    const double tolerance = 1e-8;

    std::cout << "\n=== Testing Gradient and Hessian Calculations ===\n\n";

    // Initializing the SHOT solver class
    auto solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Info));

    // ========== Test 1: Exponential exp(x) ==========
    {
        std::cout << "Test 1: Gradient and Hessian of exp(x)\n";

        auto problem = std::make_shared<Problem>(env);
        problem->name = "exp_test";

        auto x = std::make_shared<Variable>("x", 0, E_VariableType::Real, -10.0, 10.0);
        problem->add(x);

        auto nl_x = std::make_shared<ExpressionVariable>(x);
        auto expr_exp = std::make_shared<ExpressionExp>(nl_x);

        auto objective = std::make_shared<NonlinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        objective->add(expr_exp);
        problem->add(objective);

        problem->finalize();

        VectorDouble point = { 1.0 };
        double expected_gradient = std::exp(1.0); // d/dx exp(x) = exp(x) = e
        double expected_hessian = std::exp(1.0); // d^2/dx^2 exp(x) = exp(x) = e

        auto gradient = objective->calculateGradient(point, true);
        auto hessian = objective->calculateHessian(point, true);

        std::cout << "  At x=1: gradient = " << gradient[x] << " (expected: " << expected_gradient << ")\n";

        if(std::abs(gradient[x] - expected_gradient) > tolerance)
        {
            std::cout << "  FAILED: Gradient mismatch!\n";
            passed = false;
        }

        auto key_xx = std::make_pair(x, x);
        if(hessian.find(key_xx) != hessian.end())
        {
            std::cout << "  At x=1: hessian[x,x] = " << hessian[key_xx] << " (expected: " << expected_hessian << ")\n";
            if(std::abs(hessian[key_xx] - expected_hessian) > tolerance)
            {
                std::cout << "  FAILED: Hessian mismatch!\n";
                passed = false;
            }
        }
        else
        {
            std::cout << "  FAILED: Hessian entry not found!\n";
            passed = false;
        }

        if(passed)
            std::cout << "  PASSED\n";
    }

    // ========== Test 2: Logarithm log(x) ==========
    {
        std::cout << "\nTest 2: Gradient and Hessian of log(x)\n";

        auto problem = std::make_shared<Problem>(env);
        problem->name = "log_test";

        auto x = std::make_shared<Variable>("x", 0, E_VariableType::Real, 0.1, 10.0);
        problem->add(x);

        auto nl_x = std::make_shared<ExpressionVariable>(x);
        auto expr_log = std::make_shared<ExpressionLog>(nl_x);

        auto objective = std::make_shared<NonlinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        objective->add(expr_log);
        problem->add(objective);

        problem->finalize();

        VectorDouble point = { 2.0 };
        double expected_gradient = 1.0 / 2.0; // d/dx log(x) = 1/x = 0.5
        double expected_hessian = -1.0 / 4.0; // d^2/dx^2 log(x) = -1/x^2 = -0.25

        auto gradient = objective->calculateGradient(point, true);
        auto hessian = objective->calculateHessian(point, true);

        std::cout << "  At x=2: gradient = " << gradient[x] << " (expected: " << expected_gradient << ")\n";

        if(std::abs(gradient[x] - expected_gradient) > tolerance)
        {
            std::cout << "  FAILED: Gradient mismatch!\n";
            passed = false;
        }

        auto key_xx = std::make_pair(x, x);
        if(hessian.find(key_xx) != hessian.end())
        {
            std::cout << "  At x=2: hessian[x,x] = " << hessian[key_xx] << " (expected: " << expected_hessian << ")\n";
            if(std::abs(hessian[key_xx] - expected_hessian) > tolerance)
            {
                std::cout << "  FAILED: Hessian mismatch!\n";
                passed = false;
            }
        }
        else
        {
            std::cout << "  FAILED: Hessian entry not found!\n";
            passed = false;
        }

        if(passed)
            std::cout << "  PASSED\n";
    }

    // ========== Test 3: Sine and Cosine sin(x) + cos(y) ==========
    {
        std::cout << "\nTest 3: Gradient and Hessian of sin(x) + cos(y)\n";

        auto problem = std::make_shared<Problem>(env);
        problem->name = "sincos_test";

        auto x = std::make_shared<Variable>("x", 0, E_VariableType::Real, -10.0, 10.0);
        auto y = std::make_shared<Variable>("y", 1, E_VariableType::Real, -10.0, 10.0);
        problem->add(x);
        problem->add(y);

        auto nl_x = std::make_shared<ExpressionVariable>(x);
        auto nl_y = std::make_shared<ExpressionVariable>(y);
        auto expr_sin = std::make_shared<ExpressionSin>(nl_x);
        auto expr_cos = std::make_shared<ExpressionCos>(nl_y);
        auto expr_sum = std::make_shared<ExpressionSum>(expr_sin, expr_cos);

        auto objective = std::make_shared<NonlinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        objective->add(expr_sum);
        problem->add(objective);

        problem->finalize();

        VectorDouble point = { 0.0, 0.0 };
        double expected_grad_x = std::cos(0.0); // d/dx sin(x) = cos(x) = 1
        double expected_grad_y = -std::sin(0.0); // d/dy cos(y) = -sin(y) = 0
        double expected_hess_yy = -std::cos(0.0); // d^2/dy^2 cos(y) = -cos(y) = -1

        auto gradient = objective->calculateGradient(point, true);
        auto hessian = objective->calculateHessian(point, true);

        std::cout << "  At (0,0): gradient = [" << gradient[x] << ", " << gradient[y] << "]";
        std::cout << " (expected: [" << expected_grad_x << ", " << expected_grad_y << "])\n";

        if(std::abs(gradient[x] - expected_grad_x) > tolerance || std::abs(gradient[y] - expected_grad_y) > tolerance)
        {
            std::cout << "  FAILED: Gradient mismatch!\n";
            passed = false;
        }

        // Check Hessian (only non-zero elements are expected)
        auto key_yy = std::make_pair(y, y);
        if(hessian.find(key_yy) != hessian.end())
        {
            std::cout << "  hessian[y,y] = " << hessian[key_yy] << " (expected: " << expected_hess_yy << ")\n";
            if(std::abs(hessian[key_yy] - expected_hess_yy) > tolerance)
            {
                std::cout << "  FAILED: Hessian[y,y] mismatch!\n";
                passed = false;
            }
        }
        else
        {
            std::cout << "  FAILED: Hessian[y,y] not found!\n";
            passed = false;
        }

        if(passed)
            std::cout << "  PASSED\n";
    }

    // ========== Test 4: Power x^3 ==========
    {
        std::cout << "\nTest 4: Gradient and Hessian of x^3\n";

        auto problem = std::make_shared<Problem>(env);
        problem->name = "power_test";

        auto x = std::make_shared<Variable>("x", 0, E_VariableType::Real, 0.1, 10.0);
        problem->add(x);

        auto nl_x = std::make_shared<ExpressionVariable>(x);
        auto expr_const = std::make_shared<ExpressionConstant>(3.0);
        auto expr_power = std::make_shared<ExpressionPower>(nl_x, expr_const);

        auto objective = std::make_shared<NonlinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        objective->add(expr_power);
        problem->add(objective);

        problem->finalize();

        VectorDouble point = { 2.0 };
        double expected_gradient = 3.0 * std::pow(2.0, 2.0); // d/dx x^3 = 3*x^2 = 12
        double expected_hessian = 6.0 * 2.0; // d^2/dx^2 x^3 = 6*x = 12

        auto gradient = objective->calculateGradient(point, true);
        auto hessian = objective->calculateHessian(point, true);

        std::cout << "  At x=2: gradient = " << gradient[x] << " (expected: " << expected_gradient << ")\n";

        if(std::abs(gradient[x] - expected_gradient) > tolerance)
        {
            std::cout << "  FAILED: Gradient mismatch!\n";
            passed = false;
        }

        auto key_xx = std::make_pair(x, x);
        if(hessian.find(key_xx) != hessian.end())
        {
            std::cout << "  At x=2: hessian[x,x] = " << hessian[key_xx] << " (expected: " << expected_hessian << ")\n";
            if(std::abs(hessian[key_xx] - expected_hessian) > tolerance)
            {
                std::cout << "  FAILED: Hessian mismatch!\n";
                passed = false;
            }
        }
        else
        {
            std::cout << "  FAILED: Hessian entry not found!\n";
            passed = false;
        }

        if(passed)
            std::cout << "  PASSED\n";
    }

    // ========== Test 5: Quadratic x^2 + 2*x*y + y^2 ==========
    {
        std::cout << "\nTest 5: Gradient and Hessian of x^2 + 2*x*y + y^2\n";

        auto problem = std::make_shared<Problem>(env);
        problem->name = "quadratic_test";

        auto x = std::make_shared<Variable>("x", 0, E_VariableType::Real, -10.0, 10.0);
        auto y = std::make_shared<Variable>("y", 1, E_VariableType::Real, -10.0, 10.0);
        problem->add(x);
        problem->add(y);

        QuadraticTerms quadTerms;
        quadTerms.add(std::make_shared<QuadraticTerm>(1.0, x, x)); // x^2
        quadTerms.add(std::make_shared<QuadraticTerm>(2.0, x, y)); // 2*x*y
        quadTerms.add(std::make_shared<QuadraticTerm>(1.0, y, y)); // y^2

        auto objective = std::make_shared<QuadraticObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        objective->add(quadTerms);
        problem->add(objective);

        problem->finalize();

        VectorDouble point = { 1.0, 2.0 };
        // d/dx = 2*x + 2*y = 2 + 4 = 6
        // d/dy = 2*x + 2*y = 2 + 4 = 6
        double expected_grad_x = 6.0;
        double expected_grad_y = 6.0;
        // d^2/dx^2 = 2, d^2/dy^2 = 2, d^2/dxdy = 2
        double expected_hess_xx = 2.0;
        double expected_hess_yy = 2.0;
        double expected_hess_xy = 2.0;

        auto gradient = objective->calculateGradient(point, true);
        auto hessian = objective->calculateHessian(point, true);

        std::cout << "  At (1,2): gradient = [" << gradient[x] << ", " << gradient[y] << "]";
        std::cout << " (expected: [" << expected_grad_x << ", " << expected_grad_y << "])\n";

        if(std::abs(gradient[x] - expected_grad_x) > tolerance || std::abs(gradient[y] - expected_grad_y) > tolerance)
        {
            std::cout << "  FAILED: Gradient mismatch!\n";
            passed = false;
        }

        auto key_xx = std::make_pair(x, x);
        auto key_yy = std::make_pair(y, y);
        auto key_xy = std::make_pair(x, y);

        std::cout << "  Hessian: [[" << hessian[key_xx] << ", " << hessian[key_xy] << "], [";
        std::cout << hessian[key_xy] << ", " << hessian[key_yy] << "]]\n";
        std::cout << "  Expected: [[2, 2], [2, 2]]\n";

        if(std::abs(hessian[key_xx] - expected_hess_xx) > tolerance
            || std::abs(hessian[key_yy] - expected_hess_yy) > tolerance
            || std::abs(hessian[key_xy] - expected_hess_xy) > tolerance)
        {
            std::cout << "  FAILED: Hessian mismatch!\n";
            passed = false;
        }

        if(passed)
            std::cout << "  PASSED\n";
    }

    // ========== Test 6: Square root sqrt(x) ==========
    {
        std::cout << "\nTest 6: Gradient and Hessian of sqrt(x)\n";

        auto problem = std::make_shared<Problem>(env);
        problem->name = "sqrt_test";

        auto x = std::make_shared<Variable>("x", 0, E_VariableType::Real, 0.1, 10.0);
        problem->add(x);

        auto nl_x = std::make_shared<ExpressionVariable>(x);
        auto expr_sqrt = std::make_shared<ExpressionSquareRoot>(nl_x);

        auto objective = std::make_shared<NonlinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        objective->add(expr_sqrt);
        problem->add(objective);

        problem->finalize();

        VectorDouble point = { 4.0 };
        double expected_gradient = 0.5 / std::sqrt(4.0); // d/dx sqrt(x) = 1/(2*sqrt(x)) = 0.25
        double expected_hessian = -0.25 / std::pow(4.0, 1.5); // d^2/dx^2 sqrt(x) = -1/(4*x^1.5) = -1/32

        auto gradient = objective->calculateGradient(point, true);
        auto hessian = objective->calculateHessian(point, true);

        std::cout << "  At x=4: gradient = " << gradient[x] << " (expected: " << expected_gradient << ")\n";

        if(std::abs(gradient[x] - expected_gradient) > tolerance)
        {
            std::cout << "  FAILED: Gradient mismatch!\n";
            passed = false;
        }

        auto key_xx = std::make_pair(x, x);
        if(hessian.find(key_xx) != hessian.end())
        {
            std::cout << "  At x=4: hessian[x,x] = " << hessian[key_xx] << " (expected: " << expected_hessian << ")\n";
            if(std::abs(hessian[key_xx] - expected_hessian) > tolerance)
            {
                std::cout << "  FAILED: Hessian mismatch!\n";
                passed = false;
            }
        }
        else
        {
            std::cout << "  FAILED: Hessian entry not found!\n";
            passed = false;
        }

        if(passed)
            std::cout << "  PASSED\n";
    }

    // ========== Test 7: Composite exp(x) * sin(y) ==========
    {
        std::cout << "\nTest 7: Gradient and Hessian of exp(x) * sin(y)\n";

        auto problem = std::make_shared<Problem>(env);
        problem->name = "composite_test";

        auto x = std::make_shared<Variable>("x", 0, E_VariableType::Real, -10.0, 10.0);
        auto y = std::make_shared<Variable>("y", 1, E_VariableType::Real, -10.0, 10.0);
        problem->add(x);
        problem->add(y);

        auto nl_x = std::make_shared<ExpressionVariable>(x);
        auto nl_y = std::make_shared<ExpressionVariable>(y);
        auto expr_exp = std::make_shared<ExpressionExp>(nl_x);
        auto expr_sin = std::make_shared<ExpressionSin>(nl_y);
        auto expr_product = std::make_shared<ExpressionProduct>(expr_exp, expr_sin);

        auto objective = std::make_shared<NonlinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        objective->add(expr_product);
        problem->add(objective);

        problem->finalize();

        double pi_2 = M_PI / 2.0;
        VectorDouble point = { 0.0, pi_2 };
        // f = exp(x) * sin(y)
        // df/dx = exp(x) * sin(y) = 1 * 1 = 1
        // df/dy = exp(x) * cos(y) = 1 * 0 = 0
        double expected_grad_x = std::exp(0.0) * std::sin(pi_2); // 1 * 1 = 1
        double expected_grad_y = std::exp(0.0) * std::cos(pi_2); // 1 * 0 = 0

        auto gradient = objective->calculateGradient(point, true);
        auto hessian = objective->calculateHessian(point, true);

        std::cout << "  At (0, pi/2): gradient = [" << gradient[x] << ", " << gradient[y] << "]";
        std::cout << " (expected: [" << expected_grad_x << ", " << expected_grad_y << "])\n";

        if(std::abs(gradient[x] - expected_grad_x) > tolerance || std::abs(gradient[y] - expected_grad_y) > tolerance)
        {
            std::cout << "  FAILED: Gradient mismatch!\n";
            passed = false;
        }

        // d^2f/dx^2 = exp(x) * sin(y) = 1
        // d^2f/dy^2 = -exp(x) * sin(y) = -1
        // d^2f/dxdy = exp(x) * cos(y) = 0
        double expected_hess_xx = 1.0;
        double expected_hess_yy = -1.0;

        auto key_xx = std::make_pair(x, x);
        auto key_yy = std::make_pair(y, y);

        if(hessian.find(key_xx) != hessian.end())
            std::cout << "  Hessian[x,x] = " << hessian[key_xx] << " (expected: " << expected_hess_xx << ")\n";
        if(hessian.find(key_yy) != hessian.end())
            std::cout << "  Hessian[y,y] = " << hessian[key_yy] << " (expected: " << expected_hess_yy << ")\n";

        if(hessian.find(key_xx) == hessian.end() || std::abs(hessian[key_xx] - expected_hess_xx) > tolerance
            || hessian.find(key_yy) == hessian.end() || std::abs(hessian[key_yy] - expected_hess_yy) > tolerance)
        {
            std::cout << "  FAILED: Hessian mismatch!\n";
            passed = false;
        }

        if(passed)
            std::cout << "  PASSED\n";
    }

    // ========== Test 8: Nonlinear constraint log(x) + exp(y) <= 10 ==========
    {
        std::cout << "\nTest 8: Gradient and Hessian of constraint log(x) + exp(y)\n";

        auto problem = std::make_shared<Problem>(env);
        problem->name = "constraint_test";

        auto x = std::make_shared<Variable>("x", 0, E_VariableType::Real, 0.1, 10.0);
        auto y = std::make_shared<Variable>("y", 1, E_VariableType::Real, -10.0, 10.0);
        problem->add(x);
        problem->add(y);

        auto nl_x = std::make_shared<ExpressionVariable>(x);
        auto nl_y = std::make_shared<ExpressionVariable>(y);
        auto expr_log = std::make_shared<ExpressionLog>(nl_x);
        auto expr_exp = std::make_shared<ExpressionExp>(nl_y);
        auto expr_sum = std::make_shared<ExpressionSum>(expr_log, expr_exp);

        auto constraint = std::make_shared<NonlinearConstraint>(0, "nl_constr", expr_sum, SHOT_DBL_MIN, 10.0);
        problem->add(constraint);

        // Need an objective
        LinearTerms objTerms;
        objTerms.add(std::make_shared<LinearTerm>(1.0, x));
        auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        objective->add(objTerms);
        problem->add(objective);

        problem->finalize();

        VectorDouble point = { 2.0, 0.0 };
        // d/dx = 1/x = 0.5
        // d/dy = exp(y) = 1
        double expected_grad_x = 0.5;
        double expected_grad_y = 1.0;

        auto gradient = constraint->calculateGradient(point, true);
        auto hessian = constraint->calculateHessian(point, true);

        std::cout << "  At (2, 0): gradient = [" << gradient[x] << ", " << gradient[y] << "]";
        std::cout << " (expected: [" << expected_grad_x << ", " << expected_grad_y << "])\n";

        if(std::abs(gradient[x] - expected_grad_x) > tolerance || std::abs(gradient[y] - expected_grad_y) > tolerance)
        {
            std::cout << "  FAILED: Gradient mismatch!\n";
            passed = false;
        }

        // d^2/dx^2 = -1/x^2 = -0.25
        // d^2/dy^2 = exp(y) = 1
        double expected_hess_xx = -0.25;
        double expected_hess_yy = 1.0;

        auto key_xx = std::make_pair(x, x);
        auto key_yy = std::make_pair(y, y);

        if(hessian.find(key_xx) != hessian.end())
            std::cout << "  Hessian[x,x] = " << hessian[key_xx] << " (expected: " << expected_hess_xx << ")\n";
        if(hessian.find(key_yy) != hessian.end())
            std::cout << "  Hessian[y,y] = " << hessian[key_yy] << " (expected: " << expected_hess_yy << ")\n";

        if(hessian.find(key_xx) == hessian.end() || std::abs(hessian[key_xx] - expected_hess_xx) > tolerance
            || hessian.find(key_yy) == hessian.end() || std::abs(hessian[key_yy] - expected_hess_yy) > tolerance)
        {
            std::cout << "  FAILED: Hessian mismatch!\n";
            passed = false;
        }

        if(passed)
            std::cout << "  PASSED\n";
    }

    // ========== Test 9: Signomial term x^0.5 * y^1.5 ==========
    {
        std::cout << "\nTest 9: Gradient and Hessian of signomial x^0.5 * y^1.5\n";

        auto problem = std::make_shared<Problem>(env);
        problem->name = "signomial_test";

        auto x = std::make_shared<Variable>("x", 0, E_VariableType::Real, 0.1, 10.0);
        auto y = std::make_shared<Variable>("y", 1, E_VariableType::Real, 0.1, 10.0);
        problem->add(x);
        problem->add(y);

        // Create signomial: x^0.5 * y^1.5
        SignomialElements sigElements;
        sigElements.push_back(std::make_shared<SignomialElement>(x, 0.5));
        sigElements.push_back(std::make_shared<SignomialElement>(y, 1.5));
        SignomialTerms sigTerms;
        sigTerms.add(std::make_shared<SignomialTerm>(1.0, sigElements));

        auto objective = std::make_shared<NonlinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        objective->add(sigTerms);
        problem->add(objective);

        problem->finalize();

        VectorDouble point = { 4.0, 1.0 };
        // f = x^0.5 * y^1.5 = 2 * 1 = 2
        // df/dx = 0.5 * x^(-0.5) * y^1.5 = 0.5 * 0.5 * 1 = 0.25
        // df/dy = 1.5 * x^0.5 * y^0.5 = 1.5 * 2 * 1 = 3.0
        double expected_grad_x = 0.25;
        double expected_grad_y = 3.0;

        auto gradient = objective->calculateGradient(point, true);

        std::cout << "  At (4,1): gradient = [" << gradient[x] << ", " << gradient[y] << "]";
        std::cout << " (expected: [" << expected_grad_x << ", " << expected_grad_y << "])\n";

        if(std::abs(gradient[x] - expected_grad_x) > tolerance || std::abs(gradient[y] - expected_grad_y) > tolerance)
        {
            std::cout << "  FAILED: Gradient mismatch!\n";
            passed = false;
        }

        if(passed)
            std::cout << "  PASSED\n";
    }

    // ========== Test 10: Monomial term x*y*z ==========
    {
        std::cout << "\nTest 10: Gradient and Hessian of monomial x*y*z\n";

        auto problem = std::make_shared<Problem>(env);
        problem->name = "monomial_test";

        auto x = std::make_shared<Variable>("x", 0, E_VariableType::Real, 0.1, 10.0);
        auto y = std::make_shared<Variable>("y", 1, E_VariableType::Real, 0.1, 10.0);
        auto z = std::make_shared<Variable>("z", 2, E_VariableType::Real, 0.1, 10.0);
        problem->add(x);
        problem->add(y);
        problem->add(z);

        // Create monomial: x*y*z
        Variables monomialVars;
        monomialVars.push_back(x);
        monomialVars.push_back(y);
        monomialVars.push_back(z);
        MonomialTerms monomialTerms;
        monomialTerms.add(std::make_shared<MonomialTerm>(1.0, monomialVars));

        auto objective = std::make_shared<NonlinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        objective->add(monomialTerms);
        problem->add(objective);

        problem->finalize();

        VectorDouble point = { 2.0, 3.0, 4.0 };
        // f = x*y*z
        // df/dx = y*z = 12
        // df/dy = x*z = 8
        // df/dz = x*y = 6
        double expected_grad_x = 12.0;
        double expected_grad_y = 8.0;
        double expected_grad_z = 6.0;

        auto gradient = objective->calculateGradient(point, true);
        auto hessian = objective->calculateHessian(point, true);

        std::cout << "  At (2,3,4): gradient = [" << gradient[x] << ", " << gradient[y] << ", " << gradient[z] << "]";
        std::cout << " (expected: [" << expected_grad_x << ", " << expected_grad_y << ", " << expected_grad_z << "])\n";

        if(std::abs(gradient[x] - expected_grad_x) > tolerance || std::abs(gradient[y] - expected_grad_y) > tolerance
            || std::abs(gradient[z] - expected_grad_z) > tolerance)
        {
            std::cout << "  FAILED: Gradient mismatch!\n";
            passed = false;
        }

        // Hessian: d^2/dxdy = z = 4, d^2/dxdz = y = 3, d^2/dydz = x = 2
        auto key_xy = std::make_pair(x, y);
        auto key_xz = std::make_pair(x, z);
        auto key_yz = std::make_pair(y, z);

        if(hessian.find(key_xy) != hessian.end())
            std::cout << "  Hessian[x,y] = " << hessian[key_xy] << " (expected: 4)\n";
        if(hessian.find(key_xz) != hessian.end())
            std::cout << "  Hessian[x,z] = " << hessian[key_xz] << " (expected: 3)\n";
        if(hessian.find(key_yz) != hessian.end())
            std::cout << "  Hessian[y,z] = " << hessian[key_yz] << " (expected: 2)\n";

        if(hessian.find(key_xy) == hessian.end() || std::abs(hessian[key_xy] - 4.0) > tolerance
            || hessian.find(key_xz) == hessian.end() || std::abs(hessian[key_xz] - 3.0) > tolerance
            || hessian.find(key_yz) == hessian.end() || std::abs(hessian[key_yz] - 2.0) > tolerance)
        {
            std::cout << "  FAILED: Hessian mismatch!\n";
            passed = false;
        }

        if(passed)
            std::cout << "  PASSED\n";
    }

    // ========== Test 11: Mixed objective with linear + quadratic + nonlinear ==========
    {
        std::cout << "\nTest 11: Gradient and Hessian of mixed objective 2*x + x^2 + exp(y)\n";

        auto problem = std::make_shared<Problem>(env);
        problem->name = "mixed_objective_test";

        auto x = std::make_shared<Variable>("x", 0, E_VariableType::Real, -10.0, 10.0);
        auto y = std::make_shared<Variable>("y", 1, E_VariableType::Real, -10.0, 10.0);
        problem->add(x);
        problem->add(y);

        // Create objective: 2*x + x^2 + exp(y)
        LinearTerms linTerms;
        linTerms.add(std::make_shared<LinearTerm>(2.0, x));

        QuadraticTerms quadTerms;
        quadTerms.add(std::make_shared<QuadraticTerm>(1.0, x, x));

        auto nl_y = std::make_shared<ExpressionVariable>(y);
        auto expr_exp = std::make_shared<ExpressionExp>(nl_y);

        auto objective = std::make_shared<NonlinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        objective->add(linTerms);
        objective->add(quadTerms);
        objective->add(expr_exp);
        problem->add(objective);

        problem->finalize();

        VectorDouble point = { 1.0, 0.0 };
        // f = 2*x + x^2 + exp(y)
        // df/dx = 2 + 2*x = 4
        // df/dy = exp(y) = 1
        double expected_grad_x = 4.0;
        double expected_grad_y = 1.0;
        // d^2f/dx^2 = 2
        // d^2f/dy^2 = exp(y) = 1
        double expected_hess_xx = 2.0;
        double expected_hess_yy = 1.0;

        auto gradient = objective->calculateGradient(point, true);
        auto hessian = objective->calculateHessian(point, true);

        std::cout << "  At (1,0): gradient = [" << gradient[x] << ", " << gradient[y] << "]";
        std::cout << " (expected: [" << expected_grad_x << ", " << expected_grad_y << "])\n";

        if(std::abs(gradient[x] - expected_grad_x) > tolerance || std::abs(gradient[y] - expected_grad_y) > tolerance)
        {
            std::cout << "  FAILED: Gradient mismatch!\n";
            passed = false;
        }

        auto key_xx = std::make_pair(x, x);
        auto key_yy = std::make_pair(y, y);

        std::cout << "  Hessian[x,x] = " << hessian[key_xx] << " (expected: " << expected_hess_xx << ")\n";
        std::cout << "  Hessian[y,y] = " << hessian[key_yy] << " (expected: " << expected_hess_yy << ")\n";

        if(std::abs(hessian[key_xx] - expected_hess_xx) > tolerance
            || std::abs(hessian[key_yy] - expected_hess_yy) > tolerance)
        {
            std::cout << "  FAILED: Hessian mismatch!\n";
            passed = false;
        }

        if(passed)
            std::cout << "  PASSED\n";
    }

    // ========== Test 12: Mixed constraint with signomial + quadratic ==========
    {
        std::cout << "\nTest 12: Gradient and Hessian of constraint x^0.5 + y^2 <= 10\n";

        auto problem = std::make_shared<Problem>(env);
        problem->name = "mixed_constraint_test";

        auto x = std::make_shared<Variable>("x", 0, E_VariableType::Real, 0.1, 10.0);
        auto y = std::make_shared<Variable>("y", 1, E_VariableType::Real, -10.0, 10.0);
        problem->add(x);
        problem->add(y);

        // Create constraint: x^0.5 + y^2 <= 10
        SignomialElements sigElements;
        sigElements.push_back(std::make_shared<SignomialElement>(x, 0.5));
        SignomialTerms sigTerms;
        sigTerms.add(std::make_shared<SignomialTerm>(1.0, sigElements));

        QuadraticTerms quadTerms;
        quadTerms.add(std::make_shared<QuadraticTerm>(1.0, y, y));

        auto constraint = std::make_shared<NonlinearConstraint>(0, "mixed_constr", SHOT_DBL_MIN, 10.0);
        constraint->add(sigTerms);
        constraint->add(quadTerms);
        problem->add(constraint);

        // Dummy objective
        LinearTerms objTerms;
        objTerms.add(std::make_shared<LinearTerm>(1.0, x));
        auto objective = std::make_shared<LinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        objective->add(objTerms);
        problem->add(objective);

        problem->finalize();

        VectorDouble point = { 4.0, 2.0 };
        // g = x^0.5 + y^2
        // dg/dx = 0.5 * x^(-0.5) = 0.25
        // dg/dy = 2*y = 4
        double expected_grad_x = 0.25;
        double expected_grad_y = 4.0;
        // d^2g/dx^2 = -0.25 * x^(-1.5) = -0.25 / 8 = -0.03125
        // d^2g/dy^2 = 2
        double expected_hess_xx = -0.03125;
        double expected_hess_yy = 2.0;

        auto gradient = constraint->calculateGradient(point, true);
        auto hessian = constraint->calculateHessian(point, true);

        std::cout << "  At (4,2): gradient = [" << gradient[x] << ", " << gradient[y] << "]";
        std::cout << " (expected: [" << expected_grad_x << ", " << expected_grad_y << "])\n";

        if(std::abs(gradient[x] - expected_grad_x) > tolerance || std::abs(gradient[y] - expected_grad_y) > tolerance)
        {
            std::cout << "  FAILED: Gradient mismatch!\n";
            passed = false;
        }

        auto key_xx = std::make_pair(x, x);
        auto key_yy = std::make_pair(y, y);

        std::cout << "  Hessian[x,x] = " << hessian[key_xx] << " (expected: " << expected_hess_xx << ")\n";
        std::cout << "  Hessian[y,y] = " << hessian[key_yy] << " (expected: " << expected_hess_yy << ")\n";

        if(std::abs(hessian[key_xx] - expected_hess_xx) > tolerance
            || std::abs(hessian[key_yy] - expected_hess_yy) > tolerance)
        {
            std::cout << "  FAILED: Hessian mismatch!\n";
            passed = false;
        }

        if(passed)
            std::cout << "  PASSED\n";
    }

    // ========== Test 13: Full problem with nonlinear objective and constraints ==========
    {
        std::cout << "\nTest 13: Full problem - objective log(x) + y, constraint exp(x) + y^2 <= 20\n";

        auto problem = std::make_shared<Problem>(env);
        problem->name = "full_problem_test";

        auto x = std::make_shared<Variable>("x", 0, E_VariableType::Real, 0.1, 10.0);
        auto y = std::make_shared<Variable>("y", 1, E_VariableType::Real, -10.0, 10.0);
        problem->add(x);
        problem->add(y);

        // Create nonlinear objective: log(x) + y
        auto nl_x = std::make_shared<ExpressionVariable>(x);
        auto expr_log = std::make_shared<ExpressionLog>(nl_x);

        LinearTerms objLinTerms;
        objLinTerms.add(std::make_shared<LinearTerm>(1.0, y));

        auto objective = std::make_shared<NonlinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);
        objective->add(objLinTerms);
        objective->add(expr_log);
        problem->add(objective);

        // Create constraint: exp(x) + y^2 <= 20
        auto nl_x2 = std::make_shared<ExpressionVariable>(x);
        auto expr_exp = std::make_shared<ExpressionExp>(nl_x2);

        QuadraticTerms constrQuadTerms;
        constrQuadTerms.add(std::make_shared<QuadraticTerm>(1.0, y, y));

        auto constraint = std::make_shared<NonlinearConstraint>(0, "nl_constr", SHOT_DBL_MIN, 20.0);
        constraint->add(constrQuadTerms);
        constraint->add(expr_exp);
        problem->add(constraint);

        problem->finalize();

        VectorDouble point = { 1.0, 2.0 };

        // Objective: f = log(x) + y
        // df/dx = 1/x = 1
        // df/dy = 1
        double expected_obj_grad_x = 1.0;
        double expected_obj_grad_y = 1.0;

        auto obj_gradient = objective->calculateGradient(point, true);
        auto obj_hessian = objective->calculateHessian(point, true);

        std::cout << "  Objective at (1,2): gradient = [" << obj_gradient[x] << ", " << obj_gradient[y] << "]";
        std::cout << " (expected: [" << expected_obj_grad_x << ", " << expected_obj_grad_y << "])\n";

        if(std::abs(obj_gradient[x] - expected_obj_grad_x) > tolerance
            || std::abs(obj_gradient[y] - expected_obj_grad_y) > tolerance)
        {
            std::cout << "  FAILED: Objective gradient mismatch!\n";
            passed = false;
        }

        // Constraint: g = exp(x) + y^2
        // dg/dx = exp(x) = e
        // dg/dy = 2*y = 4
        double expected_constr_grad_x = std::exp(1.0);
        double expected_constr_grad_y = 4.0;

        auto constr_gradient = constraint->calculateGradient(point, true);
        auto constr_hessian = constraint->calculateHessian(point, true);

        std::cout << "  Constraint at (1,2): gradient = [" << constr_gradient[x] << ", " << constr_gradient[y] << "]";
        std::cout << " (expected: [" << expected_constr_grad_x << ", " << expected_constr_grad_y << "])\n";

        if(std::abs(constr_gradient[x] - expected_constr_grad_x) > tolerance
            || std::abs(constr_gradient[y] - expected_constr_grad_y) > tolerance)
        {
            std::cout << "  FAILED: Constraint gradient mismatch!\n";
            passed = false;
        }

        // Check Hessians
        // Objective Hessian: d^2f/dx^2 = -1/x^2 = -1
        // Constraint Hessian: d^2g/dx^2 = exp(x) = e, d^2g/dy^2 = 2
        auto key_xx = std::make_pair(x, x);
        auto key_yy = std::make_pair(y, y);

        std::cout << "  Obj Hessian[x,x] = " << obj_hessian[key_xx] << " (expected: -1)\n";
        std::cout << "  Constr Hessian[x,x] = " << constr_hessian[key_xx] << " (expected: " << std::exp(1.0) << ")\n";
        std::cout << "  Constr Hessian[y,y] = " << constr_hessian[key_yy] << " (expected: 2)\n";

        if(std::abs(obj_hessian[key_xx] - (-1.0)) > tolerance
            || std::abs(constr_hessian[key_xx] - std::exp(1.0)) > tolerance
            || std::abs(constr_hessian[key_yy] - 2.0) > tolerance)
        {
            std::cout << "  FAILED: Hessian mismatch!\n";
            passed = false;
        }

        if(passed)
            std::cout << "  PASSED\n";
    }

    if(passed)
        std::cout << "\n*** ALL GRADIENT/HESSIAN TESTS PASSED! ***\n";
    else
        std::cout << "\n*** SOME GRADIENT/HESSIAN TESTS FAILED! ***\n";

    return passed;
}

bool ModelTestSquareRootReformulation()
{
    // Test that square root of sum of quadratic terms gets reformulated to quadratic constraint
    bool passed = true;

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();
    SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);
    env->problem = problem;

    // Creating variables
    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 100.0);
    SHOT::ExpressionVariablePtr expressionVariable_x = std::make_shared<SHOT::ExpressionVariable>(var_x);

    auto var_y = std::make_shared<SHOT::Variable>("y", 1, SHOT::E_VariableType::Real, 0.0, 100.0);
    SHOT::ExpressionVariablePtr expressionVariable_y = std::make_shared<SHOT::ExpressionVariable>(var_y);

    SHOT::Variables variables = { var_x, var_y };
    problem->add(variables);

    // Create a simple linear objective
    SHOT::LinearObjectiveFunctionPtr objectiveFunction
        = std::make_shared<SHOT::LinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Minimize);
    SHOT::LinearTermPtr objLinearTerm1 = std::make_shared<SHOT::LinearTerm>(1.0, var_x);
    objectiveFunction->add(objLinearTerm1);
    problem->add(objectiveFunction);

    // Add a linear constraint before the nonlinear one
    SHOT::LinearConstraintPtr linearConstraint1
        = std::make_shared<SHOT::LinearConstraint>(0, "linearconstr1", SHOT_DBL_MIN, 50.0);
    SHOT::LinearTermPtr linTerm1 = std::make_shared<SHOT::LinearTerm>(1.0, var_x);
    linearConstraint1->add(linTerm1);
    problem->add(linearConstraint1);

    // Create constraint: sqrt(x^2 + y^2) <= 10
    // This is: x^2 + y^2 in a square root expression
    SHOT::NonlinearExpressionPtr exprSquare_x = std::make_shared<SHOT::ExpressionSquare>(expressionVariable_x);
    SHOT::NonlinearExpressionPtr exprSquare_y = std::make_shared<SHOT::ExpressionSquare>(expressionVariable_y);

    SHOT::NonlinearExpressions sumExpressions;
    sumExpressions.add(exprSquare_x);
    sumExpressions.add(exprSquare_y);
    SHOT::NonlinearExpressionPtr exprSum = std::make_shared<SHOT::ExpressionSum>(sumExpressions);

    SHOT::NonlinearExpressionPtr exprSqrt = std::make_shared<SHOT::ExpressionSquareRoot>(exprSum);

    // Create constraint: sqrt(x^2 + y^2) <= 10
    SHOT::NonlinearConstraintPtr nonlinearConstraint
        = std::make_shared<SHOT::NonlinearConstraint>(0, "sqrtconstr", exprSqrt, -SHOT_DBL_MAX, 10.0);
    problem->add(nonlinearConstraint);

    // Create constraint: sqrt(x + y) <= 5
    // This should become x + y <= 25 (linear) after reformulation
    SHOT::NonlinearExpressions sumLinearExpressions;
    sumLinearExpressions.add(expressionVariable_x);
    sumLinearExpressions.add(expressionVariable_y);
    SHOT::NonlinearExpressionPtr exprLinearSum = std::make_shared<SHOT::ExpressionSum>(sumLinearExpressions);
    SHOT::NonlinearExpressionPtr exprSqrtLinear = std::make_shared<SHOT::ExpressionSquareRoot>(exprLinearSum);

    SHOT::NonlinearConstraintPtr nonlinearConstraint2
        = std::make_shared<SHOT::NonlinearConstraint>(1, "sqrtlinearconstr", exprSqrtLinear, -SHOT_DBL_MAX, 5.0);
    problem->add(nonlinearConstraint2);

    // Add a linear constraint after the nonlinear one
    SHOT::LinearConstraintPtr linearConstraint2
        = std::make_shared<SHOT::LinearConstraint>(1, "linearconstr2", 0.0, SHOT_DBL_MAX);
    SHOT::LinearTermPtr linTerm2 = std::make_shared<SHOT::LinearTerm>(1.0, var_y);
    linearConstraint2->add(linTerm2);
    problem->add(linearConstraint2);

    std::cout << "\nProblem before reformulation:\n";
    std::cout << problem << '\n';

    std::cout << "Number of nonlinear constraints before: " << problem->nonlinearConstraints.size() << '\n';
    std::cout << "Number of quadratic constraints before: " << problem->quadraticConstraints.size() << '\n';

    problem->finalize();

    // Apply simplification and reformulation
    simplifyNonlinearExpressions(problem, true, true, true);

    std::cout << "\nProblem after reformulation:\n";
    std::cout << problem << '\n';

    std::cout << "Number of nonlinear constraints after: " << problem->nonlinearConstraints.size() << '\n';
    std::cout << "Number of quadratic constraints after: " << problem->quadraticConstraints.size() << '\n';
    std::cout << "Number of linear constraints after: " << problem->linearConstraints.size() << '\n';

    // After square root squaring reformulation:
    // - sqrt(x^2 + y^2) <= 10 becomes x^2 + y^2 <= 100 (quadratic constraint)
    // - sqrt(x + y) <= 5 becomes x + y <= 25 (linear constraint)

    // Verify all constraints are in the correct order and type
    if(problem->numericConstraints.size() != 4)
    {
        std::cout << "FAILED: Expected 4 total constraints, got " << problem->numericConstraints.size() << '\n';
        passed = false;
    }

    // Verify first constraint is still linear
    if(problem->numericConstraints.size() > 0)
    {
        auto firstConstr = problem->numericConstraints[0];
        if(firstConstr->name != "linearconstr1" || dynamic_cast<LinearConstraint*>(firstConstr.get()) == nullptr)
        {
            std::cout << "FAILED: First constraint should be 'linearconstr1' and linear type\n";
            passed = false;
        }
    }

    // Verify second constraint is now quadratic (was nonlinear sqrt(x^2+y^2))
    if(problem->numericConstraints.size() > 1)
    {
        auto secondConstr = problem->numericConstraints[1];
        if(secondConstr->name != "sqrtconstr" || dynamic_cast<QuadraticConstraint*>(secondConstr.get()) == nullptr)
        {
            std::cout << "FAILED: Second constraint should be 'sqrtconstr' and quadratic type\n";
            passed = false;
        }
    }

    // Verify third constraint is now linear (was nonlinear sqrt(x+y))
    if(problem->numericConstraints.size() > 2)
    {
        auto thirdConstr = problem->numericConstraints[2];
        if(thirdConstr->name != "sqrtlinearconstr" || dynamic_cast<LinearConstraint*>(thirdConstr.get()) == nullptr)
        {
            std::cout << "FAILED: Third constraint should be 'sqrtlinearconstr' and linear type\n";
            passed = false;
        }

        // Verify the linear constraint has the correct structure: x + y <= 25
        auto linConstr = dynamic_cast<LinearConstraint*>(thirdConstr.get());
        if(linConstr)
        {
            if(linConstr->linearTerms.size() != 2)
            {
                std::cout << "FAILED: Expected 2 linear terms in 'sqrtlinearconstr', got "
                          << linConstr->linearTerms.size() << '\n';
                passed = false;
            }

            // Check bounds were squared correctly (5^2 = 25)
            if(abs(linConstr->valueRHS - 25.0) > 1e-6)
            {
                std::cout << "FAILED: Expected RHS=25 after squaring in 'sqrtlinearconstr', got " << linConstr->valueRHS
                          << '\n';
                passed = false;
            }
        }
    }

    // Verify fourth constraint is still linear
    if(problem->numericConstraints.size() > 3)
    {
        auto fourthConstr = problem->numericConstraints[3];
        if(fourthConstr->name != "linearconstr2" || dynamic_cast<LinearConstraint*>(fourthConstr.get()) == nullptr)
        {
            std::cout << "FAILED: Fourth constraint should be 'linearconstr2' and linear type\n";
            passed = false;
        }
    }

    if(problem->quadraticConstraints.size() < 1)
    {
        std::cout << "FAILED: Expected at least 1 quadratic constraint after reformulation, got "
                  << problem->quadraticConstraints.size() << '\n';
        passed = false;
    }

    // Find the quadratic constraint (x^2 + y^2 <= 100)
    QuadraticConstraintPtr mainQuadConstraint = nullptr;
    for(auto& c : problem->quadraticConstraints)
    {
        if(c->name == "sqrtconstr" && c->quadraticTerms.size() == 2)
        {
            mainQuadConstraint = c;
            break;
        }
    }

    if(!mainQuadConstraint)
    {
        std::cout << "FAILED: Could not find quadratic constraint 'sqrtconstr'\n";
        passed = false;
    }

    // Verify the quadratic constraint has the expected structure
    if(mainQuadConstraint)
    {
        if(mainQuadConstraint->quadraticTerms.size() != 2)
        {
            std::cout << "FAILED: Expected 2 quadratic terms, got " << mainQuadConstraint->quadraticTerms.size()
                      << '\n';
            passed = false;
        }

        if(mainQuadConstraint->linearTerms.size() != 0)
        {
            std::cout << "FAILED: Expected 0 linear terms, got " << mainQuadConstraint->linearTerms.size() << '\n';
            passed = false;
        }

        // Check bounds were squared correctly (10^2 = 100)
        if(abs(mainQuadConstraint->valueRHS - 100.0) > 1e-6)
        {
            std::cout << "FAILED: Expected RHS=100 after squaring, got " << mainQuadConstraint->valueRHS << '\n';
            passed = false;
        }
        else
        {
            std::cout << "SUCCESS: Quadratic constraint has correct structure (x^2 + y^2 <= 100)\n";
        }
    }

    if(passed)
        std::cout << "\nSUCCESS: Square root reformulation correctly converted to quadratic constraint\n";
    else
        std::cout << "\nFAILED: Square root reformulation test\n";

    return passed;
}

bool ModelTestFinalizeCalledTwice()
{
    bool passed = true;

    std::cout << "Testing that calling finalize() twice warns and the solver still works:\n";

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);
    problem->name = "FinalizeTwiceTest";
    env->problem = problem;

    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 100.0);
    auto var_y = std::make_shared<SHOT::Variable>("y", 1, SHOT::E_VariableType::Integer, 0.0, 1.0);
    SHOT::ExpressionVariablePtr expressionVariable_y = std::make_shared<SHOT::ExpressionVariable>(var_y);

    problem->add(SHOT::Variables({ var_x, var_y }));

    SHOT::LinearObjectiveFunctionPtr objectiveFunction
        = std::make_shared<SHOT::LinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Minimize);
    SHOT::LinearTermPtr objLinearTerm1 = std::make_shared<SHOT::LinearTerm>(1.0, var_x);
    SHOT::LinearTermPtr objLinearTerm2 = std::make_shared<SHOT::LinearTerm>(1.0, var_y);
    SHOT::LinearTerms objLinearTerms;
    objLinearTerms.add(objLinearTerm1);
    objLinearTerms.add(objLinearTerm2);
    objectiveFunction->add(objLinearTerms);
    problem->add(objectiveFunction);

    SHOT::NonlinearExpressionPtr exprConstant = std::make_shared<SHOT::ExpressionConstant>(3);
    SHOT::NonlinearExpressionPtr exprPower
        = std::make_shared<SHOT::ExpressionPower>(expressionVariable_y, exprConstant);
    SHOT::NonlinearConstraintPtr nonlinearConstraint
        = std::make_shared<SHOT::NonlinearConstraint>(0, "nlconstr", exprPower, -10.0, 20.0);
    problem->add(nonlinearConstraint);

    std::cout << "\nCalling finalize() for the first time:\n";
    problem->finalize();

    int numVarsAfterFirst = problem->properties.numberOfVariables;
    int numConstrsAfterFirst = problem->properties.numberOfNumericConstraints;

    std::cout << "After first finalize(): " << numVarsAfterFirst << " variable(s), " << numConstrsAfterFirst
              << " constraint(s).\n";

    std::cout << "\nCalling finalize() for the second time (should warn and return early):\n";
    problem->finalize();

    std::cout << "Second finalize() returned (expected a warning above).\n";

    // Properties should be unchanged since the second call was a no-op
    if(problem->properties.numberOfVariables != numVarsAfterFirst)
    {
        std::cout << "FAILED: Variable count changed after second finalize().\n";
        passed = false;
    }

    if(problem->properties.numberOfNumericConstraints != numConstrsAfterFirst)
    {
        std::cout << "FAILED: Constraint count changed after second finalize().\n";
        passed = false;
    }

    // Verify the solver can accept the already-finalized problem
    if(!solver->setProblem(problem))
    {
        std::cout << "FAILED: solver->setProblem() failed after finalize() was called.\n";
        passed = false;
    }

    if(passed)
        std::cout << "\nSUCCESS: Second finalize() warned and returned early; solver accepted the problem.\n";
    else
        std::cout << "\nFAILED: Double finalize test.\n";

    return passed;
}

bool ModelTestFinalizeNoObjective()
{
    bool passed = true;

    std::cout << "Testing that finalize() throws error when no objective function is set:\n";

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);
    problem->name = "NoObjectiveTest";
    env->problem = problem;

    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 100.0);
    problem->add(SHOT::Variables({ var_x }));

    // Intentionally no objective function added

    try
    {
        problem->finalize();
        std::cout << "FAILED: finalize() did not throw error with no objective function.\n";
        passed = false;
    }
    catch(const std::runtime_error& e)
    {
        std::cout << "SUCCESS: finalize() threw std::runtime_error as expected: " << e.what() << "\n";
    }

    return passed;
}

bool ModelTestFinalizeNoVariables()
{
    bool passed = true;

    std::cout << "Testing that finalize() throws error when no variables are added:\n";

    std::unique_ptr<Solver> solver = std::make_unique<Solver>();
    auto env = solver->getEnvironment();

    SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);
    problem->name = "NoVariablesTest";
    env->problem = problem;

    SHOT::LinearObjectiveFunctionPtr objectiveFunction
        = std::make_shared<SHOT::LinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Minimize);
    problem->add(objectiveFunction);

    // Intentionally no variables added

    try
    {
        problem->finalize();
        std::cout << "FAILED: finalize() did not throw with no variables.\n";
        passed = false;
    }
    catch(const std::runtime_error& e)
    {
        std::cout << "SUCCESS: finalize() threw std::runtime_error as expected: " << e.what() << "\n";
    }

    return passed;
}

// Builds a Solver + Problem via buildProblem(env), applies the given objective-epigraph strategy, reformulates
// (and optionally solves) it. Shared plumbing for
// ModelTestObjectiveEpigraphStrategy/ModelTestAntiEpigraphReformulation. All MIP solvers compiled into this build, so
// the epigraph/anti-epigraph/partitioning tests below can run against every one of them rather than just a single
// default.
static std::vector<std::pair<ES_MIPSolver, std::string>> AvailableMIPSolversForEpigraphTests()
{
    std::vector<std::pair<ES_MIPSolver, std::string>> solvers;
#if defined(HAS_CBC)
    solvers.push_back({ ES_MIPSolver::Cbc, "Cbc" });
#endif
#if defined(HAS_HIGHS)
    solvers.push_back({ ES_MIPSolver::Highs, "Highs" });
#endif
#if defined(HAS_GUROBI)
    solvers.push_back({ ES_MIPSolver::Gurobi, "Gurobi" });
#endif
#if defined(HAS_CPLEX)
    solvers.push_back({ ES_MIPSolver::Cplex, "Cplex" });
#endif
    return solvers;
}

static std::pair<std::unique_ptr<SHOT::Solver>, std::shared_ptr<SHOT::Environment>> SolveWithEpigraphStrategy(
    ES_ObjectiveEpigraphStrategy epigraphStrategy,
    const std::function<SHOT::ProblemPtr(const std::shared_ptr<SHOT::Environment>&)>& buildProblem, bool solve,
    ES_MIPSolver mipSolver, const std::function<void(SHOT::Solver&)>& extraSettings = nullptr)
{
    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();

    solver->updateSetting("Output.Console.LogLevel", static_cast<int>(E_LogLevel::Warning));
    solver->updateSetting("Model.Reformulation.ObjectiveFunction.EpigraphStrategy", static_cast<int>(epigraphStrategy));
    solver->updateSetting("Termination.TimeLimit", 20.0);
    solver->updateSetting("Dual.MIP.Solver", static_cast<int>(mipSolver));

    // Partitioning of quadratic/nonlinear-sum terms is disabled by default so the resulting model structure
    // (constraint counts, objective term composition) is identical across every MIP solver, regardless of
    // whether that solver supports native quadratics — without this, HiGHS/Cbc's forced
    // Model.Reformulation.Quadratics.Strategy=Nonlinear (SolverCompatibility) would partition quadratic terms
    // that Gurobi/Cplex leave untouched, making the same test assert different structures per solver.
    // ModelTestObjectivePartitioningStrategy, which specifically exercises these settings, overrides them below
    // via extraSettings.
    solver->updateSetting(
        "Model.Reformulation.Constraint.PartitionQuadraticTerms", static_cast<int>(ES_PartitionNonlinearSums::Never));
    solver->updateSetting(
        "Model.Reformulation.Constraint.PartitionNonlinearTerms", static_cast<int>(ES_PartitionNonlinearSums::Never));
    solver->updateSetting("Model.Reformulation.ObjectiveFunction.PartitionQuadraticTerms",
        static_cast<int>(ES_PartitionNonlinearSums::Never));
    solver->updateSetting("Model.Reformulation.ObjectiveFunction.PartitionNonlinearTerms",
        static_cast<int>(ES_PartitionNonlinearSums::Never));

    if(extraSettings)
        extraSettings(*solver);

    auto problem = buildProblem(env);
    problem->finalize();

    if(!solver->setProblem(problem))
    {
        std::cout << "  FAILED: solver->setProblem() failed.\n";
        return { std::move(solver), env };
    }

    if(solve && !solver->solveProblem())
    {
        std::cout << "  FAILED: solver->solveProblem() failed.\n";
    }

    return { std::move(solver), env };
}

static bool CheckSolvedObjective(
    const std::shared_ptr<SHOT::Environment>& env, double expectedValue, const std::string& description)
{
    constexpr double tolerance = 0.01;

    if(env->results->primalSolutions.size() == 0)
    {
        std::cout << "  FAILED (" << description << "): no primal solution found.\n";
        return false;
    }

    double objValue = env->results->primalSolutions[0].objValue;
    std::cout << "  " << description << ": objective = " << objValue << " (expected " << expectedValue << ")\n";

    if(std::abs(objValue - expectedValue) > tolerance)
    {
        std::cout << "  FAILED (" << description << "): objective differs from expected by "
                  << std::abs(objValue - expectedValue) << ".\n";
        return false;
    }

    return true;
}

// Verifies that Model.Reformulation.ObjectiveFunction.EpigraphStrategy correctly controls whether a
// nonlinear/quadratic objective is reformulated into an epigraph auxiliary-variable constraint, and that a
// linear objective is left untouched either way. Also solves each variant and checks the objective against its
// known closed-form optimum, since a structurally-plausible reformulation can still be mathematically wrong
// (e.g. a sign error introduced only for maximize objectives).
bool ModelTestObjectiveEpigraphStrategy()
{
    bool passed = true;

    // Every sub-test below runs once per MIP solver available in this build. Quadratic/nonlinear-sum term
    // partitioning is disabled by default in SolveWithEpigraphStrategy, so the resulting model structure
    // (constraint counts, objective term composition) is identical across solvers regardless of native
    // quadratic-constraint support — only the numeric solve outcome should ever vary by solver.
    for(auto& [mipSolver, solverName] : AvailableMIPSolversForEpigraphTests())
    {
        std::cout << "\n===== MIP solver: " << solverName << " =====\n";

        // ── Sub-test 1: linear objective must never become an epigraph constraint ──────────────────
        std::cout << "\nSub-test 1: linear objective is unaffected by EpigraphStrategy=EpigraphConstraint\n";
        {
            auto [solver, env] = SolveWithEpigraphStrategy(
                ES_ObjectiveEpigraphStrategy::EpigraphConstraint,
                [](const std::shared_ptr<SHOT::Environment>& env)
                {
                    auto problem = std::make_shared<SHOT::Problem>(env);
                    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 7.0);
                    problem->add(SHOT::Variables({ var_x }));

                    auto objective
                        = std::make_shared<SHOT::LinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Maximize);
                    objective->add(std::make_shared<SHOT::LinearTerm>(1.0, var_x));
                    problem->add(objective);

                    return problem;
                },
                /*solve*/ false, mipSolver);

            auto reformulatedObjective
                = std::dynamic_pointer_cast<SHOT::LinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction);

            if(!reformulatedObjective)
            {
                std::cout << "  FAILED: reformulated objective is not a direct LinearObjectiveFunction.\n";
                passed = false;
            }
            else if(reformulatedObjective->direction != SHOT::E_ObjectiveFunctionDirection::Maximize)
            {
                std::cout << "  FAILED: reformulated linear objective's direction changed unexpectedly.\n";
                passed = false;
            }
            else if(reformulatedObjective->linearTerms.size() != 1)
            {
                std::cout << "  FAILED: reformulated linear objective should still have exactly one term.\n";
                passed = false;
            }

            if(env->reformulatedProblem->numericConstraints.size() != 0)
            {
                std::cout << "  FAILED: an epigraph constraint was created for a linear objective ("
                          << env->reformulatedProblem->numericConstraints.size() << " constraints found).\n";
                passed = false;
            }
        }

        // ── Sub-tests 2-3: quadratic objective -> epigraph constraint, minimize and maximize ────────
        for(bool isMaximize : { false, true })
        {
            std::string dirName = isMaximize ? "maximize" : "minimize";
            std::cout << "\nSub-test: quadratic " << dirName << " objective -> epigraph constraint\n";

            // minimize:  x^2 - 4x = (x-2)^2 - 4, over x in [0, 10] -> optimum at x=2, value -4
            // maximize: -x^2 + 6x,               over x in [0, 2]  -> optimum at x=2, value  8
            double ub = isMaximize ? 2.0 : 10.0;
            double expected = isMaximize ? 8.0 : -4.0;

            auto [solver, env] = SolveWithEpigraphStrategy(
                ES_ObjectiveEpigraphStrategy::EpigraphConstraint,
                [isMaximize, ub](const std::shared_ptr<SHOT::Environment>& env)
                {
                    auto problem = std::make_shared<SHOT::Problem>(env);
                    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, ub);
                    problem->add(SHOT::Variables({ var_x }));

                    auto objective = std::make_shared<SHOT::QuadraticObjectiveFunction>(isMaximize
                            ? SHOT::E_ObjectiveFunctionDirection::Maximize
                            : SHOT::E_ObjectiveFunctionDirection::Minimize);
                    objective->add(std::make_shared<SHOT::QuadraticTerm>(isMaximize ? -1.0 : 1.0, var_x, var_x));
                    objective->add(std::make_shared<SHOT::LinearTerm>(isMaximize ? 6.0 : -4.0, var_x));
                    problem->add(objective);

                    return problem;
                },
                /*solve*/ true, mipSolver);

            auto reformulatedObjective
                = std::dynamic_pointer_cast<SHOT::LinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction);

            if(!reformulatedObjective || reformulatedObjective->linearTerms.size() != 1
                || reformulatedObjective->direction != SHOT::E_ObjectiveFunctionDirection::Minimize)
            {
                std::cout << "  FAILED: reformulated quadratic objective is not a single-term minimize epigraph "
                             "objective.\n";
                passed = false;
            }

            // With partitioning disabled, the quadratic term stays inline on the epigraph-defining constraint
            // itself, so exactly one constraint (no separate square-term partition) should have been created.
            if(env->reformulatedProblem->numericConstraints.size() != 1)
            {
                std::cout << "  FAILED: expected exactly 1 epigraph constraint, found "
                          << env->reformulatedProblem->numericConstraints.size() << ".\n";
                passed = false;
            }

            passed = CheckSolvedObjective(
                         env, expected, "[" + solverName + "] quadratic " + dirName + " via epigraph constraint")
                && passed;
        }

        // ── Sub-test 4: nonlinear minimize objective (exp, convex) -> epigraph constraint ───────────
        // ── Sub-test 5: nonlinear maximize objective (log, concave) -> epigraph constraint ───────────
        // exp(x) is convex, so "minimize exp(x)" is a convex problem; log(x) is concave, so "maximize log(x)"
        // is a convex problem too ("minimize log(x)"/"maximize exp(x)" would both be nonconvex and are
        // deliberately not tested here).
        for(bool isMaximize : { false, true })
        {
            std::string dirName = isMaximize ? "maximize" : "minimize";
            std::string fnName = isMaximize ? "log" : "exp";
            std::cout << "\nSub-test: nonlinear " << dirName << " objective (" << fnName
                      << ") -> epigraph constraint\n";

            // maximize log(x), x in [0.1, 5] -> x=5, value log(5)
            // minimize exp(x), x in [0, 2]   -> x=0, value exp(0) = 1
            double lb = isMaximize ? 0.1 : 0.0;
            double ub = isMaximize ? 5.0 : 2.0;
            double expected = isMaximize ? std::log(5.0) : 1.0;

            auto [solver, env] = SolveWithEpigraphStrategy(
                ES_ObjectiveEpigraphStrategy::EpigraphConstraint,
                [isMaximize, lb, ub](const std::shared_ptr<SHOT::Environment>& env)
                {
                    auto problem = std::make_shared<SHOT::Problem>(env);
                    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, lb, ub);
                    problem->add(SHOT::Variables({ var_x }));

                    auto nl_x = std::make_shared<SHOT::ExpressionVariable>(var_x);
                    SHOT::NonlinearExpressionPtr expr = isMaximize
                        ? std::static_pointer_cast<SHOT::NonlinearExpression>(
                              std::make_shared<SHOT::ExpressionLog>(nl_x))
                        : std::static_pointer_cast<SHOT::NonlinearExpression>(
                              std::make_shared<SHOT::ExpressionExp>(nl_x));
                    auto objective = std::make_shared<SHOT::NonlinearObjectiveFunction>(isMaximize
                            ? SHOT::E_ObjectiveFunctionDirection::Maximize
                            : SHOT::E_ObjectiveFunctionDirection::Minimize,
                        expr, 0.0);
                    problem->add(objective);

                    return problem;
                },
                /*solve*/ true, mipSolver);

            auto reformulatedObjective
                = std::dynamic_pointer_cast<SHOT::LinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction);

            if(!reformulatedObjective || reformulatedObjective->linearTerms.size() != 1
                || reformulatedObjective->direction != SHOT::E_ObjectiveFunctionDirection::Minimize)
            {
                std::cout << "  FAILED: reformulated nonlinear objective is not a single-term minimize epigraph "
                             "objective.\n";
                passed = false;
            }

            // A plain (non-sum) nonlinear expression term needs no square/sum partitioning, so exactly one
            // constraint (the epigraph-defining one) should have been created.
            if(env->reformulatedProblem->numericConstraints.size() != 1)
            {
                std::cout << "  FAILED: expected exactly 1 epigraph constraint, found "
                          << env->reformulatedProblem->numericConstraints.size() << ".\n";
                passed = false;
            }

            passed = CheckSolvedObjective(
                         env, expected, "[" + solverName + "] nonlinear " + dirName + " via epigraph constraint")
                && passed;
        }

        // ── Sub-test 6: quadratic maximize objective, forced to stay a direct objective function ────
        std::cout << "\nSub-test 6: quadratic maximize objective stays a direct objective function\n";
        {
            auto [solver, env] = SolveWithEpigraphStrategy(
                ES_ObjectiveEpigraphStrategy::ObjectiveFunction,
                [](const std::shared_ptr<SHOT::Environment>& env)
                {
                    auto problem = std::make_shared<SHOT::Problem>(env);
                    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 2.0);
                    problem->add(SHOT::Variables({ var_x }));

                    auto objective = std::make_shared<SHOT::QuadraticObjectiveFunction>(
                        SHOT::E_ObjectiveFunctionDirection::Maximize);
                    objective->add(std::make_shared<SHOT::QuadraticTerm>(-1.0, var_x, var_x));
                    objective->add(std::make_shared<SHOT::LinearTerm>(6.0, var_x));
                    problem->add(objective);

                    return problem;
                },
                /*solve*/ true, mipSolver);

            bool hasEpigraphConstraint = std::any_of(env->reformulatedProblem->numericConstraints.begin(),
                env->reformulatedProblem->numericConstraints.end(),
                [](auto& C) { return C->name == "shot_objconstr"; });

            if(hasEpigraphConstraint)
            {
                std::cout
                    << "  FAILED: an epigraph constraint was created despite EpigraphStrategy=ObjectiveFunction.\n";
                passed = false;
            }

            // With partitioning disabled, the quadratic term stays inline: the direct objective keeps its
            // native quadratic term and no partition constraint is created. (QuadraticObjectiveFunction derives
            // from LinearObjectiveFunction in this model hierarchy, so a dynamic_pointer_cast<LinearObjectiveFunction>
            // alone can't distinguish them — hasQuadraticTerms is the correct check.)
            if(!env->reformulatedProblem->objectiveFunction->properties.hasQuadraticTerms)
            {
                std::cout << "  FAILED: expected a direct objective function retaining its native quadratic "
                             "term.\n";
                passed = false;
            }

            if(env->reformulatedProblem->numericConstraints.size() != 0)
            {
                std::cout << "  FAILED: expected 0 constraints (no square-term partitioning), found "
                          << env->reformulatedProblem->numericConstraints.size() << ".\n";
                passed = false;
            }

            passed = CheckSolvedObjective(
                         env, 8.0, "[" + solverName + "] quadratic maximize via direct objective function")
                && passed;
        }

        // ── Sub-test 7: nonlinear (log) maximize objective, forced to stay a direct objective function
        std::cout << "\nSub-test 7: nonlinear maximize objective (log) stays a direct objective function\n";
        {
            auto [solver, env] = SolveWithEpigraphStrategy(
                ES_ObjectiveEpigraphStrategy::ObjectiveFunction,
                [](const std::shared_ptr<SHOT::Environment>& env)
                {
                    auto problem = std::make_shared<SHOT::Problem>(env);
                    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.1, 5.0);
                    problem->add(SHOT::Variables({ var_x }));

                    auto nl_x = std::make_shared<SHOT::ExpressionVariable>(var_x);
                    auto objective = std::make_shared<SHOT::NonlinearObjectiveFunction>(
                        SHOT::E_ObjectiveFunctionDirection::Maximize, std::make_shared<SHOT::ExpressionLog>(nl_x), 0.0);
                    problem->add(objective);

                    return problem;
                },
                /*solve*/ true, mipSolver);

            if(env->reformulatedProblem->numericConstraints.size() != 0)
            {
                std::cout << "  FAILED: an epigraph constraint was created despite EpigraphStrategy=ObjectiveFunction ("
                          << env->reformulatedProblem->numericConstraints.size() << " constraints found).\n";
                passed = false;
            }

            // A plain log(x) term needs no sum-partitioning, so the direct objective stays a
            // NonlinearObjectiveFunction wrapping the (negated, since the original is a maximize objective)
            // nonlinear expression.
            auto reformulatedObjective = std::dynamic_pointer_cast<SHOT::NonlinearObjectiveFunction>(
                env->reformulatedProblem->objectiveFunction);

            if(!reformulatedObjective || !reformulatedObjective->properties.hasNonlinearExpression)
            {
                std::cout << "  FAILED: expected a direct NonlinearObjectiveFunction wrapping the log expression.\n";
                passed = false;
            }

            passed = CheckSolvedObjective(
                         env, std::log(5.0), "[" + solverName + "] nonlinear maximize via direct objective function")
                && passed;
        }

        // ── Sub-tests 8-9: mixed linear + quadratic + nonlinear objective -> epigraph constraint ────
        // minimize: (x^2 - 4x) + 2w + exp(z), x in [0,10], w in [0,3], z in [0,2]
        //   -> convex (x^2-4x convex, 2w affine, exp(z) convex); optimum x=2,w=0,z=0 -> -4 + 0 + 1 = -3
        // maximize: (-x^2 + 6x) + 4w + log(z), x in [0,2], w in [0,3], z in [1,5]
        //   -> concave (-x^2+6x concave, 4w affine, log(z) concave); optimum x=2,w=3,z=5 -> 8 + 12 + log(5)
        for(bool isMaximize : { false, true })
        {
            std::string dirName = isMaximize ? "maximize" : "minimize";
            std::cout << "\nSub-test: mixed linear+quadratic+nonlinear " << dirName
                      << " objective -> epigraph constraint\n";

            double expected = isMaximize ? (8.0 + 12.0 + std::log(5.0)) : (-4.0 + 0.0 + 1.0);

            auto buildMixed = [isMaximize](const std::shared_ptr<SHOT::Environment>& env)
            {
                auto problem = std::make_shared<SHOT::Problem>(env);
                auto var_x = std::make_shared<SHOT::Variable>(
                    "x", 0, SHOT::E_VariableType::Real, 0.0, isMaximize ? 2.0 : 10.0);
                auto var_w = std::make_shared<SHOT::Variable>("w", 1, SHOT::E_VariableType::Real, 0.0, 3.0);
                auto var_z = std::make_shared<SHOT::Variable>(
                    "z", 2, SHOT::E_VariableType::Real, isMaximize ? 1.0 : 0.0, isMaximize ? 5.0 : 2.0);
                problem->add(SHOT::Variables({ var_x, var_w, var_z }));

                auto objective = std::make_shared<SHOT::NonlinearObjectiveFunction>(isMaximize
                        ? SHOT::E_ObjectiveFunctionDirection::Maximize
                        : SHOT::E_ObjectiveFunctionDirection::Minimize);
                objective->add(std::make_shared<SHOT::QuadraticTerm>(isMaximize ? -1.0 : 1.0, var_x, var_x));
                objective->add(std::make_shared<SHOT::LinearTerm>(isMaximize ? 6.0 : -4.0, var_x));
                objective->add(std::make_shared<SHOT::LinearTerm>(isMaximize ? 4.0 : 2.0, var_w));
                auto nl_z = std::make_shared<SHOT::ExpressionVariable>(var_z);
                objective->add(isMaximize ? std::static_pointer_cast<SHOT::NonlinearExpression>(
                                                std::make_shared<SHOT::ExpressionLog>(nl_z))
                                          : std::static_pointer_cast<SHOT::NonlinearExpression>(
                                                std::make_shared<SHOT::ExpressionExp>(nl_z)));
                problem->add(objective);

                return problem;
            };

            auto [solver, env] = SolveWithEpigraphStrategy(
                ES_ObjectiveEpigraphStrategy::EpigraphConstraint, buildMixed, /*solve*/ true, mipSolver);

            auto reformulatedObjective
                = std::dynamic_pointer_cast<SHOT::LinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction);

            if(!reformulatedObjective || reformulatedObjective->linearTerms.size() != 1
                || reformulatedObjective->direction != SHOT::E_ObjectiveFunctionDirection::Minimize)
            {
                std::cout << "  FAILED: reformulated mixed objective is not a single-term minimize epigraph "
                             "objective.\n";
                passed = false;
            }

            passed = CheckSolvedObjective(
                         env, expected, "[" + solverName + "] mixed " + dirName + " via epigraph constraint")
                && passed;
        }

        // ── Sub-tests 10-11: mixed linear + quadratic + nonlinear objective, direct objective function
        for(bool isMaximize : { false, true })
        {
            std::string dirName = isMaximize ? "maximize" : "minimize";
            std::cout << "\nSub-test: mixed linear+quadratic+nonlinear " << dirName
                      << " objective stays a direct objective function\n";

            double expected = isMaximize ? (8.0 + 12.0 + std::log(5.0)) : (-4.0 + 0.0 + 1.0);

            auto buildMixed = [isMaximize](const std::shared_ptr<SHOT::Environment>& env)
            {
                auto problem = std::make_shared<SHOT::Problem>(env);
                auto var_x = std::make_shared<SHOT::Variable>(
                    "x", 0, SHOT::E_VariableType::Real, 0.0, isMaximize ? 2.0 : 10.0);
                auto var_w = std::make_shared<SHOT::Variable>("w", 1, SHOT::E_VariableType::Real, 0.0, 3.0);
                auto var_z = std::make_shared<SHOT::Variable>(
                    "z", 2, SHOT::E_VariableType::Real, isMaximize ? 1.0 : 0.0, isMaximize ? 5.0 : 2.0);
                problem->add(SHOT::Variables({ var_x, var_w, var_z }));

                auto objective = std::make_shared<SHOT::NonlinearObjectiveFunction>(isMaximize
                        ? SHOT::E_ObjectiveFunctionDirection::Maximize
                        : SHOT::E_ObjectiveFunctionDirection::Minimize);
                objective->add(std::make_shared<SHOT::QuadraticTerm>(isMaximize ? -1.0 : 1.0, var_x, var_x));
                objective->add(std::make_shared<SHOT::LinearTerm>(isMaximize ? 6.0 : -4.0, var_x));
                objective->add(std::make_shared<SHOT::LinearTerm>(isMaximize ? 4.0 : 2.0, var_w));
                auto nl_z = std::make_shared<SHOT::ExpressionVariable>(var_z);
                objective->add(isMaximize ? std::static_pointer_cast<SHOT::NonlinearExpression>(
                                                std::make_shared<SHOT::ExpressionLog>(nl_z))
                                          : std::static_pointer_cast<SHOT::NonlinearExpression>(
                                                std::make_shared<SHOT::ExpressionExp>(nl_z)));
                problem->add(objective);

                return problem;
            };

            auto [solver, env] = SolveWithEpigraphStrategy(
                ES_ObjectiveEpigraphStrategy::ObjectiveFunction, buildMixed, /*solve*/ true, mipSolver);

            bool hasEpigraphConstraint = std::any_of(env->reformulatedProblem->numericConstraints.begin(),
                env->reformulatedProblem->numericConstraints.end(),
                [](auto& C) { return C->name == "shot_objconstr"; });

            if(hasEpigraphConstraint)
            {
                std::cout
                    << "  FAILED: an epigraph constraint was created despite EpigraphStrategy=ObjectiveFunction.\n";
                passed = false;
            }

            passed = CheckSolvedObjective(
                         env, expected, "[" + solverName + "] mixed " + dirName + " via direct objective function")
                && passed;
        }

        // ── Sub-test 12: EpigraphStrategy=Unchanged leaves a direct quadratic/nonlinear objective exactly as
        // given — no epigraph constraint is introduced. This must produce the same result as sub-tests 6/7
        // above (which force the same outcome via EpigraphStrategy=ObjectiveFunction), since there is nothing
        // here for Unchanged to fold FROM other than an already-direct objective function.
        for(bool isQuadratic : { true, false })
        {
            std::string kindName = isQuadratic ? "quadratic" : "nonlinear (log)";
            std::cout << "\nSub-test 12: " << kindName
                      << " maximize objective is left as a direct objective function by "
                         "EpigraphStrategy=Unchanged\n";

            auto [solver, env] = SolveWithEpigraphStrategy(
                ES_ObjectiveEpigraphStrategy::Unchanged,
                [isQuadratic](const std::shared_ptr<SHOT::Environment>& env)
                {
                    auto problem = std::make_shared<SHOT::Problem>(env);

                    if(isQuadratic)
                    {
                        auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 2.0);
                        problem->add(SHOT::Variables({ var_x }));

                        auto objective = std::make_shared<SHOT::QuadraticObjectiveFunction>(
                            SHOT::E_ObjectiveFunctionDirection::Maximize);
                        objective->add(std::make_shared<SHOT::QuadraticTerm>(-1.0, var_x, var_x));
                        objective->add(std::make_shared<SHOT::LinearTerm>(6.0, var_x));
                        problem->add(objective);
                    }
                    else
                    {
                        auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.1, 5.0);
                        problem->add(SHOT::Variables({ var_x }));

                        auto nl_x = std::make_shared<SHOT::ExpressionVariable>(var_x);
                        auto objective = std::make_shared<SHOT::NonlinearObjectiveFunction>(
                            SHOT::E_ObjectiveFunctionDirection::Maximize, std::make_shared<SHOT::ExpressionLog>(nl_x),
                            0.0);
                        problem->add(objective);
                    }

                    return problem;
                },
                /*solve*/ true, mipSolver);

            bool hasEpigraphConstraint = std::any_of(env->reformulatedProblem->numericConstraints.begin(),
                env->reformulatedProblem->numericConstraints.end(),
                [](auto& C) { return C->name == "shot_objconstr"; });

            if(hasEpigraphConstraint)
            {
                std::cout << "  FAILED: an epigraph constraint was created despite EpigraphStrategy=Unchanged.\n";
                passed = false;
            }

            if(isQuadratic)
            {
                // With partitioning disabled, the quadratic term stays inline (native).
                if(!env->reformulatedProblem->objectiveFunction->properties.hasQuadraticTerms)
                {
                    std::cout << "  FAILED: expected a direct objective function retaining its native quadratic "
                                 "term.\n";
                    passed = false;
                }

                passed = CheckSolvedObjective(
                             env, 8.0, "[" + solverName + "] quadratic maximize, EpigraphStrategy=Unchanged")
                    && passed;
            }
            else
            {
                auto reformulatedObjective = std::dynamic_pointer_cast<SHOT::NonlinearObjectiveFunction>(
                    env->reformulatedProblem->objectiveFunction);

                if(!reformulatedObjective || !reformulatedObjective->properties.hasNonlinearExpression)
                {
                    std::cout
                        << "  FAILED: expected a direct NonlinearObjectiveFunction wrapping the log expression.\n";
                    passed = false;
                }

                passed = CheckSolvedObjective(
                             env, std::log(5.0), "[" + solverName + "] nonlinear maximize, EpigraphStrategy=Unchanged")
                    && passed;
            }
        }
    }

    return passed;
}

// Verifies the "anti-epigraph" direction: a linear objective that is a thin wrapper around an epigraph-style
// defining constraint (single objective variable, appearing linearly in exactly one nonlinear/quadratic
// constraint) should be folded back into a direct nonlinear/quadratic objective function, with the defining
// constraint removed — unless EpigraphStrategy=EpigraphConstraint explicitly asks to keep it as a constraint.
bool ModelTestAntiEpigraphReformulation()
{
    bool passed = true;

    // Every sub-test below runs once per MIP solver available in this build. Term partitioning is disabled by
    // default in SolveWithEpigraphStrategy, so the epigraph-defining constraint's quadratic term stays inline
    // (never pre-split into an auxiliary variable) and the anti-epigraph fold's pattern-match sees the same
    // structure regardless of solver.
    for(auto& [mipSolver, solverName] : AvailableMIPSolversForEpigraphTests())
    {
        std::cout << "\n===== MIP solver: " << solverName << " =====\n";

        // ── Sub-test 1: minimize z s.t. z >= exp(x), x in [0, 2] -> should fold to minimize exp(x), x=0 ─
        // exp(x) is convex, so "z >= exp(x)" is a convex epigraph constraint and "minimize exp(x)" is a convex
        // problem (log(x) is concave, which would make this nonconvex, so it is deliberately not used here).
        std::cout << "\nSub-test 1: anti-epigraph folds 'minimize z s.t. z >= exp(x)' into 'minimize exp(x)'\n";
        {
            auto buildProblem = [](const std::shared_ptr<SHOT::Environment>& env)
            {
                auto problem = std::make_shared<SHOT::Problem>(env);
                auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 2.0);
                auto var_z = std::make_shared<SHOT::Variable>("z", 1, SHOT::E_VariableType::Real, -100.0, 100.0);
                problem->add(SHOT::Variables({ var_x, var_z }));

                auto objective
                    = std::make_shared<SHOT::LinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Minimize);
                objective->add(std::make_shared<SHOT::LinearTerm>(1.0, var_z));
                problem->add(objective);

                // z - exp(x) >= 0  <=>  z >= exp(x)
                auto nl_x = std::make_shared<SHOT::ExpressionVariable>(var_x);
                auto constraint = std::make_shared<SHOT::NonlinearConstraint>(0, "epidef", 0.0, SHOT_DBL_MAX);
                constraint->add(std::make_shared<SHOT::LinearTerm>(1.0, var_z));
                constraint->add(std::make_shared<SHOT::ExpressionNegate>(std::make_shared<SHOT::ExpressionExp>(nl_x)));
                problem->add(constraint);

                return problem;
            };

            double expected = 1.0;

            // ObjectiveFunction: the fold should happen, leaving a direct NonlinearObjectiveFunction and no
            // constraints.
            {
                auto [solver, env] = SolveWithEpigraphStrategy(
                    ES_ObjectiveEpigraphStrategy::ObjectiveFunction, buildProblem, /*solve*/ true, mipSolver);

                if(!env->reformulatedProblem->antiEpigraphObjectiveVariable)
                {
                    std::cout
                        << "  FAILED: anti-epigraph fold did not happen (antiEpigraphObjectiveVariable not set).\n";
                    passed = false;
                }

                auto reformulatedObjective = std::dynamic_pointer_cast<SHOT::NonlinearObjectiveFunction>(
                    env->reformulatedProblem->objectiveFunction);

                if(!reformulatedObjective || !reformulatedObjective->properties.hasNonlinearExpression)
                {
                    std::cout << "  FAILED: expected a direct NonlinearObjectiveFunction wrapping exp(x).\n";
                    passed = false;
                }

                if(env->reformulatedProblem->numericConstraints.size() != 0)
                {
                    std::cout << "  FAILED: expected the defining constraint to be removed by the fold, found "
                              << env->reformulatedProblem->numericConstraints.size() << " constraint(s).\n";
                    passed = false;
                }

                passed = CheckSolvedObjective(env, expected,
                             "[" + solverName + "] anti-epigraph fold, EpigraphStrategy=ObjectiveFunction")
                    && passed;
            }

            // Unchanged and EpigraphConstraint: the fold should be skipped either way, keeping the objective as
            // "minimize z" and the defining constraint as-is (1 constraint) — Unchanged because it performs no
            // epigraph/anti-epigraph reformulation at all, EpigraphConstraint because that form is exactly what
            // the fold would undo.
            for(auto strategy :
                { ES_ObjectiveEpigraphStrategy::Unchanged, ES_ObjectiveEpigraphStrategy::EpigraphConstraint })
            {
                std::string strategyName
                    = strategy == ES_ObjectiveEpigraphStrategy::Unchanged ? "Unchanged" : "EpigraphConstraint";

                auto [solver, env] = SolveWithEpigraphStrategy(strategy, buildProblem, /*solve*/ true, mipSolver);

                if(env->reformulatedProblem->antiEpigraphObjectiveVariable)
                {
                    std::cout << "  FAILED: anti-epigraph fold happened despite EpigraphStrategy=" << strategyName
                              << ".\n";
                    passed = false;
                }

                auto reformulatedObjective = std::dynamic_pointer_cast<SHOT::LinearObjectiveFunction>(
                    env->reformulatedProblem->objectiveFunction);

                if(!reformulatedObjective || reformulatedObjective->linearTerms.size() != 1
                    || reformulatedObjective->direction != SHOT::E_ObjectiveFunctionDirection::Minimize)
                {
                    std::cout << "  FAILED: expected the objective to remain 'minimize z' unchanged.\n";
                    passed = false;
                }

                if(env->reformulatedProblem->numericConstraints.size() != 1)
                {
                    std::cout << "  FAILED: expected the defining constraint to remain (1 constraint), found "
                              << env->reformulatedProblem->numericConstraints.size() << ".\n";
                    passed = false;
                }

                passed = CheckSolvedObjective(env, expected,
                             "[" + solverName + "] anti-epigraph kept as constraint, EpigraphStrategy=" + strategyName)
                    && passed;
            }
        }

        // ── Sub-test 2: maximize z s.t. z <= -x^2, x in [1, 3] -> should fold to maximize -x^2, x=1 ─────
        // -x^2 is concave, so "z <= -x^2" is a convex (hypograph) constraint and "maximize -x^2" is a convex
        // problem (x^2 is convex, which would make "maximize x^2" nonconvex, so it is deliberately not used
        // here).
        std::cout << "\nSub-test 2: anti-epigraph folds 'maximize z s.t. z <= -x^2' into 'maximize -x^2'\n";
        {
            auto buildProblem = [](const std::shared_ptr<SHOT::Environment>& env)
            {
                auto problem = std::make_shared<SHOT::Problem>(env);
                auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 1.0, 3.0);
                auto var_z = std::make_shared<SHOT::Variable>("z", 1, SHOT::E_VariableType::Real, -100.0, 100.0);
                problem->add(SHOT::Variables({ var_x, var_z }));

                auto objective
                    = std::make_shared<SHOT::LinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Maximize);
                objective->add(std::make_shared<SHOT::LinearTerm>(1.0, var_z));
                problem->add(objective);

                // z - (-x^2) <= 0  <=>  z + x^2 <= 0  <=>  z <= -x^2
                auto constraint = std::make_shared<SHOT::QuadraticConstraint>(0, "epidef", SHOT_DBL_MIN, 0.0);
                constraint->add(std::make_shared<SHOT::LinearTerm>(1.0, var_z));
                constraint->add(std::make_shared<SHOT::QuadraticTerm>(1.0, var_x, var_x));
                problem->add(constraint);

                return problem;
            };

            double expected = -1.0;

            // ObjectiveFunction: the fold should happen, leaving a direct native-quadratic objective (matches
            // ModelTestObjectiveEpigraphStrategy's sub-test 6 structure, which solves correctly on every solver).
            {
                auto [solver, env] = SolveWithEpigraphStrategy(
                    ES_ObjectiveEpigraphStrategy::ObjectiveFunction, buildProblem, /*solve*/ true, mipSolver);

                if(!env->reformulatedProblem->antiEpigraphObjectiveVariable)
                {
                    std::cout << "  FAILED: anti-epigraph fold did not happen for the quadratic case.\n";
                    passed = false;
                }

                if(env->reformulatedProblem->numericConstraints.size() != 0)
                {
                    std::cout << "  FAILED: expected the defining constraint to be removed by the fold, found "
                              << env->reformulatedProblem->numericConstraints.size() << " constraint(s).\n";
                    passed = false;
                }

                passed = CheckSolvedObjective(env, expected, "[" + solverName + "] anti-epigraph fold, quadratic")
                    && passed;
            }

            // Unchanged: no reformulation, so the fold must not happen and the defining constraint (a native
            // maximize quadratic epigraph constraint) must remain.
            {
                auto [solver, env] = SolveWithEpigraphStrategy(
                    ES_ObjectiveEpigraphStrategy::Unchanged, buildProblem, /*solve*/ true, mipSolver);

                if(env->reformulatedProblem->antiEpigraphObjectiveVariable)
                {
                    std::cout << "  FAILED: anti-epigraph fold happened despite EpigraphStrategy=Unchanged.\n";
                    passed = false;
                }

                if(env->reformulatedProblem->numericConstraints.size() != 1)
                {
                    std::cout << "  FAILED: expected the defining constraint to remain (1 constraint), found "
                              << env->reformulatedProblem->numericConstraints.size() << ".\n";
                    passed = false;
                }

                passed = CheckSolvedObjective(env, expected,
                             "[" + solverName + "] anti-epigraph kept as constraint, quadratic, Unchanged")
                    && passed;
            }
        }

        // ── Sub-test 3: minimize z s.t. z >= (x^2-4x) + 2w + exp(y) -> should fold to a direct mixed
        // linear+quadratic+nonlinear objective ────────────────────────────────────────────────────────
        // (x^2-4x) + 2w + exp(y) is convex (x^2-4x convex, 2w affine, exp(y) convex), so "z >= ..." is a convex
        // epigraph constraint and the folded-back objective is a convex minimization. x in [0,10], w in [0,3],
        // y in [0,2] -> optimum at x=2, w=0, y=0 -> -4 + 0 + 1 = -3.
        std::cout << "\nSub-test 3: anti-epigraph folds 'minimize z s.t. z >= (x^2-4x)+2w+exp(y)' into a direct mixed "
                     "linear+quadratic+nonlinear objective\n";
        {
            auto buildProblem = [](const std::shared_ptr<SHOT::Environment>& env)
            {
                auto problem = std::make_shared<SHOT::Problem>(env);
                auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 10.0);
                auto var_w = std::make_shared<SHOT::Variable>("w", 1, SHOT::E_VariableType::Real, 0.0, 3.0);
                auto var_y = std::make_shared<SHOT::Variable>("y", 2, SHOT::E_VariableType::Real, 0.0, 2.0);
                auto var_z = std::make_shared<SHOT::Variable>("z", 3, SHOT::E_VariableType::Real, -100.0, 100.0);
                problem->add(SHOT::Variables({ var_x, var_w, var_y, var_z }));

                auto objective
                    = std::make_shared<SHOT::LinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Minimize);
                objective->add(std::make_shared<SHOT::LinearTerm>(1.0, var_z));
                problem->add(objective);

                // z - x^2 + 4x - 2w - exp(y) >= 0  <=>  z >= (x^2-4x) + 2w + exp(y)
                auto nl_y = std::make_shared<SHOT::ExpressionVariable>(var_y);
                auto constraint = std::make_shared<SHOT::NonlinearConstraint>(0, "epidef", 0.0, SHOT_DBL_MAX);
                constraint->add(std::make_shared<SHOT::LinearTerm>(1.0, var_z));
                constraint->add(std::make_shared<SHOT::QuadraticTerm>(-1.0, var_x, var_x));
                constraint->add(std::make_shared<SHOT::LinearTerm>(4.0, var_x));
                constraint->add(std::make_shared<SHOT::LinearTerm>(-2.0, var_w));
                constraint->add(std::make_shared<SHOT::ExpressionNegate>(std::make_shared<SHOT::ExpressionExp>(nl_y)));
                problem->add(constraint);

                return problem;
            };

            double expected = -4.0 + 0.0 + 1.0;

            // ObjectiveFunction: the fold should happen, leaving a direct NonlinearObjectiveFunction (mixing
            // quadratic, linear and nonlinear terms) and no constraints.
            {
                auto [solver, env] = SolveWithEpigraphStrategy(
                    ES_ObjectiveEpigraphStrategy::ObjectiveFunction, buildProblem, /*solve*/ true, mipSolver);

                if(!env->reformulatedProblem->antiEpigraphObjectiveVariable)
                {
                    std::cout << "  FAILED: anti-epigraph fold did not happen for the mixed-term case "
                                 "(antiEpigraphObjectiveVariable not set).\n";
                    passed = false;
                }

                auto reformulatedObjective = std::dynamic_pointer_cast<SHOT::NonlinearObjectiveFunction>(
                    env->reformulatedProblem->objectiveFunction);

                if(!reformulatedObjective || !reformulatedObjective->properties.hasNonlinearExpression
                    || !reformulatedObjective->properties.hasQuadraticTerms)
                {
                    std::cout << "  FAILED: expected a direct NonlinearObjectiveFunction with both quadratic and "
                                 "nonlinear parts.\n";
                    passed = false;
                }

                if(env->reformulatedProblem->numericConstraints.size() != 0)
                {
                    std::cout << "  FAILED: expected the defining constraint to be removed by the fold, found "
                              << env->reformulatedProblem->numericConstraints.size() << " constraint(s).\n";
                    passed = false;
                }

                passed = CheckSolvedObjective(env, expected,
                             "[" + solverName + "] mixed-term anti-epigraph fold, EpigraphStrategy=ObjectiveFunction")
                    && passed;
            }

            // Unchanged and EpigraphConstraint: the fold should be skipped either way, keeping the objective as
            // "minimize z" and the defining constraint as-is (1 constraint).
            for(auto strategy :
                { ES_ObjectiveEpigraphStrategy::Unchanged, ES_ObjectiveEpigraphStrategy::EpigraphConstraint })
            {
                std::string strategyName
                    = strategy == ES_ObjectiveEpigraphStrategy::Unchanged ? "Unchanged" : "EpigraphConstraint";

                auto [solver, env] = SolveWithEpigraphStrategy(strategy, buildProblem, /*solve*/ true, mipSolver);

                if(env->reformulatedProblem->antiEpigraphObjectiveVariable)
                {
                    std::cout << "  FAILED: anti-epigraph fold happened despite EpigraphStrategy=" << strategyName
                              << " for the mixed-term case.\n";
                    passed = false;
                }

                auto reformulatedObjective = std::dynamic_pointer_cast<SHOT::LinearObjectiveFunction>(
                    env->reformulatedProblem->objectiveFunction);

                if(!reformulatedObjective || reformulatedObjective->linearTerms.size() != 1
                    || reformulatedObjective->direction != SHOT::E_ObjectiveFunctionDirection::Minimize)
                {
                    std::cout << "  FAILED: expected the mixed-term objective to remain 'minimize z' unchanged.\n";
                    passed = false;
                }

                if(env->reformulatedProblem->numericConstraints.size() != 1)
                {
                    std::cout << "  FAILED: expected the mixed-term defining constraint to remain (1 "
                                 "constraint), found "
                              << env->reformulatedProblem->numericConstraints.size() << ".\n";
                    passed = false;
                }

                passed = CheckSolvedObjective(env, expected,
                             "[" + solverName
                                 + "] mixed-term anti-epigraph kept as constraint, EpigraphStrategy=" + strategyName)
                    && passed;
            }
        }
    }

    return passed;
}

// Verifies Model.Reformulation.{Constraint,ObjectiveFunction}.Partition{Quadratic,Nonlinear}Terms: forcing
// partitioning (Always) vs disabling it (Never), for both the epigraph-constraint and direct-objective-function
// representations. Partitioning is what creates extra square-term/sum-term auxiliary constraints beyond the
// single defining constraint (epigraph) or none at all (direct); disabling it should always leave exactly the
// minimum: 1 constraint for epigraph form, 0 for direct form.
bool ModelTestObjectivePartitioningStrategy()
{
    bool passed = true;

    auto buildQuadratic = [](const std::shared_ptr<SHOT::Environment>& env)
    {
        // maximize -x^2 + 6x, x in [0, 2] -> concave, convex problem, optimum at x=2, value 8
        auto problem = std::make_shared<SHOT::Problem>(env);
        auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 2.0);
        problem->add(SHOT::Variables({ var_x }));

        auto objective
            = std::make_shared<SHOT::QuadraticObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Maximize);
        objective->add(std::make_shared<SHOT::QuadraticTerm>(-1.0, var_x, var_x));
        objective->add(std::make_shared<SHOT::LinearTerm>(6.0, var_x));
        problem->add(objective);

        return problem;
    };
    const double quadraticExpected = 8.0;

    auto buildNonlinearSum = [](const std::shared_ptr<SHOT::Environment>& env)
    {
        // maximize log(x) + log(y), x, y in [1, 5] -> concave sum, convex problem, optimum at x=y=5
        auto problem = std::make_shared<SHOT::Problem>(env);
        auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 1.0, 5.0);
        auto var_y = std::make_shared<SHOT::Variable>("y", 1, SHOT::E_VariableType::Real, 1.0, 5.0);
        problem->add(SHOT::Variables({ var_x, var_y }));

        auto nl_x = std::make_shared<SHOT::ExpressionVariable>(var_x);
        auto nl_y = std::make_shared<SHOT::ExpressionVariable>(var_y);
        SHOT::NonlinearExpressions terms;
        terms.add(std::make_shared<SHOT::ExpressionLog>(nl_x));
        terms.add(std::make_shared<SHOT::ExpressionLog>(nl_y));
        auto sum = std::make_shared<SHOT::ExpressionSum>(terms);

        auto objective = std::make_shared<SHOT::NonlinearObjectiveFunction>(
            SHOT::E_ObjectiveFunctionDirection::Maximize, sum, 0.0);
        problem->add(objective);

        return problem;
    };
    const double nonlinearSumExpected = 2.0 * std::log(5.0);

    auto setPartitioning
        = [](const std::string& settingName, ES_PartitionNonlinearSums strategy) -> std::function<void(SHOT::Solver&)>
    {
        return [settingName, strategy](SHOT::Solver& solver)
        { solver.updateSetting(settingName, static_cast<int>(strategy)); };
    };

    // Every sub-test below runs once per MIP solver available in this build.
    for(auto& [mipSolver, solverName] : AvailableMIPSolversForEpigraphTests())
    {
        std::cout << "\n===== MIP solver: " << solverName << " =====\n";

        // ── Quadratic, epigraph constraint, partitioning forced ─────────────────────────────────────
        std::cout << "\nSub-test: quadratic epigraph constraint, Constraint.PartitionQuadraticTerms=Always\n";
        {
            auto [solver, env] = SolveWithEpigraphStrategy(ES_ObjectiveEpigraphStrategy::EpigraphConstraint,
                buildQuadratic, /*solve*/ true, mipSolver,
                setPartitioning(
                    "Model.Reformulation.Constraint.PartitionQuadraticTerms", ES_PartitionNonlinearSums::Always));

            if(env->reformulatedProblem->numericConstraints.size() != 2)
            {
                std::cout << "  FAILED: expected 2 constraints (epigraph + square-term partition), found "
                          << env->reformulatedProblem->numericConstraints.size() << ".\n";
                passed = false;
            }

            passed = CheckSolvedObjective(
                         env, quadraticExpected, "[" + solverName + "] quadratic epigraph, partitioning forced")
                && passed;
        }

        // ── Quadratic, epigraph constraint, partitioning disabled -> exactly 1 constraint ───────────
        std::cout << "\nSub-test: quadratic epigraph constraint, Constraint.PartitionQuadraticTerms=Never\n";
        {
            auto [solver, env] = SolveWithEpigraphStrategy(ES_ObjectiveEpigraphStrategy::EpigraphConstraint,
                buildQuadratic, /*solve*/ true, mipSolver,
                setPartitioning(
                    "Model.Reformulation.Constraint.PartitionQuadraticTerms", ES_PartitionNonlinearSums::Never));

            if(env->reformulatedProblem->numericConstraints.size() != 1)
            {
                std::cout << "  FAILED: expected exactly 1 constraint (epigraph only, no partitioning), found "
                          << env->reformulatedProblem->numericConstraints.size() << ".\n";
                passed = false;
            }

            passed = CheckSolvedObjective(
                         env, quadraticExpected, "[" + solverName + "] quadratic epigraph, partitioning disabled")
                && passed;
        }

        // ── Quadratic, direct objective function, partitioning forced ───────────────────────────────
        std::cout
            << "\nSub-test: quadratic direct objective function, ObjectiveFunction.PartitionQuadraticTerms=Always\n";
        {
            auto [solver, env] = SolveWithEpigraphStrategy(ES_ObjectiveEpigraphStrategy::ObjectiveFunction,
                buildQuadratic, /*solve*/ true, mipSolver,
                setPartitioning("Model.Reformulation.ObjectiveFunction.PartitionQuadraticTerms",
                    ES_PartitionNonlinearSums::Always));

            // Gurobi/Cplex support native convex quadratic objectives, so for a convex problem
            // TaskReformulateProblem::reformulateObjectiveFunction takes a dedicated shortcut
            // (useConvexQuadraticObjective, ~line 547) that copies the quadratic objective straight through —
            // it never consults PartitionQuadraticTerms at all, so "Always" is a no-op there. Cbc/HiGHS lack
            // that native support and fall through to the generic path, which does respect the setting.
            bool solverBypassesObjectivePartitioning
                = mipSolver == ES_MIPSolver::Gurobi || mipSolver == ES_MIPSolver::Cplex;

            if(solverBypassesObjectivePartitioning)
            {
                if(!env->reformulatedProblem->objectiveFunction->properties.hasQuadraticTerms)
                {
                    std::cout << "  FAILED: expected the quadratic term to stay native (PartitionQuadraticTerms "
                                 "is bypassed for this solver's convex quadratic objective support).\n";
                    passed = false;
                }

                if(env->reformulatedProblem->numericConstraints.size() != 0)
                {
                    std::cout << "  FAILED: expected exactly 0 constraints (no partitioning applied), found "
                              << env->reformulatedProblem->numericConstraints.size() << ".\n";
                    passed = false;
                }
            }
            else
            {
                auto reformulatedObjective = std::dynamic_pointer_cast<SHOT::LinearObjectiveFunction>(
                    env->reformulatedProblem->objectiveFunction);

                if(!reformulatedObjective || reformulatedObjective->linearTerms.size() != 2)
                {
                    std::cout << "  FAILED: expected a 2-term direct LinearObjectiveFunction (quadratic term "
                                 "partitioned away).\n";
                    passed = false;
                }

                if(env->reformulatedProblem->numericConstraints.size() != 1)
                {
                    std::cout << "  FAILED: expected exactly 1 constraint (square-term partition), found "
                              << env->reformulatedProblem->numericConstraints.size() << ".\n";
                    passed = false;
                }
            }

            passed = CheckSolvedObjective(
                         env, quadraticExpected, "[" + solverName + "] quadratic direct, partitioning forced")
                && passed;
        }

        // ── Quadratic, direct objective function, partitioning disabled -> exactly 0 constraints ────
        std::cout
            << "\nSub-test: quadratic direct objective function, ObjectiveFunction.PartitionQuadraticTerms=Never\n";
        {
            auto [solver, env] = SolveWithEpigraphStrategy(ES_ObjectiveEpigraphStrategy::ObjectiveFunction,
                buildQuadratic, /*solve*/ true, mipSolver,
                setPartitioning(
                    "Model.Reformulation.ObjectiveFunction.PartitionQuadraticTerms", ES_PartitionNonlinearSums::Never));

            if(!env->reformulatedProblem->objectiveFunction->properties.hasQuadraticTerms)
            {
                std::cout << "  FAILED: expected the quadratic term to stay in a direct objective function.\n";
                passed = false;
            }

            if(env->reformulatedProblem->numericConstraints.size() != 0)
            {
                std::cout << "  FAILED: expected exactly 0 constraints (no partitioning, no epigraph), found "
                          << env->reformulatedProblem->numericConstraints.size() << ".\n";
                passed = false;
            }

            passed = CheckSolvedObjective(
                         env, quadraticExpected, "[" + solverName + "] quadratic direct, partitioning disabled")
                && passed;
        }

        // ── Nonlinear sum, epigraph constraint, partitioning forced ─────────────────────────────────
        std::cout << "\nSub-test: nonlinear sum epigraph constraint, Constraint.PartitionNonlinearTerms=Always\n";
        {
            auto [solver, env] = SolveWithEpigraphStrategy(ES_ObjectiveEpigraphStrategy::EpigraphConstraint,
                buildNonlinearSum, /*solve*/ true, mipSolver,
                setPartitioning(
                    "Model.Reformulation.Constraint.PartitionNonlinearTerms", ES_PartitionNonlinearSums::Always));

            // Each of the 2 summands gets its own auxiliary variable + constraint, plus the epigraph-defining
            // constraint itself: 3 constraints in total.
            if(env->reformulatedProblem->numericConstraints.size() != 3)
            {
                std::cout << "  FAILED: expected 3 constraints (epigraph + 2 sum-term partitions), found "
                          << env->reformulatedProblem->numericConstraints.size() << ".\n";
                passed = false;
            }

            passed = CheckSolvedObjective(
                         env, nonlinearSumExpected, "[" + solverName + "] nonlinear sum epigraph, partitioning forced")
                && passed;
        }

        // ── Nonlinear sum, epigraph constraint, partitioning disabled -> exactly 1 constraint ────────
        std::cout << "\nSub-test: nonlinear sum epigraph constraint, Constraint.PartitionNonlinearTerms=Never\n";
        {
            auto [solver, env] = SolveWithEpigraphStrategy(ES_ObjectiveEpigraphStrategy::EpigraphConstraint,
                buildNonlinearSum, /*solve*/ true, mipSolver,
                setPartitioning(
                    "Model.Reformulation.Constraint.PartitionNonlinearTerms", ES_PartitionNonlinearSums::Never));

            if(env->reformulatedProblem->numericConstraints.size() != 1)
            {
                std::cout << "  FAILED: expected exactly 1 constraint (epigraph only, no partitioning), found "
                          << env->reformulatedProblem->numericConstraints.size() << ".\n";
                passed = false;
            }

            passed = CheckSolvedObjective(env, nonlinearSumExpected,
                         "[" + solverName + "] nonlinear sum epigraph, partitioning disabled")
                && passed;
        }

        // ── Nonlinear sum, direct objective function, partitioning forced ───────────────────────────
        std::cout << "\nSub-test: nonlinear sum direct objective function, "
                     "ObjectiveFunction.PartitionNonlinearTerms=Always\n";
        {
            auto [solver, env] = SolveWithEpigraphStrategy(ES_ObjectiveEpigraphStrategy::ObjectiveFunction,
                buildNonlinearSum, /*solve*/ true, mipSolver,
                setPartitioning("Model.Reformulation.ObjectiveFunction.PartitionNonlinearTerms",
                    ES_PartitionNonlinearSums::Always));

            // Each of the 2 summands gets its own auxiliary variable + constraint, and (no epigraph here) the
            // objective becomes a direct 2-term LinearObjectiveFunction referencing both auxiliary variables.
            auto reformulatedObjective
                = std::dynamic_pointer_cast<SHOT::LinearObjectiveFunction>(env->reformulatedProblem->objectiveFunction);

            if(!reformulatedObjective || reformulatedObjective->linearTerms.size() != 2)
            {
                std::cout << "  FAILED: expected a 2-term direct LinearObjectiveFunction (sum terms partitioned "
                             "away).\n";
                passed = false;
            }

            if(env->reformulatedProblem->numericConstraints.size() != 2)
            {
                std::cout << "  FAILED: expected 2 constraints (one per sum-term partition), found "
                          << env->reformulatedProblem->numericConstraints.size() << ".\n";
                passed = false;
            }

            passed = CheckSolvedObjective(
                         env, nonlinearSumExpected, "[" + solverName + "] nonlinear sum direct, partitioning forced")
                && passed;
        }

        // ── Nonlinear sum, direct objective function, partitioning disabled -> exactly 0 constraints ─
        std::cout << "\nSub-test: nonlinear sum direct objective function, "
                     "ObjectiveFunction.PartitionNonlinearTerms=Never\n";
        {
            auto [solver, env] = SolveWithEpigraphStrategy(ES_ObjectiveEpigraphStrategy::ObjectiveFunction,
                buildNonlinearSum, /*solve*/ true, mipSolver,
                setPartitioning(
                    "Model.Reformulation.ObjectiveFunction.PartitionNonlinearTerms", ES_PartitionNonlinearSums::Never));

            auto reformulatedObjective = std::dynamic_pointer_cast<SHOT::NonlinearObjectiveFunction>(
                env->reformulatedProblem->objectiveFunction);

            if(!reformulatedObjective || !reformulatedObjective->properties.hasNonlinearExpression)
            {
                std::cout << "  FAILED: expected the sum to stay in a direct NonlinearObjectiveFunction.\n";
                passed = false;
            }

            if(env->reformulatedProblem->numericConstraints.size() != 0)
            {
                std::cout << "  FAILED: expected exactly 0 constraints (no partitioning, no epigraph), found "
                          << env->reformulatedProblem->numericConstraints.size() << ".\n";
                passed = false;
            }

            passed = CheckSolvedObjective(
                         env, nonlinearSumExpected, "[" + solverName + "] nonlinear sum direct, partitioning disabled")
                && passed;
        }
    }

    return passed;
}

// Verifies SHOT::SignomialElement::getBounds() (forward: the bound of variable^power, given the variable's own
// bound) and ::tightenBounds() (reverse: given a target bound for variable^power, tighten the variable's own
// bound) directly, for a range of integer, negative, and fractional powers -- both on domains that cross zero
// and ones that don't.
bool ModelTestSignomialElementBounds()
{
    bool passed = true;
    constexpr double tolerance = 1e-6;

    auto makeVariable = [](double lb, double ub)
    { return std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, lb, ub); };

    auto checkInterval
        = [&passed](const std::string& description, SHOT::Interval actual, double expectedLower, double expectedUpper)
    {
        std::cout << "  " << description << ": [" << actual.l() << ", " << actual.u() << "] (expected ["
                  << expectedLower << ", " << expectedUpper << "])\n";

        if(std::abs(actual.l() - expectedLower) > tolerance || std::abs(actual.u() - expectedUpper) > tolerance)
        {
            std::cout << "  FAILED: " << description << " did not match the expected interval.\n";
            passed = false;
        }
    };

    // ── getBounds(): forward bound of variable^power ────────────────────────────────────────────────
    std::cout << "\nSub-test: SignomialElement::getBounds() (forward: variable^power)\n";
    {
        struct Case
        {
            std::string description;
            double lb, ub, power, expectedLower, expectedUpper;
        };

        std::vector<Case> cases = {
            { "power=2 (even), domain crosses zero: x in [-3,4]", -3.0, 4.0, 2.0, 0.0, 16.0 },
            { "power=2 (even), positive-only domain: x in [2,5]", 2.0, 5.0, 2.0, 4.0, 25.0 },
            { "power=2 (even), negative-only domain: x in [-5,-2]", -5.0, -2.0, 2.0, 4.0, 25.0 },
            { "power=3 (odd), domain crosses zero: x in [-3,4]", -3.0, 4.0, 3.0, -27.0, 64.0 },
            { "power=3 (odd), negative-only domain: x in [-4,-2]", -4.0, -2.0, 3.0, -64.0, -8.0 },
            { "power=4 (even), domain crosses zero: x in [-3,2]", -3.0, 2.0, 4.0, 0.0, 81.0 },
            { "power=5 (odd), domain crosses zero: x in [-2,3]", -2.0, 3.0, 5.0, -32.0, 243.0 },
            { "power=-1, positive-only domain: x in [2,4]", 2.0, 4.0, -1.0, 0.25, 0.5 },
            { "power=0.5 (sqrt), positive-only domain: x in [4,9]", 4.0, 9.0, 0.5, 2.0, 3.0 },
            { "power=1.5, positive-only domain: x in [1,4]", 1.0, 4.0, 1.5, 1.0, 8.0 },
        };

        for(auto& C : cases)
        {
            auto variable = makeVariable(C.lb, C.ub);
            SHOT::SignomialElement element(variable, C.power);
            checkInterval(C.description, element.getBounds(), C.expectedLower, C.expectedUpper);
        }
    }

    // ── tightenBounds(): reverse -- given a target bound for variable^power, tighten variable itself ─
    std::cout << "\nSub-test: SignomialElement::tightenBounds() (reverse: variable from a variable^power bound)\n";
    {
        struct Case
        {
            std::string description;
            double initialLb, initialUb, power, targetLower, targetUpper, expectedLower, expectedUpper;
        };

        std::vector<Case> cases = {
            // An odd power's target bound crosses zero, so the tightened variable bound must too (not get clamped to
            // non-negative).
            { "power=3 (odd), target crosses zero: x^3 in [-27,8]", -100.0, 100.0, 3.0, -27.0, 8.0, -3.0, 2.0 },
            { "power=5 (odd), target crosses zero: x^5 in [-32,243]", -100.0, 100.0, 5.0, -32.0, 243.0, -2.0, 3.0 },
            // Even powers: only meaningful to tighten unambiguously when the variable's domain is already
            // one-signed (otherwise the positive and negative root branches can't both be represented by a
            // single interval).
            { "power=2 (even), non-negative domain: x^2 in [4,16]", 0.0, 100.0, 2.0, 4.0, 16.0, 2.0, 4.0 },
            { "power=4 (even), non-negative domain: x^4 in [16,81]", 0.0, 100.0, 4.0, 16.0, 81.0, 2.0, 3.0 },
            { "power=-1, target x^-1 in [0.25,0.5]", 1.0, 10.0, -1.0, 0.25, 0.5, 2.0, 4.0 },
            { "power=0.5 (sqrt), target sqrt(x) in [2,3]", 0.0, 100.0, 0.5, 2.0, 3.0, 4.0, 9.0 },
        };

        for(auto& C : cases)
        {
            auto variable = makeVariable(C.initialLb, C.initialUb);
            SHOT::SignomialElement element(variable, C.power);
            element.tightenBounds(SHOT::Interval(C.targetLower, C.targetUpper));
            checkInterval(C.description, SHOT::Interval(variable->lowerBound, variable->upperBound), C.expectedLower,
                C.expectedUpper);
        }
    }

    return passed;
}

// Verifies bound computation for LinearTerm, QuadraticTerm, and a representative set of NonlinearExpression
// subclasses -- complementing ModelTestSignomialElementBounds's coverage of SignomialElement.
bool ModelTestTermAndExpressionBounds()
{
    bool passed = true;
    constexpr double tolerance = 1e-6;

    auto makeVariable = [](double lb, double ub, int index = 0)
    { return std::make_shared<SHOT::Variable>("x", index, SHOT::E_VariableType::Real, lb, ub); };

    auto checkInterval
        = [&passed](const std::string& description, SHOT::Interval actual, double expectedLower, double expectedUpper)
    {
        std::cout << "  " << description << ": [" << actual.l() << ", " << actual.u() << "] (expected ["
                  << expectedLower << ", " << expectedUpper << "])\n";

        if(std::abs(actual.l() - expectedLower) > tolerance || std::abs(actual.u() - expectedUpper) > tolerance)
        {
            std::cout << "  FAILED: " << description << " did not match the expected interval.\n";
            passed = false;
        }
    };

    // ── LinearTerm: coefficient * variable ───────────────────────────────────────────────────────────
    std::cout << "\nSub-test: LinearTerm bounds (coefficient * variable)\n";
    {
        struct Case
        {
            std::string description;
            double coefficient, lb, ub, expectedLower, expectedUpper;
        };

        std::vector<Case> cases = {
            { "positive coefficient: 3*x, x in [-2,5]", 3.0, -2.0, 5.0, -6.0, 15.0 },
            { "negative coefficient, domain crosses zero: -2*x, x in [-2,5]", -2.0, -2.0, 5.0, -10.0, 4.0 },
            { "negative coefficient, positive-only domain: -2*x, x in [1,5]", -2.0, 1.0, 5.0, -10.0, -2.0 },
        };

        for(auto& C : cases)
        {
            auto variable = makeVariable(C.lb, C.ub);
            SHOT::LinearTerm term(C.coefficient, variable);
            SHOT::IntervalVector intervalVector = { variable->getBound() };
            checkInterval(C.description, term.calculate(intervalVector), C.expectedLower, C.expectedUpper);
        }
    }

    // ── QuadraticTerm: coefficient * variable1 * variable2 (square when variable1 == variable2) ──────
    std::cout << "\nSub-test: QuadraticTerm bounds (coefficient * variable1 * variable2)\n";
    {
        // Square term, domain crosses zero: coefficient * x^2, x in [-3,4].
        {
            auto var_x = makeVariable(-3.0, 4.0, 0);
            SHOT::QuadraticTerm term(1.0, var_x, var_x);
            SHOT::IntervalVector intervalVector = { var_x->getBound() };
            checkInterval(
                "square term, positive coefficient: x^2, x in [-3,4]", term.calculate(intervalVector), 0.0, 16.0);
        }
        {
            auto var_x = makeVariable(-3.0, 4.0, 0);
            SHOT::QuadraticTerm term(-2.0, var_x, var_x);
            SHOT::IntervalVector intervalVector = { var_x->getBound() };
            checkInterval(
                "square term, negative coefficient: -2*x^2, x in [-3,4]", term.calculate(intervalVector), -32.0, 0.0);
        }

        // Bilinear terms: coefficient * x * y, x and y distinct variables at indexes 0 and 1
        {
            auto var_x = makeVariable(1.0, 3.0, 0);
            auto var_y = makeVariable(2.0, 4.0, 1);
            SHOT::QuadraticTerm term(1.0, var_x, var_y);
            SHOT::IntervalVector intervalVector = { var_x->getBound(), var_y->getBound() };
            checkInterval("bilinear term, both positive domains: x*y, x in [1,3], y in [2,4]",
                term.calculate(intervalVector), 2.0, 12.0);
        }
        {
            auto var_x = makeVariable(-2.0, 3.0, 0);
            auto var_y = makeVariable(1.0, 4.0, 1);
            SHOT::QuadraticTerm term(1.0, var_x, var_y);
            SHOT::IntervalVector intervalVector = { var_x->getBound(), var_y->getBound() };
            checkInterval("bilinear term, one domain crosses zero: x*y, x in [-2,3], y in [1,4]",
                term.calculate(intervalVector), -8.0, 12.0);
        }
        {
            auto var_x = makeVariable(1.0, 3.0, 0);
            auto var_y = makeVariable(2.0, 4.0, 1);
            SHOT::QuadraticTerm term(-1.0, var_x, var_y);
            SHOT::IntervalVector intervalVector = { var_x->getBound(), var_y->getBound() };
            checkInterval("bilinear term, negative coefficient: -1*x*y, x in [1,3], y in [2,4]",
                term.calculate(intervalVector), -12.0, -2.0);
        }
    }

    // ── MonomialTerm: coefficient * variable1 * variable2 * ... (each variable implicitly to the power 1) ────
    std::cout << "\nSub-test: MonomialTerm bounds (coefficient * variable1 * variable2 * ...)\n";
    {
        {
            auto var_x = makeVariable(1.0, 3.0, 0);
            auto var_y = makeVariable(2.0, 4.0, 1);
            SHOT::MonomialTerm term(2.0, SHOT::Variables { var_x, var_y });
            SHOT::IntervalVector intervalVector = { var_x->getBound(), var_y->getBound() };
            checkInterval("two variables, both positive domains: 2*x*y, x in [1,3], y in [2,4]",
                term.calculate(intervalVector), 4.0, 24.0);
        }
        {
            auto var_x = makeVariable(-2.0, 3.0, 0);
            auto var_y = makeVariable(1.0, 2.0, 1);
            auto var_z = makeVariable(-1.0, 4.0, 2);
            SHOT::MonomialTerm term(1.0, SHOT::Variables { var_x, var_y, var_z });
            SHOT::IntervalVector intervalVector = { var_x->getBound(), var_y->getBound(), var_z->getBound() };
            checkInterval("three variables, mixed-sign domains: x*y*z, x in [-2,3], y in [1,2], z in [-1,4]",
                term.calculate(intervalVector), -16.0, 24.0);
        }
        {
            auto var_x = makeVariable(1.0, 3.0, 0);
            auto var_y = makeVariable(2.0, 4.0, 1);
            SHOT::MonomialTerm term(-1.0, SHOT::Variables { var_x, var_y });
            SHOT::IntervalVector intervalVector = { var_x->getBound(), var_y->getBound() };
            checkInterval(
                "negative coefficient: -1*x*y, x in [1,3], y in [2,4]", term.calculate(intervalVector), -12.0, -2.0);
        }
    }

    // ── SignomialTerm: coefficient * product of variable^power elements ─────────────────────────────
    // Complements ModelTestSignomialElementBounds's per-element coverage by combining several elements (with
    // different powers) into a single term, the way a real signomial expression like x^3*y^2 would appear.
    std::cout << "\nSub-test: SignomialTerm bounds (coefficient * product of variable^power elements)\n";
    {
        {
            auto var_e = makeVariable(-2.0, 1.0, 0);
            auto var_f = makeVariable(1.0, 2.0, 1);
            SHOT::SignomialElements elements = { std::make_shared<SHOT::SignomialElement>(var_e, 3.0),
                std::make_shared<SHOT::SignomialElement>(var_f, 2.0) };
            SHOT::SignomialTerm term(1.0, elements);
            SHOT::IntervalVector intervalVector = { var_e->getBound(), var_f->getBound() };
            checkInterval("e^3 * f^2, e in [-2,1] (odd power), f in [1,2] (even power)", term.calculate(intervalVector),
                -32.0, 4.0);
        }
        {
            auto var_e = makeVariable(-2.0, 1.0, 0);
            auto var_f = makeVariable(1.0, 2.0, 1);
            SHOT::SignomialElements elements = { std::make_shared<SHOT::SignomialElement>(var_e, 3.0),
                std::make_shared<SHOT::SignomialElement>(var_f, 2.0) };
            SHOT::SignomialTerm term(-1.0, elements);
            SHOT::IntervalVector intervalVector = { var_e->getBound(), var_f->getBound() };
            checkInterval(
                "negative coefficient: -1 * e^3 * f^2, same domains", term.calculate(intervalVector), -4.0, 32.0);
        }
    }

    // ── NonlinearExpressions: getBounds() (forward) and tightenBounds() (reverse), where supported ───
    std::cout << "\nSub-test: NonlinearExpression bounds (Exp, Log, SquareRoot, Square, Invert, Negate)\n";
    {
        struct Case
        {
            std::string description;
            std::function<SHOT::NonlinearExpressionPtr(SHOT::VariablePtr)> build;
            double lb, ub;
            double expectedLower, expectedUpper;
            // For the reverse (tightenBounds) check: the initial (wide) variable domain to tighten from.
            double tightenFromLb, tightenFromUb;
        };

        std::vector<Case> cases = {
            { "exp(x), x in [0,2]", [](SHOT::VariablePtr v)
                { return std::make_shared<SHOT::ExpressionExp>(std::make_shared<SHOT::ExpressionVariable>(v)); }, 0.0,
                2.0, 1.0, std::exp(2.0), -10.0, 10.0 },
            { "log(x), x in [1,10]", [](SHOT::VariablePtr v)
                { return std::make_shared<SHOT::ExpressionLog>(std::make_shared<SHOT::ExpressionVariable>(v)); }, 1.0,
                10.0, 0.0, std::log(10.0), 0.001, 1000.0 },
            { "sqrt(x), x in [4,9]", [](SHOT::VariablePtr v)
                { return std::make_shared<SHOT::ExpressionSquareRoot>(std::make_shared<SHOT::ExpressionVariable>(v)); },
                4.0, 9.0, 2.0, 3.0, 0.0, 100.0 },
            { "1/x, x in [2,4]", [](SHOT::VariablePtr v)
                { return std::make_shared<SHOT::ExpressionInvert>(std::make_shared<SHOT::ExpressionVariable>(v)); },
                2.0, 4.0, 0.25, 0.5, 1.0, 10.0 },
            { "-x, x in [-3,4]", [](SHOT::VariablePtr v)
                { return std::make_shared<SHOT::ExpressionNegate>(std::make_shared<SHOT::ExpressionVariable>(v)); },
                -3.0, 4.0, -4.0, 3.0, -100.0, 100.0 },
        };

        for(auto& C : cases)
        {
            // Forward: getBounds() from the expression's own (tight) domain.
            auto forwardVariable = makeVariable(C.lb, C.ub);
            auto forwardExpression = C.build(forwardVariable);
            checkInterval("forward " + C.description, forwardExpression->getBounds(), C.expectedLower, C.expectedUpper);

            // Reverse: tightenBounds() from a wide domain down to the same expected variable range.
            auto reverseVariable = makeVariable(C.tightenFromLb, C.tightenFromUb);
            auto reverseExpression = C.build(reverseVariable);
            reverseExpression->tightenBounds(SHOT::Interval(C.expectedLower, C.expectedUpper));
            checkInterval("reverse " + C.description,
                SHOT::Interval(reverseVariable->lowerBound, reverseVariable->upperBound), C.lb, C.ub);
        }

        // Square is tested separately since tightenBounds() can only unambiguously recover a variable domain
        // that's already one-signed (an even power's inverse can't represent two disjoint sign branches with a
        // single interval) -- same documented limitation as SignomialElement's even-power cases.
        {
            auto forwardVariable = makeVariable(-3.0, 4.0);
            SHOT::ExpressionSquare forwardExpression(std::make_shared<SHOT::ExpressionVariable>(forwardVariable));
            checkInterval("forward x^2, domain crosses zero: x in [-3,4]", forwardExpression.getBounds(), 0.0, 16.0);

            auto reverseVariable = makeVariable(0.0, 100.0);
            SHOT::ExpressionSquare reverseExpression(std::make_shared<SHOT::ExpressionVariable>(reverseVariable));
            reverseExpression.tightenBounds(SHOT::Interval(4.0, 16.0));
            checkInterval("reverse x^2, non-negative domain: target [4,16]",
                SHOT::Interval(reverseVariable->lowerBound, reverseVariable->upperBound), 2.0, 4.0);
        }
    }

    std::cout << "\nSub-test: ExpressionPower bounds (base^exponent, constant exponent)\n";
    {
        struct Case
        {
            std::string description;
            double power, lb, ub, expectedLower, expectedUpper;
        };

        std::vector<Case> forwardCases = {
            { "power=2 (even), domain crosses zero: x in [-3,4]", 2.0, -3.0, 4.0, 0.0, 16.0 },
            { "power=3 (odd), domain crosses zero: x in [-3,4]", 3.0, -3.0, 4.0, -27.0, 64.0 },
            { "power=5 (odd), domain crosses zero: x in [-2,3]", 5.0, -2.0, 3.0, -32.0, 243.0 },
        };

        for(auto& C : forwardCases)
        {
            auto baseVariable = makeVariable(C.lb, C.ub);
            SHOT::ExpressionPower expression(std::make_shared<SHOT::ExpressionVariable>(baseVariable),
                std::make_shared<SHOT::ExpressionConstant>(C.power));
            checkInterval("forward " + C.description, expression.getBounds(), C.expectedLower, C.expectedUpper);
        }

        // Reverse (tightenBounds()): an odd power's target bound crosses zero, so
        // the tightened base variable's bound must too, not get clamped to non-negative.
        struct ReverseCase
        {
            std::string description;
            double power, initialLb, initialUb, targetLower, targetUpper, expectedLower, expectedUpper;
        };

        std::vector<ReverseCase> reverseCases = {
            { "power=3 (odd), target crosses zero: base^3 in [-27,8]", 3.0, -100.0, 100.0, -27.0, 8.0, -3.0, 2.0 },
            { "power=5 (odd), target crosses zero: base^5 in [-32,243]", 5.0, -100.0, 100.0, -32.0, 243.0, -2.0, 3.0 },
            { "power=2 (even), non-negative domain: base^2 in [4,16]", 2.0, 0.0, 100.0, 4.0, 16.0, 2.0, 4.0 },
        };

        for(auto& C : reverseCases)
        {
            auto baseVariable = makeVariable(C.initialLb, C.initialUb);
            SHOT::ExpressionPower expression(std::make_shared<SHOT::ExpressionVariable>(baseVariable),
                std::make_shared<SHOT::ExpressionConstant>(C.power));

            expression.tightenBounds(SHOT::Interval(C.targetLower, C.targetUpper));
            checkInterval(C.description, SHOT::Interval(baseVariable->lowerBound, baseVariable->upperBound),
                C.expectedLower, C.expectedUpper);
        }
    }

    // ── ExpressionConstant and ExpressionVariable: the two leaf expression types ─────────────────────
    std::cout << "\nSub-test: ExpressionConstant and ExpressionVariable bounds\n";
    {
        SHOT::ExpressionConstant constantExpression(7.0);
        checkInterval("ExpressionConstant(7.0)", constantExpression.getBounds(), 7.0, 7.0);

        if(constantExpression.tightenBounds(SHOT::Interval(1.0, 2.0)))
        {
            std::cout << "  FAILED: ExpressionConstant::tightenBounds() should never report a change.\n";
            passed = false;
        }

        auto variable = makeVariable(-3.0, 4.0);
        SHOT::ExpressionVariable variableExpression(variable);
        checkInterval("ExpressionVariable, x in [-3,4]", variableExpression.getBounds(), -3.0, 4.0);

        variableExpression.tightenBounds(SHOT::Interval(0.0, 2.0));
        checkInterval("ExpressionVariable after tightenBounds([0,2])",
            SHOT::Interval(variable->lowerBound, variable->upperBound), 0.0, 2.0);
    }

    // ── ExpressionDivide: firstChild / secondChild ───────────────────────────────────────────────────
    std::cout << "\nSub-test: ExpressionDivide bounds (firstChild / secondChild)\n";
    {
        auto var_a = makeVariable(4.0, 10.0, 0);
        auto var_b = makeVariable(2.0, 5.0, 1);
        SHOT::ExpressionDivide forwardExpression(
            std::make_shared<SHOT::ExpressionVariable>(var_a), std::make_shared<SHOT::ExpressionVariable>(var_b));
        checkInterval("forward a/b, a in [4,10], b in [2,5]", forwardExpression.getBounds(), 0.8, 5.0);

        // Reverse: b is fixed at a single point here so the resulting target for a is unambiguous -- when both
        // operands are themselves ranges, tightening one still depends on the other's current range, same as
        // for any two-variable relationship.
        auto reverse_a = makeVariable(0.0, 100.0, 0);
        auto reverse_b = makeVariable(2.0, 2.0, 1);
        SHOT::ExpressionDivide reverseExpression(std::make_shared<SHOT::ExpressionVariable>(reverse_a),
            std::make_shared<SHOT::ExpressionVariable>(reverse_b));
        reverseExpression.tightenBounds(SHOT::Interval(2.0, 5.0));
        checkInterval("reverse a/b in [2,5], b fixed at 2 -> tighten a",
            SHOT::Interval(reverse_a->lowerBound, reverse_a->upperBound), 4.0, 10.0);
    }

    // ── ExpressionSum: sum of several child expressions ──────────────────────────────────────────────
    std::cout << "\nSub-test: ExpressionSum bounds (sum of children)\n";
    {
        auto var_x = makeVariable(-2.0, 3.0, 0);
        auto var_y = makeVariable(1.0, 4.0, 1);
        auto var_z = makeVariable(0.0, 2.0, 2);

        SHOT::NonlinearExpressions forwardChildren;
        forwardChildren.add(std::make_shared<SHOT::ExpressionVariable>(var_x));
        forwardChildren.add(std::make_shared<SHOT::ExpressionVariable>(var_y));
        forwardChildren.add(std::make_shared<SHOT::ExpressionVariable>(var_z));
        SHOT::ExpressionSum forwardExpression(forwardChildren);
        checkInterval("forward x+y+z, x in [-2,3], y in [1,4], z in [0,2]", forwardExpression.getBounds(), -1.0, 9.0);

        // Reverse: y and z are fixed, so the target range for the sum translates directly into a target range
        // for x alone.
        auto reverse_x = makeVariable(0.0, 100.0, 0);
        auto reverse_y = makeVariable(3.0, 3.0, 1);
        auto reverse_z = makeVariable(5.0, 5.0, 2);

        SHOT::NonlinearExpressions reverseChildren;
        reverseChildren.add(std::make_shared<SHOT::ExpressionVariable>(reverse_x));
        reverseChildren.add(std::make_shared<SHOT::ExpressionVariable>(reverse_y));
        reverseChildren.add(std::make_shared<SHOT::ExpressionVariable>(reverse_z));
        SHOT::ExpressionSum reverseExpression(reverseChildren);
        reverseExpression.tightenBounds(SHOT::Interval(12.0, 20.0));
        checkInterval("reverse x+3+5 in [12,20] -> tighten x",
            SHOT::Interval(reverse_x->lowerBound, reverse_x->upperBound), 4.0, 12.0);
    }

    // ── ExpressionProduct: product of several child expressions ─────────────────────────────────────
    std::cout << "\nSub-test: ExpressionProduct bounds (product of children)\n";
    {
        auto var_x = makeVariable(1.0, 3.0, 0);
        auto var_y = makeVariable(2.0, 4.0, 1);
        auto var_z = makeVariable(-1.0, 2.0, 2);

        SHOT::NonlinearExpressions forwardChildren;
        forwardChildren.add(std::make_shared<SHOT::ExpressionVariable>(var_x));
        forwardChildren.add(std::make_shared<SHOT::ExpressionVariable>(var_y));
        forwardChildren.add(std::make_shared<SHOT::ExpressionVariable>(var_z));
        SHOT::ExpressionProduct forwardExpression(forwardChildren);
        checkInterval("forward x*y*z, x in [1,3], y in [2,4], z in [-1,2]", forwardExpression.getBounds(), -12.0, 24.0);

        // Reverse: y and z are fixed, so the target range for the product translates directly into a target
        // range for x alone.
        auto reverse_x = makeVariable(0.0, 100.0, 0);
        auto reverse_y = makeVariable(2.0, 2.0, 1);
        auto reverse_z = makeVariable(3.0, 3.0, 2);

        SHOT::NonlinearExpressions reverseChildren;
        reverseChildren.add(std::make_shared<SHOT::ExpressionVariable>(reverse_x));
        reverseChildren.add(std::make_shared<SHOT::ExpressionVariable>(reverse_y));
        reverseChildren.add(std::make_shared<SHOT::ExpressionVariable>(reverse_z));
        SHOT::ExpressionProduct reverseExpression(reverseChildren);
        reverseExpression.tightenBounds(SHOT::Interval(24.0, 60.0));
        checkInterval("reverse x*2*3 in [24,60] -> tighten x",
            SHOT::Interval(reverse_x->lowerBound, reverse_x->upperBound), 4.0, 10.0);
    }

    // ── Trigonometric and absolute-value expressions: forward getBounds() only ──────────────────────
    // These expressions don't support reverse bound tightening at all (tightenBounds() always returns false
    // for each of them), so only the forward direction is meaningful to test here. Domains are chosen within a
    // single monotonic branch of each function to keep the expected interval unambiguous.
    std::cout << "\nSub-test: trigonometric and absolute-value expression bounds (forward only)\n";
    {
        double pi = M_PI;

        struct Case
        {
            std::string description;
            std::function<SHOT::NonlinearExpressionPtr(SHOT::VariablePtr)> build;
            double lb, ub, expectedLower, expectedUpper;
        };

        std::vector<Case> cases = {
            { "sin(x), x in [0,pi/2]", [](SHOT::VariablePtr v)
                { return std::make_shared<SHOT::ExpressionSin>(std::make_shared<SHOT::ExpressionVariable>(v)); }, 0.0,
                pi / 2.0, 0.0, 1.0 },
            { "cos(x), x in [0,pi/2]", [](SHOT::VariablePtr v)
                { return std::make_shared<SHOT::ExpressionCos>(std::make_shared<SHOT::ExpressionVariable>(v)); }, 0.0,
                pi / 2.0, 0.0, 1.0 },
            { "tan(x), x in [0,pi/4]", [](SHOT::VariablePtr v)
                { return std::make_shared<SHOT::ExpressionTan>(std::make_shared<SHOT::ExpressionVariable>(v)); }, 0.0,
                pi / 4.0, 0.0, 1.0 },
            { "asin(x), x in [0,1]", [](SHOT::VariablePtr v)
                { return std::make_shared<SHOT::ExpressionArcSin>(std::make_shared<SHOT::ExpressionVariable>(v)); },
                0.0, 1.0, 0.0, pi / 2.0 },
            { "acos(x), x in [0,1]", [](SHOT::VariablePtr v)
                { return std::make_shared<SHOT::ExpressionArcCos>(std::make_shared<SHOT::ExpressionVariable>(v)); },
                0.0, 1.0, 0.0, pi / 2.0 },
            { "atan(x), x in [0,1]", [](SHOT::VariablePtr v)
                { return std::make_shared<SHOT::ExpressionArcTan>(std::make_shared<SHOT::ExpressionVariable>(v)); },
                0.0, 1.0, 0.0, pi / 4.0 },
            { "abs(x), x in [-3,4]", [](SHOT::VariablePtr v)
                { return std::make_shared<SHOT::ExpressionAbs>(std::make_shared<SHOT::ExpressionVariable>(v)); }, -3.0,
                4.0, 0.0, 4.0 },
        };

        for(auto& C : cases)
        {
            auto variable = makeVariable(C.lb, C.ub);
            auto expression = C.build(variable);
            checkInterval(C.description, expression->getBounds(), C.expectedLower, C.expectedUpper);

            if(expression->tightenBounds(SHOT::Interval(C.expectedLower, C.expectedUpper)))
            {
                std::cout << "  FAILED: " << C.description << ": tightenBounds() should always return false.\n";
                passed = false;
            }
        }
    }

    return passed;
}

// Verifies bound tightening on a single constraint that combines all five term categories at once -- linear,
// quadratic, monomial, signomial, and a general nonlinear expression -- exercising the actual production bound
// tightening pass (Problem::doFBBT()) rather than the individual term/expression classes in isolation, the way
// the tests above do.
//
// The constraint is total - 3*a - b^2 - c*d - e^3*f^2 - exp(g) = 0, i.e. total = 3*a + b^2 + c*d + e^3*f^2 +
// exp(g), with each variable's own domain chosen so every term's contribution is independently hand-computable:
//   3*a,        a in [1,2]             -> [3,6]
//   b^2,        b in [2,3]             -> [4,9]
//   c*d,        c in [1,2], d in [3,4] -> [3,8]
//   e^3*f^2,    e in [-2,1], f in [1,2] -> [-32,4]
//   exp(g),     g in [0,1]             -> [1, e]
// Summing these gives total's expected tightened bound: [-21, 27+e] ~= [-21, 29.71828].
bool ModelTestMixedTermBoundTightening()
{
    bool passed = true;

    auto solver = std::make_unique<SHOT::Solver>();
    auto env = solver->getEnvironment();

    auto problem = std::make_shared<SHOT::Problem>(env);

    auto var_total = std::make_shared<SHOT::Variable>("total", 0, SHOT::E_VariableType::Real, -1000.0, 1000.0);
    auto var_a = std::make_shared<SHOT::Variable>("a", 1, SHOT::E_VariableType::Real, 1.0, 2.0);
    auto var_b = std::make_shared<SHOT::Variable>("b", 2, SHOT::E_VariableType::Real, 2.0, 3.0);
    auto var_c = std::make_shared<SHOT::Variable>("c", 3, SHOT::E_VariableType::Real, 1.0, 2.0);
    auto var_d = std::make_shared<SHOT::Variable>("d", 4, SHOT::E_VariableType::Real, 3.0, 4.0);
    auto var_e = std::make_shared<SHOT::Variable>("e", 5, SHOT::E_VariableType::Real, -2.0, 1.0);
    auto var_f = std::make_shared<SHOT::Variable>("f", 6, SHOT::E_VariableType::Real, 1.0, 2.0);
    auto var_g = std::make_shared<SHOT::Variable>("g", 7, SHOT::E_VariableType::Real, 0.0, 1.0);
    problem->add(SHOT::Variables { var_total, var_a, var_b, var_c, var_d, var_e, var_f, var_g });

    // A dummy objective is required for a valid problem; it plays no part in the bound tightening being tested.
    auto objective = std::make_shared<SHOT::LinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Minimize);
    objective->add(std::make_shared<SHOT::LinearTerm>(1.0, var_total));
    problem->add(objective);

    auto constraint = std::make_shared<SHOT::NonlinearConstraint>(0, "mixed", 0.0, 0.0);
    constraint->add(std::make_shared<SHOT::LinearTerm>(1.0, var_total));
    constraint->add(std::make_shared<SHOT::LinearTerm>(-3.0, var_a));
    constraint->add(std::make_shared<SHOT::QuadraticTerm>(-1.0, var_b, var_b));
    constraint->add(std::make_shared<SHOT::MonomialTerm>(-1.0, SHOT::Variables { var_c, var_d }));
    constraint->add(std::make_shared<SHOT::SignomialTerm>(-1.0,
        SHOT::SignomialElements { std::make_shared<SHOT::SignomialElement>(var_e, 3.0),
            std::make_shared<SHOT::SignomialElement>(var_f, 2.0) }));
    constraint->add(std::make_shared<SHOT::ExpressionNegate>(
        std::make_shared<SHOT::ExpressionExp>(std::make_shared<SHOT::ExpressionVariable>(var_g))));
    problem->add(constraint);

    problem->finalize();
    problem->doFBBT();

    double expectedLower = -21.0;
    double expectedUpper = 6.0 + 9.0 + 8.0 + 4.0 + std::exp(1.0);
    constexpr double tolerance = 0.1;

    std::cout << "\nSub-test: bound tightening on a constraint mixing linear, quadratic, monomial, signomial, "
                 "and nonlinear-expression terms\n";
    std::cout << "  total: [" << var_total->lowerBound << ", " << var_total->upperBound << "] (expected ["
              << expectedLower << ", " << expectedUpper << "])\n";
    std::cout << "  a: [" << var_a->lowerBound << ", " << var_a->upperBound << "]\n";
    std::cout << "  b: [" << var_b->lowerBound << ", " << var_b->upperBound << "]\n";
    std::cout << "  c: [" << var_c->lowerBound << ", " << var_c->upperBound << "]\n";
    std::cout << "  d: [" << var_d->lowerBound << ", " << var_d->upperBound << "]\n";
    std::cout << "  e: [" << var_e->lowerBound << ", " << var_e->upperBound << "]\n";
    std::cout << "  f: [" << var_f->lowerBound << ", " << var_f->upperBound << "]\n";
    std::cout << "  g: [" << var_g->lowerBound << ", " << var_g->upperBound << "]\n";

    if(std::abs(var_total->lowerBound - expectedLower) > tolerance
        || std::abs(var_total->upperBound - expectedUpper) > tolerance)
    {
        std::cout << "  FAILED: expected total to be tightened to approximately [" << expectedLower << ", "
                  << expectedUpper << "].\n";
        passed = false;
    }

    // None of the individual contributing variables should have been affected -- only total's own bound
    // depends on all of the others combined.
    if(var_a->lowerBound != 1.0 || var_a->upperBound != 2.0)
    {
        std::cout << "  FAILED: variable a's bound should not have changed.\n";
        passed = false;
    }

    return passed;
}
