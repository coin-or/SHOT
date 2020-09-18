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

#include "../src/Model/Variables.h"
#include "../src/Model/Terms.h"
#include "../src/Model/Constraints.h"
#include "../src/Model/NonlinearExpressions.h"
#include "../src/Model/Problem.h"

#include "../src/Tasks/TaskReformulateProblem.h"

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
bool ModelTestConvexity();
bool ModelTestCopy();

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
    auto gradientLinear = linearConstraint->calculateGradient(point, true);

    for(auto const& G : gradientLinear)
    {
        std::cout << G.first->name << ": " << G.second << '\n';
    }

    std::cout << "\nCalculating Hessian for function in linear constraint (there should be none):\n";
    auto hessianLinear = linearConstraint->calculateHessian(point, true);

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
    auto gradientQuadratic = quadraticConstraint->calculateGradient(point, true);

    for(auto const& G : gradientQuadratic)
    {
        std::cout << G.first->name << ": " << G.second << '\n';
    }

    std::cout << "\nCalculating hessian for function in quadratic constraint:\n";
    auto hessianQuadratic = quadraticConstraint->calculateHessian(point, true);

    for(auto const& H : hessianQuadratic)
    {
        std::cout << "(" + H.first.first->name << "," << H.first.second->name << "): " << H.second << '\n';
    }

    std::cout << "\nCalculating gradient for function in first nonlinear constraint:\n";
    auto gradientNonlinear = nonlinearConstraint->calculateGradient(point, true);

    for(auto const& G : gradientNonlinear)
    {
        std::cout << G.first->name << ":  " << G.second << '\n';
    }

    std::cout << "\nCalculating hessian for function in first nonlinear constraint (there should be one element):\n";
    auto hessianNonlinear = nonlinearConstraint->calculateHessian(point, true);

    for(auto const& H : hessianNonlinear)
    {
        std::cout << "(" + H.first.first->name << "," << H.first.second->name << "): " << H.second << '\n';
    }

    std::cout << "\nCalculating gradient for function in second nonlinear constraint:\n";
    auto gradientNonlinear2 = nonlinearConstraint2->calculateGradient(point, true);

    for(auto const& G : gradientNonlinear2)
    {
        std::cout << G.first->name << ":  " << G.second << '\n';
    }

    std::cout << "\nCalculating hessian for function in second nonlinear constraint:\n";
    auto hessianNonlinear2 = nonlinearConstraint2->calculateHessian(point, true);

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
