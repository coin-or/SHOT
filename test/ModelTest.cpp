/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/
#include <iostream>
#include "Terms.h"
#include "Constraints.h"
#include "Model/NonlinearExpressions.h"
#include "ObjectiveFunction.h"

using namespace SHOT;

bool ModelTestVariables();
bool ModelTestTerms();
bool ModelTestNonlinearExpressions();
bool ModelTestObjective();
bool ModelTestConstraints();

int ModelTest(int argc, char *argv[])
{
    int defaultchoice = 1;

    int choice = defaultchoice;

    if (argc > 1)
    {
        if (sscanf(argv[1], "%d", &choice) != 1)
        {
            printf("Couldn't parse that input as a number\n");
            return -1;
        }
    }

    bool passed = true;

    switch (choice)
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
    default:
        passed = false;
        std::cout << "Test #" << choice << " does not exist!\n";
    }

    if (passed)
        return 0;
    else
        return -1;
}

bool ModelTestVariables()
{
    bool passed = true;

    std::cout << "Creating variable:" << std::endl;
    VariablePtr var_x(new Variable());
    var_x->name = "x";
    var_x->index = 0;
    var_x->type = E_VariableType::Real;
    var_x->lowerBound = 0.0;
    var_x->upperBound = 100.0;

    std::cout << "Variable " << var_x << " created." << std::endl;

    VectorDouble point;
    point.push_back(2.0);

    double value = var_x->calculate(point);
    double realValue = point.at(0);

    std::cout << "Calculating variable value: " << value << " (should be equal to " << realValue << ")." << std::endl;

    if (value != realValue)
        passed = false;

    return passed;
}

bool ModelTestTerms()
{
    bool passed = true;

    std::cout << "Creating variable:" << std::endl;
    VariablePtr var_x(new Variable());
    var_x->name = "x";
    var_x->index = 0;
    var_x->type = E_VariableType::Real;
    var_x->lowerBound = 0.0;
    var_x->upperBound = 100.0;

    std::cout << "Variable " << var_x->name << " created." << std::endl;

    VectorDouble point;
    point.push_back(3.0);

    std::cout << "Creating linear term:" << std::endl;
    LinearTermPtr linearTerm(new LinearTerm());
    linearTerm->coefficient = 1.1;
    linearTerm->variable = var_x;

    std::cout << "Linear term created: " << linearTerm << std::endl;

    double value = linearTerm->calculate(point);
    double realValue = linearTerm->coefficient * point.at(0);

    std::cout << "Calculating term value: " << value << " (should be equal to " << realValue << ")." << std::endl;

    if (value != realValue)
        passed = false;

    std::cout << "Creating quadratic term:" << std::endl;

    VariablePtr var_y(new Variable());
    var_y->name = "y";
    var_y->index = 1;
    var_y->type = E_VariableType::Integer;
    var_y->lowerBound = 0.0;
    var_y->upperBound = 1.0;

    QuadraticTermPtr quadraticTerm1(new QuadraticTerm());
    quadraticTerm1->coefficient = 1;
    quadraticTerm1->firstVariable = var_x;
    quadraticTerm1->secondVariable = var_y;

    std::cout << "Quadratic term created:" << quadraticTerm1 << std::endl;

    VectorDouble point2;
    point2.push_back(2.0);
    point2.push_back(3.0);

    value = quadraticTerm1->calculate(point2);
    realValue = quadraticTerm1->coefficient * point2.at(0) * point2.at(1);

    std::cout << "Calculating term value: " << value << " (should be equal to " << realValue << ")." << std::endl;

    if (value != realValue)
        passed = false;

    std::cout << "Creating quadratic term:" << std::endl;

    QuadraticTermPtr quadraticTerm2(new QuadraticTerm());
    quadraticTerm2->coefficient = 1;
    quadraticTerm2->firstVariable = var_x;
    quadraticTerm2->secondVariable = var_x;

    std::cout << "Quadratic term created:" << quadraticTerm2 << std::endl;

    value = quadraticTerm2->calculate(point2);
    realValue = quadraticTerm2->coefficient * point2.at(0) * point2.at(0);

    std::cout << "Calculating term value: " << value << " (should be equal to " << realValue << ")." << std::endl;

    if (value != realValue)
        passed = false;

    return passed;
}

bool ModelTestNonlinearExpressions()
{
    bool passed = true;

    std::cout << "Creating first variable:" << std::endl;
    VariablePtr var_x(new Variable());
    var_x->name = "x";
    var_x->index = 0;
    var_x->type = E_VariableType::Real;
    var_x->lowerBound = 0.0;
    var_x->upperBound = 100.0;
    std::shared_ptr<ExpressionVariable> expressionVariable_x(new ExpressionVariable(var_x));
    std::cout << "Variable " << var_x->name << " created." << std::endl;

    std::cout << "Creating second variable:" << std::endl;
    VariablePtr var_y(new Variable());
    var_y->name = "y";
    var_y->index = 1;
    var_y->type = E_VariableType::Integer;
    var_y->lowerBound = 0.0;
    var_y->upperBound = 1.0;
    std::shared_ptr<ExpressionVariable> expressionVariable_y(new ExpressionVariable(var_y));
    std::cout << "Variable " << var_y->name << " created." << std::endl;

    VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);

    std::cout << "Creating negate expression" << std::endl;

    NonlinearExpressionPtr exprNegate(new ExpressionNegate(expressionVariable_x));

    std::cout << "Negate expression " << exprNegate << " created" << std::endl;

    auto value = exprNegate->calculate(point);
    double realValue = -var_x->calculate(point);

    std::cout
        << "Calculating expression value: " << value << " (should be equal to " << realValue << ")." << std::endl;

    if (value != realValue)
        passed = false;

    std::cout << "Creating plus expression" << std::endl;

    NonlinearExpressionPtr exprPlus(new ExpressionPlus(expressionVariable_x, expressionVariable_y));

    std::cout << "Plus expression " << exprPlus << " created" << std::endl;

    value = exprPlus->calculate(point);
    realValue = var_x->calculate(point) + var_y->calculate(point);

    std::cout
        << "Calculating expression value: " << value << " (should be equal to " << realValue << ")." << std::endl;

    if (value != realValue)
        passed = false;

    std::cout << "Creating power expression" << std::endl;

    NonlinearExpressionPtr exprPower(new ExpressionPower(exprPlus, exprNegate));

    std::cout << "Power expression " << exprPower << " created " << std::endl;

    value = exprPower->calculate(point);
    realValue = pow(var_x->calculate(point) + var_y->calculate(point), -var_x->calculate(point));

    std::cout
        << "Calculating expression value: " << value << " (should be equal to " << realValue << ")." << std::endl;

    if (value != realValue)
        passed = false;

    return passed;
}

bool ModelTestObjective()
{
    /*
    bool passed = true;

    std::cout << "Creating first variable:" << std::endl;
    VariablePtr var_x(new Variable());
    var_x->name = "x";
    var_x->index = 0;
    var_x->type = E_VariableType::Real;
    var_x->lowerBound = 0.0;
    var_x->upperBound = 100.0;
    std::shared_ptr<ExpressionVariable> expressionVariable_x(new ExpressionVariable(var_x));
    std::cout << "Variable " << var_x->name << " created." << std::endl;

    std::cout << "Creating second variable:" << std::endl;
    VariablePtr var_y(new Variable());
    var_y->name = "y";
    var_y->index = 1;
    var_y->type = E_VariableType::Integer;
    var_y->lowerBound = 0.0;
    var_y->upperBound = 1.0;
    std::shared_ptr<ExpressionVariable> expressionVariable_y(new ExpressionVariable(var_y));
    std::cout << "Variable " << var_y->name << " created." << std::endl;

    std::cout << "Creating linear terms" << std::endl;

    LinearTermPtr linearTerm1(new LinearTerm());
    linearTerm1->coefficient = -1.0;
    linearTerm1->variable = var_x;

    LinearTermPtr linearTerm2(new LinearTerm());
    linearTerm2->coefficient = 1.2;
    linearTerm2->variable = var_y;

    LinearTerms linearTerms;
    linearTerms.terms.push_back(linearTerm1);
    linearTerms.terms.push_back(linearTerm2);

    std::cout << "Linear terms " << linearTerms << " created" << std::endl;

    std::cout << "Creating quadratic terms" << std::endl;

    QuadraticTermPtr quadraticTerm1(new QuadraticTerm());
    quadraticTerm1->coefficient = 1;
    quadraticTerm1->firstVariable = var_x;
    quadraticTerm1->secondVariable = var_y;

    QuadraticTermPtr quadraticTerm2(new QuadraticTerm());
    quadraticTerm2->coefficient = 2;
    quadraticTerm2->firstVariable = var_x;
    quadraticTerm2->secondVariable = var_x;

    QuadraticTerms quadraticTerms;
    quadraticTerms.terms.push_back(quadraticTerm1);
    quadraticTerms.terms.push_back(quadraticTerm2);

    std::cout << "Quadratic terms " << quadraticTerms << " created" << std::endl;

    std::cout << "Creating quadratic constraint:" << std::endl;
    QuadraticConstraintPtr quadraticConstraint(new QuadraticConstraint());
    quadraticConstraint->constraintIndex = 0;
    quadraticConstraint->constraintName = "quadconstr";
    quadraticConstraint->valueLHS = -10;
    quadraticConstraint->valueRHS = 20;
    quadraticConstraint->linearTerms = linearTerms;
    quadraticConstraint->quadraticTerms = quadraticTerms;

    std::cout << "Quadratic constraint created" << std::endl;

    VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);

    std::cout << "Creating nonlinear expression:" << std::endl;
    std::vector<NonlinearExpressionPtr> expressions;
    expressions.push_back(expressionVariable_x);
    expressions.push_back(expressionVariable_x);
    expressions.push_back(expressionVariable_y);
    NonlinearExpressionPtr exprTimes(new ExpressionTimes(expressions));
    std::cout << "Nonlinear expression " << exprTimes << " created" << std::endl;

    std::cout << "Creating objective function:" << std::endl;
    ObjectiveFunctionPtr nonlinearObjective(new NonlinearObjectiveFunction());
    nonlinearObjective->direction = E_ObjectiveFunctionDirection::Minimize;

    nonlinearConstraint->valueLHS = -10;
    nonlinearConstraint->valueRHS = 20;
    nonlinearConstraint->linearTerms = linearTerms;
    nonlinearConstraint->quadraticTerms = quadraticTerms;
    nonlinearConstraint->nonlinearExpression = exprTimes;

    std::cout << "Nonlinear constraint " << nonlinearConstraint << " created" << std::endl;

    constraintValue = nonlinearConstraint->calculateNumericValue(point);
    realValue = linearTerm1->coefficient * point.at(0) + linearTerm2->coefficient * point.at(1) + quadraticTerm1->coefficient * point.at(0) * point.at(1) +
                +quadraticTerm2->coefficient * point.at(0) * point.at(0) + point.at(0) * point.at(0) * point.at(1);

    std::cout
        << "Calculating constraint value: " << constraintValue.functionValue << " (should be equal to " << realValue << ")." << std::endl;

    if (constraintValue.functionValue != realValue)
        passed = false;

    bool isFulfilled = nonlinearConstraint->isFulfilled(point);

    std::cout << "Is nonlinear constraint valid in point (x,y)=(" << point.at(0) << "," << point.at(1) << ")? ";
    std::cout << (nonlinearConstraint->isFulfilled(point) ? "yes" : "no");
    std::cout << ". Function value: " << nonlinearConstraint->calculateFunctionValue(point);
    std::cout << std::endl;

    if (isFulfilled)
        passed = false;

    point.at(0) = 1.0;
    point.at(1) = 1.0;

    isFulfilled = nonlinearConstraint->isFulfilled(point);

    std::cout << "Is nonlinear constraint valid in point (x,y)=(" << point.at(0) << "," << point.at(1) << ")? ";
    std::cout << (nonlinearConstraint->isFulfilled(point) ? "yes" : "no");
    std::cout << ". Function value: " << nonlinearConstraint->calculateFunctionValue(point);
    std::cout << std::endl;

    if (!isFulfilled)
        passed = false;

    return passed;*/
    return true;
}

bool ModelTestConstraints()
{
    bool passed = true;

    std::cout << "Creating first variable:" << std::endl;
    VariablePtr var_x(new Variable());
    var_x->name = "x";
    var_x->index = 0;
    var_x->type = E_VariableType::Real;
    var_x->lowerBound = 0.0;
    var_x->upperBound = 100.0;
    std::shared_ptr<ExpressionVariable> expressionVariable_x(new ExpressionVariable(var_x));
    std::cout << "Variable " << var_x->name << " created." << std::endl;

    std::cout << "Creating second variable:" << std::endl;
    VariablePtr var_y(new Variable());
    var_y->name = "y";
    var_y->index = 1;
    var_y->type = E_VariableType::Integer;
    var_y->lowerBound = 0.0;
    var_y->upperBound = 1.0;
    std::shared_ptr<ExpressionVariable> expressionVariable_y(new ExpressionVariable(var_y));
    std::cout << "Variable " << var_y->name << " created." << std::endl;

    std::cout << "Creating linear terms" << std::endl;

    LinearTermPtr linearTerm1(new LinearTerm());
    linearTerm1->coefficient = -1.0;
    linearTerm1->variable = var_x;

    LinearTermPtr linearTerm2(new LinearTerm());
    linearTerm2->coefficient = 1.2;
    linearTerm2->variable = var_y;

    LinearTerms linearTerms;
    linearTerms.terms.push_back(linearTerm1);
    linearTerms.terms.push_back(linearTerm2);

    std::cout << "Linear terms " << linearTerms << " created" << std::endl;

    std::cout << "Creating quadratic terms" << std::endl;

    QuadraticTermPtr quadraticTerm1(new QuadraticTerm());
    quadraticTerm1->coefficient = 1;
    quadraticTerm1->firstVariable = var_x;
    quadraticTerm1->secondVariable = var_y;

    QuadraticTermPtr quadraticTerm2(new QuadraticTerm());
    quadraticTerm2->coefficient = 2;
    quadraticTerm2->firstVariable = var_x;
    quadraticTerm2->secondVariable = var_x;

    QuadraticTerms quadraticTerms;
    quadraticTerms.terms.push_back(quadraticTerm1);
    quadraticTerms.terms.push_back(quadraticTerm2);

    std::cout << "Quadratic terms " << quadraticTerms << " created" << std::endl;

    std::cout << "Creating quadratic constraint:" << std::endl;
    QuadraticConstraintPtr quadraticConstraint(new QuadraticConstraint());
    quadraticConstraint->constraintIndex = 0;
    quadraticConstraint->constraintName = "quadconstr";
    quadraticConstraint->valueLHS = -10;
    quadraticConstraint->valueRHS = 20;
    quadraticConstraint->linearTerms = linearTerms;
    quadraticConstraint->quadraticTerms = quadraticTerms;

    std::cout << "Quadratic constraint created" << std::endl;

    VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);

    auto constraintValue = quadraticConstraint->calculateNumericValue(point);
    double realValue = linearTerm1->coefficient * point.at(0) + linearTerm2->coefficient * point.at(1) + quadraticTerm1->coefficient * point.at(0) * point.at(1) +
                       +quadraticTerm2->coefficient * point.at(0) * point.at(0);

    std::cout
        << "Calculating constraint value: " << constraintValue.functionValue << " (should be equal to " << realValue << ")." << std::endl;

    if (constraintValue.functionValue != realValue)
        passed = false;

    std::cout << "Creating nonlinear expression:" << std::endl;
    std::vector<NonlinearExpressionPtr> expressions;
    expressions.push_back(expressionVariable_x);
    expressions.push_back(expressionVariable_x);
    expressions.push_back(expressionVariable_y);
    NonlinearExpressionPtr exprTimes(new ExpressionTimes(expressions));
    std::cout << "Nonlinear expression " << exprTimes << " created" << std::endl;

    std::cout << "Creating nonlinear constraint:" << std::endl;
    NonlinearConstraintPtr nonlinearConstraint(new NonlinearConstraint());
    nonlinearConstraint->constraintIndex = 0;
    nonlinearConstraint->constraintName = "quadconstr";
    nonlinearConstraint->valueLHS = -10;
    nonlinearConstraint->valueRHS = 20;
    nonlinearConstraint->linearTerms = linearTerms;
    nonlinearConstraint->quadraticTerms = quadraticTerms;
    nonlinearConstraint->nonlinearExpression = exprTimes;

    std::cout << "Nonlinear constraint " << nonlinearConstraint << " created" << std::endl;

    constraintValue = nonlinearConstraint->calculateNumericValue(point);
    realValue = linearTerm1->coefficient * point.at(0) + linearTerm2->coefficient * point.at(1) + quadraticTerm1->coefficient * point.at(0) * point.at(1) +
                +quadraticTerm2->coefficient * point.at(0) * point.at(0) + point.at(0) * point.at(0) * point.at(1);

    std::cout
        << "Calculating constraint value: " << constraintValue.functionValue << " (should be equal to " << realValue << ")." << std::endl;

    if (constraintValue.functionValue != realValue)
        passed = false;

    bool isFulfilled = nonlinearConstraint->isFulfilled(point);

    std::cout << "Is nonlinear constraint valid in point (x,y)=(" << point.at(0) << "," << point.at(1) << ")? ";
    std::cout << (nonlinearConstraint->isFulfilled(point) ? "yes" : "no");
    std::cout << ". Function value: " << nonlinearConstraint->calculateFunctionValue(point);
    std::cout << std::endl;

    if (isFulfilled)
        passed = false;

    point.at(0) = 1.0;
    point.at(1) = 1.0;

    isFulfilled = nonlinearConstraint->isFulfilled(point);

    std::cout << "Is nonlinear constraint valid in point (x,y)=(" << point.at(0) << "," << point.at(1) << ")? ";
    std::cout << (nonlinearConstraint->isFulfilled(point) ? "yes" : "no");
    std::cout << ". Function value: " << nonlinearConstraint->calculateFunctionValue(point);
    std::cout << std::endl;

    if (!isFulfilled)
        passed = false;

    return passed;
}