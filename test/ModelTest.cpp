/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/
#include <iostream>
#include "Model/Terms.h"
#include "Model/Constraints.h"
#include "Model/NonlinearExpressions.h"
#include "Model/ObjectiveFunction.h"
#include "Model/OptimizationProblem.h"

using namespace SHOT;

bool ModelTestVariables();
bool ModelTestTerms();
bool ModelTestNonlinearExpressions();
bool ModelTestObjective();
bool ModelTestConstraints();
bool ModelTestProblem();

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
    case 6:
        passed = ModelTestProblem();
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

    std::cout << "Creating variable:\n";
    VariablePtr var_x = std::make_shared<Variable>("x", 0, E_VariableType::Real, 0.0, 100.0);
    std::cout << "Variable " << var_x << " created.\n";

    VectorDouble point;
    point.push_back(2.0);

    double value = var_x->calculate(point);
    double realValue = point.at(0);

    std::cout << "Calculating variable value: " << value << " (should be equal to " << realValue << ").\n";

    if (value != realValue)
        passed = false;

    return passed;
}

bool ModelTestTerms()
{
    bool passed = true;

    VariablePtr var_x = std::make_shared<Variable>("x", 0, E_VariableType::Real, 0.0, 100.0);
    VariablePtr var_y = std::make_shared<Variable>("y", 1, E_VariableType::Integer, 0.0, 1.0);

    VectorDouble point;
    point.push_back(3.0);

    std::cout << "Creating linear term: \n";
    LinearTermPtr linearTerm = std::make_shared<LinearTerm>(-1, var_x);
    std::cout << "Linear term created: " << linearTerm << "\n";

    double value = linearTerm->calculate(point);
    double realValue = linearTerm->coefficient * point.at(0);

    std::cout << "Calculating term value: " << value << " (should be equal to " << realValue << ").\n";

    if (value != realValue)
        passed = false;

    std::cout << "Creating quadratic term: \n";
    QuadraticTermPtr quadraticTerm1 = std::make_shared<QuadraticTerm>(1, var_x, var_y);
    std::cout << "Quadratic term created: " << quadraticTerm1 << "\n";

    VectorDouble point2;
    point2.push_back(2.0);
    point2.push_back(3.0);

    value = quadraticTerm1->calculate(point2);
    realValue = quadraticTerm1->coefficient * point2.at(0) * point2.at(1);

    std::cout << "Calculating term value: " << value << " (should be equal to " << realValue << ").\n";

    if (value != realValue)
        passed = false;

    std::cout << "Creating quadratic term: \n";
    QuadraticTermPtr quadraticTerm2 = std::make_shared<QuadraticTerm>(1, var_x, var_x);
    std::cout << "Quadratic term created: " << quadraticTerm2 << "\n";

    value = quadraticTerm2->calculate(point2);
    realValue = quadraticTerm2->coefficient * point2.at(0) * point2.at(0);

    std::cout << "Calculating term value: " << value << " (should be equal to " << realValue << ").\n";

    if (value != realValue)
        passed = false;

    return passed;
}

bool ModelTestNonlinearExpressions()
{
    bool passed = true;

    VariablePtr var_x = std::make_shared<Variable>("x", 0, E_VariableType::Real, 0.0, 100.0);
    ExpressionVariablePtr expressionVariable_x = std::make_shared<ExpressionVariable>(var_x);
    VariablePtr var_y = std::make_shared<Variable>("y", 1, E_VariableType::Integer, 0.0, 1.0);
    ExpressionVariablePtr expressionVariable_y = std::make_shared<ExpressionVariable>(var_y);

    VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);

    std::cout << "Creating negate expression\n";
    NonlinearExpressionPtr exprNegate = std::make_shared<ExpressionNegate>(expressionVariable_x);
    std::cout << "Negate expression " << exprNegate << " created\n";

    auto value = exprNegate->calculate(point);
    double realValue = -var_x->calculate(point);

    std::cout
        << "Calculating expression value: " << value << " (should be equal to " << realValue << ").\n";

    if (value != realValue)
        passed = false;

    std::cout << "Creating plus expression\n";

    NonlinearExpressionPtr exprPlus = std::make_shared<ExpressionPlus>(expressionVariable_x, expressionVariable_y);
    std::cout << "Plus expression " << exprPlus << " created\n";

    value = exprPlus->calculate(point);
    realValue = var_x->calculate(point) + var_y->calculate(point);

    std::cout
        << "Calculating expression value: " << value << " (should be equal to " << realValue << ").\n";

    if (value != realValue)
        passed = false;

    std::cout << "Creating power expression\n";
    NonlinearExpressionPtr exprPower = std::make_shared<ExpressionPower>(exprPlus, exprNegate);
    std::cout << "Power expression " << exprPower << " created \n";

    value = exprPower->calculate(point);
    realValue = pow(var_x->calculate(point) + var_y->calculate(point), -var_x->calculate(point));

    std::cout
        << "Calculating expression value: " << value << " (should be equal to " << realValue << ").\n";

    if (value != realValue)
        passed = false;

    return passed;
}

bool ModelTestObjective()
{
    bool passed = true;

    VariablePtr var_x = std::make_shared<Variable>("x", 0, E_VariableType::Real, 0.0, 100.0);
    ExpressionVariablePtr expressionVariable_x = std::make_shared<ExpressionVariable>(var_x);
    VariablePtr var_y = std::make_shared<Variable>("y", 1, E_VariableType::Integer, 0.0, 1.0);
    ExpressionVariablePtr expressionVariable_y = std::make_shared<ExpressionVariable>(var_y);

    std::cout << "Creating linear terms\n";
    LinearTermPtr linearTerm1 = std::make_shared<LinearTerm>(-1, var_x);
    LinearTermPtr linearTerm2 = std::make_shared<LinearTerm>(1.2, var_y);
    LinearTerms linearTerms;
    linearTerms.add(linearTerm1);
    linearTerms.add(linearTerm2);
    std::cout << "Linear terms " << linearTerms << " created\n";

    std::cout << "Creating quadratic terms\n";
    QuadraticTermPtr quadraticTerm1 = std::make_shared<QuadraticTerm>(1, var_x, var_y);
    QuadraticTermPtr quadraticTerm2 = std::make_shared<QuadraticTerm>(2, var_x, var_x);
    QuadraticTerms quadraticTerms;
    quadraticTerms.add(quadraticTerm1);
    quadraticTerms.add(quadraticTerm2);
    std::cout << "Quadratic terms " << quadraticTerms << " created\n";

    std::cout << "Creating nonlinear expression:\n";
    NonlinearExpressions expressions;
    expressions.add(expressionVariable_x);
    expressions.add(expressionVariable_x);
    expressions.add(expressionVariable_y);
    NonlinearExpressionPtr exprTimes = std::make_shared<ExpressionTimes>(expressions);
    std::cout << "Nonlinear expression " << exprTimes << " created\n";

    VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);

    std::cout << "Creating objective function:\n";
    NonlinearObjectiveFunctionPtr nonlinearObjective = std::make_shared<NonlinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize,
                                                                                                    linearTerms,
                                                                                                    quadraticTerms,
                                                                                                    exprTimes,
                                                                                                    10.0);

    std::cout << "Objective function "
              << nonlinearObjective << " created\n";
    double objectiveValue = nonlinearObjective->calculateNumericValue(point);
    double realValue = linearTerm1->coefficient * point.at(0) + linearTerm2->coefficient * point.at(1) + quadraticTerm1->coefficient * point.at(0) * point.at(1) +
                       +quadraticTerm2->coefficient * point.at(0) * point.at(0) + point.at(0) * point.at(0) * point.at(1);

    std::cout
        << "Calculating objective value: " << objectiveValue << " (should be equal to " << realValue << ").\n";

    if (objectiveValue != realValue)
        passed = false;

    return passed;
}

bool ModelTestConstraints()
{
    bool passed = true;

    VariablePtr var_x = std::make_shared<Variable>("x", 0, E_VariableType::Real, 0.0, 100.0);
    ExpressionVariablePtr expressionVariable_x = std::make_shared<ExpressionVariable>(var_x);
    VariablePtr var_y = std::make_shared<Variable>("y", 1, E_VariableType::Integer, 0.0, 1.0);
    ExpressionVariablePtr expressionVariable_y = std::make_shared<ExpressionVariable>(var_y);

    std::cout << "Creating linear terms\n";
    LinearTermPtr linearTerm1 = std::make_shared<LinearTerm>(-1, var_x);
    LinearTermPtr linearTerm2 = std::make_shared<LinearTerm>(1.2, var_y);
    LinearTerms linearTerms;
    linearTerms.add(linearTerm1);
    linearTerms.add(linearTerm2);
    std::cout << "Linear terms " << linearTerms << " created\n";

    std::cout << "Creating quadratic terms\n";
    QuadraticTermPtr quadraticTerm1 = std::make_shared<QuadraticTerm>(1, var_x, var_y);
    QuadraticTermPtr quadraticTerm2 = std::make_shared<QuadraticTerm>(2, var_x, var_x);
    QuadraticTerms quadraticTerms;
    quadraticTerms.add(quadraticTerm1);
    quadraticTerms.add(quadraticTerm2);
    std::cout << "Quadratic terms " << quadraticTerms << " created\n";

    std::cout << "Creating quadratic constraint:\n";
    QuadraticConstraintPtr quadraticConstraint = std::make_shared<QuadraticConstraint>(0, "quadconstr", linearTerms, quadraticTerms, -10.0, 20.0);
    std::cout << "Quadratic constraint created\n";

    VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);

    auto constraintValue = quadraticConstraint->calculateNumericValue(point);
    double realValue = linearTerm1->coefficient * point.at(0) + linearTerm2->coefficient * point.at(1) + quadraticTerm1->coefficient * point.at(0) * point.at(1) +
                       +quadraticTerm2->coefficient * point.at(0) * point.at(0);

    std::cout
        << "Calculating constraint value: " << constraintValue.functionValue << " (should be equal to " << realValue << ").\n";

    if (constraintValue.functionValue != realValue)
        passed = false;

    std::cout << "Creating nonlinear expression:\n";
    NonlinearExpressions expressions;
    expressions.add(expressionVariable_x);
    expressions.add(expressionVariable_x);
    expressions.add(expressionVariable_y);
    NonlinearExpressionPtr exprTimes = std::make_shared<ExpressionTimes>(expressions);
    std::cout << "Nonlinear expression " << exprTimes << " created\n";

    std::cout << "Creating nonlinear constraint:\n";
    NonlinearConstraintPtr nonlinearConstraint = std::make_shared<NonlinearConstraint>(0, "nlconstr", linearTerms, quadraticTerms, exprTimes, -10.0, 20.0);
    std::cout << "Nonlinear constraint " << nonlinearConstraint << " created\n";

    constraintValue = nonlinearConstraint->calculateNumericValue(point);
    realValue = linearTerm1->coefficient * point.at(0) + linearTerm2->coefficient * point.at(1) + quadraticTerm1->coefficient * point.at(0) * point.at(1) + quadraticTerm2->coefficient * point.at(0) * point.at(0) + point.at(0) * point.at(0) * point.at(1);

    std::cout << "Calculating nonlinear constraint value " << constraintValue.functionValue << "(should be equal to " << realValue << ").\n";

    if (constraintValue.functionValue != realValue)
        passed = false;

    bool isFulfilled = nonlinearConstraint->isFulfilled(point);

    std::cout << "Is nonlinear constraint valid in point (x,y)=(" << point.at(0) << "," << point.at(1) << ")? ";
    std::cout << (nonlinearConstraint->isFulfilled(point) ? "yes" : "no");
    std::cout << ". Function value: " << nonlinearConstraint->calculateFunctionValue(point);
    std::cout << "\n";

    if (isFulfilled)
        passed = false;

    point.at(0) = 1.0;
    point.at(1) = 1.0;

    isFulfilled = nonlinearConstraint->isFulfilled(point);

    std::cout << "Is nonlinear constraint valid in point (x,y)=(" << point.at(0) << "," << point.at(1) << ")? ";
    std::cout << (nonlinearConstraint->isFulfilled(point) ? "yes" : "no");
    std::cout << ". Function value: " << nonlinearConstraint->calculateFunctionValue(point);
    std::cout << "\n";

    if (!isFulfilled)
        passed = false;

    return passed;
}

bool ModelTestProblem()
{
    bool passed = true;

    // Creating variables
    OptimizationProblemPtr problem = std::make_shared<OptimizationProblem>();

    VariablePtr var_x = std::make_shared<Variable>("x", 0, E_VariableType::Real, 0.0, 100.0);
    problem->add(var_x);
    ExpressionVariablePtr expressionVariable_x = std::make_shared<ExpressionVariable>(var_x);

    VariablePtr var_y = std::make_shared<Variable>("y", 1, E_VariableType::Integer, 0.0, 1.0);
    problem->add(var_y);
    ExpressionVariablePtr expressionVariable_y = std::make_shared<ExpressionVariable>(var_y);

    std::cout << "Creating objective function\n";

    NonlinearObjectiveFunctionPtr objectiveFunction = std::make_shared<NonlinearObjectiveFunction>(E_ObjectiveFunctionDirection::Minimize);

    //std::cout << "Creating linear terms\n";
    LinearTermPtr objLinearTerm1 = std::make_shared<LinearTerm>(1.0, var_x);
    LinearTermPtr objLinearTerm2 = std::make_shared<LinearTerm>(1.0, var_y);

    LinearTerms objLinearTerms;
    objLinearTerms.add(objLinearTerm1);
    objLinearTerms.add(objLinearTerm2);

    objectiveFunction->add(objLinearTerms);
    //std::cout << "Linear terms " << objLinearTerms << " created\n";

    //std::cout << "Creating quadratic terms\n";
    QuadraticTermPtr objQuadraticTerm1 = std::make_shared<QuadraticTerm>(1, var_x, var_y);
    QuadraticTermPtr objQuadraticTerm2 = std::make_shared<QuadraticTerm>(2, var_x, var_x);
    QuadraticTerms objQuadraticTerms;
    objQuadraticTerms.add(objQuadraticTerm1);
    objQuadraticTerms.add(objQuadraticTerm2);
    objectiveFunction->add(objQuadraticTerms);
    //std::cout << "Quadratic terms " << quadraticTerms << " created\n";

    //std::cout << "Creating nonlinear expression:\n";
    NonlinearExpressions objExpressions;
    objExpressions.add(expressionVariable_x);
    objExpressions.add(expressionVariable_x);
    objExpressions.add(expressionVariable_y);
    NonlinearExpressionPtr objExprTimes = std::make_shared<ExpressionTimes>(objExpressions);
    objectiveFunction->add(objExprTimes);
    //std::cout << "Nonlinear expression " << exprTimes << " created\n";
    problem->add(objectiveFunction);

    std::cout << "Objective function " << objectiveFunction << "created\n";

    VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);

    std::cout << "Creating linear constraint\n";
    LinearTermPtr linearTerm1 = std::make_shared<LinearTerm>(-1, var_x);
    LinearTermPtr linearTerm2 = std::make_shared<LinearTerm>(1.2, var_y);
    LinearTerms linearTerms;
    linearTerms.add(linearTerm1);
    linearTerms.add(linearTerm2);
    LinearConstraintPtr linearConstraint = std::make_shared<LinearConstraint>(0, "linconstr", linearTerms, -2.0, 4.0);
    problem->add(linearConstraint);
    std::cout << "Linear constraint " << linearConstraint << " created\n";

    std::cout << "Creating quadratic constraint:\n";
    QuadraticTermPtr quadraticTerm1 = std::make_shared<QuadraticTerm>(1, var_x, var_y);
    QuadraticTermPtr quadraticTerm2 = std::make_shared<QuadraticTerm>(2, var_x, var_x);
    QuadraticTerms quadraticTerms;
    quadraticTerms.add(quadraticTerm1);
    quadraticTerms.add(quadraticTerm2);
    QuadraticConstraintPtr quadraticConstraint = std::make_shared<QuadraticConstraint>(1, "quadconstr", quadraticTerms, -10.0, 20.0);
    problem->add(quadraticConstraint);
    std::cout << "Quadratic constraint " << quadraticConstraint << " created\n";

    std::cout << "Creating nonlinear constraint:\n";
    NonlinearExpressions expressions;
    expressions.add(expressionVariable_x);
    expressions.add(expressionVariable_x);
    expressions.add(expressionVariable_y);
    NonlinearExpressionPtr exprTimes = std::make_shared<ExpressionTimes>(expressions);
    NonlinearConstraintPtr nonlinearConstraint = std::make_shared<NonlinearConstraint>(2, "nlconstr", linearTerms, exprTimes, -10.0, 20.0);
    problem->add(nonlinearConstraint);
    std::cout << "Nonlinear constraint " << nonlinearConstraint << " created\n";

    std::cout << "Problem created:\n";
    std::cout << problem;

    point.at(0) = 20.0;
    point.at(1) = 1.0;

    std::cout << "Testing for an invalid point for all constraints:\n";
    auto mostDevConstraint = problem->getMostDeviatingNumericConstraint(point);
    if (!mostDevConstraint)
    {
        passed = false;
    }
    else
    {
        double error = mostDevConstraint.get().error;
        auto name = mostDevConstraint.get().constraint->constraintName;
        std::cout << "The most deviating constraint in the point (x,y) = (" << point.at(0) << ',' << point.at(1) << ") is "
                  << name << " with error " << error << "\n";

        if (error != 800.0)
            passed = false;
    }

    std::cout << "Test finished\n";

    point.at(0) = 0.0;
    point.at(1) = 1.0;

    std::cout << "Testing to get all deviating constraint values in a valid point (there should be none)\n";

    auto deviatingConstraints = problem->getAllDeviatingNumericConstraints(point, 0.0);
    std::cout << "Number of invalid constraints in the point (x,y) = (" << point.at(0) << ',' << point.at(1) << ") is " 
              << deviatingConstraints.size() << '\n';

    if (deviatingConstraints.size() != 0)
    {
        passed = false;
    }

    std::cout << "Test finished\n";

    std::cout << "Testing to get the most deviating constraint value in a valid point:\n";

    auto mostDevConstraint2 = problem->getMostDeviatingNumericConstraint(point);
    if (!mostDevConstraint2)
    {
        std::cout << "Constraint not found, everything ok\n";
        passed = true;
    }
    else
    {
        std::cout << "Constraint found, something went wrong\n";
        passed = false;
    }

    std::cout << "Test finished\n";

    return passed;
}
