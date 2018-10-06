/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/
#include <iostream>
#include "Model/ModelShared.h"
#include "Model/Terms.h"
#include "Model/Constraints.h"
#include "Model/NonlinearExpressions.h"
#include "Model/ObjectiveFunction.h"
#include "Model/Problem.h"

#include "Environment.h"

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
    SHOT::VariablePtr var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 100.0);
    std::cout << "Variable " << var_x << " created.\n";

    SHOT::VectorDouble point;
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

    if (value != realValue)
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

    if (value != realValue)
        passed = false;

    std::cout << "Creating quadratic term: \n";
    SHOT::QuadraticTermPtr quadraticTerm2 = std::make_shared<SHOT::QuadraticTerm>(1, var_x, var_x);
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

    std::cout
        << "Calculating expression value: " << value << " (should be equal to " << realValue << ").\n";

    if (value != realValue)
        passed = false;

    std::cout
        << "Creating plus expression\n";

    SHOT::NonlinearExpressionPtr exprPlus = std::make_shared<SHOT::ExpressionPlus>(expressionVariable_x, expressionVariable_y);
    std::cout << "Plus expression " << exprPlus << " created\n";

    value = exprPlus->calculate(point);
    realValue = var_x->calculate(point) + var_y->calculate(point);

    std::cout
        << "Calculating expression value: " << value << " (should be equal to " << realValue << ").\n";

    if (value != realValue)
        passed = false;

    std::cout << "Creating power expression\n";
    SHOT::NonlinearExpressionPtr exprPower = std::make_shared<SHOT::ExpressionPower>(exprPlus, exprNegate);
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
    SHOT::NonlinearExpressionPtr exprTimes = std::make_shared<SHOT::ExpressionTimes>(expressions);
    std::cout << "Nonlinear expression " << exprTimes << " created\n";

    SHOT::VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);

    std::cout << "Creating objective function:\n";
    SHOT::NonlinearObjectiveFunctionPtr nonlinearObjective = std::make_shared<SHOT::NonlinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Minimize,
                                                                                                                linearTerms,
                                                                                                                quadraticTerms,
                                                                                                                exprTimes,
                                                                                                                10.0);

    std::cout << "Objective function "
              << nonlinearObjective << " created\n";
    double objectiveValue = nonlinearObjective->calculateValue(point);
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
    SHOT::QuadraticConstraintPtr quadraticConstraint = std::make_shared<SHOT::QuadraticConstraint>(0, "quadconstr", linearTerms, quadraticTerms, -10.0, 20.0);
    std::cout << "Quadratic constraint created\n";

    SHOT::VectorDouble point;
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
    SHOT::NonlinearExpressions expressions;
    expressions.add(expressionVariable_x);
    expressions.add(expressionVariable_x);
    expressions.add(expressionVariable_y);
    SHOT::NonlinearExpressionPtr exprTimes = std::make_shared<SHOT::ExpressionTimes>(expressions);
    std::cout << "Nonlinear expression " << exprTimes << " created\n";

    std::cout << "Creating nonlinear constraint:\n";
    SHOT::NonlinearConstraintPtr nonlinearConstraint = std::make_shared<SHOT::NonlinearConstraint>(0, "nlconstr", linearTerms, quadraticTerms, exprTimes, -10.0, 20.0);
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

    SHOT::EnvironmentPtr env(new SHOT::Environment);
    env->output = SHOT::OutputPtr(new SHOT::Output());
    SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

    // Creating variables

    auto var_x = std::make_shared<SHOT::Variable>("x", 0, SHOT::E_VariableType::Real, 0.0, 100.0);
    SHOT::ExpressionVariablePtr expressionVariable_x = std::make_shared<SHOT::ExpressionVariable>(var_x);

    auto var_y = std::make_shared<SHOT::Variable>("y", 1, SHOT::E_VariableType::Integer, 0.0, 1.0);
    SHOT::ExpressionVariablePtr expressionVariable_y = std::make_shared<SHOT::ExpressionVariable>(var_y);

    auto var_z = std::make_shared<SHOT::Variable>("z", 2, SHOT::E_VariableType::Integer, 0.0, 2.0);
    SHOT::ExpressionVariablePtr expressionVariable_z = std::make_shared<SHOT::ExpressionVariable>(var_z);

    SHOT::Variables variables = {var_x, var_y, var_z};
    problem->add(variables);

    std::cout << "Creating objective function\n";

    SHOT::NonlinearObjectiveFunctionPtr objectiveFunction = std::make_shared<SHOT::NonlinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Minimize);

    //std::cout << "Creating linear terms\n";
    SHOT::LinearTermPtr objLinearTerm1 = std::make_shared<SHOT::LinearTerm>(1.0, var_x);
    SHOT::LinearTermPtr objLinearTerm2 = std::make_shared<SHOT::LinearTerm>(1.0, var_y);

    SHOT::LinearTerms objLinearTerms;
    objLinearTerms.add(objLinearTerm1);
    objLinearTerms.add(objLinearTerm2);

    objectiveFunction->add(objLinearTerms);
    //std::cout << "Linear terms " << objLinearTerms << " created\n";

    //std::cout << "Creating quadratic terms\n";
    SHOT::QuadraticTermPtr objQuadraticTerm1 = std::make_shared<SHOT::QuadraticTerm>(1, var_x, var_y);
    SHOT::QuadraticTermPtr objQuadraticTerm2 = std::make_shared<SHOT::QuadraticTerm>(2, var_x, var_x);
    SHOT::QuadraticTerms objQuadraticTerms;
    objQuadraticTerms.add(objQuadraticTerm1);
    objQuadraticTerms.add(objQuadraticTerm2);
    objectiveFunction->add(objQuadraticTerms);
    //std::cout << "Quadratic terms " << quadraticTerms << " created\n";

    //std::cout << "Creating nonlinear expression:\n";
    SHOT::NonlinearExpressions objExpressions;
    objExpressions.add(expressionVariable_x);
    objExpressions.add(expressionVariable_y);
    SHOT::NonlinearExpressionPtr objExprTimes = std::make_shared<SHOT::ExpressionSum>(objExpressions);
    objectiveFunction->add(objExprTimes);
    //std::cout << "Nonlinear expression " << exprTimes << " created\n";
    problem->add(objectiveFunction);

    std::cout << "Objective function " << objectiveFunction << "created\n";

    SHOT::VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);
    point.push_back(1.0);

    std::cout << "Creating linear constraint\n";
    SHOT::LinearTermPtr linearTerm1 = std::make_shared<SHOT::LinearTerm>(-1, var_x);
    SHOT::LinearTermPtr linearTerm2 = std::make_shared<SHOT::LinearTerm>(1.2, var_y);
    SHOT::LinearTerms linearTerms;
    linearTerms.add(linearTerm1);
    linearTerms.add(linearTerm2);
    SHOT::LinearConstraintPtr linearConstraint = std::make_shared<SHOT::LinearConstraint>(0, "linconstr", linearTerms, -2.0, 4.0);
    problem->add(linearConstraint);
    std::cout << "Linear constraint " << linearConstraint << " created\n";

    std::cout << "Creating quadratic constraint:\n";
    SHOT::QuadraticTermPtr quadraticTerm1 = std::make_shared<SHOT::QuadraticTerm>(1, var_x, var_y);
    SHOT::QuadraticTermPtr quadraticTerm2 = std::make_shared<SHOT::QuadraticTerm>(2, var_x, var_x);
    SHOT::QuadraticTerms quadraticTerms;
    quadraticTerms.add(quadraticTerm1);
    quadraticTerms.add(quadraticTerm2);
    SHOT::QuadraticConstraintPtr quadraticConstraint = std::make_shared<SHOT::QuadraticConstraint>(1, "quadconstr", quadraticTerms, -10.0, 20.0);
    problem->add(quadraticConstraint);
    std::cout << "Quadratic constraint " << quadraticConstraint << " created\n";

    std::cout << "Creating nonlinear constraint:\n";
    SHOT::NonlinearExpressions expressions;
    expressions.add(expressionVariable_z);
    expressions.add(expressionVariable_x);

    SHOT::NonlinearExpressionPtr exprTimes = std::make_shared<SHOT::ExpressionSum>(expressions);
    SHOT::NonlinearExpressionPtr exprConstant = std::make_shared<SHOT::ExpressionConstant>(3);
    SHOT::NonlinearExpressionPtr exprPower = std::make_shared<SHOT::ExpressionPower>(expressionVariable_z, exprConstant);

    SHOT::NonlinearExpressions expressions2;
    expressions2.add(exprPower);
    expressions2.add(exprTimes);

    SHOT::NonlinearExpressionPtr exprSum = std::make_shared<SHOT::ExpressionSum>(expressions2);

    SHOT::NonlinearConstraintPtr nonlinearConstraint = std::make_shared<SHOT::NonlinearConstraint>(2, "nlconstr", linearTerms, exprSum, -10.0, 20.0);
    problem->add(nonlinearConstraint);
    std::cout << "Nonlinear constraint " << nonlinearConstraint << " created\n";
    std::cout << "Finalizing problem:\n";
    problem->finalize();
    std::cout << "Problem finalized\n";

    std::cout << "Problem created:\n";
    std::cout << problem << '\n';

    std::cout << "Calculating gradient for function in linear constraint:\n";
    auto gradientLinear = linearConstraint->calculateGradient(point);

    for (auto const &G : gradientLinear)
    {
        std::cout << G.first->name << ": " << G.second << '\n';
    }

    std::cout << "Calculating gradient for function in quadratic constraint:\n";
    auto gradientQuadratic = quadraticConstraint->calculateGradient(point);

    for (auto const &G : gradientQuadratic)
    {
        std::cout << G.first->name << ": " << G.second << '\n';
    }

    std::cout << "Calculating gradient for function in nonlinear constraint:\n";
    auto gradientNonlinear = nonlinearConstraint->calculateGradient(point);

    for (auto const &G : gradientNonlinear)
    {
        std::cout << G.first->name << ":  " << G.second << '\n';
    }

    SHOT::Interval X(1., 2.);
    SHOT::Interval Y(2., 3.);
    SHOT::Interval Z(3., 4.);

    SHOT::IntervalVector vector;
    vector.push_back(X);
    vector.push_back(Y);
    vector.push_back(Z);

    std::cout << exprTimes->calculate(vector) << '\n';
    std::cout << nonlinearConstraint->calculateFunctionValue(vector) << '\n';

    point.at(0) = 20.0;
    point.at(1) = 1.0;
    point.at(2) = 1.0;

    std::cout << "Testing for an invalid point for all constraints:\n";
    auto mostDevConstraint = problem->getMostDeviatingNumericConstraint(point);
    if (!mostDevConstraint)
    {
        passed = false;
    }
    else
    {
        double error = mostDevConstraint.get().error;
        auto name = mostDevConstraint.get().constraint->name;
        std::cout << "The most deviating constraint in the point (x,y) = (" << point.at(0) << ',' << point.at(1) << ") is "
                  << name << " with error " << error << "\n";

        if (error != 800.0)
            passed = false;
    }

    std::cout << "Test finished\n";

    point.at(0) = 0.0;
    point.at(1) = 1.0;
    point.at(2) = 2.0;

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
