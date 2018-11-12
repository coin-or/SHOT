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

#include "ModelingSystem/ModelingSystemOS.h"
#include "ModelingSystem/ModelingSystemGAMS.h"

#include "SHOTSolver.h"
#include "Environment.h"

bool ModelTestVariables();
bool ModelTestTerms();
bool ModelTestNonlinearExpressions();
bool ModelTestObjective();
bool ModelTestConstraints();
bool ModelTestCreateProblem();
bool ModelTestReadOSiLProblem();
bool ModelTestReadGAMSProblem();
bool ModelTestModelingSystemOS();

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
        passed = ModelTestCreateProblem();
        break;
    case 7:
        passed = ModelTestReadOSiLProblem();
        break;
    case 8:
        passed = ModelTestReadGAMSProblem();
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
    SHOT::NonlinearExpressionPtr exprProduct = std::make_shared<SHOT::ExpressionProduct>(expressions);
    std::cout << "Nonlinear expression " << exprProduct << " created\n";

    SHOT::VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);

    std::cout << "Creating objective function:\n";
    SHOT::NonlinearObjectiveFunctionPtr nonlinearObjective = std::make_shared<SHOT::NonlinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Minimize,
                                                                                                                linearTerms,
                                                                                                                quadraticTerms,
                                                                                                                exprProduct,
                                                                                                                10.0);

    std::cout << "Objective function "
              << nonlinearObjective << " created\n";
    double objectiveValue = nonlinearObjective->calculateValue(point);
    double realValue = nonlinearObjective->constant + linearTerm1->coefficient * point.at(0) + linearTerm2->coefficient * point.at(1) + quadraticTerm1->coefficient * point.at(0) * point.at(1) +
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
    SHOT::NonlinearExpressionPtr exprProduct = std::make_shared<SHOT::ExpressionProduct>(expressions);
    std::cout << "Nonlinear expression " << exprProduct << " created\n";

    std::cout << "Creating nonlinear constraint:\n";
    SHOT::NonlinearConstraintPtr nonlinearConstraint = std::make_shared<SHOT::NonlinearConstraint>(0, "nlconstr", linearTerms, quadraticTerms, exprProduct, -10.0, 20.0);
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

bool ModelTestCreateProblem()
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

    SHOT::NonlinearObjectiveFunctionPtr objectiveFunction = std::make_shared<SHOT::NonlinearObjectiveFunction>(SHOT::E_ObjectiveFunctionDirection::Minimize);
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
    SHOT::LinearConstraintPtr linearConstraint = std::make_shared<SHOT::LinearConstraint>(0, "linconstr", linearTerms, -2.0, 4.0);
    problem->add(linearConstraint);

    std::cout << '\n';
    std::cout << "Linear constraint " << linearConstraint << " created\n";

    SHOT::QuadraticTermPtr quadraticTerm1 = std::make_shared<SHOT::QuadraticTerm>(1, var_x, var_y);
    SHOT::QuadraticTermPtr quadraticTerm2 = std::make_shared<SHOT::QuadraticTerm>(2, var_x, var_x);
    SHOT::QuadraticTerms quadraticTerms;
    quadraticTerms.add(quadraticTerm1);
    quadraticTerms.add(quadraticTerm2);
    SHOT::QuadraticConstraintPtr quadraticConstraint = std::make_shared<SHOT::QuadraticConstraint>(1, "quadconstr", quadraticTerms, -10.0, 20.0);
    problem->add(quadraticConstraint);

    std::cout << '\n';
    std::cout << "Quadratic constraint " << quadraticConstraint << " created\n";

    SHOT::NonlinearExpressionPtr exprPlus = std::make_shared<SHOT::ExpressionPlus>(expressionVariable_z, expressionVariable_x);
    SHOT::NonlinearExpressionPtr exprConstant = std::make_shared<SHOT::ExpressionConstant>(3);
    SHOT::NonlinearExpressionPtr exprPower = std::make_shared<SHOT::ExpressionPower>(expressionVariable_z, exprConstant);

    SHOT::NonlinearExpressions expressions;
    expressions.add(exprPower);
    expressions.add(exprPlus);

    SHOT::NonlinearExpressionPtr exprSum = std::make_shared<SHOT::ExpressionSum>(expressions);

    SHOT::NonlinearConstraintPtr nonlinearConstraint = std::make_shared<SHOT::NonlinearConstraint>(2, "nlconstr", linearTerms, exprSum, -10.0, 20.0);
    problem->add(nonlinearConstraint);

    std::cout << '\n';
    std::cout << "Nonlinear constraint " << nonlinearConstraint << " created\n";

    std::cout << '\n';
    std::cout << "Finalizing problem:\n";
    problem->finalize();

    std::cout << '\n';
    std::cout << "Problem created:\n\n";
    std::cout << problem << '\n';

    std::cout << '\n';
    std::cout << "Nonlinear DAG:\n\n";
    std::cout << problem->factorableFunctionsDAG << '\n';

    SHOT::VectorDouble point;
    point.push_back(2.0);
    point.push_back(3.0);
    point.push_back(1.0);

    std::cout << "Calculating gradient for function in linear constraint:\n";
    auto gradientLinear = linearConstraint->calculateGradient(point);

    for (auto const &G : gradientLinear)
    {
        std::cout << G.first->name << ": " << G.second << '\n';
    }

    std::cout << "\nCalculating gradient for function in quadratic constraint:\n";
    auto gradientQuadratic = quadraticConstraint->calculateGradient(point);

    for (auto const &G : gradientQuadratic)
    {
        std::cout << G.first->name << ": " << G.second << '\n';
    }

    std::cout << "\nCalculating gradient for function in nonlinear constraint:\n";
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

    std::cout << "\nCalculating function interval value for interval vector\n";
    std::cout << "x = " << X << '\n';
    std::cout << "y = " << Y << '\n';
    std::cout << "z = " << Z << '\n';

    auto linearIntervalValue = linearConstraint->calculateFunctionValue(vector);
    std::cout << "\nValue for linear constraint is:\n";
    std::cout << linearIntervalValue << '\n';

    if (abs(linearIntervalValue.l() - 0.4) > (1.0e-12) || abs(linearIntervalValue.u() - 2.6) > (1.0e-12))
    {
        std::cout << "Interval is not correct.\n";
        passed = false;
    }

    auto quadraticIntervalValue = quadraticConstraint->calculateFunctionValue(vector);
    std::cout << "\nValue for quadratic constraint is:\n";
    std::cout << quadraticIntervalValue << '\n';

    if (abs(quadraticIntervalValue.l() - 4) > (1.0e-12) || abs(quadraticIntervalValue.u() - 14) > (1.0e-12))
    {
        std::cout << "Interval is not correct.\n";
        passed = false;
    }

    auto nonlinearIntervalValue = nonlinearConstraint->calculateFunctionValue(vector);
    std::cout << "\nValue for nonlinear constraint is:\n";
    std::cout << nonlinearIntervalValue << '\n';

    if (abs(nonlinearIntervalValue.l() - 31.4) > (1.0e-12) || abs(nonlinearIntervalValue.u() - 72.6) > (1.0e-12))
    {
        std::cout << "Interval is not correct.\n";
        passed = false;
    }

    point.at(0) = 20.0;
    point.at(1) = 1.0;
    point.at(2) = 1.0;

    std::cout << "\nTesting for an invalid point for all constraints:\n";
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

    std::cout << '\n';

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

    std::cout << '\n';

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

    return passed;
}

bool ModelTestReadOSiLProblem()
{
    bool passed = true;

    {
        SHOT::EnvironmentPtr env(new SHOT::Environment);
        env->output = SHOT::OutputPtr(new SHOT::Output());
        env->process = SHOT::ProcessPtr(new SHOT::ProcessInfo(env));
        env->settings = SHOT::SettingsPtr(new SHOT::Settings(env->output));
        env->tasks = SHOT::TaskHandlerPtr(new SHOT::TaskHandler(env));
        env->report = SHOT::ReportPtr(new SHOT::Report(env));
        env->model = SHOT::ModelPtr(new SHOT::Model(env));
        std::unique_ptr<SHOT::SHOTSolver> solver(new SHOT::SHOTSolver(env));
        solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(ENUM_OUTPUT_LEVEL_summary));

        auto modelSystem = std::make_shared<SHOT::ModelingSystemOS>(env);
        SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

        std::string problemFile;
        problemFile = "data/tls2.osil";

        std::cout << "Testing to read problem in OSiL format: " << problemFile << '\n';

        if (modelSystem->createProblem(problem, problemFile, E_OSInputFileFormat::OSiL) != E_ProblemCreationStatus::NormalCompletion)
        {
            std::cout << "Error while reading problem";
            passed = false;
        }
        else
        {
            std::cout << "Problem read successfully:\n\n";
            std::cout << problem << "\n\n";
            std::cout << problem->factorableFunctionsDAG << '\n';
        }
    }

    //TODO: Figure out why nl format import is not working...

    {
        SHOT::EnvironmentPtr env(new SHOT::Environment);
        env->output = SHOT::OutputPtr(new SHOT::Output());
        env->process = SHOT::ProcessPtr(new SHOT::ProcessInfo(env));
        env->settings = SHOT::SettingsPtr(new SHOT::Settings(env->output));
        env->tasks = SHOT::TaskHandlerPtr(new SHOT::TaskHandler(env));
        env->report = SHOT::ReportPtr(new SHOT::Report(env));
        env->model = SHOT::ModelPtr(new SHOT::Model(env));
        std::unique_ptr<SHOT::SHOTSolver> solver(new SHOT::SHOTSolver(env));
        solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(ENUM_OUTPUT_LEVEL_summary));

        auto modelSystem = std::make_shared<SHOT::ModelingSystemOS>(env);
        SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

        std::string problemFile;
        problemFile = "data/tls2.nl";

        std::cout << "Testing to read problem in nl format: " << problemFile << '\n';

        if (modelSystem->createProblem(problem, problemFile, E_OSInputFileFormat::Ampl) != E_ProblemCreationStatus::NormalCompletion)
        {
            std::cout << "Error while reading problem";
            passed = false;
        }
        else
        {
            std::cout << "Problem read successfully:\n\n";
            std::cout << problem << "\n\n";
            std::cout << problem->factorableFunctionsDAG << '\n';
        }
    }

    return passed;
}

bool ModelTestReadGAMSProblem()
{
    bool passed = true;

    {
        SHOT::EnvironmentPtr env(new SHOT::Environment);
        env->output = SHOT::OutputPtr(new SHOT::Output());
        env->process = SHOT::ProcessPtr(new SHOT::ProcessInfo(env));
        env->settings = SHOT::SettingsPtr(new SHOT::Settings(env->output));
        env->tasks = SHOT::TaskHandlerPtr(new SHOT::TaskHandler(env));
        env->report = SHOT::ReportPtr(new SHOT::Report(env));
        env->model = SHOT::ModelPtr(new SHOT::Model(env));
        std::unique_ptr<SHOT::SHOTSolver> solver(new SHOT::SHOTSolver(env));
        solver->updateSetting("Console.LogLevel", "Output", static_cast<int>(ENUM_OUTPUT_LEVEL_debug));

        auto modelSystem = std::make_shared<SHOT::ModelingSystemGAMS>(env);
        SHOT::ProblemPtr problem = std::make_shared<SHOT::Problem>(env);

        std::string problemFile;
        problemFile = "data/fo7.gms";

        std::cout << "Testing to read problem in GAMS format: " << problemFile << '\n';

        if (modelSystem->createProblem(problem, problemFile, E_GAMSInputSource::ProblemFile) != E_ProblemCreationStatus::NormalCompletion)
        {
            std::cout << "Error while reading problem";
            passed = false;
        }
        else
        {
            std::cout << "Problem read successfully:\n\n";
            std::cout << problem << "\n\n";
            std::cout << problem->factorableFunctionsDAG << '\n';
        }
    }

    return passed;
}
