/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "Problem.h"

namespace SHOT
{

void Problem::updateVariables()
{
    auto numVariables = allVariables.size();

    // Update bound vectors
    if (variableLowerBounds.size() != numVariables)
        variableLowerBounds.resize(numVariables);

    if (variableUpperBounds.size() != numVariables)
        variableUpperBounds.resize(numVariables);

    int numNonlinearVars = 0;

    nonlinearVariables.clear();

    for (int i = 0; i < numVariables; i++)
    {
        variableLowerBounds[i] = allVariables[i]->lowerBound;
        variableUpperBounds[i] = allVariables[i]->upperBound;

        if (allVariables[i]->isNonlinear)
            nonlinearVariables.push_back(allVariables[i]);
    }

    variablesUpdated = true;
};

void Problem::updateProperties()
{
    if (!variablesUpdated)
        updateVariables();

    properties.isConvex = true;
    properties.isNonconvex = false;

    properties.numberOfVariables = allVariables.size();
    properties.numberOfRealVariables = realVariables.size();
    properties.numberOfBinaryVariables = binaryVariables.size();
    properties.numberOfIntegerVariables = integerVariables.size();
    properties.numberOfDiscreteVariables = properties.numberOfBinaryVariables + properties.numberOfIntegerVariables;
    properties.numberOfSemicontinuousVariables = semicontinuousVariables.size();
    properties.numberOfNonlinearVariables = nonlinearVariables.size();

    properties.numberOfNumericConstraints = numericConstraints.size();
    properties.numberOfLinearConstraints = linearConstraints.size();
    properties.numberOfQuadraticConstraints = quadraticConstraints.size();
    properties.numberOfNonlinearConstraints = nonlinearConstraints.size();
    properties.numberOfNonlinearExpressions = nonlinearConstraints.size();

    bool isObjNonlinear = (objectiveFunction->properties.classification > E_ObjectiveFunctionClassification::Quadratic);
    bool isObjQuadratic = (objectiveFunction->properties.classification == E_ObjectiveFunctionClassification::Quadratic);
    bool areConstrsNonlinear = (properties.numberOfNonlinearConstraints > 0);
    bool areConstrsQuadratic = (properties.numberOfQuadraticConstraints > 0);
    bool areVarsDiscrete = (properties.numberOfDiscreteVariables > 0);

    if (areConstrsNonlinear || isObjNonlinear)
        properties.isNonlinear = true;

    if (isObjNonlinear)
        properties.numberOfNonlinearExpressions++;

    if (areVarsDiscrete)
    {
        properties.isDiscrete = true;

        if (areConstrsNonlinear || isObjNonlinear)
            properties.isMINLPProblem = true;
        else if (areConstrsQuadratic)
            properties.isMIQCQPProblem = true;
        else if (isObjQuadratic)
            properties.isMIQPProblem = true;
        else
            properties.isMILPProblem = true;
    }
    else
    {
        properties.isDiscrete = false;

        if (areConstrsNonlinear || isObjNonlinear)
            properties.isNLPProblem = true;
        else if (areConstrsQuadratic)
            properties.isQCQPProblem = true;
        else if (isObjQuadratic)
            properties.isQPProblem = true;
        else
            properties.isLPProblem = true;
    }

    properties.isValid = true;
};

void Problem::updateFactorableFunctions()
{
    factorableFunctionsDAG = std::make_shared<FactorableFunctionGraph>();

    for (auto V : nonlinearVariables)
    {
        std::cout << "making variable " << V << " nonlinear\n";
        auto factorableFunctionsVar = std::make_shared<FactorableFunction>();
        factorableFunctionsVar->set(factorableFunctionsDAG.get());
        V->factorableFunctionVariable = factorableFunctionsVar;
        factorableFunctionVariables.push_back(*factorableFunctionsVar.get());
    }
    for (auto C : nonlinearConstraints)
    {
        std::cout << "updating factorable functions in constraint\n";
        C->updateFactorableFunction();
        factorableFunctions.push_back(*C->factorableFunction.get());
    }

    int objectiveFactorableFunctionIndex;

    if (objectiveFunction->properties.hasNonlinearExpression)
    {
        std::cout << "updating facorable functions in objective\n";
        auto objective = static_cast<NonlinearObjectiveFunction *>(objectiveFunction.get());
        objective->updateFactorableFunction();
        factorableFunctions.push_back(*objective->factorableFunction.get());

        objective->factorableFunctionIndex = factorableFunctions.size() - 1;
    }

    // Gets the jacobian in sparse format; note that it will add nodes to the DAG!
    auto jacobian = factorableFunctionsDAG->SFAD(properties.numberOfNonlinearConstraints, &factorableFunctions[0], properties.numberOfNonlinearVariables, &factorableFunctionVariables[0]);

    for (int i = 0; i < std::get<0>(jacobian); i++)
    {
        auto nonlinearVariable = nonlinearVariables[std::get<2>(jacobian)[i]];
        auto jacobianElement = std::get<3>(jacobian)[i];

        if (objectiveFunction->properties.hasNonlinearExpression && std::get<1>(jacobian)[i] == objectiveFactorableFunctionIndex)
        {
            auto objective = static_cast<NonlinearObjectiveFunction *>(objectiveFunction.get());
            objective->symbolicSparseJacobian.push_back(std::make_pair(nonlinearVariable, jacobianElement));
            continue;
        }

        auto nonlinearConstraint = nonlinearConstraints[std::get<1>(jacobian)[i]];
        nonlinearConstraint->symbolicSparseJacobian.push_back(std::make_pair(nonlinearVariable, jacobianElement));
    }
};

Problem::Problem(){

};

Problem::~Problem(){};

void Problem::finalize()
{
    updateVariables();
    updateProperties();
    updateFactorableFunctions();
}

void Problem::add(Variables variables)
{
    for (auto V : variables)
        add(V);
}

void Problem::add(VariablePtr variable)
{
    allVariables.push_back(variable);

    switch (variable->type)
    {
    case (E_VariableType::Real):
        realVariables.push_back(variable);
        break;
    case (E_VariableType::Binary):
        binaryVariables.push_back(variable);
        break;
    case (E_VariableType::Integer):
        integerVariables.push_back(variable);
        break;
    case (E_VariableType::Semicontinuous):
        semicontinuousVariables.push_back(variable);
        break;
    default:
        break;
    }

    variable->takeOwnership(shared_from_this());
    variablesUpdated = false;
};

void Problem::add(LinearConstraintPtr constraint)
{
    numericConstraints.push_back(std::dynamic_pointer_cast<NumericConstraint>(constraint));
    linearConstraints.push_back(constraint);

    constraint->takeOwnership(shared_from_this());
};

void Problem::add(QuadraticConstraintPtr constraint)
{
    numericConstraints.push_back(std::dynamic_pointer_cast<NumericConstraint>(constraint));
    quadraticConstraints.push_back(constraint);

    constraint->takeOwnership(shared_from_this());
};

void Problem::add(NonlinearConstraintPtr constraint)
{
    numericConstraints.push_back(std::dynamic_pointer_cast<NumericConstraint>(constraint));
    nonlinearConstraints.push_back(constraint);

    constraint->takeOwnership(shared_from_this());
};

void Problem::add(ObjectiveFunctionPtr objective)
{
    objectiveFunction = objective;
    objectiveFunction->updateProperties();

    objective->takeOwnership(shared_from_this());
};

template <class T>
void Problem::add(std::vector<T> elements)
{
    for (auto E : elements)
    {
        add(E);

        E->takeOwnership(shared_from_this());
    }
};

VariablePtr Problem::getVariable(int variableIndex)
{
    if (variableIndex > allVariables.size())
    {
        throw new VariableNotFoundException(" with variableIndex " + std::to_string(variableIndex));
    }

    return allVariables.at(variableIndex);
};

double Problem::getVariableLowerBound(int variableIndex)
{
    return allVariables.at(variableIndex)->lowerBound;
};

double Problem::getVariableUpperBound(int variableIndex)
{
    return allVariables.at(variableIndex)->upperBound;
};

VectorDouble Problem::getVariableLowerBounds()
{
    if (!variablesUpdated)
    {
        updateVariables();
    }

    return variableLowerBounds;
};

VectorDouble Problem::getVariableUpperBounds()
{
    if (!variablesUpdated)
    {
        updateVariables();
    }

    return variableUpperBounds;
};

void Problem::setVariableLowerBound(int variableIndex, double bound)
{
    allVariables.at(variableIndex)->lowerBound = bound;
    variablesUpdated = true;
};

void Problem::setVariableUpperBound(int variableIndex, double bound)
{
    allVariables.at(variableIndex)->upperBound = bound;
    variablesUpdated = true;
};

void Problem::setVariableBounds(int variableIndex, double lowerBound, double upperBound)
{
    allVariables.at(variableIndex)->lowerBound = lowerBound;
    allVariables.at(variableIndex)->upperBound = upperBound;
    variablesUpdated = true;
};

boost::optional<NumericConstraintValue> Problem::getMostDeviatingNumericConstraint(const VectorDouble &point)
{
    return (this->getMostDeviatingNumericConstraint(point, numericConstraints));
};

boost::optional<NumericConstraintValue> Problem::getMostDeviatingNumericConstraint(const VectorDouble &point, NumericConstraints constraintSelection)
{
    boost::optional<NumericConstraintValue> optional;
    double error = -1;

    for (auto C : constraintSelection)
    {
        auto constraintValue = C->calculateNumericValue(point);

        if (constraintValue.isFulfilled)
            continue;

        if (!optional) // No constraint with error found yet
        {
            optional = constraintValue;
            error = constraintValue.error;
        }
        else if (constraintValue.error > error)
        {
            optional = constraintValue;
            error = constraintValue.error;
        }
    }

    return optional;
};

template <class T>
NumericConstraintValues Problem::getAllDeviatingConstraints(const VectorDouble &point, double tolerance, std::vector<T> constraintSelection)
{
    NumericConstraintValues constraintValues;
    for (auto C : constraintSelection)
    {
        NumericConstraintValue constraintValue = C->calculateNumericValue(point);
        if (constraintValue.error > tolerance)
            constraintValues.push_back(constraintValue);
    }

    return constraintValues;
};

NumericConstraintValues Problem::getAllDeviatingNumericConstraints(const VectorDouble &point, double tolerance)
{
    return getAllDeviatingConstraints(point, tolerance, numericConstraints);
};

NumericConstraintValues Problem::getAllDeviatingLinearConstraints(const VectorDouble &point, double tolerance)
{
    return getAllDeviatingConstraints(point, tolerance, linearConstraints);
};

NumericConstraintValues Problem::getAllDeviatingQuadraticConstraints(const VectorDouble &point, double tolerance)
{
    return getAllDeviatingConstraints(point, tolerance, quadraticConstraints);
};

NumericConstraintValues Problem::getAllDeviatingNonlinearConstraints(const VectorDouble &point, double tolerance)
{
    return getAllDeviatingConstraints(point, tolerance, nonlinearConstraints);
};

bool Problem::areLinearConstraintsFulfilled(VectorDouble point, double tolerance)
{
    auto deviatingConstraints = getAllDeviatingLinearConstraints(point, tolerance);
    return (deviatingConstraints.size() == 0);
};

bool Problem::areQuadraticConstraintsFulfilled(VectorDouble point, double tolerance)
{
    auto deviatingConstraints = getAllDeviatingQuadraticConstraints(point, tolerance);
    return (deviatingConstraints.size() == 0);
};

bool Problem::areNonlinearConstraintsFulfilled(VectorDouble point, double tolerance)
{
    auto deviatingConstraints = getAllDeviatingNonlinearConstraints(point, tolerance);
    return (deviatingConstraints.size() == 0);
};

bool Problem::areNumericConstraintsFulfilled(VectorDouble point, double tolerance)
{
    auto deviatingConstraints = getAllDeviatingNumericConstraints(point, tolerance);
    return (deviatingConstraints.size() == 0);
};

bool Problem::areIntegralityConstraintsFulfilled(VectorDouble point, double tolerance)
{
    for (auto V : integerVariables)
    {
        if (abs(point.at(V->index) - round(point.at(V->index))) > tolerance)
            return false;
    }

    return true;
};

bool Problem::areVariableBoundsFulfilled(VectorDouble point, double tolerance)
{
    for (int i = 0; i < properties.numberOfVariables; ++i)
    {
        if (point.at(i) - tolerance > allVariables.at(i)->upperBound)
        {
            return false;
        }
        if (point.at(i) + tolerance < allVariables.at(i)->lowerBound)
        {
            return false;
        }
    }

    return true;
};

std::ostream &operator<<(std::ostream &stream, const Problem &problem)
{
    stream << problem.objectiveFunction << '\n';

    if (problem.numericConstraints.size() > 0)
        stream << "subject to:\n";

    for (auto C : problem.numericConstraints)
    {
        stream << C << '\n';
    }

    stream << "variables:\n";

    for (auto V : problem.allVariables)
    {
        stream << V << '\n';
    }

    return stream;
};
}; // namespace SHOT