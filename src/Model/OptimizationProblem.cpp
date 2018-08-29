/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "OptimizationProblem.h"

using namespace SHOT;

OptimizationProblem::OptimizationProblem()
{
}

OptimizationProblem::~OptimizationProblem()
{
    // TODO Auto-generated destructor stub
}

void OptimizationProblem::updateProperties()
{
}

Variables OptimizationProblem::getAllVariables()
{
    return allVariables;
}

Variables OptimizationProblem::getRealVariables()
{
    return realVariables;
}

Variables OptimizationProblem::getBinaryVariables()
{
    return binaryVariables;
}

Variables OptimizationProblem::getIntegerVariables()
{
    return integerVariables;
}

Variables OptimizationProblem::getSemicontinuousVariables()
{
    return semicontinousVariables;
}

void OptimizationProblem::addVariable(VariablePtr variable)
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
        semicontinousVariables.push_back(variable);
        break;
    default:
        break;
    }
}

void OptimizationProblem::addVariables(Variables variables)
{
    for (auto V : variables)
    {
        addVariable(V);
    }
}

void OptimizationProblem::addConstraint(LinearConstraintPtr constraint)
{
    linearConstraints.push_back(constraint);
    numericConstraints.push_back(std::dynamic_pointer_cast<NumericConstraint>(constraint));
}

void OptimizationProblem::addConstraint(QuadraticConstraintPtr constraint)
{
    quadraticConstraints.push_back(constraint);
    numericConstraints.push_back(std::dynamic_pointer_cast<NumericConstraint>(constraint));
}

void OptimizationProblem::addConstraint(NonlinearConstraintPtr constraint)
{
    nonlinearConstraints.push_back(constraint);
    numericConstraints.push_back(std::dynamic_pointer_cast<NumericConstraint>(constraint));
}

VariablePtr OptimizationProblem::getVariable(int variableIndex)
{
    if (variableIndex > allVariables.size())
    {
        throw new VariableNotFoundException(" with variableIndex " + std::to_string(variableIndex));
    }

    return allVariables.at(variableIndex);
}

NumericConstraints OptimizationProblem::getAllNumericConstraints()
{
    return numericConstraints;
}

LinearConstraints OptimizationProblem::getLinearConstraints()
{
    return linearConstraints;
}

QuadraticConstraints OptimizationProblem::getQuadraticConstraints()
{
    return quadraticConstraints;
}

NonlinearConstraints OptimizationProblem::getNonlinearConstraints()
{
    return nonlinearConstraints;
}

boost::optional<NumericConstraintValue> OptimizationProblem::getMostDeviatingNumericConstraint(const VectorDouble &point)
{
    return (this->getMostDeviatingNumericConstraint(point, numericConstraints));
}

boost::optional<NumericConstraintValue> OptimizationProblem::getMostDeviatingNumericConstraint(const VectorDouble &point, NumericConstraints constraintSelection)
{
    boost::optional<NumericConstraintValue> optionalConstraintValue;

    NumericConstraintValue constraintValue;

    for (auto C : constraintSelection)
    {
        constraintValue = C->calculateNumericValue(point);

        if (constraintValue.isFulfilled)
            continue;

        if (optionalConstraintValue) // No constraint with error found yet
        {
            optionalConstraintValue = constraintValue;
        }
        else if (constraintValue.error > optionalConstraintValue->error)
        {
            optionalConstraintValue = constraintValue;
        }
    }

    return constraintValue;
}

NumericConstraintValues OptimizationProblem::getAllDeviatingNumericConstraints(const VectorDouble &point, double tolerance)
{
    NumericConstraintValues constraintValues;

    for (auto C : numericConstraints)
    {
        NumericConstraintValue constraintValue = C->calculateNumericValue(point);

        if (constraintValue.error > tolerance)

            constraintValues.push_back(constraintValue);
    }

    return constraintValues;
}