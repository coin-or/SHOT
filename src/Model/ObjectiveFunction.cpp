/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "ObjectiveFunction.h"
#include "Variables.h"
#include "Terms.h"
#include "NonlinearExpressions.h"
#include "OptimizationProblem.h"

namespace SHOT
{

void ObjectiveFunction::takeOwnership(OptimizationProblemPtr owner)
{
    ownerProblem = owner;
};

E_Curvature ObjectiveFunction::checkConvexity()
{
    return E_Curvature::Convex;
};

void ObjectiveFunction::updateProperties()
{
    if (properties.direction == E_ObjectiveFunctionDirection::Minimize)
    {
        properties.isMinimize = true;
        properties.isMaximize = false;
    }

    properties.curvature = checkConvexity();
};

std::ostream &operator<<(std::ostream &stream, const ObjectiveFunction &objective)
{
    return objective.print(stream); // polymorphic print via reference
};

std::ostream &operator<<(std::ostream &stream, ObjectiveFunctionPtr objective)
{
    stream << *objective;
    return stream;
};

void LinearObjectiveFunction::add(LinearTerms terms)
{
    if (linearTerms.terms.size() == 0)
    {
        linearTerms = terms;
        properties.isValid = false;
    }
    else
    {
        for (auto T : terms.terms)
        {
            add(T);
        }
    }
};

void LinearObjectiveFunction::add(LinearTermPtr term)
{
    linearTerms.terms.push_back(term);
    properties.isValid = false;
};

void LinearObjectiveFunction::updateProperties()
{
    if (linearTerms.terms.size() > 0)
    {
        properties.hasLinearTerms = true;
    }

    if (!(properties.hasNonlinearExpression || properties.hasSignomialTerms || properties.hasNonalgebraicPart || properties.hasQuadraticTerms))
        E_ObjectiveFunctionClassification classification = E_ObjectiveFunctionClassification::Linear;

    ObjectiveFunction::updateProperties();
};

double LinearObjectiveFunction::calculateValue(const VectorDouble &point)
{
    double value = linearTerms.calculate(point);
    return value;
};

Interval LinearObjectiveFunction::calculateValue(const IntervalVector &intervalVector)
{
    Interval value = linearTerms.calculate(intervalVector);
    return value;
};

SparseVariableVector
LinearObjectiveFunction::calculateGradient(const VectorDouble &point)
{
    SparseVariableVector gradient;

    for (auto T : linearTerms.terms)
    {
        auto element = gradient.insert(std::make_pair(T->variable, T->coefficient));
        if (!element.second)
        {
            // Element already exists for the variable

            element.second += T->coefficient;
        }
    }

    UtilityFunctions::erase_if<VariablePtr, double>(gradient, 0.0);

    return gradient;
};

std::ostream &LinearObjectiveFunction::print(std::ostream &stream) const
{
    if (properties.isMinimize)
        stream << "minimize: ";
    else if (properties.isMaximize)
        stream << "maximize: ";

    if (constant != 0.0)
        stream << constant;

    if (properties.hasLinearTerms)
        stream << linearTerms;

    return stream;
};

std::ostream &operator<<(std::ostream &stream, LinearObjectiveFunctionPtr objective)
{
    stream << *objective;
    return stream;
};

void QuadraticObjectiveFunction::add(QuadraticTerms terms)
{
    if (quadraticTerms.terms.size() == 0)
    {
        quadraticTerms = terms;
        properties.isValid = false;
    }
    else
    {
        for (auto T : terms.terms)
        {
            add(T);
        }
    }
};

void QuadraticObjectiveFunction::add(QuadraticTermPtr term)
{
    quadraticTerms.terms.push_back(term);
    properties.isValid = false;
};

void QuadraticObjectiveFunction::updateProperties()
{
    if (quadraticTerms.terms.size() > 0)
    {
        properties.hasQuadraticTerms = true;

        if (!(properties.hasNonlinearExpression || properties.hasSignomialTerms || properties.hasNonalgebraicPart))
            E_ObjectiveFunctionClassification classification = E_ObjectiveFunctionClassification::Quadratic;
    }

    LinearObjectiveFunction::updateProperties();
};

double QuadraticObjectiveFunction::calculateValue(const VectorDouble &point)
{
    double value = LinearObjectiveFunction::calculateValue(point);
    value += quadraticTerms.calculate(point);
    return value;
};

Interval QuadraticObjectiveFunction::calculateValue(
    const IntervalVector &intervalVector)
{
    Interval value = LinearObjectiveFunction::calculateValue(intervalVector);
    value += quadraticTerms.calculate(intervalVector);
    return value;
};

SparseVariableVector
QuadraticObjectiveFunction::calculateGradient(const VectorDouble &point)
{
    SparseVariableVector gradient = LinearObjectiveFunction::calculateGradient(point);

    for (auto T : quadraticTerms.terms)
    {
        if (T->firstVariable == T->secondVariable) // variable squared
        {
            auto value = 2 * T->coefficient * point[T->firstVariable->index];
            auto element = gradient.insert(std::make_pair(T->firstVariable, value));

            if (!element.second)
            {
                // Element already exists for the variable
                element.first->second += value;
            }
        }
        else
        {
            auto value = T->coefficient * point[T->secondVariable->index];
            auto element = gradient.insert(std::make_pair(T->firstVariable, value));

            if (!element.second)
            {
                // Element already exists for the variable
                element.first->second += value;
            }

            value = T->coefficient * point[T->firstVariable->index];
            element = gradient.insert(std::make_pair(T->secondVariable, value));

            if (!element.second)
            {
                // Element already exists for the variable
                element.first->second += value;
            }
        }
    }

    UtilityFunctions::erase_if<VariablePtr, double>(gradient, 0.0);

    return gradient;
};

std::ostream &QuadraticObjectiveFunction::print(std::ostream &stream) const
{
    LinearObjectiveFunction::print(stream);

    if (properties.hasQuadraticTerms)
        stream << quadraticTerms;

    return stream;
};

std::ostream &operator<<(std::ostream &stream, QuadraticObjectiveFunctionPtr objective)
{
    stream << *objective;
    return stream;
};

void NonlinearObjectiveFunction::add(NonlinearExpressionPtr expression)
{
    if (nonlinearExpression != nullptr)
    {
        auto tmpExpr = nonlinearExpression;
        auto nonlinearExpression(new ExpressionPlus(tmpExpr, expression));
    }
    else
    {
        nonlinearExpression = expression;
    }

    properties.isValid = false;
}

void NonlinearObjectiveFunction::updateFactorableFunction()
{
    factorableFunction = std::make_shared<FactorableFunction>(nonlinearExpression->getFactorableFunction());
}

void NonlinearObjectiveFunction::updateProperties()
{
    if (nonlinearExpression != nullptr)
    {
        properties.hasNonlinearExpression = true;

        if (properties.hasNonalgebraicPart)
        {
            E_ObjectiveFunctionClassification classification = E_ObjectiveFunctionClassification::Nonalgebraic;
        }
        else if (properties.hasSignomialTerms)
        {
            E_ObjectiveFunctionClassification classification = E_ObjectiveFunctionClassification::GeneralizedSignomial;
        }
        else
        {
            E_ObjectiveFunctionClassification classification = E_ObjectiveFunctionClassification::Nonlinear;
        }
    }

    QuadraticObjectiveFunction::updateProperties();
}

double NonlinearObjectiveFunction::calculateValue(const VectorDouble &point)
{
    double value = QuadraticObjectiveFunction::calculateValue(point);
    value += nonlinearExpression->calculate(point);
    return value;
};

Interval NonlinearObjectiveFunction::calculateValue(
    const IntervalVector &intervalVector)
{
    Interval value = QuadraticObjectiveFunction::calculateValue(intervalVector);
    value += nonlinearExpression->calculate(intervalVector);

    return value;
};

SparseVariableVector NonlinearObjectiveFunction::calculateGradient(const VectorDouble &point)
{
    SparseVariableVector gradient = QuadraticObjectiveFunction::calculateGradient(point);

    for (auto E : symbolicSparseJacobian)
    {
        double value[1];
        ownerProblem->factorableFunctionsDAG->eval(1, &E.second, value, 3, &ownerProblem->factorableFunctionVariables[0], &point[0]);

        auto element = gradient.insert(std::make_pair(E.first, value[0]));

        if (!element.second)
        {
            // Element already exists for the variable
            element.first->second += value[0];
        }
    }

    UtilityFunctions::erase_if<VariablePtr, double>(gradient, 0.0);

    return gradient;
};

std::ostream &NonlinearObjectiveFunction::print(std::ostream &stream) const
{
    QuadraticObjectiveFunction::print(stream);

    if (properties.hasNonlinearExpression)
        stream << " +(" << nonlinearExpression << ')';

    return stream;
};

std::ostream &operator<<(std::ostream &stream, NonlinearObjectiveFunctionPtr objective)
{
    stream << *objective;
    return stream;
};

} // namespace SHOT