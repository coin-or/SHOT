/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "ObjectiveFunction.h"

namespace SHOT
{

void ObjectiveFunction::takeOwnership(ProblemPtr owner) { ownerProblem = owner; };

void ObjectiveFunction::updateProperties()
{
    if(direction == E_ObjectiveFunctionDirection::Minimize)
    {
        properties.isMinimize = true;
        properties.isMaximize = false;
    }
    else
    {
        properties.isMinimize = false;
        properties.isMaximize = true;
    }

    properties.curvature = checkConvexity();
};

std::ostream& operator<<(std::ostream& stream, const ObjectiveFunction& objective)
{
    return objective.print(stream); // polymorphic print via reference
};

std::ostream& operator<<(std::ostream& stream, ObjectiveFunctionPtr objective)
{
    stream << *objective;
    return stream;
};

void LinearObjectiveFunction::add(LinearTerms terms)
{
    if(linearTerms.size() == 0)
    {
        linearTerms = terms;
        properties.isValid = false;
    }
    else
    {
        for(auto T : terms)
        {
            add(T);
        }
    }
};

void LinearObjectiveFunction::add(LinearTermPtr term)
{
    linearTerms.push_back(term);
    properties.isValid = false;
};

void LinearObjectiveFunction::updateProperties()
{
    if(linearTerms.size() == 0)
    {
        properties.hasLinearTerms = false;
    }
    else
    {
        properties.hasLinearTerms = true;
    }

    properties.classification = E_ObjectiveFunctionClassification::Linear;

    ObjectiveFunction::updateProperties();
};

E_Curvature LinearObjectiveFunction::checkConvexity() { return E_Curvature::Convex; };

double LinearObjectiveFunction::calculateValue(const VectorDouble& point)
{
    double value = constant + linearTerms.calculate(point);
    return value;
};

Interval LinearObjectiveFunction::calculateValue(const IntervalVector& intervalVector)
{
    Interval value = linearTerms.calculate(intervalVector);
    return value;
};

SparseVariableVector LinearObjectiveFunction::calculateGradient(const VectorDouble& point)
{
    SparseVariableVector gradient;

    for(auto T : linearTerms)
    {
        auto element = gradient.insert(std::make_pair(T->variable, T->coefficient));
        if(!element.second)
        {
            // Element already exists for the variable

            element.second += T->coefficient;
        }
    }

    UtilityFunctions::erase_if<VariablePtr, double>(gradient, 0.0);

    return gradient;
};

std::ostream& LinearObjectiveFunction::print(std::ostream& stream) const
{
    if(properties.isMinimize)
        stream << "minimize: ";
    else if(properties.isMaximize)
        stream << "maximize: ";

    if(constant != 0.0)
        stream << constant;

    if(properties.hasLinearTerms)
        stream << linearTerms;

    return stream;
};

std::ostream& operator<<(std::ostream& stream, LinearObjectiveFunctionPtr objective)
{
    stream << *objective;
    return stream;
};

void QuadraticObjectiveFunction::add(QuadraticTerms terms)
{
    if(quadraticTerms.size() == 0)
    {
        quadraticTerms = terms;
        properties.isValid = false;
    }
    else
    {
        for(auto T : terms)
        {
            add(T);
        }
    }
};

void QuadraticObjectiveFunction::add(QuadraticTermPtr term)
{
    quadraticTerms.push_back(term);
    properties.isValid = false;
};

void QuadraticObjectiveFunction::updateProperties()
{
    LinearObjectiveFunction::updateProperties();

    if(quadraticTerms.size() > 0)
    {
        properties.hasQuadraticTerms = true;
        properties.classification = E_ObjectiveFunctionClassification::Quadratic;

        for(auto const& T : quadraticTerms)
        {
            if(T->isBilinear)
            {
                if(T->isBinary)
                {
                    properties.hasBinaryBilinearTerms = true;
                }
                else
                {
                    properties.hasNonBinaryBilinearTerms = true;
                }
            }
            else
            {
                if(T->isBinary)
                {
                    properties.hasBinarySquareTerms = true;
                }
                else
                {
                    properties.hasNonBinarySquareTerms = true;
                }
            }
        }
    }
    else
    {
        properties.hasQuadraticTerms = false;
    }
};

E_Curvature QuadraticObjectiveFunction::checkConvexity() { return E_Curvature::Convex; };

double QuadraticObjectiveFunction::calculateValue(const VectorDouble& point)
{
    double value = LinearObjectiveFunction::calculateValue(point);
    value += quadraticTerms.calculate(point);
    return value;
};

Interval QuadraticObjectiveFunction::calculateValue(const IntervalVector& intervalVector)
{
    Interval value = LinearObjectiveFunction::calculateValue(intervalVector);
    value += quadraticTerms.calculate(intervalVector);
    return value;
};

SparseVariableVector QuadraticObjectiveFunction::calculateGradient(const VectorDouble& point)
{
    SparseVariableVector gradient = LinearObjectiveFunction::calculateGradient(point);

    for(auto T : quadraticTerms)
    {
        if(T->firstVariable == T->secondVariable) // variable squared
        {
            auto value = 2 * T->coefficient * point[T->firstVariable->index];
            auto element = gradient.insert(std::make_pair(T->firstVariable, value));

            if(!element.second)
            {
                // Element already exists for the variable
                element.first->second += value;
            }
        }
        else
        {
            auto value = T->coefficient * point[T->secondVariable->index];
            auto element = gradient.insert(std::make_pair(T->firstVariable, value));

            if(!element.second)
            {
                // Element already exists for the variable
                element.first->second += value;
            }

            value = T->coefficient * point[T->firstVariable->index];
            element = gradient.insert(std::make_pair(T->secondVariable, value));

            if(!element.second)
            {
                // Element already exists for the variable
                element.first->second += value;
            }
        }
    }

    UtilityFunctions::erase_if<VariablePtr, double>(gradient, 0.0);

    return gradient;
};

std::ostream& QuadraticObjectiveFunction::print(std::ostream& stream) const
{
    LinearObjectiveFunction::print(stream);

    if(properties.hasQuadraticTerms)
        stream << quadraticTerms;

    return stream;
};

std::ostream& operator<<(std::ostream& stream, QuadraticObjectiveFunctionPtr objective)
{
    stream << *objective;
    return stream;
};

void NonlinearObjectiveFunction::add(NonlinearExpressionPtr expression)
{
    if(nonlinearExpression.get() != nullptr)
    {
        auto tmpExpr = nonlinearExpression;
        auto nonlinearExpression(std::make_shared<ExpressionPlus>(tmpExpr, expression));
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
    QuadraticObjectiveFunction::updateProperties();

    if(nonlinearExpression != nullptr)
    {
        properties.hasNonlinearExpression = true;
        properties.classification = E_ObjectiveFunctionClassification::Nonlinear;
    }
    else
    {
        properties.hasNonlinearExpression = false;
    }
}

E_Curvature NonlinearObjectiveFunction::checkConvexity() { return E_Curvature::Convex; };

double NonlinearObjectiveFunction::calculateValue(const VectorDouble& point)
{
    double value = QuadraticObjectiveFunction::calculateValue(point);

    if(this->properties.hasNonlinearExpression)
        value += nonlinearExpression->calculate(point);

    return value;
};

Interval NonlinearObjectiveFunction::calculateValue(const IntervalVector& intervalVector)
{
    Interval value = QuadraticObjectiveFunction::calculateValue(intervalVector);

    if(this->properties.hasNonlinearExpression)
        value += nonlinearExpression->calculate(intervalVector);

    return value;
};

SparseVariableVector NonlinearObjectiveFunction::calculateGradient(const VectorDouble& point)
{
    SparseVariableVector gradient = QuadraticObjectiveFunction::calculateGradient(point);

    for(auto E : symbolicSparseJacobian)
    {
        double value[1];

        if(auto sharedOwnerProblem = ownerProblem.lock())
        {
            // Collecting the values corresponding to nonlinear variables from the point
            VectorDouble newPoint;
            newPoint.reserve(sharedOwnerProblem->factorableFunctionVariables.size());

            for(auto& V : sharedOwnerProblem->nonlinearVariables)
            {
                newPoint.push_back(point.at(V->index));
            }

            sharedOwnerProblem->factorableFunctionsDAG->eval(1, &E.second, value,
                sharedOwnerProblem->factorableFunctionVariables.size(),
                &sharedOwnerProblem->factorableFunctionVariables[0], &newPoint[0]);
        }

        auto element = gradient.insert(std::make_pair(E.first, value[0]));

        if(!element.second)
        {
            // Element already exists for the variable
            element.first->second += value[0];
        }
    }

    UtilityFunctions::erase_if<VariablePtr, double>(gradient, 0.0);

    return gradient;
};

std::ostream& NonlinearObjectiveFunction::print(std::ostream& stream) const
{
    QuadraticObjectiveFunction::print(stream);

    if(properties.hasNonlinearExpression)
        stream << " +(" << nonlinearExpression << ')';

    return stream;
};

std::ostream& operator<<(std::ostream& stream, NonlinearObjectiveFunctionPtr objective)
{
    stream << *objective;
    return stream;
};

} // namespace SHOT