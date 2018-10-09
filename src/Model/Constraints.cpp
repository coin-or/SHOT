/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "Variables.h"
#include "Terms.h"
#include "NonlinearExpressions.h"
#include "Constraints.h"
#include "Problem.h"
namespace SHOT
{

void Constraint::takeOwnership(ProblemPtr owner)
{
    ownerProblem = owner;
}

std::ostream &operator<<(std::ostream &stream, const Constraint &constraint)
{
    stream << "[" << constraint.index << "]";

    if (constraint.name != "")
        stream << ' ' << constraint.name;

    stream << ":\t";

    return constraint.print(stream); // polymorphic print via reference
}

std::ostream &operator<<(std::ostream &stream, ConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
}

NumericConstraintValue NumericConstraint::calculateNumericValue(const VectorDouble &point)
{
    double value = calculateFunctionValue(point);

    NumericConstraintValue constrValue;
    constrValue.constraint = getPointer();
    constrValue.functionValue = value;
    constrValue.isFulfilledRHS = (value <= valueRHS);
    constrValue.errorRHS = std::max(value - valueRHS, 0.0);

    constrValue.isFulfilledLHS = (value >= valueLHS);
    constrValue.errorLHS = std::max(valueLHS - value, 0.0);

    constrValue.isFulfilled =
        (constrValue.isFulfilledRHS && constrValue.isFulfilledLHS);
    constrValue.error = std::max(constrValue.errorRHS, constrValue.errorLHS);

    return constrValue;
}

bool NumericConstraint::isFulfilled(const VectorDouble &point)
{
    auto constraintValue = calculateNumericValue(point);

    return (constraintValue.isFulfilledLHS && constraintValue.isFulfilledRHS);
};

void LinearConstraint::add(LinearTerms terms)
{
    if (linearTerms.terms.size() == 0)
    {
        linearTerms = terms;
    }
    else
    {
        for (auto T : terms.terms)
        {
            add(T);
        }
    }
};

void LinearConstraint::add(LinearTermPtr term)
{
    linearTerms.terms.push_back(term);
};

double LinearConstraint::calculateFunctionValue(const VectorDouble &point)
{
    double value = linearTerms.calculate(point);
    value += constant;
    return value;
};

Interval LinearConstraint::calculateFunctionValue(const IntervalVector &intervalVector)
{
    Interval value = linearTerms.calculate(intervalVector);
    value += Interval(constant);
    return value;
};

bool LinearConstraint::isFulfilled(const VectorDouble &point)
{
    return NumericConstraint::isFulfilled(point);
};

SparseVariableVector LinearConstraint::calculateGradient(const VectorDouble &point)
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

NumericConstraintValue LinearConstraint::calculateNumericValue(const VectorDouble &point)
{
    return NumericConstraint::calculateNumericValue(point);
};

std::shared_ptr<NumericConstraint> LinearConstraint::getPointer()
{
    return std::dynamic_pointer_cast<NumericConstraint>(shared_from_this());
};

void QuadraticConstraint::add(LinearTerms terms)
{
    LinearConstraint::add(terms);
};

void QuadraticConstraint::add(LinearTermPtr term)
{
    LinearConstraint::add(term);
};

void QuadraticConstraint::add(QuadraticTerms terms)
{
    if (quadraticTerms.terms.size() == 0)
    {
        quadraticTerms = terms;
    }
    else
    {
        for (auto T : terms.terms)
        {
            add(T);
        }
    }
};

void QuadraticConstraint::add(QuadraticTermPtr term)
{
    quadraticTerms.terms.push_back(term);
};

double QuadraticConstraint::calculateFunctionValue(const VectorDouble &point)
{
    double value = LinearConstraint::calculateFunctionValue(point);
    value += quadraticTerms.calculate(point);

    return value;
};

Interval QuadraticConstraint::calculateFunctionValue(const IntervalVector &intervalVector)
{
    Interval value = LinearConstraint::calculateFunctionValue(intervalVector);
    value += quadraticTerms.calculate(intervalVector);
    return value;
};

bool QuadraticConstraint::isFulfilled(const VectorDouble &point)
{
    return NumericConstraint::isFulfilled(point);
};

SparseVariableVector QuadraticConstraint::calculateGradient(const VectorDouble &point)
{
    SparseVariableVector gradient = LinearConstraint::calculateGradient(point);

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

NumericConstraintValue QuadraticConstraint::calculateNumericValue(const VectorDouble &point)
{
    return NumericConstraint::calculateNumericValue(point);
};

std::shared_ptr<NumericConstraint> QuadraticConstraint::getPointer()
{
    return std::dynamic_pointer_cast<NumericConstraint>(shared_from_this());
};

void NonlinearConstraint::add(LinearTerms terms)
{
    LinearConstraint::add(terms);
};

void NonlinearConstraint::add(LinearTermPtr term)
{
    LinearConstraint::add(term);
};

void NonlinearConstraint::add(QuadraticTerms terms)
{
    QuadraticConstraint::add(terms);
};

void NonlinearConstraint::add(QuadraticTermPtr term)
{
    QuadraticConstraint::add(term);
};

void NonlinearConstraint::add(NonlinearExpressionPtr expression)
{
    /*if (nonlinearExpression.get() != nullptr)
    {
        nonlinearExpression = std::make_shared<ExpressionPlus>(nonlinearExpression, expression);
    }
    else
    {*/
    std::cout << "E: " << expression << std::endl;
    nonlinearExpression = expression;
    //}
};

void NonlinearConstraint::updateFactorableFunction()
{
    factorableFunction = std::make_shared<FactorableFunction>(nonlinearExpression->getFactorableFunction());
};

double NonlinearConstraint::calculateFunctionValue(const VectorDouble &point)
{
    double value = QuadraticConstraint::calculateFunctionValue(point);
    value += nonlinearExpression->calculate(point);

    return value;
};

Interval NonlinearConstraint::calculateFunctionValue(const IntervalVector &intervalVector)
{
    Interval value = QuadraticConstraint::calculateFunctionValue(intervalVector);
    value += nonlinearExpression->calculate(intervalVector);

    return value;
};

SparseVariableVector NonlinearConstraint::calculateGradient(const VectorDouble &point)
{
    SparseVariableVector gradient = QuadraticConstraint::calculateGradient(point);

    for (auto E : symbolicSparseJacobian)
    {
        double value[1];

        if (auto sharedOwnerProblem = ownerProblem.lock())
        {
            sharedOwnerProblem->factorableFunctionsDAG->eval(1, &E.second, value, sharedOwnerProblem->factorableFunctionVariables.size(), &sharedOwnerProblem->factorableFunctionVariables[0], &point[0]);
        }

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

bool NonlinearConstraint::isFulfilled(const VectorDouble &point)
{
    return NumericConstraint::isFulfilled(point);
};

NumericConstraintValue NonlinearConstraint::calculateNumericValue(const VectorDouble &point)
{
    return NumericConstraint::calculateNumericValue(point);
};

std::shared_ptr<NumericConstraint> NonlinearConstraint::getPointer()
{
    return std::dynamic_pointer_cast<NumericConstraint>(shared_from_this());
};

std::ostream &operator<<(std::ostream &stream, NumericConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
};

std::ostream &operator<<(std::ostream &stream, LinearConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
};

std::ostream &LinearConstraint::print(std::ostream &stream) const
{
    if (valueLHS > -std::numeric_limits<double>::max())
        stream << valueLHS << " <= ";

    if (linearTerms.terms.size() > 0)
        stream << linearTerms;

    if (valueRHS < std::numeric_limits<double>::max())
        stream << " <= " << valueRHS;

    return stream;
};

std::ostream &operator<<(std::ostream &stream,
                         QuadraticConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
};

std::ostream &QuadraticConstraint::print(std::ostream &stream) const
{
    if (valueLHS > -std::numeric_limits<double>::max())
        stream << valueLHS << " <= ";

    if (linearTerms.terms.size() > 0)
        stream << linearTerms;

    if (quadraticTerms.terms.size() > 0)
        stream << " +" << quadraticTerms;

    if (valueRHS < std::numeric_limits<double>::max())
        stream << " <= " << valueRHS;

    return stream;
};

std::ostream &operator<<(std::ostream &stream, NonlinearConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
};

std::ostream &NonlinearConstraint::print(std::ostream &stream) const
{
    if (valueLHS > -std::numeric_limits<double>::max())
        stream << valueLHS << " <= ";

    if (linearTerms.terms.size() > 0)
        stream << linearTerms;

    if (quadraticTerms.terms.size() > 0)
        stream << " +" << quadraticTerms;

    stream << " +" << nonlinearExpression;

    if (valueRHS < std::numeric_limits<double>::max())
        stream << " <= " << valueRHS;

    return stream;
};
} // namespace SHOT