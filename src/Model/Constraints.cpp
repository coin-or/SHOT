/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "Constraints.h"

namespace SHOT
{

void Constraint::takeOwnership(ProblemPtr owner) { ownerProblem = owner; }

std::ostream& operator<<(std::ostream& stream, const Constraint& constraint)
{
    stream << "[" << constraint.index << "]";

    switch(constraint.properties.classification)
    {
    case(E_ConstraintClassification::Linear):
        stream << "(L)   ";
        break;

    case(E_ConstraintClassification::Quadratic):
        stream << "(Q)   ";
        break;

    case(E_ConstraintClassification::QuadraticConsideredAsNonlinear):
        stream << "(QNL) ";
        break;

    case(E_ConstraintClassification::Nonlinear):
        stream << "(NL)  ";
        break;

    default:
        stream << "(?)   ";
        break;
    }

    if(constraint.name != "")
        stream << ' ' << constraint.name;

    stream << ":\t";

    return constraint.print(stream); // polymorphic print via reference
}

std::ostream& operator<<(std::ostream& stream, ConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
}

NumericConstraintValue NumericConstraint::calculateNumericValue(const VectorDouble& point, double correction)
{
    double value = calculateFunctionValue(point) - correction;

    NumericConstraintValue constrValue;
    constrValue.constraint = getPointer();
    constrValue.functionValue = value;
    constrValue.isFulfilledRHS = (value <= valueRHS);
    constrValue.normalizedRHSValue = value - valueRHS;

    constrValue.isFulfilledLHS = (value >= valueLHS);
    constrValue.normalizedLHSValue = valueLHS - value;

    constrValue.isFulfilled = (constrValue.isFulfilledRHS && constrValue.isFulfilledLHS);

    constrValue.normalizedValue = std::max(constrValue.normalizedRHSValue, constrValue.normalizedLHSValue);
    constrValue.error = std::max(0.0, constrValue.normalizedValue);

    return constrValue;
}

bool NumericConstraint::isFulfilled(const VectorDouble& point)
{
    auto constraintValue = calculateNumericValue(point);

    return (constraintValue.isFulfilledLHS && constraintValue.isFulfilledRHS);
};

void LinearConstraint::add(LinearTerms terms)
{
    if(linearTerms.size() == 0)
    {
        linearTerms = terms;
        properties.hasLinearTerms = true;
    }
    else
    {
        for(auto T : terms)
        {
            add(T);
        }
    }
};

void LinearConstraint::add(LinearTermPtr term)
{
    linearTerms.add(term);
    properties.hasLinearTerms = true;
};

double LinearConstraint::calculateFunctionValue(const VectorDouble& point)
{
    double value = linearTerms.calculate(point);
    value += constant;
    return value;
};

Interval LinearConstraint::calculateFunctionValue(const IntervalVector& intervalVector)
{
    Interval value = linearTerms.calculate(intervalVector);
    value += Interval(constant);
    return value;
};

bool LinearConstraint::isFulfilled(const VectorDouble& point) { return NumericConstraint::isFulfilled(point); };

SparseVariableVector LinearConstraint::calculateGradient(const VectorDouble& point, bool eraseZeroes = true)
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

    if(eraseZeroes)
        UtilityFunctions::erase_if<VariablePtr, double>(gradient, 0.0);

    return gradient;
};

NumericConstraintValue LinearConstraint::calculateNumericValue(const VectorDouble& point, double correction)
{
    return NumericConstraint::calculateNumericValue(point);
};

std::shared_ptr<NumericConstraint> LinearConstraint::getPointer()
{
    return std::dynamic_pointer_cast<NumericConstraint>(shared_from_this());
};

void LinearConstraint::updateProperties()
{
    if(linearTerms.size() > 0)
    {
        properties.hasLinearTerms = true;
        properties.classification = E_ConstraintClassification::Linear;
    }
    else
    {
        properties.hasLinearTerms = false;
        properties.classification = E_ConstraintClassification::Linear;
    }
};

void QuadraticConstraint::add(LinearTerms terms) { LinearConstraint::add(terms); };

void QuadraticConstraint::add(LinearTermPtr term) { LinearConstraint::add(term); };

void QuadraticConstraint::add(QuadraticTerms terms)
{
    if(quadraticTerms.size() == 0)
    {
        quadraticTerms = terms;
        properties.hasQuadraticTerms = true;
    }
    else
    {
        for(auto T : terms)
        {
            add(T);
        }
    }
};

void QuadraticConstraint::add(QuadraticTermPtr term)
{
    quadraticTerms.push_back(term);
    properties.hasQuadraticTerms = true;
};

double QuadraticConstraint::calculateFunctionValue(const VectorDouble& point)
{
    double value = LinearConstraint::calculateFunctionValue(point);
    value += quadraticTerms.calculate(point);

    return value;
};

Interval QuadraticConstraint::calculateFunctionValue(const IntervalVector& intervalVector)
{
    Interval value = LinearConstraint::calculateFunctionValue(intervalVector);
    value += quadraticTerms.calculate(intervalVector);
    return value;
};

bool QuadraticConstraint::isFulfilled(const VectorDouble& point) { return NumericConstraint::isFulfilled(point); };

SparseVariableVector QuadraticConstraint::calculateGradient(const VectorDouble& point, bool eraseZeroes = true)
{
    SparseVariableVector gradient = LinearConstraint::calculateGradient(point, eraseZeroes);

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

    if(eraseZeroes)
        UtilityFunctions::erase_if<VariablePtr, double>(gradient, 0.0);

    return gradient;
};

NumericConstraintValue QuadraticConstraint::calculateNumericValue(const VectorDouble& point, double correction)
{
    return NumericConstraint::calculateNumericValue(point);
};

std::shared_ptr<NumericConstraint> QuadraticConstraint::getPointer()
{
    return std::dynamic_pointer_cast<NumericConstraint>(shared_from_this());
};

void QuadraticConstraint::updateProperties()
{
    LinearConstraint::updateProperties();

    if(quadraticTerms.size() > 0)
    {
        properties.hasQuadraticTerms = true;
        properties.classification = E_ConstraintClassification::Quadratic;
    }
    else
    {
        properties.hasQuadraticTerms = false;
    }
};

void NonlinearConstraint::add(LinearTerms terms) { LinearConstraint::add(terms); };

void NonlinearConstraint::add(LinearTermPtr term) { LinearConstraint::add(term); };

void NonlinearConstraint::add(QuadraticTerms terms) { QuadraticConstraint::add(terms); };

void NonlinearConstraint::add(QuadraticTermPtr term) { QuadraticConstraint::add(term); };

void NonlinearConstraint::add(NonlinearExpressionPtr expression)
{
    if(nonlinearExpression.get() != nullptr)
    {
        nonlinearExpression = std::make_shared<ExpressionPlus>(nonlinearExpression, expression);
    }
    else
    {
        nonlinearExpression = expression;
    }

    properties.hasNonlinearExpression = true;
};

void NonlinearConstraint::updateFactorableFunction()
{
    factorableFunction = std::make_shared<FactorableFunction>(nonlinearExpression->getFactorableFunction());
};

double NonlinearConstraint::calculateFunctionValue(const VectorDouble& point)
{
    double value = QuadraticConstraint::calculateFunctionValue(point);

    if(this->properties.hasNonlinearExpression)
        value += nonlinearExpression->calculate(point);

    return value;
};

Interval NonlinearConstraint::calculateFunctionValue(const IntervalVector& intervalVector)
{
    Interval value = QuadraticConstraint::calculateFunctionValue(intervalVector);

    if(this->properties.hasNonlinearExpression)
        value += nonlinearExpression->calculate(intervalVector);

    return value;
};

SparseVariableVector NonlinearConstraint::calculateGradient(const VectorDouble& point, bool eraseZeroes = true)
{
    SparseVariableVector gradient = QuadraticConstraint::calculateGradient(point, eraseZeroes);

    try
    {
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

            if(value[0] != value[0])
            {
                std::cout << "nan" << std::endl;
                value[0] = 0.0;
            }

            auto element = gradient.insert(std::make_pair(E.first, value[0]));

            if(!element.second)
            {
                // Element already exists for the variable
                element.first->second += value[0];
            }
        }

        if(eraseZeroes)
            UtilityFunctions::erase_if<VariablePtr, double>(gradient, 0.0);
    }
    catch(mc::FFGraph::Exceptions& e)
    {
        std::cout << "Error when evaluating gradient: " << e.what();
    }

    return gradient;
};

bool NonlinearConstraint::isFulfilled(const VectorDouble& point) { return NumericConstraint::isFulfilled(point); };

NumericConstraintValue NonlinearConstraint::calculateNumericValue(const VectorDouble& point, double correction)
{
    return NumericConstraint::calculateNumericValue(point);
};

std::shared_ptr<NumericConstraint> NonlinearConstraint::getPointer()
{
    return std::dynamic_pointer_cast<NumericConstraint>(shared_from_this());
};

void NonlinearConstraint::updateProperties()
{
    QuadraticConstraint::updateProperties();

    if(nonlinearExpression != nullptr)
    {
        properties.hasNonlinearExpression = true;
        properties.classification = E_ConstraintClassification::Nonlinear;
    }
    else
    {
        properties.hasNonlinearExpression = false;
        properties.classification = E_ConstraintClassification::Nonlinear;
    }
};

std::ostream& operator<<(std::ostream& stream, NumericConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
};

std::ostream& operator<<(std::ostream& stream, LinearConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
};

std::ostream& LinearConstraint::print(std::ostream& stream) const
{
    if(valueLHS > SHOT_DBL_MIN && valueLHS != valueRHS)
        stream << valueLHS << " <= ";

    if(linearTerms.size() > 0)
        stream << linearTerms;

    if(constant > 0)
        stream << '+' << constant;

    if(constant < 0)
        stream << constant;

    if(valueLHS == valueRHS)
        stream << " = " << valueRHS;
    else if(valueRHS < SHOT_DBL_MAX)
        stream << " <= " << valueRHS;

    return stream;
};

std::ostream& operator<<(std::ostream& stream, QuadraticConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
};

std::ostream& QuadraticConstraint::print(std::ostream& stream) const
{
    if(valueLHS > SHOT_DBL_MIN)
        stream << valueLHS << " <= ";

    if(linearTerms.size() > 0)
        stream << linearTerms;

    if(quadraticTerms.size() > 0)
        stream << " +" << quadraticTerms;

    if(constant > 0)
        stream << '+' << constant;

    if(constant < 0)
        stream << constant;

    if(valueRHS < SHOT_DBL_MAX)
        stream << " <= " << valueRHS;

    return stream;
};

std::ostream& operator<<(std::ostream& stream, NonlinearConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
};

std::ostream& NonlinearConstraint::print(std::ostream& stream) const
{
    if(valueLHS > SHOT_DBL_MIN)
        stream << valueLHS << " <= ";

    if(linearTerms.size() > 0)
        stream << linearTerms;

    if(quadraticTerms.size() > 0)
        stream << " +" << quadraticTerms;

    stream << " +" << nonlinearExpression;

    if(constant > 0)
        stream << '+' << constant;

    if(constant < 0)
        stream << constant;

    if(valueRHS < SHOT_DBL_MAX)
        stream << " <= " << valueRHS;

    return stream;
};
} // namespace SHOT