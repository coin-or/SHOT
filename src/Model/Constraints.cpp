/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "Constraints.h"
#include "../Utilities.h"
#include "Problem.h"

namespace SHOT
{

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
    case(E_ConstraintClassification::Nonlinear):
        stream << "(NL)  ";
        break;

    default:
        stream << "(?)   ";
        break;
    }

    if(constraint.name != "")
        stream << ' ' << constraint.name;

    switch(constraint.properties.convexity)
    {
    case(E_Convexity::Linear):
        stream << " (linear)";
        break;
    case(E_Convexity::Convex):
        stream << " (convex)";
        break;
    case(E_Convexity::Nonconvex):
        stream << " (nonconvex)";
        break;
    case(E_Convexity::NotSet):
    case(E_Convexity::Unknown):
        break;
    default:
        break;
    }

    stream << ":\t";

    return constraint.print(stream); // polymorphic print via reference
}

std::ostream& operator<<(std::ostream& stream, ConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
}

std::shared_ptr<Variables> NumericConstraint::getGradientSparsityPattern()
{
    if(gradientSparsityPattern)
        return (gradientSparsityPattern);

    gradientSparsityPattern = std::make_shared<Variables>();
    initializeGradientSparsityPattern();

    // Sorts the variables
    std::sort(gradientSparsityPattern->begin(), gradientSparsityPattern->end(),
        [](const VariablePtr& variableOne, const VariablePtr& variableTwo) {
            return (variableOne->index < variableTwo->index);
        });

    // Remove duplicates
    // auto last = std::unique(gradientSparsityPattern->begin(), gradientSparsityPattern->end());
    // gradientSparsityPattern->erase(last, gradientSparsityPattern->end());

    return (gradientSparsityPattern);
}

std::shared_ptr<std::vector<std::pair<VariablePtr, VariablePtr>>> NumericConstraint::getHessianSparsityPattern()
{
    if(hessianSparsityPattern)
        return (hessianSparsityPattern);

    hessianSparsityPattern = std::make_shared<std::vector<std::pair<VariablePtr, VariablePtr>>>();
    initializeHessianSparsityPattern();

    // Sorts the elements
    std::sort(hessianSparsityPattern->begin(), hessianSparsityPattern->end(),
        [](const std::pair<VariablePtr, VariablePtr>& elementOne,
            const std::pair<VariablePtr, VariablePtr>& elementTwo) {
            if(elementOne.first->index < elementTwo.first->index)
                return (true);
            if(elementOne.second->index == elementTwo.second->index)
                return (elementOne.first->index < elementTwo.first->index);
            return (false);
        });

    return (hessianSparsityPattern);
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
        for(auto& T : terms)
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

void LinearConstraint::takeOwnership(ProblemPtr owner)
{
    ownerProblem = owner;
    linearTerms.takeOwnership(owner);
};

SparseVariableVector LinearConstraint::calculateGradient(const VectorDouble& point, bool eraseZeroes = true)
{
    SparseVariableVector gradient = linearTerms.calculateGradient(point);

    if(eraseZeroes)
        Utilities::erase_if<VariablePtr, double>(gradient, 0.0);

    return gradient;
};

void LinearConstraint::initializeGradientSparsityPattern()
{
    for(auto& T : linearTerms)
    {
        if(T->coefficient == 0.0)
            continue;

        if(std::find(gradientSparsityPattern->begin(), gradientSparsityPattern->end(), T->variable)
            == gradientSparsityPattern->end())
            gradientSparsityPattern->push_back(T->variable);
    }
};

void LinearConstraint::initializeHessianSparsityPattern(){};

SparseVariableMatrix LinearConstraint::calculateHessian(const VectorDouble& point, bool eraseZeroes = true)
{
    SparseVariableMatrix hessian;

    return hessian;
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
    }
    else
    {
        properties.hasLinearTerms = false;
    }

    properties.convexity = E_Convexity::Linear;
    properties.classification = E_ConstraintClassification::Linear;
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
        for(auto& T : terms)
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

void QuadraticConstraint::takeOwnership(ProblemPtr owner)
{
    LinearConstraint::takeOwnership(owner);
    quadraticTerms.takeOwnership(owner);
};

SparseVariableVector QuadraticConstraint::calculateGradient(const VectorDouble& point, bool eraseZeroes = true)
{
    SparseVariableVector linearGradient = LinearConstraint::calculateGradient(point, eraseZeroes);
    SparseVariableVector quadraticGradient = quadraticTerms.calculateGradient(point);

    return (Utilities::combineSparseVariableVectors(linearGradient, quadraticGradient));
};

void QuadraticConstraint::initializeGradientSparsityPattern()
{
    LinearConstraint::initializeGradientSparsityPattern();

    for(auto& T : quadraticTerms)
    {
        if(T->coefficient == 0.0)
            continue;

        if(std::find(gradientSparsityPattern->begin(), gradientSparsityPattern->end(), T->firstVariable)
            == gradientSparsityPattern->end())
            gradientSparsityPattern->push_back(T->firstVariable);

        if(T->firstVariable == T->secondVariable)
            continue;

        if(std::find(gradientSparsityPattern->begin(), gradientSparsityPattern->end(), T->secondVariable)
            == gradientSparsityPattern->end())
            gradientSparsityPattern->push_back(T->secondVariable);
    }
};

SparseVariableMatrix QuadraticConstraint::calculateHessian(const VectorDouble& point, bool eraseZeroes = true)
{
    SparseVariableMatrix hessian;

    for(auto& T : quadraticTerms)
    {
        if(T->coefficient == 0.0)
            continue;

        if(T->firstVariable == T->secondVariable) // variable squared
        {
            auto value = 2 * T->coefficient;
            auto element = hessian.insert(std::make_pair(std::make_pair(T->firstVariable, T->secondVariable), value));

            if(!element.second)
            {
                // Element already exists for the variable
                element.first->second += value;
            }
        }
        else
        {
            // Only save elements above the diagonal since the Hessian is symmetric
            if(T->firstVariable->index < T->secondVariable->index)
            {
                auto value = T->coefficient;
                auto element
                    = hessian.insert(std::make_pair(std::make_pair(T->firstVariable, T->secondVariable), value));

                if(!element.second)
                {
                    // Element already exists for the variable
                    element.first->second += value;
                }
            }
            else
            {
                auto value = T->coefficient;
                auto element
                    = hessian.insert(std::make_pair(std::make_pair(T->secondVariable, T->firstVariable), value));

                if(!element.second)
                {
                    // Element already exists for the variable
                    element.first->second += value;
                }
            }
        }
    }

    return hessian;
};

void QuadraticConstraint::initializeHessianSparsityPattern()
{
    LinearConstraint::initializeHessianSparsityPattern();

    for(auto& T : quadraticTerms)
    {
        if(T->coefficient == 0.0)
            continue;

        auto firstVariable
            = (T->firstVariable->index < T->secondVariable->index) ? T->firstVariable : T->secondVariable;
        auto secondVariable
            = (T->firstVariable->index < T->secondVariable->index) ? T->secondVariable : T->firstVariable;

        auto key = std::make_pair(firstVariable, secondVariable);

        if(std::find(hessianSparsityPattern->begin(), hessianSparsityPattern->end(), key)
            == hessianSparsityPattern->end())
            hessianSparsityPattern->push_back(key);
    }
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

    auto convexity = quadraticTerms.getConvexity();
    properties.convexity = Utilities::combineConvexity(convexity, properties.convexity);
};

void NonlinearConstraint::add(LinearTerms terms) { LinearConstraint::add(terms); };

void NonlinearConstraint::add(LinearTermPtr term) { LinearConstraint::add(term); };

void NonlinearConstraint::add(QuadraticTerms terms) { QuadraticConstraint::add(terms); };

void NonlinearConstraint::add(QuadraticTermPtr term) { QuadraticConstraint::add(term); };

void NonlinearConstraint::add(MonomialTerms terms)
{
    if(monomialTerms.size() == 0)
    {
        monomialTerms = terms;
        properties.hasMonomialTerms = true;
    }
    else
    {
        for(auto& T : terms)
        {
            add(T);
        }
    }
};

void NonlinearConstraint::add(MonomialTermPtr term)
{
    monomialTerms.push_back(term);
    properties.hasMonomialTerms = true;
};

void NonlinearConstraint::add(SignomialTerms terms)
{
    if(signomialTerms.size() == 0)
    {
        signomialTerms = terms;
        properties.hasSignomialTerms = true;
    }
    else
    {
        for(auto& T : terms)
        {
            add(T);
        }
    }
};

void NonlinearConstraint::add(SignomialTermPtr term)
{
    signomialTerms.push_back(term);
    properties.hasSignomialTerms = true;
};

void NonlinearConstraint::add(NonlinearExpressionPtr expression)
{
    if(nonlinearExpression)
    {
        NonlinearExpressions terms;
        terms.push_back(nonlinearExpression);
        terms.push_back(expression);
        nonlinearExpression = std::make_shared<ExpressionSum>(std::move(terms));
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

    if(this->properties.hasMonomialTerms)
        value += monomialTerms.calculate(point);

    if(this->properties.hasSignomialTerms)
        value += signomialTerms.calculate(point);

    if(this->properties.hasNonlinearExpression)
        value += nonlinearExpression->calculate(point);

    return value;
};

Interval NonlinearConstraint::calculateFunctionValue(const IntervalVector& intervalVector)
{
    Interval value = QuadraticConstraint::calculateFunctionValue(intervalVector);

    if(this->properties.hasMonomialTerms)
        value += monomialTerms.calculate(intervalVector);

    if(this->properties.hasSignomialTerms)
        value += signomialTerms.calculate(intervalVector);

    if(this->properties.hasNonlinearExpression)
        value += nonlinearExpression->calculate(intervalVector);

    return value;
};

SparseVariableVector NonlinearConstraint::calculateGradient(const VectorDouble& point, bool eraseZeroes = true)
{
    SparseVariableVector gradient = QuadraticConstraint::calculateGradient(point, eraseZeroes);

    try
    {
        for(auto& E : symbolicSparseJacobian)
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

            if(eraseZeroes && value[0] == 0.0)
                continue;

            auto element = gradient.insert(std::make_pair(E.first, value[0]));

            if(!element.second)
            {
                // Element already exists for the variable
                element.first->second += value[0];
            }
        }
    }
    catch(mc::FFGraph::Exceptions& e)
    {
        std::cout << "Error when evaluating gradient: " << e.what();
    }

    SparseVariableVector monomialGradient;

    if(this->properties.hasMonomialTerms)
    {
        monomialGradient = monomialTerms.calculateGradient(point);
    }

    SparseVariableVector signomialGradient;

    if(this->properties.hasSignomialTerms)
    {
        signomialGradient = signomialTerms.calculateGradient(point);
    }

    auto result = Utilities::combineSparseVariableVectors(gradient, monomialGradient, signomialGradient);

    if(eraseZeroes)
        Utilities::erase_if<VariablePtr, double>(result, 0.0);

    return result;
};

void NonlinearConstraint::initializeGradientSparsityPattern()
{
    QuadraticConstraint::initializeGradientSparsityPattern();

    if(this->properties.hasMonomialTerms)
    {
        for(auto& T : monomialTerms)
        {
            if(T->coefficient == 0.0)
                continue;

            for(auto& V : T->variables)
            {
                if(std::find(gradientSparsityPattern->begin(), gradientSparsityPattern->end(), V)
                    == gradientSparsityPattern->end())
                    gradientSparsityPattern->push_back(V);
            }
        }
    }

    if(this->properties.hasSignomialTerms)
    {
        for(auto& T : signomialTerms)
        {
            if(T->coefficient == 0.0)
                continue;

            for(auto& E : T->elements)
            {
                if(std::find(gradientSparsityPattern->begin(), gradientSparsityPattern->end(), E->variable)
                    == gradientSparsityPattern->end())
                    gradientSparsityPattern->push_back(E->variable);
            }
        }
    }

    for(auto& E : symbolicSparseJacobian)
    {
        if(std::find(gradientSparsityPattern->begin(), gradientSparsityPattern->end(), E.first)
            == gradientSparsityPattern->end())
            gradientSparsityPattern->push_back(E.first);
    }
};

SparseVariableMatrix NonlinearConstraint::calculateHessian(const VectorDouble& point, bool eraseZeroes = true)
{
    SparseVariableMatrix hessian = QuadraticConstraint::calculateHessian(point, eraseZeroes);

    try
    {
        for(auto& E : symbolicSparseHessian)
        {
            auto factorableFunction = std::get<1>(E);

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

                sharedOwnerProblem->factorableFunctionsDAG->eval(1, &factorableFunction, value,
                    sharedOwnerProblem->factorableFunctionVariables.size(),
                    &sharedOwnerProblem->factorableFunctionVariables[0], &newPoint[0]);
            }

            if(value[0] != value[0])
            {
                std::cout << "nan" << std::endl;
                value[0] = 0.0;
            }

            if(value[0] == 0.0)
                continue;

            if(E.first.first->index > E.first.second->index)
                // Hessian is symmetric, so discard elements below the diagonal
                continue;

            auto element = hessian.insert(std::make_pair(std::get<0>(E), value[0]));

            if(!element.second)
            {
                // Element already exists for the variable
                element.first->second += value[0];
            }
        }
    }
    catch(mc::FFGraph::Exceptions& e)
    {
        std::cout << "Error when evaluating hessian: " << e.what();
    }

    if(properties.hasMonomialTerms)
    {
        hessian = Utilities::combineSparseVariableMatrices(monomialTerms.calculateHessian(point), hessian);
    }

    if(properties.hasSignomialTerms)
    {
        hessian = Utilities::combineSparseVariableMatrices(signomialTerms.calculateHessian(point), hessian);
    }

    if(eraseZeroes)
        Utilities::erase_if<std::pair<VariablePtr, VariablePtr>, double>(hessian, 0.0);

    return (hessian);
};

void NonlinearConstraint::initializeHessianSparsityPattern()
{
    QuadraticConstraint::initializeHessianSparsityPattern();

    for(auto& T : this->monomialTerms)
    {
        if(T->coefficient == 0)
            continue;

        for(auto& V1 : T->variables)
        {
            for(auto& V2 : T->variables)
            {
                if(V1 == V2)
                    continue;

                std::pair<VariablePtr, VariablePtr> variablePair;

                if(V1->index < V2->index)
                    variablePair = std::make_pair(V1, V2);
                else
                {
                    variablePair = std::make_pair(V2, V1);
                }

                if(std::find(hessianSparsityPattern->begin(), hessianSparsityPattern->end(), variablePair)
                    == hessianSparsityPattern->end())
                    hessianSparsityPattern->push_back(variablePair);
            }
        }
    }

    for(auto& T : this->signomialTerms)
    {
        if(T->coefficient == 0)
            continue;

        for(auto& E1 : T->elements)
        {
            for(auto& E2 : T->elements)
            {
                if(E1 == E2)
                    continue;

                std::pair<VariablePtr, VariablePtr> variablePair;

                if(E1->variable->index < E2->variable->index)
                    variablePair = std::make_pair(E1->variable, E2->variable);
                else
                {
                    variablePair = std::make_pair(E2->variable, E1->variable);
                }

                if(std::find(hessianSparsityPattern->begin(), hessianSparsityPattern->end(), variablePair)
                    == hessianSparsityPattern->end())
                    hessianSparsityPattern->push_back(variablePair);
            }
        }
    }

    for(auto& E : symbolicSparseHessian)
    {
        if(std::find(hessianSparsityPattern->begin(), hessianSparsityPattern->end(), E.first)
            == hessianSparsityPattern->end())
            hessianSparsityPattern->push_back(E.first);
    }
};

bool NonlinearConstraint::isFulfilled(const VectorDouble& point) { return NumericConstraint::isFulfilled(point); };

void NonlinearConstraint::takeOwnership(ProblemPtr owner)
{
    QuadraticConstraint::takeOwnership(owner);
    monomialTerms.takeOwnership(owner);
    signomialTerms.takeOwnership(owner);
    if(nonlinearExpression != nullptr)
        nonlinearExpression->takeOwnership(owner);
};

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

    properties.classification = E_ConstraintClassification::Nonlinear;
    variablesInNonlinearExpression.clear();

    if(nonlinearExpression != nullptr)
    {
        properties.hasNonlinearExpression = true;

        nonlinearExpression->appendNonlinearVariables(variablesInNonlinearExpression);

        auto convexity = nonlinearExpression->getConvexity();
        properties.convexity = Utilities::combineConvexity(convexity, properties.convexity);
    }
    else
    {
        properties.hasNonlinearExpression = false;
    }

    if(monomialTerms.size() > 0)
    {
        properties.hasMonomialTerms = true;
        properties.classification = E_ConstraintClassification::Nonlinear;

        for(auto& T : monomialTerms)
        {
            for(auto& V : T->variables)
            {
                if(std::find(variablesInMonomialTerms.begin(), variablesInMonomialTerms.end(), V)
                    == variablesInMonomialTerms.end())
                    variablesInMonomialTerms.push_back(V);
            }
        }

        auto convexity = monomialTerms.getConvexity();
        properties.convexity = Utilities::combineConvexity(convexity, properties.convexity);
    }
    else
    {
        properties.hasMonomialTerms = false;
    }

    if(signomialTerms.size() > 0)
    {
        properties.hasSignomialTerms = true;
        properties.classification = E_ConstraintClassification::Nonlinear;

        for(auto& T : signomialTerms)
        {
            for(auto& E : T->elements)
            {
                if(std::find(variablesInSignomialTerms.begin(), variablesInSignomialTerms.end(), E->variable)
                    == variablesInSignomialTerms.end())
                    variablesInSignomialTerms.push_back(E->variable);
            }

            auto convexity = signomialTerms.getConvexity();
            properties.convexity = Utilities::combineConvexity(convexity, properties.convexity);
        }
    }
    else
    {
        properties.hasSignomialTerms = false;
    }

    std::sort(variablesInNonlinearExpression.begin(), variablesInNonlinearExpression.end(),
        [](const VariablePtr& variableOne, const VariablePtr& variableTwo) {
            return (variableOne->index < variableTwo->index);
        });
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

    if(monomialTerms.size() > 0)
        stream << " +" << monomialTerms;

    if(signomialTerms.size() > 0)
        stream << " +" << signomialTerms;

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