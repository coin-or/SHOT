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

#include "spdlog/fmt/fmt.h"

namespace SHOT
{

std::ostream& operator<<(std::ostream& stream, const Constraint& constraint)
{
    std::stringstream type;

    switch(constraint.properties.classification)
    {
    case(E_ConstraintClassification::Linear):
        type << "L";
        break;

    case(E_ConstraintClassification::Quadratic):
        type << "Q";
        break;

    case(E_ConstraintClassification::QuadraticConsideredAsNonlinear):
    case(E_ConstraintClassification::Nonlinear):
        type << "NL";
        break;

    default:
        type << "?";
        break;
    }

    switch(constraint.properties.convexity)
    {
    case(E_Convexity::Linear):
        type << "-convex";
        break;

    case(E_Convexity::Convex):
        type << "-convex";
        break;

    case(E_Convexity::Concave):
        type << "-concave";
        break;

    case(E_Convexity::Nonconvex):
        type << "-nonconvex";
        break;

    case(E_Convexity::Unknown):
        type << "-unknown";
        break;

    default:
        type << "-not set";
        break;
    }

    std::stringstream contains;

    if(constraint.properties.hasLinearTerms)
        contains << "L";
    else
        contains << " ";

    if(constraint.properties.hasQuadraticTerms)
        contains << "Q";
    else
        contains << " ";

    if(constraint.properties.hasMonomialTerms)
        contains << "M";
    else
        contains << " ";

    if(constraint.properties.hasSignomialTerms)
        contains << "S";
    else
        contains << " ";

    if(constraint.properties.hasNonlinearExpression)
        contains << "E";
    else
        contains << " ";

    stream << fmt::format(
        "[{:>5d},{:<12s}] [{:<5s}] {:>12s}:", constraint.index, type.str(), contains.str(), constraint.name);

    return constraint.print(stream); // polymorphic print via reference
}

std::ostream& operator<<(std::ostream& stream, ConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
}

void NumericConstraint::initializeGradientSparsityPattern() { gradientSparsityPattern = std::make_shared<Variables>(); }

std::shared_ptr<Variables> NumericConstraint::getGradientSparsityPattern()
{
    if(gradientSparsityPattern)
        return (gradientSparsityPattern);

    initializeGradientSparsityPattern();

    // Sorts the variables
    std::sort(gradientSparsityPattern->begin(), gradientSparsityPattern->end(),
        [](const VariablePtr& variableOne, const VariablePtr& variableTwo) {
            return (variableOne->index < variableTwo->index);
        });

    // Remove duplicates
    auto last = std::unique(gradientSparsityPattern->begin(), gradientSparsityPattern->end());
    gradientSparsityPattern->erase(last, gradientSparsityPattern->end());

    return (gradientSparsityPattern);
}

void NumericConstraint::initializeHessianSparsityPattern()
{
    hessianSparsityPattern = std::make_shared<std::vector<std::pair<VariablePtr, VariablePtr>>>();
}

std::shared_ptr<std::vector<std::pair<VariablePtr, VariablePtr>>> NumericConstraint::getHessianSparsityPattern()
{
    if(hessianSparsityPattern)
        return (hessianSparsityPattern);

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
}

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
}

void LinearConstraint::add(LinearTermPtr term)
{
    linearTerms.add(term);
    properties.hasLinearTerms = true;
}

double LinearConstraint::calculateFunctionValue(const VectorDouble& point)
{
    double value = linearTerms.calculate(point);
    value += constant;
    return value;
}

Interval LinearConstraint::calculateFunctionValue(const IntervalVector& intervalVector)
{
    Interval value = linearTerms.calculate(intervalVector);
    value += Interval(constant);
    return value;
}

Interval LinearConstraint::getConstraintFunctionBounds()
{
    Interval value = linearTerms.getBounds();
    value += Interval(constant);
    return value;
}

bool LinearConstraint::isFulfilled(const VectorDouble& point) { return NumericConstraint::isFulfilled(point); }

void LinearConstraint::takeOwnership(ProblemPtr owner)
{
    ownerProblem = owner;
    linearTerms.takeOwnership(owner);
}

SparseVariableVector LinearConstraint::calculateGradient(const VectorDouble& point, bool eraseZeroes = true)
{
    SparseVariableVector gradient = linearTerms.calculateGradient(point);

    if(eraseZeroes)
        Utilities::erase_if<VariablePtr, double>(gradient, 0.0);

    return gradient;
}

void LinearConstraint::initializeGradientSparsityPattern()
{
    NumericConstraint::initializeGradientSparsityPattern();

    for(auto& T : linearTerms)
    {
        if(T->coefficient == 0.0)
            continue;

        if(std::find(gradientSparsityPattern->begin(), gradientSparsityPattern->end(), T->variable)
            == gradientSparsityPattern->end())
            gradientSparsityPattern->push_back(T->variable);
    }
}

void LinearConstraint::initializeHessianSparsityPattern() { NumericConstraint::initializeHessianSparsityPattern(); }

SparseVariableMatrix LinearConstraint::calculateHessian(
    [[maybe_unused]] const VectorDouble& point, [[maybe_unused]] bool eraseZeroes = true)
{
    SparseVariableMatrix hessian;
    return hessian;
}

NumericConstraintValue LinearConstraint::calculateNumericValue(
    const VectorDouble& point, [[maybe_unused]] double correction)
{
    return NumericConstraint::calculateNumericValue(point);
}

std::shared_ptr<NumericConstraint> LinearConstraint::getPointer()
{
    return std::dynamic_pointer_cast<NumericConstraint>(shared_from_this());
}

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
    properties.monotonicity = linearTerms.getMonotonicity();
}

void QuadraticConstraint::add(LinearTerms terms) { LinearConstraint::add(terms); }

void QuadraticConstraint::add(LinearTermPtr term) { LinearConstraint::add(term); }

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
}

void QuadraticConstraint::add(QuadraticTermPtr term)
{
    quadraticTerms.push_back(term);
    properties.hasQuadraticTerms = true;
}

double QuadraticConstraint::calculateFunctionValue(const VectorDouble& point)
{
    double value = LinearConstraint::calculateFunctionValue(point);
    value += quadraticTerms.calculate(point);

    return value;
}

Interval QuadraticConstraint::calculateFunctionValue(const IntervalVector& intervalVector)
{
    Interval value = LinearConstraint::calculateFunctionValue(intervalVector);
    value += quadraticTerms.calculate(intervalVector);
    return value;
}

Interval QuadraticConstraint::getConstraintFunctionBounds()
{
    Interval value = LinearConstraint::getConstraintFunctionBounds();
    value += quadraticTerms.getBounds();
    return value;
}

bool QuadraticConstraint::isFulfilled(const VectorDouble& point) { return NumericConstraint::isFulfilled(point); }

void QuadraticConstraint::takeOwnership(ProblemPtr owner)
{
    LinearConstraint::takeOwnership(owner);
    quadraticTerms.takeOwnership(owner);
}

SparseVariableVector QuadraticConstraint::calculateGradient(const VectorDouble& point, bool eraseZeroes = true)
{
    SparseVariableVector linearGradient = LinearConstraint::calculateGradient(point, eraseZeroes);
    SparseVariableVector quadraticGradient = quadraticTerms.calculateGradient(point);

    return (Utilities::combineSparseVariableVectors(linearGradient, quadraticGradient));
}

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
}

SparseVariableMatrix QuadraticConstraint::calculateHessian(
    [[maybe_unused]] const VectorDouble& point, [[maybe_unused]] bool eraseZeroes = true)
{
    SparseVariableMatrix hessian;

    for(auto& T : quadraticTerms)
    {
        if(T->coefficient == 0.0)
            continue;

        if(T->firstVariable == T->secondVariable) // variable squared
        {
            auto value = 2 * T->coefficient;
            auto element = hessian.emplace(std::make_pair(T->firstVariable, T->secondVariable), value);

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
                auto element = hessian.emplace(std::make_pair(T->firstVariable, T->secondVariable), value);

                if(!element.second)
                {
                    // Element already exists for the variable
                    element.first->second += value;
                }
            }
            else
            {
                auto value = T->coefficient;
                auto element = hessian.emplace(std::make_pair(T->secondVariable, T->firstVariable), value);

                if(!element.second)
                {
                    // Element already exists for the variable
                    element.first->second += value;
                }
            }
        }
    }

    return hessian;
}

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
}

NumericConstraintValue QuadraticConstraint::calculateNumericValue(
    const VectorDouble& point, [[maybe_unused]] double correction)
{
    return NumericConstraint::calculateNumericValue(point);
}

std::shared_ptr<NumericConstraint> QuadraticConstraint::getPointer()
{
    return std::dynamic_pointer_cast<NumericConstraint>(shared_from_this());
}

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

    if(valueLHS != SHOT_DBL_MIN)
        properties.convexity = E_Convexity::Nonconvex;

    properties.monotonicity = Utilities::combineMonotonicity(properties.monotonicity, quadraticTerms.getMonotonicity());
}

void NonlinearConstraint::add(LinearTerms terms) { LinearConstraint::add(terms); }

void NonlinearConstraint::add(LinearTermPtr term) { LinearConstraint::add(term); }

void NonlinearConstraint::add(QuadraticTerms terms) { QuadraticConstraint::add(terms); }

void NonlinearConstraint::add(QuadraticTermPtr term) { QuadraticConstraint::add(term); }

void NonlinearConstraint::add(MonomialTerms terms)
{
    if(monomialTerms.size() == 0)
    {
        monomialTerms = terms;
    }
    else
    {
        for(auto& T : terms)
        {
            add(T);
        }
    }

    properties.hasMonomialTerms = true;
    properties.classification = E_ConstraintClassification::Nonlinear;
}

void NonlinearConstraint::add(MonomialTermPtr term)
{
    monomialTerms.push_back(term);
    properties.hasMonomialTerms = true;
    properties.classification = E_ConstraintClassification::Nonlinear;
}

void NonlinearConstraint::add(SignomialTerms terms)
{
    if(signomialTerms.size() == 0)
    {
        signomialTerms = terms;
    }
    else
    {
        for(auto& T : terms)
        {
            add(T);
        }
    }

    properties.hasSignomialTerms = true;
    properties.classification = E_ConstraintClassification::Nonlinear;
}

void NonlinearConstraint::add(SignomialTermPtr term)
{
    signomialTerms.push_back(term);
    properties.hasSignomialTerms = true;
    properties.classification = E_ConstraintClassification::Nonlinear;
}

void NonlinearConstraint::add(NonlinearExpressionPtr expression)
{
    if(nonlinearExpression)
    {
        NonlinearExpressions terms;

        if(nonlinearExpression->getType() == E_NonlinearExpressionTypes::Sum)
        {
            for(auto& TERM : std::dynamic_pointer_cast<ExpressionSum>(nonlinearExpression)->children)
                terms.add(TERM);
        }
        else
        {
            terms.push_back(nonlinearExpression);
        }

        if(expression->getType() == E_NonlinearExpressionTypes::Sum)
        {
            for(auto& TERM : std::dynamic_pointer_cast<ExpressionSum>(expression)->children)
                terms.add(TERM);
        }
        else
        {
            terms.push_back(expression);
        }

        nonlinearExpression = std::make_shared<ExpressionSum>(std::move(terms));
    }
    else
    {
        nonlinearExpression = expression;
    }

    properties.hasNonlinearExpression = true;
    properties.classification = E_ConstraintClassification::Nonlinear;
}

void NonlinearConstraint::updateFactorableFunction()
{
    factorableFunction = std::make_shared<FactorableFunction>(nonlinearExpression->getFactorableFunction());
}

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
}

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
}

Interval NonlinearConstraint::getConstraintFunctionBounds()
{
    Interval value = QuadraticConstraint::getConstraintFunctionBounds();

    if(this->properties.hasMonomialTerms)
        value += monomialTerms.getBounds();

    if(this->properties.hasSignomialTerms)
        value += signomialTerms.getBounds();

    try
    {
        if(this->properties.hasNonlinearExpression)
            value += nonlinearExpression->getBounds();
    }
    catch(const mc::Interval::Exceptions&)
    {
        return (Interval(SHOT_DBL_MIN, SHOT_DBL_MAX));
    }

    return value;
}

SparseVariableVector NonlinearConstraint::calculateGradient(const VectorDouble& point, bool eraseZeroes = true)
{
    SparseVariableVector gradient = QuadraticConstraint::calculateGradient(point, eraseZeroes);

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

    if(this->properties.hasNonlinearExpression)
    {
        if(!nonlinearGradientSparsityMapGenerated)
            initializeGradientSparsityPattern();

        if(auto sharedOwnerProblem = ownerProblem.lock())
        {
            int numberOfNonlinearVariables = sharedOwnerProblem->properties.numberOfVariablesInNonlinearExpressions;

            std::vector<double> pointNonlinearSubset(numberOfNonlinearVariables, 0.0);

            for(auto& VAR : sharedOwnerProblem->nonlinearExpressionVariables)
                pointNonlinearSubset[VAR->properties.nonlinearVariableIndex] = point[VAR->index];

            CppAD::sparse_rcv<std::vector<size_t>, std::vector<double>> subset(nonlinearGradientSparsityPattern);
            sharedOwnerProblem->ADFunctions.subgraph_jac_rev(pointNonlinearSubset, subset);

            const std::vector<size_t>& col(subset.col());
            const std::vector<double>& value(subset.val());

            std::vector<size_t> rowMajor = subset.row_major();

            for(auto k : rowMajor)
            {
                double coefficient = value[k];

                if(coefficient == 0.0)
                    continue;

                auto VAR = sharedOwnerProblem->nonlinearExpressionVariables[col[k]];

                auto element = gradient.emplace(VAR, coefficient);

                if(!element.second)
                {
                    // Element already exists for the variable
                    element.first->second += coefficient;
                }
            }
        }
    }

    auto result = Utilities::combineSparseVariableVectors(gradient, monomialGradient, signomialGradient);

    if(eraseZeroes)
        Utilities::erase_if<VariablePtr, double>(result, 0.0);

    return result;
}

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

    if(this->properties.hasNonlinearExpression)
    {
        if(auto sharedOwnerProblem = ownerProblem.lock())
        {
            assert(sharedOwnerProblem->properties.numberOfVariablesInNonlinearExpressions > 0);
            assert(sharedOwnerProblem->properties.numberOfNonlinearExpressions > 0);
            assert(this->nonlinearExpressionIndex >= 0);

            // For some reason we need to have all nonlinear variables activated, otherwise not all nonzero elements
            // of the gradient may be detected
            auto nonlinearVariablesInExpressionMap
                = std::vector<bool>(sharedOwnerProblem->properties.numberOfVariablesInNonlinearExpressions, true);

            auto nonlinearFunctionMap
                = std::vector<bool>(sharedOwnerProblem->properties.numberOfNonlinearExpressions, false);

            nonlinearFunctionMap[this->nonlinearExpressionIndex] = true;

            CppAD::sparse_rc<std::vector<size_t>> pattern;

            sharedOwnerProblem->ADFunctions.subgraph_sparsity(
                nonlinearVariablesInExpressionMap, nonlinearFunctionMap, false, pattern);

            // Save for later use when calculating gradients
            nonlinearGradientSparsityPattern = pattern;

            const std::vector<size_t>& variableIndices(nonlinearGradientSparsityPattern.col());

            for(size_t i = 0; i < nonlinearGradientSparsityPattern.nnz(); i++)
            {
                for(auto& VAR : variablesInNonlinearExpression)
                {
                    if((size_t)VAR->properties.nonlinearVariableIndex == variableIndices[i])
                    {
                        if(std::find(gradientSparsityPattern->begin(), gradientSparsityPattern->end(), VAR)
                            == gradientSparsityPattern->end())
                            gradientSparsityPattern->push_back(VAR);

                        continue;
                    }
                }
            }
        }
    }

    nonlinearGradientSparsityMapGenerated = true;
}

SparseVariableMatrix NonlinearConstraint::calculateHessian(const VectorDouble& point, bool eraseZeroes = true)
{
    SparseVariableMatrix hessian = QuadraticConstraint::calculateHessian(point, eraseZeroes);

    if(properties.hasMonomialTerms)
    {
        hessian = Utilities::combineSparseVariableMatrices(monomialTerms.calculateHessian(point), hessian);
    }

    if(properties.hasSignomialTerms)
    {
        hessian = Utilities::combineSparseVariableMatrices(signomialTerms.calculateHessian(point), hessian);
    }

    if(this->properties.hasNonlinearExpression)
    {
        if(!nonlinearHessianSparsityMapGenerated)
            initializeHessianSparsityPattern();

        if(auto sharedOwnerProblem = ownerProblem.lock())
        {
            int numberOfNonlinearVariables = sharedOwnerProblem->properties.numberOfVariablesInNonlinearExpressions;

            std::vector<double> pointNonlinearSubset(numberOfNonlinearVariables, 0.0);

            std::vector<double> weights(sharedOwnerProblem->properties.numberOfNonlinearExpressions, 0.0);
            weights[this->nonlinearExpressionIndex] = 1.0;

            for(auto& VAR : sharedOwnerProblem->nonlinearExpressionVariables)
                pointNonlinearSubset[VAR->properties.nonlinearVariableIndex] = point[VAR->index];

            // TODO: utilize sparsity pattern
            auto calculatedHessian = sharedOwnerProblem->ADFunctions.SparseHessian(pointNonlinearSubset, weights);

            for(auto& V1 : variablesInNonlinearExpression)
            {
                for(auto& V2 : variablesInNonlinearExpression)
                {
                    size_t hessianIndex = V1->properties.nonlinearVariableIndex * numberOfNonlinearVariables
                        + V2->properties.nonlinearVariableIndex;

                    double hessianValue = calculatedHessian[hessianIndex];

                    if(hessianValue == 0.0)
                        continue;

                    // Only save elements above the diagonal since the Hessian is symmetric
                    if(V1->index <= V2->index)
                    {
                        auto element = hessian.emplace(std::make_pair(V1, V2), hessianValue);

                        if(!element.second)
                        {
                            // Element already exists for the variable
                            element.first->second += hessianValue;
                        }
                    }
                }
            }
        }
    }

    if(eraseZeroes)
        Utilities::erase_if<std::pair<VariablePtr, VariablePtr>, double>(hessian, 0.0);

    return (hessian);
}

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

    if(this->properties.hasNonlinearExpression)
    {
        if(auto sharedOwnerProblem = ownerProblem.lock())
        {
            // For some reason we need to have all nonlinear variables activated, otherwise not all nonzero elements of
            // the hessian may be detected
            auto nonlinearVariablesInExpressionMap
                = std::vector<bool>(sharedOwnerProblem->properties.numberOfVariablesInNonlinearExpressions, true);

            auto nonlinearFunctionMap
                = std::vector<bool>(sharedOwnerProblem->properties.numberOfNonlinearExpressions, false);

            nonlinearFunctionMap[this->nonlinearExpressionIndex] = true;

            CppAD::sparse_rc<std::vector<size_t>> pattern;

            sharedOwnerProblem->ADFunctions.for_hes_sparsity(
                nonlinearVariablesInExpressionMap, nonlinearFunctionMap, false, pattern);

            nonlinearHessianSparsityPattern = pattern;

            const std::vector<size_t>& rowIndices(nonlinearHessianSparsityPattern.row());
            const std::vector<size_t>& colIndices(nonlinearHessianSparsityPattern.col());

            for(size_t i = 0; i < nonlinearHessianSparsityPattern.nnz(); i++)
            {
                for(auto& V1 : variablesInNonlinearExpression)
                {
                    for(auto& V2 : variablesInNonlinearExpression)
                    {
                        if((size_t)V1->properties.nonlinearVariableIndex == rowIndices[i]
                            && (size_t)V2->properties.nonlinearVariableIndex == colIndices[i])
                        {
                            std::pair<VariablePtr, VariablePtr> variablePair;

                            if(V1->index < V2->index)
                                variablePair = std::make_pair(V1, V2);
                            else
                                variablePair = std::make_pair(V2, V1);

                            if(std::find(hessianSparsityPattern->begin(), hessianSparsityPattern->end(), variablePair)
                                == hessianSparsityPattern->end())
                                hessianSparsityPattern->push_back(variablePair);

                            continue;
                        }
                    }
                }
            }
        }
    }

    nonlinearHessianSparsityMapGenerated = true;
}

bool NonlinearConstraint::isFulfilled(const VectorDouble& point) { return NumericConstraint::isFulfilled(point); }

void NonlinearConstraint::takeOwnership(ProblemPtr owner)
{
    QuadraticConstraint::takeOwnership(owner);
    monomialTerms.takeOwnership(owner);
    signomialTerms.takeOwnership(owner);
    if(nonlinearExpression != nullptr)
        nonlinearExpression->takeOwnership(owner);
}

NumericConstraintValue NonlinearConstraint::calculateNumericValue(
    const VectorDouble& point, [[maybe_unused]] double correction)
{
    return NumericConstraint::calculateNumericValue(point);
}

std::shared_ptr<NumericConstraint> NonlinearConstraint::getPointer()
{
    return std::dynamic_pointer_cast<NumericConstraint>(shared_from_this());
}

void NonlinearConstraint::updateProperties()
{
    QuadraticConstraint::updateProperties();

    properties.classification = E_ConstraintClassification::Nonlinear;

    variablesInNonlinearExpression.clear();

    if(nonlinearExpression != nullptr)
    {
        properties.hasNonlinearExpression = true;

        nonlinearExpression->appendNonlinearVariables(variablesInNonlinearExpression);

        assert(variablesInNonlinearExpression.size() > 0
            || nonlinearExpression->getType() == E_NonlinearExpressionTypes::Constant
            || nonlinearExpression->getBounds().l() == nonlinearExpression->getBounds().u());

        try
        {
            auto convexity = nonlinearExpression->getConvexity();
            properties.convexity = Utilities::combineConvexity(convexity, properties.convexity);
        }
        catch(const mc::Interval::Exceptions&)
        {
            properties.convexity = E_Convexity::Unknown;
        }
    }
    else
    {
        assert(variablesInNonlinearExpression.size() == 0);
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

    if(properties.hasMonomialTerms)
        properties.monotonicity
            = Utilities::combineMonotonicity(properties.monotonicity, monomialTerms.getMonotonicity());

    if(properties.hasSignomialTerms)
        properties.monotonicity
            = Utilities::combineMonotonicity(properties.monotonicity, signomialTerms.getMonotonicity());

    if(properties.hasNonlinearExpression)
        properties.monotonicity
            = Utilities::combineMonotonicity(properties.monotonicity, nonlinearExpression->getMonotonicity());

    std::sort(variablesInNonlinearExpression.begin(), variablesInNonlinearExpression.end(),
        [](const VariablePtr& variableOne, const VariablePtr& variableTwo) {
            return (variableOne->index < variableTwo->index);
        });
}

std::ostream& operator<<(std::ostream& stream, NumericConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
}

std::ostream& operator<<(std::ostream& stream, LinearConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
}

std::ostream& LinearConstraint::print(std::ostream& stream) const
{
    if(valueLHS > SHOT_DBL_MIN && valueLHS != valueRHS)
        stream << valueLHS << " <= ";

    if(linearTerms.size() > 0)
        stream << linearTerms;

    if(constant > 0)
        stream << " +" << constant;

    if(constant < 0)
        stream << ' ' << constant;

    if(valueLHS == valueRHS)
        stream << " = " << valueRHS;
    else if(valueRHS < SHOT_DBL_MAX)
        stream << " <= " << valueRHS;

    return stream;
}

std::ostream& operator<<(std::ostream& stream, QuadraticConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
}

std::ostream& QuadraticConstraint::print(std::ostream& stream) const
{
    if(valueLHS > SHOT_DBL_MIN && valueLHS != valueRHS)
        stream << valueLHS << " <= ";

    if(linearTerms.size() > 0)
        stream << linearTerms;

    if(quadraticTerms.size() > 0)
        stream << quadraticTerms;

    if(constant > 0)
        stream << " +" << constant;

    if(constant < 0)
        stream << ' ' << constant;

    if(valueLHS == valueRHS)
        stream << " = " << valueRHS;
    else if(valueRHS < SHOT_DBL_MAX)
        stream << " <= " << valueRHS;

    return stream;
}

std::ostream& operator<<(std::ostream& stream, NonlinearConstraintPtr constraint)
{
    stream << *constraint;
    return stream;
}

std::ostream& NonlinearConstraint::print(std::ostream& stream) const
{
    if(valueLHS > SHOT_DBL_MIN && valueLHS != valueRHS)
        stream << valueLHS << " <= ";

    if(linearTerms.size() > 0)
        stream << linearTerms;

    if(quadraticTerms.size() > 0)
        stream << quadraticTerms;

    if(monomialTerms.size() > 0)
        stream << monomialTerms;

    if(signomialTerms.size() > 0)
        stream << signomialTerms;

    if(nonlinearExpression != nullptr)
        stream << " +(" << nonlinearExpression << ')';

    if(constant > 0)
        stream << " +" << constant;

    if(constant < 0)
        stream << ' ' << constant;

    if(valueLHS == valueRHS)
        stream << " = " << valueRHS;
    else if(valueRHS < SHOT_DBL_MAX)
        stream << " <= " << valueRHS;

    return stream;
}
} // namespace SHOT