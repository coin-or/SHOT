/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "ObjectiveFunction.h"

#include "../Environment.h"
#include "Problem.h"
#include "../Settings.h"
#include "../Utilities.h"

namespace SHOT
{

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
}

Interval ObjectiveFunction::getBounds()
{
    IntervalVector variableBounds;

    if(auto sharedOwnerProblem = ownerProblem.lock())
    {
        variableBounds = sharedOwnerProblem->getVariableBounds();
    }

    auto interval = calculateValue(variableBounds);

    return (interval);
}

void ObjectiveFunction::initializeGradientSparsityPattern() { gradientSparsityPattern = std::make_shared<Variables>(); }

std::shared_ptr<Variables> ObjectiveFunction::getGradientSparsityPattern()
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

void ObjectiveFunction::initializeHessianSparsityPattern()
{
    hessianSparsityPattern = std::make_shared<std::vector<std::pair<VariablePtr, VariablePtr>>>();
}

std::shared_ptr<std::vector<std::pair<VariablePtr, VariablePtr>>> ObjectiveFunction::getHessianSparsityPattern()
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

static std::ostream& operator<<(std::ostream& stream, const ObjectiveFunction& objective)
{
    std::stringstream type;

    switch(objective.properties.classification)
    {
    case(E_ObjectiveFunctionClassification::Linear):
        type << "L";
        break;

    case(E_ObjectiveFunctionClassification::Quadratic):
        type << "Q";
        break;

    case(E_ObjectiveFunctionClassification::QuadraticConsideredAsNonlinear):
    case(E_ObjectiveFunctionClassification::Nonlinear):
        type << "NL";
        break;

    default:
        type << "?";
        break;
    }

    switch(objective.properties.convexity)
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

    if(objective.properties.hasLinearTerms)
        contains << "L";
    else
        contains << " ";

    if(objective.properties.hasQuadraticTerms)
        contains << "Q";
    else
        contains << " ";

    if(objective.properties.hasMonomialTerms)
        contains << "M";
    else
        contains << " ";

    if(objective.properties.hasSignomialTerms)
        contains << "S";
    else
        contains << " ";

    if(objective.properties.hasNonlinearExpression)
        contains << "E";
    else
        contains << " ";

    stream << fmt::format("[{:<12s}] [{:<5s}]\t", type.str(), contains.str());

    return objective.print(stream); // polymorphic print via reference
}

std::ostream& operator<<(std::ostream& stream, ObjectiveFunctionPtr objective)
{
    stream << *objective;
    return stream;
}

void LinearObjectiveFunction::add(LinearTerms terms)
{
    if(linearTerms.size() == 0)
    {
        linearTerms = terms;
        properties.isValid = false;
    }
    else
    {
        for(auto& T : terms)
        {
            add(T);
        }
    }
}

void LinearObjectiveFunction::add(LinearTermPtr term)
{
    linearTerms.push_back(term);
    properties.isValid = false;
}

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
    properties.convexity = E_Convexity::Linear;

    ObjectiveFunction::updateProperties();
}

bool LinearObjectiveFunction::isDualUnbounded()
{
    for(auto& T : linearTerms)
    {
        if(T->coefficient == 0)
            continue;

        if(T->variable->isDualUnbounded())
            return true;
    }

    return false;
}

void LinearObjectiveFunction::takeOwnership(ProblemPtr owner)
{
    ownerProblem = owner;
    linearTerms.takeOwnership(owner);
}

double LinearObjectiveFunction::calculateValue(const VectorDouble& point)
{
    double value = constant + linearTerms.calculate(point);
    return value;
}

Interval LinearObjectiveFunction::calculateValue(const IntervalVector& intervalVector)
{
    Interval value = linearTerms.calculate(intervalVector);
    return value;
}

SparseVariableVector LinearObjectiveFunction::calculateGradient(
    [[maybe_unused]] const VectorDouble& point, [[maybe_unused]] bool eraseZeroes = true)
{
    SparseVariableVector gradient;

    for(auto& T : linearTerms)
    {
        auto element = gradient.emplace(T->variable, T->coefficient);
        if(!element.second)
        {
            // Element already exists for the variable
            element.first->second += T->coefficient;
        }
    }

    if(eraseZeroes)
        Utilities::erase_if<VariablePtr, double>(gradient, 0.0);

    return gradient;
}

void LinearObjectiveFunction::initializeGradientSparsityPattern()
{
    ObjectiveFunction::initializeGradientSparsityPattern();

    for(auto& T : linearTerms)
    {
        if(T->coefficient == 0.0)
            continue;

        if(std::find(gradientSparsityPattern->begin(), gradientSparsityPattern->end(), T->variable)
            == gradientSparsityPattern->end())
            gradientSparsityPattern->push_back(T->variable);
    }
}

SparseVariableMatrix LinearObjectiveFunction::calculateHessian(
    [[maybe_unused]] const VectorDouble& point, [[maybe_unused]] bool eraseZeroes = true)
{
    SparseVariableMatrix hessian;

    return hessian;
}

void LinearObjectiveFunction::initializeHessianSparsityPattern()
{
    ObjectiveFunction::initializeHessianSparsityPattern();
}

std::ostream& LinearObjectiveFunction::print(std::ostream& stream) const
{
    if(constant != 0.0)
        stream << constant;

    if(properties.hasLinearTerms)
        stream << linearTerms;

    return stream;
}

[[maybe_unused]] static std::ostream& operator<<(std::ostream& stream, LinearObjectiveFunctionPtr objective)
{
    stream << *objective;
    return stream;
}

void QuadraticObjectiveFunction::add(QuadraticTerms terms)
{
    if(quadraticTerms.size() == 0)
    {
        quadraticTerms = terms;
        properties.isValid = false;
    }
    else
    {
        for(auto& T : terms)
        {
            add(T);
        }
    }
}

void QuadraticObjectiveFunction::add(QuadraticTermPtr term)
{
    quadraticTerms.push_back(term);
    properties.isValid = false;
}

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

        auto convexity = quadraticTerms.getConvexity();
        properties.convexity = Utilities::combineConvexity(convexity, properties.convexity);
    }
    else
    {
        properties.hasQuadraticTerms = false;
    }
}

bool QuadraticObjectiveFunction::isDualUnbounded()
{
    if(LinearObjectiveFunction::isDualUnbounded())
        return true;

    for(auto& T : quadraticTerms)
    {
        if(T->coefficient == 0)
            continue;

        if(T->firstVariable->isDualUnbounded())
            return true;

        if(T->secondVariable->isDualUnbounded())
            return true;
    }

    return false;
}

void QuadraticObjectiveFunction::takeOwnership(ProblemPtr owner)
{
    LinearObjectiveFunction::takeOwnership(owner);
    quadraticTerms.takeOwnership(owner);
}

double QuadraticObjectiveFunction::calculateValue(const VectorDouble& point)
{
    double value = LinearObjectiveFunction::calculateValue(point);
    value += quadraticTerms.calculate(point);
    return value;
}

Interval QuadraticObjectiveFunction::calculateValue(const IntervalVector& intervalVector)
{
    Interval value = LinearObjectiveFunction::calculateValue(intervalVector);
    value += quadraticTerms.calculate(intervalVector);
    return value;
}

SparseVariableVector QuadraticObjectiveFunction::calculateGradient(const VectorDouble& point, bool eraseZeroes = true)
{
    SparseVariableVector gradient = LinearObjectiveFunction::calculateGradient(point, eraseZeroes);

    for(auto& T : quadraticTerms)
    {
        if(T->firstVariable == T->secondVariable) // variable squared
        {
            auto value = 2 * T->coefficient * point[T->firstVariable->index];
            auto element = gradient.emplace(T->firstVariable, value);

            if(!element.second)
            {
                // Element already exists for the variable
                element.first->second += value;
            }
        }
        else
        {
            auto value = T->coefficient * point[T->secondVariable->index];
            auto element = gradient.emplace(T->firstVariable, value);

            if(!element.second)
            {
                // Element already exists for the variable
                element.first->second += value;
            }

            value = T->coefficient * point[T->firstVariable->index];
            element = gradient.emplace(T->secondVariable, value);

            if(!element.second)
            {
                // Element already exists for the variable
                element.first->second += value;
            }
        }
    }

    if(eraseZeroes)
        Utilities::erase_if<VariablePtr, double>(gradient, 0.0);

    return gradient;
}

void QuadraticObjectiveFunction::initializeGradientSparsityPattern()
{
    LinearObjectiveFunction::initializeGradientSparsityPattern();

    for(auto& T : quadraticTerms)
    {
        if(T->coefficient == 0.0)
            continue;

        if(std::find(gradientSparsityPattern->begin(), gradientSparsityPattern->end(), T->firstVariable)
            == gradientSparsityPattern->end())
            gradientSparsityPattern->push_back(T->firstVariable);

        if(std::find(gradientSparsityPattern->begin(), gradientSparsityPattern->end(), T->secondVariable)
            == gradientSparsityPattern->end())
            gradientSparsityPattern->push_back(T->secondVariable);
    }
}

SparseVariableMatrix QuadraticObjectiveFunction::calculateHessian(
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

void QuadraticObjectiveFunction::initializeHessianSparsityPattern()
{
    LinearObjectiveFunction::initializeHessianSparsityPattern();

    for(auto& T : quadraticTerms)
    {
        if(T->coefficient == 0.0)
            continue;

        auto firstVariable
            = (T->firstVariable->index < T->secondVariable->index) ? T->firstVariable : T->secondVariable;
        auto secondVariable
            = (T->firstVariable->index > T->secondVariable->index) ? T->secondVariable : T->firstVariable;

        auto key = std::make_pair(firstVariable, secondVariable);

        if(std::find(hessianSparsityPattern->begin(), hessianSparsityPattern->end(), key)
            == hessianSparsityPattern->end())
            hessianSparsityPattern->push_back(key);
    }
}

std::ostream& QuadraticObjectiveFunction::print(std::ostream& stream) const
{
    LinearObjectiveFunction::print(stream);

    if(properties.hasQuadraticTerms)
        stream << quadraticTerms;

    return stream;
}

[[maybe_unused]] static std::ostream& operator<<(std::ostream& stream, QuadraticObjectiveFunctionPtr objective)
{
    stream << *objective;
    return stream;
}

void NonlinearObjectiveFunction::add(MonomialTerms terms)
{
    if(monomialTerms.size() == 0)
    {
        monomialTerms = terms;
        properties.isValid = false;
    }
    else
    {
        for(auto& T : terms)
        {
            add(T);
        }
    }
}

void NonlinearObjectiveFunction::add(MonomialTermPtr term)
{
    monomialTerms.push_back(term);
    properties.isValid = false;
}

void NonlinearObjectiveFunction::add(SignomialTerms terms)
{
    if(signomialTerms.size() == 0)
    {
        signomialTerms = terms;
        properties.isValid = false;
    }
    else
    {
        for(auto& T : terms)
        {
            add(T);
        }
    }
}

void NonlinearObjectiveFunction::add(SignomialTermPtr term)
{
    signomialTerms.push_back(term);
    properties.isValid = false;
}

void NonlinearObjectiveFunction::add(NonlinearExpressionPtr expression)
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

    properties.isValid = false;
}

void NonlinearObjectiveFunction::updateFactorableFunction()
{
    factorableFunction = std::make_shared<FactorableFunction>(nonlinearExpression->getFactorableFunction());
}

void NonlinearObjectiveFunction::updateProperties()
{
    QuadraticObjectiveFunction::updateProperties();

    properties.classification = E_ObjectiveFunctionClassification::Nonlinear;
    variablesInNonlinearExpression.clear();

    if(nonlinearExpression != nullptr)
    {
        properties.hasNonlinearExpression = true;

        nonlinearExpression->appendNonlinearVariables(variablesInNonlinearExpression);

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
        properties.hasNonlinearExpression = false;
    }

    if(monomialTerms.size() > 0)
    {
        properties.hasMonomialTerms = true;
        properties.classification = E_ObjectiveFunctionClassification::Nonlinear;

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
        properties.classification = E_ObjectiveFunctionClassification::Nonlinear;

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

    variablesInNonlinearExpression.sortByIndex();
}

void NonlinearObjectiveFunction::takeOwnership(ProblemPtr owner)
{
    QuadraticObjectiveFunction::takeOwnership(owner);
    monomialTerms.takeOwnership(owner);
    signomialTerms.takeOwnership(owner);
    if(nonlinearExpression != nullptr)
        nonlinearExpression->takeOwnership(owner);
}

double NonlinearObjectiveFunction::calculateValue(const VectorDouble& point)
{
    double value = QuadraticObjectiveFunction::calculateValue(point);
    value += monomialTerms.calculate(point);
    value += signomialTerms.calculate(point);

    if(this->properties.hasNonlinearExpression)
        value += nonlinearExpression->calculate(point);

    return value;
}

Interval NonlinearObjectiveFunction::calculateValue(const IntervalVector& intervalVector)
{
    Interval value = QuadraticObjectiveFunction::calculateValue(intervalVector);
    value += monomialTerms.calculate(intervalVector);
    value += signomialTerms.calculate(intervalVector);

    try
    {
        if(this->properties.hasNonlinearExpression)
            value += nonlinearExpression->calculate(intervalVector);
    }
    catch(const mc::Interval::Exceptions&)
    {
        return (Interval(SHOT_DBL_MIN, SHOT_DBL_MAX));
    }

    return value;
}

SparseVariableVector NonlinearObjectiveFunction::calculateGradient(const VectorDouble& point, bool eraseZeroes = true)
{
    SparseVariableVector gradient = QuadraticObjectiveFunction::calculateGradient(point, eraseZeroes);

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
        Utilities::erase_if<VariablePtr, double>(gradient, 0.0);

    return result;
}

void NonlinearObjectiveFunction::initializeGradientSparsityPattern()
{
    QuadraticObjectiveFunction::initializeGradientSparsityPattern();

    bool debug = false;
    std::stringstream stream;
    std::stringstream filename;

    if(auto sharedOwnerProblem = ownerProblem.lock())
    {
        if(sharedOwnerProblem->env->settings->getSetting<bool>("Debug.Enable", "Output"))
        {
            debug = true;

            filename << sharedOwnerProblem->env->settings->getSetting<std::string>("Debug.Path", "Output");

            for(auto& V : *gradientSparsityPattern)
                stream << V->name << '\n';
        }
    }

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
                {
                    gradientSparsityPattern->push_back(V);

                    if(debug)
                        stream << "(monomial) " << V->name << '\n';
                }
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
                {
                    gradientSparsityPattern->push_back(E->variable);

                    if(debug)
                        stream << "(signomial) " << E->variable->name << '\n';
                }
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
                        {
                            gradientSparsityPattern->push_back(VAR);

                            if(debug)
                                stream << "(nonlinear expr) " << VAR->name << '\n';
                        }

                        continue;
                    }
                }
            }
        }
    }

    if(debug)
    {
        filename << "/sparsitypattern_jacobian_objective";

        if(properties.isReformulated)
            filename << "_ref";

        filename << ".txt";

        Utilities::writeStringToFile(filename.str(), stream.str());
    }

    nonlinearGradientSparsityMapGenerated = true;
}

SparseVariableMatrix NonlinearObjectiveFunction::calculateHessian(const VectorDouble& point, bool eraseZeroes = true)
{
    SparseVariableMatrix hessian = QuadraticObjectiveFunction::calculateHessian(point, eraseZeroes);

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

            CppAD::sparse_rcv<std::vector<size_t>, std::vector<double>> subset(nonlinearHessianSparsityPattern);

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

    return hessian;
}

void NonlinearObjectiveFunction::initializeHessianSparsityPattern()
{
    QuadraticObjectiveFunction::initializeHessianSparsityPattern();

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
                = std::vector<bool>(sharedOwnerProblem->properties.numberOfNonlinearExpressions, true);

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

std::ostream& NonlinearObjectiveFunction::print(std::ostream& stream) const
{
    QuadraticObjectiveFunction::print(stream);

    if(monomialTerms.size() > 0)
        stream << monomialTerms;

    if(signomialTerms.size() > 0)
        stream << signomialTerms;

    if(nonlinearExpression != nullptr)
        stream << " +(" << nonlinearExpression << ')';

    return stream;
}

std::ostream& operator<<(std::ostream& stream, NonlinearObjectiveFunctionPtr objective)
{
    stream << *objective;
    return stream;
}

} // namespace SHOT