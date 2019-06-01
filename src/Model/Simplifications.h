/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Stefan Vigerske, GAMS Development Corp.
   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Environment.h"
#include "../Enums.h"
#include "../Structs.h"

#include "../Model/Variables.h"
#include "../Model/AuxiliaryVariables.h"
#include "../Model/Terms.h"
#include "../Model/NonlinearExpressions.h"
#include "../Model/ObjectiveFunction.h"
#include "../Model/Constraints.h"
#include "../Model/Problem.h"

#include <optional>

namespace SHOT
{

inline NonlinearExpressionPtr simplify(NonlinearExpressionPtr expression);

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionConstant> expression)
{
    return (expression);
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionVariable> expression)
{
    return (expression);
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionNegate> expression)
{
    bool isCancelled = false;
    auto child = expression->child;

    while(child->getType() == E_NonlinearExpressionTypes::Negate)
    {
        child = std::dynamic_pointer_cast<ExpressionNegate>(child)->child;
        isCancelled = !isCancelled;
    }

    if(isCancelled) // Negations cancel themselves out
    {
        return (std::dynamic_pointer_cast<NonlinearExpression>(simplify(child)));
    }
    else // One negation remains
    {
        expression->child = simplify(child);

        if(expression->child->getType() == E_NonlinearExpressionTypes::Sum)
        {
            auto sum = std::dynamic_pointer_cast<ExpressionSum>(expression->child);

            for(auto& T : sum->children)
            {
                if(T->getType() == E_NonlinearExpressionTypes::Negate)
                {
                    // Negations cancel out
                    T = std::dynamic_pointer_cast<ExpressionNegate>(T)->child;
                }
                else if(T->getType() == E_NonlinearExpressionTypes::Constant)
                {
                    std::dynamic_pointer_cast<ExpressionConstant>(T)->constant *= -1;
                }
                else if(T->getType() == E_NonlinearExpressionTypes::Product)
                {
                    std::dynamic_pointer_cast<ExpressionProduct>(T)->children.add(
                        std::make_shared<ExpressionConstant>(-1.0));
                    T = simplify(T);
                }
                else
                {
                    T = std::make_shared<ExpressionNegate>(T);
                }
            }

            return (std::dynamic_pointer_cast<NonlinearExpression>(sum));
        }
        else if(expression->child->getType() == E_NonlinearExpressionTypes::Constant)
        {
            std::dynamic_pointer_cast<ExpressionConstant>(expression->child)->constant *= -1;

            return (expression->child);
        }
        else if(expression->child->getType() == E_NonlinearExpressionTypes::Variable)
        {
            auto variable = std::dynamic_pointer_cast<ExpressionVariable>(expression->child)->variable;

            auto product = std::make_shared<ExpressionProduct>();

            product->children.add(std::make_shared<ExpressionConstant>(-1.0));
            product->children.add(std::make_shared<ExpressionVariable>(variable));

            return (product);
        }
        else if(expression->child->getType() == E_NonlinearExpressionTypes::Product)
        {
            auto product = std::dynamic_pointer_cast<ExpressionProduct>(expression->child);

            product->children.add(std::make_shared<ExpressionConstant>(-1.0));

            return (simplify(expression->child));
        }

        return expression;
    }
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionInvert> expression)
{
    bool isCancelled = true;
    auto child = expression->child;

    while(child->getType() == E_NonlinearExpressionTypes::Invert)
    {
        child = std::dynamic_pointer_cast<ExpressionNegate>(child)->child;
        isCancelled = !isCancelled;
    }

    if(!isCancelled) // Invertions cancel themselves out
    {
        return (simplify(child));
    }
    else // One invertion remains
    {
        expression->child = simplify(child);

        if(expression->child->getType() == E_NonlinearExpressionTypes::Power)
        {
            auto child = std::dynamic_pointer_cast<ExpressionPower>(expression->child);

            if(child->secondChild->getType() == E_NonlinearExpressionTypes::Constant)
            {
                auto power = std::dynamic_pointer_cast<ExpressionConstant>(child->secondChild);
                power->constant *= -1.0;

                return child;
            }
        }
    }

    return (expression);
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionSquareRoot> expression)
{
    auto child = simplify(expression->child);

    if(child->getType() == E_NonlinearExpressionTypes::Square)
    {
        // Cancellation
        return (std::dynamic_pointer_cast<ExpressionSquare>(child)->child);
    }

    expression->child = child;
    return expression;
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionSquare> expression)
{
    auto child = simplify(expression->child);

    if(child->getType() == E_NonlinearExpressionTypes::SquareRoot)
    {
        // Cancellation
        return (std::dynamic_pointer_cast<ExpressionSquareRoot>(child)->child);
    }

    expression->child = child;
    return expression;
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionExp> expression)
{
    auto child = simplify(expression->child);

    if(child->getType() == E_NonlinearExpressionTypes::Log)
    {
        // Cancellation
        return (std::dynamic_pointer_cast<ExpressionLog>(child)->child);
    }

    expression->child = child;
    return expression;
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionLog> expression)
{
    auto child = simplify(expression->child);

    if(child->getType() == E_NonlinearExpressionTypes::Exp)
    {
        // Cancellation
        return (std::dynamic_pointer_cast<ExpressionExp>(child)->child);
    }

    expression->child = child;
    return expression;
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionCos> expression)
{
    auto child = simplify(expression->child);

    if(child->getType() == E_NonlinearExpressionTypes::ArcCos)
    {
        // Cancellation
        return (std::dynamic_pointer_cast<ExpressionArcCos>(child)->child);
    }

    expression->child = child;
    return expression;
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionArcCos> expression)
{
    auto child = simplify(expression->child);

    if(child->getType() == E_NonlinearExpressionTypes::Cos)
    {
        // Cancellation
        return (std::dynamic_pointer_cast<ExpressionCos>(child)->child);
    }

    expression->child = child;
    return expression;
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionSin> expression)
{
    auto child = simplify(expression->child);

    if(child->getType() == E_NonlinearExpressionTypes::ArcSin)
    {
        // Cancellation
        return (std::dynamic_pointer_cast<ExpressionArcSin>(child)->child);
    }

    expression->child = child;
    return expression;
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionArcSin> expression)
{
    auto child = simplify(expression->child);

    if(child->getType() == E_NonlinearExpressionTypes::Sin)
    {
        // Cancellation
        return (std::dynamic_pointer_cast<ExpressionSin>(child)->child);
    }

    expression->child = child;
    return expression;
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionTan> expression)
{
    auto child = simplify(expression->child);

    if(child->getType() == E_NonlinearExpressionTypes::ArcTan)
    {
        // Cancellation
        return (std::dynamic_pointer_cast<ExpressionArcTan>(child)->child);
    }

    expression->child = child;
    return expression;
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionArcTan> expression)
{
    auto child = simplify(expression->child);

    if(child->getType() == E_NonlinearExpressionTypes::Tan)
    {
        // Cancellation
        return (std::dynamic_pointer_cast<ExpressionTan>(child)->child);
    }

    expression->child = child;
    return expression;
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionAbs> expression)
{
    auto child = simplify(expression->child);

    if(child->getType() == E_NonlinearExpressionTypes::Negate)
    {
        // Cancellation
        expression->child = std::dynamic_pointer_cast<ExpressionNegate>(child)->child;
    }

    expression->child = child;
    return expression;
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionDivide> expression)
{
    auto firstChild = simplify(expression->firstChild);
    auto secondChild = simplify(expression->secondChild);

    bool firstChildIsConstant = false;
    bool secondChildIsConstant = false;
    double firstChildConstant = 1.0;
    double secondChildConstant = 1.0;

    if(firstChild->getType() == E_NonlinearExpressionTypes::Constant)
    {
        firstChildConstant = std::dynamic_pointer_cast<ExpressionConstant>(firstChild)->constant;
        firstChildIsConstant = true;
    }

    if(secondChild->getType() == E_NonlinearExpressionTypes::Constant)
    {
        secondChildConstant = std::dynamic_pointer_cast<ExpressionConstant>(secondChild)->constant;
        secondChildIsConstant = true;
    }

    if(firstChildIsConstant && secondChildIsConstant)
        return (std::make_shared<ExpressionConstant>(firstChildConstant / secondChildConstant));

    if(firstChildIsConstant && firstChildConstant == 1.0)
        return (std::make_shared<ExpressionInvert>(secondChild));

    if(secondChildIsConstant && secondChildConstant == 1.0)
        return (firstChild);

    if(secondChild->getType() == E_NonlinearExpressionTypes::Power)
    {
        auto child = std::dynamic_pointer_cast<ExpressionPower>(secondChild);

        if(child->firstChild->getType() == E_NonlinearExpressionTypes::Variable
            && child->secondChild->getType() == E_NonlinearExpressionTypes::Constant)
        {
            auto power = std::dynamic_pointer_cast<ExpressionConstant>(child->secondChild);
            power->constant *= -1.0;

            std::make_shared<ExpressionProduct>(firstChild, secondChild);
        }
    }
    else if(secondChild->getType() == E_NonlinearExpressionTypes::Square)
    {
        auto square = std::dynamic_pointer_cast<ExpressionSquare>(secondChild);

        if(square->child->getType() == E_NonlinearExpressionTypes::Variable)
        {
            return (std::make_shared<ExpressionProduct>(firstChild,
                std::make_shared<ExpressionPower>(square->child, std::make_shared<ExpressionConstant>(-2.0))));
        }
    }

    return (std::make_shared<ExpressionDivide>(firstChild, secondChild));
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionPower> expression)
{
    auto firstChild = simplify(expression->firstChild);
    auto secondChild = simplify(expression->secondChild);

    bool firstChildIsConstant = false;
    bool secondChildIsConstant = false;
    double firstChildConstant = 1.0;
    double secondChildConstant = 1.0;

    if(firstChild->getType() == E_NonlinearExpressionTypes::Constant)
    {
        firstChildConstant = std::dynamic_pointer_cast<ExpressionConstant>(firstChild)->constant;
        firstChildIsConstant = true;
    }

    if(secondChild->getType() == E_NonlinearExpressionTypes::Constant)
    {
        secondChildConstant = std::dynamic_pointer_cast<ExpressionConstant>(secondChild)->constant;
        secondChildIsConstant = true;
    }

    if(firstChildIsConstant && secondChildIsConstant)
        return (std::make_shared<ExpressionConstant>(std::pow(firstChildConstant, secondChildConstant)));

    if(firstChildIsConstant && firstChildConstant == 1.0)
        return (std::make_shared<ExpressionConstant>(1.0));

    if(firstChildIsConstant && firstChildConstant == 0.0)
        return (std::make_shared<ExpressionConstant>(0.0));

    if(secondChildIsConstant)
    {
        if(secondChildConstant == 0.0)
            return (std::make_shared<ExpressionConstant>(1.0));
        else if(secondChildConstant == 1.0)
            return (firstChild);
        else if(secondChildConstant == 2.0)
            return (std::make_shared<ExpressionSquare>(firstChild));
        else if(secondChildConstant == 0.5)
            return (std::make_shared<ExpressionSquareRoot>(firstChild));
        else if(secondChildConstant == -1.0)
            return (std::make_shared<ExpressionInvert>(firstChild));
    }

    // Extract constants in if first child is product and has a constant as its first child.
    // Since the children have been simplified, we can assume that the constant (if it exists) is first.
    if(firstChild->getType() == E_NonlinearExpressionTypes::Product
        && secondChild->getType() == E_NonlinearExpressionTypes::Constant && firstChild->getNumberOfChildren() > 1)
    {
        auto product = std::dynamic_pointer_cast<ExpressionProduct>(firstChild);
        auto power = std::dynamic_pointer_cast<ExpressionConstant>(secondChild)->constant;

        double constant = std::dynamic_pointer_cast<ExpressionConstant>(product->children.at(0))->constant;

        NonlinearExpressions children;

        for(auto it = product->children.begin() + 1; it != product->children.end(); it++)
        {
            children.add(*it);
        }

        auto newProduct = std::make_shared<ExpressionProduct>();

        if(constant != 1.0)
            newProduct->children.add(std::make_shared<ExpressionConstant>(std::pow(constant, power)));

        newProduct->children.add(
            std::make_shared<ExpressionPower>(std::make_shared<ExpressionProduct>(children), secondChild));

        return (newProduct);
    }

    return (std::make_shared<ExpressionPower>(firstChild, secondChild));
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionProduct> expression)
{
    double constant = 1.0;

    NonlinearExpressions children;

    for(auto& C : expression->children)
    {
        C = simplify(C);
        // Can now assume there are no Times types in the child

        if(C->getType() == E_NonlinearExpressionTypes::Constant)
        {
            constant *= std::dynamic_pointer_cast<ExpressionConstant>(C)->constant;

            if(constant == 0.0)
                return (std::make_shared<ExpressionConstant>(0.0));
        }
        else if(C->getType() == E_NonlinearExpressionTypes::Product)
        {
            for(auto& CC : std::dynamic_pointer_cast<ExpressionProduct>(C)->children)
            {
                if(CC->getType() == E_NonlinearExpressionTypes::Constant)
                {
                    constant *= std::dynamic_pointer_cast<ExpressionConstant>(CC)->constant;

                    if(constant == 0.0)
                        return (std::make_shared<ExpressionConstant>(0.0));
                }
                else
                {
                    children.add(CC);
                }
            }
        }
        else
        {
            children.add(C);
        }
    }

    auto product = std::make_shared<ExpressionProduct>();

    if(constant != 1.0)
        product->children.add(std::make_shared<ExpressionConstant>(constant));

    for(auto& C : children)
    {
        product->children.add(C);
    }

    return (product);
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionSum> expression)
{
    double constant = 0.0;

    NonlinearExpressions children;

    for(auto& C : expression->children)
    {
        C = simplify(C);

        if(C->getType() == E_NonlinearExpressionTypes::Constant)
        {
            constant += std::dynamic_pointer_cast<ExpressionConstant>(C)->constant;
        }
        else if(C->getType() == E_NonlinearExpressionTypes::Sum)
        {
            for(auto& CC : std::dynamic_pointer_cast<ExpressionSum>(C)->children)
            {
                if(CC->getType() == E_NonlinearExpressionTypes::Constant)
                {
                    constant += std::dynamic_pointer_cast<ExpressionConstant>(CC)->constant;
                }
                else
                {
                    children.add(CC);
                }
            }
        }
        else
        {
            children.add(C);
        }
    }

    auto sum = std::make_shared<ExpressionSum>();

    if(constant != 0.0)
        sum->children.add(std::make_shared<ExpressionConstant>(constant));

    if(children.size() == 0) // Everything has been simplified away
        return (std::make_shared<ExpressionConstant>(0.0));

    for(auto& C : children)
    {
        sum->children.add(C);
    }

    return (sum);
}

inline NonlinearExpressionPtr simplify(NonlinearExpressionPtr expression)
{
    switch(expression->getType())
    {
    case E_NonlinearExpressionTypes::Constant:
    case E_NonlinearExpressionTypes::Variable:
        break;
    case E_NonlinearExpressionTypes::Negate:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionNegate>(expression));
    case E_NonlinearExpressionTypes::Invert:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionInvert>(expression));
    case E_NonlinearExpressionTypes::SquareRoot:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionSquareRoot>(expression));
    case E_NonlinearExpressionTypes::Square:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionSquare>(expression));
    case E_NonlinearExpressionTypes::Log:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionLog>(expression));
    case E_NonlinearExpressionTypes::Exp:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionExp>(expression));
    case E_NonlinearExpressionTypes::Cos:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionCos>(expression));
    case E_NonlinearExpressionTypes::ArcCos:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionArcCos>(expression));
    case E_NonlinearExpressionTypes::Sin:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionSin>(expression));
    case E_NonlinearExpressionTypes::ArcSin:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionArcSin>(expression));
    case E_NonlinearExpressionTypes::Tan:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionTan>(expression));
    case E_NonlinearExpressionTypes::ArcTan:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionArcTan>(expression));
    case E_NonlinearExpressionTypes::Abs:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionAbs>(expression));
    case E_NonlinearExpressionTypes::Divide:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionDivide>(expression));
    case E_NonlinearExpressionTypes::Power:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionPower>(expression));
    case E_NonlinearExpressionTypes::Sum:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionSum>(expression));
    case E_NonlinearExpressionTypes::Product:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionProduct>(expression));
    }

    return (expression);
}

inline std::optional<LinearTermPtr> convertProductToLinearTerm(std::shared_ptr<ExpressionProduct> product)
{
    std::optional<LinearTermPtr> resultingLinearTerm;

    if(product->getNumberOfChildren() == 0)
        return resultingLinearTerm;

    if(!product->isLinearTerm())
        return resultingLinearTerm;

    // Now know we have a linear term

    if(product->getNumberOfChildren() == 1)
    {
        resultingLinearTerm = std::make_shared<LinearTerm>(
            1.0, std::dynamic_pointer_cast<ExpressionVariable>(product->children.at(0))->variable);
    }
    else if(product->children.at(0)->getType() == E_NonlinearExpressionTypes::Constant
        && product->children.at(1)->getType() == E_NonlinearExpressionTypes::Variable)
    {
        auto variable = std::dynamic_pointer_cast<ExpressionVariable>(product->children.at(1))->variable;
        double constant = std::dynamic_pointer_cast<ExpressionConstant>(product->children.at(0))->constant;

        resultingLinearTerm = std::make_shared<LinearTerm>(constant, variable);
    }
    else if(product->children.at(0)->getType() == E_NonlinearExpressionTypes::Variable
        && product->children.at(1)->getType() == E_NonlinearExpressionTypes::Constant)
    {
        auto variable = std::dynamic_pointer_cast<ExpressionVariable>(product->children.at(0))->variable;
        double constant = std::dynamic_pointer_cast<ExpressionConstant>(product->children.at(1))->constant;

        resultingLinearTerm = std::make_shared<LinearTerm>(constant, variable);
    }

    return resultingLinearTerm;
}

inline std::optional<LinearTermPtr> convertPowerToLinearTerm(std::shared_ptr<ExpressionPower> power)
{
    std::optional<LinearTermPtr> resultingLinearTerm;

    if(power->getNumberOfChildren() == 0)
        return resultingLinearTerm;

    if(power->firstChild->getType() != E_NonlinearExpressionTypes::Variable
        || power->secondChild->getType() != E_NonlinearExpressionTypes::Constant)
        return resultingLinearTerm;

    double powerValue = std::dynamic_pointer_cast<ExpressionConstant>(power->secondChild)->constant;

    if(abs(powerValue - 1.0) > 1e-6)
        return resultingLinearTerm;

    // Now know we have a linear term

    resultingLinearTerm
        = std::make_shared<LinearTerm>(1.0, std::dynamic_pointer_cast<ExpressionVariable>(power->firstChild)->variable);

    return resultingLinearTerm;
}

inline std::optional<QuadraticTermPtr> convertProductToQuadraticTerm(std::shared_ptr<ExpressionProduct> product)
{
    std::optional<QuadraticTermPtr> resultingQuadraticTerm;

    if(product->getNumberOfChildren() == 0)
        return resultingQuadraticTerm;

    if(!product->isQuadraticTerm())
        return resultingQuadraticTerm;

    // Now know we have a quadratic term

    double coefficient = 1.0;
    VariablePtr firstVariable;
    VariablePtr secondVariable;

    for(auto& C : product->children)
    {
        if(C->getType() == E_NonlinearExpressionTypes::Square)
        {
            auto square = std::dynamic_pointer_cast<ExpressionSquare>(C);

            firstVariable = std::dynamic_pointer_cast<ExpressionVariable>(square->child)->variable;
            secondVariable = firstVariable;
        }
        else if(C->getType() == E_NonlinearExpressionTypes::Variable)
        {
            if(!firstVariable)
                firstVariable = std::dynamic_pointer_cast<ExpressionVariable>(C)->variable;
            else
                secondVariable = std::dynamic_pointer_cast<ExpressionVariable>(C)->variable;
        }
        else if(C->getType() == E_NonlinearExpressionTypes::Constant)
        {
            coefficient = coefficient * std::dynamic_pointer_cast<ExpressionConstant>(C)->constant;
        }
        else
        {
            return (resultingQuadraticTerm);
        }
    }

    resultingQuadraticTerm = std::make_shared<QuadraticTerm>(coefficient, firstVariable, secondVariable);

    return resultingQuadraticTerm;
}

inline std::optional<QuadraticTermPtr> convertSquareToQuadraticTerm(std::shared_ptr<ExpressionSquare> product)
{
    std::optional<QuadraticTermPtr> resultingQuadraticTerm;

    if(product->getNumberOfChildren() == 0)
        return resultingQuadraticTerm;

    if(product->child->getType() != E_NonlinearExpressionTypes::Variable)
        return resultingQuadraticTerm;

    // Now know we have a quadratic term

    double coefficient = 1.0;
    auto variable = std::dynamic_pointer_cast<ExpressionVariable>(product->child)->variable;

    resultingQuadraticTerm = std::make_shared<QuadraticTerm>(coefficient, variable, variable);

    return resultingQuadraticTerm;
}

inline std::optional<QuadraticTermPtr> convertPowerToQuadraticTerm(std::shared_ptr<ExpressionPower> power)
{
    std::optional<QuadraticTermPtr> resultingQuadraticTerm;

    if(power->getNumberOfChildren() == 0)
        return resultingQuadraticTerm;

    if(power->firstChild->getType() != E_NonlinearExpressionTypes::Variable
        || power->secondChild->getType() != E_NonlinearExpressionTypes::Constant)
        return resultingQuadraticTerm;

    double powerValue = std::dynamic_pointer_cast<ExpressionConstant>(power->secondChild)->constant;

    if(abs(powerValue - 2.0) > 1e-6)
        return resultingQuadraticTerm;

    // Now know we have a quadratic term

    double coefficient = 1.0;
    auto variable = std::dynamic_pointer_cast<ExpressionVariable>(power->firstChild)->variable;

    resultingQuadraticTerm = std::make_shared<QuadraticTerm>(coefficient, variable, variable);

    return resultingQuadraticTerm;
}

inline std::optional<MonomialTermPtr> convertProductToMonomialTerm(std::shared_ptr<ExpressionProduct> product)
{
    std::optional<MonomialTermPtr> resultingMonomialTerm;

    if(product->getNumberOfChildren() == 0 || !product->isMonomialTerm())
    {
        return resultingMonomialTerm;
    }

    double coefficient = 1.0;
    Variables variables;

    for(auto& C : product->children)
    {
        if(C->getType() == E_NonlinearExpressionTypes::Variable)
        {
            variables.push_back(std::dynamic_pointer_cast<ExpressionVariable>(C)->variable);
        }
        else if(C->getType() == E_NonlinearExpressionTypes::Constant)
        {
            coefficient = coefficient * std::dynamic_pointer_cast<ExpressionConstant>(C)->constant;
        }
        else
        {
            // Returns the unassigned optional argument since the term is not monomial
            return resultingMonomialTerm;
        }
    }

    resultingMonomialTerm = std::make_shared<MonomialTerm>(coefficient, variables);

    return resultingMonomialTerm;
}

inline std::optional<SignomialTermPtr> convertToSignomialTerm(NonlinearExpressionPtr expression);

inline std::optional<SignomialTermPtr> convertExpressionToSignomialTerm(std::shared_ptr<ExpressionConstant> expression)
{
    auto signomialTerm = std::make_shared<SignomialTerm>();
    signomialTerm->coefficient = expression->constant;

    std::optional<SignomialTermPtr> resultingSignomialTerm = signomialTerm;

    return resultingSignomialTerm;
}

inline std::optional<SignomialTermPtr> convertExpressionToSignomialTerm(std::shared_ptr<ExpressionVariable> expression)
{
    auto signomialTerm = std::make_shared<SignomialTerm>();
    signomialTerm->coefficient = 1.0;
    signomialTerm->elements.push_back(std::make_shared<SignomialElement>(expression->variable, 1.0));

    std::optional<SignomialTermPtr> resultingSignomialTerm = signomialTerm;

    return resultingSignomialTerm;
}

inline std::optional<SignomialTermPtr> convertExpressionToSignomialTerm(std::shared_ptr<ExpressionNegate> expression)
{
    std::optional<SignomialTermPtr> resultingSignomialTerm;

    if(expression->getNumberOfChildren() == 0)
        return resultingSignomialTerm;

    auto childSignomial = convertToSignomialTerm(expression->child);

    if(!childSignomial)
        return resultingSignomialTerm;

    childSignomial->get()->coefficient *= -1.0;

    return childSignomial;
}

inline std::optional<SignomialTermPtr> convertExpressionToSignomialTerm(std::shared_ptr<ExpressionInvert> expression)
{
    std::optional<SignomialTermPtr> resultingSignomialTerm;

    if(expression->getNumberOfChildren() == 0)
        return resultingSignomialTerm;

    auto childSignomial = convertToSignomialTerm(expression->child);

    if(!childSignomial)
        return resultingSignomialTerm;

    for(auto& E : childSignomial->get()->elements)
    {
        E->power *= (-1.0);
    }

    childSignomial->get()->coefficient = 1.0 / childSignomial->get()->coefficient;

    return childSignomial;
}

inline std::optional<SignomialTermPtr> convertExpressionToSignomialTerm(
    std::shared_ptr<ExpressionSquareRoot> expression)
{
    std::optional<SignomialTermPtr> resultingSignomialTerm;

    if(expression->getNumberOfChildren() == 0)
        return resultingSignomialTerm;

    auto childSignomial = convertToSignomialTerm(expression->child);

    if(!childSignomial)
        return resultingSignomialTerm;

    for(auto& E : childSignomial->get()->elements)
    {
        E->power *= 0.5;
    }

    childSignomial->get()->coefficient = sqrt(childSignomial->get()->coefficient);

    return childSignomial;
}

inline std::optional<SignomialTermPtr> convertExpressionToSignomialTerm(std::shared_ptr<ExpressionSquare> expression)
{
    std::optional<SignomialTermPtr> resultingSignomialTerm;

    if(expression->getNumberOfChildren() == 0)
        return resultingSignomialTerm;

    auto childSignomial = convertToSignomialTerm(expression->child);

    if(!childSignomial)
        return resultingSignomialTerm;

    for(auto& E : childSignomial->get()->elements)
    {
        E->power *= 2.0;
    }

    childSignomial->get()->coefficient = pow(childSignomial->get()->coefficient, 2.0);

    return childSignomial;
}

inline std::optional<SignomialTermPtr> convertExpressionToSignomialTerm(std::shared_ptr<ExpressionDivide> expression)
{
    std::optional<SignomialTermPtr> resultingSignomialTerm;

    if(expression->getNumberOfChildren() == 0)
        return resultingSignomialTerm;

    auto firstChildSignomial = convertToSignomialTerm(expression->firstChild);
    auto secondChildSignomial = convertToSignomialTerm(expression->secondChild);

    if(!firstChildSignomial || !secondChildSignomial)
        return resultingSignomialTerm;

    SignomialElements addedElements;

    for(auto& E2 : secondChildSignomial->get()->elements)
    {
        bool added = false;

        for(auto& E1 : firstChildSignomial->get()->elements)
        {
            if(E1->variable == E2->variable)
            {
                E1->power -= E2->power;
                added = true;
                continue;
            }
        }

        if(!added)
            addedElements.push_back(E2);
    }

    for(auto& E : addedElements)
    {
        E->power *= -1.0;
        firstChildSignomial->get()->elements.push_back(E);
    }

    firstChildSignomial->get()->coefficient /= secondChildSignomial->get()->coefficient;

    return firstChildSignomial;
}

inline std::optional<SignomialTermPtr> convertExpressionToSignomialTerm(std::shared_ptr<ExpressionPower> expression)
{
    std::optional<SignomialTermPtr> resultingSignomialTerm;

    if(expression->getNumberOfChildren() == 0)
        return resultingSignomialTerm;

    if(expression->secondChild->getType() != E_NonlinearExpressionTypes::Constant) // Not a signomial term
        return resultingSignomialTerm;

    auto childSignomial = convertToSignomialTerm(expression->firstChild);

    if(!childSignomial) // Not a signomial term
        return resultingSignomialTerm;

    double power = std::dynamic_pointer_cast<ExpressionConstant>(expression->secondChild)->constant;

    for(auto& E : childSignomial->get()->elements)
    {
        E->power *= power;
    }

    childSignomial->get()->coefficient = pow(childSignomial->get()->coefficient, power);

    return childSignomial;
}

inline std::optional<SignomialTermPtr> convertExpressionToSignomialTerm(std::shared_ptr<ExpressionProduct> product)
{
    std::optional<SignomialTermPtr> resultingSignomialTerm;

    if(product->getNumberOfChildren() == 0)
    {
        return resultingSignomialTerm;
    }

    auto signomialTerm = std::make_shared<SignomialTerm>();
    signomialTerm->coefficient = 1.0;

    for(auto& C : product->children)
    {
        auto childSignomial = convertToSignomialTerm(C);

        if(!childSignomial) // Not a signomial term
            return resultingSignomialTerm;

        signomialTerm->coefficient *= childSignomial->get()->coefficient;

        for(auto& E2 : childSignomial->get()->elements)
        {
            bool added = false;

            for(auto& E1 : signomialTerm->elements)
            {
                if(E1->variable == E2->variable)
                {
                    E1->power += E2->power;
                    added = true;
                    break;
                }
            }

            if(!added)
                signomialTerm->elements.push_back(E2);
        }
    }

    resultingSignomialTerm = signomialTerm;

    return resultingSignomialTerm;
}

inline std::optional<SignomialTermPtr> convertToSignomialTerm(NonlinearExpressionPtr expression)
{
    switch(expression->getType())
    {
    case E_NonlinearExpressionTypes::Constant:
        return convertExpressionToSignomialTerm(std::dynamic_pointer_cast<ExpressionConstant>(expression));
    case E_NonlinearExpressionTypes::Variable:
        return convertExpressionToSignomialTerm(std::dynamic_pointer_cast<ExpressionVariable>(expression));
    case E_NonlinearExpressionTypes::Negate:
        return convertExpressionToSignomialTerm(std::dynamic_pointer_cast<ExpressionNegate>(expression));
    case E_NonlinearExpressionTypes::Invert:
        return convertExpressionToSignomialTerm(std::dynamic_pointer_cast<ExpressionInvert>(expression));
    case E_NonlinearExpressionTypes::SquareRoot:
        return convertExpressionToSignomialTerm(std::dynamic_pointer_cast<ExpressionSquareRoot>(expression));
    case E_NonlinearExpressionTypes::Square:
        return convertExpressionToSignomialTerm(std::dynamic_pointer_cast<ExpressionSquare>(expression));
    case E_NonlinearExpressionTypes::Divide:
        return convertExpressionToSignomialTerm(std::dynamic_pointer_cast<ExpressionDivide>(expression));
    case E_NonlinearExpressionTypes::Power:
        return convertExpressionToSignomialTerm(std::dynamic_pointer_cast<ExpressionPower>(expression));
    case E_NonlinearExpressionTypes::Product:
        return convertExpressionToSignomialTerm(std::dynamic_pointer_cast<ExpressionProduct>(expression));
    default:
        break;
    }

    std::optional<SignomialTermPtr> resultingSignomialTerm;
    return (resultingSignomialTerm);
}

inline std::tuple<LinearTerms, QuadraticTerms, MonomialTerms, SignomialTerms, NonlinearExpressionPtr, double>
    extractTermsAndConstant(NonlinearExpressionPtr expression)
{
    double constant = 0.0;
    LinearTerms linearTerms;
    QuadraticTerms quadraticTerms;
    MonomialTerms monomialTerms;
    SignomialTerms signomialTerms;
    NonlinearExpressionPtr nonlinearExpression;

    if(expression->getType() == E_NonlinearExpressionTypes::Constant)
    {
        constant += std::dynamic_pointer_cast<ExpressionConstant>(expression)->constant;

        nonlinearExpression = std::make_shared<ExpressionConstant>(0.0);
    }
    else if(expression->getType() == E_NonlinearExpressionTypes::Variable)
    {
        auto variable = std::dynamic_pointer_cast<ExpressionVariable>(expression);
        linearTerms.add(std::make_shared<LinearTerm>(1.0, variable->variable));

        nonlinearExpression = std::make_shared<ExpressionConstant>(0.0);
    }
    else if(expression->getType() == E_NonlinearExpressionTypes::Square)
    {
        auto square = std::dynamic_pointer_cast<ExpressionSquare>(expression);

        if(auto optional = convertSquareToQuadraticTerm(square); optional)
        {
            quadraticTerms.add(optional.value());
        }
        else if(auto optional = convertExpressionToSignomialTerm(square); optional)
        {
            signomialTerms.add(optional.value());
        }
        else
        {
            nonlinearExpression = expression;
        }
    }
    else if(expression->getType() == E_NonlinearExpressionTypes::SquareRoot)
    {
        auto squareRoot = std::dynamic_pointer_cast<ExpressionSquareRoot>(expression);

        if(auto optional = convertExpressionToSignomialTerm(squareRoot); optional)
        {
            signomialTerms.add(optional.value());
        }
        else
        {
            nonlinearExpression = expression;
        }
    }
    else if(expression->getType() == E_NonlinearExpressionTypes::Invert)
    {
        auto inversion = std::dynamic_pointer_cast<ExpressionInvert>(expression);

        if(auto optional = convertExpressionToSignomialTerm(inversion); optional)
        {
            signomialTerms.add(optional.value());
        }
        else
        {
            nonlinearExpression = expression;
        }
    }
    else if(expression->getType() == E_NonlinearExpressionTypes::Negate)
    {
        auto negation = std::dynamic_pointer_cast<ExpressionNegate>(expression);

        if(auto optional = convertExpressionToSignomialTerm(negation); optional)
        {
            signomialTerms.add(optional.value());
        }
        else
        {
            nonlinearExpression = expression;
        }
    }
    else if(expression->getType() == E_NonlinearExpressionTypes::Divide)
    {
        auto division = std::dynamic_pointer_cast<ExpressionDivide>(expression);

        if(auto optional = convertExpressionToSignomialTerm(division); optional)
        {
            signomialTerms.add(optional.value());
        }
        else
        {
            nonlinearExpression = expression;
        }
    }
    else if(expression->getType() == E_NonlinearExpressionTypes::Power)
    {
        auto power = std::dynamic_pointer_cast<ExpressionPower>(expression);

        if(auto optional = convertPowerToLinearTerm(power); optional)
        {
            linearTerms.add(optional.value());
        }
        else if(auto optional = convertPowerToQuadraticTerm(power); optional)
        {
            quadraticTerms.add(optional.value());
        }
        else if(auto optional = convertExpressionToSignomialTerm(power); optional)
        {
            signomialTerms.add(optional.value());
        }
        else
        {
            nonlinearExpression = expression;
        }
    }
    else if(expression->getType() == E_NonlinearExpressionTypes::Product)
    {
        auto product = std::dynamic_pointer_cast<ExpressionProduct>(expression);

        if(auto optional = convertProductToLinearTerm(product); optional)
        {
            linearTerms.add(optional.value());
        }
        else if(auto optional = convertProductToQuadraticTerm(product); optional)
        {
            quadraticTerms.add(optional.value());
        }
        else if(auto optional = convertProductToMonomialTerm(product); optional)
        {
            monomialTerms.add(optional.value());
        }
        else if(auto optional = convertExpressionToSignomialTerm(product); optional)
        {
            signomialTerms.add(optional.value());
        }
        else
        {
            nonlinearExpression = expression;
        }
    }
    else if(expression->getType() == E_NonlinearExpressionTypes::Sum)
    {
        for(auto& C : std::dynamic_pointer_cast<ExpressionSum>(expression)->children)
        {
            auto [tmpLinearTerms, tmpQuadraticTerms, tmpMonomialTerms, tmpSignomialTerms, tmpNonlinearExpression,
                tmpConstant]
                = extractTermsAndConstant(C);

            linearTerms.add(tmpLinearTerms);
            quadraticTerms.add(tmpQuadraticTerms);
            monomialTerms.add(tmpMonomialTerms);
            signomialTerms.add(tmpSignomialTerms);
            constant += tmpConstant;

            if(tmpNonlinearExpression == nullptr) // the nonlinear expression has been fully extracted
            {
                C = std::make_shared<ExpressionConstant>(0.0);
            }
            else
            {
                C = tmpNonlinearExpression;
            }
        }

        bool areAllZero = true;

        for(auto& C : std::dynamic_pointer_cast<ExpressionSum>(expression)->children)
        {
            if(!(C->getType() == E_NonlinearExpressionTypes::Constant
                   && std::dynamic_pointer_cast<ExpressionConstant>(C)->constant == 0.0))
            {
                areAllZero = false;
                break;
            }
        }

        if(areAllZero)
            nonlinearExpression = std::make_shared<ExpressionConstant>(0.0);
        else
            nonlinearExpression = expression;
    }
    else
    {
        nonlinearExpression = expression;
    }

    if(nonlinearExpression != nullptr && nonlinearExpression->getType() == E_NonlinearExpressionTypes::Constant
        && std::dynamic_pointer_cast<ExpressionConstant>(nonlinearExpression)->constant == 0.0)
    {
        nonlinearExpression = nullptr;
    }

    return std::make_tuple(linearTerms, quadraticTerms, monomialTerms, signomialTerms, nonlinearExpression, constant);
}

void simplifyNonlinearExpressions(ProblemPtr problem);

NonlinearExpressionPtr copyNonlinearExpression(NonlinearExpression* expression, const ProblemPtr destination);
NonlinearExpressionPtr copyNonlinearExpression(NonlinearExpression* expression, Problem* destination);

} // namespace SHOT
