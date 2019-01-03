/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Stefan Vigerske, GAMS Development Corp.
   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "../Shared.h"

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
    bool isCancelled = true;
    auto child = expression->child;

    while (child->getType() == E_NonlinearExpressionTypes::Negate)
    {
        child = std::dynamic_pointer_cast<ExpressionNegate>(child)->child;
        isCancelled = !isCancelled;
    }

    if (!isCancelled) // Negations cancel themselves out
    {
        return (std::dynamic_pointer_cast<NonlinearExpression>(simplify(child)));
    }
    else // One negation remains
    {
        expression->child = simplify(child);

        if (expression->child->getType() == E_NonlinearExpressionTypes::Sum)
        {
            auto sum = std::dynamic_pointer_cast<ExpressionSum>(expression->child);

            for (auto &T : sum->children.expressions)
            {
                if (T->getType() == E_NonlinearExpressionTypes::Negate)
                {
                    // Negations cancel out
                    T = std::dynamic_pointer_cast<ExpressionNegate>(T)->child;
                }
                else
                {
                    T = std::make_shared<ExpressionNegate>(T);
                }
            }

            return (std::dynamic_pointer_cast<NonlinearExpression>(sum));
        }

        return expression;
    }
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionInvert> expression)
{
    bool isCancelled = true;
    auto child = expression->child;

    while (child->getType() == E_NonlinearExpressionTypes::Invert)
    {
        child = std::dynamic_pointer_cast<ExpressionNegate>(child)->child;
        isCancelled = !isCancelled;
    }

    if (!isCancelled) // Invertions cancel themselves out
    {
        return (simplify(child));
    }
    else // One invertion remains
    {
        expression->child = simplify(child);
        return expression;
    }

    return (expression);
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionSquareRoot> expression)
{
    auto child = simplify(expression->child);

    if (child->getType() == E_NonlinearExpressionTypes::Square)
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

    if (child->getType() == E_NonlinearExpressionTypes::SquareRoot)
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

    if (child->getType() == E_NonlinearExpressionTypes::Log)
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

    if (child->getType() == E_NonlinearExpressionTypes::Exp)
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

    if (child->getType() == E_NonlinearExpressionTypes::ArcCos)
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

    if (child->getType() == E_NonlinearExpressionTypes::Cos)
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

    if (child->getType() == E_NonlinearExpressionTypes::ArcSin)
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

    if (child->getType() == E_NonlinearExpressionTypes::Sin)
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

    if (child->getType() == E_NonlinearExpressionTypes::ArcTan)
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

    if (child->getType() == E_NonlinearExpressionTypes::Tan)
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

    if (child->getType() == E_NonlinearExpressionTypes::Negate)
    {
        // Cancellation
        expression->child = std::dynamic_pointer_cast<ExpressionNegate>(child)->child;
    }

    expression->child = child;
    return expression;
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionPlus> expression)
{
    auto firstChild = simplify(expression->firstChild);
    auto secondChild = simplify(expression->secondChild);

    // Can asume that both children now are not of Plus or Minus type, but rather all sums are of Sum type

    NonlinearExpressions terms;
    double constant = 0.0;

    if (firstChild->getType() == E_NonlinearExpressionTypes::Sum)
    {
        for (auto &T : std::dynamic_pointer_cast<ExpressionSum>(firstChild)->children.expressions)
        {
            if (T->getType() == E_NonlinearExpressionTypes::Constant)
            {
                constant += std::dynamic_pointer_cast<ExpressionConstant>(T)->constant;
            }
            else
            {
                terms.add(T);
            }
        }
    }
    else
    {
        terms.add(firstChild);
    }

    if (secondChild->getType() == E_NonlinearExpressionTypes::Sum)
    {
        for (auto &T : std::dynamic_pointer_cast<ExpressionSum>(secondChild)->children.expressions)
        {
            if (T->getType() == E_NonlinearExpressionTypes::Constant)
            {
                constant += std::dynamic_pointer_cast<ExpressionConstant>(T)->constant;
            }
            else
            {
                terms.add(T);
            }
        }
    }
    else
    {
        terms.add(secondChild);
    }

    if (constant != 0.0)
        terms.add(std::make_shared<ExpressionConstant>(constant));

    return (std::make_shared<ExpressionSum>(terms));
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionMinus> expression)
{
    auto firstChild = simplify(expression->firstChild);
    auto secondChild = simplify(expression->secondChild);

    // Can asume that both children now are not of Plus or Minus type, but rather all sums are of Sum type

    NonlinearExpressions terms;
    double constant = 0.0;

    if (firstChild->getType() == E_NonlinearExpressionTypes::Sum)
    {
        for (auto &T : std::dynamic_pointer_cast<ExpressionSum>(firstChild)->children.expressions)
        {
            if (T->getType() == E_NonlinearExpressionTypes::Constant)
            {
                constant += std::dynamic_pointer_cast<ExpressionConstant>(T)->constant;
            }
            else
            {
                terms.add(T);
            }
        }
    }
    else
    {
        terms.add(firstChild);
    }

    if (secondChild->getType() == E_NonlinearExpressionTypes::Sum)
    {
        for (auto &T : std::dynamic_pointer_cast<ExpressionSum>(secondChild)->children.expressions)
        {
            if (T->getType() == E_NonlinearExpressionTypes::Constant)
            {
                constant -= std::dynamic_pointer_cast<ExpressionConstant>(T)->constant;
            }
            else
            {
                // Simplify is needed to propagate negation
                terms.add(simplify(std::make_shared<ExpressionNegate>(T)));
            }
        }
    }
    else
    {
        // Simplify is needed to propagate negation
        terms.add(simplify(std::make_shared<ExpressionNegate>(secondChild)));
    }

    if (constant != 0.0)
        terms.add(std::make_shared<ExpressionConstant>(constant));

    return (std::make_shared<ExpressionSum>(terms));
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionTimes> expression)
{
    auto firstChild = simplify(expression->firstChild);
    auto secondChild = simplify(expression->secondChild);

    auto tmp = secondChild->getType();

    if (firstChild->getType() == E_NonlinearExpressionTypes::Constant)
    {
        if (secondChild->getType() == E_NonlinearExpressionTypes::Constant)
        {
            double constant = std::dynamic_pointer_cast<ExpressionConstant>(firstChild)->constant * std::dynamic_pointer_cast<ExpressionConstant>(secondChild)->constant;
            return std::make_shared<ExpressionConstant>(constant);
        }
        else
        {
            auto product = std::make_shared<ExpressionProduct>();
            product->children.add(firstChild);
            product->children.add(secondChild);
            return product;
        }
    }
    else if (secondChild->getType() == E_NonlinearExpressionTypes::Constant)
    {
        auto product = std::make_shared<ExpressionProduct>();
        product->children.add(secondChild);
        product->children.add(firstChild);
        return product;
    }
    else
    {
        auto product = std::make_shared<ExpressionProduct>();
        product->children.add(firstChild);
        product->children.add(secondChild);
        return product;
    }
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionDivide> expression)
{
    auto firstChild = simplify(expression->firstChild);
    auto secondChild = simplify(expression->secondChild);

    bool firstChildIsConstant = false;
    bool secondChildIsConstant = false;
    double firstChildConstant = 1.0;
    double secondChildConstant = 1.0;

    if (firstChild->getType() == E_NonlinearExpressionTypes::Constant)
    {
        firstChildConstant = std::dynamic_pointer_cast<ExpressionConstant>(firstChild)->constant;
        firstChildIsConstant = true;
    }

    if (secondChild->getType() == E_NonlinearExpressionTypes::Constant)
    {
        secondChildConstant = std::dynamic_pointer_cast<ExpressionConstant>(secondChild)->constant;
        secondChildIsConstant = true;
    }

    if (firstChildIsConstant && secondChildIsConstant)
        return (std::make_shared<ExpressionConstant>(firstChildConstant / secondChildConstant));

    if (firstChildIsConstant && firstChildConstant == 1.0)
        return (std::make_shared<ExpressionInvert>(secondChild));

    if (secondChildIsConstant && secondChildConstant == 1.0)
        return (firstChild);

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

    if (firstChild->getType() == E_NonlinearExpressionTypes::Constant)
    {
        firstChildConstant = std::dynamic_pointer_cast<ExpressionConstant>(firstChild)->constant;
        firstChildIsConstant = true;
    }

    if (secondChild->getType() == E_NonlinearExpressionTypes::Constant)
    {
        secondChildConstant = std::dynamic_pointer_cast<ExpressionConstant>(secondChild)->constant;
        secondChildIsConstant = true;
    }

    if (firstChildIsConstant && secondChildIsConstant)
        return (std::make_shared<ExpressionConstant>(std::pow(firstChildConstant, secondChildConstant)));

    if (firstChildIsConstant && firstChildConstant == 1.0)
        return (std::make_shared<ExpressionConstant>(1.0));

    if (secondChildIsConstant)
    {
        if (secondChildConstant == 1.0)
            return (firstChild);
        else if (secondChildConstant == 2.0)
            return (std::make_shared<ExpressionSquare>(firstChild));
        else if (secondChildConstant == 0.5)
            return (std::make_shared<ExpressionSquareRoot>(firstChild));
        else if (secondChildConstant == -1.0)
            return (std::make_shared<ExpressionInvert>(firstChild));
    }

    return (std::make_shared<ExpressionPower>(firstChild, secondChild));
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionProduct> expression)
{
    double constant = 1.0;

    NonlinearExpressions children;

    for (auto &C : expression->children.expressions)
    {
        C = simplify(C);
        // Can now assume there are no Times types in the child

        if (C->getType() == E_NonlinearExpressionTypes::Constant)
        {
            constant *= std::dynamic_pointer_cast<ExpressionConstant>(C)->constant;

            if (constant == 0.0)
                return (std::make_shared<ExpressionConstant>(0.0));
        }
        else if (C->getType() == E_NonlinearExpressionTypes::Product)
        {
            for (auto &CC : std::dynamic_pointer_cast<ExpressionProduct>(C)->children.expressions)
            {
                if (CC->getType() == E_NonlinearExpressionTypes::Constant)
                {
                    constant *= std::dynamic_pointer_cast<ExpressionConstant>(CC)->constant;

                    if (constant == 0.0)
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

    if (constant != 1.0)
        product->children.add(std::make_shared<ExpressionConstant>(constant));

    for (auto &C : children.expressions)
    {
        product->children.add(C);
    }

    return (product);
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionSum> expression)
{
    double constant = 0.0;

    NonlinearExpressions children;

    for (auto &C : expression->children.expressions)
    {
        C = simplify(C);
        // Can now assume there are no Plus and Minus types in the child

        if (C->getType() == E_NonlinearExpressionTypes::Constant)
        {
            constant += std::dynamic_pointer_cast<ExpressionConstant>(C)->constant;
        }
        else if (C->getType() == E_NonlinearExpressionTypes::Sum)
        {
            for (auto &CC : std::dynamic_pointer_cast<ExpressionSum>(C)->children.expressions)
            {
                if (CC->getType() == E_NonlinearExpressionTypes::Constant)
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

    if (constant != 0.0)
        sum->children.add(std::make_shared<ExpressionConstant>(constant));

    for (auto &C : children.expressions)
    {
        sum->children.add(C);
    }

    return (sum);
}

inline NonlinearExpressionPtr simplify(NonlinearExpressionPtr expression)
{
    switch (expression->getType())
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
    case E_NonlinearExpressionTypes::Plus:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionPlus>(expression));
    case E_NonlinearExpressionTypes::Minus:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionMinus>(expression));
    case E_NonlinearExpressionTypes::Times:
        return simplifyExpression(std::dynamic_pointer_cast<ExpressionTimes>(expression));
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
} // namespace SHOT
