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
    bool isCancelled = false;
    auto child = expression->child;

    while (child->getType() == E_NonlinearExpressionTypes::Negate)
    {
        child = std::dynamic_pointer_cast<ExpressionNegate>(child)->child;
        isCancelled = !isCancelled;
    }

    if (isCancelled) // Negations cancel themselves out
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
                else if (T->getType() == E_NonlinearExpressionTypes::Constant)
                {
                    std::dynamic_pointer_cast<ExpressionConstant>(T)->constant *= -1;
                }
                else if (T->getType() == E_NonlinearExpressionTypes::Product)
                {
                    std::dynamic_pointer_cast<ExpressionProduct>(T)->children.add(std::make_shared<ExpressionConstant>(-1.0));
                    T = simplify(T);
                }
                else
                {
                    T = std::make_shared<ExpressionNegate>(T);
                }
            }

            return (std::dynamic_pointer_cast<NonlinearExpression>(sum));
        }
        else if (expression->child->getType() == E_NonlinearExpressionTypes::Constant)
        {
            std::dynamic_pointer_cast<ExpressionConstant>(expression->child)->constant *= -1;

            return (expression->child);
        }
        else if (expression->child->getType() == E_NonlinearExpressionTypes::Variable)
        {
            auto variable = std::dynamic_pointer_cast<ExpressionVariable>(expression->child)->variable;

            auto product = std::make_shared<ExpressionProduct>();

            product->children.add(std::make_shared<ExpressionConstant>(-1.0));
            product->children.add(std::make_shared<ExpressionVariable>(variable));

            return (product);
        }
        else if (expression->child->getType() == E_NonlinearExpressionTypes::Product)
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

    if (firstChild->getType() == E_NonlinearExpressionTypes::Constant && secondChild->getType() == E_NonlinearExpressionTypes::Constant)
    {
        double constant = std::dynamic_pointer_cast<ExpressionConstant>(firstChild)->constant * std::dynamic_pointer_cast<ExpressionConstant>(secondChild)->constant;
        return std::make_shared<ExpressionConstant>(constant);
    }

    auto product = std::make_shared<ExpressionProduct>();

    if (firstChild->getType() == E_NonlinearExpressionTypes::Product)
    {
        for (auto &C : std::dynamic_pointer_cast<ExpressionProduct>(firstChild)->children.expressions)
        {
            product->children.add(C);
        }
    }
    else
    {
        product->children.add(firstChild);
    }

    if (secondChild->getType() == E_NonlinearExpressionTypes::Product)
    {
        for (auto &C : std::dynamic_pointer_cast<ExpressionProduct>(secondChild)->children.expressions)
        {
            product->children.add(C);
        }
    }
    else
    {
        product->children.add(secondChild);
    }

    return simplify(product);
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

    if (firstChildIsConstant && firstChildConstant == 0.0)
        return (std::make_shared<ExpressionConstant>(0.0));

    if (secondChildIsConstant)
    {
        if (secondChildConstant == 0.0)
            return (std::make_shared<ExpressionConstant>(1.0));
        else if (secondChildConstant == 1.0)
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

    if (children.expressions.size() == 0) // Everything has been simplified away
        return (std::make_shared<ExpressionConstant>(0.0));

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

inline std::optional<LinearTermPtr> convertProductToLinearTerm(std::shared_ptr<ExpressionProduct> product)
{
    std::optional<LinearTermPtr> resultingLinearTerm;

    if (product->getNumberOfChildren() == 0)
        return resultingLinearTerm;

    if (!product->isLinearTerm())
        return resultingLinearTerm;

    // Now know we have a linear term

    if (product->getNumberOfChildren() == 1)
    {
        resultingLinearTerm = std::make_shared<LinearTerm>(1.0, std::dynamic_pointer_cast<ExpressionVariable>(product->children.expressions.at(0))->variable);
    }
    else if (product->children.expressions.at(0)->getType() == E_NonlinearExpressionTypes::Constant &&
             product->children.expressions.at(1)->getType() == E_NonlinearExpressionTypes::Variable)
    {
        auto variable = std::dynamic_pointer_cast<ExpressionVariable>(product->children.expressions.at(1))->variable;
        double constant = std::dynamic_pointer_cast<ExpressionConstant>(product->children.expressions.at(0))->constant;

        resultingLinearTerm = std::make_shared<LinearTerm>(constant, variable);
    }
    else if (product->children.expressions.at(0)->getType() == E_NonlinearExpressionTypes::Variable &&
             product->children.expressions.at(1)->getType() == E_NonlinearExpressionTypes::Constant)
    {
        auto variable = std::dynamic_pointer_cast<ExpressionVariable>(product->children.expressions.at(0))->variable;
        double constant = std::dynamic_pointer_cast<ExpressionConstant>(product->children.expressions.at(1))->constant;

        resultingLinearTerm = std::make_shared<LinearTerm>(constant, variable);
    }

    return resultingLinearTerm;
}

inline std::optional<QuadraticTermPtr> convertProductToQuadraticTerm(std::shared_ptr<ExpressionProduct> product)
{
    std::optional<QuadraticTermPtr> resultingQuadraticTerm;

    if (product->getNumberOfChildren() == 0)
        return resultingQuadraticTerm;

    if (!product->isQuadraticTerm())
        return resultingQuadraticTerm;

    // Now know we have a quadratic term

    double coefficient = 1.0;
    VariablePtr firstVariable;
    VariablePtr secondVariable;

    for (auto &C : product->children.expressions)
    {
        if (C->getType() == E_NonlinearExpressionTypes::Square)
        {
            auto square = std::dynamic_pointer_cast<ExpressionSquare>(C);

            firstVariable = std::dynamic_pointer_cast<ExpressionVariable>(square->child)->variable;
            secondVariable = firstVariable;
        }
        else if (C->getType() == E_NonlinearExpressionTypes::Variable)
        {
            if (!firstVariable)
                firstVariable = std::dynamic_pointer_cast<ExpressionVariable>(C)->variable;
            else
                secondVariable = std::dynamic_pointer_cast<ExpressionVariable>(C)->variable;
        }
        else if (C->getType() == E_NonlinearExpressionTypes::Constant)
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

inline std::optional<MonomialTermPtr> convertProductToMonomialTerm(std::shared_ptr<ExpressionProduct> product)
{
    std::optional<MonomialTermPtr> resultingMonomialTerm;

    if (product->getNumberOfChildren() == 0 || !product->isMonomialTerm())
    {
        return resultingMonomialTerm;
    }

    double coefficient = 1.0;
    Variables variables;

    for (auto &C : product->children.expressions)
    {
        if (C->getType() == E_NonlinearExpressionTypes::Variable)
        {
            variables.push_back(std::dynamic_pointer_cast<ExpressionVariable>(C)->variable);
        }
        else if (C->getType() == E_NonlinearExpressionTypes::Constant)
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

inline std::tuple<LinearTerms, QuadraticTerms, NonlinearExpressionPtr, double> extractTermsAndConstant(NonlinearExpressionPtr expression)
{
    double constant = 0.0;
    LinearTerms linearTerms;
    QuadraticTerms quadraticTerms;
    NonlinearExpressionPtr nonlinearExpression;

    //std::cout << "extract: " << *expression << '\n';

    if (expression->getType() == E_NonlinearExpressionTypes::Constant)
    {
        constant += std::dynamic_pointer_cast<ExpressionConstant>(expression)->constant;

        nonlinearExpression = std::make_shared<ExpressionConstant>(0.0);
    }
    else if (expression->getType() == E_NonlinearExpressionTypes::Variable)
    {
        auto variable = std::dynamic_pointer_cast<ExpressionVariable>(expression);
        linearTerms.add(std::make_shared<LinearTerm>(1.0, variable->variable));

        nonlinearExpression = std::make_shared<ExpressionConstant>(0.0);
    }
    else if (expression->getType() == E_NonlinearExpressionTypes::Product)
    {
        auto product = std::dynamic_pointer_cast<ExpressionProduct>(expression);

        if (product->isQuadraticTerm())
        {
            auto optional = convertProductToQuadraticTerm(product);

            if (optional)
            {
                quadraticTerms.add(optional.value());

                //std::cout << "product converted to quadratic term: " << optional.value() << std::endl;

                nonlinearExpression = std::make_shared<ExpressionConstant>(0.0);
            }
        }
        else if (product->isLinearTerm())
        {
            auto optional = convertProductToLinearTerm(product);

            if (optional)
            {
                linearTerms.add(optional.value());

                //std::cout << "product converted to linear term: " << optional.value() << std::endl;

                nonlinearExpression = std::make_shared<ExpressionConstant>(0.0);
            }
        }
        else
        {
            nonlinearExpression = expression;

            //std::cout << "no extraction from product: " << *nonlinearExpression << std::endl;
        }
    }
    else if (expression->getType() == E_NonlinearExpressionTypes::Sum)
    {
        for (auto &C : std::dynamic_pointer_cast<ExpressionSum>(expression)->children.expressions)
        {
            //std::cout << "extracting from term in sum: " << *C << std::endl;
            auto [tmpLinearTerms, tmpQuadraticTerms, tmpNonlinearExpression, tmpConstant] = extractTermsAndConstant(C);

            //if (tmpNonlinearExpression != nullptr)
            //    std::cout << "extracting results: " << *tmpNonlinearExpression << std::endl;
            linearTerms.add(tmpLinearTerms);
            quadraticTerms.add(tmpQuadraticTerms);
            constant += tmpConstant;

            if (tmpNonlinearExpression == nullptr) // the nonlinear expression has been fully extracted
            {
                C = std::make_shared<ExpressionConstant>(0.0);
            }
            else
            {
                C = tmpNonlinearExpression;
            }
        }

        nonlinearExpression = expression;
    }
    else
    {
        nonlinearExpression = expression;

        //std::cout << "no extraction from: " << *nonlinearExpression << std::endl;
    }

    //if (nonlinearExpression != nullptr)
    //    std::cout << "final extracting results nonlinear: " << *nonlinearExpression << std::endl;

    //std::cout << "number of terms: " << linearTerms.terms.size() << ", " << quadraticTerms.terms.size() << std::endl;

    return std::make_tuple(linearTerms, quadraticTerms, nonlinearExpression, constant);
}

void simplifyNonlinearExpressions(ProblemPtr problem);

} // namespace SHOT
