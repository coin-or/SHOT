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

NonlinearExpressionPtr copyNonlinearExpression(NonlinearExpression* expression, const ProblemPtr destination);
NonlinearExpressionPtr copyNonlinearExpression(NonlinearExpression* expression, Problem* destination = nullptr);

inline NonlinearExpressionPtr simplify(NonlinearExpressionPtr expression);

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionConstant> expression)
{
    return (expression);
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionVariable> expression)
{
    if(expression->variable->lowerBound == expression->variable->upperBound)
        return (std::make_shared<ExpressionConstant>(expression->variable->lowerBound));

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
        else if(child->getType() == E_NonlinearExpressionTypes::Constant)
        {
            std::dynamic_pointer_cast<ExpressionConstant>(child)->constant
                = 1.0 / std::dynamic_pointer_cast<ExpressionConstant>(child)->constant;

            return (child);
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

    if(child->getType() == E_NonlinearExpressionTypes::Product && child->getNumberOfChildren() == 2)
    {
        auto childrenProduct = std::dynamic_pointer_cast<ExpressionProduct>(child)->children;

        if(childrenProduct[0]->getType() == E_NonlinearExpressionTypes::Constant)
        {
            double coefficient = childrenProduct[0]->getBounds().l();

            if(coefficient != 1.0)
            {
                auto product = std::make_shared<ExpressionProduct>();
                product->children.add(std::make_shared<ExpressionConstant>(std::sqrt(coefficient)));
                product->children.add(std::make_shared<ExpressionSquareRoot>(childrenProduct[1]));
            }
            else
            {
                return (simplify(std::make_shared<ExpressionSquareRoot>(childrenProduct[1])));
            }
        }
        else if(childrenProduct[1]->getType() == E_NonlinearExpressionTypes::Constant)
        {
            double coefficient = childrenProduct[1]->getBounds().l();

            if(coefficient != 1.0)
            {
                auto product = std::make_shared<ExpressionProduct>();
                product->children.add(std::make_shared<ExpressionConstant>(std::sqrt(coefficient)));
                product->children.add(std::make_shared<ExpressionSquareRoot>(childrenProduct[0]));
            }
            else
            {
                return (simplify(std::make_shared<ExpressionSquareRoot>(childrenProduct[0])));
            }
        }
    }
    else if(child->getType() == E_NonlinearExpressionTypes::Constant)
    {
        std::dynamic_pointer_cast<ExpressionConstant>(child)->constant
            = std::sqrt(std::dynamic_pointer_cast<ExpressionConstant>(child)->constant);

        return (child);
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

    if(child->getType() == E_NonlinearExpressionTypes::Constant)
    {
        std::dynamic_pointer_cast<ExpressionConstant>(child)->constant
            *= std::dynamic_pointer_cast<ExpressionConstant>(child)->constant;
        return (child);
    }

    if(child->getType() == E_NonlinearExpressionTypes::Product)
    {
        auto childrenProduct = std::dynamic_pointer_cast<ExpressionProduct>(child)->children;

        auto product = std::make_shared<ExpressionProduct>();

        for(auto C : childrenProduct)
            product->children.add(simplify(std::make_shared<ExpressionSquare>(C)));

        return (product);
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

    if(child->getType() == E_NonlinearExpressionTypes::Constant)
    {
        std::dynamic_pointer_cast<ExpressionConstant>(child)->constant
            = std::exp(std::dynamic_pointer_cast<ExpressionConstant>(child)->constant);

        return (child);
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

    if(child->getType() == E_NonlinearExpressionTypes::Constant
        && std::dynamic_pointer_cast<ExpressionConstant>(child)->constant == 1.0)
    {
        return (std::make_shared<ExpressionConstant>(0));
    }

    if(child->getType() == E_NonlinearExpressionTypes::Constant)
    {
        std::dynamic_pointer_cast<ExpressionConstant>(child)->constant
            = std::log(std::dynamic_pointer_cast<ExpressionConstant>(child)->constant);

        return (child);
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

    if(child->getType() == E_NonlinearExpressionTypes::Constant)
    {
        std::dynamic_pointer_cast<ExpressionConstant>(child)->constant
            = std::cos(std::dynamic_pointer_cast<ExpressionConstant>(child)->constant);

        return (child);
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

    if(child->getType() == E_NonlinearExpressionTypes::Constant)
    {
        std::dynamic_pointer_cast<ExpressionConstant>(child)->constant
            = std::acos(std::dynamic_pointer_cast<ExpressionConstant>(child)->constant);

        return (child);
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

    if(child->getType() == E_NonlinearExpressionTypes::Constant)
    {
        std::dynamic_pointer_cast<ExpressionConstant>(child)->constant
            = std::sin(std::dynamic_pointer_cast<ExpressionConstant>(child)->constant);

        return (child);
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

    if(child->getType() == E_NonlinearExpressionTypes::Constant)
    {
        std::dynamic_pointer_cast<ExpressionConstant>(child)->constant
            = std::asin(std::dynamic_pointer_cast<ExpressionConstant>(child)->constant);

        return (child);
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

    if(child->getType() == E_NonlinearExpressionTypes::Constant)
    {
        std::dynamic_pointer_cast<ExpressionConstant>(child)->constant
            = std::tan(std::dynamic_pointer_cast<ExpressionConstant>(child)->constant);

        return (child);
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

    if(child->getType() == E_NonlinearExpressionTypes::Constant)
    {
        std::dynamic_pointer_cast<ExpressionConstant>(child)->constant
            = std::atan(std::dynamic_pointer_cast<ExpressionConstant>(child)->constant);

        return (child);
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

    if(child->getType() == E_NonlinearExpressionTypes::Constant)
    {
        std::dynamic_pointer_cast<ExpressionConstant>(child)->constant
            = std::abs(std::dynamic_pointer_cast<ExpressionConstant>(child)->constant);

        return (child);
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

            return (std::make_shared<ExpressionProduct>(firstChild, secondChild));
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
        {
            // Expand expression if of type (c+ d x)^2 or (c x+ d y)^2 to identify quadratics
            if(firstChild->getType() == E_NonlinearExpressionTypes::Sum && firstChild->getNumberOfChildren() == 2)
            {
                auto firstTerm = std::dynamic_pointer_cast<ExpressionSum>(firstChild)->children[0];
                auto secondTerm = std::dynamic_pointer_cast<ExpressionSum>(firstChild)->children[1];

                bool firstTermIsValid = false;
                bool secondTermIsValid = false;

                if(firstTerm->getType() == E_NonlinearExpressionTypes::Constant)
                {
                    firstTermIsValid = true;
                }
                else if(firstTerm->getType() == E_NonlinearExpressionTypes::Variable)
                {
                    firstTermIsValid = true;
                }
                else if(firstTerm->getType() == E_NonlinearExpressionTypes::Product
                    && firstTerm->getNumberOfChildren() == 2)
                {
                    auto firstFactor = std::dynamic_pointer_cast<ExpressionProduct>(firstTerm)->children[0];
                    auto secondFactor = std::dynamic_pointer_cast<ExpressionProduct>(firstTerm)->children[1];

                    if(firstFactor->getType() == E_NonlinearExpressionTypes::Constant
                        && secondFactor->getType() == E_NonlinearExpressionTypes::Variable)
                    {
                        firstTermIsValid = true;
                    }
                    else if(secondFactor->getType() == E_NonlinearExpressionTypes::Constant
                        && firstFactor->getType() == E_NonlinearExpressionTypes::Variable)
                    {
                        firstTermIsValid = true;
                    }
                }

                if(secondTerm->getType() == E_NonlinearExpressionTypes::Constant)
                {
                    secondTermIsValid = true;
                }
                else if(secondTerm->getType() == E_NonlinearExpressionTypes::Variable)
                {
                    secondTermIsValid = true;
                }
                else if(secondTerm->getType() == E_NonlinearExpressionTypes::Product
                    && secondTerm->getNumberOfChildren() == 2)
                {
                    auto firstFactor = std::dynamic_pointer_cast<ExpressionProduct>(secondTerm)->children[0];
                    auto secondFactor = std::dynamic_pointer_cast<ExpressionProduct>(secondTerm)->children[1];

                    if(firstFactor->getType() == E_NonlinearExpressionTypes::Constant
                        && secondFactor->getType() == E_NonlinearExpressionTypes::Variable)
                    {
                        secondTermIsValid = true;
                    }
                    else if(secondFactor->getType() == E_NonlinearExpressionTypes::Constant
                        && firstFactor->getType() == E_NonlinearExpressionTypes::Variable)
                    {
                        secondTermIsValid = true;
                    }
                }

                if(firstTermIsValid && secondTermIsValid)
                {
                    auto newTerm
                        = simplify(std::make_shared<ExpressionSum>(std::make_shared<ExpressionSquare>(firstTerm),
                            std::make_shared<ExpressionProduct>(std::make_shared<ExpressionConstant>(2.0),
                                copyNonlinearExpression(firstTerm.get()), copyNonlinearExpression(secondTerm.get())),
                            std::make_shared<ExpressionSquare>(secondTerm)));

                    return (newTerm);
                }
            }

            return (std::make_shared<ExpressionSquare>(firstChild));
        }
        else if(secondChildConstant == 0.5)
            return (std::make_shared<ExpressionSquareRoot>(firstChild));
        else if(secondChildConstant == -1.0)
            return (std::make_shared<ExpressionInvert>(firstChild));
        else if(firstChild->getType() == E_NonlinearExpressionTypes::Product && firstChild->getNumberOfChildren() > 1
            && std::dynamic_pointer_cast<ExpressionProduct>(firstChild)->children.at(0)->getType()
                == E_NonlinearExpressionTypes::Constant)
        {
            // Extract constants if first child is product and has a constant as its first child.
            // Since the children have been simplified, we can assume that the constant (if it exists) is first.

            auto product = std::dynamic_pointer_cast<ExpressionProduct>(firstChild);
            double constant = std::dynamic_pointer_cast<ExpressionConstant>(product->children.at(0))->constant;

            NonlinearExpressions children;

            for(auto it = product->children.begin() + 1; it != product->children.end(); it++)
            {
                children.add(*it);
            }

            auto newProduct = std::make_shared<ExpressionProduct>();

            if(constant != 1.0)
                newProduct->children.add(std::make_shared<ExpressionConstant>(std::pow(constant, secondChildConstant)));

            newProduct->children.add(
                std::make_shared<ExpressionPower>(std::make_shared<ExpressionProduct>(children), secondChild));

            return (newProduct);
        }
    }

    return (std::make_shared<ExpressionPower>(firstChild, secondChild));
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionSum> expression)
{
    if(expression->getNumberOfChildren() == 1)
        return (expression->children[0]);

    double constant = 0.0;

    NonlinearExpressions children;

    std::map<VariablePtr, double> linearVariableCoefficients;

    for(auto& C : expression->children)
    {
        if(C->getType() == E_NonlinearExpressionTypes::Sum)
        {
            for(auto& CC : std::dynamic_pointer_cast<ExpressionSum>(C)->children)
            {
                if(CC->getType() == E_NonlinearExpressionTypes::Constant)
                {
                    constant += std::dynamic_pointer_cast<ExpressionConstant>(CC)->constant;
                }
                else
                {
                    children.add(simplify(CC));
                }
            }

            continue;
        }

        C = simplify(C);

        if(C->getType() == E_NonlinearExpressionTypes::Constant)
        {
            constant += std::dynamic_pointer_cast<ExpressionConstant>(C)->constant;
        }
        else if(C->getType() == E_NonlinearExpressionTypes::Variable)
        {
            auto variable = std::dynamic_pointer_cast<ExpressionVariable>(C)->variable;

            if(auto it { linearVariableCoefficients.find(variable) }; it != std::end(linearVariableCoefficients))
            {
                (*it).second += 1.0;
            }
            else
            {
                linearVariableCoefficients.insert({ variable, 1.0 });
            }
        }
        else if(C->getType() == E_NonlinearExpressionTypes::Product
            && std::dynamic_pointer_cast<ExpressionProduct>(C)->isLinearTerm())
        {
            if(auto linearTerm = std::dynamic_pointer_cast<ExpressionProduct>(C)->getLinearTerm(); linearTerm)
            {
                double coefficient = std::get<0>(*linearTerm);
                VariablePtr variable = std::get<1>(*linearTerm);

                if(auto it { linearVariableCoefficients.find(variable) }; it != std::end(linearVariableCoefficients))
                {
                    (*it).second += coefficient;
                }
                else
                {
                    linearVariableCoefficients.insert({ variable, coefficient });
                }
            }
            else
            {
                children.add(C);
            }
        }
        else
        {
            children.add(C);
        }
    }

    if(children.size() == 0 && linearVariableCoefficients.size() == 0) // Everything has been simplified away
        return (std::make_shared<ExpressionConstant>(constant));

    auto sum = std::make_shared<ExpressionSum>(children);

    if(constant != 0.0)
        sum->children.add(std::make_shared<ExpressionConstant>(constant));

    for(auto& P : linearVariableCoefficients)
    {
        if(P.second == 1.0)
            sum->children.add(std::make_shared<ExpressionVariable>(P.first));
        else if(P.second != 0.0)
            sum->children.add(std::make_shared<ExpressionProduct>(
                std::make_shared<ExpressionConstant>(P.second), std::make_shared<ExpressionVariable>(P.first)));
    }

    if(sum->getNumberOfChildren() == 0)
        return (std::make_shared<ExpressionConstant>(0.0));

    return (sum);
}

inline NonlinearExpressionPtr simplifyExpression(std::shared_ptr<ExpressionProduct> expression)
{
    if(expression->getNumberOfChildren() == 1)
        return (expression->children[0]);

    double constant = 1.0;

    NonlinearExpressions children;
    NonlinearExpressions unaddedChildren;

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
        else if(C->getType() == E_NonlinearExpressionTypes::Sum)
        {
            unaddedChildren.add(C);
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

    if(unaddedChildren.size() == 1)
    {
        auto sum = std::make_shared<ExpressionSum>();

        for(auto& T : std::dynamic_pointer_cast<ExpressionSum>(unaddedChildren[0])->children)
        {
            auto newProduct = std::make_shared<ExpressionProduct>();

            if(constant != 1.0)
                newProduct->children.add(std::make_shared<ExpressionConstant>(constant));

            for(auto& C : children)
            {
                newProduct->children.add(copyNonlinearExpression(C.get()));
            }

            newProduct->children.add(copyNonlinearExpression(T.get()));

            sum->children.add(newProduct);
        }

        return (simplifyExpression(sum));
    }

    if(children.size() == 0 && unaddedChildren.size() == 0) // Everything has been simplified away
        return (std::make_shared<ExpressionConstant>(constant));

    auto product = std::make_shared<ExpressionProduct>();

    if(constant != 1.0)
        product->children.add(std::make_shared<ExpressionConstant>(constant));

    for(auto& C : children)
    {
        product->children.add(C);
    }

    for(auto& C : unaddedChildren)
    {
        product->children.add(C);
    }

    return (product);
}

inline NonlinearExpressionPtr simplify(NonlinearExpressionPtr expression)
{
    std::stringstream ss;
    auto type = expression->getType();

    /*switch(type)
    {
    case E_NonlinearExpressionTypes::Constant:
        ss << "\nBefore simplification of constant: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Variable:
        ss << "\nBefore simplification of variable: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Negate:
        ss << "\nBefore simplification of negate: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Invert:
        ss << "\nBefore simplification of invert: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::SquareRoot:
        ss << "\nBefore simplification of square root: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Square:
        ss << "\nBefore simplification of square: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Log:
        ss << "\nBefore simplification of log: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Exp:
        ss << "\nBefore simplification of exp: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Cos:
        ss << "\nBefore simplification of cos: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::ArcCos:
        ss << "\nBefore simplification of arccos: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Sin:
        ss << "\nBefore simplification of sin: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::ArcSin:
        ss << "\nBefore simplification of arcsin: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Tan:
        ss << "\nBefore simplification of tan: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::ArcTan:
        ss << "\nBefore simplification of arctan: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Abs:
        ss << "\nBefore simplification of abs: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Divide:
        ss << "\nBefore simplification of divide: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Power:
        ss << "\nBefore simplification of power: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Sum:
        ss << "\nBefore simplification of sum: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Product:
        ss << "\nBefore simplification of product: " << *expression << std::endl;
        break;
    default:
        assert(false);
    }*/

    switch(type)
    {
    case E_NonlinearExpressionTypes::Constant:
        break;
    case E_NonlinearExpressionTypes::Variable:
        expression = simplifyExpression(std::dynamic_pointer_cast<ExpressionVariable>(expression));
        break;
    case E_NonlinearExpressionTypes::Negate:
        expression = simplifyExpression(std::dynamic_pointer_cast<ExpressionNegate>(expression));
        break;
    case E_NonlinearExpressionTypes::Invert:
        expression = simplifyExpression(std::dynamic_pointer_cast<ExpressionInvert>(expression));
        break;
    case E_NonlinearExpressionTypes::SquareRoot:
        expression = simplifyExpression(std::dynamic_pointer_cast<ExpressionSquareRoot>(expression));
        break;
    case E_NonlinearExpressionTypes::Square:
        expression = simplifyExpression(std::dynamic_pointer_cast<ExpressionSquare>(expression));
        break;
    case E_NonlinearExpressionTypes::Log:
        expression = simplifyExpression(std::dynamic_pointer_cast<ExpressionLog>(expression));
        break;
    case E_NonlinearExpressionTypes::Exp:
        expression = simplifyExpression(std::dynamic_pointer_cast<ExpressionExp>(expression));
        break;
    case E_NonlinearExpressionTypes::Cos:
        expression = simplifyExpression(std::dynamic_pointer_cast<ExpressionCos>(expression));
        break;
    case E_NonlinearExpressionTypes::ArcCos:
        expression = simplifyExpression(std::dynamic_pointer_cast<ExpressionArcCos>(expression));
        break;
    case E_NonlinearExpressionTypes::Sin:
        expression = simplifyExpression(std::dynamic_pointer_cast<ExpressionSin>(expression));
        break;
    case E_NonlinearExpressionTypes::ArcSin:
        expression = simplifyExpression(std::dynamic_pointer_cast<ExpressionArcSin>(expression));
        break;
    case E_NonlinearExpressionTypes::Tan:
        expression = simplifyExpression(std::dynamic_pointer_cast<ExpressionTan>(expression));
        break;
    case E_NonlinearExpressionTypes::ArcTan:
        expression = simplifyExpression(std::dynamic_pointer_cast<ExpressionArcTan>(expression));
        break;
    case E_NonlinearExpressionTypes::Abs:
        expression = simplifyExpression(std::dynamic_pointer_cast<ExpressionAbs>(expression));
        break;
    case E_NonlinearExpressionTypes::Divide:
        expression = simplifyExpression(std::dynamic_pointer_cast<ExpressionDivide>(expression));
        break;
    case E_NonlinearExpressionTypes::Power:
        expression = simplifyExpression(std::dynamic_pointer_cast<ExpressionPower>(expression));
        break;
    case E_NonlinearExpressionTypes::Sum:
        expression = simplifyExpression(std::dynamic_pointer_cast<ExpressionSum>(expression));
        break;
    case E_NonlinearExpressionTypes::Product:
        expression = simplifyExpression(std::dynamic_pointer_cast<ExpressionProduct>(expression));
        break;
    default:
        assert(false);
    }

    /*switch(type)
    {
    case E_NonlinearExpressionTypes::Constant:
        ss << " After simplification of constant: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Variable:
        ss << " After simplification of variable: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Negate:
        ss << " After simplification of negate: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Invert:
        ss << " After simplification of invert: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::SquareRoot:
        ss << " After simplification of square root: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Square:
        ss << " After simplification of square: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Log:
        ss << " After simplification of log: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Exp:
        ss << " After simplification of exp: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Cos:
        ss << " After simplification of cos: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::ArcCos:
        ss << " After simplification of arccos: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Sin:
        ss << " After simplification of sin: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::ArcSin:
        ss << " After simplification of arcsin: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Tan:
        ss << " After simplification of tan: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::ArcTan:
        ss << " After simplification of arctan: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Abs:
        ss << " After simplification of abs: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Divide:
        ss << " After simplification of divide: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Power:
        ss << " After simplification of power: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Sum:
        ss << " After simplification of sum: " << *expression << std::endl;
        break;
    case E_NonlinearExpressionTypes::Product:
        ss << " After simplification of product: " << *expression << std::endl;
        break;
    default:
        assert(false);
    }

    std::cout << ss.str() << std::endl;*/

    return (expression);
}

inline std::optional<double> convertProductToConstant(std::shared_ptr<ExpressionProduct> product)
{
    std::optional<double> resultingConstant;

    if(product->getNumberOfChildren() == 0)
        return resultingConstant;

    double tmpConstant = 1.0;

    for(auto& C : product->children)
    {
        if(C->getType() != E_NonlinearExpressionTypes::Constant)
            return resultingConstant;

        tmpConstant *= std::dynamic_pointer_cast<ExpressionConstant>(C)->constant;
    }

    resultingConstant = tmpConstant;

    return resultingConstant;
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

inline std::optional<std::tuple<QuadraticTermPtr, LinearTermPtr, double>> convertSquareToUnivariteQuadraticExpression(
    std::shared_ptr<ExpressionSquare> product)
{
    std::optional<std::tuple<QuadraticTermPtr, LinearTermPtr, double>> resultingExpression;

    if(product->getNumberOfChildren() == 0)
        return resultingExpression;

    if(product->child->getType() != E_NonlinearExpressionTypes::Sum)
        return resultingExpression;

    // Now know we have a square of a sum

    auto sum = std::dynamic_pointer_cast<ExpressionSum>(product->child);

    if(sum->getNumberOfChildren() != 2)
        return resultingExpression;

    double constant = 0.0;
    VariablePtr variable;
    double variableCoefficient = 1.0;

    if(sum->children[0]->getType() == E_NonlinearExpressionTypes::Constant)
    {
        constant = std::dynamic_pointer_cast<ExpressionConstant>(sum->children[0])->constant;

        if(sum->children[1]->getType() == E_NonlinearExpressionTypes::Variable)
            variable = std::dynamic_pointer_cast<ExpressionVariable>(sum->children[1])->variable;
        else if(sum->children[1]->getType() == E_NonlinearExpressionTypes::Product
            && std::dynamic_pointer_cast<ExpressionProduct>(sum->children[1])->isLinearTerm())
        {
            auto linearTerm
                = convertProductToLinearTerm(std::dynamic_pointer_cast<ExpressionProduct>(sum->children[1]));

            variable = linearTerm.value()->variable;
            variableCoefficient = linearTerm.value()->coefficient;
        }
        else
            return resultingExpression;
    }
    else if(sum->children[1]->getType() == E_NonlinearExpressionTypes::Constant)
    {
        constant = std::dynamic_pointer_cast<ExpressionConstant>(sum->children[1])->constant;

        if(sum->children[0]->getType() == E_NonlinearExpressionTypes::Variable)
            variable = std::dynamic_pointer_cast<ExpressionVariable>(sum->children[0])->variable;
        else if(sum->children[0]->getType() == E_NonlinearExpressionTypes::Product
            && std::dynamic_pointer_cast<ExpressionProduct>(sum->children[0])->isLinearTerm())
        {
            auto linearTerm
                = convertProductToLinearTerm(std::dynamic_pointer_cast<ExpressionProduct>(sum->children[0]));

            variable = linearTerm.value()->variable;
            variableCoefficient = linearTerm.value()->coefficient;
        }
        else
            return resultingExpression;
    }

    resultingExpression = std::make_tuple(
        std::make_shared<QuadraticTerm>(variableCoefficient * variableCoefficient, variable, variable),
        std::make_shared<LinearTerm>(2.0 * constant * variableCoefficient, variable), constant * constant);

    return resultingExpression;
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
        if(C->getType() == E_NonlinearExpressionTypes::Constant)
        {
            signomialTerm->coefficient *= std::dynamic_pointer_cast<ExpressionConstant>(C)->constant;
            continue;
        }

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
    extractTermsAndConstant(NonlinearExpressionPtr expression, bool extractMonomials, bool extractSignomials,
        bool extractQuadratics, bool extractLinears)
{
    double constant = 0.0;
    LinearTerms linearTerms;
    QuadraticTerms quadraticTerms;
    MonomialTerms monomialTerms;
    SignomialTerms signomialTerms;
    NonlinearExpressionPtr nonlinearExpression;

    if(expression->getType() == E_NonlinearExpressionTypes::Constant)
    {
        auto expressionConstant = std::dynamic_pointer_cast<ExpressionConstant>(expression);

        if(expressionConstant->constant != 0.0)
        {
            constant += expressionConstant->constant;
        }
    }
    else if(expression->getType() == E_NonlinearExpressionTypes::Variable && extractLinears)
    {
        auto variable = std::dynamic_pointer_cast<ExpressionVariable>(expression);
        linearTerms.add(std::make_shared<LinearTerm>(1.0, variable->variable));

        nonlinearExpression = std::make_shared<ExpressionConstant>(0.0);
    }
    else if(expression->getType() == E_NonlinearExpressionTypes::Square)
    {
        auto square = std::dynamic_pointer_cast<ExpressionSquare>(expression);

        if(auto optional = convertSquareToQuadraticTerm(square); optional && extractQuadratics)
        {
            quadraticTerms.add(optional.value());
        }
        else if(auto optional = convertSquareToUnivariteQuadraticExpression(square);
                optional && extractQuadratics && extractLinears)
        {
            quadraticTerms.add(std::get<0>(optional.value()));
            linearTerms.add(std::get<1>(optional.value()));
            constant += std::get<2>(optional.value());
        }
        else if(auto optional = convertExpressionToSignomialTerm(square); optional && extractSignomials)
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

        if(auto optional = convertExpressionToSignomialTerm(squareRoot); optional && extractSignomials)
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

        if(auto optional = convertExpressionToSignomialTerm(inversion); optional && extractSignomials)
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

        bool converted = false;

        if(negation->child->getType() == E_NonlinearExpressionTypes::Product)
        {
            if(auto optional
                = convertProductToQuadraticTerm(std::dynamic_pointer_cast<ExpressionProduct>(negation->child));
                optional && extractQuadratics)
            {
                optional.value()->coefficient *= -1.0;
                quadraticTerms.add(optional.value());
                converted = true;
            }
        }
        else if(negation->child->getType() == E_NonlinearExpressionTypes::Power)
        {
            if(auto optional = convertPowerToQuadraticTerm(std::dynamic_pointer_cast<ExpressionPower>(negation->child));
                optional && extractQuadratics)
            {
                optional.value()->coefficient *= -1.0;
                quadraticTerms.add(optional.value());
                converted = true;
            }
        }
        else if(negation->child->getType() == E_NonlinearExpressionTypes::Square)
        {
            if(auto optional
                = convertSquareToQuadraticTerm(std::dynamic_pointer_cast<ExpressionSquare>(negation->child));
                optional && extractQuadratics)
            {
                optional.value()->coefficient *= -1.0;
                quadraticTerms.add(optional.value());
                converted = true;
            }
        }

        if(!converted)
        {
            if(auto optional = convertExpressionToSignomialTerm(negation); optional && extractSignomials)
            {
                signomialTerms.add(optional.value());
            }
            else
            {
                nonlinearExpression = expression;
            }
        }
    }
    else if(expression->getType() == E_NonlinearExpressionTypes::Divide)
    {
        auto division = std::dynamic_pointer_cast<ExpressionDivide>(expression);

        if(auto optional = convertExpressionToSignomialTerm(division); optional && extractSignomials)
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

        if(auto optional = convertPowerToLinearTerm(power); optional && extractLinears)
        {
            linearTerms.add(optional.value());
        }
        else if(auto optional = convertPowerToQuadraticTerm(power); optional && extractQuadratics)
        {
            quadraticTerms.add(optional.value());
        }
        else if(auto optional = convertExpressionToSignomialTerm(power); optional && extractSignomials)
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

        if(auto optional = convertProductToLinearTerm(product); optional && extractLinears)
        {
            linearTerms.add(optional.value());
        }
        else if(auto optional = convertProductToConstant(product); optional)
        {
            constant += optional.value();
        }
        else if(auto optional = convertProductToQuadraticTerm(product); optional && extractQuadratics)
        {
            quadraticTerms.add(optional.value());
        }
        else if(auto optional = convertProductToMonomialTerm(product); optional && extractMonomials)
        {
            monomialTerms.add(optional.value());
        }
        else if(auto optional = convertExpressionToSignomialTerm(product); optional && extractSignomials)
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
        NonlinearExpressions children;

        // Extracts the child expressions of the sum
        for(auto& C : std::dynamic_pointer_cast<ExpressionSum>(expression)->children)
        {
            auto [tmpLinearTerms, tmpQuadraticTerms, tmpMonomialTerms, tmpSignomialTerms, tmpNonlinearExpression,
                tmpConstant]
                = extractTermsAndConstant(C, extractMonomials, extractSignomials, extractQuadratics, extractLinears);

            linearTerms.add(tmpLinearTerms);
            quadraticTerms.add(tmpQuadraticTerms);
            monomialTerms.add(tmpMonomialTerms);
            signomialTerms.add(tmpSignomialTerms);
            constant += tmpConstant;

            if(tmpNonlinearExpression != nullptr)
                children.push_back(tmpNonlinearExpression);
        }

        if(children.size() == 0)
            // The nonlinear expression has been fully extracted
            nonlinearExpression = std::make_shared<ExpressionConstant>(0.0);
        else
        {
            std::dynamic_pointer_cast<ExpressionSum>(expression)->children = children;
            nonlinearExpression = expression;
        }
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

    // Need to go through them again and check if there are fixed variables that can be simplified
    LinearTerms newLinearTerms;
    QuadraticTerms newQuadraticTerms;
    MonomialTerms newMonomialTerms;
    SignomialTerms newSignomialTerms;
    double newConstant = constant;

    for(auto& LT : linearTerms)
    {
        if(LT->variable->lowerBound == LT->variable->upperBound)
            newConstant += LT->coefficient * LT->variable->lowerBound;
        else
            newLinearTerms.add(LT);
    }

    for(auto& QT : quadraticTerms)
    {
        VariablePtr firstVariable = QT->firstVariable;
        VariablePtr secondVariable = QT->secondVariable;

        bool firstVariableFixed = firstVariable->lowerBound == firstVariable->upperBound;
        bool secondVariableFixed = secondVariable->lowerBound == secondVariable->upperBound;

        if(firstVariableFixed && secondVariableFixed)
            newConstant += QT->coefficient * firstVariable->lowerBound * secondVariable->lowerBound;
        else if(firstVariableFixed)
            newLinearTerms.add(
                std::make_shared<LinearTerm>(QT->coefficient * firstVariable->lowerBound, secondVariable));
        else if(secondVariableFixed)
            newLinearTerms.add(
                std::make_shared<LinearTerm>(QT->coefficient * secondVariable->lowerBound, firstVariable));
        else
            newQuadraticTerms.add(QT);
    }

    for(auto& MT : monomialTerms)
    {
        double coefficient = MT->coefficient;
        Variables variables;

        for(auto& V : MT->variables)
        {
            if(V->lowerBound == V->upperBound)
                coefficient *= V->lowerBound;
            else
                variables.push_back(V);
        }

        if(variables.size() == 0)
            newConstant += coefficient;
        else if(variables.size() == 1)
            newLinearTerms.add(std::make_shared<LinearTerm>(coefficient, variables[0]));
        else
            newMonomialTerms.add(std::make_shared<MonomialTerm>(coefficient, variables));
    }

    for(auto& ST : signomialTerms)
    {
        double coefficient = ST->coefficient;
        SignomialElements elements;

        for(auto& E : ST->elements)
            if(E->variable->lowerBound == E->variable->upperBound)
                coefficient *= std::pow(E->variable->lowerBound, E->power);
            else
                elements.push_back(E);

        if(elements.size() == 0)
            newConstant += coefficient;
        else if(elements.size() == 1)
        {
            if(elements[0]->power == 1.0)
                newLinearTerms.add(std::make_shared<LinearTerm>(coefficient, elements[0]->variable));
            else if(elements[0]->power == 2.0)
                newQuadraticTerms.add(
                    std::make_shared<QuadraticTerm>(coefficient, elements[0]->variable, elements[0]->variable));
            else
                newSignomialTerms.add(std::make_shared<SignomialTerm>(coefficient, elements));
        }
        else
            newSignomialTerms.add(std::make_shared<SignomialTerm>(coefficient, elements));
    }

    if(nonlinearExpression != nullptr)
        nonlinearExpression = simplify(nonlinearExpression);

    if(auto sharedOwnerProblem = expression->ownerProblem.lock())
    {
        for(auto& T : newLinearTerms)
            T->takeOwnership(sharedOwnerProblem);

        for(auto& T : newQuadraticTerms)
            T->takeOwnership(sharedOwnerProblem);

        for(auto& T : newMonomialTerms)
            T->takeOwnership(sharedOwnerProblem);

        for(auto& T : newSignomialTerms)
            T->takeOwnership(sharedOwnerProblem);

        if(nonlinearExpression)
            nonlinearExpression->takeOwnership(sharedOwnerProblem);
    }

    return std::make_tuple(
        newLinearTerms, newQuadraticTerms, newMonomialTerms, newSignomialTerms, nonlinearExpression, newConstant);
}

void simplifyNonlinearExpressions(
    ProblemPtr problem, bool extractMonomials, bool extractSignomials, bool extractQuadratics);

} // namespace SHOT
