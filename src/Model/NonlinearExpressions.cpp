/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "NonlinearExpressions.h"

namespace SHOT
{
std::optional<std::tuple<double, VariablePtr, double>> ExpressionSum::getAsLinearTermPlusConstant()
{
    std::optional<std::tuple<double, VariablePtr, double>> result;

    if(getNumberOfChildren() > 2)
        return (result);

    double constant = 0.0;
    double coefficient = 1.0;
    VariablePtr variable;

    for(auto& C : children)
    {
        if(C->getType() == E_NonlinearExpressionTypes::Constant)
        {
            constant += std::dynamic_pointer_cast<ExpressionConstant>(C)->constant;
        }
        else if(C->getType() == E_NonlinearExpressionTypes::Variable)
        {
            if(variable)
                return (result);

            variable = std::dynamic_pointer_cast<ExpressionVariable>(C)->variable;
        }
        else if(C->getType() == E_NonlinearExpressionTypes::Product && C->getNumberOfChildren() == 2)
        {
            if(variable)
                return (result);

            auto product = std::dynamic_pointer_cast<ExpressionProduct>(C);

            if(product->children[0]->getType() == E_NonlinearExpressionTypes::Constant
                && product->children[1]->getType() == E_NonlinearExpressionTypes::Variable)
            {
                coefficient = std::dynamic_pointer_cast<ExpressionConstant>(product->children[0])->constant;
                variable = std::dynamic_pointer_cast<ExpressionVariable>(product->children[1])->variable;
            }
            else if(product->children[1]->getType() == E_NonlinearExpressionTypes::Constant
                && product->children[0]->getType() == E_NonlinearExpressionTypes::Variable)
            {
                coefficient = std::dynamic_pointer_cast<ExpressionConstant>(product->children[1])->constant;
                variable = std::dynamic_pointer_cast<ExpressionVariable>(product->children[0])->variable;
            }
            else
            {
                return (result);
            }
        }
        else
        {
            return (result);
        }
    }

    if(variable)
        result = std::make_tuple(coefficient, variable, constant);

    return (result);
}

namespace
{
// Checks whether the expression is an affine function of the scaled variables x/s, where s = linearCoefficient *
// linearVariable + constant is the linear factor of the perspective, i.e. whether s times the expression is linear
bool isAffineInPerspectiveVariables(const NonlinearExpressionPtr& expression, double linearCoefficient,
    const VariablePtr& linearVariable, double constant)
{
    switch(expression->getType())
    {
    case E_NonlinearExpressionTypes::Constant:
        return (true);

    case E_NonlinearExpressionTypes::Divide:
    {
        auto divide = std::dynamic_pointer_cast<ExpressionDivide>(expression);
        auto nominator = divide->firstChild;
        auto denominator = divide->secondChild;

        if(nominator->getType() != E_NonlinearExpressionTypes::Variable
            && !(nominator->getType() == E_NonlinearExpressionTypes::Product
                && std::dynamic_pointer_cast<ExpressionProduct>(nominator)->isLinearTerm()))
            return (false);

        if(denominator->getType() != E_NonlinearExpressionTypes::Sum)
            return (false);

        auto linearTermAndConstant
            = std::dynamic_pointer_cast<ExpressionSum>(denominator)->getAsLinearTermPlusConstant();

        return (linearTermAndConstant && std::get<0>(*linearTermAndConstant) == linearCoefficient
            && std::get<1>(*linearTermAndConstant) == linearVariable
            && std::get<2>(*linearTermAndConstant) == constant);
    }

    case E_NonlinearExpressionTypes::Negate:
        return (isAffineInPerspectiveVariables(std::dynamic_pointer_cast<ExpressionNegate>(expression)->child,
            linearCoefficient, linearVariable, constant));

    case E_NonlinearExpressionTypes::Product:
    {
        if(expression->getNumberOfChildren() != 2)
            return (false);

        auto product = std::dynamic_pointer_cast<ExpressionProduct>(expression);

        if(product->children[0]->getType() == E_NonlinearExpressionTypes::Constant)
            return (isAffineInPerspectiveVariables(product->children[1], linearCoefficient, linearVariable, constant));

        if(product->children[1]->getType() == E_NonlinearExpressionTypes::Constant)
            return (isAffineInPerspectiveVariables(product->children[0], linearCoefficient, linearVariable, constant));

        return (false);
    }

    case E_NonlinearExpressionTypes::Sum:
    {
        for(auto& C : std::dynamic_pointer_cast<ExpressionSum>(expression)->children)
        {
            if(!isAffineInPerspectiveVariables(C, linearCoefficient, linearVariable, constant))
                return (false);
        }

        return (true);
    }

    default:
        return (false);
    }
}

// Checks whether s times the expression is convex (sign > 0) or concave (sign < 0), where s is the linear factor of
// the perspective. This holds if the expression is a convex (concave) function of the scaled variables x/s.
bool isPerspectiveConvex(const NonlinearExpressionPtr& expression, double sign, double linearCoefficient,
    const VariablePtr& linearVariable, double constant)
{
    if(isAffineInPerspectiveVariables(expression, linearCoefficient, linearVariable, constant))
        return (true);

    switch(expression->getType())
    {
    case E_NonlinearExpressionTypes::Variable:
    {
        // s * y = constant * y + linearCoefficient * y^2 if y is the variable in the linear factor
        if(std::dynamic_pointer_cast<ExpressionVariable>(expression)->variable != linearVariable)
            return (false);

        return (sign * linearCoefficient >= 0);
    }

    case E_NonlinearExpressionTypes::Negate:
        return (isPerspectiveConvex(std::dynamic_pointer_cast<ExpressionNegate>(expression)->child, -sign,
            linearCoefficient, linearVariable, constant));

    case E_NonlinearExpressionTypes::Product:
    {
        if(expression->getNumberOfChildren() != 2)
            return (false);

        auto product = std::dynamic_pointer_cast<ExpressionProduct>(expression);

        for(int i = 0; i < 2; i++)
        {
            if(product->children[i]->getType() != E_NonlinearExpressionTypes::Constant)
                continue;

            double factor = std::dynamic_pointer_cast<ExpressionConstant>(product->children[i])->constant;

            if(factor == 0.0)
                return (true);

            return (isPerspectiveConvex(product->children[1 - i], factor > 0 ? sign : -sign, linearCoefficient,
                linearVariable, constant));
        }

        return (false);
    }

    case E_NonlinearExpressionTypes::Sum:
    {
        for(auto& C : std::dynamic_pointer_cast<ExpressionSum>(expression)->children)
        {
            if(!isPerspectiveConvex(C, sign, linearCoefficient, linearVariable, constant))
                return (false);
        }

        return (true);
    }

    case E_NonlinearExpressionTypes::Square:
        return (sign > 0
            && isAffineInPerspectiveVariables(std::dynamic_pointer_cast<ExpressionSquare>(expression)->child,
                linearCoefficient, linearVariable, constant));

    case E_NonlinearExpressionTypes::Log:
        return (sign < 0
            && isAffineInPerspectiveVariables(std::dynamic_pointer_cast<ExpressionLog>(expression)->child,
                linearCoefficient, linearVariable, constant));

    default:
        return (false);
    }
}
} // namespace

bool checkPerspectiveConvexity(
    NonlinearExpressionPtr expression, double linearCoefficient, VariablePtr linearVariable, double constant)
{
    return (isPerspectiveConvex(expression, 1.0, linearCoefficient, linearVariable, constant));
}
} // namespace SHOT
