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
std::optional<std::tuple<double, VariablePtr, double>> ExpressionSum::getLinearTermAndConstant()
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
        }
    }

    if(variable)
        result = std::make_tuple(coefficient, variable, constant);

    return (result);
}

bool checkPerspectiveConvexity(
    NonlinearExpressionPtr expression, double linearCoefficient, VariablePtr linearVariable, double constant)
{
    auto isConvex = false;

    switch(expression->getType())
    {
    case E_NonlinearExpressionTypes::Divide:
        isConvex = checkPerspectiveConvexity(
            std::dynamic_pointer_cast<ExpressionDivide>(expression), linearCoefficient, linearVariable, constant);
        break;
    case E_NonlinearExpressionTypes::Negate:
        isConvex = checkPerspectiveConvexity(
            std::dynamic_pointer_cast<ExpressionNegate>(expression), linearCoefficient, linearVariable, constant);
        break;
    case E_NonlinearExpressionTypes::Log:
        isConvex = checkPerspectiveConvexity(
            std::dynamic_pointer_cast<ExpressionLog>(expression), linearCoefficient, linearVariable, constant);
        break;
    case E_NonlinearExpressionTypes::Square:
        isConvex = checkPerspectiveConvexity(
            std::dynamic_pointer_cast<ExpressionSquare>(expression), linearCoefficient, linearVariable, constant);
        break;
    case E_NonlinearExpressionTypes::Product:
        isConvex = checkPerspectiveConvexity(
            std::dynamic_pointer_cast<ExpressionProduct>(expression), linearCoefficient, linearVariable, constant);
        break;
    default:
        break;
    }

    return (isConvex);
}

bool checkPerspectiveConvexity(
    std::shared_ptr<ExpressionDivide> expression, double linearCoefficient, VariablePtr linearVariable, double constant)
{
    auto nominator = expression->firstChild;
    auto denominator = expression->secondChild;

    if(nominator->getType() == E_NonlinearExpressionTypes::Variable) { }
    else if(nominator->getType() == E_NonlinearExpressionTypes::Product
        && std::dynamic_pointer_cast<ExpressionProduct>(nominator)->isLinearTerm())
    {
    }
    else
    {
        return (false);
    }

    if(denominator->getType() != E_NonlinearExpressionTypes::Sum)
        return (false);

    if(auto linearTermAndConstant = std::dynamic_pointer_cast<ExpressionSum>(denominator)->getLinearTermAndConstant();
        linearTermAndConstant)
    {
        double denominatorCoefficient = std::get<0>(*linearTermAndConstant);
        VariablePtr denominatorVariable = std::get<1>(*linearTermAndConstant);
        double denominatorConstant = std::get<2>(*linearTermAndConstant);

        if(denominatorCoefficient == linearCoefficient && denominatorVariable == linearVariable
            && denominatorConstant == constant)
        {
            return (true);
        }
    }

    return (false);
}

bool checkPerspectiveConvexity(
    std::shared_ptr<ExpressionNegate> expression, double linearCoefficient, VariablePtr linearVariable, double constant)
{
    if(expression->child->getType() == E_NonlinearExpressionTypes::Log)
    {
        return (checkPerspectiveConvexity(
            std::dynamic_pointer_cast<ExpressionLog>(expression->child), linearCoefficient, linearVariable, constant));
    }

    if(expression->child->getType() == E_NonlinearExpressionTypes::Product)
    {
        auto product = std::dynamic_pointer_cast<ExpressionProduct>(expression->child);

        if(product->children[0]->getType() == E_NonlinearExpressionTypes::Constant
            && product->children[0]->getBounds().l() > 0
            && product->children[1]->getType() == E_NonlinearExpressionTypes::Log)
        {
            return (checkPerspectiveConvexity(std::dynamic_pointer_cast<ExpressionLog>(product->children[1]),
                linearCoefficient, linearVariable, constant));
        }
    }

    if(expression->child->getType() == E_NonlinearExpressionTypes::Divide)
    {
        return (checkPerspectiveConvexity(std::dynamic_pointer_cast<ExpressionDivide>(expression->child),
            linearCoefficient, linearVariable, constant));
    }

    return (false);
}

bool checkPerspectiveConvexity(
    std::shared_ptr<ExpressionSquare> expression, double linearCoefficient, VariablePtr linearVariable, double constant)
{
    if(expression->child->getType() == E_NonlinearExpressionTypes::Divide)
    {
        return (checkPerspectiveConvexity(std::dynamic_pointer_cast<ExpressionDivide>(expression->child),
            linearCoefficient, linearVariable, constant));
    }

    return (false);
}

bool checkPerspectiveConvexity(
    std::shared_ptr<ExpressionLog> expression, double linearCoefficient, VariablePtr linearVariable, double constant)
{
    if(expression->child->getType() == E_NonlinearExpressionTypes::Sum && expression->child->getNumberOfChildren() == 2)
    {
        auto sum = std::dynamic_pointer_cast<ExpressionSum>(expression->child);

        if(sum->children[0]->getType() == E_NonlinearExpressionTypes::Constant
            && sum->children[0]->getBounds().l() == 1.0
            && sum->children[1]->getType() == E_NonlinearExpressionTypes::Divide)
        {
            return (checkPerspectiveConvexity(std::dynamic_pointer_cast<ExpressionDivide>(sum->children[1]),
                linearCoefficient, linearVariable, constant));
        }
    }

    return (false);
}

bool checkPerspectiveConvexity(std::shared_ptr<ExpressionProduct> expression, double linearCoefficient,
    VariablePtr linearVariable, double constant)
{
    if(expression->getNumberOfChildren() == 2)
    {
        if(expression->children[0]->getType() == E_NonlinearExpressionTypes::Constant
            && expression->children[0]->getBounds().l() < 0
            && expression->children[1]->getType() == E_NonlinearExpressionTypes::Divide)
        {
            return (checkPerspectiveConvexity(std::dynamic_pointer_cast<ExpressionDivide>(expression->children[1]),
                linearCoefficient, linearVariable, constant));
        }

        if(expression->children[0]->getType() == E_NonlinearExpressionTypes::Constant
            && expression->children[0]->getBounds().l() < 0
            && expression->children[1]->getType() == E_NonlinearExpressionTypes::Log)
        {
            return (checkPerspectiveConvexity(std::dynamic_pointer_cast<ExpressionLog>(expression->children[1]),
                linearCoefficient, linearVariable, constant));
        }

        if(auto linearTerm = std::dynamic_pointer_cast<ExpressionProduct>(expression)->getLinearTerm(); linearTerm)
        {
            double coefficient = std::get<0>(*linearTerm);
            VariablePtr variable = std::get<1>(*linearTerm);

            if(linearCoefficient * coefficient > 0 && linearVariable == variable)
                return (true);
        }
    }

    return (false);
}
}