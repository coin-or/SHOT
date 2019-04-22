/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Stefan Vigerske, GAMS Development Corp.
   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "Simplifications.h"

namespace SHOT
{

void simplifyNonlinearExpressions(ProblemPtr problem)
{
    if(problem->objectiveFunction->properties.hasNonlinearExpression)
    {
        auto nonlinearObjective = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(problem->objectiveFunction);

        auto nonlinearExpression = simplify(nonlinearObjective->nonlinearExpression);

        auto [tmpLinearTerms, tmpQuadraticTerms, tmpMonomialTerms, tmpSignomialTerms, tmpNonlinearExpression,
            tmpConstant]
            = extractTermsAndConstant(nonlinearExpression);

        if(tmpMonomialTerms.size() == 0 && tmpSignomialTerms.size() == 0 && !tmpNonlinearExpression
            && nonlinearObjective->monomialTerms.size() == 0 && nonlinearObjective->signomialTerms.size() == 0)
        {
            // The objective is no longer nonlinear

            if(tmpQuadraticTerms.size() > 0 || nonlinearObjective->quadraticTerms.size() > 0)
            {
                // The objective is quadratic
                auto newObjective = std::make_shared<QuadraticObjectiveFunction>();
                newObjective->constant = nonlinearObjective->constant;
                newObjective->direction = nonlinearObjective->direction;
                newObjective->linearTerms = nonlinearObjective->linearTerms;
                newObjective->quadraticTerms = nonlinearObjective->quadraticTerms;

                if(tmpLinearTerms.size() > 0)
                    newObjective->add(tmpLinearTerms);

                if(tmpQuadraticTerms.size() > 0)
                    newObjective->add(tmpQuadraticTerms);

                if(tmpConstant != 0.0)
                    newObjective->constant += tmpConstant;

                problem->objectiveFunction = nonlinearObjective;
            }
            else
            {
                // The objective is linear
                auto newObjective = std::make_shared<LinearObjectiveFunction>();
                newObjective->constant = nonlinearObjective->constant;
                newObjective->direction = nonlinearObjective->direction;
                newObjective->linearTerms = nonlinearObjective->linearTerms;

                if(tmpLinearTerms.size() > 0)
                    newObjective->add(tmpLinearTerms);

                if(tmpConstant != 0.0)
                    newObjective->constant += tmpConstant;

                problem->objectiveFunction = nonlinearObjective;
            }
        }
        else
        {
            if(tmpLinearTerms.size() > 0)
                nonlinearObjective->add(tmpLinearTerms);

            if(tmpQuadraticTerms.size() > 0)
                nonlinearObjective->add(tmpQuadraticTerms);

            if(tmpMonomialTerms.size() > 0)
                nonlinearObjective->add(std::move(tmpMonomialTerms));

            if(tmpSignomialTerms.size() > 0)
                nonlinearObjective->add(std::move(tmpSignomialTerms));

            nonlinearObjective->nonlinearExpression = tmpNonlinearExpression;

            if(tmpConstant != 0.0)
                nonlinearObjective->constant += tmpConstant;
        }
    }

    for(auto& C : problem->numericConstraints)
    {
        if(!C->properties.hasNonlinearExpression)
            continue;

        auto nonlinearConstraint = std::dynamic_pointer_cast<NonlinearConstraint>(C);
        auto nonlinearExpression = simplify(nonlinearConstraint->nonlinearExpression);

        auto [tmpLinearTerms, tmpQuadraticTerms, tmpMonomialTerms, tmpSignomialTerms, tmpNonlinearExpression,
            tmpConstant]
            = extractTermsAndConstant(nonlinearExpression);

        if(tmpMonomialTerms.size() == 0 && tmpSignomialTerms.size() == 0 && !tmpNonlinearExpression
            && nonlinearConstraint->monomialTerms.size() == 0 && nonlinearConstraint->signomialTerms.size() == 0)
        {
            // The constraint is no longer nonlinear

            if(tmpQuadraticTerms.size() > 0 || nonlinearConstraint->quadraticTerms.size() > 0)
            {
                // The constraint is quadratic
                auto newConstraint = std::make_shared<QuadraticConstraint>();
                newConstraint->index = nonlinearConstraint->index;
                newConstraint->name = nonlinearConstraint->name;
                newConstraint->valueLHS = nonlinearConstraint->valueLHS;
                newConstraint->valueRHS = nonlinearConstraint->valueRHS;
                newConstraint->constant = nonlinearConstraint->constant;
                newConstraint->linearTerms = nonlinearConstraint->linearTerms;
                newConstraint->quadraticTerms = nonlinearConstraint->quadraticTerms;

                if(tmpLinearTerms.size() > 0)
                    newConstraint->add(tmpLinearTerms);

                if(tmpQuadraticTerms.size() > 0)
                    newConstraint->add(tmpQuadraticTerms);

                if(tmpConstant != 0.0)
                    newConstraint->constant += tmpConstant;

                C = newConstraint;
            }
            else
            {
                // The constraint is linear
                auto newConstraint = std::make_shared<LinearConstraint>();
                newConstraint->index = nonlinearConstraint->index;
                newConstraint->name = nonlinearConstraint->name;
                newConstraint->valueLHS = nonlinearConstraint->valueLHS;
                newConstraint->valueRHS = nonlinearConstraint->valueRHS;
                newConstraint->constant = nonlinearConstraint->constant;
                newConstraint->linearTerms = nonlinearConstraint->linearTerms;

                if(tmpLinearTerms.size() > 0)
                    newConstraint->add(tmpLinearTerms);

                if(tmpConstant != 0.0)
                    newConstraint->constant += tmpConstant;

                C = newConstraint;
            }
        }
        else
        {
            if(tmpLinearTerms.size() > 0)
                nonlinearConstraint->add(tmpLinearTerms);

            if(tmpQuadraticTerms.size() > 0)
                nonlinearConstraint->add(tmpQuadraticTerms);

            if(tmpMonomialTerms.size() > 0)
                nonlinearConstraint->add(std::move(tmpMonomialTerms));

            if(tmpSignomialTerms.size() > 0)
                nonlinearConstraint->add(std::move(tmpSignomialTerms));

            if(tmpNonlinearExpression)
                nonlinearConstraint->nonlinearExpression = tmpNonlinearExpression;
            else
                nonlinearConstraint->nonlinearExpression = std::make_shared<ExpressionConstant>(0.0);

            if(tmpConstant != 0.0)
                nonlinearConstraint->constant += tmpConstant;
        }
    }

    for(auto& C : problem->nonlinearConstraints)
    {
        if(C->nonlinearExpression->getType() == E_NonlinearExpressionTypes::SquareRoot && C->linearTerms.size() == 0
            && C->quadraticTerms.size() == 0)
        {
            // Can take the square of both sides

            C->nonlinearExpression = std::dynamic_pointer_cast<ExpressionSquareRoot>(C->nonlinearExpression)->child;
            if(abs(C->valueLHS) < SHOT_DBL_MAX)
                C->valueLHS = C->valueLHS * C->valueLHS;
            if(abs(C->valueRHS) < SHOT_DBL_MAX)
                C->valueRHS = C->valueRHS * C->valueRHS;
        }
    }

    for(auto& C : problem->nonlinearConstraints)
    {
        if(C->nonlinearExpression->getType() == E_NonlinearExpressionTypes::Sum)
        {
            // Removes linear terms, quadratics and constants

            for(auto& T : std::dynamic_pointer_cast<ExpressionSum>(C->nonlinearExpression)->children)
            {
                if(T->getType() == E_NonlinearExpressionTypes::Constant)
                {
                    C->constant += std::dynamic_pointer_cast<ExpressionConstant>(T)->constant;
                    std::dynamic_pointer_cast<ExpressionConstant>(T)->constant
                        = 0.0; // Will be removed during simplification later on
                }
                else if(T->getType() == E_NonlinearExpressionTypes::Variable)
                {
                    auto variable = std::dynamic_pointer_cast<ExpressionVariable>(T);

                    C->linearTerms.add(std::make_shared<LinearTerm>(1.0, variable->variable));
                    T = std::make_shared<ExpressionConstant>(0.0); // Will be removed during simplification later on
                }
                else if(T->getType() == E_NonlinearExpressionTypes::Product && T->getNumberOfChildren() == 2)
                {
                    auto product = std::dynamic_pointer_cast<ExpressionProduct>(T);

                    if(product->children.at(0)->getType() == E_NonlinearExpressionTypes::Constant
                        && product->children.at(1)->getType() == E_NonlinearExpressionTypes::Variable)
                    {
                        double constant
                            = std::dynamic_pointer_cast<ExpressionConstant>(product->children.at(0))->constant;
                        auto variable
                            = std::dynamic_pointer_cast<ExpressionVariable>(product->children.at(1))->variable;

                        C->linearTerms.add(std::make_shared<LinearTerm>(constant, variable));
                        T = std::make_shared<ExpressionConstant>(0.0); // Will be removed during simplification later on
                    }
                    else if(product->children.at(1)->getType() == E_NonlinearExpressionTypes::Constant
                        && product->children.at(0)->getType() == E_NonlinearExpressionTypes::Variable)
                    {
                        double constant
                            = std::dynamic_pointer_cast<ExpressionConstant>(product->children.at(1))->constant;
                        auto variable
                            = std::dynamic_pointer_cast<ExpressionVariable>(product->children.at(0))->variable;

                        C->linearTerms.add(std::make_shared<LinearTerm>(constant, variable));
                        T = std::make_shared<ExpressionConstant>(0.0); // Will be removed during simplification later on
                    }
                    else if(product->children.at(1)->getType() == E_NonlinearExpressionTypes::Variable
                        && product->children.at(0)->getType() == E_NonlinearExpressionTypes::Variable)
                    {
                        auto firstVariable
                            = std::dynamic_pointer_cast<ExpressionVariable>(product->children.at(0))->variable;
                        auto secondVariable
                            = std::dynamic_pointer_cast<ExpressionVariable>(product->children.at(1))->variable;

                        C->quadraticTerms.add(std::make_shared<QuadraticTerm>(1.0, firstVariable, secondVariable));
                        T = std::make_shared<ExpressionConstant>(0.0); // Will be removed during simplification later on
                    }
                }
                else if(T->getType() == E_NonlinearExpressionTypes::Product && T->getNumberOfChildren() == 3)
                {
                    auto product = std::dynamic_pointer_cast<ExpressionProduct>(T);
                    Variables variables;
                    double constant = 0.0;
                    int numVariables = 0;
                    int numConstants = 0;

                    for(auto& E : product->children)
                    {
                        if(E->getType() == E_NonlinearExpressionTypes::Constant)
                        {
                            numConstants++;
                            constant = std::dynamic_pointer_cast<ExpressionConstant>(E)->constant;
                        }
                        else if(E->getType() == E_NonlinearExpressionTypes::Variable)
                        {
                            numVariables++;
                            variables.push_back(std::dynamic_pointer_cast<ExpressionVariable>(E)->variable);
                        }
                    }

                    if(numVariables + numConstants != 3)
                    {
                        continue;
                    }

                    if(numVariables == 1)
                    {
                        C->linearTerms.add(std::make_shared<LinearTerm>(constant, variables.at(0)));
                    }

                    if(numVariables == 2)
                    {
                        C->quadraticTerms.add(
                            std::make_shared<QuadraticTerm>(constant, variables.at(0), variables.at(1)));
                    }

                    T = std::make_shared<ExpressionConstant>(0.0); // Will be removed during simplification later on
                }
            }
        }
    }

    // Runs again to remove zeroes
    for(auto& C : problem->nonlinearConstraints)
    {
        C->nonlinearExpression = simplify(C->nonlinearExpression);
    }
}

NonlinearExpressionPtr copyNonlinearExpression(NonlinearExpression* expression, const ProblemPtr destination)
{
    return copyNonlinearExpression(expression, destination.get());
}

NonlinearExpressionPtr copyNonlinearExpression(NonlinearExpression* expression, Problem* destination)
{
    unsigned int i;
    std::ostringstream outStr;
    int numChildren;

    // auto tmp = expression->getType();

    switch(expression->getType())
    {
    case E_NonlinearExpressionTypes::Sum:
        numChildren = ((ExpressionSum*)expression)->getNumberOfChildren();
        switch(numChildren)
        {
        case 0:
            return std::make_shared<ExpressionConstant>(0.);
        case 1:
            return copyNonlinearExpression(((ExpressionSum*)expression)->children[0].get(), destination);
        default:
            NonlinearExpressions terms;
            for(i = 0; i < numChildren; i++)
                terms.push_back(copyNonlinearExpression(((ExpressionSum*)expression)->children[i].get(), destination));
            return std::make_shared<ExpressionSum>(terms);
        }

    case E_NonlinearExpressionTypes::Negate:
        return std::make_shared<ExpressionNegate>(
            copyNonlinearExpression(((ExpressionNegate*)expression)->child.get(), destination));

    case E_NonlinearExpressionTypes::Divide:
        return std::make_shared<ExpressionDivide>(
            copyNonlinearExpression(((ExpressionDivide*)expression)->firstChild.get(), destination),
            copyNonlinearExpression(((ExpressionDivide*)expression)->secondChild.get(), destination));

    case E_NonlinearExpressionTypes::Power:
        return std::make_shared<ExpressionPower>(
            copyNonlinearExpression(((ExpressionPower*)expression)->firstChild.get(), destination),
            copyNonlinearExpression(((ExpressionPower*)expression)->secondChild.get(), destination));

    case E_NonlinearExpressionTypes::Product:
        numChildren = ((ExpressionProduct*)expression)->getNumberOfChildren();
        switch(numChildren)
        {
        case 0:
            return std::make_shared<ExpressionConstant>(0.);
        case 1:
            return copyNonlinearExpression(((ExpressionProduct*)expression)->children[0].get(), destination);
        default:
            NonlinearExpressions factors;
            for(i = 0; i < numChildren; i++)
                factors.push_back(
                    copyNonlinearExpression(((ExpressionProduct*)expression)->children[i].get(), destination));
            return std::make_shared<ExpressionProduct>(factors);
        }

    case E_NonlinearExpressionTypes::Abs:
        return std::make_shared<ExpressionAbs>(
            copyNonlinearExpression((((ExpressionAbs*)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Square:
        return std::make_shared<ExpressionSquare>(
            copyNonlinearExpression((((ExpressionSquare*)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::SquareRoot:
        return std::make_shared<ExpressionSquareRoot>(
            copyNonlinearExpression((((ExpressionSquareRoot*)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Invert:
        return std::make_shared<ExpressionInvert>(
            copyNonlinearExpression((((ExpressionInvert*)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Log:
        return std::make_shared<ExpressionLog>(
            copyNonlinearExpression((((ExpressionLog*)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Exp:
        return std::make_shared<ExpressionExp>(
            copyNonlinearExpression((((ExpressionExp*)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Sin:
        return std::make_shared<ExpressionSin>(
            copyNonlinearExpression((((ExpressionSin*)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Cos:
        return std::make_shared<ExpressionCos>(
            copyNonlinearExpression((((ExpressionCos*)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Tan:
        return std::make_shared<ExpressionTan>(
            copyNonlinearExpression((((ExpressionTan*)expression)->child).get(), destination));

    case E_NonlinearExpressionTypes::Constant:
        return std::make_shared<ExpressionConstant>((((ExpressionConstant*)expression)->constant));

    case E_NonlinearExpressionTypes::Variable:
    {
        int variableIndex = ((ExpressionVariable*)expression)->variable->index;
        return std::make_shared<ExpressionVariable>(destination->getVariable(variableIndex));
    }
    default:
        throw new OperationNotImplementedException(std::to_string((int)(expression->getType())));
        break;
    }

    return nullptr;
}

} // namespace SHOT
