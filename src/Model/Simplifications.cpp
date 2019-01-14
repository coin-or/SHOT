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
    if (problem->objectiveFunction->properties.hasNonlinearExpression)
    {
        auto objective = std::dynamic_pointer_cast<NonlinearObjectiveFunction>(problem->objectiveFunction);

        auto nonlinearExpression = simplify(objective->nonlinearExpression);

        auto [tmpLinearTerms, tmpQuadraticTerms, tmpNonlinearExpression, tmpConstant] = extractTermsAndConstant(nonlinearExpression);

        if (tmpLinearTerms.size() > 0)
            objective->add(tmpLinearTerms);

        if (tmpQuadraticTerms.size() > 0)
            objective->add(tmpQuadraticTerms);

        if (tmpNonlinearExpression != nullptr)
            objective->nonlinearExpression = simplify(tmpNonlinearExpression);
    }

    for (auto &C : problem->nonlinearConstraints)
    {
        auto nonlinearExpression = simplify(C->nonlinearExpression);

        auto [tmpLinearTerms, tmpQuadraticTerms, tmpNonlinearExpression, tmpConstant] = extractTermsAndConstant(nonlinearExpression);

        if (tmpLinearTerms.size() > 0)
            C->add(tmpLinearTerms);

        if (tmpQuadraticTerms.size() > 0)
            C->add(tmpQuadraticTerms);

        if (tmpNonlinearExpression != nullptr)
            C->nonlinearExpression = simplify(tmpNonlinearExpression);
    }

    for (auto &C : problem->nonlinearConstraints)
    {
        if (C->nonlinearExpression->getType() == E_NonlinearExpressionTypes::SquareRoot && C->linearTerms.size() == 0 && C->quadraticTerms.size() == 0)
        {
            // Can take the square of both sides

            C->nonlinearExpression = std::dynamic_pointer_cast<ExpressionSquareRoot>(C->nonlinearExpression)->child;
            if (abs(C->valueLHS) < SHOT_DBL_MAX)
                C->valueLHS = C->valueLHS * C->valueLHS;
            if (abs(C->valueRHS) < SHOT_DBL_MAX)
                C->valueRHS = C->valueRHS * C->valueRHS;
        }
    }

    for (auto &C : problem->nonlinearConstraints)
    {
        if (C->nonlinearExpression->getType() == E_NonlinearExpressionTypes::Sum)
        {
            // Removes linear terms, quadratics and constants

            for (auto &T : std::dynamic_pointer_cast<ExpressionSum>(C->nonlinearExpression)->children.expressions)
            {
                if (T->getType() == E_NonlinearExpressionTypes::Constant)
                {
                    C->constant += std::dynamic_pointer_cast<ExpressionConstant>(T)->constant;
                    std::dynamic_pointer_cast<ExpressionConstant>(T)->constant = 0.0; // Will be removed during simplification later on
                }
                else if (T->getType() == E_NonlinearExpressionTypes::Variable)
                {
                    auto variable = std::dynamic_pointer_cast<ExpressionVariable>(T);

                    C->linearTerms.add(std::make_shared<LinearTerm>(1.0, variable->variable));
                    T = std::make_shared<ExpressionConstant>(0.0); // Will be removed during simplification later on
                }
                else if (T->getType() == E_NonlinearExpressionTypes::Product && T->getNumberOfChildren() == 2)
                {
                    auto product = std::dynamic_pointer_cast<ExpressionProduct>(T);

                    if (product->children.expressions.at(0)->getType() == E_NonlinearExpressionTypes::Constant && product->children.expressions.at(1)->getType() == E_NonlinearExpressionTypes::Variable)
                    {
                        double constant = std::dynamic_pointer_cast<ExpressionConstant>(product->children.expressions.at(0))->constant;
                        auto variable = std::dynamic_pointer_cast<ExpressionVariable>(product->children.expressions.at(1))->variable;

                        C->linearTerms.add(std::make_shared<LinearTerm>(constant, variable));
                        T = std::make_shared<ExpressionConstant>(0.0); // Will be removed during simplification later on
                    }
                    else if (product->children.expressions.at(1)->getType() == E_NonlinearExpressionTypes::Constant && product->children.expressions.at(0)->getType() == E_NonlinearExpressionTypes::Variable)
                    {
                        double constant = std::dynamic_pointer_cast<ExpressionConstant>(product->children.expressions.at(1))->constant;
                        auto variable = std::dynamic_pointer_cast<ExpressionVariable>(product->children.expressions.at(0))->variable;

                        C->linearTerms.add(std::make_shared<LinearTerm>(constant, variable));
                        T = std::make_shared<ExpressionConstant>(0.0); // Will be removed during simplification later on
                    }
                    else if (product->children.expressions.at(1)->getType() == E_NonlinearExpressionTypes::Variable && product->children.expressions.at(0)->getType() == E_NonlinearExpressionTypes::Variable)
                    {
                        auto firstVariable = std::dynamic_pointer_cast<ExpressionVariable>(product->children.expressions.at(0))->variable;
                        auto secondVariable = std::dynamic_pointer_cast<ExpressionVariable>(product->children.expressions.at(1))->variable;

                        C->quadraticTerms.add(std::make_shared<QuadraticTerm>(1.0, firstVariable, secondVariable));
                        T = std::make_shared<ExpressionConstant>(0.0); // Will be removed during simplification later on
                    }
                }
                else if (T->getType() == E_NonlinearExpressionTypes::Product && T->getNumberOfChildren() == 3)
                {
                    auto product = std::dynamic_pointer_cast<ExpressionProduct>(T);
                    Variables variables;
                    double constant = 0.0;
                    int numVariables = 0;
                    int numConstants = 0;

                    for (auto &E : product->children.expressions)
                    {
                        if (E->getType() == E_NonlinearExpressionTypes::Constant)
                        {
                            numConstants++;
                            constant = std::dynamic_pointer_cast<ExpressionConstant>(E)->constant;
                        }
                        else if (E->getType() == E_NonlinearExpressionTypes::Variable)
                        {
                            numVariables++;
                            variables.push_back(std::dynamic_pointer_cast<ExpressionVariable>(E)->variable);
                        }
                    }

                    if (numVariables + numConstants != 3)
                    {
                        continue;
                    }

                    if (numVariables == 1)
                    {
                        C->linearTerms.add(std::make_shared<LinearTerm>(constant, variables.at(0)));
                    }

                    if (numVariables == 2)
                    {
                        C->quadraticTerms.add(std::make_shared<QuadraticTerm>(constant, variables.at(0), variables.at(1)));
                    }

                    T = std::make_shared<ExpressionConstant>(0.0); // Will be removed during simplification later on
                }
            }
        }
    }

    // Runs again to remove zeroes
    for (auto &C : problem->nonlinearConstraints)
    {
        C->nonlinearExpression = simplify(C->nonlinearExpression);
    }
}

} // namespace SHOT
