/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "Terms.h"
#include "Problem.h"
#include "../Settings.h"

#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include "Eigen/src/SparseCore/SparseUtil.h"

namespace SHOT
{
Interval Term::getBounds()
{
    IntervalVector variableBounds;

    if(auto sharedOwnerProblem = ownerProblem.lock())
    {
        variableBounds = sharedOwnerProblem->getVariableBounds();
    }

    auto interval = calculate(variableBounds);

    return (interval);
}

void QuadraticTerms::updateConvexity()
{
    if(size() == 0)
    {
        convexity = E_Convexity::Linear;
        return;
    }

    std::vector<Eigen::Triplet<double>> elements;
    elements.reserve(2 * size());

    std::map<VariablePtr, int> variableMap;

    bool allSquares = true;
    bool allPositive = true;
    bool allNegative = true;
    bool allBilinear = true;

    int variableCounter = 0;

    for(auto& T : (*this))
    {
        if(T->firstVariable == T->secondVariable)
        {
            int currentVariableIndex;
            auto element = variableMap.emplace(T->firstVariable, variableCounter);

            if(element.second)
            {
                // Variable not already indexed found
                currentVariableIndex = variableCounter;
                variableCounter++;
            }
            else
            {
                currentVariableIndex = element.first->second;
            }

            allPositive = allPositive && T->coefficient >= 0;
            allNegative = allNegative && T->coefficient <= 0;
            allBilinear = false;

            elements.emplace_back(currentVariableIndex, currentVariableIndex, T->coefficient);
        }
        else
        {
            int currentFirstVariableIndex;
            auto element = variableMap.emplace(T->firstVariable, variableCounter);

            if(element.second)
            {
                // Variable not already indexed found, but inserted into map
                currentFirstVariableIndex = variableCounter;
                variableCounter++;
            }
            else
            {
                currentFirstVariableIndex = element.first->second;
            }

            int currentSecondVariableIndex;
            element = variableMap.emplace(T->secondVariable, variableCounter);

            if(element.second)
            {
                // Variable not already indexed found, but inserted into map
                currentSecondVariableIndex = variableCounter;
                variableCounter++;
            }
            else
            {
                currentSecondVariableIndex = element.first->second;
            }

            allSquares = false;

            // Matrix is self adjoint, so only need lower triangular elements
            if(T->firstVariable->index > T->secondVariable->index)
            {
                elements.emplace_back(currentFirstVariableIndex, currentSecondVariableIndex, 0.5 * T->coefficient);
            }
            else
            {
                elements.emplace_back(currentSecondVariableIndex, currentFirstVariableIndex, 0.5 * T->coefficient);
            }
        }
    }

    if(allSquares && allPositive)
    {
        convexity = E_Convexity::Convex;
        return;
    }

    if(allSquares && allNegative)
    {
        convexity = E_Convexity::Concave;
        return;
    }

    if(allBilinear)
    {
        convexity = E_Convexity::Nonconvex;
        return;
    }

    int numberOfVariables = variableMap.size();

    Eigen::SparseMatrix<double> matrix(numberOfVariables, numberOfVariables);
    matrix.setFromTriplets(elements.begin(), elements.end());

    Eigen::SelfAdjointEigenSolver<Eigen::SparseMatrix<double>> eigenSolver(
        matrix, Eigen::DecompositionOptions::EigenvaluesOnly);

    if(eigenSolver.info() != Eigen::Success)
    {
        convexity = E_Convexity::Unknown;
        return;
    }

    bool areAllPositiveOrZero = true;
    bool areAllNegativeOrZero = true;

    double eigenvalueTolerance = 0.0;

    if(auto sharedOwnerProblem = ownerProblem.lock())
    {
        if(sharedOwnerProblem->env->settings)
        {
            eigenvalueTolerance = sharedOwnerProblem->env->settings->getSetting<double>(
                "Convexity.Quadratics.EigenValueTolerance", "Model");
        }
        else
        {
            eigenvalueTolerance = 1e-5;
        }
    }

    for(int i = 0; i < numberOfVariables; i++)
    {
        double eigenvalue = eigenSolver.eigenvalues().col(0)[i];

        areAllNegativeOrZero = areAllNegativeOrZero && eigenvalue <= eigenvalueTolerance;
        areAllPositiveOrZero = areAllPositiveOrZero && eigenvalue >= -eigenvalueTolerance;
    }

    if(areAllPositiveOrZero)
        convexity = E_Convexity::Convex;
    else if(areAllNegativeOrZero)
        convexity = E_Convexity::Concave;
    else
        convexity = E_Convexity::Nonconvex;
};

MonomialTerm::MonomialTerm(const MonomialTerm* term, ProblemPtr destinationProblem)
{
    this->coefficient = term->coefficient;
    this->isBilinear = term->isBilinear;
    this->isSquare = term->isSquare;
    this->isBinary = term->isBinary;

    for(auto& V : term->variables)
    {
        this->variables.push_back(destinationProblem->getVariable(V->index));
    }
};

SignomialTerm::SignomialTerm(const SignomialTerm* term, ProblemPtr destinationProblem)
{
    this->coefficient = term->coefficient;

    for(auto& E : term->elements)
    {
        this->elements.push_back(
            std::make_shared<SignomialElement>(destinationProblem->getVariable(E->variable->index), E->power));
    }
};
} // namespace SHOT