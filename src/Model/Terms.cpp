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

    allSquares = true;
    allPositive = true;
    allNegative = true;
    allBilinear = true;

    int variableCounter = 0;
    variableMap.clear();

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

            elements.emplace_back(currentVariableIndex, currentVariableIndex, 2 * T->coefficient);
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
            if(currentFirstVariableIndex > currentSecondVariableIndex)
            {
                elements.emplace_back(currentFirstVariableIndex, currentSecondVariableIndex, T->coefficient);
                // std::cout << currentFirstVariableIndex + 1 << " " << currentSecondVariableIndex + 1 << " "
                //          << T->coefficient << std::endl;
            }
            else
            {
                elements.emplace_back(currentSecondVariableIndex, currentFirstVariableIndex, T->coefficient);

                // std::cout << currentSecondVariableIndex + 1 << " " << currentFirstVariableIndex + 1 << " "
                //          << T->coefficient << std::endl;
            }
        }
    }

    // These are used to avoid using Eigen in obvious cases

    if(allSquares && allPositive)
    {
        convexity = E_Convexity::Convex;
        minEigenValueWithinTolerance = true;
        maxEigenValueWithinTolerance = false;
        return;
    }

    if(allSquares && allNegative)
    {
        convexity = E_Convexity::Concave;
        minEigenValueWithinTolerance = false;
        maxEigenValueWithinTolerance = true;
        return;
    }

    if(allBilinear)
    {
        convexity = E_Convexity::Nonconvex;
        minEigenValueWithinTolerance = false;
        maxEigenValueWithinTolerance = false;
        return;
    }

    int numberOfVariables = variableMap.size();

    Eigen::SparseMatrix<double> matrix(numberOfVariables, numberOfVariables);
    matrix.setFromTriplets(elements.begin(), elements.end());

    // std::cout << matrix.toDense() << std::endl;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(
        matrix, Eigen::DecompositionOptions::ComputeEigenvectors);

    if(eigenSolver.info() != Eigen::Success)
    {
        convexity = E_Convexity::Unknown;
        minEigenValueWithinTolerance = false;
        maxEigenValueWithinTolerance = false;
        return;
    }

    eigenvalues = eigenSolver.eigenvalues();
    eigenvectors = eigenSolver.eigenvectors();

    // std::cout << eigenvalues << std::endl;

    // std::cout << eigenvectors << std::endl;

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
        double eigenvalue = eigenSolver.eigenvalues()[i];

        this->minEigenValue = std::min(this->minEigenValue, eigenvalue);
        this->maxEigenValue = std::max(this->maxEigenValue, eigenvalue);

        areAllNegativeOrZero = areAllNegativeOrZero && eigenvalue <= eigenvalueTolerance;
        areAllPositiveOrZero = areAllPositiveOrZero && eigenvalue >= -eigenvalueTolerance;
    }

    if(areAllPositiveOrZero)
        convexity = E_Convexity::Convex;
    else if(areAllNegativeOrZero)
        convexity = E_Convexity::Concave;
    else
        convexity = E_Convexity::Nonconvex;

    if(this->minEigenValue >= -eigenvalueTolerance)
        minEigenValueWithinTolerance = true;

    if(this->maxEigenValue <= eigenvalueTolerance)
        maxEigenValueWithinTolerance = true;
}

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
}

SignomialTerm::SignomialTerm(const SignomialTerm* term, ProblemPtr destinationProblem)
{
    this->coefficient = term->coefficient;

    for(auto& E : term->elements)
    {
        this->elements.push_back(
            std::make_shared<SignomialElement>(destinationProblem->getVariable(E->variable->index), E->power));
    }
}
} // namespace SHOT
