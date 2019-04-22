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
void QuadraticTerms::updateConvexity()
{
    if(size() == 0)
    {
        convexity = E_Convexity::Linear;
        return;
    }

    std::vector<Eigen::Triplet<double>> elements;
    elements.reserve(2 * size());

    std::map<VariablePtr, bool> variableMap;

    bool allSquares = true;
    bool allPositive = true;
    bool allNegative = true;

    for(auto& T : (*this))
    {
        if(T->firstVariable == T->secondVariable)
        {
            variableMap.insert(std::make_pair(T->firstVariable, true));
            allPositive = allPositive && T->coefficient >= 0;
            allNegative = allNegative && T->coefficient <= 0;

            elements.push_back(
                Eigen::Triplet<double>(T->firstVariable->index, T->firstVariable->index, T->coefficient));
        }
        else
        {
            variableMap.insert(std::make_pair(T->firstVariable, true));
            variableMap.insert(std::make_pair(T->secondVariable, true));
            allSquares = false;

            // Matrix is self adjoint, so only need lower triangular elements
            if(T->firstVariable->index > T->secondVariable->index)
            {
                elements.push_back(
                    Eigen::Triplet<double>(T->firstVariable->index, T->secondVariable->index, 0.5 * T->coefficient));
            }
            else
            {
                elements.push_back(
                    Eigen::Triplet<double>(T->secondVariable->index, T->firstVariable->index, 0.5 * T->coefficient));
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
        eigenvalueTolerance = sharedOwnerProblem->env->settings->getSetting<double>(
            "Convexity.Quadratics.EigenValueTolerance", "Model");
    }

    for(int i = 0; i < numberOfVariables; i++)
    {
        double eigenvalue = eigenSolver.eigenvalues().col(0)[i];

        std::cout << "eigenvalue " << eigenvalue << std::endl;

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