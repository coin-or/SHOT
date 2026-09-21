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
#include "../Timing.h"

#include <Eigen/Sparse>

namespace SHOT
{
Interval Term::getBounds()
{
    // The bound vector of the problem is not copied, since this is called for every term, e.g. in bound tightening
    if(auto sharedOwnerProblem = ownerProblem.lock())
        return (calculate(sharedOwnerProblem->getVariableBounds()));

    return (calculate(IntervalVector()));
}

void QuadraticTerms::updateConvexity()
{
    if(size() == 0)
    {
        convexity = E_Convexity::Linear;
        return;
    }

    // The state of the previous calculation is cleared, since the convexity is recalculated when terms have been
    // added. Otherwise the matrix contains both the old and the new elements, and e.g. x^2 merged with -1.5x^2 into
    // the term -0.5x^2 gives the matrix element 2 + (-1) = 1, which is classified as convex.
    elements.clear();
    elements.reserve(2 * size());

    minEigenValue = SHOT::SHOT_DBL_MAX;
    maxEigenValue = SHOT::SHOT_DBL_MIN;
    minEigenValueWithinTolerance = false;
    maxEigenValueWithinTolerance = false;

    eigenvectorsComputed = false;
    LDLFactorizationPerformed = false;
    LDLFactorizationSuccessful = false;
    LDLDiag.clear();

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

    if(auto sharedOwnerProblem = ownerProblem.lock())
    {
        sharedOwnerProblem->env->timing->startTimer("EigenvalueComputation");
    }

    // The eigenvectors are only needed by the eigenvalue decomposition of the reformulation, and computing them
    // takes a large part of the time for a dense matrix, so they are computed on demand in computeEigenvectors()
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(
        matrix, Eigen::DecompositionOptions::EigenvaluesOnly);

    if(eigenSolver.info() != Eigen::Success)
    {
        convexity = E_Convexity::Unknown;
        minEigenValueWithinTolerance = false;
        maxEigenValueWithinTolerance = false;
        return;
    }

    eigenvalues = eigenSolver.eigenvalues();
    eigenvectorsComputed = false;

    if(auto sharedOwnerProblem = ownerProblem.lock())
    {
        sharedOwnerProblem->env->timing->stopTimer("EigenvalueComputation");
    }

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
                "Model.Convexity.Quadratics.EigenValueTolerance");
        }
        else
        {
            eigenvalueTolerance = 1e-5;
        }
    }

    for(int i = 0; i < numberOfVariables; i++)
    {
        double eigenvalue = eigenvalues[i];

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

void QuadraticTerms::computeEigenvectors()
{
    if(eigenvectorsComputed)
        return;

    assert(convexity != E_Convexity::NotSet);

    int numberOfVariables = variableMap.size();

    Eigen::SparseMatrix<double> matrix(numberOfVariables, numberOfVariables);
    matrix.setFromTriplets(elements.begin(), elements.end());

    if(auto sharedOwnerProblem = ownerProblem.lock())
    {
        sharedOwnerProblem->env->timing->startTimer("EigenvalueComputation");
    }

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(
        matrix, Eigen::DecompositionOptions::ComputeEigenvectors);

    if(auto sharedOwnerProblem = ownerProblem.lock())
    {
        sharedOwnerProblem->env->timing->stopTimer("EigenvalueComputation");
    }

    if(eigenSolver.info() != Eigen::Success)
        return;

    eigenvalues = eigenSolver.eigenvalues();
    eigenvectors = eigenSolver.eigenvectors();
    eigenvectorsComputed = true;
}

void QuadraticTerms::createGradientStructure()
{
    cachedGradient.clear();
    gradientElements.clear();
    gradientElements.reserve(size());

    for(auto& T : (*this))
    {
        if(T->coefficient == 0.0)
            continue;

        // Inserting into a map does not invalidate pointers to the values of the other elements, so the pointers
        // are valid also after the following terms have been added
        auto firstElement = &(cachedGradient.emplace(T->firstVariable, 0.0).first->second);
        auto secondElement = (T->firstVariable == T->secondVariable)
            ? nullptr
            : &(cachedGradient.emplace(T->secondVariable, 0.0).first->second);

        gradientElements.push_back({ T.get(), firstElement, secondElement });
    }

    gradientStructureCreated = true;
}

void QuadraticTerms::performLDLFactorization()
{
    if(LDLFactorizationPerformed)
        return;

    assert(convexity != E_Convexity::NotSet);

    // The following cases should not be handled with the LDL reformulation
    if(allSquares && allPositive)
    {
        LDLFactorizationPerformed = true;
        LDLFactorizationSuccessful = false;
        return;
    }

    if(allSquares && allNegative)
    {
        LDLFactorizationPerformed = true;
        LDLFactorizationSuccessful = false;
        return;
    }

    if(allBilinear)
    {
        LDLFactorizationPerformed = true;
        LDLFactorizationSuccessful = false;
        return;
    }

    int numberOfVariables = variableMap.size();

    // The matrix is symmetric with real elements, so the factorization does not need complex arithmetic
    Eigen::SparseMatrix<double> matrix(numberOfVariables, numberOfVariables);
    matrix.setFromTriplets(elements.begin(), elements.end());

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> eigenSolverLDL;

    eigenSolverLDL.compute(matrix);

    switch(eigenSolverLDL.info())
    {
    case Eigen::Success:
        break;
    case Eigen::NumericalIssue:
        // std::cout << "Error: LDL::info(): Numerical issue." << std::endl;
    default:
        LDLFactorizationPerformed = true;
        LDLFactorizationSuccessful = false;
        return;
    }

    Eigen::SparseMatrix<double> matrixL = eigenSolverLDL.matrixL();
    auto permInv = eigenSolverLDL.permutationPinv();

    LDLMatrixL = (permInv * matrixL).eval();

    Eigen::VectorXd diagonalD = eigenSolverLDL.vectorD();

    for(int i = 0; i < diagonalD.size(); i++)
        LDLDiag.push_back(diagonalD[i]);

    // The matrix only holds the lower triangular elements, so the upper ones are added to compare with the
    // reconstructed matrix
    Eigen::SparseMatrix<double> original = matrix;
    original += Eigen::SparseMatrix<double>(matrix.transpose());
    original -= Eigen::SparseMatrix<double>(matrix.diagonal().asDiagonal());

    Eigen::MatrixXd error = LDLMatrixL * diagonalD.asDiagonal() * LDLMatrixL.transpose() - original;

    // The error to the reconstructed matrix is too large, will not use the decomposition. The tolerance is relative to
    // the largest element, since the round-off error grows with the elements, but not smaller than for elements of 1.
    double errorTolerance = 1e-12 * std::max(1.0, Eigen::MatrixXd(original).cwiseAbs().maxCoeff());

    if(error.cwiseAbs().maxCoeff() > errorTolerance)
    {
        LDLFactorizationPerformed = true;
        LDLFactorizationSuccessful = false;
        return;
    }

    LDLFactorizationPerformed = true;
    LDLFactorizationSuccessful = true;
}

MonomialTerm::MonomialTerm(const MonomialTerm* term, ProblemPtr destinationProblem)
{
    this->coefficient = term->coefficient;
    this->isBilinear = term->isBilinear;
    this->isSquare = term->isSquare;
    this->isBinary = term->isBinary;

    for(auto& V : term->variables)
    {
        this->variables.push_back(destinationProblem->getVariable(V->getIndex()));
    }
}

SignomialTerm::SignomialTerm(const SignomialTerm* term, ProblemPtr destinationProblem)
{
    this->coefficient = term->coefficient;

    for(auto& E : term->elements)
    {
        this->elements.push_back(
            std::make_shared<SignomialElement>(destinationProblem->getVariable(E->variable->getIndex()), E->power));
    }
}
} // namespace SHOT
