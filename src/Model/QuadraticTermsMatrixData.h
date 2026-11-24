/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#pragma once
#include "Terms.h"

#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>

#include <cstddef>
#include <type_traits>
#include <vector>

namespace SHOT
{

// The counts of nonzeros in QuadraticTerms are std::ptrdiff_t so that Terms.h does not need Eigen
static_assert(std::is_same_v<Eigen::Index, std::ptrdiff_t>, "Eigen::Index is assumed to be std::ptrdiff_t");

// The matrix data of QuadraticTerms, which is only needed by the files that check the convexity of the terms or
// decompose them
struct QuadraticTermsMatrixData
{
    // The lower triangle of the matrix of the terms
    std::vector<Eigen::Triplet<double>> elements;

    Eigen::VectorXd eigenvalues;

    // Only computed on demand by computeEigenvectors(), since they are only needed by the eigenvalue decomposition
    Eigen::MatrixXd eigenvectors;

    // Sparse, since a dense one takes O(n^2) memory, e.g., 50 GB for 80000 variables
    Eigen::SparseMatrix<double> LDLMatrixL;
};

} // namespace SHOT
