/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "NLPSolverBase.h"

namespace SHOT
{

NLPSolverBase::NLPSolverBase() = default;

E_NLPSolutionStatus NLPSolverBase::solveProblem()
{
    auto solStatus = solveProblemInstance();

    return (solStatus);
}
} // namespace SHOT