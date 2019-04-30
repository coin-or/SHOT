/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolutionLimitStrategyUnlimited.h"
#include "../DualSolver.h"
#include "../MIPSolver/IMIPSolver.h"

namespace SHOT
{

MIPSolutionLimitStrategyUnlimited::MIPSolutionLimitStrategyUnlimited(EnvironmentPtr envPtr) { env = envPtr; }

bool MIPSolutionLimitStrategyUnlimited::updateLimit() { return false; }

int MIPSolutionLimitStrategyUnlimited::getNewLimit() { return env->dualSolver->MIPSolver->getSolutionLimit(); }

int MIPSolutionLimitStrategyUnlimited::getInitialLimit()
{
    auto tmpVal = env->dualSolver->MIPSolver->getSolutionLimit();
    return tmpVal;
}
} // namespace SHOT