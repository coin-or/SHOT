/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolutionLimitStrategyUnlimited.h"

MIPSolutionLimitStrategyUnlimited::MIPSolutionLimitStrategyUnlimited(IMIPSolver *MIPSolver)
{
    this->MIPSolver = MIPSolver;
}

MIPSolutionLimitStrategyUnlimited::~MIPSolutionLimitStrategyUnlimited()
{
}

bool MIPSolutionLimitStrategyUnlimited::updateLimit()
{
    return false;
}

int MIPSolutionLimitStrategyUnlimited::getNewLimit()
{
    return MIPSolver->getSolutionLimit();
}

int MIPSolutionLimitStrategyUnlimited::getInitialLimit()
{
    auto tmpVal = MIPSolver->getSolutionLimit();
    return tmpVal;
}
