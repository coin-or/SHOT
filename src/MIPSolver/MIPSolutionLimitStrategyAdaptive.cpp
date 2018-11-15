/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolutionLimitStrategyAdaptive.h"

namespace SHOT
{

MIPSolutionLimitStrategyAdaptive::MIPSolutionLimitStrategyAdaptive(EnvironmentPtr envPtr)
{
    env = envPtr;

    numSolLimIncremented = 1;
}

MIPSolutionLimitStrategyAdaptive::~MIPSolutionLimitStrategyAdaptive()
{
}

bool MIPSolutionLimitStrategyAdaptive::updateLimit()
{
    auto currIter = env->process->getCurrentIteration();
    auto prevIter = env->process->getPreviousIteration();

    if (!currIter->isMIP())
    {
        return false;
    }

    if (prevIter->isMIP() && prevIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
    {
        return false;
    }

    // Solution limit has not been updated in the maximal number of iterations
    if (prevIter->isMIP() && currIter->iterationNumber - lastIterSolLimIncreased > env->settings->getIntSetting("MIP.SolutionLimit.IncreaseIterations", "Dual"))
    {
        return true;
    }

    // We have a feasible MIP solution to the original problem, but not proven optimal by MIP solver
    if (prevIter->isMIP() && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit && prevIter->maxDeviation < prevIter->usedConstraintTolerance)
    {
        return true;
    }

    return false;
}

int MIPSolutionLimitStrategyAdaptive::getNewLimit()
{
    auto currIter = env->process->getCurrentIteration();

    int newLimit;
    newLimit = env->dualSolver->getSolutionLimit() + 1;
    lastIterSolLimIncreased = currIter->iterationNumber;

    return newLimit;
}

int MIPSolutionLimitStrategyAdaptive::getInitialLimit()
{
    return env->settings->getIntSetting("MIP.SolutionLimit.Initial", "Dual");
}
} // namespace SHOT