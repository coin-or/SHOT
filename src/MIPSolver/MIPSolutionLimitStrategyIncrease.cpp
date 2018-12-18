/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolutionLimitStrategyIncrease.h"

namespace SHOT
{

MIPSolutionLimitStrategyIncrease::MIPSolutionLimitStrategyIncrease(EnvironmentPtr envPtr)
{
    env = envPtr;

    lastIterSolLimIncreased = 1;
    numSolLimIncremented = 1;
    lastIterOptimal = 1;
}

MIPSolutionLimitStrategyIncrease::~MIPSolutionLimitStrategyIncrease()
{
}

bool MIPSolutionLimitStrategyIncrease::updateLimit()
{
    auto currIter = env->process->getCurrentIteration();
    auto prevIter = env->process->getPreviousIteration();

    if (!currIter->isMIP())
    {
        lastIterSolLimIncreased = currIter->iterationNumber;
        return (false);
    }

    if (prevIter->isMIP() && prevIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
    {
        lastIterOptimal = prevIter->iterationNumber;
        return (false);
    }

    // Solution limit has not been updated in the maximal number of iterations
    if (prevIter->isMIP() && (currIter->iterationNumber - lastIterSolLimIncreased > env->settings->getIntSetting("MIP.SolutionLimit.IncreaseIterations", "Dual") && currIter->iterationNumber - lastIterOptimal > env->settings->getIntSetting("MIP.SolutionLimit.IncreaseIterations", "Dual")))
    {
        env->output->outputInfo("     Force solution limit update.");
        return (true);
    }

    bool useObjectiveLinesearchUpdate = true;

    if (prevIter->maxDeviationConstraint == -1 && useObjectiveLinesearchUpdate)
    {
        return (false);
    }

    if (prevIter->isMIP() && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit)
    {

        if (prevIter->maxDeviation < env->settings->getDoubleSetting("MIP.SolutionLimit.UpdateTolerance", "Dual"))
            return (true);

        if (prevIter->maxDeviation < env->settings->getDoubleSetting("ConstraintTolerance", "Termination"))
            return (true);

        if (prevIter->maxDeviationConstraint == -1 && prevIter->maxDeviation < env->settings->getDoubleSetting("MIP.SolutionLimit.UpdateTolerance", "Dual") * std::max(1.0, abs(prevIter->objectiveValue)))
        {
            return (true);
        }
    }

    return (false);
}

int MIPSolutionLimitStrategyIncrease::getNewLimit()
{
    auto currIter = env->process->getCurrentIteration();

    int newLimit;

    newLimit = env->dualSolver->getSolutionLimit() + 1;
    lastIterSolLimIncreased = currIter->iterationNumber;

    return (newLimit);
}

int MIPSolutionLimitStrategyIncrease::getInitialLimit()
{
    return (env->settings->getIntSetting("MIP.SolutionLimit.Initial", "Dual"));
}
} // namespace SHOT