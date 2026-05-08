/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "MIPSolutionLimitStrategyIncrease.h"
#include "../Settings.h"
#include "../Results.h"
#include "../DualSolver.h"
#include "../MIPSolver/IMIPSolver.h"
#include "../Iteration.h"

namespace SHOT
{

MIPSolutionLimitStrategyIncrease::MIPSolutionLimitStrategyIncrease(EnvironmentPtr envPtr)
{
    env = envPtr;

    lastIterSolLimIncreased = 1;
    numSolLimIncremented = 1;
    lastIterOptimal = 1;
}

bool MIPSolutionLimitStrategyIncrease::updateLimit()
{
    auto currIter = env->results->getCurrentIteration();
    auto prevIter = env->results->getPreviousIteration();

    if(!currIter->isMIP())
    {
        lastIterSolLimIncreased = currIter->iterationNumber;
        return (false);
    }

    if(prevIter->isMIP() && prevIter->solutionStatus == E_ProblemSolutionStatus::Optimal)
    {
        lastIterOptimal = prevIter->iterationNumber;
        return (false);
    }

    // Solution limit has not been updated in the maximal number of iterations
    if(prevIter->isMIP()
        && (currIter->iterationNumber - lastIterSolLimIncreased
                > env->settings->getSetting<int>("Dual.MIP.SolutionLimit.IncreaseIterations")
            && currIter->iterationNumber - lastIterOptimal
                > env->settings->getSetting<int>("Dual.MIP.SolutionLimit.IncreaseIterations")))
    {
        env->output->outputDebug("     Force solution limit update.");
        return (true);
    }

    if(prevIter->isMIP() && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit)
    {
        if(prevIter->numHyperplanesAdded == 0)
            return (true);

        if(prevIter->maxDeviation < env->settings->getSetting<double>("Dual.MIP.SolutionLimit.UpdateTolerance"))
            return (true);

        if(prevIter->maxDeviation < env->settings->getSetting<double>("Termination.ConstraintTolerance"))
            return (true);

        if(prevIter->maxDeviationConstraint == -1
            && prevIter->maxDeviation < env->settings->getSetting<double>("Dual.MIP.SolutionLimit.UpdateTolerance")
                    * std::max(1.0, std::abs(prevIter->objectiveValue)))
        {
            return (true);
        }
    }

    return (false);
}

int MIPSolutionLimitStrategyIncrease::getNewLimit()
{
    auto currIter = env->results->getCurrentIteration();

    int newLimit;

    newLimit = env->dualSolver->MIPSolver->getSolutionLimit() + 1;
    lastIterSolLimIncreased = currIter->iterationNumber;

    return (newLimit);
}

int MIPSolutionLimitStrategyIncrease::getInitialLimit()
{
    return (env->settings->getSetting<int>("Dual.MIP.SolutionLimit.Initial"));
}
} // namespace SHOT