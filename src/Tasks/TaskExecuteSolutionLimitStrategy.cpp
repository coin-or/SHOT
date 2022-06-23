/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskExecuteSolutionLimitStrategy.h"

#include "../DualSolver.h"
#include "../Iteration.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"
#include "../Utilities.h"

#include "../Model/Problem.h"

#include "../MIPSolver/IMIPSolver.h"

#include "../MIPSolver/IMIPSolutionLimitStrategy.h"
#include "../MIPSolver/MIPSolutionLimitStrategyUnlimited.h"
#include "../MIPSolver/MIPSolutionLimitStrategyIncrease.h"
#include "../MIPSolver/MIPSolutionLimitStrategyAdaptive.h"

namespace SHOT
{

TaskExecuteSolutionLimitStrategy::TaskExecuteSolutionLimitStrategy(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->timing->startTimer("DualStrategy");

    isInitialized = false;
    temporaryOptLimitUsed = false;

    solutionLimitStrategy = std::make_unique<MIPSolutionLimitStrategyIncrease>(env);
    auto initLim = solutionLimitStrategy->getInitialLimit();
    env->dualSolver->MIPSolver->setSolutionLimit(initLim);
    previousSolLimit = initLim;

    env->timing->stopTimer("DualStrategy");
}

TaskExecuteSolutionLimitStrategy::~TaskExecuteSolutionLimitStrategy() = default;

void TaskExecuteSolutionLimitStrategy::run()
{
    env->timing->startTimer("DualStrategy");
    if(!isInitialized)
    {
        isInitialized = true;
    }

    auto currIter = env->results->getCurrentIteration();
    auto prevIter = env->results->getPreviousIteration();

    if(env->reformulatedProblem->properties.convexity != E_ProblemConvexity::Convex)
    {
        if(temporaryOptLimitUsed)
        {
            temporaryOptLimitUsed = false;
            env->dualSolver->MIPSolver->setSolutionLimit(previousSolLimit);
        }

        if(currIter->iterationNumber - env->solutionStatistics.iterationLastDualBoundUpdate
                > env->settings->getSetting<int>("MIP.SolutionLimit.ForceOptimal.Iteration", "Dual")
            && env->results->getCurrentDualBound() > SHOT_DBL_MIN)
        {
            previousSolLimit = prevIter->usedMIPSolutionLimit;
            env->dualSolver->MIPSolver->setSolutionLimit(2100000000);
            temporaryOptLimitUsed = true;
            currIter->MIPSolutionLimitUpdated = true;
            env->output->outputDebug(
                "        Forced optimal iteration since too many iterations since last dual bound update");

            env->timing->stopTimer("DualStrategy");
            return;
        }

        if(env->timing->getElapsedTime("Total") - env->solutionStatistics.timeLastDualBoundUpdate
                > env->settings->getSetting<double>("MIP.SolutionLimit.ForceOptimal.Time", "Dual")
            && env->results->getCurrentDualBound() > SHOT_DBL_MIN)
        {
            previousSolLimit = prevIter->usedMIPSolutionLimit;
            env->dualSolver->MIPSolver->setSolutionLimit(2100000000);
            temporaryOptLimitUsed = true;
            currIter->MIPSolutionLimitUpdated = true;
            env->output->outputDebug(
                "        Forced optimal iteration since too long time since last dual bound update");

            env->timing->stopTimer("DualStrategy");
            return;
        }

        if(std::abs(env->results->getPrimalBound()) < SHOT_DBL_MAX
            && prevIter->solutionStatus == E_ProblemSolutionStatus::SolutionLimit
            && std::abs(prevIter->objectiveValue - env->results->getPrimalBound()) < 0.001)
        {
            previousSolLimit = prevIter->usedMIPSolutionLimit;
            env->dualSolver->MIPSolver->setSolutionLimit(2100000000);
            temporaryOptLimitUsed = true;
            currIter->MIPSolutionLimitUpdated = true;
            env->output->outputDebug(
                "        Forced optimal iteration since difference between MIP solution and primal is small");

            env->timing->stopTimer("DualStrategy");
            return;
        }
    }

    currIter->MIPSolutionLimitUpdated = solutionLimitStrategy->updateLimit();

    if(currIter->MIPSolutionLimitUpdated)
    {
        int newLimit = solutionLimitStrategy->getNewLimit();

        if(newLimit != env->results->getPreviousIteration()->usedMIPSolutionLimit)
        {
            env->dualSolver->MIPSolver->setSolutionLimit(newLimit);
        }
    }

    env->timing->stopTimer("DualStrategy");
    return;
}

std::string TaskExecuteSolutionLimitStrategy::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT