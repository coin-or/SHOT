/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskExecuteSolutionLimitStrategy.h"

namespace SHOT
{

TaskExecuteSolutionLimitStrategy::TaskExecuteSolutionLimitStrategy(EnvironmentPtr envPtr) : TaskBase(envPtr)
{
    env->process->startTimer("DualStrategy");

    isInitialized = false;
    temporaryOptLimitUsed = false;

    solutionLimitStrategy = new MIPSolutionLimitStrategyIncrease(env);
    auto initLim = solutionLimitStrategy->getInitialLimit();
    env->dualSolver->setSolutionLimit(initLim);
    previousSolLimit = initLim;

    env->process->stopTimer("DualStrategy");
}

TaskExecuteSolutionLimitStrategy::~TaskExecuteSolutionLimitStrategy()
{
    delete solutionLimitStrategy;
}

void TaskExecuteSolutionLimitStrategy::run()
{
    env->process->startTimer("DualStrategy");
    if (!isInitialized)
    {
        isInitialized = true;
    }

    auto currIter = env->process->getCurrentIteration();
    auto prevIter = env->process->getPreviousIteration();

    //previousSolLimit = prevIter->usedMIPSolutionLimit;
    //std::cout << "prev solution limit" << previousSolLimit << " at iteration " << currIter->iterationNumber << std::endl;

    if (temporaryOptLimitUsed)
    {
        temporaryOptLimitUsed = false;
        env->dualSolver->setSolutionLimit(previousSolLimit);
    }

    if (currIter->iterationNumber - env->solutionStatistics.iterationLastDualBoundUpdate > env->settings->getIntSetting("MIP.SolutionLimit.ForceOptimal.Iteration", "Dual") && env->process->getDualBound() > -OSDBL_MAX)
    {
        previousSolLimit = prevIter->usedMIPSolutionLimit;
        env->dualSolver->setSolutionLimit(2100000000);
        temporaryOptLimitUsed = true;
        currIter->MIPSolutionLimitUpdated = true;
        env->output->outputInfo(
            "     Forced optimal iteration since too many iterations since last dual bound update");
    }
    else if (env->process->getElapsedTime("Total") - env->solutionStatistics.timeLastDualBoundUpdate > env->settings->getDoubleSetting("MIP.SolutionLimit.ForceOptimal.Time", "Dual") && env->process->getDualBound() > -OSDBL_MAX)
    {
        previousSolLimit = prevIter->usedMIPSolutionLimit;
        env->dualSolver->setSolutionLimit(2100000000);
        temporaryOptLimitUsed = true;
        currIter->MIPSolutionLimitUpdated = true;
        env->output->outputAlways(
            "     Forced optimal iteration since too long time since last dual bound update");
    }
    else if (env->process->getPrimalBound() < OSDBL_MAX && abs(prevIter->objectiveValue - env->process->getPrimalBound()) < 0.001)
    {
        previousSolLimit = prevIter->usedMIPSolutionLimit + 1;
        env->dualSolver->setSolutionLimit(2100000000);
        temporaryOptLimitUsed = true;
        currIter->MIPSolutionLimitUpdated = true;
        env->output->outputInfo(
            "     Forced optimal iteration since difference between MIP solution and primal is small");
    }
    else
    {
        currIter->MIPSolutionLimitUpdated = solutionLimitStrategy->updateLimit();

        if (currIter->MIPSolutionLimitUpdated)
        {
            int newLimit = solutionLimitStrategy->getNewLimit();

            if (newLimit != env->process->getPreviousIteration()->usedMIPSolutionLimit)
            {
                env->dualSolver->setSolutionLimit(newLimit);
            }
        }
    }

    env->process->stopTimer("DualStrategy");
}

std::string TaskExecuteSolutionLimitStrategy::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT