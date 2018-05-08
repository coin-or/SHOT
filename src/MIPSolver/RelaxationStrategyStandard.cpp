/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "RelaxationStrategyStandard.h"

RelaxationStrategyStandard::RelaxationStrategyStandard(EnvironmentPtr envPtr)
{
    env = envPtr;
}

RelaxationStrategyStandard::~RelaxationStrategyStandard()
{
}

void RelaxationStrategyStandard::setInitial()
{
    LPFinished = false;

    if (env->settings->getIntSetting("Relaxation.IterationLimit", "Dual") > 0 && env->settings->getDoubleSetting("Relaxation.TimeLimit", "Dual") > 0)
    {
        this->setActive();
    }
    else
    {
        this->setInactive();
    }
}

void RelaxationStrategyStandard::executeStrategy()
{
    int iterInterval = env->settings->getIntSetting("Relaxation.Frequency", "Dual");
    if (iterInterval != 0 && env->process->getCurrentIteration()->iterationNumber % iterInterval == 0)
    {
        return (this->setActive());
    }

    if (isLPStepFinished() || isCurrentToleranceReached() || isGapReached() || isIterationLimitReached() || isTimeLimitReached() || isRelaxedSolutionEpsilonValid() || isObjectiveStagnant())
    {
        this->setInactive();
    }
    else
    {
        this->setActive();
    }
}

void RelaxationStrategyStandard::setActive()
{
    if (env->dualSolver->getDiscreteVariableStatus())
    {
        env->process->stopTimer("DualProblemsDiscrete");
        env->process->startTimer("DualProblemsRelaxed");
        env->dualSolver->activateDiscreteVariables(false);

        env->process->getCurrentIteration()->type = E_IterationProblemType::Relaxed;
    }
}

void RelaxationStrategyStandard::setInactive()
{
    if (!env->dualSolver->getDiscreteVariableStatus())
    {
        env->process->stopTimer("DualProblemsRelaxed");
        env->process->startTimer("DualProblemsDiscrete");
        env->dualSolver->activateDiscreteVariables(true);

        env->process->getCurrentIteration()->type = E_IterationProblemType::MIP;

        LPFinished = true;
    }
}

E_IterationProblemType RelaxationStrategyStandard::getProblemType()
{
    if (env->dualSolver->getDiscreteVariableStatus())

        return (E_IterationProblemType::MIP);
    else
        return (E_IterationProblemType::Relaxed);
}

bool RelaxationStrategyStandard::isIterationLimitReached()
{
    auto prevIter = env->process->getPreviousIteration();

    if (prevIter->iterationNumber < env->settings->getIntSetting("Relaxation.IterationLimit", "Dual"))
    {
        return (false);
    }

    return (true);
}

bool RelaxationStrategyStandard::isTimeLimitReached()
{
    if (env->process->getElapsedTime("DualProblemsRelaxed") < env->settings->getDoubleSetting("Relaxation.TimeLimit", "Dual"))
    {
        return (false);
    }

    return (true);
}

bool RelaxationStrategyStandard::isLPStepFinished()
{
    return (LPFinished);
}

bool RelaxationStrategyStandard::isObjectiveStagnant()
{
    int numSteps = 10;

    auto prevIter = env->process->getPreviousIteration();

    if (prevIter->iterationNumber < numSteps)
        return (false);

    auto prevIter2 = &env->process->iterations[prevIter->iterationNumber - numSteps];

    //TODO: should be substituted with parameter
    if (std::abs((prevIter->objectiveValue - prevIter2->objectiveValue) / prevIter->objectiveValue) < 0.000001)
        return (true);

    return (false);
}
