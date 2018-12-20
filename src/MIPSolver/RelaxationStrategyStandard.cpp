/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "RelaxationStrategyStandard.h"

namespace SHOT
{

RelaxationStrategyStandard::RelaxationStrategyStandard(EnvironmentPtr envPtr)
{
    env = envPtr;
    setInitial();
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
    if (iterInterval != 0 && env->results->getCurrentIteration()->iterationNumber % iterInterval == 0)
    {
        return (this->setActive());
    }

    if (isLPStepFinished() || isConstraintToleranceReached() || isGapReached() || isIterationLimitReached() || isTimeLimitReached() || isObjectiveStagnant())
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
    if (env->dualSolver->MIPSolver->getDiscreteVariableStatus() && env->results->iterations.size() > 0)
    {
        env->timing->stopTimer("DualProblemsDiscrete");
        env->timing->startTimer("DualProblemsRelaxed");
        env->dualSolver->MIPSolver->activateDiscreteVariables(false);

        env->results->getCurrentIteration()->type = E_IterationProblemType::Relaxed;
    }
}

void RelaxationStrategyStandard::setInactive()
{
    if (!env->dualSolver->MIPSolver->getDiscreteVariableStatus())
    {
        env->timing->stopTimer("DualProblemsRelaxed");
        env->timing->startTimer("DualProblemsDiscrete");
        env->dualSolver->MIPSolver->activateDiscreteVariables(true);

        env->results->getCurrentIteration()->type = E_IterationProblemType::MIP;

        LPFinished = true;
    }
}

E_IterationProblemType RelaxationStrategyStandard::getProblemType()
{
    if (env->dualSolver->MIPSolver->getDiscreteVariableStatus())

        return (E_IterationProblemType::MIP);
    else
        return (E_IterationProblemType::Relaxed);
}

bool RelaxationStrategyStandard::isIterationLimitReached()
{
    if (env->results->iterations.size() < 2)
    {
        return false;
    }

    auto prevIter = env->results->getPreviousIteration();

    if (prevIter->iterationNumber < env->settings->getIntSetting("Relaxation.IterationLimit", "Dual"))
    {
        return (false);
    }

    return (true);
}

bool RelaxationStrategyStandard::isTimeLimitReached()
{
    if (env->timing->getElapsedTime("DualProblemsRelaxed") < env->settings->getDoubleSetting("Relaxation.TimeLimit", "Dual"))
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

    if (env->results->iterations.size() < 2)
    {
        return false;
    }

    auto prevIter = env->results->getPreviousIteration();

    if (prevIter->iterationNumber < numSteps)
        return (false);

    auto prevIter2 = env->results->iterations[prevIter->iterationNumber - numSteps];

    //TODO: should be substituted with parameter
    if (std::abs((prevIter->objectiveValue - prevIter2->objectiveValue) / prevIter->objectiveValue) < 0.000001)
        return (true);

    return (false);
}
} // namespace SHOT