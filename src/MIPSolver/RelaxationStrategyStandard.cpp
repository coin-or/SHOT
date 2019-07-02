/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "RelaxationStrategyStandard.h"

#include "../DualSolver.h"
#include "../Iteration.h"
#include "../Results.h"
#include "../Settings.h"
#include "../Timing.h"

#include "../Model/Problem.h"

namespace SHOT
{

RelaxationStrategyStandard::RelaxationStrategyStandard(EnvironmentPtr envPtr)
{
    env = envPtr;
    setInitial();
}

RelaxationStrategyStandard::~RelaxationStrategyStandard() = default;

void RelaxationStrategyStandard::setInitial()
{
    LPFinished = false;

    if(env->settings->getSetting<bool>("Relaxation.Use", "Dual")
        && env->settings->getSetting<int>("Relaxation.IterationLimit", "Dual") > 0
        && env->settings->getSetting<double>("Relaxation.TimeLimit", "Dual") > 0)
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
    int iterInterval = env->settings->getSetting<int>("Relaxation.Frequency", "Dual");
    if(iterInterval != 0 && env->results->getCurrentIteration()->iterationNumber % iterInterval == 0)
    {
        return (this->setActive());
    }

    if(isLPStepFinished())
    {
        this->setInactive();
    }
    else if(isConstraintToleranceReached())
    {
        this->setInactive();
    }
    else if(isGapReached())
    {
        this->setInactive();
    }
    else if(isIterationLimitReached())
    {
        this->setInactive();
    }
    else if(isTimeLimitReached())
    {
        this->setInactive();
    }
    else if(isObjectiveStagnant())
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
    if(env->dualSolver->MIPSolver->getDiscreteVariableStatus() && env->results->getNumberOfIterations() > 0)
    {
        env->timing->stopTimer("DualProblemsDiscrete");
        env->timing->startTimer("DualProblemsRelaxed");
        env->dualSolver->MIPSolver->activateDiscreteVariables(false);

        env->results->getCurrentIteration()->isDualProblemDiscrete = false;
        env->results->getCurrentIteration()->dualProblemClass = env->dualSolver->MIPSolver->getProblemClass();
    }
}

void RelaxationStrategyStandard::setInactive()
{
    if(!env->dualSolver->MIPSolver->getDiscreteVariableStatus())
    {
        env->timing->stopTimer("DualProblemsRelaxed");
        env->timing->startTimer("DualProblemsDiscrete");
        env->dualSolver->MIPSolver->activateDiscreteVariables(true);

        env->results->getCurrentIteration()->isDualProblemDiscrete = true;
        env->results->getCurrentIteration()->dualProblemClass = env->dualSolver->MIPSolver->getProblemClass();

        LPFinished = true;
    }
}

bool RelaxationStrategyStandard::isIterationLimitReached()
{
    if(env->results->getNumberOfIterations() < 2)
    {
        return false;
    }

    auto prevIter = env->results->getPreviousIteration();

    if(prevIter->iterationNumber < env->settings->getSetting<int>("Relaxation.IterationLimit", "Dual"))
    {
        return (false);
    }

    return (true);
}

bool RelaxationStrategyStandard::isTimeLimitReached()
{
    if(env->timing->getElapsedTime("DualProblemsRelaxed")
        < env->settings->getSetting<double>("Relaxation.TimeLimit", "Dual"))
    {
        return (false);
    }

    return (true);
}

bool RelaxationStrategyStandard::isLPStepFinished() { return (LPFinished); }

bool RelaxationStrategyStandard::isObjectiveStagnant()
{
    int numSteps = 10;

    if(env->results->getNumberOfIterations() < 2)
    {
        return false;
    }

    auto prevIter = env->results->getPreviousIteration();

    if(prevIter->iterationNumber < numSteps)
        return (false);

    auto prevIter2 = env->results->iterations[prevIter->iterationNumber - numSteps];

    // TODO: should be substituted with parameter
    if(std::abs((prevIter->objectiveValue - prevIter2->objectiveValue) / prevIter->objectiveValue) < 0.000001)
        return (true);

    return (false);
}
} // namespace SHOT