/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckIterationLimit.h"

TaskCheckIterationLimit::TaskCheckIterationLimit(EnvironmentPtr envPtr, std::string taskIDTrue) : TaskBase(envPtr), taskIDIfTrue(taskIDTrue)
{
}

TaskCheckIterationLimit::~TaskCheckIterationLimit()
{
}

void TaskCheckIterationLimit::run()
{
    auto currIter = env->process->getCurrentIteration();

    if (currIter->iterationNumber >= env->settings->getIntSetting("Relaxation.IterationLimit", "Dual") + env->settings->getIntSetting("IterationLimit", "Termination"))
    {
        env->process->terminationReason = E_TerminationReason::IterationLimit;
        env->tasks->setNextTask(taskIDIfTrue);
    }
}

std::string TaskCheckIterationLimit::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
