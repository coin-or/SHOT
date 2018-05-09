/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckTimeLimit.h"

TaskCheckTimeLimit::TaskCheckTimeLimit(EnvironmentPtr envPtr, std::string taskIDTrue) : TaskBase(envPtr), taskIDIfTrue(taskIDTrue)
{
}

TaskCheckTimeLimit::~TaskCheckTimeLimit()
{
}

void TaskCheckTimeLimit::run()
{
    auto currIter = env->process->getCurrentIteration();

    if (env->process->getElapsedTime("Total") >= env->settings->getDoubleSetting("TimeLimit", "Termination"))
    {
        env->process->terminationReason = E_TerminationReason::TimeLimit;
        env->tasks->setNextTask(taskIDIfTrue);
    }
}

std::string TaskCheckTimeLimit::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
