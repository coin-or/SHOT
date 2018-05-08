/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckObjectiveGapNotMet.h"

TaskCheckObjectiveGapNotMet::TaskCheckObjectiveGapNotMet(EnvironmentPtr envPtr, std::string taskIDTrue) : TaskBase(envPtr), taskIDIfTrue(taskIDTrue)
{
}

TaskCheckObjectiveGapNotMet::~TaskCheckObjectiveGapNotMet()
{
}

void TaskCheckObjectiveGapNotMet::run()
{
    auto currIter = env->process->getCurrentIteration();

    if (env->process->primalSolutions.size() > 0 &&
        !env->process->isRelativeObjectiveGapToleranceMet() &&
        !env->process->isAbsoluteObjectiveGapToleranceMet())
    {
        env->process->terminationReason = E_TerminationReason::ObjectiveGapNotReached;
        env->process->tasks->setNextTask(taskIDIfTrue);
    }
}

std::string TaskCheckObjectiveGapNotMet::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
