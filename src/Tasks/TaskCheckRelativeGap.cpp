/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckRelativeGap.h"

namespace SHOT
{

TaskCheckRelativeGap::TaskCheckRelativeGap(EnvironmentPtr envPtr, std::string taskIDTrue) : TaskBase(envPtr), taskIDIfTrue(taskIDTrue)
{
}

TaskCheckRelativeGap::~TaskCheckRelativeGap()
{
}

void TaskCheckRelativeGap::run()
{
    if (env->results->isRelativeObjectiveGapToleranceMet())
    {
        env->results->terminationReason = E_TerminationReason::RelativeGap;
        env->tasks->setNextTask(taskIDIfTrue);
    }
}

std::string TaskCheckRelativeGap::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT