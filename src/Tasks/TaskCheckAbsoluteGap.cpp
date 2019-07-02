/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckAbsoluteGap.h"

#include "../Enums.h"
#include "../Iteration.h"
#include "../Results.h"
#include "../Settings.h"
#include "../TaskHandler.h"
#include "../Timing.h"

namespace SHOT
{

TaskCheckAbsoluteGap::TaskCheckAbsoluteGap(EnvironmentPtr envPtr, std::string taskIDTrue)
    : TaskBase(envPtr), taskIDIfTrue(taskIDTrue)
{
}

TaskCheckAbsoluteGap::~TaskCheckAbsoluteGap() = default;

void TaskCheckAbsoluteGap::run()
{
    if(env->results->isAbsoluteObjectiveGapToleranceMet())
    {
        env->results->terminationReason = E_TerminationReason::AbsoluteGap;
        env->tasks->setNextTask(taskIDIfTrue);

        env->results->terminationReasonDescription = "Terminated since absolute gap met requirements.";
    }
}

std::string TaskCheckAbsoluteGap::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT