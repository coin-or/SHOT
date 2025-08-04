/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckUserTermination.h"

#include "../EventHandler.h"
#include "../Results.h"
#include "../TaskHandler.h"

#include <any>

namespace SHOT
{

TaskCheckUserTermination::TaskCheckUserTermination(EnvironmentPtr envPtr, std::string taskIDTrue)
    : TaskBase(envPtr), taskIDIfTrue(taskIDTrue)
{
}

TaskCheckUserTermination::~TaskCheckUserTermination() = default;

void TaskCheckUserTermination::run()
{
    env->events->notify(E_EventType::UserTerminationCheck, std::any());

    if(env->tasks->isTerminated()
        || env->results->getCurrentIteration()->solutionStatus == E_ProblemSolutionStatus::Abort)
    {
        env->results->terminationReason = E_TerminationReason::UserAbort;
        env->tasks->setNextTask(taskIDIfTrue);
        env->results->terminationReasonDescription = "Terminated by user.";
    }
}

std::string TaskCheckUserTermination::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT