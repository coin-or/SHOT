/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckTimeLimit.h"

#include "../Results.h"
#include "../Settings.h"
#include "../TaskHandler.h"
#include "../Timing.h"

namespace SHOT
{

TaskCheckTimeLimit::TaskCheckTimeLimit(EnvironmentPtr envPtr, std::string taskIDTrue)
    : TaskBase(envPtr), taskIDIfTrue(taskIDTrue)
{
}

TaskCheckTimeLimit::~TaskCheckTimeLimit() = default;

void TaskCheckTimeLimit::run()
{
    auto currIter = env->results->getCurrentIteration();

    if(env->timing->getElapsedTime("Total") >= env->settings->getSetting<double>("TimeLimit", "Termination"))
    {
        env->results->terminationReason = E_TerminationReason::TimeLimit;
        env->tasks->setNextTask(taskIDIfTrue);
        env->results->terminationReasonDescription = "Terminated since time limit was reached.";
    }
}

std::string TaskCheckTimeLimit::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT