/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE
   This software is licensed under the Eclipse Public License 2.0.
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckIterationLimit.h"

#include "../Iteration.h"
#include "../Results.h"
#include "../Settings.h"
#include "../TaskHandler.h"
#include "../Timing.h"

namespace SHOT
{

TaskCheckIterationLimit::TaskCheckIterationLimit(EnvironmentPtr envPtr, std::string taskIDTrue)
    : TaskBase(envPtr), taskIDIfTrue(taskIDTrue)
{
}

TaskCheckIterationLimit::~TaskCheckIterationLimit() = default;

void TaskCheckIterationLimit::run()
{
    auto currIter = env->results->getCurrentIteration();

    auto mainlimit = env->settings->getSetting<int>("IterationLimit", "Termination");

    if(mainlimit == SHOT_INT_MAX)
        return;

    if(currIter->iterationNumber >= mainlimit)
    {
        env->results->terminationReason = E_TerminationReason::IterationLimit;
        env->tasks->setNextTask(taskIDIfTrue);
        env->results->terminationReasonDescription = "Terminated since the iteration limit was reached.";
    }
}

std::string TaskCheckIterationLimit::getType()
{
    std::string type = typeid(this).name();
    return (type);
}
} // namespace SHOT